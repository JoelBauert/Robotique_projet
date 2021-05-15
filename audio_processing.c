#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <pid.h>
#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

#define MIN_VALUE_THRESHOLD	10000 //maximum tolerated difference - empirically determined by testing

#define MIN_FREQ		20	//we don't analyze before this index to not use resources for nothing
#define MAX_FREQ		40	//we don't analyze after this index to not use resources for nothing


//speed and state variables for motor controls
static float speed_R;
static float speed_L;
static uint8_t state;
static uint16_t frequency;

/*
 * functions to pass the variables to main
 */

float get_speed_right(void)
{
	return speed_R;
}
float get_speed_left(void)
{
	return speed_L;
}
uint8_t get_state(void)
{
	return state;
}

uint16_t get_frequency(void)
{
	return frequency;
}

/*
*	Simple function used to detect the highest value in a buffer
*/

float sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			frequency = i; //position detected but not the real frequency
		}
	}
	return max_norm;
}

/*
 *	function to determine which direction the sound is coming from
 *	and how to turn or advance towards it
 *
 *	parameters:
 *	micro0-2			magnitudes of the three microphone signals
 */

void find_sound(float micro0, float micro1, float micro2)
{
	if(micro2 > micro1){ //arrière droite
		if(micro2 > micro0){
			if(micro1 > micro0){
				// turn left
				speed_L = -MOTOR_SPEED_LIMIT;
				speed_R = MOTOR_SPEED_LIMIT;
				state = BACK_LEFT;
				return;
			}
		}
		// turn right
		speed_L = MOTOR_SPEED_LIMIT;
		speed_R = -MOTOR_SPEED_LIMIT;
		state = BACK_RIGHT;
		return;
	}
	else if(micro2 > micro0){
		// turn left
		speed_L = -MOTOR_SPEED_LIMIT;
		speed_R = MOTOR_SPEED_LIMIT;
		state = BACK_LEFT;
		return;
	}
	else if(micro1 > micro2 && micro0 > micro2){ // front right or left
		float speed = 0;
		// if micro1 > micro0 -> error = micro1-micro0 > 0 -> turn left
		// if micro1 < micro0 -> error = micro1-micro0 < 0 -> turn right
		if(micro1 > micro0) //devant droit
			state = FRONT_RIGHT;
		else //devant gauche
			state = FRONT_LEFT;

		// go forward with directional bias
		speed = calcul_pid(micro1, micro0, MOTOR_SPEED_LIMIT);
		speed_L = 550-speed;
		speed_R = 550+speed;

		//Securite pour les moteurs
		if(speed_L > MOTOR_SPEED_LIMIT){
			speed_L = MOTOR_SPEED_LIMIT;
		}
		else if(speed_L < -MOTOR_SPEED_LIMIT){
				speed_L = -MOTOR_SPEED_LIMIT;
		}
		if(speed_R > MOTOR_SPEED_LIMIT){
			speed_R = MOTOR_SPEED_LIMIT;
		}
		else if(speed_R < -MOTOR_SPEED_LIMIT){
			speed_R = -MOTOR_SPEED_LIMIT;
		}
	}
}
/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	Then calculate the FFT and magnitude and initiate the calculation for the sound's location
*
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs (only for the 3 mics we use: to minimize time spent in the loop)
	*
	*/

	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;
		micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		//doFFT_optimized(FFT_SIZE, micFront_cmplx_input); //we don't need it for our application
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		//arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE); //we don't need it for our application
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3
		if(mustSend > 8){
			//signals to send the result to the computer
			chBSemSignal(&sendToComputer_sem);
			mustSend = 0;
		}
		nb_samples = 0;
		mustSend++;

		find_sound(sound_remote(micRight_output), sound_remote(micLeft_output), sound_remote(micBack_output));
	}
}

void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}
