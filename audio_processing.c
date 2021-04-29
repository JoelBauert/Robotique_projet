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

#define MIN_VALUE_THRESHOLD	10000

#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define FREQ_FORWARD	16	//250Hz
#define FREQ_LEFT		19	//296Hz
#define FREQ_RIGHT		23	//359HZ
#define FREQ_BACKWARD	26	//406Hz
#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing

#define FREQ_FORWARD_L		(FREQ_FORWARD-1)
#define FREQ_FORWARD_H		(FREQ_FORWARD+1)
#define FREQ_LEFT_L			(FREQ_LEFT-1)
#define FREQ_LEFT_H			(FREQ_LEFT+1)
#define FREQ_RIGHT_L		(FREQ_RIGHT-1)
#define FREQ_RIGHT_H		(FREQ_RIGHT+1)
#define FREQ_BACKWARD_L		(FREQ_BACKWARD-1)
#define FREQ_BACKWARD_H		(FREQ_BACKWARD+1)

//parameter to find the frequency with the index position
#define B					150
#define A					150/512


static float speed_R;
static float speed_L;
static uint8_t state;
static float frequency;

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

float get_frequency(void)
{
	frequency = B-abs(frequency)*A;
	return frequency;
}

float sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
//	int16_t max_norm_index = -1;

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			frequency = i; //position detected but not the real frequency
		}
	}
	return max_norm;
}


void find_sound(float micro0, float micro1, float micro2)
{
	//graphe index
	static uint16_t i = 0;
	i++;
	chprintf((BaseSequentialStream *)&SD3, "%f %f %f %d ", micro2, micro1, micro0, i);
	//if(micro2 > micro1 && micro2 > micro0 && micro0 > micro1){ // back right
	if(micro2 > micro1){ //arrière droite
		if(micro2 > micro0){
			if(micro1 > micro0){
				// turn left
				speed_L = -MOTOR_SPEED_LIMIT;
				speed_R = MOTOR_SPEED_LIMIT;
				chprintf((BaseSequentialStream *)&SD3, "%f %f\r\n",speed_L, speed_R);
				state = BACK_LEFT;
				return;
			}
		}
		// turn right
		speed_L = MOTOR_SPEED_LIMIT;
		speed_R = -MOTOR_SPEED_LIMIT;
		chprintf((BaseSequentialStream *)&SD3, "%f %f\r\n",speed_L, speed_R);
		state = BACK_RIGHT;
		return;
	}
	//if(micro2 > micro1 && micro2 > micro0 && micro1 > micro0){ // back left
	else if(micro2 > micro0){
		// turn left
		speed_L = -MOTOR_SPEED_LIMIT;
		speed_R = MOTOR_SPEED_LIMIT;
		chprintf((BaseSequentialStream *)&SD3, "%f %f\r\n",speed_L, speed_R);
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

		speed = calcul_pid(micro1, micro0, MOTOR_SPEED_LIMIT);
		speed_L = 550-speed;
		speed_R = 550+speed;

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
		chprintf((BaseSequentialStream *)&SD3, "%f %f\r\n",speed_L, speed_R);
	}
}
/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
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
	*	1024 samples, then we compute the FFTs.
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
		float micro0, micro1, micro2;
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
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
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
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

		micro0 = sound_remote(micRight_output);
		micro1 = sound_remote(micLeft_output);
		micro2 = sound_remote(micBack_output);

		find_sound(micro0, micro1, micro2);
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
