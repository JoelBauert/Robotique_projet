#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

#define BACK_RIGHT		1
#define BACK_LEFT		2
#define FRONT_RIGHT		3
#define FRONT_LEFT		4

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT
} BUFFER_NAME_t;

//éventuellement le mettre dans un fichier à part


float get_speed_right(void);
float get_speed_left(void);
uint8_t get_state(void);

//dans pi_processor.h max= #define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)


void find_sound(float micro0, float micro1, float micro2);

void processAudioData(int16_t *data, uint16_t num_samples);

/*
*	put the invoking thread into sleep until it can process the audio datas
*/
void wait_send_to_computer(void);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

#endif /* AUDIO_PROCESSING_H */
