#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include <sensors/proximity.h>

#include <audio_processing.h>
#include <pid.h>
#include <fft.h>
#include <distance.h>
#include <communications.h>
#include <arm_math.h>
#include <spi_comm.h>
#include <leds.h>

//define PID parameters - empirically determined by testing
#define Kp				0.055
#define Ki				0.001
#define Kd				0
#define THRESHOLD		10000
//define color parameters
#define MAX_COLOR		255
#define MIN_COLOR		0
//define audible frequency parameters
#define MIN_FREQU		22 // 343 Hz
#define MAX_FREQU		38 // 593 Hz
//uncomment to use the microphones
#define USE_MIC

//uncomment to use double buffering to send the FFT to the computer
#define DOUBLE_BUFFERING

//uncomment to send the mic data to the computer
//#define SEND_TO_COMPUTER

// initialize communication bus
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

/*
 *  @brief: the function takes an index that is proportional to the frequency and converts into a color
*	params :
*	uint16_t index_frequency: it takes the index of frequency calculated previously or any other parameter
*	return a color
*/
uint8_t color_convertion(uint16_t index_frequency)
{
	float color = 0;
	float A = (float)MAX_COLOR/(MAX_FREQU-MIN_FREQU);
	float B = -(float)MAX_COLOR/(MAX_FREQU-MIN_FREQU)*MIN_FREQU;
	if(index_frequency >= MAX_FREQU)
		color = MAX_COLOR;
	else if(index_frequency <= MIN_FREQU)
		color = MIN_COLOR;
	else
		color = (A*index_frequency+B);
	return color;

}

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //inits the motors
    motors_init();
    // init message bus
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    //init proximity sensors
    proximity_start();
    //start distance measurement
    distance_start();
    //start spi for leds
    spi_comm_start();

    //init pid_parameter Kp, Ki, Kd, THRESHOLD
    set_pid_param(Kp, Ki, Kd, THRESHOLD);

    //send_tab is used to save the state of the buffer to send (double buffering)
    //to avoid modifications of the buffer while sending it
    static float send_tab[FFT_SIZE];

#ifdef USE_MIC
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);
#endif  /* USE_MIC */

    /* Infinite loop. */
    while (1)
{
#ifdef USE_MIC
#ifdef SEND_TO_COMPUTER
        //waits until a result must be sent to the computer
        wait_send_to_computer();
#else
        /* motor controls */
        int state = get_state();
        //if we are close to an obstacle: moving forward is not permissible, but turning around is permissible
        if(get_stop() && (state == FRONT_LEFT || state == FRONT_RIGHT)){
			left_motor_set_speed(0);
			right_motor_set_speed(0);
        }
        else{
        	left_motor_set_speed(get_speed_left());
        	right_motor_set_speed(get_speed_right());
        }

        /* LED controls */
        uint8_t color = color_convertion(get_frequency());
        if(state == BACK_RIGHT){
        	// go from green to yellow
        	clear_leds();
        	toggle_rgb_led(LED4, GREEN_LED, MAX_COLOR);
        	toggle_rgb_led(LED4, RED_LED, color);
        }
        else if(state == BACK_LEFT){
        	// from red to pink
        	clear_leds();
        	toggle_rgb_led(LED6, RED_LED, MAX_COLOR);
        	toggle_rgb_led(LED6, BLUE_LED, color);
        }
        else if(state == FRONT_RIGHT){
        	// blue to pink
        	clear_leds();
        	toggle_rgb_led(LED8, BLUE_LED, MAX_COLOR);
        	toggle_rgb_led(LED8, RED_LED, color);
        }
        else if(state == FRONT_LEFT){
        	// red to yellow
        	clear_leds();
        	toggle_rgb_led(LED2, RED_LED, MAX_COLOR);
        	toggle_rgb_led(LED2, GREEN_LED, color);
        }
        else
        	clear_leds();
#endif /* SEND_TO_COMPUTER */
#ifdef DOUBLE_BUFFERING
        //we copy the buffer to avoid conflicts
        arm_copy_f32(get_audio_buffer_ptr(LEFT_OUTPUT), send_tab, FFT_SIZE);
#ifdef SEND_TO_COMPUTER
        SendFloatToComputer((BaseSequentialStream *) &SD3, send_tab, FFT_SIZE);
#endif /* SEND_TO_COMPUTER */
#else
        SendFloatToComputer((BaseSequentialStream *) &SD3, get_audio_buffer_ptr(LEFT_OUTPUT), FFT_SIZE);
#endif  /* DOUBLE_BUFFERING */
#else
        //temp tab used to store values in complex_float format
        //needed bx doFFT_c
        static complex_float temp_tab[FFT_SIZE];
        //time measurement variables
        volatile uint16_t time_fft = 0;
        volatile uint16_t time_mag  = 0;

        float* bufferCmplxInput = get_audio_buffer_ptr(LEFT_CMPLX_INPUT);
        float* bufferOutput = get_audio_buffer_ptr(LEFT_OUTPUT);

        uint16_t size = ReceiveInt16FromComputer((BaseSequentialStream *) &SD3, bufferCmplxInput, FFT_SIZE);

        if(size == FFT_SIZE){
            /*
            *   Optimized FFT
            */
            
            chSysLock();
            //reset the timer counter
            GPTD12.tim->CNT = 0;

            doFFT_optimized(FFT_SIZE, bufferCmplxInput);

            time_fft = GPTD12.tim->CNT;
            chSysUnlock();

            /*
            *   End of optimized FFT
            */

            /*
            *   Non optimized FFT
            */

            // //need to convert the float buffer into complex_float struct array
            // for(uint16_t i = 0 ; i < (2*FFT_SIZE) ; i+=2){
            //     temp_tab[i/2].real = bufferCmplxInput[i];
            //     temp_tab[i/2].imag = bufferCmplxInput[i+1];
            // }

            // chSysLock();
            // //reset the timer counter
            // GPTD12.tim->CNT = 0;

            // //do a non optimized FFT
            // doFFT_c(FFT_SIZE, temp_tab);

            // time_fft = GPTD12.tim->CNT;
            // chSysUnlock();
            
            // //reconverts the result into a float buffer
            // for(uint16_t i = 0 ; i < (2*FFT_SIZE) ; i+=2){
            //     bufferCmplxInput[i] = temp_tab[i/2].real;
            //     bufferCmplxInput[i+1] = temp_tab[i/2].imag;
            // }

            /*
            *   End of non optimized FFT
            */

            chSysLock();
            //reset the timer counter
            GPTD12.tim->CNT = 0;

            arm_cmplx_mag_f32(bufferCmplxInput, bufferOutput, FFT_SIZE);

            time_mag = GPTD12.tim->CNT;
            chSysUnlock();

            SendFloatToComputer((BaseSequentialStream *) &SD3, bufferOutput, FFT_SIZE);
            //chprintf((BaseSequentialStream *) &SDU1, "time fft = %d us, time magnitude = %d us\n",time_fft, time_mag);

        }
#endif  /* USE_MIC */
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
