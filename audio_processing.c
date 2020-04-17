/*
 * audio_processing.c
 *
 *  Created on: 3 ���. 2020 �.
 *      Author: ������
 */
#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>

#include <fft.h>
#include <arm_math.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, 1);

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
#define FREQ_INIT	    64	//1kHz

#define MAX_FREQ		128	//we don't analyze after this index to not use resources for nothing

#define FREQ_INIT_L		(FREQ_INIT-1) //adjustable
#define FREQ_INIT_H		(FREQ_INIT+1)

uint8_t colour;
/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/
void sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	//turn
	if(max_norm_index >= FREQ_INIT_L && max_norm_index <= FREQ_INIT_H){
		status(TRUE);
		colour = BLACK;
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

		sound_remote(micLeft_output);
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

void audio_detection(float *micleft, float *micright, float *micfront, float *micback,
					 float *micLeft_mag, float *micFront_mag){
	float max_norm_left = MIN_VALUE_THRESHOLD;
	float max_norm_front = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index_left = -1;
	int16_t max_norm_index_front = -1;
	float arg_micLeft = 0;
	float arg_micRight = 0;
	float arg_micFront = 0;
	float arg_micBack = 0;
	float arg_diff_side = 0;
	float arg_diff_fb = 0;

	doFFT_optimized(FFT_SIZE, micleft);
	doFFT_optimized(FFT_SIZE, micright);
	doFFT_optimized(FFT_SIZE, micfront);
	doFFT_optimized(FFT_SIZE, micback);

	arm_cmplx_mag_f32(micleft, micLeft_mag, FFT_SIZE);
	arm_cmplx_mag_f32(micfront, micFront_mag, FFT_SIZE);

	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
			if(micLeft_mag[i] > max_norm_left){
				max_norm_left = micLeft_mag[i];
				max_norm_index_left = i;
			}
			if(micFront_mag[i] > max_norm_front){
				max_norm_front = micFront_mag[i];
				max_norm_index_front = i;
			}
		}
	arg_micLeft = atan2f(micleft[(2*max_norm_index_left)+1], micleft[(2*max_norm_index_left)]);
	arg_micRight = atan2f(micright[(2*max_norm_index_left)+1], micright[2*max_norm_index_left]);
	arg_micFront = atan2f(micfront[(2*max_norm_index_front)+1], micfront[2*max_norm_index_front]);
	arg_micBack = atan2f(micback[(2*max_norm_index_front)+1], micback[2*max_norm_index_front]);
	arg_diff_side = arg_micLeft-arg_micRight;
	arg_diff_fb = arg_micFront-arg_micBack;
	int16_t arg_diff_side_deg = (180/PI)*arg_diff_side;
	int16_t arg_diff_fb_deg = (180/PI)*arg_diff_fb;

}

uint8_t get_colour_detected(void){
	return colour;
}
