/*
 * audio_processing.h
 *
 *  Created on: 3 апр. 2020 г.
 *      Author: денчик
 */

#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

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

enum {
	//containing the colors to be detected, which correspond to the paths to be taken.
	//Two times the same color because the first one is to enter in the corridor and
	//the second is to return at the initial position.
	RED = 0,
	GREEN
};

void processAudioData(int16_t *data, uint16_t num_samples);

/*
*	put the invoking thread into sleep until it can process the audio datas
*/
void wait_send_to_computer(void);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

//void audio_detection(float *micleft, float *micright, float *micfront, float *micback,
	//				 float *micLeft_mag, float *micFront_mag);
/*
 * Simple function that return the color to follow
 */
uint8_t get_colour_detected(void);

#endif /* AUDIO_PROCESSING_H */
