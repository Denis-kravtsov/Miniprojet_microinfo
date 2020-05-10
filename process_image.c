#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <audio_processing.h>
#include <process_image.h>
#include "leds.h"

#define COLOUR_THRESHOLD 	1000
#define ADJUSTED_MEAN 		20
static float distance_cm = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle

int colour_image = -1;
//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);
bool rising_slope, falling_slope, colour_found = FALSE;
uint16_t lineWidth = 0;
int16_t temp = 0;

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
uint16_t extract_line_width(uint8_t *buffer){

	volatile uint16_t i = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint16_t begin = 0, end = 0, width = 0;
	uint32_t mean = 0;
	bool variable = FALSE;
	static uint16_t last_width = 0;
	uint16_t last_line_position = 0;
	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;
	if (buffer[IMAGE_BUFFER_SIZE/2]<160 && buffer[IMAGE_BUFFER_SIZE/2]>80){
		colour_found = TRUE;
	}
	else{
		colour_found = FALSE;
	}
	variable = FALSE;
	do{
		wrong_line = 0;
		if(!variable)
			i=0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE)){
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] > mean + ADJUSTED_MEAN && buffer[i+WIDTH_SLOPE] < mean){
		    	begin = i;
		    	stop = 1;
		        falling_slope = TRUE;
		        variable = TRUE;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (!variable)
			i = 0;
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE)){
		    stop = 0;
		    
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE - WIDTH_SLOPE){
		        if(buffer[i] < mean && buffer[i+WIDTH_SLOPE] > mean + ADJUSTED_MEAN){
		        	end = i;
		            stop = 1;
		            rising_slope = TRUE;
		        }i++;
		    }
		}

		if(i < (IMAGE_BUFFER_SIZE ) && (!end) && begin){  //if an end was not found
		    	end = IMAGE_BUFFER_SIZE;
		    	stop = 1;
		    	rising_slope = FALSE;
		    }
		if(i < (IMAGE_BUFFER_SIZE ) && end && !begin){  //if no begin found but an end was found
			begin = 1;
			falling_slope = FALSE;
		}
		else if (i > IMAGE_BUFFER_SIZE || (!end && !begin)){  //if no begin and no end was found
		    line_not_found = 1;
		    falling_slope = FALSE;
		    rising_slope = FALSE;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
			variable = TRUE;
		}
	}while(wrong_line);

	if(line_not_found){
		begin = 0;
		end = 0;
		width = last_width;
		line_position = last_line_position;
		variable = FALSE;
	}else{
		variable = FALSE;
		last_width = width = (end - begin);
		last_line_position = line_position = (begin + end)/2; //gives the line position.
	}

	//sets a maximum width or returns the measured width
	if((PXTOCM/width) > MAX_DISTANCE){
		return PXTOCM/MAX_DISTANCE;
	}else{
		return width;
	}
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 20, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();
	po8030_set_awb(0);
	//po8030_set_ae(0);
	//po8030_set_exposure(64, 0);
	//po8030_set_contrast(128);
	po8030_set_rgb_gain(0x5D, 0x7F, 0x0);

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

	bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		switch (colour_image){
				//Extracts only the red pixels
			case RED:
				for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
					//extracts first 5bits of the first byte
					//takes nothing from the second byte
					image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
					set_rgb_led(0, 10, 0, 0);
					set_rgb_led(1, 10, 0, 0);
					set_rgb_led(2, 10, 0, 0);
					set_rgb_led(3, 10, 0, 0);
				}
				break;
				//Extracts only the green pixels
			case GREEN:
				for(uint16_t i = 0 ; i < (2*IMAGE_BUFFER_SIZE) ; i+=2){
					//extracts last 3bits of the first byte and first 3bits of the second byte,
					//put them together in a unique byte and store them in the first six places of the byte
					temp = (((((((uint16_t)img_buff_ptr[i+1]&0xE0)>>5)+(((uint16_t)img_buff_ptr[i]&0x7)<<3))<<2))) - ((uint8_t)img_buff_ptr[i]&0xF8);
					if(temp <= 0){
						image[i/2] = 0;
					}
					else
						image[i/2] = temp;
					set_rgb_led(0, 0, 10, 0);
					set_rgb_led(1, 0, 10, 0);
					set_rgb_led(2, 0, 10, 0);
					set_rgb_led(3, 0, 10, 0);
				}

				break;
		}
		//search for a line in the image and gets its width in pixels
		lineWidth = extract_line_width(image);
		//converts the width into a distance between the robot and the camera
		if(lineWidth){
			distance_cm = PXTOCM/lineWidth;
		}

		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;
    }
}

uint16_t get_line_position(void){
	return line_position;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
void colour_status(bool status){
	colour_found = status;
}
bool get_falling_edge(void){
	return falling_slope;
}
bool get_rising_edge(void){
	return rising_slope;
}
bool get_colour_status(void){
	return colour_found;
}
uint16_t get_line_width(void){
	return lineWidth;
}
void set_color(int color){
	colour_image=color;
}
int get_colour(void){
	return colour_image;
}
