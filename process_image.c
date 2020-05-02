#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <audio_processing.h>
#include <process_image.h>
#include "leds.h"
#define COLOUR_THRESHOLD 1000
static float distance_cm = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
int colour_image=1;
//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);
bool rising_slope, falling_slope, colour_found = FALSE;
uint16_t lineWidth = 0;
/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
uint16_t extract_line_width(uint8_t *buffer){

	volatile uint16_t i = 0;
	uint16_t begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;
	bool variable = FALSE;
	static uint16_t last_width = 0;
	uint16_t last_line_position = 0;
	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;
	if (buffer[IMAGE_BUFFER_SIZE]<120 && buffer[IMAGE_BUFFER_SIZE]>80){
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
		//line_not_found = 1;
		/*chprintf((BaseSequentialStream *)&SD3, " line_not_found1 = %f\n\r", line_not_found);
		chprintf((BaseSequentialStream *)&SD3, " begin1 = %f\n\r", begin);
		chprintf((BaseSequentialStream *)&SD3, " end1 = %f\n\r", end);
		chprintf((BaseSequentialStream *)&SD3, " stop1 = %f\n\r", stop);
		chprintf((BaseSequentialStream *)&SD3, " mean1 = %f\n\r", mean);*/
		/*volatile uint8_t debug_buffer[IMAGE_BUFFER_SIZE/4];
		for(int j = 0; j<IMAGE_BUFFER_SIZE; j+=4)
			debug_buffer[j/4] = buffer[j];*/
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{ 
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] > mean+20 && buffer[i+WIDTH_SLOPE] < mean)
		    {
		        begin = i;
		       // chprintf((BaseSequentialStream *)&SD3, " begin = %f\n\r", begin);
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
		    
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE - WIDTH_SLOPE)
		    {
		        if(buffer[i] < mean && buffer[i+WIDTH_SLOPE] > mean+20)
		        {
		            end = i;
		            //chprintf((BaseSequentialStream *)&SD3, " end = %f\n\r", end);
		            stop = 1;
		            rising_slope = TRUE;
		        }
		        i++;
		    }


		    //if an end was not found
		}
		if(i < (IMAGE_BUFFER_SIZE ) && (!end) && begin){//A VERIFIER!!!
		    	end = IMAGE_BUFFER_SIZE;
		    	//chprintf((BaseSequentialStream *)&SD3, " end = %f\n\r", end);
		    	stop = 1;
		    	rising_slope = FALSE;
		    }
		if(i < (IMAGE_BUFFER_SIZE ) && end && !begin){ //if no begin found but an end was found
			begin = 1;
			falling_slope = FALSE;
		}
		else if (i > IMAGE_BUFFER_SIZE || (!end && !begin))//if no begin and no end was found
		{
		    line_not_found = 1;
		    falling_slope = FALSE;
		    rising_slope = FALSE;
		   // colour_found = TRUE; //A VERIFIER!!!
		}
		/*chprintf((BaseSequentialStream *)&SD3, " line_not_found2 = %f\n\r", line_not_found);
		chprintf((BaseSequentialStream *)&SD3, " begin2 = %f\n\r", begin);
		chprintf((BaseSequentialStream *)&SD3, " end2 = %f\n\r", end);
		chprintf((BaseSequentialStream *)&SD3, " stop2 = %f\n\r", stop);
		chprintf((BaseSequentialStream *)&SD3, " mean2 = %f\n\r", mean);*/
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
	//	line_position = last_line_position;
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
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();
	//chThdSleepMilliseconds(1000);
	//po8030_set_awb(0);
	//po8030_set_ae(0);
	//po8030_set_exposure(128, 0);
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

		//Extracts only the red pixels
		switch (colour_image){
			case BLUE1:
				for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
					//image[i/2] = (uint8_t)img_buff_ptr[i+1]&0x1F;
					image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;

				}


				break;
			case RED:
				for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
					//extracts first 5bits of the first byte
					//takes nothing from the second byte
					image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
					//image[i/2] = (uint8_t)img_buff_ptr[i+1]&0x1F;
					set_front_led(1);
					set_body_led(0);
				}

				break;
			case GREEN://A VERIFIER LA COULEUR VERTE!!!
				for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
					//extracts first 5bits of the first byte
					//takes nothing from the second byte
					image[i/2] = ((((uint8_t)img_buff_ptr[i+1]&0xE0)>>5)+(((uint8_t)img_buff_ptr[i]&0x7)<<3))<<2;
					//image[i/2] = (uint8_t)img_buff_ptr[i+1]&0x1F;
					set_body_led(1);
					set_front_led(0);
				}

				break;
		/*	case BLUE2://A VERIFIER LA COULEUR VERTE!!!
				for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
					//extracts first 5bits of the first byte
					//takes nothing from the second byte
					//image[i/2] = ((((uint8_t)img_buff_ptr[i+1]&0xE0)>>5)+(((uint8_t)img_buff_ptr[i]&0x7)<<3))<<2;
					image[i/2] = (uint8_t)img_buff_ptr[i+1]&0x1F;

				}*/
				break;

		}
		//for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//image[i/2] = ((uint8_t)img_buff_ptr[i+1]&0x1F)<<3;
			//image[i/2] = ((((uint8_t)img_buff_ptr[i+1]&0xE0)>>5)+(((uint8_t)img_buff_ptr[i]&0x7)<<3))<<2;
			//image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
	//	}

		//search for a line in the image and gets its width in pixels
		lineWidth = extract_line_width(image);
		//chprintf((BaseSequentialStream *)&SD3, " lineWidth = %f\n\r", lineWidth);
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

float get_distance_cm(void){
	return distance_cm;
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
