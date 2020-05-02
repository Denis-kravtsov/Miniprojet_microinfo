/*
 * obstacle.c
 *
 *  Created on: 9 apr 2020
 *      Author: Marco
 */

#include <obstacle.h>
#include <arm_math.h>
#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include "sensors/proximity.h"
#include "motors.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"
#include "audio/audio_thread.h"
#include "leds.h"
#include "process_image.h"
#include <audio_processing.h>

#define MIN_THRESHOLD 10
static THD_WORKING_AREA(waObstacle, 2048);
systime_t time2;

static THD_FUNCTION(Obstacle, arg)
{
	 (void) arg;
	 chRegSetThreadName(__FUNCTION__);

	 messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	 proximity_msg_t prox_values;
	 int16_t leftSpeed = 0, rightSpeed = 0;
	 int16_t prox_values_temp[8];
	 uint8_t stop_loop = 0;
	 systime_t time;

	 volatile int16_t speed_correction = 0;
	 int16_t corridor_approx_pos = 0;
	 bool turn_init = FALSE;
	 volatile int mean_prox = 0;
	 volatile uint16_t dif_time;
	 while(stop_loop == 0) {
		 	mean_prox = 0;
	    	time = chVTGetSystemTime();
	    	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
	    	if(get_falling_edge() || get_rising_edge()){
	    		corridor_approx_pos = -(get_line_position() - (IMAGE_BUFFER_SIZE/2));
	    		speed_correction = (corridor_approx_pos);
	    	    //if the line is nearly in front of the camera, don't rotate
	    	    if(abs(speed_correction) < ROTATION_THRESHOLD){
	    	    	speed_correction = 0;
	    	    }
	    	}
	    	for(uint16_t i = 0; i < 8; i++){
	    		mean_prox += prox_values.delta[i];

	    	}
	    	mean_prox /= 8;

	    	//UNCOMMNENT THE NEEDED FUNCTION
	    	//TEST FUNCTION GENERAL(FOR DEBUG) 1
	    /*	if(((prox_values.reflected[2] > 3500) || prox_values.reflected[5] > 3500)){ //&& !(get_colour_status())){
	      	//	turn_init=TRUE;
   		     right_motor_set_speed(0);
   			 left_motor_set_speed(0);
   			 	 dif_time=time-time2;
	 		  	if(dif_time > 2000 || !time2){
	    		//while(turn_init){
	    				time2 = chVTGetSystemTime();


	    			 if(get_colour()==BLUE1){
	    			 	    			     	 	    			set_color(RED);
	    			 	    			     	 	    			}
	    			 else if(get_colour()==RED){
	    			     	 	    			set_color(GREEN);
	    			     	 	    			break;
	    			     	 	    			//time2 = chVTGetSystemTime();
	    			     	 	    			// chThdSleepMilliseconds(100);
	    			     	 	    			}
	    			     	 	    			else if(get_colour()==GREEN){
	    			     	 	    				set_color(RED);
	    			     	 	    				//time2 = chVTGetSystemTime();
	    			     	 	    				break;

	    			     	 	    			//	 chThdSleepMilliseconds(100);
	    			     	 	    			}
	    			 break;
	    		}
	    		if(get_colour_status()){
	    		    turn_init = FALSE;
	    		}

	    	}*/

	    	//TEST FUNCTION 1
	    	if(((prox_values.reflected[2] > 3600||(prox_values.reflected[5] > 3600)) && corridor_approx_pos < 0)){ //!(get_colour_status())){
	    		//turn_init=TRUE;
	    		//while(turn_init){
	    		for(int i=0; i<10;i++){
	    			right_motor_set_speed(MOTOR_SPEED_LIMIT);
	    			left_motor_set_speed(-MOTOR_SPEED_LIMIT);

	    								 //right_motor_set_speed(MOTOR_SPEED_LIMIT/2 - ROTATION_COEFF * speed_correction);
	    								 //left_motor_set_speed(MOTOR_SPEED_LIMIT/2 + ROTATION_COEFF * speed_correction);
	    			//break;
	    		}
		 		  if(!time2){
		 			  time2=chVTGetSystemTime();
		 		  }
	    		dif_time = time-time2;
	    		if(dif_time > 2000){
	    			time2 = chVTGetSystemTime();
	    			switch_colour();
	    		}
    	 	    		//	else if(get_colour()==BLUE2){
    	 	    			//	set_color(RED);
    	 	    			//}

	    	}
	    	//TEST FUNCTION 2
	 	  else if(((prox_values.reflected[2] > 3600) ||(prox_values.reflected[2] > 3600)) && corridor_approx_pos > 0){ //!(get_colour_status())){

	 		for(int i=0; i<10;i++){
	 			right_motor_set_speed(-MOTOR_SPEED_LIMIT);
	 			left_motor_set_speed(MOTOR_SPEED_LIMIT);
	 									 //right_motor_set_speed(MOTOR_SPEED_LIMIT/2 - ROTATION_COEFF * speed_correction);
	 									 //left_motor_set_speed(MOTOR_SPEED_LIMIT/2 + ROTATION_COEFF * speed_correction);
	 			break;
	 		}
	 		dif_time = time-time2;
	 		  if(!time2){
	 			  time2=chVTGetSystemTime();
	 		  }
	 		  if(dif_time > 2000){
	 			 time2 = chVTGetSystemTime();
	 			 switch_colour();
						//else if(get_colour()==BLUE2){
							//set_color(RED);
						//}
			}
	 	  }
	    	else{
		 	 	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
			    leftSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[0]*2 - prox_values.delta[1];
				rightSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[7]*2 - prox_values.delta[6];
				right_motor_set_speed(rightSpeed);
				left_motor_set_speed(leftSpeed);
	    	}


	   		chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz.
	   		break;

	}
}


void obstacle_start(void){
	chThdCreateStatic(waObstacle, sizeof(waObstacle), NORMALPRIO, Obstacle, NULL);
}

int get_distance_cm_prox(int prox_values){
	int cm = 0;
		cm = -0.0174*prox_values + 71.274;
	return cm;
}
void switch_colour(void){
	if(get_colour()==RED){
		set_color(GREEN);
		//break;
	}
	else if(get_colour()==BLUE1){
		set_color(GREEN);
		//break;
	}
	else if(get_colour()==GREEN){
		set_color(RED);
		//break;
	}
}
