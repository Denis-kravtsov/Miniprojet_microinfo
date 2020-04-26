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
#define MIN_THRESHOLD 10
static THD_WORKING_AREA(waObstacle, 2048);


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
	 int mean_prox = 0;
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
	    	for(int i = 0; i < 8; i++){
	    		mean_prox += prox_values.delta[i];

	    	}
	    	mean_prox = mean_prox/8;
	    	//UNCOMMNENT THE NEEDED FUNCTION
	    	//TEST FUNCTION GENERAL(FOR DEBUG) 1
	    	if(abs(prox_values.delta[2] || prox_values.delta[5]) < 100 && !(get_colour_status())){
	      		turn_init=TRUE;
	    		while(turn_init){
	    		     right_motor_set_speed(0);
	    			 left_motor_set_speed(0);
	    			 break;
	    		}
	    		if(get_colour_status()){
	    		    turn_init = FALSE;
	    		}

	    	}
	    	//TEST FUNCTION GENERAL(FOR DEBUG) 2
	    	/*if(abs(prox_values.delta[2] || prox_values.delta[5]) < mean_prox && !(get_colour_status())){
	      		turn_init=TRUE;
	    		while(turn_init){
	    		     right_motor_set_speed(0);
	    			 left_motor_set_speed(0);
	    			 break;
	    		}
	    		if(get_colour_status()){
	    		    turn_init = FALSE;
	    		}

	    	}*/
	    	        //applies the speed from the PI regulator and the correction for the rotation
	    	//TEST FUNCTION 1
	    	/*if(abs(prox_values.delta[2] || prox_values.delta[5]) < 100 && !(get_colour_status())){
	    		turn_init=TRUE;
	    		while(turn_init){
	    			 right_motor_set_speed(MOTOR_SPEED_LIMIT/2 - ROTATION_COEFF * speed_correction);
	    			 left_motor_set_speed(MOTOR_SPEED_LIMIT/2 + ROTATION_COEFF * speed_correction);
	    			 break;
	    		}
	    		if(get_colour_status()){
	    			turn_init = FALSE;
	    		}

	    	}*/
	    	//TEST FUNCTION 2
	    	/*if((get_line_width()>IMAGE_BUFFER_SIZE/2) && ((prox_values.delta[2] || prox_values.delta[5]) < 50)){
	    		 right_motor_set_speed(MOTOR_SPEED_LIMIT/2 - ROTATION_COEFF * speed_correction);
	    		left_motor_set_speed(MOTOR_SPEED_LIMIT/2 + ROTATION_COEFF * speed_correction);
	    	}*/
	    	else{

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

