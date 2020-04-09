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
	 uint8_t rgb_state = 0, rgb_counter = 0;
	 uint16_t melody_state = 0, melody_counter = 0;

	 while(stop_loop == 0) {
	    		time = chVTGetSystemTime();

	    		messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
	    		leftSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[0]*2 - prox_values.delta[1];
	    		rightSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[7]*2 - prox_values.delta[6];
	    		right_motor_set_speed(rightSpeed);
	    		left_motor_set_speed(leftSpeed);

	    		switch(rgb_state) {
	    			case 0: // Red.
	    				set_rgb_led(0, 10, 0, 0);
	    				set_rgb_led(1, 10, 0, 0);
	  					set_rgb_led(2, 10, 0, 0);
	    				set_rgb_led(3, 10, 0, 0);
	 					break;
	    							case 1: // Green.
	    								set_rgb_led(0, 0, 10, 0);
	    								set_rgb_led(1, 0, 10, 0);
	    								set_rgb_led(2, 0, 10, 0);
	    								set_rgb_led(3, 0, 10, 0);
	    								break;
	    							case 2: // Blue.
	    								set_rgb_led(0, 0, 0, 10);
	    								set_rgb_led(1, 0, 0, 10);
	    								set_rgb_led(2, 0, 0, 10);
	    								set_rgb_led(3, 0, 0, 10);
	    								break;
	    			            }
	    						rgb_counter++;
	    						if(rgb_counter == 100) {
	    							rgb_counter = 0;
	    							rgb_state = (rgb_state+1)%3;
	    							set_body_led(2);
	    							set_front_led(2);
	    						}

	    						melody_counter++;
	    						if(melody_counter == 2000) {
	    							melody_counter = 0;
	    							melody_state = (melody_state+1)%NB_SONGS;
	    							playMelody(melody_state, ML_SIMPLE_PLAY, NULL);
	    						}

	    						chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz.
	    						break;
	 }
}


void obstacle_start(void){
	chThdCreateStatic(waObstacle, sizeof(waObstacle), NORMALPRIO, Obstacle, NULL);
}

