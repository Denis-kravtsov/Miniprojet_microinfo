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
	 int16_t sens_values_temp[2];
	 while(stop_loop == 0) {
	    	time = chVTGetSystemTime();

	    	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
	    	sens_values_temp[0]=prox_values.delta[4];
	    	chThdSleepUntilWindowed(time, time + MS2ST(10));
	    	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
	    	sens_values_temp[1]=prox_values.delta[4];
    		messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
	   		leftSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[0]*2 - prox_values.delta[1];
	   		rightSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[7]*2 - prox_values.delta[6];
	   		right_motor_set_speed(rightSpeed);
	   		left_motor_set_speed(leftSpeed);
	    	if(sens_values_temp[1]-sens_values_temp[0]!=MIN_THRESHOLD){
	    		avoidance_logic(prox_values);
	    	}


	   		chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz.
	    	break;
	 }
}


void obstacle_start(void){
	chThdCreateStatic(waObstacle, sizeof(waObstacle), NORMALPRIO, Obstacle, NULL);
}

void avoidance_logic(proximity_msg_t data){
	while((data.delta[3]*2) != 0){
		change(FALSE);
	}
	change(TRUE);
}
