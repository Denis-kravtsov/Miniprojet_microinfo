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
static bool colour_change_init = FALSE;
static THD_FUNCTION(Obstacle, arg)
{
	(void) arg;
	chRegSetThreadName(__FUNCTION__);
	if(acquire_status()){
	 messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	 proximity_msg_t prox_values;
	 int16_t leftSpeed = 0, rightSpeed = 0;
	 //int16_t prox_values_temp[8];
	 uint8_t stop_loop = 0;
	 systime_t time;

	 volatile int16_t speed_correction = 0;
	 int16_t corridor_approx_pos = 0;
	 bool turn_init = FALSE;
	 //volatile int mean_prox = 0;
	 volatile uint16_t dif_time;
	 while(stop_loop == 0) {
		 	//mean_prox = 0;
	    	time = chVTGetSystemTime();
	    	colour_change_init = FALSE;
	    	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
	    	if(get_falling_edge() || get_rising_edge()){
	    		corridor_approx_pos = -(get_line_position() - (IMAGE_BUFFER_SIZE/2));
	    		//speed_correction = (corridor_approx_pos);
	    	    //if the line is nearly in front of the camera, don't rotate
	    	    if(abs(speed_correction) < ROTATION_THRESHOLD){
	    	    	speed_correction = 0;
	    	    }
	    	}
	    	/*for(uint16_t i = 0; i < 8; i++){
	    		mean_prox += prox_values.delta[i];

	    	}
	    	mean_prox /= 8;*/

	    	//UNCOMMNENT THE NEEDED FUNCTION
	    	//TEST FUNCTION GENERAL(FOR DEBUG) 1
	    	/*if(((get_distance_cm_prox(prox_values.delta[2]) > 4) && (get_distance_cm_prox(prox_values.delta[5]) > 4)) && (corridor_approx_pos <= 0)){ //&& !(get_colour_status())){
	      	//	turn_init=TRUE;
   		     right_motor_set_speed(0);
   			 left_motor_set_speed(0);
   			 	 dif_time=time-time2;
	 		  	if(dif_time > 2000 || !time2){
	    		//while(turn_init){
	    				time2 = chVTGetSystemTime();


	    			 //if(get_colour()==BLUE1){
	    				// set_color(RED);
	    			 	//    			     	 	    			}

	 		  	if(get_colour()==RED2){
	    				 set_color(GREEN1);
	    				 break;
	    			     time2 = chVTGetSystemTime();
	    			     // chThdSleepMilliseconds(100);
	    			 }
	    			 else if(get_colour()==GREEN1){
	    				 set_color(RED2);
	    			     time2 = chVTGetSystemTime();
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
	    	//LOGIQUR DERRIRE: les capteurs IR sur les cotes doivent etre libres des 2 cotes(choix du corridor) ou
	    	//1 des 2 capteurs sur les cotes est libre et les capteurs devant sont libres aussi(retour couloir central)
	    	if((((get_distance_cm_prox(prox_values.delta[2]) > 4) && (get_distance_cm_prox(prox_values.delta[5]) > 4))||
	    	   (((get_distance_cm_prox(prox_values.delta[2]) > 4) || (get_distance_cm_prox(prox_values.delta[5]) > 4))&&
	    		((get_distance_cm_prox(prox_values.delta[0]) > 4)&&(get_distance_cm_prox(prox_values.delta[7]) > 4))))&&
	    		 (corridor_approx_pos < 0)){ //!(get_colour_status())){
	    		//turn_init=TRUE;
	    		//while(turn_init){
	    		//for(int i=0; i<10;i++){
	    			turn_init = TRUE;
	    			right_motor_set_speed(MOTOR_SPEED_LIMIT);
	    			left_motor_set_speed(MOTOR_SPEED_LIMIT/6);

	    								 //right_motor_set_speed(MOTOR_SPEED_LIMIT/2 - ROTATION_COEFF * speed_correction);
	    								 //left_motor_set_speed(MOTOR_SPEED_LIMIT/2 + ROTATION_COEFF * speed_correction);
	    			//break;
	    		//}
	    		if(!time2){
		 			time2=chVTGetSystemTime();
		 		}
	    		dif_time = time-time2;
	    		//CECI VA PRESQUE JAMAIS ETRE UTILE MAIS A VERIFIER!!!
	    		if(dif_time > 1000){
	    			time2 = chVTGetSystemTime();
	    			switch_colour();
	    		}
	    		//else if(get_colour()==RED2){
	    			//set_color(RED1);
	    		//}
	    	}
	    	//TEST FUNCTION 2
	 	  else if((((get_distance_cm_prox(prox_values.delta[2]) > 4) && (get_distance_cm_prox(prox_values.delta[5]) > 4))||
	 	 		  (((get_distance_cm_prox(prox_values.delta[2]) > 4) || (get_distance_cm_prox(prox_values.delta[5]) > 4))&&
				   ((get_distance_cm_prox(prox_values.delta[0]) > 4)&&(get_distance_cm_prox(prox_values.delta[7]) > 4))))&&
	 			    (corridor_approx_pos >= 0)){ //!(get_colour_status())){

	 		//for(int i=0; i<10;i++){
	 		    turn_init = TRUE;
	 			right_motor_set_speed(MOTOR_SPEED_LIMIT/6);
	 			left_motor_set_speed(MOTOR_SPEED_LIMIT);
	 									 //right_motor_set_speed(MOTOR_SPEED_LIMIT/2 - ROTATION_COEFF * speed_correction);
	 									 //left_motor_set_speed(MOTOR_SPEED_LIMIT/2 + ROTATION_COEFF * speed_correction);
	 		//	break;
	 		//}

	 		if(!time2){
	 			time2=chVTGetSystemTime();
	 		}
	 		dif_time = time-time2;
	 		if(dif_time > 1000){
	 			time2 = chVTGetSystemTime();
//CECI VA PRESQUE JAMAIS ETRE UTILE MAIS A VERIFIER!!!
	 			switch_colour();
	 		}
	 		//else if(get_colour()==RED2){
	 			//set_color(RED1);
			//}
	 	  }
	    	else{
		 	 	//messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
		 	 	time2=chVTGetSystemTime();
		 	 	if(turn_init && !colour_change_init){
		 	 	//LOGIQUR DERRIRE: les capteurs IR sur les cotes doivent etre libres des 2 cotes(choix du corridor) ou
		 	 	//1 des 2 capteurs sur les cotes est libre et les capteurs devant sont libres aussi(retour couloir central)
		 	 		switch_colour();
		 	 		turn_init=FALSE;
		 	 	}
			    leftSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[0]*2 - prox_values.delta[1];
				rightSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[7]*2 - prox_values.delta[6];
				right_motor_set_speed(rightSpeed);
				left_motor_set_speed(leftSpeed);
	    	}


	   		chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz.
	   		break;

	}
	}
}


void obstacle_start(void){
	chThdCreateStatic(waObstacle, sizeof(waObstacle), NORMALPRIO, Obstacle, NULL);
}

int get_distance_cm_prox(int prox_values){
	int cm = 0;
		cm = 100*pow(prox_values,-0.6);
	return cm;
}
void switch_colour(void){
	colour_change_init=TRUE;
	if(get_colour()==RED1){
		set_color(RED2);
	}
	else if(get_colour()==RED2){
		set_color(GREEN1);
		//break;
	}
	else if(get_colour()==GREEN1){
		set_color(GREEN2);
		//break;
	}
	else if(get_colour()==GREEN2){
		set_color(RED1);
		//break;
	}
}
