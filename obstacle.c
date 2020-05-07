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

/*#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     13 // [cm]
#define RAYON_INT			2	// [cm]
#define RAYON_EXT			9	// [cm]
#define PI                  3.1415926536f
//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE      5.5f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define POSITION_CONTROL    1
static int16_t counter_step_right = 0;          // in [step]
static int16_t counter_step_left = 0; 		    // in [step]
static int16_t position_to_reach_right = 0;	    // in [step]
static int16_t position_to_reach_left = 0;	    // in [step]
static uint8_t position_right_reached = 0;
static uint8_t position_left_reached = 0;
static uint8_t state_motor = 0;*/

static THD_WORKING_AREA(waObstacle, 4096);
systime_t time2;
systime_t time3;
bool turn_init = 0;
static bool colour_change_init = FALSE;
static THD_FUNCTION(Obstacle, arg)
{
	(void) arg;
	chRegSetThreadName(__FUNCTION__);
	//if(acquire_status()){
	 messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	 proximity_msg_t prox_values;
	 int16_t leftSpeed = 0, rightSpeed = 0;
	 //int16_t prox_values_temp[8];
	 uint8_t stop_loop = 0;
	 systime_t time;

	 volatile int16_t speed_correction = 0;
	 volatile int16_t corridor_approx_pos = 0;

	 //volatile int mean_prox = 0;
	 volatile uint16_t dif_time;
	 volatile uint16_t dif_time2;
	 while(stop_loop == 0) {
		 	//mean_prox = 0;
	    	time = chVTGetSystemTime();
	    	colour_change_init = FALSE;
	    	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
	    	if(get_falling_edge() && get_rising_edge()){
	    		corridor_approx_pos = -(get_line_position() - (IMAGE_BUFFER_SIZE/2));
	    		//speed_correction = (corridor_approx_pos);
	    	    //if the line is nearly in front of the camera, don't rotate
	    	    if(abs(speed_correction) < ROTATION_THRESHOLD){
	    	    	speed_correction = 0;
	    	    }
	    	}
	    	else if(get_falling_edge() && !get_rising_edge()){
	    		corridor_approx_pos = -300;
	    	}
	    	else if(!get_falling_edge() && get_rising_edge()){
	    		corridor_approx_pos = 300;
	    	}
	    	else
	    		corridor_approx_pos = 0;

	    	if(!check_prox(prox_values)){
	    		if((get_distance_cm_prox(prox_values.delta[2]) > 6) && (get_distance_cm_prox(prox_values.delta[5]) > 6)){
	    			turn_init = TRUE;
	    			if(corridor_approx_pos > 0){
	    				for(int i=0; i<20;i++){
	    					right_motor_set_speed(-MOTOR_SPEED_LIMIT);
	    					left_motor_set_speed(MOTOR_SPEED_LIMIT);
	   					//break;
	    				}
	    			}
	   				else if(corridor_approx_pos < 0){
	   					for(int i=0; i<20;i++){

	   						right_motor_set_speed(MOTOR_SPEED_LIMIT);
	   						left_motor_set_speed(-MOTOR_SPEED_LIMIT);
	   						//break;
	   					}
	    			}
	   				else if(corridor_approx_pos == 0){
	   				    leftSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[0]*2 - prox_values.delta[1];
	   					rightSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[7]*2 - prox_values.delta[6];
	   					right_motor_set_speed(rightSpeed);
	   					left_motor_set_speed(leftSpeed);
	   				}
	   				if(!time2){
	   						time2=chVTGetSystemTime();
	   				}
	   				dif_time = time-time2;
	   				if(dif_time > 2000){
	   					time2 = chVTGetSystemTime();
	   				//CECI VA PRESQUE JAMAIS ETRE UTILE MAIS A VERIFIER!!!
	   						//switch_colour();
	   				}
	    			/*if(!time2){
	    				time2=chVTGetSystemTime();
	    		    	}
	    				    				 		dif_time = time-time2;
	    				    				 		if(dif_time > 2000){
	    				    				 			time2 = chVTGetSystemTime();
	    				    			//CECI VA PRESQUE JAMAIS ETRE UTILE MAIS A VERIFIER!!!
	    				    				 			switch_colour();
	    				    				 		}*/
	    		}
	    		else if(((get_distance_cm_prox(prox_values.delta[5]) > 5)) &&
	    				((get_distance_cm_prox(prox_values.delta[0]) > 5)&&(get_distance_cm_prox(prox_values.delta[7]) > 5))){
	    			for(int i=0; i<100;i++){
	    				//turn_init = TRUE;
	    				right_motor_set_speed(MOTOR_SPEED_LIMIT);
	    	   			left_motor_set_speed(MOTOR_SPEED_LIMIT/24);
	    				//break;
	    			}
	    		}
	    	//}

	    		else if(((get_distance_cm_prox(prox_values.delta[2]) > 5))  &&
	    			    ((get_distance_cm_prox(prox_values.delta[0]) > 5)&&(get_distance_cm_prox(prox_values.delta[7]) > 5))){
	    			for(int i=0; i<100;i++){
	    			    //turn_init = TRUE;
	    				right_motor_set_speed(MOTOR_SPEED_LIMIT/24);
	    				left_motor_set_speed(MOTOR_SPEED_LIMIT);
	    			    //break;
	    			}
	    		}
	    	//}
	    	else{
		 	 	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
		 	 	//time2=chVTGetSystemTime();
		 	 	//colour_change_init= FALSE;
		 	 	if(!time3){
		 	 		time3=chVTGetSystemTime();
		 	 	}
		 	 	if(turn_init){
		 	 	//LOGIQUR DERRIRE: les capteurs IR sur les cotes doivent etre libres des 2 cotes(choix du corridor) ou
		 	 	//1 des 2 capteurs sur les cotes est libre et les capteurs devant sont libres aussi(retour couloir central)
		 	 		time3 = chVTGetSystemTime();
		 	 		dif_time2 = time3-time2;
		 	 		if(dif_time2 > 5000){

		 	 			switch_colour();
		 	 			turn_init=FALSE;
		 	 		}
		 	 	}
			    leftSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[0]*2 - prox_values.delta[1];
				rightSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[7]*2 - prox_values.delta[6];
				right_motor_set_speed(rightSpeed);
				left_motor_set_speed(leftSpeed);
	    	}
	    }
	    else{
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			set_front_led(1);
			set_body_led(1);
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
		cm = 100*pow(prox_values,-0.6);
	return cm;
}

void switch_colour(void){
	//colour_change_init=TRUE;
	if(get_colour()==RED1){
		set_color(GREEN1);
	}
	else if(get_colour()==RED2){
		set_color(GREEN1);
		//break;
	}
	else if(get_colour()==GREEN1){
		set_color(RED1);
		//break;
	}
	else if(get_colour()==GREEN2){
		set_color(RED1);
		//break;
	}
}

int check_prox(proximity_msg_t data) {
	int count = 0;
	for(int i=0; i<8; i++){
		if (get_distance_cm_prox(data.delta[i]) > 3){
			count++;
		}
	}
	if(count <= 2)
		return TRUE;
	else
		return FALSE;
}

