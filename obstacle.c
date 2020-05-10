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

#define MIN_THRESHOLD 		10
#define TURN_TIME			2000
#define CIRCUIT_TIME		1500
#define DIRECTION_LEFT		-300
#define DIRECTION_RIGHT		300
#define IR_RIGHT_T			3.5
#define IR_LEFT_T			5
#define IR_RIGHT_TURN		4.5
#define IR_LEFT_TURN		5.5

static	uint8_t status = 0;


static THD_WORKING_AREA(waObstacle, 1024);

static THD_FUNCTION(Obstacle, arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);
	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	bool turn_init = FALSE;
	bool corridor_centrale = FALSE;
	proximity_msg_t prox_values;
	systime_t time, time2, time3, time4 = 0;
	int16_t corridor_approx_pos = 0;
	int16_t leftSpeed = 0, rightSpeed = 0;
	uint16_t dif_time, dif_time2 = 0;

	while(1) {
		messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
		//tells where the needed corridor is(left or right)
	    if(get_falling_edge() && get_rising_edge()){
	    	corridor_approx_pos = -(get_line_position() - (IMAGE_BUFFER_SIZE/2));
	    }
	    else if(get_falling_edge() && !get_rising_edge()){
	    	corridor_approx_pos = DIRECTION_LEFT;
	    }
	    else if(!get_falling_edge() && get_rising_edge()){
	    	corridor_approx_pos = DIRECTION_RIGHT;
	    }
	    else{
	    	corridor_approx_pos = 0;
	    }
	    //checks if the program is in standby mode
	    if(get_status()){
	    	//checks if an object was detected
			if(!check_prox(prox_values)){
				stopCurrentMelody();
				set_front_led(0);
				set_body_led(0);
				//checks if there are corridors on both sides of the robot(T intersection)
				if((get_distance_cm_prox(prox_values.delta[2]) > IR_RIGHT_T) && (get_distance_cm_prox(prox_values.delta[5]) > IR_LEFT_T)){
					//tells that the turn has been initiated
					turn_init = TRUE;
					//if the needed corridor is on the right, turn right
					if(corridor_approx_pos > 0){
						right_motor_set_speed(MOTOR_SPEED_LIMIT/24);
						left_motor_set_speed(MOTOR_SPEED_LIMIT);
						//experimental value to obtain a good curve during the turn
						chThdSleepMilliseconds(400);
					}
					//turns left
					else if(corridor_approx_pos < 0){
						right_motor_set_speed(MOTOR_SPEED_LIMIT);
						left_motor_set_speed(MOTOR_SPEED_LIMIT/24);
						chThdSleepMilliseconds(400);
					}
					//if robot turned for less than 90 degrees, corrects its position
					else if(corridor_approx_pos == 0){
						leftSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[0]*2 - prox_values.delta[1];
						rightSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[7]*2 - prox_values.delta[6];
						right_motor_set_speed(rightSpeed);
						left_motor_set_speed(leftSpeed);
						chThdSleepMilliseconds(200);
					}
					//timer that in the end gives the time of the exit from the turn (also used to change colours)
					time = chVTGetSystemTime();
				}
				//turns left in the normal turn (not T intesection)
				else if(((get_distance_cm_prox(prox_values.delta[5]) > IR_LEFT_TURN)) && (!turn_init) && (!corridor_centrale)){
					corridor_centrale = TRUE;
					right_motor_set_speed(MOTOR_SPEED_LIMIT);
					left_motor_set_speed(MOTOR_SPEED_LIMIT/24);
					chThdSleepMilliseconds(500);
					//timer that in the end gives the time of the exit from the turn
					time2 = chVTGetSystemTime();
				}
				//turns right in the normal turn (not T intesection)
				else if(((get_distance_cm_prox(prox_values.delta[2]) > IR_RIGHT_TURN)) && (!turn_init) && (!corridor_centrale)){
					corridor_centrale = TRUE;
					right_motor_set_speed(MOTOR_SPEED_LIMIT/24);
					left_motor_set_speed(MOTOR_SPEED_LIMIT);
					chThdSleepMilliseconds(500);
					//timer that in the end gives the time of the exit from the turn
					time2 = chVTGetSystemTime();
				}
				//goes straight in the corridor, and at the same time corrects its position, in order
				//not to touch the walls
				else{
					messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
					//1.unlocks the booleen variable, to allow the execution of conditions that make the robot turn
					//2.switches the colour
					if(turn_init){
						time3 = chVTGetSystemTime();
						//measures the time elapsed since the end of the turn(T intersection)
						dif_time = time3-time;
						if(dif_time > CIRCUIT_TIME){
							switch_colour();
							turn_init=FALSE;
						}
					}
					//1.unlocks the booleen variable, to allow the execution of conditions that make the robot turn in T intersection
					if(corridor_centrale){
						time4 = chVTGetSystemTime();
						//measures the time elapsed since the end of the turn(not T intersection)
						dif_time2 = time4-time2;
						if(dif_time2 > TURN_TIME){
							corridor_centrale=FALSE;
						}
					}
					//motor speeds to keep the robot centered in the corridor
					leftSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[0]*2 - prox_values.delta[1];
					rightSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[7]*2 - prox_values.delta[6];
					right_motor_set_speed(rightSpeed);
					left_motor_set_speed(leftSpeed);
				}
			}
			//obstacle detected
			else{
				right_motor_set_speed(0);
				left_motor_set_speed(0);
				set_front_led(1);
				set_body_led(1);
				playMelody(RUSSIA, ML_SIMPLE_PLAY, NULL);
			}
		}
	    //standby mode
	    else{
			turn_init = FALSE;
			//turns off colour detection
	    	set_color(-1);
	    	set_body_led(1);
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			set_rgb_led(0, 0, 0, 10);
			set_rgb_led(1, 0, 0, 10);
			set_rgb_led(2, 0, 0, 10);
			set_rgb_led(3, 0, 0, 10);
	    }
	 }
}
void obstacle_start(void){
	chThdCreateStatic(waObstacle, sizeof(waObstacle), NORMALPRIO, Obstacle, NULL);
}
//conversion of prox.delta values to cm (experimental values)
float get_distance_cm_prox(int prox_values){
	float cm = 0;
	cm = 100*pow(prox_values,-0.6);
	return cm;
}
//switches the colour
void switch_colour(void){
	if(get_colour()==RED){
		set_color(GREEN);
	}
	else if(get_colour()==GREEN){
		set_color(RED);
	}
}
//checks for the obstacle
int check_prox(proximity_msg_t data){
	int count = 0;
	//checks for the number of IR sensors that are not blocked by an object
	for(int i=0; i<8; i++){
		if (get_distance_cm_prox(data.delta[i]) > 4.5){
			count++;
		}
	}
	if(count <= 2)
		return TRUE;
	else
		return FALSE;
}
//gives the mode (standby or standard)
uint8_t get_status(void){
	return status;
}
//sets the mode
void set_status(uint8_t valeur){
	status = valeur;
}
