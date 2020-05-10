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
#define CIRCUIT_TIME_RED	1500
#define CIRCUIT_TIME_GREEN	1500
static	uint8_t status = 0;

static THD_WORKING_AREA(waObstacle, 4096);

static THD_FUNCTION(Obstacle, arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);
	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");

	bool turn_init = FALSE;
	bool corridor_centrale = FALSE;

	proximity_msg_t prox_values;
	systime_t time = 0;
	systime_t time2 = 0;
	systime_t time3 = 0;
	systime_t time4 = 0;
	systime_t time5 = 0;

	int16_t corridor_approx_pos = 0;
	int16_t leftSpeed = 0, rightSpeed = 0;
	uint16_t dif_time  = 0;
	uint16_t dif_time2 = 0;
	uint16_t dif_time3 = 0;
	uint16_t dif_time4 = 0;

	while(1) {
		time = chVTGetSystemTime();
		messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
	    if(get_falling_edge() && get_rising_edge()){
	    	corridor_approx_pos = -(get_line_position() - (IMAGE_BUFFER_SIZE/2));

	    }
	    else if(get_falling_edge() && !get_rising_edge()){
	    	corridor_approx_pos = -300;
	    }
	    else if(!get_falling_edge() && get_rising_edge()){
	    	corridor_approx_pos = 300;
	    }
	    else
	    	corridor_approx_pos = 0;

	    if(get_status()){
			if(!check_prox(prox_values)){
				stopCurrentMelody();
				set_front_led(0);
				set_body_led(0);
				if((get_distance_cm_prox(prox_values.delta[2]) > 3.5) && (get_distance_cm_prox(prox_values.delta[5]) > 5)){
					turn_init = TRUE;
					if(corridor_approx_pos > 0){
						right_motor_set_speed(MOTOR_SPEED_LIMIT/24);
						left_motor_set_speed(MOTOR_SPEED_LIMIT);
						chThdSleepMilliseconds(400);
					}
					else if(corridor_approx_pos < 0){
						right_motor_set_speed(MOTOR_SPEED_LIMIT);
						left_motor_set_speed(MOTOR_SPEED_LIMIT/24);
						chThdSleepMilliseconds(400);
					}
					else if(corridor_approx_pos == 0){
						leftSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[0]*2 - prox_values.delta[1];
						rightSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[7]*2 - prox_values.delta[6];
						right_motor_set_speed(rightSpeed);
						left_motor_set_speed(leftSpeed);
						chThdSleepMilliseconds(200);
					}
					/*if(!time2){
						time2=chVTGetSystemTime();
					}*/
					dif_time = time-time2;
					if(dif_time > TURN_TIME){
						time2 = chVTGetSystemTime();
					}
				}
				else if(((get_distance_cm_prox(prox_values.delta[5]) > 5.5)) && !turn_init && !corridor_centrale){
					corridor_centrale = TRUE;
					right_motor_set_speed(MOTOR_SPEED_LIMIT);
					left_motor_set_speed(MOTOR_SPEED_LIMIT/24);
					chThdSleepMilliseconds(500);
					/*if(!time4){
						time4=chVTGetSystemTime();
					}*/
					dif_time4 = time-time4;
					if(dif_time > TURN_TIME){
						time4 = chVTGetSystemTime();
					}
				}
				else if(((get_distance_cm_prox(prox_values.delta[2]) > 4.5)) && !turn_init && !corridor_centrale){
					corridor_centrale = TRUE;

					right_motor_set_speed(MOTOR_SPEED_LIMIT/24);
					left_motor_set_speed(MOTOR_SPEED_LIMIT);
					chThdSleepMilliseconds(500);
					/*if(!time4){
						time4=chVTGetSystemTime();
					}*/
					dif_time4 = time-time4;
					if(dif_time4 > TURN_TIME){
						time4 = chVTGetSystemTime();
					}
				}
				else{
					messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
					/*if(!time3){
						time3=chVTGetSystemTime();
					}*/
					if(turn_init){
						//LOGIQUR DERRIRE: les capteurs IR sur les cotes doivent etre libres des 2 cotes(choix du corridor) ou
						//1 des 2 capteurs sur les cotes est libre et les capteurs devant sont libres aussi(retour couloir central)
						time3 = chVTGetSystemTime();
						dif_time2 = time3-time2;
						if(get_colour() == RED){
							if(dif_time2 > CIRCUIT_TIME_RED){
								switch_colour();
								turn_init=FALSE;
							}
						}
						else{
							if(dif_time2 > CIRCUIT_TIME_GREEN){
								switch_colour();
								turn_init=FALSE;
							}
						}
					}
					/*if(!time5){
						time5=chVTGetSystemTime();
					}*/
					if(corridor_centrale){
						//LOGIQUR DERRIRE: les capteurs IR sur les cotes doivent etre libres des 2 cotes(choix du corridor) ou
						//1 des 2 capteurs sur les cotes est libre et les capteurs devant sont libres aussi(retour couloir central)
						time5 = chVTGetSystemTime();
						dif_time3 = time5-time4;
						if(dif_time3 > TURN_TIME){
							corridor_centrale=FALSE;
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
				playMelody(RUSSIA, ML_SIMPLE_PLAY, NULL);
				//break;
			}
		}
	    else{
			turn_init = FALSE;
	    	set_color(-1);
	    	set_body_led(1);
		right_motor_set_speed(0);
		left_motor_set_speed(0);
		set_rgb_led(0, 0, 0, 10);
		set_rgb_led(1, 0, 0, 10);
		set_rgb_led(2, 0, 0, 10);
		set_rgb_led(3, 0, 0, 10);
	    }
	    	//chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz.
	    	//break;
	 }
}

void obstacle_start(void){
	chThdCreateStatic(waObstacle, sizeof(waObstacle), NORMALPRIO, Obstacle, NULL);
}

float get_distance_cm_prox(int prox_values){
	float cm = 0;
	cm = 100*pow(prox_values,-0.6);
	return cm;
}

void switch_colour(void){
	if(get_colour()==RED){
		set_color(GREEN);
	}
	else if(get_colour()==GREEN){
		set_color(RED);
	}
}

int check_prox(proximity_msg_t data) {
	int count = 0;
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

uint8_t get_status(void){
	return status;
}

void set_status(uint8_t valeur){
	status = valeur;
}
