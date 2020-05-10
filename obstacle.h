/*
 * obstacle.h
 *
 *  Created on: 9 apr 2020
 *      Author: Marco
 */

#ifndef OBSTACLE_H_
#define OBSTACLE_H_

#include "sensors/proximity.h"
/*
*	starts the thread manipulating movement
*/
void obstacle_start(void);
/*
*	switches the colour to detect
*/
void switch_colour(void);
/*
* 	Input: prox_values.delta[i]
*	converts prox_values.delta values to cm (returns cm)
*/
float get_distance_cm_prox(int prox_values);
/*
*	Checks, if an object is detected in the corridor
*/
int check_prox(proximity_msg_t data);
/*
*	checks for the functioning mode (standby or normal)
*/
uint8_t get_status(void);
/*
*	Input: booleen value
*	sets the functioning mode
*/
void set_status(uint8_t valeur);


#endif /* OBSTACLE_H_ */
