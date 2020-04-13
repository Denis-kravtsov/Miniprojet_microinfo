/*
 * obstacle.h
 *
 *  Created on: 9 apr 2020
 *      Author: Marco
 */

#ifndef OBSTACLE_H_
#define OBSTACLE_H_
#include "sensors/proximity.h"
void obstacle_start(void);

void avoidance_logic(proximity_msg_t data);


#endif /* OBSTACLE_H_ */
