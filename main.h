/**********************************************************************************
* @main.h
* @This file contains declaration of functions and variables used in main.c
* motor_status variable is global for GUI to track it.
*
* @author Vatsal Sheth & Puneet Bansal
 *********************************************************************************/


#ifndef MAIN_H_
#define MAIN_H_

#include "motor.h"

#define On 0x01
#define Off 0x00

uint8_t motor_status, motor_on_flag, motor_off_flag, rpm_prev;

void unused_port_init();
void pwm_port_init();
void hall_port_init();

#endif /* MAIN_H_ */
