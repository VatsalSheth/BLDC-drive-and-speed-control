/**********************************************************************************
* @motor.h
* @This file contains declaration of functions and variables used in main.c
* RPM_avg and current_sensing variables are global for GUI to track it.
*
* @author Vatsal Sheth & Puneet Bansal
**********************************************************************************/

#ifndef MOTOR_H_
#define MOTOR_H_

#include "msp.h"

#define H_U 0x01
#define L_U 0x04
#define H_V 0x08
#define L_V 0x20
#define H_W 0x40
#define L_W 0x80

#define Rising_edge 0x00
#define Falling_edge 0x07

#define PWM_Period 375  //8Khz
#define startup_duty_cycle 10      //starting duty cycle 10%

uint16_t count1,count2;
uint16_t arr1[100],arr2[100];


#define CW

typedef struct
{
    uint8_t interrupt_dir;
    uint8_t Mosfet_Value;
    uint8_t Mosfet_H_Value;
}Hall_commutaion;

volatile uint8_t pid_flag, rpm_count;
volatile uint8_t pwm_drive_pin,interrupt_bypass;
volatile uint16_t time_diff, current_time, RPM_avg, RPM[10], RPM_user, expected_time;
volatile int32_t error;
volatile float current_sensing, duty_cycle;

void motor_init();
void pwm_init();
void mosfet_drive(uint8_t mv);
void comp_init();
void motor_on();
void motor_off();
void calc_param();
void pid_calc();
void speed(uint16_t diff);
void time_diff_init();
void adc_init();
void update_desired_rpm();
void update_error();
#endif /* MOTOR_H_ */
