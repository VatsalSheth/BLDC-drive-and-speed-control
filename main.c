/**********************************************************************************
* @main.c
* @This file contains main, port initialization functions. It stops watchdog, updates
* the clock frequency 12Mhz, motor control and calls check for control logic
*
* @author Vatsal Sheth & Puneet Bansal
 *********************************************************************************/

#include "main.h"

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    CS->KEY = CS_KEY_VAL ;                  // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
    // Select ACLK = REFO, SMCLK = MCLK = DCO
    CS->CTL1 = CS_CTL1_SELA_2 | CS_CTL1_SELS_3 | CS_CTL1_SELM_3 | CS_CTL1_DIVS1;
    CS->KEY = 0;                            // Lock CS module from unintended accesses

    unused_port_init();

    RPM_user = 1000;    //Initial start speed
    motor_on_flag = 1;
    motor_off_flag = 0;
    rpm_prev = 0;
    motor_status = Off;

    pwm_port_init();
    hall_port_init();
    motor_init();

    __enable_irq();

    while(1)
    {
        if(motor_status == On && motor_on_flag==1)
        {
            motor_on_flag = 0;
            motor_off_flag = 1;
            motor_on();
        }
        else if(motor_status == Off && motor_off_flag==1)
        {
            motor_off_flag = 0;
            motor_on_flag = 1;
            motor_off();
        }

        if(RPM_user != rpm_prev)
        {
            update_desired_rpm();
            rpm_prev = RPM_user;
        }

        if(motor_status == On)
        {
            calc_param();
        }
    }
}

//unused port are set as input with internal pull up to consume power
void unused_port_init()
{
    P1->REN = 0xff;
    P2->REN = 0xff;
    P3->REN = BIT1 | BIT4;
    P4->REN = 0xff;
    P5->REN = BIT3 | BIT4 | BIT5 | BIT6 | BIT7;
    P6->REN = 0x6f;
    P7->REN = 0xff;
    P8->REN = 0xff;
    P9->REN = 0xff;
    P10->REN = 0xff;
}

//initialize GPIO for hall sensor inteface
void hall_port_init()
{
    P5->DIR &= ~(BIT0| BIT1 | BIT2);                //using pins 5.0,5.6,5.7 as input
    P5->REN |= (BIT0 | BIT1 | BIT2);                //Pull up/down enabled
    P5->OUT |= (BIT0 | BIT1 | BIT2);                //For pull up

    P5->IE|=BIT0 | BIT1 | BIT2;                     //enabling interrupt for hall sensor
    P5->IFG = 0X00;                                 //clearing interrupt flag initially
}

//initialize GPIO for driving MOSFET
void pwm_port_init()
{
    P3->OUT = 0x00;
    P3->DIR|= BIT0| BIT2| BIT3| BIT5| BIT6| BIT7;   //Setting these pins as outputs for PWM
}

