/**********************************************************************************
* @motor.c
* @This file contains functions for motor control, feedback and drive.
* It manages ADC, comparator, PWM, GPIO drives, sensor interface, control logic
*
* @author Vatsal Sheth & Puneet Bansal
**********************************************************************************/

#include "motor.h"

//Commutation Sequence lookup table
#ifdef CW
    Hall_commutaion Hall_Lookup[]={
                                   {Rising_edge,L_U | L_V | L_W, 0},
                                   {Rising_edge,(H_V | L_U | L_V), H_V},
                                   {Rising_edge,(H_U | L_U | L_W), H_U},
                                   {Falling_edge,(H_U | L_U | L_V), H_U},
                                   {Rising_edge,(H_W | L_V | L_W), H_W},
                                   {Falling_edge,(H_V | L_V | L_W), H_V},
                                   {Falling_edge,(H_W | L_U | L_W), H_W}
                                  };
#endif

//Comarator initializaton
void comp_init()
{
    uint8_t i;

    COMP_E1->CTL0 = COMP_E_CTL0_IPEN | COMP_E_CTL0_IPSEL_1;            // Enable V+, input channel CE1
    COMP_E1->CTL1 = COMP_E_CTL1_PWRMD_0;    // high speed power mode
    COMP_E1->CTL2 = COMP_E_CTL2_CEREFL_1 |   // VREF 1.2V
                    COMP_E_CTL2_RS_2 |      // Ladder enabled by COMP_E_CTL2_RS_2
                    COMP_E_CTL2_RSEL|       //VREF is applied to -terminal
                    COMP_E_CTL2_REF0_12;    // COMP_E_CTL2_REF0_12 (12/32)*1.2V = 0.45V
   COMP_E1->CTL3 = BIT1;                   // Input Buffer Disable @P1.1/CE1
   COMP_E1->CTL1 |= COMP_E_CTL1_ON;        // Turn On Comparator_E
   COMP_E1->INT |= COMP_E_INT_IE;
   NVIC->ISER[0] |= 1 << ((COMP_E1_IRQn) & 31);

   for (i = 0; i < 75; i++);               // delay for the reference to settle
}

//motor start sequence
void motor_on()
{
    uint8_t tmp;

    interrupt_bypass=0;
    pid_flag = 0;
    rpm_count = 0;
    duty_cycle = startup_duty_cycle;
    current_sensing=0;

    P6->DIR = BIT4;
    P6->OUT|= BIT4; //power switch on

    tmp=(P5->IN & 0x07);
    mosfet_drive(Hall_Lookup[tmp].Mosfet_Value);
    pwm_drive_pin = Hall_Lookup[tmp].Mosfet_H_Value;
    P5->IES = Hall_Lookup[tmp].interrupt_dir;
}

//motor off sequence
void motor_off()
{
    P6->OUT &= ~BIT4;
    P3->OUT = 0xa4; //drive all mosfet to cut off state
    current_sensing=0;
    RPM_avg = 0;
}

//motor initialization
void motor_init()
{
    pwm_init();
    comp_init();
    time_diff_init();
    adc_init();

    NVIC->ISER[1] |= 1 << ((PORT5_IRQn) & 31);
}

//pwm initialization
void pwm_init()
{
    TIMER_A0->CCR[0] = PWM_Period-1;
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_CCIE;
    TIMER_A0->CCR[1] =  (uint32_t)((PWM_Period*duty_cycle)/100);
    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__UP | TIMER_A_CTL_CLR | TIMER_A_CTL_IE;         //UP mode timer with smclk and overflow interrupt
    NVIC->ISER[0] |= 1 << ((TA0_N_IRQn)&31);
}

//refrence timer for speed and error measurement for motor control
void time_diff_init()
{
    time_diff = 0;
    current_time = 0;
    TIMER_A1->CCR[1] = 30000;   //10ms compare value
    TIMER_A1->CCTL[1] = TIMER_A_CCTLN_CCIE;
    TIMER_A1->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__CONTINUOUS | TIMER_A_CTL_CLR;         //Continuous mode timer with smclk
    NVIC->ISER[0] |= 1 << ((TA1_N_IRQn)&31);
}

//adc initialization in one shot mode
void adc_init()
{
    while(REF_A->CTL0 & REF_A_CTL0_GENBUSY);     // Wait till ref generator busy
    REF_A->CTL0 |= REF_A_CTL0_VSEL_0 | REF_A_CTL0_ON;     // 1.2V reference

    // Configure ADC - Pulse sample mode
    ADC14->CTL0 |= ADC14_CTL0_SHT0_2 | ADC14_CTL0_ON | ADC14_CTL0_SHP;    // SAMPCON from sampling timer
    ADC14->MCTL[0] = ADC14_MCTLN_VRSEL_1 | ADC14_MCTLN_INCH_10;           // ADC input ch 10 (PIN P4.3)
    ADC14->IER0 = 0x0001;           // ADC_IFG upon conv result-ADCMEM0 because single conversion and CSTARTADDR reset value is 0. So value will be in mem[0].

    while(!(REF_A->CTL0 & REF_A_CTL0_GENRDY));   // Wait for reference generator to settle
    ADC14->CTL0 |= ADC14_CTL0_ENC;

    NVIC->ISER[0] |= 1 << ((ADC14_IRQn) & 31);
}

//calculate new duty cycle based on error and calculate speed
void calc_param()
{
    if(pid_flag == 1)
    {
        pid_flag = 0;
        speed(time_diff);
        pid_calc();
        ADC14->CTL0 |= ADC14_CTL0_SC;   //trigger ADC sample and conversion
    }
}

//proportional control logic
void pid_calc()
{
    if(error<300)
    {
        duty_cycle -= (float)((float)error/700);
    }
    else if(error<500)
    {
        duty_cycle -= (float)((float)error/500);
    }
    else
    {
        duty_cycle -= (float)((float)error/300);
    }

    if(duty_cycle > 75) //Ensure duty cycle never exceeds 75%
    {
        duty_cycle = 75;
    }
    else if(duty_cycle<5)   //Ensure duty cycle never goes below 5%
    {
        duty_cycle = 5;
    }
    TIMER_A0->CCR[1] =  (uint32_t)(((float)(PWM_Period*duty_cycle))/100);   //update compare value to change duty cycle
}

//calculate average RPM of last 10 RPM values
void speed(uint16_t diff)
{
    uint8_t i=0;
    uint16_t tmp=0;

    tmp = (3 * 1000000 * 60)/(diff * 24);
    if(tmp<6000)
    {
        RPM[rpm_count] = tmp;
        rpm_count = (rpm_count+1)%10;
        for(i=0; i<10; i++)
        {
            tmp += RPM[i];
        }

        RPM_avg = (tmp/10);
    }
}

//Comparator isr for overload protection
void COMP_E1_IRQHandler()
{
    if(COMP_E1->IV & COMP_E_INT_IFG)
    {
        motor_off();
    }
}

//refrence timer isr to generate flags at 10ms to calculate control and speed values
void TA1_N_IRQHandler(void)
{
    TIMER_A1->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;
    TIMER_A1->CCR[1] += 30000;
    pid_flag = 1;
}

//pwm generation logic on GPIOs where required as per current hall state
void TA0_N_IRQHandler(void)
{
    if(TIMER_A0->CTL & TIMER_A_CTL_IFG)                 //set all R G B bits on overflow
    {
        TIMER_A0->CTL &= ~TIMER_A_CTL_IFG;
        P3->OUT |= pwm_drive_pin;
    }
    if(TIMER_A0->CCTL[1] & TIMER_A_CCTLN_CCIFG)         //clear R bit on compare 1
    {
        TIMER_A0->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;
        P3->OUT &= ~pwm_drive_pin;
    }
}

//hall sensor GPIO ISR. Alternate ISR are skipped for time difference calculation as a firmware fix for not accessible GPIO with glitch filter
void PORT5_IRQHandler(void)
{
    uint8_t hall;
    uint16_t tmp;

    if((interrupt_bypass %2 )==0)
    {
        tmp = TA1R;
    }

    //read the hall state and control drive
    if(P5->IFG & (BIT0 | BIT1 | BIT2))
    {
        P5->IFG = 0X00;
        hall = (P5->IN & 0x07);
        mosfet_drive(Hall_Lookup[hall].Mosfet_Value);
        pwm_drive_pin = Hall_Lookup[hall].Mosfet_H_Value;
        P5->IES = Hall_Lookup[hall].interrupt_dir;
    }

    //alternate time and error measurement
    if((interrupt_bypass %2 )==0)
    {
        if(tmp >= current_time)
        {
          time_diff = tmp-current_time;
        }
        else
        {
          time_diff = 65535 - (current_time - tmp);
        }
        update_error();
        current_time=tmp;
    }

    interrupt_bypass++;
}

//ADC conversion complete isr to calculate current value
void ADC14_IRQHandler(void)     //read adc value from memory and set the local flag
{
    uint16_t tmp;
    tmp= ADC14->MEM[0];
    current_sensing= ((tmp* 1.2) / (16384 * 0.6));
}

//drive required GPIO as per current hall state
void mosfet_drive(uint8_t mv)
{
    P3->OUT=mv;
}

//update values whenever set RPM is changed from GUI
void update_desired_rpm()
{
    expected_time = (3 * 1000000 * 60)/(RPM_user * 24); //expected time between hall state changes
    duty_cycle = (((float)(75*RPM_user))/5500); //jump duty cycle as per observation in control table for updated set speed
    TIMER_A0->CCR[1] =  (uint32_t)(((float)(PWM_Period*duty_cycle))/100);   //update compare value on duty cycle
}

//update the error
void update_error()
{
    error = expected_time - time_diff;
}
