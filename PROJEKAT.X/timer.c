#include <p30fxxxx.h>
#include "timer.h"
#include "outcompare_h_1.h"
unsigned int r1,r2;
void Init_T1(void)
{
    TMR1 = 0;
    PR1 = TMR1_period;
    T1CONbits.TCS = 0; // 0 = Internal clock (FOSC/4)
    IFS0bits.T1IF = 0; // clear timer1 interrupt flag
    IEC0bits.T1IE = 1; // enable timer1 interrupt
    T1CONbits.TON = 0; // Timer1 off
}
void Init_T4(void)
{   TMR4 = 0;
    PR4 = TMR4_period;
    T4CONbits.TCS = 0; // 0 = Internal clock (FOSC/4)
    IFS1bits.T4IF = 0; // clear timer2 interrupt flag
    IEC1bits.T4IE = 1; // enable timer2 interrupt
    T4CONbits.TON = 0; // Timer2 off
}

void Init_T5(void)
{   TMR5 = 0;
    PR5 = TMR5_period;
    T5CONbits.TCS = 0; // 0 = Internal clock (FOSC/4)
    IFS1bits.T5IF = 0; // clear timer2 interrupt flag
    IEC1bits.T5IE = 1; // enable timer2 interrupt
    T5CONbits.TON = 0; // Timer2 off
}

void Init_T2(void)
{    //PR2 is the reference register for the timer counter register TMR2
        //TMR2 counts from 0 to PR2 and then resets, raising an interrupt flag
       
        PR2=499; //  20kHz, FOSC/20kHz-1
       
        //OC1R determines the DUTY CYCLE
        //OC1RS is a secondary register that we use to ascribe value to OC1R when TMR2=PR2
        //this is why we need an initial value for OC1R, in the beginning TMR=0 and PR2=some_value
        //so OC1R cannot get it's first value from OC1RS like it will after the end of every timer period
        OC1RS=20;//postavimo pwm
        OC1R=400;//inicijalni pwm pri paljenju samo
        OC1CON =OC_IDLE_CON & OC_TIMER2_SRC & OC_PWM_FAULT_PIN_DISABLE& T2_PS_1_256;//konfiguracija pwma
       
        OC2RS = 20;
        OC2R = 400; //OC1R/PR2*100
        OC2CON =OC_IDLE_CON & OC_TIMER2_SRC & OC_PWM_FAULT_PIN_DISABLE& T2_PS_1_256;
       
        T2CONbits.TON=1;//ukljucujemo timer koji koristi

        r1=250; //LIJEVA GUSJENICA 350
        r2=420; //DESNA GUSJENICA 368
        OC1RS=r1;//ovim postavljamo faktor ispune
        OC2RS=r2;
}
