#include <p30fxxxx.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "timer.h"
#include "adc1.h"
#include "outcompare_h_1.h"

#define SPEED_OF_SOUND (0.0343) // centimeters per microsecond
#define INSTRUCTION_CLOCK_PERIOD (0.1) // microseconds
#define SIZE_OF_WORD (6)

#define TRIG_BACK LATDbits.LATD8
#define ECHO_BACK PORTFbits.RF6

#define TRIG_FORWARD LATBbits.LATB4
#define ECHO_FORWARD PORTBbits.RB5

_FOSC(CSW_FSCM_OFF & XT_PLL4);
_FWDT(WDT_OFF);

unsigned int broj1,broj2;
static unsigned char time_overflow_back=0;
static unsigned char time_overflow_forward=0;
static int us_counter=0;
static float measured_distance_back = 0;
static float measured_distance_forward = 0;                  
unsigned char tempRX, tempRX2, tempRX2_bluetooth;
unsigned int broj1,broj2;
unsigned int  fotootpornik1,fotootpornik2,fotootpornik3,fotootpornik4,temp0,temp1,temp2,temp3;
static unsigned char word_start[SIZE_OF_WORD];
static unsigned char position_UART2=0;

void __attribute__ ((__interrupt__, no_auto_psv)) _T4Interrupt(void)
{
    TMR4= 0;
    ECHO_FORWARD = 0;
    time_overflow_forward = 1;
    IFS1bits.T4IF = 0;    
}
void __attribute__ ((__interrupt__, no_auto_psv)) _T5Interrupt(void)
{
    TMR5= 0;
    ECHO_BACK = 0;
    time_overflow_back = 1;
    IFS1bits.T5IF = 0;    
}
void __attribute__ ((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
    TMR1 = 0;
    us_counter++;
    IFS0bits.T1IF = 0;    
}
void __attribute__ ((__interrupt__, no_auto_psv)) _T2Interrupt(void)
{
    TMR2 = 0;
    IFS0bits.T2IF = 0;    
}

static void DelayUs (int vreme)
{
    us_counter = 0;
    T1CONbits.TON = 1;
    while(us_counter < vreme);
    T1CONbits.TON = 0;
}

static void MeasureBackDistance()
{
    // logical one lasts for 10us
    TRIG_BACK = 1;
    DelayUs(3); //  3 instead of 10 to make logical one lasts for 10us
    TRIG_BACK = 0;
    DelayUs(3);
    while(!ECHO_BACK); //  the value of the echo pin becomes 1 (the rising edge detected)
    TMR5 = 0; // reset T4
    IFS1bits.T5IF = 0;
    T5CONbits.TON = 1; // turn on T4, time measurement begins
    while(ECHO_BACK);        // the value of the echo pin becomes 0 (the falling edge detected)
    T5CONbits.TON = 0;  // turn off T4, time measurement stops
    unsigned int measured_time_back;
    if(time_overflow_back == 1)     // time overflow happens
    {
        measured_time_back = TMR5_period;
        time_overflow_back = 0;
    }
    else                            // the signal sent has returned
    {
        measured_time_back = TMR5;
    }
    TMR5 = 0;
    // operation /2 is used because the ultrasonic pulse travels to the obstacle and back
    // operation *INSTRUCTION_CLOCK_PERIOD is used to get the time in microseconds
    measured_distance_back= (measured_time_back*INSTRUCTION_CLOCK_PERIOD)/2*SPEED_OF_SOUND;        
   
}

static void MeasureForwardDistance()
{
    // logical one lasts for 10us
    TRIG_FORWARD= 1;
    DelayUs(5); //  3 instead of 10 to make logical one lasts for 10us
    TRIG_FORWARD = 0;
    DelayUs(5);
    while(!ECHO_FORWARD); //  the value of the echo pin becomes 1 (the rising edge detected)
    TMR4 = 0; // reset T4
    IFS1bits.T4IF = 0;
    T4CONbits.TON = 1; // turn on T4, time measurement begins
    while(ECHO_FORWARD);        // the value of the echo pin becomes 0 (the falling edge detected)
    T4CONbits.TON = 0;  // turn off T4, time measurement stops
    unsigned int measured_time_forward;
    if(time_overflow_forward == 1)     // time overflow happens
    {
        measured_time_forward = TMR4_period;
        time_overflow_forward = 0;
    }
    else                            // the signal sent has returned
    {
        measured_time_forward = TMR4;
    }
    TMR4 = 0;
    // operation /2 is used because the ultrasonic pulse travels to the obstacle and back
    // operation *INSTRUCTION_CLOCK_PERIOD is used to get the time in microseconds
    measured_distance_forward = (measured_time_forward*INSTRUCTION_CLOCK_PERIOD)/2*SPEED_OF_SOUND;        
   
}

void initUART1(void)
{
    //OVO JE KOPIRANO IZ Touch screen.X
    U1BRG=0x0040; //ovim odredjujemo baudrate
    U1MODEbits.ALTIO=1; //biramo koje pinove koristimo za komunikaciju osnovne ili alternativne, koristimo alternativne

    IEC0bits.U1RXIE = 1;
    U1STA&=0xfffc;
    U1MODEbits.UARTEN=1;
    U1STAbits.UTXEN=1;
}

void initUART2(void)
{
    U2BRG=0x0040;//ovim odredjujemo baudrate

    IEC1bits.U2RXIE=1;//omogucavamo rx2 interupt

    U2STA&=0xfffc;

    U2MODEbits.UARTEN=1;//ukljucujemo ovaj modul

    U2STAbits.UTXEN=1;//ukljucujemo predaju
}

void __attribute__((__interrupt__)) _U1RXInterrupt(void)
{
    IFS0bits.U1RXIF = 0;
    tempRX=U1RXREG;

}
unsigned int ReadUART2(void) {
    return tempRX2;
}

void WriteUART2(unsigned int data)
{
 while(!U2STAbits.TRMT);

    if(U2MODEbits.PDSEL == 3)
        U2TXREG = data;
    else
        U2TXREG = data & 0xFF;
}

void WriteUART2dec2string(unsigned int data)
{
unsigned char temp;
    temp = data/10000;
    WriteUART2(temp+'0');
data=data-temp*10000;
temp=data/1000;
WriteUART2(temp+'0');
data=data-temp*1000;
temp=data/100;
WriteUART2(temp+'0');
data=data-temp*100;
temp=data/10;
WriteUART2(temp+'0');
data=data-temp*10;
WriteUART2(data+'0');
}

void WriteString(unsigned char* word){
    unsigned int i=0, length=0;
   
    while(word[length] != '\0') length++;
   
    for(i=0; i<length; ++i) WriteUART2(word[i]);
   
}
void WriteStringUART2(register const char *str)
{
    while((*str)!=0)
    {
        WriteCharUART2(*str++);
    }
}
void WriteCharUART2(unsigned int data)
{
    while(!U2STAbits.TRMT);         /// Waiting for the transmit register to be available
    if(U2MODEbits.PDSEL == 3)       /// 9-bit data without parity bit
        U2TXREG = data;
    else
        U2TXREG = data & 0xFF;
}



void WriteUART1(unsigned int data)
{
 while(!U1STAbits.TRMT);

    if(U1MODEbits.PDSEL == 3)
        U1TXREG = data;
    else
        U1TXREG = data & 0xFF;
}
void WriteStringUART1(register const char *str)
{
    while((*str)!=0)
    {
        WriteUART1(*str++);
    }

}

void WriteUART1dec2string(unsigned int data)
{
    unsigned char temp;
    temp=data/1000;
    WriteUART1(temp+'0');  
    data=data-temp*1000;
    temp=data/100;
    WriteUART1(temp+'0');
    data=data-temp*100;
    temp=data/10;
    WriteUART1(temp+'0');
    data=data-temp*10;
    WriteUART1(data+'0');
}


void WriteObstacleDistance2(float data)
{
    unsigned char temp;
    data = data*10;
    temp=data/10000;
    WriteUART1(temp+'0');
    // WriteUART1(13);
    data=data-temp*10000;
    temp=data/1000;
    WriteUART1(temp+'0');
    data=data-temp*1000;
    temp=data/100;
    WriteUART1(temp+'0');
    WriteUART1(13);
  data=data-temp*100;
    data = data*10;
    temp=data/10;
    WriteUART1(temp+'0');
   // WriteUART1('.');
    data=data-temp*10;
    WriteUART1(data+'0');
}


void __attribute__((__interrupt__)) _ADCInterrupt(void)
{
    fotootpornik1=ADCBUF0;
    fotootpornik2=ADCBUF1;
    fotootpornik3=ADCBUF2;
    fotootpornik4=ADCBUF3;
   
    IFS0bits.ADIF = 0;

}

void __attribute__ ((__interrupt__, no_auto_psv)) _U2RXInterrupt(void)
{
    IFS1bits.U2RXIF = 0;
    tempRX2_bluetooth = U2RXREG;
   
    if(tempRX2_bluetooth != 0)
    {
        word_start[position_UART2] = tempRX2_bluetooth;
        tempRX2_bluetooth = 0;

        if(position_UART2 < SIZE_OF_WORD - 1)
        {
           position_UART2++;
           word_start[position_UART2] = 0;
        }
        else position_UART2 = 0;
    }
}


void pinInit()
{      //FOTOOTPORNICI PINOVI
       ADPCFGbits.PCFG0 = 0;
       TRISBbits.TRISB0 = 1;
       ADPCFGbits.PCFG1 = 0;
       TRISBbits.TRISB1 = 1;
       ADPCFGbits.PCFG2 = 0;
       TRISBbits.TRISB2 = 1;
       ADPCFGbits.PCFG3 = 0;
       TRISBbits.TRISB3 = 1;
       
     //  PROGRAMATOR PINOVI
        ADPCFGbits.PCFG6 = 0;
       TRISBbits.TRISB6 = 1;
       ADPCFGbits.PCFG7 = 0;
       TRISBbits.TRISB7 = 1;
       
     
       //Ulazi za motore
         TRISFbits.TRISF0 = 0;
         TRISFbits.TRISF1 = 0;
         TRISFbits.TRISF2 = 0;
         TRISFbits.TRISF3 = 0;
         
         //EN za drajver
         TRISDbits.TRISD0 = 0;
         TRISDbits.TRISD1 = 0;
         
         //Pinovi?? RB9,RB10,RB11,RB12,RD2.RD3
         
         
         //INTERAPT?? RA11 i RD9
         TRISDbits.TRISD9 = 1; // ulaz za interupt
         TRISAbits.TRISA11 = 1; // ulaz za interupt
         
         //ultrazvucni1
             ADPCFGbits.PCFG4= 1;
             TRISBbits.TRISB4 = 0;
             
             ADPCFGbits.PCFG5 = 1;
             TRISBbits.TRISB5 = 1;
             
         //ultrazvucni2
             TRISDbits.TRISD8 = 0;
             TRISFbits.TRISF6 = 1;
             
         //BT:RF4 je TX,RF5 je Rx(UART1)
         //UART2-RC13 i RC14
             
       
 
       
 }


void desno() {

    //turn off both enable signals for 10us
    //so there is no chance of a short circuit
    //before restoring both tank treads to equal speed
   // stop();
    // motor 2 forward
    LATFbits.LATF0 = 0; // motor 2 in4
    LATFbits.LATF1 = 1; // motor 2 in3    

    // motor 1 backwards
    LATFbits.LATF2 = 1; // motor 1 in1
    LATFbits.LATF3 = 0; // motor 1 in2
   
    
}

void levo() {

    //turn off both enable signals for 10us
    //so there is no chance of a short circuit
    //before restoring both tank treads to equal speed
    //stop();
 

    // motor 2 backwards
    LATFbits.LATF0 = 1; // motor 2 in4
    LATFbits.LATF1 = 0; // motor 2 in3

    // motor 1 forward
    LATFbits.LATF2 = 0; // motor 1 in1
    LATFbits.LATF3 = 1; // motor 1 in2
   
}


void napred (){
   
    //DelayUs(100);
 
     
    // motor 1 forward
    LATFbits.LATF0 = 0; // motor 1 in1
    LATFbits.LATF1 = 1; // motor 1 in2
   
    // motor 2 forward
    LATFbits.LATF2 = 0; // motor 2 in4
    LATFbits.LATF3 = 1; // motor 2 in3
    
}

void nazad() {
    // DelayUs(100);
   
    // motor 2 backwards
    LATFbits.LATF0 = 1; // motor 2 in4
    LATFbits.LATF1 = 0; // motor 2 in3

    // motor 1 backwards
    LATFbits.LATF2 = 1; // motor 1 in1
    LATFbits.LATF3 = 0; // motor 1 in2
}



void stop() {
    // DelayUs(100);
   
    // motor 2 backwards
    LATFbits.LATF0 = 0; // motor 2 in4
    LATFbits.LATF1 = 0; // motor 2 in3

    // motor 1 backwards
    LATFbits.LATF2 = 0; // motor 1 in1
    LATFbits.LATF3 = 0; // motor 1 in2
}

int main(void)
{
       
   
   
    pinInit();
    initUART1();
    initUART2();
    ADCinit();  
    Init_T1();
    Init_T2();
    Init_T4();
    Init_T5();
    
   
   memset(word_start, 0, sizeof(word_start));  
    position_UART2 = 0;
    WriteCharUART2(10);
    WriteStringUART2("Ispisi START.");
    WriteCharUART2(10);
    while (word_start[0]!='S' 
        || word_start[1]!='T'
        || word_start[2]!='A'
        || word_start[3]!='R'
        || word_start[4]!='T'
        || word_start[5]!='\0'
    );
    WriteStringUART2("Tenk je upaljen.");
    WriteCharUART2(10);
  
    while(1)
        
    {
        
  /*  MeasureBackDistance();
     if (measured_distance_back > 17) {
    napred();
     }
     else 
         nazad();
   */ 
  if (fotootpornik1 < 1000 && fotootpornik1 < fotootpornik2 && fotootpornik1 < fotootpornik3 && fotootpornik1 < fotootpornik4)

      
    {
      desno();
      for(broj1=0;broj1<2300;broj1++)
            for(broj2=0;broj2<900;broj2++);
   MeasureBackDistance(); // Izmerite udaljenost unapred
   
    if (measured_distance_back > 17)
   {
       nazad(); // Ako je udaljenost unapred ve?a od 17, krenite napred
        while (measured_distance_back >= 17)
        {
            MeasureBackDistance(); // Ponovo izmerite udaljenost unapred
            if (measured_distance_back < 17)
            {
                stop(); // Zaustavite tenk ako je udaljenost unapred manja od 17
            }
        }
    }
        else
    {
        stop(); // Zaustavite tenk ako je udaljenost unaprijed ve? manja od 17
    }
}  
    
    //ovo je fotootpornik kod LM-rad
  
  else if (fotootpornik2 < 1000 && fotootpornik2 < fotootpornik1 && fotootpornik2 < fotootpornik3 && fotootpornik2 < fotootpornik4)
{
    nazad(); // Okrenite tenk udesno ako je fotootpornik 3 najmanji od svih i manji od 1000
    // Provjerite da li je udaljenost unaprijed ve?a ili jednaka 17 cm prije nego ?to krenete naprijed
    MeasureBackDistance(); 
    if (measured_distance_back >= 17)
    {
        nazad(); // Krenite naprijed ako je fotootpornik 2 najmanji od svih i manji od 1000
        while (measured_distance_back >= 17)
        {
            MeasureBackDistance(); // Ponovo izmjerite udaljenost unaprijed
            if (measured_distance_back < 17)
            {
                stop(); // Zaustavite tenk ako je udaljenost unaprijed manja od 17
            }
        }
    }
    else
    {
        stop(); // Zaustavite tenk ako je udaljenost unaprijed ve? manja od 17
    }
}

     
     else if (fotootpornik3 < 1000 && fotootpornik3 < fotootpornik2 && fotootpornik3 < fotootpornik1 && fotootpornik3 < fotootpornik4)
{
    levo(); // Okrenite tenk udesno ako je fotootpornik 3 najmanji od svih i manji od 1000
    for(broj1=0;broj1<2200;broj1++)
            for(broj2=0;broj2<800;broj2++);
    MeasureBackDistance(); // Izmerite udaljenost unapred
    if (measured_distance_back > 17)
    {
        nazad(); // Ako je udaljenost unapred ve?a od 17, krenite napred
        while (measured_distance_back >= 17)
        {
            MeasureBackDistance(); // Ponovo izmerite udaljenost unapred
            if (measured_distance_back < 17)
            {
                stop(); // Zaustavite tenk ako je udaljenost unapred manja od 17
            }
        }
    }
    else {
        stop();
    }
}
  else if (fotootpornik4 < 1000 && fotootpornik4 < fotootpornik1 && fotootpornik4 < fotootpornik2 && fotootpornik4 < fotootpornik3)
{
    napred(); // Okrenite tenk udesno ako je fotootpornik 3 najmanji od svih i manji od 1000
    MeasureForwardDistance(); // Izmerite udaljenost unapred
    if (measured_distance_forward > 17)
    {
        napred(); // Ako je udaljenost unapred ve?a od 17, krenite napred
        while (measured_distance_forward>= 17)
        {
            MeasureForwardDistance(); // Ponovo izmerite udaljenost unapred
            if (measured_distance_forward < 17)
            {
                stop(); // Zaustavite tenk ako je udaljenost unapred manja od 17
            }
        }
    }
    else {
        stop();
    }
}

    else
{
    stop(); // Zaustavite tenk ako nijedan fotootpornik nije osvijetljen
}
    
  
      if (word_start[0] == 'S' && 
        word_start[1] == 'T' &&
        word_start[2] == 'O' &&
        word_start[3] == 'P' &&
        word_start[4] == '\0')
    {
        memset(word_start, 0, sizeof(word_start)); // Resetujte word_start
        WriteStringUART2("Tenk je ugasen."); // Ispi?ite da je tenk uga?en
        WriteCharUART2(10);

        break; // Izlaz iz while petlje
    }
    
  
    }

    return 0;
}
