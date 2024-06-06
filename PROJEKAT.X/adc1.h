#ifndef ADC_H
#define ADC_H

#include<p30fxxxx.h>

//void ADCinit_analog(void);
   
   
void ADCinit(void);

void __attribute__((__interrupt__)) _ADCInterrupt(void);


#endif /* ADC_H */
