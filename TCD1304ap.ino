#include "tcd1304ap.h"


void setup()
{
  Serial.begin(115200);
  CPU_PRESCALE(CPU_16MHz);
  pinMode(PhiM,OUTPUT);
  pinMode(ICG,OUTPUT);
  pinMode(SH,OUTPUT);
  pinMode(LED,OUTPUT);

/*
min data frequency: 0.067Mhz
max data frequency with no errors:
- 0.083 MHz with ADC prescaler=2 (160 instructions avalable for ISR, 21.1 Hz measurement rate)
- 0.074 MHz with ADC prescaler=4 (158 instructions avalable for ISR, 18.8 Hz measurement rate)
- no seetings with ADC prescaler>=8
- 0.071 MHz with ADC prescaler=16 (10 instruction acailable for ISR) ???
max data frequency with no pixels shift:
- 0.133 MHz with any ADC prescaler (119 instructions per pixel, 32.6 Hz measurement rate)
*/
  tcd1304ap::init(0.071,16); 

  unsigned int OCR2A_val=OCR2A;
  Serial.print("OCR2A=");
  Serial.print(OCR2A_val);
  Serial.print("\n");
  unsigned int OCR1A_val=OCR1A;
  Serial.print("OCR1A=");
  Serial.print(OCR1A_val);
  Serial.print("\n");
  unsigned int OCR1B_val=OCR1B;
  Serial.print("OCR1B=");
  Serial.print(OCR1B_val);
  Serial.print("\n");
  unsigned int ICR1_val=ICR1;
  Serial.print("ICR1=");
  Serial.print(ICR1_val);
  Serial.print("\n");
/*  unsigned int OCR0A_val=OCR0A;
  Serial.print("OCR0A=");
  Serial.print(OCR0A_val);
  Serial.print("\n")*/;
  Serial.print("We have ");
  Serial.print(ICR1_val-OCR1A_val+OCR1B_val-10); //from timer compare match A to timer1 compare match B, 10 CPU cycles to enter and leave interrupt routine
  Serial.print(" CPU cycles to execute the interupt reading the ADC.\n");
  Serial.print("data rate=");
  Serial.print(16./(ICR1_val-1),3); //from timer compare match A to timer1 compare match B, 10 CPU cycles to enter and leave interrupt routine
  Serial.print(" MHz.\n");
  
/*  GTCCR=(1<<TSM)|(1<<PSRASY)|(1<<PSRSYNC);//timer synchronization mode
  //reset of timers values
  TCNT0=0;
  TCNT1=0;
  TCNT2=0;
 //clear timer1 compare match A interupt flag (OCF1A=1) and timer1 compare match B interupt flag (OCF1B=1)
  TIFR1= (1<<OCF1A)|( 1<<OCF1B); 
  //enable timer1 compare match A interupt (OCE1A=1)
  TIMSK1= (1<<OCIE1A);
 //clear timer0 compare match A interupt flag (OCF0A=1)
  TIFR0= (1<<OCF0A); 
  //enable timer0 compare match A interupt (OCIE0A=1)
  TIMSK0= 1<<OCIE0A;//0;   
  GTCCR=0;//exit timer synchronization mde
  

  delay_us(22); 
  
  GTCCR=(1<<TSM)|(1<<PSRASY)|(1<<PSRSYNC);//timer synchronization mode
  unsigned int timer0_val=TCNT0;
  unsigned int timer1_val=TCNT1;
  unsigned int timer2_val=TCNT2;
  GTCCR=0;//exit timer synchronization mode
  Serial.print("TCNT0=");
  Serial.print(timer0_val);
  Serial.print("\n");
  Serial.print("TCNT1=");
  Serial.print(timer1_val);
  Serial.print(", ICR1=");
  Serial.print(ICR1);
  Serial.print("\n");
  Serial.print("TCNT2=");
  Serial.print(timer2_val);
  Serial.print(", OCR2A=");
  Serial.print(OCR2A);
  Serial.print("\n");*/
}

void loop()
{
//  Serial.print("loop iteration.\n");
  
  tcd1304ap::readout_and_capture(6000);
  tcd1304ap::send_last_measurement_hex();
//  tcd1304ap::send_last_measurement(50); //all, every 50 pixels
//  tcd1304ap::send_last_measurement(1,1960,2055); // edge of shadow
//  tcd1304ap::send_last_measurement(1,3548,3647); //last 100 pixels
  //  dprint("loop end.\n");
}


