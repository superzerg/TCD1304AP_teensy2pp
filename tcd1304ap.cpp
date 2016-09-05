#include "tcd1304ap.h"

// declare static members of tcd1304ap
volatile uint8_t tcd1304ap::current_data[SIGNAL_NELEMENT];
volatile uint8_t  tcd1304ap::current_light_shield[LIGHTSHIELD_NELEMENT];
volatile int8_t tcd1304ap::current_state; 
volatile unsigned int tcd1304ap::current_nread; //number of elements read
volatile unsigned int tcd1304ap::current_nread_adc; //number of adc read
volatile unsigned int tcd1304ap::current_nread_error; //number of failed adc read
float tcd1304ap::current_freq_data_MHz; //data rate of the sensor (in MHz, between 0.2 and 1)
unsigned int tcd1304ap::current_time_light_us; //time during which the led is switched on (in us, must be >1.0)
uint8_t tcd1304ap::current_prescaler_adc;
  
ISR (TIMER1_COMPA_vect) //read the result of the last ADC conversion
{
  tcd1304ap::current_nread++;
  if( (ADCSRA&16)==LOW ) //ADC conversion not finished 1<<ADIF=16
  {
    tcd1304ap::current_nread_error++;
    if(tcd1304ap::current_nread>=FULL_LINE_NELEMENT)
    {
      tcd1304ap::current_state=STATE_ALL_DATA_MEASURED;
      TIMSK1=0; //stop timer1 interupts
      TIFR1=TIFR1;
    }
    if(TIFR0&2) //timer0 compare match A
    {
      LED_OFF();
      TIMSK0=0; //stop timer0 interupts
      TIFR0=TIFR0; //clear timer0 interupts flags
    }
//    dshow("read error.\n");
    return;
  }
  ADCSRA |= 16; //reset ADC interlock flag  1<<ADIF=16
//  dshow("read ok.\n");
  TIFR1=4; //(1<<OCF1B) : reset compare match B flag
  if(TIFR0&2) //timer0 compare match A
  {
    LED_OFF();
    TIMSK0=0; //stop timer0 interupts
    TIFR0=TIFR0; //clear timer0 interupts flags
  }
  if (tcd1304ap::current_nread>=32) 
  {
    tcd1304ap::current_nread_adc=tcd1304ap::current_nread-32;
    if (tcd1304ap::current_nread_adc<SIGNAL_NELEMENT)
    {
      tcd1304ap::current_data[tcd1304ap::current_nread_adc]=ADCH;
    }
    else
    {
      if (tcd1304ap::current_nread_adc>=FULL_LINE_NELEMENT)
      {
        tcd1304ap::current_state=STATE_ALL_DATA_MEASURED;
        TIMSK1=0; //stop timer1 interupts
        TIFR1=TIFR1;
      }
    }
    return;
  }
  if ( (tcd1304ap::current_nread>=16) && ( tcd1304ap::current_nread<=28 ))
  {
    tcd1304ap::current_light_shield[tcd1304ap::current_nread-16]=ADCH;
  }
}

ISR (TIMER0_COMPA_vect) //read the result of the last ADC conversion
{
  LED_OFF();
  TIMSK0=0; //disable timer0 interupts
}

/*ISR (TIMER1_COMPB_vect) //start an ADC conversion
{
  ;
}*/

void tcd1304ap::init(float freq_data_MHz, uint8_t prescaler_adc)
{
  Serial.print("running init.\n");
  Serial.flush();

  EIMSK=0; //disable external Interupts
  TIMSK0=0; //disable timer0 Interupts
  TIMSK1=0; //disable timer1 Interupts
  TIMSK2=0; //disable timer2 Interupts
  TIMSK3=0; //disable timer3 Interupts
  UCSR1B=0; //disable USART Interupts
  WDTCSR=0; //disable Watchdog Timer Interupts
  PCICR=0; //disable pin change Interupts
  SPCR=0; //disable SPI Interupts
  ACSR=(1<<ACD); //disable Analog Comparator Interupts
  TWCR=0; //disable 2-wire Interupts
  
  dshow("setup adc.\n");
  tcd1304ap::setup_adc(prescaler_adc);
  dshow("freq_data_MHz=");
  dshow(freq_data_MHz);
  dshow("\n");
  tcd1304ap::current_freq_data_MHz=freq_data_MHz;
  dshow("setup timer2.\n");
  tcd1304ap::setup_timer2_clock(freq_data_MHz);
  dshow("setup timer1.\n");
  tcd1304ap::setup_timer1_clock(freq_data_MHz);
  dshow("clean_current.\n");
  tcd1304ap::clean_current();
  dshow("cleaning done.\n");

  tcd1304ap::current_state=STATE_READY;
  tcd1304ap::current_nread=0;
  tcd1304ap::current_nread_adc=0;
  tcd1304ap::current_nread_error=0;

  dshow("init done.\n");

}

void tcd1304ap::clean_current(void)
{
  dshow("cleaning data.\n");

  for (unsigned int elem=0;elem<SIGNAL_NELEMENT;elem++)
  {
/*    dshow("elem=");
    dshow(elem);
    dshow("\n");*/
    tcd1304ap::current_data[elem]=0;
  }  
  dshow("cleaning light shield.\n");
  for (unsigned int elem=0;elem<LIGHTSHIELD_NELEMENT;elem++)
  {
    tcd1304ap::current_light_shield[elem]=0;
  }
}

void tcd1304ap::capture(unsigned long time_light_us)
{
  dshow("capture().\n");
  if (tcd1304ap::current_freq_data_MHz==0)
  {
    Serial.print("WARNING in tcd1304ap::capture(): capture called before init.\n");
    Serial.flush();
    tcd1304ap::init();
  }

  if (tcd1304ap::current_time_light_us!=time_light_us)
  {
    Serial.print("WARNING in tcd1304ap::capture(): changing exposition to ");
    Serial.print(time_light_us);
    Serial.print(".\n");
    Serial.flush();
    tcd1304ap::setup_timer0_clock(time_light_us);
//    tcd1304ap::current_state=STATE_READY;
    tcd1304ap::current_state=STATE_CAPTURE_DONE;
  }
  if(digitalRead(ICG)==HIGH)
  {
    digitalWrite(LED,LOW); //switch off LED
    digitalWrite(ICG,HIGH);
    digitalWrite(SH,LOW);
    digitalWrite(ICG,LOW);
    digitalWrite(SH,HIGH);
    delay_us(2);
    tcd1304ap::current_state=STATE_CAPTURING;
    digitalWrite(SH,LOW); //t1 starts here, linear CCD starts to integrate light
  }

  GTCCR=(1<<TSM)|(1<<PSRASY)|(1<<PSRSYNC);//timer synchronization mode
  //reset of timers values
  TCNT0=0;
  TCNT1=OCR2A-2;
  TCNT2=OCR2A-2;
  if (digitalRead(PhiM)==LOW)
    TCCR2B|=(1<<FOC2A); //force output compare A to set PhiM HIGH  
  delay_us(1); //t1 ends here
  tcd1304ap::current_nread=0;
  tcd1304ap::current_nread_adc=0;  
  tcd1304ap::current_nread_error=0;
 //clear timer1 compare match A interupt flag (OCF1A=1) and timer1 compare match B interupt flag (OCF1B=1)
  TIFR1= (1<<OCF1A)|( 1<<OCF1B); 
  //enable timer1 compare match A interupt (OCE1A=1)
  TIMSK1= (1<<OCIE1A);
 //clear timer0 compare match A interupt flag (OCF0A=1)
  TIFR0= (1<<OCF0A); 
  //enable timer0 compare match A interupt (OCIE0A=1)
  TIMSK0= 0;//1<<OCIE0A;
  tcd1304ap::current_state=STATE_CAPTURING;
  LED_ON(); //led switch off by timer0
  ICG_HIGH(); //t4 starts here 
  GTCCR=0;

  while (tcd1304ap::current_state!=STATE_ALL_DATA_MEASURED)
  {
    dprint(tcd1304ap::current_state);
    dshow("Waiting for all pixels to be read.");
    dprint(tcd1304ap::current_state);
    dprint(tcd1304ap::current_nread_adc);
    dprint(tcd1304ap::current_nread_error);
  }
  while(LED_STATUS()==HIGH)
  {
    dshow("Waiting for LED to switch off.");
    dprint(TCNT0);
//    delay_us(10);
  }
  
    
  GTCCR=(1<<TSM)|(1<<PSRASY)|(1<<PSRSYNC);//timer synchronization mode
  TCNT2=OCR2A-2;
  digitalWrite(PhiM,HIGH);    
  digitalWrite(ICG,LOW);
  digitalWrite(SH,HIGH); // t2 ends, t3 start
  GTCCR=0;//exit timer synchronization mde
  delay_us(2);
  digitalWrite(SH,LOW); //t3 stops, t1 starts, linear CCD start a new measurement
  tcd1304ap::current_state=STATE_CAPTURE_DONE;
  tcd1304ap::current_time_light_us=time_light_us;
  delay_us(5);
  dshow("capture() done.\n");
}

void tcd1304ap::readout_and_capture(unsigned long time_light_us)
{
  dshow("readout_and_capture().\n");
  dprint(time_light_us);
  dshow("setup timer0.\n");
  if (tcd1304ap::current_freq_data_MHz==0)
  {
    Serial.print("WARNING in tcd1304ap::readout_and_capture(): readout_and_capture called before init.\n");
    Serial.flush();
    tcd1304ap::init();
  }
  if(tcd1304ap::current_state!=STATE_READY)
  {
    tcd1304ap::clean_current();
  }
  if (tcd1304ap::current_time_light_us!=time_light_us)
  {
    Serial.print("WARNING in tcd1304ap::readout_and_capture(): exposition was changed, capturing ...\n");
    Serial.flush();
    tcd1304ap::capture(time_light_us);
  }
  if (tcd1304ap::current_state!=STATE_CAPTURE_DONE)
  {
    Serial.print("WARNING in tcd1304ap::readout_and_capture(): current_state=");
    Serial.print(tcd1304ap::current_state);
    Serial.print(" is not STATE_CAPTURE_DONE. capturing ...\n");
    Serial.flush();
    tcd1304ap::capture(time_light_us);
    if(tcd1304ap::current_state!=STATE_CAPTURE_DONE)
    {
      Serial.print("ERROR in tcd1304ap::readout_and_capture(): current_state=");
      Serial.print(tcd1304ap::current_state);
      Serial.print(" is not STATE_CAPTURE_DONE. exit ...\n");
      Serial.flush();
      return;
    }
  }
  
  GTCCR=(1<<TSM)|(1<<PSRASY)|(1<<PSRSYNC);//timer synchronization mode
  //reset of timers values
  TCNT0=0;
  TCNT1=OCR2A-2;
  TCNT2=OCR2A-2;
  if (digitalRead(PhiM)==LOW)
    TCCR2B|=(1<<FOC2A); //force output compare A to set PhiM HIGH  
  delay_us(1); //t1 ends here
  tcd1304ap::current_nread=0;
  tcd1304ap::current_nread_adc=0;
  tcd1304ap::current_nread_error=0;
  if(digitalRead(ICG)==LOW)
  {
    tcd1304ap::current_state=STATE_CAPTURING;
  }else
  {
    Serial.print("ERROR in tcd1304ap::readout_and_capture(): ICG should be LOW at that point.\n");
    Serial.flush();
    tcd1304ap::current_state=STATE_ERROR;
    return;
  } 
 //clear timer1 compare match A interupt flag (OCF1A=1) and timer1 compare match B interupt flag (OCF1B=1)
  TIFR1= (1<<OCF1A)|( 1<<OCF1B); 
  //enable timer1 compare match A interupt (OCE1A=1)
  TIMSK1= (1<<OCIE1A);
 //clear timer0 compare match A interupt flag (OCF0A=1)
  TIFR0= (1<<OCF0A); 
  //enable timer0 compare match A interupt (OCIE0A=1)
  TIMSK0=0; //(1<<OCIE0A);
  tcd1304ap::current_state=STATE_READOUT;  
  LED_ON(); //LED switch off by timer0
  ICG_HIGH(); //t1 ends here, t4 starts
  GTCCR=0;//exit timer synchronization mde
  
  while (tcd1304ap::current_state!=STATE_ALL_DATA_MEASURED)
  {
    dprint(tcd1304ap::current_state);
    dshow("Waiting for all pixels to be read.");
    dprint(tcd1304ap::current_state);
    dprint(tcd1304ap::current_nread_adc);
    dprint(tcd1304ap::current_nread_error);
  }
  dshow("all data has been measured\n");
  dshow("current_nread_error=");
  dshow(tcd1304ap::current_nread_error);
  dshow("\n");
  while(LED_STATUS()==HIGH)
  {
    dshow("Waiting for LED to switch off.");
  }

  GTCCR=(1<<TSM)|(1<<PSRASY)|(1<<PSRSYNC);//timer synchronization mode
  TCNT2=OCR2A-2;
  digitalWrite(PhiM,HIGH);    
  digitalWrite(ICG,LOW);
  digitalWrite(SH,HIGH); // t2 ends, t3 start
  GTCCR=0;//exit timer synchronization mde
  
  delay_us(2);
  digitalWrite(SH,LOW); //t3 stops, t1 starts, linear CCD stops to measure
  
  tcd1304ap::current_state=STATE_CAPTURE_DONE;  

//delay_us(5);
  dshow("exit tcd1304ap::readout_and_capture() with current_state=");
  dshow(tcd1304ap::current_state);
  dshow(".\n");
}

void tcd1304ap::setup_timer2_clock(float freq_data_MHz)
{
  //disable timer2 compare match A interupt (OCE1A=1) and timer1 compare match B interupt (OCE1B=1)
  TIMSK2= 0;
 //clear timer2 interupt flags
  TIFR2= TIFR2; 
  
  //setup OC2A output (PhiM)
  //COM2A1:0=1 toggle OC1A on compare match with OCR2A
  //WGM22:0=2 Clear Timer on Compare match (CTC) mode with OCR2A as TOP value
  //CS22:0=1 prescaler of 1
  TCCR2A=(1<<COM2A0) | (2<<WGM20);
  TCCR2B= (1<<CS20);
  //setup frequency : freq_sensor_clock=F_CPU/(2*prescaler*(1+ocr2aval))
  OCR2A=((unsigned int) ( ((float) F_CPU)/(4*freq_data_MHz*2e6)-1));
  if(OCR2A<4) 
  {
    Serial.print("ERROR in tcd1304ap::setup_timer2_clock(): freq_data_MHz=");
    Serial.print(freq_data_MHz);
    Serial.print(" is too high\n");
    Serial.flush();
    tcd1304ap::current_state=STATE_ERROR;
    return;    
  }  
}

void tcd1304ap::setup_timer1_clock(float freq_data_MHz)
{
  //disable timer1 compare match A interupt (OCE1A=1) and timer1 compare match B interupt (OCE1B=1)
  TIMSK1= 0;
 //clear timer1 interupt flags
  TIFR1= TIFR1; 

  //setup OC2A output (PhiM)
  //COM1A1:0=0 OC1A disconected
  //COM1B1:0=0 OC1B disconected
  //COM1C1:0=0 OC1C disconected
  //WGM11:0=2 & WGM13:2=3 Fast PWM mode with ICR1 as TOP value
  //CS12:0=1 prescaler of 1
  TCCR1A=(2<<WGM10);
  TCCR1B=(3<<WGM12)|(1<<CS10);
  //setup frequency : freq_timer1_clock=F_CPU/(prescaler*(1+icr1val))
  //set to reach TOP=ICR1 at the end of each pixel value sent by the sensor
  ICR1=(OCR2A+1)*8-1; // timer goes to 0 after 4 sensor_clock periods
  if(OCR2A<4) //as OCR1A+10 must be smaller than ICR1
  {
    Serial.print("ERROR in tcd1304ap::setup_timer1_clock(): TOP value of timer1=");
    Serial.print(ICR1);
    Serial.print(" is too small\n");
    Serial.flush();
    tcd1304ap::current_state=STATE_ERROR;
    return;    
  }
//  OCR1B=OCR2A; //trigger an adc conversion 1 sensor clock period after the timer goes to 0  (noisy signal)
  OCR1B=0; //trigger an adc conversion 1 sensor clock period after the timer goes to 0  (noisy signal)
  if(!tcd1304ap::current_prescaler_adc)
  {
    Serial.print("WARNING in tcd1304ap::setup_timer1_clock(): ADC prescaler was not set, setting up ADC.");   
    tcd1304ap::setup_adc();
  }
  OCR1A=OCR1B+13*tcd1304ap::current_prescaler_adc-5; //13 ADC clock cycles for ADC conversion, 5 CPU clock cycle to enter interupt routine
  if (OCR1A>ICR1)
  {
    Serial.print("WARNING in tcd1304ap::setup_timer1_clock(): data frequency or adc prescaler too high.");   
    OCR1A=ICR1;
  }
}

void tcd1304ap::setup_timer0_clock(unsigned long time_light_us)
{
  //disable timer0 compare match A interupt (OCE1A=1) and timer1 compare match B interupt (OCE1B=1)
  TIMSK0= 0;
 //clear timer0 interupt flags
  TIFR0= TIFR0; 
  //WGM22:0=2 CTC mode with OCR0A as TOP value
  //CS22:0=1 prescaler of 1
  //CS22:0=2 prescaler of 8
  //CS22:0=3 prescaler of 64
  //CS22:0=4 prescaler of 256
  //CS22:0=5 prescaler of 1024
  TCCR0A= (2<<WGM20);
  TCCR0B= (5<<CS20);
  //setup frequency : 1/T_light=F_CPU/(prescaler*2(1+ocr0aval))
  float OCR0A_val=((unsigned int) ( ((float) F_CPU)*(time_light_us*1e-6)/(1024)-1));
  if(int(OCR0A_val)>255) 
  {
    Serial.print("ERROR in tcd1304ap::setup_timer0_clock(): time_light_us=");
    Serial.print(time_light_us);
    Serial.print(" is too high, increase prescaller value of timer0.\n");
    Serial.flush();
    tcd1304ap::current_state=STATE_ERROR;
    OCR0A_val=255;
    return;    
  }
  if(int(OCR0A_val)<2) 
  {
    Serial.print("WARNING in tcd1304ap::setup_timer0_clock(): time_light_us=");
    Serial.print(time_light_us);
    Serial.print(" is too small, decrease prescaller value of timer0.\n");
    Serial.flush();
    OCR0A_val=1;
    return;        
  }
  OCR0A=uint8_t(OCR0A_val);
  dprint(OCR0A_val);
  tcd1304ap::current_time_light_us=time_light_us;
}


void tcd1304ap::setup_adc(uint8_t prescaler_adc)
{
  uint8_t ADPS0_val;
  switch(prescaler_adc)
  {
  case 2:
    ADPS0_val=1;
    break;
  case 4:
    ADPS0_val=2;
    break;
  case 8:
    ADPS0_val=3;
    break;
  case 16:
    ADPS0_val=4;
    break;
  case 32:
    ADPS0_val=5;
    break;
  case 64:
    ADPS0_val=6;
    break;
  case 128:
    ADPS0_val=7;
    break;
  default:
    Serial.print("ERROR in setup_adc(): unavailabe prescaler setting.\n");
    tcd1304ap::current_state=STATE_ERROR;
    Serial.flush();   
    return;   
  }
  tcd1304ap::current_prescaler_adc=prescaler_adc;

  //enable high speed mode (ADHSM=1)
  //set ADC trigger source - ADTS2:0=5 timer1 compare match B 
  ADCSRB = (1<<ADHSM)|(5<<ADTS0);
  //Select Vref=AVcc (REFS1:0=1) or 2.56V internal ref (REFS1:0=3
  //and set left adjust result (ADLAR=1)
  //input=ADC0 (MUX4:0=0)
  ADMUX = (3<<REFS0)|(1<<ADLAR)|((OS-38)<<MUX0);
  
  //Make an ADC conversion to initialise the analog circuitry
  //enable ADC (ADEN=1)
  //do not start a new conversion (ADSC=0)
  //enable autotriggering (ADATE=1)
  //clear ADC interupt flag (ADIF=1)
  //ADC Interupt disabled (ADIE=0)
  //set prescaler to 8 (ADPS2:0=3)
  ADCSRA = (1<<ADEN)|(1<<ADSC)|(1<<ADATE)|(1<<ADIF)|(ADPS0_val<<ADPS0);
  while( (ADCSRA&(1<<ADIF)) == LOW) //wait until 1st conversion is finished (25 ADC clock cycles instead of 13)
  {
    delay_us(5);
  }
  ADCSRA = (1<<ADEN)|(1<<ADATE)|(1<<ADIF)|(ADPS0_val<<ADPS0); //clear interupt flag
}

void tcd1304ap::measure(unsigned long time_light_us,float freq_data_MHz)
{
  if (freq_data_MHz==0)
  {
    if(tcd1304ap::current_freq_data_MHz==0)
      tcd1304ap::init();
  }else
  {
    tcd1304ap::init(freq_data_MHz);
  }

  if(time_light_us!=tcd1304ap::current_time_light_us)
  {
    dshow("capturing.\n");
    tcd1304ap::capture(time_light_us);
    dshow("done.\n");  
  }
  dshow("readout_and_capture, current_state=");
  dshow(tcd1304ap::current_state);
  dshow(".\n");
  tcd1304ap::readout_and_capture(time_light_us);
  dshow("done.\n");  
}

void tcd1304ap::send_last_measurement(uint16_t pixel_interval, uint16_t first_pixel, uint16_t last_pixel)
{
  TIMSK0=0; //disable timer0 interrupts
  TIMSK1=0; //disable timer1 interrupts
  TIMSK2=0; //disable timer2 interrupts
  Serial.print("data=");
  for (unsigned int elem=first_pixel;elem<=last_pixel;elem=elem+pixel_interval)
//  for (unsigned int elem=1960;elem<2055;elem++)
  {
    float data_elem_base10=10./256*tcd1304ap::current_data[elem];
    if (data_elem_base10<0.01)
      Serial.print(" ");
    else
      Serial.print((uint8_t) data_elem_base10);
  }
  Serial.print(" (");
  Serial.print(current_nread_error);
  Serial.print(" errors)\n");
  Serial.flush();  
}

void tcd1304ap::send_last_measurement_hex(uint16_t pixel_interval, uint16_t first_pixel, uint16_t last_pixel)
{
  TIMSK0=0; //disable timer0 interrupts
  TIMSK1=0; //disable timer1 interrupts
  TIMSK2=0; //disable timer2 interrupts
  Serial.print("data=");
  for (unsigned int elem=first_pixel;elem<=last_pixel;elem=elem+pixel_interval)
//  for (unsigned int elem=1960;elem<2055;elem++)
  {
    float data_elem=tcd1304ap::current_data[elem];
    if (data_elem==0)
      Serial.print("  ");
    else
      Serial.print((uint8_t) data_elem,HEX);
  }
  Serial.print(" (");
  Serial.print(current_nread_error);
  Serial.print(" errors)\n");
  Serial.flush();  
}


void tcd1304ap::send_last_light_shield(void)
{
  Serial.print("current_light_shield=");
  for (unsigned int elem=0;elem<LIGHTSHIELD_NELEMENT;elem++)
  {
    float data_elem_base10=10./256*tcd1304ap::current_light_shield[elem];
    if (data_elem_base10<0.01)
      Serial.print(" ");
    else
      Serial.print((uint8_t) data_elem_base10);
//    Serial.print(" ");
  }
  Serial.print(" \n");
  delay_us(100);
  Serial.flush();
}

//-----------------------------------------------------------
//-               non static part of the class              -
//-----------------------------------------------------------


#ifndef STATIC_ONLY
void tcd1304ap::clean(void)
{
  for (unsigned int elem=0;elem<SIGNAL_NELEMENT;elem++)
    this->data[elem]=0;
  for (unsigned int elem=0;elem<LIGHTSHIELD_NELEMENT;elem++)
    this->light_shield[elem]=0;
}

void tcd1304ap::capture(void)
{
  if(this->current_freq_data_MHz!=tcd1304ap::current_freq_data_MHz)
    tcd1304ap::init(this->freq_data_MHz);
  this->capture_duration=micros();
  tcd1304ap::capture(this->time_light_us);
  this->capture_duration=micros()-this->capture_duration;
  this->final_state=tcd1304ap::current_state;
}

void tcd1304ap::readout_and_capture(void)
{
  if(this->freq_data_MHz!=tcd1304ap::current_freq_data_MHz)
    tcd1304ap::init(this->freq_data_MHz);
  if(this->final_state!=STATE_CAPTURE_DONE)
    this->capture();
  if(this->time_light_us!=tcd1304ap::current_time_light_us)
    this->capture();    
 
  this->readout_and_capture_duration=micros();
  tcd1304ap::readout_and_capture(this->time_light_us);
  this->readout_and_capture_duration=micros()-this->readout_and_capture_duration;

  for (unsigned int elem=0;elem<SIGNAL_NELEMENT;elem++)
    this->data[elem]=tcd1304ap::current_data[elem];
  for (unsigned int elem=0;elem<LIGHTSHIELD_NELEMENT;elem++)
    this->light_shield[elem]=tcd1304ap::current_light_shield[elem];
  this->final_state=tcd1304ap::current_state;
  this->nread_error=tcd1304ap::current_nread_error;
}

tcd1304ap::tcd1304ap(unsigned long time_light_us,float freq_data_MHz)
{
  tcd1304ap::init(freq_data_MHz);
  this->time_light_us=time_light_us;
  this->freq_data_MHz=freq_data_MHz;
  this->final_state=STATE_UNSET;
  this->nread_error=0;
  this->clean(); 
  this->capture_duration=0;
  this->readout_and_capture_duration=0;
}

tcd1304ap::~tcd1304ap(void)
{
}

void tcd1304ap::measure(void)
{
  this->readout_and_capture();
}

void tcd1304ap::send_measurement(void)
{
  Serial.print("data=");
  for (unsigned int elem=0;elem<SIGNAL_NELEMENT;elem++)
  {
    Serial.print(this->data[elem]);
    Serial.print(" ");
  }
  Serial.print(" \n");
}

void tcd1304ap::send_light_shield(void)
{
  Serial.print("light_shield=");
  for (unsigned int elem=0;elem<LIGHTSHIELD_NELEMENT;elem++)
  {
    Serial.print(this->light_shield[elem]);
    Serial.print(" ");
  }
  Serial.print(" \n");
}

#endif //STATIC_ONLY

void delay_us(unsigned int us)
{
    us<<=2; //us multiplied by 4 as the lop takes 4 cycle to complete
    __asm__ __volatile__ (
         "1: sbiw %0,1" "\n\t" // 2 cycles, substract 1 from us
         "brne 1b" : "=w" (us) : "0" (us) // 2 cycles
   );
}


