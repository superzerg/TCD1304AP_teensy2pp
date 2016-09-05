//comment the line below only if you got more than 16kB of memory
#define STATIC_ONLY

#ifndef TCD134AP_H

#define TCD134AP_H

#include <Arduino.h>

#define STATE_ERROR -1
#define STATE_UNSET 0
#define STATE_READY 1
#define STATE_CAPTURING 2
#define STATE_CAPTURE_DONE 4
#define STATE_READOUT 5
#define STATE_ALL_DATA_MEASURED 7

#define LIGHTSHIELD_FIRST_ELEMENT 16 //elements starting from 0 to 3693
#define LIGHTSHIELD_NELEMENT 13
#define SIGNAL_FIRST_ELEMENT_FROM_LIGHTSHIELD_FIRST_ELEMENT 16
#define SIGNAL_NELEMENT 3648
#define FULL_LINE_NELEMENT 3694

#define ADHSM 7
#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))
#define CPU_16MHz       0x00

#define DEBUG 0

#if DEBUG ==1
#define dprintBIN(expression) Serial.print("# "); Serial.print( #expression ); Serial.print( ": " ); Serial.println( expression , BIN); Serial.flush()
#define dprint(expression) Serial.print("# "); Serial.print( #expression ); Serial.print( ": " ); Serial.println( expression ); Serial.flush()
#define dshow(expression) Serial.println( expression ); Serial.flush()
#else
#define dprintBIN(expression)
#define dprint(expression)
#define dshow(expression)
#endif

const int PhiM=24; // OC2A/PB4 pin
const int ICG=16; // OC3A/PC6 pin
const int SH=25; //OC1A/PB5
const int OS=38; //ADC0/PF0 pin
const int LED=10; //PC0 pin

#define LED_ON() PORTC|= 1
#define LED_OFF() PORTC&= 254
#define LED_STATUS() (PORTC&1) //return true if lED is ON, false if LED is OFF
#define ICG_HIGH() PORTC|= 1<<6
#define ICG_LOW() PORTC&= 191

class tcd1304ap
{
public:
  volatile static uint8_t current_data[SIGNAL_NELEMENT];
  volatile static uint8_t current_light_shield[LIGHTSHIELD_NELEMENT];
  volatile static int8_t current_state; 
  volatile static unsigned int current_nread; //number of element read
  volatile static unsigned int current_nread_adc; //number of adc read
  volatile static unsigned int current_nread_error; //number of failed adc read
  static float current_freq_data_MHz; //data rate of the sensor (in MHz, between 0.2 and 1)
  static unsigned int current_time_light_us; //time during which the led is switched on (in us, must be >1.0)
  static uint8_t current_prescaler_adc; //time during which the led is switched on (in us, must be >1.0)
  
  static void clean_current(void); //set all elements of data and light_shield to 0
  static void capture(unsigned long time_light_us); //start a new capture with the given time_light_us
  static void readout_and_capture(unsigned long time_light_us); //read the data from the last capture at the data rate specified in freq_data_MHz
  
  static void setup_timer2_clock(float freq_data_MHz); //setup the timer2 clock to generate the clock of the sensor for the data rate specified in freq_data_MHz.
  
   /* set up timer1 compare match A and B interupts to read the value returned by the ADC. 
  Once ADC is enabled, it will auto-trigger on timer1 compare match B
  The timer1 compare match A interupt will:
    - If an ADC conversion is still ongoing, nread_error is increased.
    - If the ADC conversion is done, to read the value returned by the ADC and set the corresponding data element.
    - If all data has been measured, state is set to STATE_ALL_DATA_MEASURED.
  */
  static void setup_timer1_clock(float freq_data_MHz); //set up timer1 compare match A interupt to read the value returned by the ADC at the given frequency. 
  static void setup_timer0_clock(unsigned long time_light_us);

  static void setup_adc(uint8_t prescaler_adc=4); //set up ADC to trigger on timer1 compare match B
  static void enable_adc(void);
  
public: 
  static void init(float freq_data_MHz=0.1, uint8_t prescaler_adc=4); 
  static void measure(unsigned long time_light_us,float freq_data_MHz=0); //start a measurement and read out the data from the linear CCD sensor
  static void send_last_measurement(uint16_t pixel_interval=1, uint16_t first_pixel=0, uint16_t last_pixel=SIGNAL_NELEMENT-1); //send the last measurement of the data through the serial connection
  static void send_last_measurement_hex(uint16_t pixel_interval=1, uint16_t first_pixel=0, uint16_t last_pixel=SIGNAL_NELEMENT-1); //send the last measurement of the data through the serial connection
  static void send_last_light_shield(void);//send the last measurement of the light shield through the serial connection

#ifndef STATIC_ONLY
private:
  unsigned int8_t data[SIGNAL_NELEMENT];
  unsigned int8_t light_shield[LIGHTSHIELD_NELEMENT];
  int8_t final_state;
  unsigned int nread_error; //number of failed adc read
  float freq_data_MHz; //data rate of the sensor (in MHz, between 0.2 and 1)
  unsigned long time_light_us; //time during which the led is switched on (in us, must be >1.0)
  unsigned int capture_duration;
  unsigned int readout_and_capture_duration;
 
  void clean(void); //set all elements of data and light_shield to 0
  void capture(void); //start a new capture.
  void readout_and_capture(void); //start a new capture while reading the data.
public: 
  tcd1304ap(unsigned long time_light_us,float freq_data_MHz=0.2); //constructor
  ~tcd1304ap(void); //destructor
  void measure(void);
  void send_measurement(void);
  void send_light_shield(void);
#endif //STATIC_ONLY
};

void delay_us(unsigned int us);

#endif //TCD134AP_H
