#include <avr/io.h>
#include <avr/interrupt.h>
#define ADC_Channel   0b00000000 // 0b00000000 ADC0 Channel 0b00000001 ADC1 Channel
#define BUF_LEN 64

volatile uint16_t circ_buffer[BUF_LEN];
volatile uint8_t buf_in_idx=0;
volatile uint8_t buf_out_idx=0;
volatile uint8_t t = 0;
boolean timer0_init()
{
  TCCR0A = 0b00000010;    // timer 0 mode 2 - CTC
  TCCR0B = 0b00000100;    // set prescaler to 256
//  TCCR0B = 0b00000101;    // set prescaler to 1024
  OCR0A = 10;            // number of ticks in Output Compare Register
  TIMSK0 = (1 << OCIE1A); // trigger interrupt when ctr (TCNT0) >= OCR0A
  DDRB = (1<<PORTB5);
  led_off();
   
}

void init_adc_channel()
{
  // ATmega328p ADC Configuration Code 
   ADCSRA = 0;
   ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);// Select ADC Prescalar to 128,
                                                  // ADC clock 16MHz/128 = 125KHz
 
   ADMUX  |= ADC_Channel | (1<<REFS0);// | (1<<ADLAR);// Select the ADC channel to convert with internal reference 1.1V 

   ADCSRA |= (1<<ADEN) | (1<<ADSC);                    //Enable ADC and do 1 conversion
}

uint16_t read_adc()
{
  ADCSRA |= (1<<ADSC);      // Start the conversion
  while( !(ADCSRA & (1<<ADIF)));// Wait for conversion to finish
  //ADCSRA |= (1<<ADIF);   // Clear ADIF,ADIF is cleared by writing a 1 to ADSCRA
  return ADC;            // Read the ADC value to 16 bit int
}

void led_on()
{
  PORTB = (1<<PORTB5);
}

void led_off()
{
  PORTB = 0;
}

ISR(TIMER0_COMPA_vect) 
{    
  circ_buffer[buf_in_idx++] = read_adc();//analogRead(0);
  if(buf_in_idx>(BUF_LEN-1))
    {
      t++;
      
    }
  
  buf_in_idx&=(BUF_LEN-1);
}

void setup() {
Serial.begin(115200);
init_adc_channel_0();
timer0_init();
sei();                  // Enable interrupts
  
}
void toggle_led()
{
  PORTB ^= (1<<PORTB);
}
void loop()
{

  while(1) //while(1) is faster than loop()
  {
    if (t==0)
    {
      buf_out_idx &=(BUF_LEN-1);
      for(;buf_out_idx<buf_in_idx;buf_out_idx++)
      {
        Serial.print(circ_buffer[buf_out_idx]);
        Serial.write('\n');          
      }
      
      
    }
    else if(~(t==0))
    {
      
      for(;buf_out_idx<BUF_LEN;buf_out_idx++)
      {
        Serial.print(circ_buffer[buf_out_idx]);
        Serial.write('\n');   
        
      }
      t--;
      buf_out_idx &=(BUF_LEN-1);

    }
  if(t>5)
  led_on();
  }
  
}
  
