#include <avr/io.h>
#include <avr/interrupt.h>

#define BUF_LEN 64
volatile uint16_t circ_buffer[BUF_LEN];
volatile uint8_t buf_in_idx=0;
volatile uint8_t buf_out_idx=0;
volatile uint8_t t = 0;
boolean timer0_init()
{
  TCCR0A = 0b00000010;    // C1:: timer 0 mode 2 - CTC
  TCCR0B = 0b00000100;    // C2:: set prescaler to 256
//  TCCR0B = 0b00000101;    // C2:: set prescaler to 1024
  OCR0A = 40;            // C3:: number of ticks in Output Compare Register
  TIMSK0 = (1 << OCIE1A); // C4:: trigger interrupt when ctr (TCNT0) >= OCR0A
  DDRB = (1<<PORTB5);
  sei();                  // C6:: Enable interrupts
  led_off();
   
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
  circ_buffer[buf_in_idx++] = analogRead(0);
  if(buf_in_idx>(BUF_LEN-1))
    {
      t++;
      
    //  Serial.println("BINGO");
    }
  
  buf_in_idx&=(BUF_LEN-1);
 // Serial.println(circ_buffer[buf_in_idx]);
  //Serial.write('\n');
}

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);

timer0_init();

}
void toggle_led()
{
  PORTB ^= (1<<PORTB);
}
void loop()
{

  while(1)
  {
    if (t==0)
    {
      buf_out_idx &=(BUF_LEN-1);
      for(;buf_out_idx<buf_in_idx;buf_out_idx++)
      {
        Serial.print(circ_buffer[buf_out_idx]);
        Serial.write('\n');  
       // Serial.print(buf_out_idx);
       // Serial.print(" ");
       // Serial.println(buf_in_idx);
        
      }
      
      
    }
    else if(~(t==0))
    {
      
      for(;buf_out_idx<BUF_LEN;buf_out_idx++)
      {
        Serial.print(circ_buffer[buf_out_idx]);
        Serial.write('\n');   
      //  Serial.print(buf_out_idx);
      //  Serial.print(" ");
      //  Serial.println(buf_in_idx);

        
      }
      t--;
      buf_out_idx &=(BUF_LEN-1);

    }
  if(t>5)
  led_on();
  }
  
}
  
