#include "IRremote.h"
#include <avr/interrupt.h > 
#include <avr/io.h > 
#if defined(__AVR_ATmega328P__)
// A mega328 is mostly like an mega168, in terms of peripherals
#define __AVR_ATmega168__ 1
#endif
#define INIT_TIMER_COUNT 6
#define RESET_TIMER1 TCNT1 = INIT_TIMER_COUNT 

#define TIMER1_MILLISECONDS_FIRE 500
#define TIMER1_MILLISECONDS_FIRE_OFF 150

// variables for Timer1 counter
int timer1_int_counter = 0;


#define HORN_VALUE 16753245
#define HEADLIGHTS_VALUE 16744575   // vol up
#define MOTOR_FASTER_VALUE 16736415   // track next
#define MOTOR_SLOWER_VALUE 16712445   // track prev
#define KILL_SWITCH 16720605  // play/pause

const byte _irReceiverPin = 2;
const byte _headlightsPin = 12;
const byte _hornPin = 11;
const byte _motorPin = 5;

byte _motorSpeed = 0;

boolean _hornOn = false;
boolean _lightsState = false;

IRrecv irrecv(_irReceiverPin);
decode_results decodedSignal;

unsigned long last = millis();

// TIMER 1
ISR(TIMER1_OVF_vect) 
{ 
	RESET_TIMER1; 
	timer1_int_counter += 1; 
	if (timer1_int_counter == TIMER1_MILLISECONDS_FIRE * 10) 
	{
	  timer1FireOn();
	}
	if (timer1_int_counter == TIMER1_MILLISECONDS_FIRE * 10 + TIMER1_MILLISECONDS_FIRE_OFF) 
	{
          timer1_int_counter = 0; 
	  timer1FireOff();
	}
};

void initInterrupts()
{
  // turn off all interrupts so the microprocessor doesn't get confused
  cli();
  initTimerInterrupts();
  // turn on all interrupts so the microprocessor can handle them
  sei();
}

void initTimerInterrupts()
{
  // DON'T ALTER THIS CODE!!!!
  //Timer2 Settings: Timer Prescaler /64, 
  TCCR2A |= (1<<CS22);    // turn on CS22 bit
  TCCR2A &= ~((1<<CS21) | (1<<CS20));    // turn off CS21 and CS20 bits
  // Use normal mode
  TCCR2A &= ~((1<<WGM21) | (1<<WGM20));   // turn off WGM21 and WGM20 bits
  // turn off register A of Timer 1
  // Timer 1 is a 16 bit register, by turning off register A 
  // we make it an 8 bit register
  TCCR1A = 0;
  // set register B of Timer 1 the same way we did for Timer 2
  // the constants used for this initialization are all '1'
  TCCR1B = _BV(WGM13); 
  TCCR1B |= (1<<CS12);    // turn on CS12 bit
  TCCR1B &= ~((1<<CS11) | (1<<CS10));    // turn off CS211 and CS10 bits
  // Use normal mode
  TCCR1B &= ~((1<<WGM11) | (1<<WGM10));   // turn off WGM11 and WGM10 bits
  // Use internal clock - external clock not used in Arduino
  ASSR |= (0<<AS2);
  TIMSK1 |= (1<<TOIE1) | (0<<OCIE1A);  
  //Timer1 Overflow Interrupt Enable
  RESET_TIMER1;
}

void setup()
{
  Serial.begin(9600);
  pinMode(_headlightsPin, OUTPUT);
  pinMode(_hornPin, OUTPUT);
  
  initInterrupts();
  
  irrecv.enableIRIn();
}

void loop()
{
  if (irrecv.decode(&decodedSignal))
  {
    if (millis() - last > 250)
    {
      switch (decodedSignal.value)
      {
        // horn button
        case HORN_VALUE:
          _hornOn = true;
          break;
         // lights button
         case HEADLIGHTS_VALUE:
           _lightsState = !_lightsState;
           break;
         // motor faster
         case MOTOR_FASTER_VALUE:
           if (_motorSpeed == 0)
             _motorSpeed = 127;
           else if (_motorSpeed == 127)
             _motorSpeed = 255;
           break;
         // motor slower
         case MOTOR_SLOWER_VALUE:
           if (_motorSpeed == 255)
             _motorSpeed = 127;
           else if (_motorSpeed == 127)
             _motorSpeed = 0;
           break;
         case KILL_SWITCH:
           _motorSpeed = 0;
           _lightsState = false;
           _hornOn = false;
           break;
      }
      
      analogWrite(_motorPin, _motorSpeed);
      Serial.println(_motorSpeed);
      Serial.println(decodedSignal.value);
    }
    last = millis();
    irrecv.resume();
  }
}
void timer1FireOn()
{
  if (_lightsState)
    digitalWrite(_headlightsPin, HIGH);
  if (_hornOn)
    digitalWrite(_hornPin, HIGH);
}
void timer1FireOff()
{
  digitalWrite(_headlightsPin, LOW);
  _hornOn = false;
  digitalWrite(_hornPin, LOW);
}
