#include "Arduino.h"

const uint8_t max_pwm_adr = 0x00;
const uint8_t min_pwm_adr = 0x02;

const uint8_t pwm_start_value = 35;
const uint8_t pwm_min_value = 35;
const uint8_t pwm_max_value = 255;
uint8_t motor_speed = 0;
uint8_t pwm_pin_change = 0;


uint64_t pwm_end = 0;
uint64_t pwm_start = 0;

const uint16_t min_pwm = 1000;
const uint16_t max_pwm = 2000;
uint16_t motor_off_timer = 1000;
uint16_t pwm_input = 550;
uint16_t soft_start_timer = 2200;

static bool motor_enabled = false;
static uint8_t sequence_step = 0;

void setup()
{  
  // The pins for the MOSFET drivers are PB1, PB2, PB3, PD2, PD3 and PD4
  DDRB  |= B00001110;           // Configure pins PB1, PB2, PB3 as output (CH, BH, AH)
  PORTB &= B00000000;           // Pins 0 to 7 set to LOW
  DDRD  |= B00011100;           // Configure pin PD2, PD3, PD4 as output (CL, BL, AL)
  PORTD &= B00000000;           // Pins 0 to 7 set to LOW

  // Timer0
  TCCR1A = 0;
  TCCR1B = 0x01; // no prescaling

  // Timer1
  TCCR2A = 0;
  TCCR2B = 0x01; // no prescaling

  // Comparator on pin PD6
  ACSR   = 0x10;             // enable comparator interrupt

  // Set PB0 (PWM in) to trigger interrupt (we use this to read PWM input)
  PCICR  |= (1 << PCIE0);    // enable PCMSK0 scan, PWM_IN (PB0), which is PCIE0 in PCICR register.                                           
  PCMSK0 |= (1 << PCINT0);   // Set pin PB0 to trigger an interrupt on state change. 

  // For the comparator to be functional the ADC has to be disabled.
  ADCSRA = (0 << ADEN);     // Disable the ADC
  ADCSRB = (1 << ACME);     // MUX select for negative input of comparator
}

void soft_start()
{
  set_pwm(pwm_start_value);
  soft_start_timer = 2000;

  while(soft_start_timer > 500) {
    delayMicroseconds(soft_start_timer);
    set_next_step();
    sequence_step++;
    sequence_step %= 6;
    soft_start_timer -= 10;
  }

  motor_speed = pwm_start_value;    //??
  ACSR |= 0x08;                    // Enable analog comparator interrupt 
}

void motor_stop()
{
  if(!motor_off_timer) // to assure that the motor doesn't stop immediately
  {
    motor_enabled = false;
    ACSR   = 0x10;            // Disable and clear (flag bit) analog comparator interrupt
    motor_off_timer = 1000;
    PORTD = B00000000;
    PORTB = B00000000;
    TCCR1A =  0;   
    TCCR2A =  0;            
  }
  motor_off_timer = motor_off_timer - 1;       
}

void loop() {
  // if the PWM input is higher than a given threshold we start the motor
  if(pwm_input > min_pwm + 40)
  {
    motor_enabled = true;    
    motor_off_timer = 1000;
  }

  if(motor_enabled)
  {  
    soft_start();
    while(motor_enabled) 
    {
      pwm_input = constrain(pwm_input, min_pwm, max_pwm);
      motor_speed = map(pwm_input, min_pwm, max_pwm, pwm_min_value, pwm_max_value);
      set_pwm(motor_speed);
      if(pwm_input < min_pwm) // stop the motor
      {
        motor_stop();
      }
    }
  } 
} 

// Setting the timers to the PWM value
void set_pwm(byte pwm_value){
  OCR1A  = pwm_value;                   // Set pin PB1 PWM duty cycle
  OCR1B  = pwm_value;                   // Set pin PB2 PWM duty cycle
  OCR2A  = pwm_value;                   // Set pin PB3 PWM duty cycle
}

// Detecting the 0 crossing on falling and rising edges.
void BEMF_A_RISING(){  
  ADMUX = 2;                // Select PC2 as comparator negative input
  ACSR |= 0x03;             // Set interrupt on rising edge*/
}

void BEMF_A_FALLING(){
  ADMUX = 2;                // Select PC2 as comparator negative input
  ACSR &= ~0x01;            // Set interrupt on falling edge*/
}

void BEMF_B_RISING(){
  ADMUX = 0;                // Select PC0 as comparator negative input
  ACSR |= 0x03;             // Set interrupt on rising edge
}

void BEMF_B_FALLING(){
  ADMUX = 0;                // Select PC0 as comparator negative input
  ACSR &= ~0x01;            // Set interrupt on falling edge*/
}

void BEMF_C_RISING(){
  ADMUX = 1;                // Select PC1 as comparator negative input
  ACSR |= 0x03;             // Set interrupt on rising edge
}

void BEMF_C_FALLING(){
  ADMUX = 1;                // Select PC1 as comparator negative input
  ACSR &= ~0x01;            // Set interrupt on falling edge
}

// ACO indicates the output of the comparator, when AIN0 (PD6 - Virtual 0) voltage is 
// higher than the negative pin voltage then the ACO bit is set, otherwise it is clear.
ISR (ANALOG_COMP_vect) {
  for(int i = 0; i < 10; i++) {
    if(sequence_step & 1)             // If step = odd (0001, 0011, 0101) 1, 3 or 5
    {
      if(!(ACSR & 0x20)) i -= 1;      // ACO = 0 (Analog Comparator Output = 0) -> BEMF > Virtual 0
    }
    else                              // else if step is 0, 2 or 4
    {
      if((ACSR & 0x20)) i -= 1;       // ACO = 1 (Analog Comparator Output = 1) -> BEMF < Virtual 0
    }
  }
  set_next_step();                    // set the next step of the sequence
  sequence_step++;                    // increment step by 1, next part of the sequence of 6
  sequence_step %= 6;                 // If step > 5 (equal to 6) then step = 0 and start over
}

void set_next_step(){        
  switch(sequence_step){
    case 0:
      AH_BL();
      BEMF_C_RISING();
      break;
    case 1:
      AH_CL();
      BEMF_B_FALLING();
      break;
    case 2:
      BH_CL();
      BEMF_A_RISING();
      break;
    case 3:
      BH_AL();
      BEMF_C_FALLING();
      break;
    case 4:
      CH_AL();
      BEMF_B_RISING();
      break;
    case 5:
      CH_BL();
      BEMF_A_FALLING();
      break;
  }
}

/*On each step we change the digital pins to be HIGH or LOW or to be PWM or no-PWM 
depending on which step of the sequence we are*/
void AH_BL(){
  PORTD  = B00001000;     
  TCCR1A =  0;         
  TCCR2A =  0x81;         // OC2A - compare match noninverting mode, downcounting, PWM 8-bit      
}

void AH_CL(){
  PORTD  = B00000100;      
  TCCR1A =  0;         
  TCCR2A =  0x81;         // OC2A - compare match noninverting mode, downcounting, PWM 8-bit      
}

void BH_CL(){
  PORTD  = B00000100;      
  TCCR2A =  0;            
  TCCR1A =  0x21;         // OC1B - compare match noninverting mode, downcounting, PWM 8-bit 
}

void BH_AL(){  
  PORTD  = B00010000;      
  TCCR2A =  0;                  
  TCCR1A =  0x21;         // OC1B - compare match noninverting mode, downcounting, PWM 8-bit 
}

void CH_AL(){ 
  PORTD  = B00010000;       
  TCCR2A =  0;          
  TCCR1A =  0x81;         // OC1A - compare match noninverting mode, downcounting, PWM 8-bit 
}

void CH_BL(){  
  PORTD  = B00001000;        
  TCCR2A =  0;             
  TCCR1A =  0x81;         // OC1A - compare match noninverting mode, downcounting, PWM 8-bit
}

/* This is the interruption routine on pin change
   in this case for digital pin PB0 which is the PWM input */
ISR(PCINT0_vect){
  // First we take the current time in micro seconds using the micros() function
  pwm_end = micros();

  if(PINB & 0x01){                                   // We make an AND with the pin state register, We verify if pin 8 is HIGH
    if(pwm_pin_change == 0){                         // If the last state was 0, then we have a state change...
      pwm_pin_change = 1;                            // Store the current state into the last state for the next loop
      pwm_start = pwm_end;                           // Set counter_1 to current value.
    }
  }
  else if(pwm_pin_change == 1){                      // If pin PB0 is LOW and the last state was HIGH then we have a state change      
    pwm_pin_change = 0;                              // Store the current state into the last state for the next loop
    pwm_input = pwm_end - pwm_start;                 // We make the time difference. PWM_INPUT is current_time - counter_1 in micro-seconds.
  }
}