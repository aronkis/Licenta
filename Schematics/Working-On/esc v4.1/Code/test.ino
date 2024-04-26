void setup()
{
  
  DDRB  |= B00001110;       // Configure pins PB1, PB2, PB3 as output (CH, BH, AH)    
  PORTB &= B00000000;           
  DDRD  |= B00011100;       // Configure pin PD2, PD3, PD4 as output (CL, BL, AL)    
  PORTD &= B00000000;   

  delay(1500);
}

void loop()
{
  PORTB = B00000010;
  PORTD = B00001000;
}