volatile uint8_t valX = 0;
volatile uint8_t valY = 0;
volatile bool X = true;

int main() {
  DDRB |= (1 << DDB3) | (1 << DDB2) | (1 << DDB1) | (1 << DDB0); //Sets Port B Pins as Outputs
  DDRD |= (1 << DDD6) | (1 << DDD5); //Sets Port D Pins as Outputs
  TCCR1A = 0; //Reset Register
  TCCR1B = 0; //Reset Register
  TCCR1A = (1 << COM1A1) | (1 << WGM10); //Enable Compare Output Mode 1 and set Waveform Generation Mode 3
  TCCR1B = (1 << WGM12) | (1 << CS10); //Select no prescaler (16MHz -> 62.75kHz PWM)
  OCR1A = 127; //Set compare register to 127 (127/255 ~ 50 -> 50% duty cycle)
  
  TCCR0A = 0; //Clear TCCR0A Register
  TCCR0B = 0; //Clear TCCR0B Register
  TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00); //Enable Compare Output Mode 1 and set Waveform Generation Mode 3
  TCCR0B = (1 << CS00); //Select no prescaler (16MHz -> 62.75kHz PWM)
  OCR0A = 127; //Set compare register to 127 (127/255 ~ 50 -> 50% duty cycle)
  
  DDRC &= ~(1 << DDC0); //Pin C0 as Input
  ADMUX = (1 << REFS0) | (1 << ADLAR); //Sets voltage reference to AVCC and ADC resolution to 8 bits
  ADCSRA = (1 << ADEN) | (1 << ADIE); //Enables ADC module and ADC interrupt
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //Sets ADC prescaler to 128 (125kHz Sample Rate; must be below 200kHz)
  ADCSRA |= (1 << ADSC); //Starts an ADC conversion (Analog Read)
  sei(); //Enables global interrupts (for interrupt service routines (ISRs))
 
  while(1) {
    if (valX < 31) { //Move Foward, Left Wheel
      PORTB |= (1 << PORTB2);
      PORTB &= ~(1 << PORTB3);
    }
    else if (valX < 223) { //Do Nothing, Left Wheel
      PORTB &= ~(1 << PORTB2);
      PORTB &= ~(1 << PORTB3);
    }
    else { //Move Backwards, Left Wheel
      PORTB |= (1 << PORTB0);
      PORTB &= ~(1 << PORTB2);
    }
    
    if (valY < 31) { //Move Foward, Right Wheel
      PORTB |= (1 << PORTB0);
      PORTD &= ~(1 << PORTD5);
    }
    else if (valY < 223) { //Do Nothing, Right Wheel
      PORTB &= ~(1 << PORTB0);
      PORTD &= ~(1 << PORTD5);
    }
    else { //Move Backwards, Right Wheel
      PORTD |= (1 << PORTD5);
      PORTB &= ~(1 << PORTB0);
    }
  }
}

ISR(ADC_vect) {
  if (X) {
    valX = ADCH; //Reads the converted value
    ADMUX |= (1 << MUX0); //Sets next reading to Pin C1
    X = false;
    ADCSRA |= (1 << ADSC); //Starts another conversion
  }
  else {
    valY = ADCH; //Reads the converted value
    ADMUX &= ~(1 << MUX0); //Sets next reading to Pin C0
    X = true;
    ADCSRA |= (1 << ADSC); //Starts another conversion
  }
}
