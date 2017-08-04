/*
 * Help with timers thanks to Dr. Erik Petrich @ Univ of Oklahoma
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#define TOP_TC0 99

void init();

void startADC0();
void stopADC0();

void startTC0();
void stopTC0();

void startTC1();
void stopTC1();

void startTC2();
void stopTC2();

void startBoost();
void stopBoost();

void start595();
void output595(uint16_t output);

void init()
{
    // MASTER tri-state all pins during init, protects inductor
    MCUCR |= (1 << PUD); 

    // set all pins as inputs
    DDRB = DDRD = 0x00;
    DDRC &= 0x80;

    // turn on all pins pullups...
    PORTB = PORTD = 0xFF;
    PORTC |= ~(0x80);

    // ...EXCEPT for OC0B (PD5), OC1A (PB1), protects inductor
    PORTB &= ~(1 << PORTB1);
    PORTD &= ~(1 << PORTD5);

    // globally enable interrupts in SREG
    sei();

    // master reenable all pullups
    MCUCR &= ~(1 << PUD);
}

void startADC0()
{
    // TRI-STATE ADC PIN FOR HIGH IMPEDANCE READING, NO PULLUP
    PORTC &= ~(1 << PORTC5);

    // disable digital input on ADC0 (PC0)
    DIDR0 |= (1 << ADC5D);

    /* ADC MULTIPLEXER SELECTION REGISTER (ADMUX) */

    // reference internal 1.1V
    ADMUX |= (1 << REFS1);
    ADMUX |= (1 << REFS0); // REFS = 0b11

    //// left adjust conversion result, ADCH = 9:2
    //ADMUX |= (1 << ADLAR);

    // right adjust conversion result, ADCH = 10:8
    ADMUX &= ~(1 << ADLAR);

    // select ADC5 as input channel
    ADMUX &= ~(1 << MUX3);
    ADMUX |=  (1 << MUX2);
    ADMUX &= ~(1 << MUX1);
    ADMUX |=  (1 << MUX0); // MUX = 0b0101

    /* ADC CONTROL AND STATUS REGISTER A (ADCSRA) */

    // set ADC prescaler to 16MHz/128 = 125kHz
    ADCSRA |= (1 << ADPS2);
    ADCSRA |= (1 << ADPS1);
    ADCSRA |= (1 << ADPS0); // ADPS = 0b111

    // enable ADC interrupts
    ADCSRA |= (1 << ADIE);

    // enable the ADC
    ADCSRA |= (1 << ADEN);

    // start sampling
    ADCSRA |= (1 << ADSC);
}

void stopADC0()
{
    /* ADC CONTROL AND STATUS REGISTER A (ADCSRA) */

    // disable ADC interrupts
    ADCSRA &= ~(1 << ADIE);

    // disable the ADC
    ADCSRA &= ~(1 << ADEN);
    
    // reset ADC prescaler to 16MHz/2
    ADCSRA &= ~(1 << ADPS2);
    ADCSRA &= ~(1 << ADPS1);
    ADCSRA &= ~(1 << ADPS0); // ADPS = 0b000
 
    /* ADC MULTIPLEXER SELECTION REGISTER (ADMUX) */
    
    // reset reference voltage to AREF pin
    ADMUX &= ~(1 << REFS1);
    ADMUX &= ~(1 << REFS0); // REFS = 0b00

    // reset right adjust conversion result, ADCH = 9:8
    ADMUX &= ~(1 << ADLAR);

    // default ADC0 as input channel
    ADMUX &= ~(1 << MUX3);
    ADMUX &= ~(1 << MUX2);
    ADMUX &= ~(1 << MUX1);
    ADMUX &= ~(1 << MUX0); // MUX = 0b0000

    // reset ADC pin as pullup
    PORTC |= (1 << PORTC5);

    // reenable digital input on ADC0 (PC0)
    DIDR0 &= ~(1 << ADC5D);
}

void startTC0()
{
    // set PWM pin OC0B tri-state
    PORTD &= ~(1 << PORTD5);
    
    // set PWM pin OC0B as input
    DDRD &= ~(1 << DDD5);

    // Wave Generation Mode, "Fast PWM" from BOTTOM to TOP_TC0 (OCRA)
    TCCR0B |=  (1 << WGM02); 
    TCCR0A |=  (1 << WGM01);
    TCCR0A |=  (1 << WGM00); // WGM0 = 0b111

    // Comp Output Mode - Chan A, "Normal Port Op, 0C0A discon"
    TCCR0A &= ~(1 << COM0A1);
    TCCR0A &= ~(1 << COM0A0); // COM0A = 0b00

    // Comp Output Mode - Chan B, "Set OC0B on C. Match, Inverted"
    TCCR0A |=  (1 << COM0B1);
    TCCR0A |=  (1 << COM0B0); // COM0B = 0b11

    // start timer, conf no prescaler
    TCCR0B &= ~(1 << CS02); 
    TCCR0B &= ~(1 << CS01);
    TCCR0B |=  (1 << CS00); // CS0 = 0b001

    // TOP_TC0 value, sets period
    OCR0A = TOP_TC0;

    // default 0% duty, PWM goes high at this value
    OCR0B = TOP_TC0;

    // unleash the waveform, set PWM pin OC0B as output
    DDRD |=  (1 << DDD5);
}

void stopTC0()
{
    // set PWM pin tri-state
    PORTD &= ~(1 << PORTD5);
    
    // set PWM pin as input
    DDRB &= ~(1 << DDB5); // PWM output now effectively stopped
    
    // stop timer, disable clock source
    TCCR0B &= ~(1 << CS02);
    TCCR0B &= ~(1 << CS01);
    TCCR0B &= ~(1 << CS00); // CS0 = 0b000

    // Wave Generation Mode, Normal
    TCCR0B &= ~(1 << WGM02); 
    TCCR0A &= ~(1 << WGM01);
    TCCR0A &= ~(1 << WGM00); // WGM = 0b000;

    // Comp Output Mode - A-chan, "Normal Port Op, 0C0A discon"
    TCCR0A &= ~(1 << COM0A1);
    TCCR0A &= ~(1 << COM0A0); // COM0A = 0b00

    // Comp Output Mode - B-chan, "Norman Port Op, 0C0B discon"
    TCCR0A &= ~(1 << COM0B1);
    TCCR0A &= ~(1 << COM0B0); // COM0B = 0b00
}

void startTC1()
{
    // set PWM pins tri-state
    PORTB &= ~(1 << PORTB1);
    
    // set PWM pin as input
    DDRB &= ~(1 << DDB1);

    // set the period, last number before rolling to 0x0000
    //ICR1 = 0x0FFF;

    // set the duty cycle, number to set pulse high
    // NOTE duty cycle is backwards, e.g. OCR1A = 0 means 100% duty
    OCR1A = 0;

    // set
    OCR1B = 0;

    // Wave Generation Mode, "PCPWM, 8-bit"
    TCCR1B |=  (1 << WGM13);
    TCCR1B |=  (1 << WGM12); 
    TCCR1A |=  (1 << WGM11);
    TCCR1A &= ~(1 << WGM10); // WGM1 = 0b1110;

    // Comp Output Mode - Chan A, "Clear OC1A on Compare Match"
    TCCR1A |=  (1 << COM1A1);
    TCCR1A &= ~(1 << COM1A0); // COM1A = 0b10

    // Comp Output Mode - Chan B, "Set OC1B on Compare Match"
    TCCR1A |=  (1 << COM1B1);
    TCCR1A |=  (1 << COM1B0); // COM1B = 0b11

    // start timer, prescaler F_osc/8
    TCCR1B &= ~(1 << CS12); 
    TCCR1B |=  (1 << CS11); // datasheet entry missing for CS11?
    TCCR1B &= ~(1 << CS10); // CS1 = 0b010

    // unleash the waveform, set PWM pins OC1A and OC1B as output
    DDRB |=  (1 << DDB1);
    DDRB |=  (1 << DDB2);
}

void stopTC1()
{
    // set PWM pin tri-state
    PORTB &= ~(1 << PORTB1);

    // set PWM pin as input
    DDRB &= ~(1 << DDB1); // PWM output now effectively stopped

    // stop timer, disable clock source
    TCCR1B &= ~(1 << CS12);
    //TCCR1B &= ~(1 << CS11); // datasheet entry missing for CS11?
    TCCR1B &= ~(1 << CS10); // CS0 = 0b000

    // conf Waveform Generation Mode, Normal
    TCCR1B &= ~(1 << WGM13);
    TCCR1B &= ~(1 << WGM12); 
    TCCR1A &= ~(1 << WGM11);
    TCCR1A &= ~(1 << WGM10); // WGM = 0b000;

    // conf Comp Output Mode, "Normal"
    TCCR1A &= ~(1 << COM1A1);
    TCCR1A &= ~(1 << COM1A0); // COM0A = 0b00
}

void startTC2()
{

}

void stopTC2()
{

}

void startBoost()
{
    startADC0(); // start monitoring boost outut voltage
    startTC0(); // start pulsing inductor
    //startTC1(); // start multiplier switching
}

void stopBoost()
{
    stopTC0(); // stop pulsing inductor
    //stopTC1(); // stop multiplier switching
    stopADC0(); // stop monitoring boost output voltage
}

void start595()
{
    // 595 SER (PD4)
    DDRD |= (1 << DDD4); // set PORTD4 as output
    PORTD &= ~(1 << PORTD4); // set SER low

    // 595 RCLK on PD3
    DDRD |= (1 << DDD3); // set PORTD3 as output
    PORTD |= (1 << PORTD3); // set RCLK high

    // 595 SRCLK on PD2
    DDRD |= (1 << DDD2); // set PORTD2 as output
    PORTD &= ~(1 << PORTD2); // set SRCLK low

    // zero out 595
    output595(0x0000);
}

/*
 * Takes in an unsigned 16 bit number and sends it serially to
 * a shift register.
 *
 * Communicate with an SN74HC595 shift register using 3 AVR pins:
 * 
 * SER on the 595 is on PD4
 * SCLK on the 595 is PD3
 * SRCLK on the 595 is on PD2
 *
 * SER bit twiddle modified from:
 * https://graphics.stanford.edu/~seander/bithacks.html
 */
void output595(uint16_t output)
{
    PORTD &= ~(1 << PORTD3); // bring SCLK low
    
    for(uint8_t i = 16; i; --i) // each bit in the vector
    {
        // SER 595 data to be shifted in
        PORTD ^= ((~(output & 1) + 1) ^ PORTD) & (1 << PORTD4);

        PORTD |= (1 << PORTD2); // SRCLK 595 high, shift in SER data
        output >>= 1; // move next bit, introduce clock delay
        PORTD &= ~(1 << PORTD2); // SRCLK 595 low
    }

    PORTD |= (1 << PORTD3); // move to storage register, SCLK high
}

void stop595()
{
    // zero out shift register
    output595(0x0000);

    // SER 595 low
    PORTD &=  ~(1 << PORTD4);

    // SRCLK and SCLK 595 high
    PORTD |=  (1 << PORTD3) | (1 << PORTD2);

    // SER 595 high
    PORTD |=  (1 << PORTD4);

    // all as inputs
    DDRD &= ~( (1 << PORTD4) | (1 << PORTD3) | (1 << PORTD2) );
}

int main(void)
{
    init();

    start595();
 
    startBoost();

    for(;;);
    
    // TODO consider ADC trigger from TC2 overflow
    return 0;
}

/* 
 * Boost converter should sit at maximum allowed duty until duty needs
 * decreasing.
 *
 * ISR doesn't really need all 10 bits, but the overhead helps to
 * ensure the ISR takes slightly longer.
 *
 * V_boost = ADC/(TOP_TC0 + 1)*1.1V*(1kohm + 47kohm)/(1kohm)
 *
 * Must check voltage AND duty! 
 */
ISR(ADC_vect)
{
    const uint16_t adc = ADC;

    if(adc < 800){      // if under maximum allowable voltage
        if((OCR0B > 9) && (adc < 700))
            OCR0B--;
        else if((OCR0B < TOP_TC0) && (adc > 700))
            OCR0B++;
    }
    else                // otherwise
        OCR0B = TOP_TC0; // set duty 0% this round

    ADCSRA |= (1 << ADSC); // start next sample
}

