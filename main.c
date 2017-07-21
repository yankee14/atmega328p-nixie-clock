#include <avr/io.h>
#include <avr/interrupt.h>

#define TOP 255
#define TOGGLE TOP-225

void init();

void startADC0();
void stopADC0();

void startTC0();
void stopTC0();

void startTC1();
void stopTC1();

void startBMPS();
void stopBMPS();

void init()
{
    // MASTER tri-state all pins during init, protects inductor
    MCUCR |= (1 << PUD); 

    // set all pins as inputs
    DDRB = DDRD = 0x00;
    DDRC &= 0x80;

    // turn on all pins pullups,
    PORTB = PORTD = 0xFF;
    PORTC |= ~(0x80);

    // EXCEPT for OC0A (PD6), OC1A (PB1), tri-state, protects inductor
    PORTB &= ~(1 << PORTB1);
    PORTD &= ~(1 << PORTD6);

    // globally enable interrupts in SREG
    sei();

    // master reenable all pullups
    MCUCR &= ~(1 << PUD);
}

void startADC0()
{
    // TRI-STATE ADC PIN FOR HIGH IMPEDANCE READING, NO PULLUP
    PORTC &= ~(1 << PORTC0);

    /* ADC MULTIPLEXER SELECTION REGISTER (ADMUX) */

    // disable digit input on ADC0 (PC0)
    DIDR0 |= (1 << ADC0D);

    // use internal 1.1V reference as reference voltage
    ADMUX |= (1 << REFS1);
    ADMUX |= (1 << REFS0); // REFS = 0b11

    // left adjust conversion result, ADCH = 9:2
    ADMUX |= (1 << ADLAR);

    // select ADC0 as input channel
    ADMUX &= ~(1 << MUX3);
    ADMUX &= ~(1 << MUX2);
    ADMUX &= ~(1 << MUX1);
    ADMUX &= ~(1 << MUX0);

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

    // TRI-STATE ADC PIN FOR HIGH IMPEDANCE READING, NO PULLUP
    PORTC |= (1 << PORTC0);

    // reenable digital input on ADC0 (PC0)
    DIDR0 &= ~(1 << ADC0D);
    
    // reset ADC prescaler to 16MHz/2
    ADCSRA &= ~(1 << ADPS2);
    ADCSRA &= ~(1 << ADPS1);
    ADCSRA &= ~(1 << ADPS0); // ADPS = 0b000
 
    /* ADC MULTIPLEXER SELECTION REGISTER (ADMUX) */
    
    // AREF reference voltage
    ADMUX &= ~(1 << REFS1);
    ADMUX &= ~(1 << REFS0); // REFS = 0b00

    // right adjust conversion result, ADCH = 9:8
    ADMUX &= ~(1 << ADLAR);

    // select ADC0 as input channel
    ADMUX &= ~(1 << MUX3);
    ADMUX &= ~(1 << MUX2);
    ADMUX &= ~(1 << MUX1);
    ADMUX &= ~(1 << MUX0);
}

void startTC0()
{
    // set PWM pin tri-state
    PORTD &= ~(1 << PORTD6);
    
    // set PWM pin as input
    DDRD &= ~(1 << DDD6);

    //what
    OCR0A = 0x01;

    // conf Wave Generation Mode, "Fast PWM" from BOTTOM to TOP (ICR1)
    TCCR0B |= (1 << WGM02); 
    TCCR0A |= (1 << WGM01);
    TCCR0A |= (1 << WGM00); // WGM = 0b111;

    // conf Comp Output Mode, "Clear OC1A on Compare Match"
    TCCR0A &= ~(1 << COM0A1);
    TCCR0A |= (1 << COM0A0); // COM1 = 0b01

    // start timer, no prescaler
    TCCR0B |=  (1 << CS02); 
    TCCR0B &= ~(1 << CS01);
    TCCR0B &= ~(1 << CS00); // CS0 = 0b001

    // unleash the waveform, conf OC0A as output
    DDRD |= (1 << DDD6);
}

void stopTC0()
{
    // set PWM pin tri-state
    PORTD &= ~(1 << PORTD6);
    
    // set PWM pin as input
    DDRB &= ~(1 << DDB6); // PWM output now effectively stopped
    
    // stop timer, disable clock source
    TCCR0B &= ~(1 << CS02);
    TCCR0B &= ~(1 << CS01); // datasheet entry missing for CS11?
    TCCR0B &= ~(1 << CS00); // CS0 = 0b000

    // conf Waveform Generation Mode, Normal
    TCCR0B &= ~(1 << WGM02); 
    TCCR0A &= ~(1 << WGM01);
    TCCR0A &= ~(1 << WGM00); // WGM = 0b000;

    // conf Comp Output Mode, "Normal"
    TCCR0A &= ~(1 << COM0A1);
    TCCR0A &= ~(1 << COM0A0); // COM0A = 0b00
}

void startTC1()
{
    // set PWM pin tri-state
    PORTB &= ~(1 << PORTB1);
    
    // set PWM pin as input
    DDRB &= ~(1 << DDB1);

    // TODO change these two
    // set the period, last number before rolling to 0x0000
    ICR1 = TOP;

    // set the duty cycle, number to set pulse high
    // NOTE duty cycle is backwards, e.g. OCR1A = 0 means 100% duty
    OCR1A = TOP;

    // conf Wave Generation Mode, "Fast PWM" from BOTTOM to TOP (ICR1)
    TCCR1B |= (1 << WGM13);
    TCCR1B |= (1 << WGM12); 
    TCCR1A |= (1 << WGM11);
    TCCR1A &= ~(1 << WGM10); // WGM = 0b1110;

    // conf Comp Output Mode, "Clear OC1A on Compare Match"
    TCCR1A |= (1 << COM1A1);
    TCCR1A |= (1 << COM1A0); // COM1 = 0b10

    // start timer, no prescaler
    TCCR1B &= ~(1 << CS12); 
    //TCCR1B &= ~(1 << CS11); // datasheet entry missing for CS11?
    TCCR1B |= (1 << CS10); // CS0 = 0b001

    // unleash the waveform, conf OC0A as output
    DDRB |= (1 << DDB1);
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

void startBMPS()
{
    startADC0(); // start monitoring boost outut voltage
    startTC1(); // start pulsing inductor
    startTC0(); // start multiplier
}

void stopBMPS()
{
    stopTC1(); // stop pulsing inductor
    stopTC0(); // stop multiplier
    stopADC0(); // stop ADC
}

int main(void)
{
    init();

    startBMPS();

    for(;;);

    stopBMPS();

// TODO consider ADC trigger from TC2 overflow
    return 0;
}

/* Must check voltage AND duty! */
ISR(ADC_vect)
{
    const unsigned char adc = ADCH; // 34 rep 10V

    if(adc > 14){
        if(OCR1A > TOP - 229){ // do not increase duty beyond 90%
            if(adc > 124) // if voltage too high
                OCR1A++; // decrease duty
            else if(adc < 122) // if voltage too low
                OCR1A--; // increase duty
        } else if(OCR1A != 255)
            OCR1A++;
    }
    
    ADCSRA |= (1 << ADSC); // start next sample
}

