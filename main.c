/*
 * Help with timers thanks to Dr. Erik Petrich @ Univ of Oklahoma
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <time.h>
#include <util/usa_dst.h> //TODO rewrite
#include <util/delay.h>

#define TOP_TC0 99
#define TOP_TC2 155

#define NIXIE_TUBE_COUNT 8
#define NIXIE_ON_TIME_US 500
#define NIXIE_DEAD_TIME_US 500

static void init();

static void startADC5();
static void stopADC5();

static void startTC0();
static void stopTC0();

static void startTC1();
static void stopTC1();

static void startTC2();
static void stopTC2();

static void start595();
static void output595(__uint24 output);

static void startBoost();
static void stopBoost();

static volatile uint8_t timeAvailable;
static time_t timeKeeper;
static volatile struct tm* timeKeeperStruct;
static volatile uint8_t centiSeconds;
static void startRTC();
static void stopRTC();

static void init()
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

static void startADC5()
{
    // TRI-STATE ADC PIN FOR HIGH IMPEDANCE READING, NO PULLUP
    PORTC &= ~(1 << PORTC5);

    // disable digital input on ADC5 (PC5)
    DIDR0 |=  (1 << ADC5D);

    // reference internal 1.1V
    ADMUX |=  (1 << REFS1);
    ADMUX |=  (1 << REFS0); // REFS = 0b11

    // right adjust conversion result, ADCH = 10:8
    ADMUX &= ~(1 << ADLAR);

    // select ADC5 as input channel
    ADMUX &= ~(1 << MUX3);
    ADMUX |=  (1 << MUX2);
    ADMUX &= ~(1 << MUX1);
    ADMUX |=  (1 << MUX0); // MUX = 0b0101

    // set ADC prescaler to 16MHz/128 = 125kHz
    ADCSRA |=  (1 << ADPS2);
    ADCSRA |=  (1 << ADPS1);
    ADCSRA |=  (1 << ADPS0); // ADPS = 0b111

    // enable ADC interrupts
    ADCSRA |=  (1 << ADIE);

    // ADC Auto Trigger Source, "Timer/Counter 0 Compare Match A"
    ADCSRB &= ~(1 << ADTS2);
    ADCSRB |=  (1 << ADTS1);
    ADCSRB |=  (1 << ADTS0); // ADTS = 0b011

    // enable ADC auto triggering
    ADCSRA |=  (1 << ADATE);

    // enable the ADC
    ADCSRA |=  (1 << ADEN);
}

static void stopADC5()
{
    // disable ADC interrupts
    ADCSRA &= ~(1 << ADIE);

    // disable the ADC
    ADCSRA &= ~(1 << ADEN);

    // disable ADC auto triggering
    ADCSRA &= ~(1 << ADATE);

    // default ADC Auto Trigger Source, "Free Running Mode"
    ADCSRB &= ~(1 << ADTS2);
    ADCSRB &= ~(1 << ADTS1);
    ADCSRB &= ~(1 << ADTS0);

    // reset ADC prescaler to 16MHz/2
    ADCSRA &= ~(1 << ADPS2);
    ADCSRA &= ~(1 << ADPS1);
    ADCSRA &= ~(1 << ADPS0); // ADPS = 0b000
 
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

    // reenable digital input on ADC5 (PC5)
    DIDR0 &= ~(1 << ADC5D);
}

/*
 * Timer/Counter 0 is using PWM on pin OC0B (PD5) to drive the gate of
 * an N-channel MOSFET in a switched-mode power supply.
 *
 * From TCNT0 = [0, OCR0B - 1], OC0B drives low.
 * From TCNT0 = [OCR0B, TOP_TC0], OC0B drives high.
 *
 * The frequency is set by the value in OCR0A and the prescaler.
 * The duty cycle is set by the value in OCR0B.
 *
 * The duty cycle (OCR0B) is adjusted by the ADC ISR once per TC0
 * period.
 */
static void startTC0()
{
    // set PWM pin OC0B tri-state
    PORTD &= ~(1 << PORTD5);
    
    // set PWM pin OC0B as input during config
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

static void stopTC0()
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

    // default timer registers
    OCR0A = 0x00;
    OCR0B = 0x00;
    TCNT0 = 0x00;
}

/*
 *
 */
static void startTC1()
{
    // set PWM pins tri-state, OC1A (PB1) and OC1B (PB2)
    PORTB &= ~(1 << PORTB1);
    PORTB &= ~(1 << PORTB2);
    
    // set PWM pins as inputs, OC1A (PB1) and OC1B (PB2)
    DDRB &= ~(1 << DDB1);

    // clear
    OCR1A = 0x60;

    // set
    OCR1B = 0xA0;

    // Wave Generation Mode, "PCPWM, 8-bit"
    TCCR1B &= ~(1 << WGM13);
    TCCR1B &= ~(1 << WGM12); 
    TCCR1A &= ~(1 << WGM11);
    TCCR1A |=  (1 << WGM10); // WGM1 = 0b0001;

    // Comp Output Mode - Chan A, "Clear OC1A on Compare Match"
    TCCR1A |=  (1 << COM1A1);
    TCCR1A &= ~(1 << COM1A0); // COM1A = 0b10

    // Comp Output Mode - Chan B, "Set OC1B on Compare Match"
    TCCR1A |=  (1 << COM1B1);
    TCCR1A |=  (1 << COM1B0); // COM1B = 0b11

    // start timer, prescaler F_osc/8
    TCCR1B &= ~(1 << CS12); 
    TCCR1B &= ~(1 << CS11); // datasheet entry missing for CS11?
    TCCR1B |=  (1 << CS10); // CS1 = 0b010

    // unleash the waveform, set PWM pins OC1A and OC1B as output
    DDRB |=  (1 << DDB1);
    DDRB |=  (1 << DDB2);
}

static void stopTC1()
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

    // default timer registers
    OCR1A = 0x0000;
    OCR1B = 0x0000;
    TCNT1 = 0x0000;
}

static void startTC2()
{
    /* CONFIGURE INTERRUPTS */

    // TC2 Interrupt Mask Register, "ISR on Compare Match A (OCR2A)"
    TIMSK2 &= ~(1 << OCIE2B);
    TIMSK2 |=  (1 << OCIE2A);
    TIMSK2 &= ~(1 << TOIE2); // TIMSK2 = 0b010

    /* CONFIGURE TIMER/COUNTER 2 */

    // Wave Generation Mode, "CTC" BOTTOM to TOP_TC2 (OCR2A)
    TCCR2B &= ~(1 << WGM22);
    TCCR2A |=  (1 << WGM21);
    TCCR2A &= ~(1 << WGM20); // WGM0 = 0b010

    // Comp Output Mode - Chan A, "Normal Port Op, 0C2A discon"
    TCCR2A &= ~(1 << COM2A1);
    TCCR2A &= ~(1 << COM2A0); // COM0A = 0b00

    // Comp Output Mode - Chan B, "Normal Port Op, OC2B discon"
    TCCR2A &= ~(1 << COM2B1);
    TCCR2A &= ~(1 << COM2B0); // COM0B = 0b00

    // TOP_TC2 value, sets period 100.1Hz (assuming 1024 prescaler)
    OCR2A = TOP_TC2;

    // start timer, prescaler F_osc/1024
    TCCR2B |=  (1 << CS22);
    TCCR2B |=  (1 << CS21);
    TCCR2B |=  (1 << CS20); // CS2 = 0b111
}

static void stopTC2()
{
    /* DISABLE INTERRUPTS */

    // turn off TC2 ISRs
    TIMSK2 &= ~(1 << OCIE2B);
    TIMSK2 &= ~(1 << OCIE2A);
    TIMSK2 &= ~(1 << TOIE2); // TIMSK2 = 0b000

    /* DEFAULT SETTINGS TIMER/COUNTER 2 */

    // stop timer, disable clock source
    TCCR2B |=  (1 << CS22);
    TCCR2B |=  (1 << CS21);
    TCCR2B |=  (1 << CS20); // CS2 = 0b111

    // Wave Generation Mode, "Normal"
    TCCR2B &= ~(1 << WGM22);
    TCCR2A |=  (1 << WGM21);
    TCCR2A &= ~(1 << WGM20); // WGM0 = 0b000

    // Comp Output Mode - Chan A, "Normal Port Op, 0C2A discon"
    TCCR2A &= ~(1 << COM2A1);
    TCCR2A &= ~(1 << COM2A0); // COM0A = 0b00

    // Comp Output Mode - Chan B, "Normal Port Op, OC2B discon"
    TCCR2A &= ~(1 << COM2B1);
    TCCR2A &= ~(1 << COM2B0); // COM0B = 0b00

    // default "Output Compare Register" Values
    OCR2A = 0x00;
    OCR2B = 0x00;
    TCNT2 = 0x00;
}

static void start595()
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
    output595(0x00000);
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
static void output595(__uint24 output)
{
    PORTD &= ~(1 << PORTD3); // bring SCLK 595 low
    
    for(uint8_t i = 20; i; --i) // each bit in the vector
    {
        // SER 595 data to be shifted in
        PORTD ^= ((~(output & 1) + 1) ^ PORTD) & (1 << PORTD4);

        PORTD |= (1 << PORTD2); // SRCLK 595 high, shift in SER data
        output >>= 1; // move next bit, introduce clock delay
        PORTD &= ~(1 << PORTD2); // SRCLK 595 low
    }

    PORTD |= (1 << PORTD3); // bits to storage register, SCLK 595 high
}

static void stop595()
{
    // zero out shift register
    output595(0x00000);

    // SER 595 low
    PORTD &=  ~(1 << PORTD4);

    // SRCLK and SCLK 595 high
    PORTD |=  (1 << PORTD3) | (1 << PORTD2);

    // SER 595 high
    PORTD |=  (1 << PORTD4);

    // all as inputs
    DDRD &= ~( (1 << PORTD4) | (1 << PORTD3) | (1 << PORTD2) );
}

static void startBoost()
{
    startADC5(); // start monitoring boost outut voltage
    startTC0(); // start pulsing inductor
    _delay_ms(100);
    startTC1(); // start multiplier switching
    _delay_ms(100);
}

static void stopBoost()
{
    stopTC0(); // stop pulsing inductor
    stopTC1(); // stop multiplier switching
    stopADC5(); // stop monitoring boost output voltage
}

static void startRTC()
{
    set_system_time(0);
    //set_zone(-6 * ONE_HOUR);
    //set_dst(usa_dst);

    start595();
    startTC2();
    _delay_ms(3000);
}

static void stopRTC()
{
    stopTC2();
    stop595();
}

/*
 * Variable uint8_t nixie[8], array index refers to an
 * individual tube, value at index refers to the number displayed on
 * that tube.
 *
 * Nixie tube 0: hours MSB
 * Nixie tube 1: hours LSB
 * Nixie tube 2: minutes MSB
 * Nixie tube 3: minutes LSB
 * Nixie tube 4: seconds MSB
 * Nixie tube 5: seconds LSB
 * Nixie tube 6: milliseconds MSB
 * Nixie tube 7: milliseconds LSB
 */
int main(void)
{
    init();

    startBoost();
    startRTC();

    uint8_t hours = 0, minutes = 0, seconds = 0, _centiSeconds = 0;
    uint8_t nixie[8] = {0};
    for(; ; ){
        if(timeAvailable){
            ATOMIC_BLOCK(ATOMIC_FORCEON){
                timeAvailable = 0;
                hours = (uint8_t)(*timeKeeperStruct).tm_hour;
                minutes = (uint8_t)(*timeKeeperStruct).tm_min;
                seconds = (uint8_t)(*timeKeeperStruct).tm_sec;
            }

            nixie[0] = hours / 10;
            nixie[1] = hours % 10;
            nixie[2] = minutes / 10;
            nixie[3] = minutes % 10;
            nixie[4] = seconds / 10;
            nixie[5] = seconds % 10;
        }

        _centiSeconds = 99 - centiSeconds;

        nixie[6] = _centiSeconds / 10;
        nixie[7] = _centiSeconds % 10;

        for(uint8_t i = 0; i < NIXIE_TUBE_COUNT; ++i){
           output595( (__uint24)1 << (i + 12) |\
                      (__uint24)1 << (nixie[i] + 2) );

#if NIXIE_ON_TIME_US            
            _delay_us(NIXIE_ON_TIME_US);
#endif

#if NIXIE_DEAD_TIME_US
            output595(0x00000);
            _delay_us(NIXIE_DEAD_TIME_US);
#endif
        }
    }

    stopBoost();
    stopRTC();

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

    if(adc < 800){// if under maximum allowable voltage
        if((OCR0B > 9) && (adc < 640)) // if duty lt 90% && V < 36
            OCR0B--;// raise the duty cycle 
        else if((OCR0B < TOP_TC0) && (adc > 640)) // otherwise
            OCR0B++;// lower the duty cycle
    }
    else                    // otherwise
        OCR0B = TOP_TC0;        // set duty 0% this round

    // clear Timer/Counter 0 Output Compare A Match Flag
    TIFR0 |= (OCR0B);
}

/*
 * Every time TC2 "overflows", 1 centisecond has passed, and this
 * ISR is excuted.
 * Once 1 cs have passed 100 times, 1 second has passed.
 * When 1 second passes, reset the cs timer, and update the time vars.
 *
 * Since timeKeeper and timeKeeperStruct* are updated in this ISR,
 * it may be wise to ensure atomic access to the time vars in threads
 * from main.
 */
ISR(TIMER2_COMPA_vect)
{
    if(!--centiSeconds) { // if one second has passed
        timeAvailable = 1;
        centiSeconds = 99; // reset timer to "100"
        system_tick(); // notify time.h that 1 second passed
        time(&timeKeeper); // update timeKeeper with new time
        timeKeeperStruct = localtime(&timeKeeper); // update struct
    }
}

