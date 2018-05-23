/*
 * PM - 2018
 * Proiect
 * Danaila Larisa-Andreea
 */

#include <avr/io.h>
#include <util/delay.h>
#define F_CPU 16000000UL //Defines clock speed
#define USART_BAUDRATE 9600 //Baudrate for serial comm.
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

char state;
char rot_dir = 'L';
uint8_t speed = 100;

char get_data(void) {

	while (!(UCSR0A & (1 << RXC0)));
	return UDR0; // retrieves byte from serial port (bluetooth module)

}

void forward(void)
{
    state = 'U';
    PORTB |= (1<<PB0) | (1<<PB3);
	PORTB &= ~(1<<PB2) & ~(1<<PB4);
    OCR0A = speed;
    OCR0B = speed;
}

void backward(void)
{
    state = 'D';
	PORTB &= ~(1<<PB0) & ~(1<<PB3);
    PORTB |= (1<<PB2) | (1<<PB4);
    OCR0A = speed;
    OCR0B = speed;
}
void stop(void)
{


    state = 'C';
    PORTB = 0;
    OCR0A = 0;
    OCR0B = 0;

}

void right_turn(void)
{
	PORTB = 0;
    PORTB |= (1<<PB3);
	if (state == 'U') forward();
	else if (state == 'D') backward();
}

void left_turn(void)
{
	PORTB = 0;
    PORTB |= (1<<PB0);

	if (state == 'U') forward();
	if (state == 'D') backward();
}

void left_pirouette(void)
{
    state = 'a';
    rot_dir = 'L';

	PORTB = 0;
	PORTB |= (1<<PB0);

    OCR0A = 254;
    OCR0B = 254;
}

void right_pirouette(void)
{
    state = 'e';
    rot_dir = 'R';

	PORTB = 0;
	PORTB |= (1<<PB3);
    OCR0A = 254;
    OCR0B = 254;
}

void faster(void)
{
    if (OCR0A < 244 && OCR0B < 244)
    {
        speed += 10;
        OCR0A = speed;
        OCR0B = speed;
    }
}

void slower(void)
{
    if (OCR0A > 10 && OCR0B > 10)
    {
        speed -= 10;
        OCR0A = speed;
        OCR0B = speed;
    }
}
void check_ultrasonic(void) //still working on this
{
	// TODO until friday
}

void motoare(void)
{
	DDRB |= (1 << PB0) | (1 << PB4) | (1 << PB2) | (1 << PB3);

	while(1) {

		char received_byte;
	    received_byte = get_data();

	    switch (received_byte) // which ASCII character was received?
	    {

	        case 'U':   forward();
	                    break; // increase PWM duty cycle

	        case 'C':   stop();
	                    break; // break motor by raising both direction inputs, PWM duty cycle 0%

	        case 'D':   backward();
	                    break; // decrease PWM duty cycle

	        case 'L':   left_turn();
	                    break;

	        case 'R':   right_turn();
	                    break;

	        case 'a':   left_pirouette();
	                    break;

	        case 'e':   right_pirouette();
	                    break;

	        case 'f':   faster();
	                    break;

	        case 's':   slower();
	                    break;

	        case 'H':   // I'm still working on this
						if (distanceSensor) distanceSensor = 0;
	                    else distanceSensor = 1;
	                    UDR = 'H';
	                    break;

	        default:    UDR0 = '?';
	                    break; //Character unknown to my routine, discard character

	    }
		check_ultrasonic();
	}

}

void test(void) {
	// activare motor
	DDRB |= (1 << PB0) | (1 << PB4) | (1 << PB2) | (1 << PB3);
	forward();
	_delay_ms(5000);
	backward();
	_delay_ms(1000);
	_delay_ms(1000);
	_delay_ms(1000);
}

void init_timer0(void)
{
	TCCR0A |= (1 << WGM00) | (1 << WGM01) | (1 << COM0A1) | (1 << COM0B1);
	TCCR0B |= (1 << CS00);
	OCR0A = 0;
	OCR0B = 0;
}

void init_usart(void)
{

	//UART
	UCSR0A = 0;
    UCSR0B |= (1 << RXEN0) | (1 << TXEN0); //enable Tx and Rx
    UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01); //8 data bit

    UBRR0L = BAUD_PRESCALE;          //sets
    UBRR0H = (BAUD_PRESCALE >> 8); //baudrate registers
    //UCSRB |= (1 << RXCIE); //Enable USART-interrupt

}

int main(void) {

	init_timer0();
	init_usart();
	motoare();
	//test();
	while (1);

	return 0;
}
