/*
 * SumoBotProject.c
 *
 * 
 *  Author: Connie Cuevas
 */ 
#define F_CPU 16000000UL   // Sets the clock the crystal on the UNO
#include <avr/io.h>        // Includes the C library for I/O devices
#include <util/delay.h>    // Includes the C library for the _ms_delay() method
#include <avr/interrupt.h> // Includes the C library for interrupts
#include "serial.h"
// Global variables
unsigned int i;
unsigned char sreg;
unsigned int right = 0, left = 0, r_corner = 0, l_corner=0, lcomplete = 0, rcomplete = 0, counter = 0;

/********************************************************************************/
/* [move_forwardA] moves front-left & back-left wheel forwards */
void move_forwardA() {
	// PD3 and PD5 
	PORTD |= 0b00100000;
	PORTD &= 0b11110111;
};

/* [move_backwardA] moves front-left & back-left wheel backwards */
void move_backwardA() {
	// PD3 and PD5
	PORTD |= 0b00001000;
	PORTD &= 0b11011111;
};

/* [move_forwardC] moves front-right & back-right wheel forwards */
void move_forwardC() { 
	// PD6 and PB1
	PORTB |= 0b00000010;
	PORTD &= 0b10111111;
};

/* [move_backwardC] moves front-right & back-right wheel backwards */
void move_backwardC() {
	// PD6 and PB1
	PORTD |= 0b01000000;
	PORTB &= 0b11111101;
};

/* [stop_all] stops all of the wheels from runnig */
void stop_all() {
	PORTB &= 0b11111101;
	PORTD &= 0b10010111;
};

/* [rotate_clockwise] rotates robot to move clockwise */
void rotate_clockwise() {
	move_forwardA();
	move_backwardC();
	//_delay_ms(100);
};

/* [rotate_cclockwise] rotates robot to move counter-clockwise */
void rotate_cclockwise() {
	move_forwardC();
	move_backwardA();
};

/* [ISR INT0_vect] is an interrupt for the qti that will detect 
 * if the robot has crossed the boundary. If it crosses the boundary 
 * it will turn on the LED and stop all motors (PD2) */
ISR(INT0_vect){					
	// turn on LED
	while(1) {
		stop_all();
	}
};

/* [ISR PCINT0_vect] is an interrupt that uses the front-left qti to avoid
 * white lines. It will rotate clockwise until it does not see a white line
 * anymore. (PD4)*/
/*ISR(PCINT0_vect) {
	stop_all(); // Stop wheels
	cli();
	// Rotate until no white line is detected
	while(!(PIND & 0b00010000)) {
		rotate_clockwise();
	}
	sei();
};*/

/* [ISR PCINT2_vect] is an interrupt that uses the front-right qti to avoid
 * white lines. It will rotate counter-clockwise until it does not see a white 
 * line anymore. (PD7)*/
ISR(PCINT2_vect) {
	stop_all();							// Stop wheels
	cli();								// disable interrupts to avoid problems
	if(!(PIND & 0b10000000)){
		while(!(PIND & 0b10000000)) {
			rotate_cclockwise();		// Rotate until no white line is detected
		}
	}
	if(!(PIND & 0b00010000)){
		while(!(PIND & 0b00010000)) {
			rotate_clockwise();			// Rotate until no white line is detected
		}
	}
	sei();								// Re-enable interrupts and continue
};


// Initializes the timer and interrupt
void initSonar() {
// 	PCICR |= 0X04;		  // enables interrupt
// 	PCMSK2 |= 0X04;		  // enables PD3
	TCCR1B |= 0b00000011; //prescaler 64
	sei();				  // init interrupt
};

// Trigger pulse
void startSonarMeasurement() {
	PCMSK0 &= 0b11000111;  // disable PB3, PB4, PB5
	PCMSK1 &= 0x11111110;  // disable PC0
	
	DDRB |= 0b00111000;		// PB3,PB4,PB5 is output
	DDRC |= 0x01;			// PC0 is output
	
	PORTB |= 0b00111000;	// make values high
	PORTC |= 0x01;
	_delay_us(5);
	PORTB &= 0b11000111;	//make values low
	PORTC &= 0b11111110;
	
	DDRB &= 0b11000111;		// make PB3,PB4, PB5 input
	DDRC &= 0b11111110;     // make PC0 input
	PCMSK2 |= 0b00100000;
};

/*ISR resets the timer when the rising edge is detected, and reads
the timer and disables the interrupt when the falling edge is detected*/
ISR(PCINT0_vect){
	if (PINB & 0b00100000) { // PB3, PB4, PB5, PC0
		sreg = SREG;
		cli();
		TCNT1 = 0;
		SREG = sreg;
	}
	else {
	    sreg = SREG;
		cli();
		i = TCNT1;
		SREG = sreg;
	}
};

//calls startSonarMeasurement and calculates the range in inches to the closest object
float getSonar() {
	startSonarMeasurement();
	_delay_ms(19);
	float dist = i * .02712;
	return dist;
};

void poll_sonars () { // this will be in while loop
	init_uart();
	_delay_ms(1000);
	float range = getSonar();
	while(range > 36) {
		rotate_clockwise();
	}

	// printf("Distance: %u   [inches]\r\n", (uint16_t)range);
	//TODO:: Please write your application code */
};

void init_interrupts () {
	// organize all interrupt intialization here
}

void set_pins() {
	// set input and output pins here
}

void set_ports() {
	// set port values
}

void make_slow () {
	OCR1A = 0b0000010;
	OCR1B = 0b0000000;
	
	OCR2A = 0b00000010;
	TCNT1 = 0;
}

void make_fast() {
	OCR1A = 0xFF;
}

void Freq() {
		// PWM - Phase and Frequency correct - mode 8
		TCCR1A |= 0b00110000;	//setting mode 8 and enabling inverted PWM on OC1B
		TCCR1B |= 0b00010001;	//setting mode 8 and a prescaler of 1
		
		ICR1 = 0xFFFF;			// defining TOP for the counter - changes the frequency of the PWM
		
		DDRB |= 0b00000100;		// defining PB2, which is OC1B, as an output
		// "Above" the value of OCR1B the pwm output is high, "below" it is low
		OCR1B = 0x0000;
		_delay_ms(1000);
		OCR1B = 0xF000;
		_delay_ms(1000);
		OCR1B = 0xFFFF;
		_delay_ms(1000);
}  //modify phase and frequency
	
	
int main(void)
{
	//enable external interrupt: using PCI0
	EICRA |= 0x01;
	EIMSK |= 0x01;
	
	// enable: PIN change interrupt
	PCICR |= 0x07;		 // enables interrupts PCIE0, PCIE1,PCIE2 0b00000111;
	
	// enables interrupt by masking pins
	PCMSK2 |= 0b10010100; // QTI: FR, FL, MD
	PCMSK1 |= 0x01;		  // Sonar: BL
	PCMSK0 |= 0b00111101; // Sonar: BR, FR, FL; QTI: BL, BR
	
	// set pin as output for motors (PD3,PD5,PD6,PB1) and LED
	DDRD |= 0b01101000; // Set PD3,PD5,PD6 to output
	DDRB |= 0b00000010; // Set PB1 to output

	// set pin as input for QTI sensors (PD2, PD4, PD7, PB0, PB2)
	DDRD &= 0b01101011; // Set PD2, PD4, PD7 as inputs 
	DDRB &= 0b11111010; // Set PB0, PB2 as inputs
	
	// set up Port values for motors and QTI's 
	PORTB &= 0x00;
	PORTD &= 0x00;
	
	// create timer 
// 	TCCR1B |= 0b00010011;	// setting mode to normal and prescaler to 256
// 	TCCR1A |= 0b00010011;
	
	// enable interrupts
	sei();
	
	// init sonar
	init_uart();
	//initSonar();
	
	_delay_ms(2000);
	
    while(1) {
			move_forwardA();
 			move_forwardC();
// 			poll_sonars();
	}
}