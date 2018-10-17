/*
 * BotController.c
 *
 * Created: 11/29/2016 2:12:13 PM
 *  Author: ConnieC14
 */ 

#define F_CPU 16000000UL   // Sets the clock the crystal on the UNO
#include <avr/io.h>        // Includes the C library for I/O devices
#include <util/delay.h>    // Includes the C library for the _ms_delay() method
#include <avr/interrupt.h> // Includes the C library for interrupts
#include "serial.h"
// Global variables
unsigned int i;
unsigned int j;
unsigned int k;
unsigned char sreg;
unsigned int counter=0;
float range;
float range1;

/********************************************************************************/
/* [move_forwardL] moves front-left & back-left wheel forwards */
void move_forwardL() {
    // PD2 and PD3 
    PORTD |= 0b00000100;
    PORTD &= 0b11110111;
};

/* [move_backwardL] moves front-left & back-left wheel backwards */

void move_backwardL() {
    // PD3 and PD2
    PORTD |= 0b00001000;
    PORTD &= 0b11111011;
};

/* [move_forwardR] moves front-right & back-right wheel forwards */
void move_forwardR() { 
    // PD4 and PD5
    PORTD |= 0b00010000;
    PORTD &= 0b11011111;
};

/* [move_backwardR] moves front-right & back-right wheel backwards */
void move_backwardR() {
    // PD5 and PD4
    PORTD |= 0b00100000;
    PORTD &= 0b11101111;
};

/* [move_backwardR] moves all wheel backwards */
void move_backward() {
	move_backwardL();
	move_backwardR();
};

/* [move_forwardR] moves all wheel forwards */
void move_forward() {
	move_forwardL();
	move_forwardR();
};

/* [stop_all] stops all of the wheels from running */
void stop_all() {
    PORTD &= 0b11000011;
};

/* [rotate_clockwise] rotates robot to move clockwise */
void rotate_clockwise() {
    move_forwardL();
    move_backwardR();
};

/* [rotate_cclockwise] rotates robot to move counter-clockwise */
void rotate_cclockwise() {
    move_forwardR();
    move_backwardL();
};

/********************************************************************************/

/* [ISR PCINT1_vect] is an interrupt for the qti that will detect 
 * if the robot has crossed the boundary. If it crosses the boundary 
 * it will turn on the LED and stop all motors (PC0) */
ISR(PCINT1_vect){  
	cli();
	// PC1, PC2
	if (!(PINC & 0b00000010) || !(PINC & 0b00000100)){
		move_backward();
		_delay_ms(750);
	}
	if (!(PINC & 0b00000001)) {
		// turn on LED that's connected to AND gate
		PORTC |= 0b00000010;
		while(1) {
			stop_all();
		}
	}
	sei();
};

/* [ISR PCINT2_vect] is an interrupt that uses the front-right qti to avoid
 * white lines. It will rotate counter-clockwise until it does not see a white 
 * line anymore. (PD7, PD6)*/
ISR(PCINT2_vect) {
    cli();								   // disable interrupts to avoid problems
	if (!(PIND & 0b10000000) || !(PIND & 0b01000000)) {
		move_forward();
		_delay_ms(750);
	}
    sei();                                // Re-enable interrupts and continue
};

/*ISR resets the timer when the rising edge is detected, and reads
the timer and disables the interrupt when the falling edge is detected*/
ISR(PCINT0_vect){
	// PB2, PB3, PB4, PB5
	if ( (PINB & 0b00010000) || (PINB & 0b00001000) || (PINB & 0b00000100)) {
        sreg = SREG;		// save global interrupt flag
        cli();				// disable interrupts
        TCNT1 = 0;			// set TCNT1 to 0
        SREG = sreg;		// restore global interrupt flag
    }
	if (!(PINB & 0b00000100)) {
		sreg = SREG;
		cli();
		i = TCNT1;
		SREG = sreg;
	}
	
	if (!(PINB & 0b00001000) ) {
		sreg = SREG;
		cli();
		j = TCNT1;
		SREG = sreg;
	}
	
	if (!(PINB & 0b0001000)) {
		sreg = SREG;
		cli();
		k = TCNT1;
		SREG = sreg;
	}
	    else {}
};

/********************************************************************************/
// Trigger pulse
void startSonarMeasurement(int which_sonar) {
	//printf("counter value during switch: %u \n", which_sonar);
		if (which_sonar == 0) {
		// sonar 0
			cli();	// disable PB2
			DDRB |= 0b00000100;     // PB2 is output
			PORTB |= 0b00000100;    // make values high
			_delay_us(5);
			PORTB &= 0b11111011;    // make values low
			DDRB &= 0b11111011;     // make  PB2 input
			sei();   // re-enable pin
		}
		
		if(which_sonar == 1) {
			// sonar 1
			cli();	// disable PB3
			DDRB |= 0b00001000;     // PB3 is output
			PORTB |= 0b00001000;    // make values high
			_delay_us(5);
			PORTB &= 0b11110111;    // make values low
			DDRB &= 0b11110111;     // make  PB3 input
			sei();   // re-enable pin
		}
			
		if (which_sonar == 2) {
			// sonar 2
			cli();	// disable PB4
			DDRB |= 0b00010000;     // PB4 is output
			PORTB |= 0b00010000;    // make values high
			_delay_us(5);
			PORTB &= 0b11101111;    // make values low
			DDRB &= 0b11101111;     // make  PB4 input	
			sei();
		}
		
// 		else if(which_sonar == 3) {
// 		// sonar 3
// 		cli();					// disable PB5
// 		DDRB |= 0b00100000;     // PB5 is output
// 		PORTB |= 0b00100000;    // make values high
// 		_delay_us(5);
// 		PORTB &= 0b11011111;    // make values low
// 		sei();					// make  PB3 input
// 		}
};

//calls startSonarMeasurement and calculates the range in inches to the closest object
float getSonar(int which_sonar) {
    startSonarMeasurement(which_sonar);
	_delay_ms(19);
	int p = 0;
	if (counter == 0) {
		p = i;
	}
	if (counter == 1) {
		p = j;
	}
	if (counter == 2) {
		p = k;
	}
    float dist = p * .02712;
    return dist;
};

void increment_counter() {
	counter ++;
	//printf("Counter incremented, value is: %u \n",(counter % 4));
	counter = (counter % 3); // 4
};

/* TODO: Create the loop where you poll sonars, base it off of a number 1-4 and then based on number check that specific sonar. 
   isr stays the same then make another function that masks the bits based on number 1-4; */
void poll_sonars () { // this will be in while loop
	
	if (counter == 0) {
		range = getSonar(counter);
		printf("range: %u \n",(uint16_t)range);
	}
	if (counter == 1) {
		range = getSonar(counter);
		printf("range: %u \n",(uint16_t)range);
	}
	if (counter == 2) {
		range1 = getSonar(counter);
		printf("range1: %u \n",(uint16_t)range1);
	}
	if ((range > 0.0) && (range < 30.0)) {
		//printf("range0 \n");
		cli();
		move_backward();
		_delay_ms(235);
		sei();
			//}
		}
	if ((range1 > 0.0) && (range1 < 30.0)) {
		//printf("range1 \n");
		cli();
		move_forward();
		_delay_ms(235);
		sei();
		}
		else {
			rotate_clockwise();
		}
};
 
/********************************************************************************/ 
 /* [init_interrupts] initializes all the necessary interrupts */
void init_interrupts () {	    
	// enables PIN change interrupt
	PCICR |= 0x07;         // enables interrupts PCIE0, PCIE1,PCIE2 0b00000111;
	    
	// enables interrupt by masking pins
	//PCMSK2 |= 0b11000000; // QTI: D7 FR, D6 BR
	PCMSK0 |= 0b00111111; // Sonar: BR, FR, FL; QTI: B0 FL, B1 BL
	PCMSK1 |= 0b00001101;		  // QTI: C0 MD
};

/* [initSonar] initializes the timer and interrupt */
void initSonar() {
	//     PCICR |= 0X04;          // enables interrupt
	//     PCMSK2 |= 0X04;         // enables
	TCCR1B |= 0b00000011; //prescaler 64
	init_uart();
	sei();                  // init interrupt
};

/********************************************************************************/    
int main(void)
{
	// initialize all necessary interrupts
	init_interrupts();
	
    // set pin as output for motors (PD2,PD3,PD4,PD5)
    DDRD |= 0b00111100; // Set PD2,PD3,PD4,PD5 to output
	DDRC |= 0b00000010; // Set LED to output

    // set pin as input for QTI sensors (PD6, PD7, PB0, PB1, PC0)
    DDRD &= 0b00111111; // Set PD6, PD7 as inputs 
    //DDRB &= 0b11111100; // Set PB0, PB1 as inputs
    DDRC &= 0b11110010; // Set PC0 as input
	
    // set up Port values for motors and QTI's 
    PORTB &= 0x00;
    PORTD &= 0x00;
    PORTC &= 0x00;
	
    // initialize sonar, uart and interrupts with sei
    initSonar();
    
    _delay_ms(2000);
    rotate_clockwise();
    while(1) {
		poll_sonars();
		increment_counter();
		_delay_ms(100);
    }
	
}​

