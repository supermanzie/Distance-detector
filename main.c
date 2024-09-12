/*
 * Final1.c
 *
 * Created: 4/25/2024 3:00:20 PM
 * Author : kmanz
 */ 
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>

#define F_CPU 16000000UL // CPU frequency
#define TRIG_PIN 0        // PC0 for trigger
#define ECHO_PIN 1        // PC1 for echo
#define RED_PIN PC4
#define GREEN_PIN PC3
#define THRESHOLD 20

void LCD_Init(void);
void UART_Init(void);
void UART_Clear(void);
void UART_Get(void);
void UART_Put(void);
void LCD_Write_Data(void);
void UART_Edit(void);
void LCD_Write_Command(void);
void LCD_Read_Data(void);
void LCD_Delay(void);
//void UART_Edit(void);
void Mega328P_Init(void);

unsigned char ASCII;
unsigned char DATA;

static volatile int pulse = 0;
static volatile int i = 0;
int echoFlag;

void UART_init(unsigned int ubrr) {
	UBRR0H = (unsigned char)(ubrr >> 8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8 data bits, 1 stop bit
}


void init() {
	DDRC |= (1<<TRIG_PIN);
	PORTC = 0x00; //set pins low
	DDRC &= ~(1 << ECHO_PIN); // Set echoPin as input
	EIMSK |= (1 << INT0); // Enable interrupt INT0
	EICRA |= (1 << ISC00); // Interrupt on rising and falling edge
}
/*
void delayFunction_Timer0(int a) {
	OCR0A = a;
	TCNT0 = 0x00;
	TCCR0A |= (1 << WGM01); // Set Timer0 to CTC mode
	TCCR0B |= (1 << CS00); // No pre-scaler
	while ((TIFR0 & (1 << OCF0A)) == 0) {} // Wait for OCF0 value
	TCCR0B = 0x00; // Stop clock
	TIFR0 = (1 << OCF0A); // Clear flag
}

void signalPulse() {
	PORTC |= (1 << TRIG_PIN); // Set trigPin high
	delayFunction_Timer0(0x9F); // 0x9F = 159 (10?s)
	PORTC &= ~(1 << TRIG_PIN); // Set trigPin low
}
*/
void LCD_Puts(const char *str)	//Display a string on the LCD Module
{
	while (*str)
	{
		DATA = *str++;
		LCD_Write_Data();
	}
}

void UART_init(unsigned int ubrr) {
	UBRR0H = (unsigned char)(ubrr >> 8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8 data bits, 1 stop bit
}

void UART_transmit(unsigned char data) {
	while (!(UCSR0A & (1 << UDRE0))); // Wait for empty transmit buffer
	UDR0 = data; // Put data into buffer, sends the data
}

unsigned char UART_receive() {
	while (!(UCSR0A & (1 << RXC0))); // Wait for data to be received
	return UDR0; // Get and return received data from buffer
}


void UART_Puts(const char *str)	//Display a string in the PC Terminal Program
{
	while (*str)
	{
		ASCII = *str++;
		UART_Put();
	}
}


ISR(INT0_vect) {
	if (echoFlag == 0) {
		TCCR1B |= 1 << CS10; // Record pulse start time
		echoFlag = 1;
		} else {
		TCCR1B = 0; // Record pulse end time
		pulse = TCNT1;
		TCNT1 = 0; // Calculate pulse duration
		echoFlag = 0;
	}
}

int main() {
	init(); // Initialize sensor and interrupt
	//UART_init(); // Initialize USART (optional)

	sei(); // Enable global interrupts

	while (1) {
		PORTC |= (1 << TRIG_PIN); // Set trigPin high
		_delay_us(15);
		PORTC &= ~(1 << TRIG_PIN); // Set trigPin low
		int distance = pulse/58;
		if (distance > THRESHOLD) {
			// Set green (PC3) and turn off red (PC4)
			PORTC |= (1 << GREEN_PIN);
			PORTC &= ~(1 << RED_PIN);
			} else {
			// Set red (PC4) and turn off green (PC3)
			PORTC |= (1 << RED_PIN);
			PORTC &= ~(1 << GREEN_PIN);
		}
		// Transmit distance over UART
		char buffer[10];
		sprintf(buffer, "%d cm\n", distance);
		
		for (int i = 0; buffer[i] != '\0'; i++) {
			UART_transmit(buffer[i]);
		}
		
		
		DATA = 0x01;					//clear mode
		LCD_Write_Command();
		DATA = 0x34;					//set mode
		LCD_Write_Command();
		DATA = 0x08;					//display on or off
		LCD_Write_Command();
		DATA = 0x02;					//home
		LCD_Write_Command();
		DATA = 0x06;					//set entry mode
		LCD_Write_Command();
		DATA = 0x0f;					//shift cursor or display
		LCD_Write_Command();
		LCD_Puts("Distance: ");
		LCD_Puts(buffer);
		_delay_ms(50);

		// Process the echo measurement (microSecPulse)
		// Display or use the distance data as needed
	}

	return 0;
}
/*
ISR(TIMER1_CAPT_vect) {
	static uint16_t start_time = 0;
	static uint8_t state = 0;

	if (state == 0) {
		start_time = ICR1; // Store the time of the rising edge
		TCCR1B |= (1 << ICES1); // Next interrupt on falling edge
		state = 1;
		} else {
		pulse_width = ICR1 - start_time; // Calculate pulse width
		TCCR1B &= ~(1 << ICES1); // Next interrupt on rising edge
		state = 0;
	}
}

void ultrasonic_init() {
	// Set trigger pin as output
	DDRC |= (1 << TRIG_PIN);
	
	// Set echo pin as input
	DDRC &= ~(1 << ECHO_PIN);

	// Set Timer 1 for input capture
	TCCR1B |= (1 << ICNC1); // Enable noise canceler
	TIMSK1 |= (1 << ICIE1); // Enable input capture interrupt
	TCCR1B |= (1 << CS11); // Prescaler 8
}

uint16_t measure_distance() {
	// Trigger pulse
	PORTC |= (1 << TRIG_PIN); // Set trigger pin high
	_delay_us(10); // Wait for 10 microseconds
	PORTC &= ~(1 << TRIG_PIN); // Set trigger pin low

	// Wait for echo
	while (!(TIFR1 & (1 << ICF1))); // Wait for capture interrupt flag
	TIFR1 |= (1 << ICF1); // Clear capture interrupt flag

	// Calculate distance
	uint32_t distance = (pulse_width * 10) / 58; // Convert pulse width to distance in cm

	return distance;
}

int main() {
	// Initialize
	ultrasonic_init();
	sei(); // Enable global interrupts

	// Main loop
	while (1) {
		uint16_t distance1 = measure_distance();
		long distance = distance1;
		if (distance > THRESHOLD) {
			// Set green (PC3) and turn off red (PC4)
			PORTC |= (1 << GREEN_PIN);
			PORTC &= ~(1 << RED_PIN);
			} else {
			// Set red (PC4) and turn off green (PC3)
			PORTC |= (1 << RED_PIN);
			PORTC &= ~(1 << GREEN_PIN);
		}
		// Do something with the distance measurement, like sending it over UART or displaying it
		_delay_ms(100); // Delay between measurements
	}

	return 0;
}
*/