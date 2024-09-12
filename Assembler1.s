
/*
 * Assembler1.s
 *
 * Created: 4/25/2024 3:02:45 PM
 *  Author: kmanz
 */ 


.section ".data"					//define a section called .data for constants stored in FLASH
.equ	DDRB,0x04					//set Port B Data Direction Register with address offset
.equ	DDRD,0x0A					//set Port D Data Direction Register with address offset
.equ	PORTB,0x05					//set Port B Data Register. When addressing as I/O Register: address offset is 0x05
.equ	PORTD,0x0B					//setPort D Data Register. When addressing as I/O Register: address offset is 0x0B
.equ	U2X0,1						//set Double Speed Operation to 1 to double transfer rate
.equ	UBRR0L,0xC4					//set USART Baud Rate n Register Low byte with address offset 
.equ	UBRR0H,0xC5					//set USART Baud Rate n Register High byte with address
.equ	UCSR0A,0xC0					//set USART Control and Status Register n A with address offset
.equ	UCSR0B,0xC1					//set USART Control and Status Register n B with address offset
.equ	UCSR0C,0xC2					//set USART Control and Status Register n C with address offset
.equ	UDR0,0xC6					//set USART I/O Data Register n with address offset
.equ	RXC0,0x07					//initialize receive complete flag
.equ	UDRE0,0x05					//set Data Register Empty (UDRE) flag. indicates whether the transmit buffer is ready to receive new data
.equ	ADCSRA,0x7A					//set ADC Control and Status Register A with address offset
.equ	ADMUX,0x7C					//set ADC Multiplexer Selection Register with address offset
.equ	ADCSRB,0x7B					//set ADC Control and Status Register B with address offset
.equ	DIDR0,0x7E					//set Digital Input Disable Register 0 with address offset
.equ	DIDR1,0x7F					//set Digital Input Disable Register 1 with address offset
.equ	ADSC,6						//set ADC Start Conversion
.equ	ADIF,4						//set ADC Interrupt Flag
.equ	ADCL,0x78					//set ADC Data Register Low 
.equ	ADCH,0x79					//set ADC Data Register High
.equ	EECR,0x1F					//set EEPROM Control Register
.equ	EEDR,0x20					//set EEPROM Data Register
.equ	EEARL,0x21					//setEEPROM Address Register Low
.equ	EEARH,0x22					//set EEPROM Address Register High
.equ	EERE,0						//Disable EEPROM Read
.equ	EEPE,1						//Enable EEPROM Write Enable. Need EEMPE to be 1 to write
.equ	EEMPE,2						//Enable EEPROM Master Write Enable. Needs to be enabled for EEPE to work
.equ	EERIE,3						//Set Enable EEPROM Ready Interrupt Flag
.equ TEAM_NAME_ADDR, 0x100 

.global HADC				//student comment here
.global LADC				//student comment here
.global ASCII				//student comment here
.global DATA				//student comment here

.set	temp,0				//student comment here

.section ".text"			//student comment here
.global Mega328P_Init
Mega328P_Init:
		ldi	r16,0x07		;PB0(R*W),PB1(RS),PB2(E) as fixed outputs
		out	DDRB,r16		//configuring the pins of port B for output
		//ldi	r16,0			//student comment here
		//out	PORTB,r16		//set output pins to low
		//out	U2X0,r16		;initialize UART, 8bits, no parity, 1 stop, 9600
		//ldi	r17,0x0			//student comment here
		//ldi	r16,0x67		//student comment here
		//sts	UBRR0H,r17		//store 0 as the high byte value for the baud rate
		//sts	UBRR0L,r16		//store 0x67 as the low byte value for the baud rate 
		//ldi	r16,24			//student comment here
		//sts	UCSR0B,r16		//transmitter and receiver enable
		//ldi	r16,6			//student comment here
		//sts	UCSR0C,r16		//2-bit stop bit and 
		//ldi r16,0x87		//initialize ADC
		//sts	ADCSRA,r16      //student comment here
		//ldi r16,0x40		//student comment here
		//sts ADMUX,r16		//student comment here
		//ldi r16,0			//student comment here
		//sts ADCSRB,r16		//student comment here
		//ldi r16,0xFE		//student comment here
		//sts DIDR0,r16		//student comment here
		//ldi r16,0xFF		//student comment here
		//sts DIDR1,r16		//student comment here
		ret					//student comment here
	
.global LCD_Write_Command
LCD_Write_Command:
	call	UART_Off		//disable
	call	LCD_Delay		;delay
	ldi		r16,0xFF		;PD0 - PD7 as outputs
	out		DDRD,r16		//turn on all bits and output to register D
	lds		r16,DATA		//load direct from data space DATA into register 16
	out		PORTD,r16		//output data to port d
	call	LCD_Delay		//delay
	ldi		r16,4			//load 4 into register 16 to set bit 2
	out		PORTB,r16		//output 4 to port b setting bit 2 on
	call	LCD_Delay		//delay
	ldi		r16,0			//load 0 into register 16 to reset
	out		PORTB,r16		//output 0 to port b to reset
	call	LCD_Delay		//delay
	call	UART_On			//enable
	ret						//return to lcd

.global LCD_Delay
LCD_Delay:
	ldi		r16,0xFA		//load 0xFA, or 250 into register 16
D0:	ldi		r17,0xFF		//load 0xFF, or 255, into register 17
D1:	dec		r17				//decrement 1 from register 17 count
	brne	D1				//branch to D1 until it is not positive
	dec		r16				//decrement 1 from register 16 count
	brne	D0				//branch back to outer loop D0
	ret						//return to the lcd write command used above

.global LCD_Write_Data
LCD_Write_Data:
	call	UART_Off		//disable
	ldi		r16,0xFF		//load 0xFF into register 16 to turn all bits on
	out		DDRD,r16		//output all bits in register d
	lds		r16,DATA		//load direct from data space DATA into register 16
	out		PORTD,r16		//output DATA to port d which outputs to the LCD
	ldi		r16,6			//load 6 into register 16 to set bit 1 and 2 on
	out		PORTB,r16		//output 6 to port b setting bit 1 and 2 on
	call	LCD_Delay		//delay
	ldi		r16,0			//load 0 into register 16
	out		PORTB,r16		//output 0 to port b to reset
	out		PORTD,r16		//output 0 to port d to reset
	call	UART_On			//enable
	ret						//return                    

.global LCD_Read_Data
LCD_Read_Data:
	call	UART_Off		//disable
	ldi		r16,0x00		//load 0x00 into register 16 to reset
	out		DDRD,r16		//output to register d
	out		PORTB,4			//output 4 to port b turning bit 2 on
	in		r16,PORTD		//load I/O location port d to register 16
	sts		DATA,r16		//store direct to data space and write DATA to register 16
	out		PORTB,0			//output 0  to port b to reset it
	call	UART_On			//enable
	ret						//return

.global UART_On
UART_On:
	ldi		r16,2				//set register 16 value to 2 for DDRD1 bit
	out		DDRD,r16			//use the DDRD1 pin for output
	ldi		r16,24				//set register 16 value to 24 (for transmit and receive bits)
	sts		UCSR0B,r16			//enable transmit and receive bits
	ret							//end subroutine and turn UART on

.global UART_Off
UART_Off:
	ldi	r16,0					//set register 16 value to zero
	sts UCSR0B,r16				//disble all bits including transmit and receive bits
	ret							//end subroutine and turn UART off

.global UART_Clear
UART_Clear:
	lds		r16,UCSR0A			//student comment here
	sbrs	r16,RXC0			//student comment here
	ret							//student comment here
	lds		r16,UDR0			//student comment here
	rjmp	UART_Clear			//student comment here

.global UART_Get
UART_Get:
	lds		r16,UCSR0A			//student comment here
	sbrs	r16,RXC0			//student comment here
	rjmp	UART_Get			//student comment here
	lds		r16,UDR0			//student comment here
	sts		ASCII,r16			//student comment here
	ret							//student comment here

.global UART_Get2
UART_Get2:
    //lds     r16, UCSR0A  
    //sbrc    r16, RXC0   
    //rjmp    skip   
    lds     r16, UDR0  
    sts     ASCII, r16  
//skip:
    //ret

/*
.global UART_Edit
UART_Edit:
    call	UART_On
    ldi		r17,0x0          //student comment here
    ldi		r16,0xCF         //student comment here
    sts		UBRR0H,r17        //store 0 as the high byte value for the baud rate
    sts		UBRR0L,r16        //store 0x65 as the low byte value for the baud rate
	ldi		r16, 1
	out		U2X0,r16		;initialize UART, 8bits, no parity, 1 stop, 9600
	ldi		r16,41			//odd parity, 2-bit stop bit, 7-bit data bit
	sts		UCSR0C,r16		//1-bit stop bit and
    //out		UBRR0H, r17
    //out		UBRR0L, r16
    ret
*/
.global UART_Put
UART_Put:
	lds		r17,UCSR0A			//student comment here
	sbrs	r17,UDRE0			//student comment here
	rjmp	UART_Put			//student comment here
	lds		r16,ASCII			//student comment here
	sts		UDR0,r16			//student comment here
	ret							//student comment here