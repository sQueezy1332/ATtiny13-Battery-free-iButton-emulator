#pragma once
/*
  iButton_Emulator.c - Battery-free iButton emulator based on ATtiny13 microcontroller.

  Copyright (c) 2022 Dmitry Muravyev. All right reserved.

  MIT License

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#if defined(__AVR__)
#include <util/crc16.h>
#endif
//#define __AVR_TINY__
//#include <setjmp.h>
//jmp_buf env;

#define NOINLINE __attribute__((noinline))
// Constants, types and definitions
#define timeslot_divider (1000000.0 / (F_CPU / 64))        // Period timer clock cycles im microseconds
#define BUTTON_PIN PB0			//(5)
#define BATTERY_PIN PB4			//(3)
#define LED_PIN PB3				//(2)
#define READ_CMD 0x33	
#define SEARCH_CMD 0xF0	
#define ALARM_CMD 0xEC
#define SKIP_CMD 0xCC
#define EEPROM_SIZE 64			// EEPROM capacity
#define REPEAT 3

#ifdef __AVR_ATmega328P__
#define PINREG PIND
#define INTREG EIMSK
#define IFREG EIFR
#define WDTREG WDTCSR
#ifndef WDTIE
#define WDTIE WDIE
#endif // !WDTIE
#define MASTER_PIN PD3			//(3)
#define ONEWIRE_PIN PD2			//(2)	//INT0
#else
#define PINREG PINB
#define INTREG GIMSK
#define IFREG GIFR
#define WDTREG WDTCR
#define MASTER_PIN PB2			//(7)
#define ONEWIRE_PIN PB1			//(6)	//INT0
#endif // __AVR_ATmega328P__


static byte dataBytes[8]{0x1};
static byte repeats_init = 0;
//static byte error = 0;


////* Interrupts*/
EMPTY_INTERRUPT(INT0_vect);                     // We can save two more bytes if we use assembler and the iret command, but I haven't found an easy way to do this.
ISR(WDT_vect, ISR_ALIASOF(INT0_vect));
ISR(PCINT0_vect, ISR_ALIASOF(INT0_vect));
ISR(TIM0_OVF_vect, ISR_ALIASOF(INT0_vect));
ISR(EE_RDY_vect, ISR_ALIASOF(INT0_vect));
ISR(ANA_COMP_vect, ISR_ALIASOF(INT0_vect));
ISR(TIM0_COMPA_vect, ISR_ALIASOF(INT0_vect));
ISR(TIM0_COMPB_vect, ISR_ALIASOF(INT0_vect));
ISR(ADC_vect, ISR_ALIASOF(INT0_vect));


// Implementation

// Delay and sleep functions
// Delay for microseconds
// Since the only timer is busy by controlling timeslots - we cannot use it directly, so we will use workaround based on processor cycles.

void delayUs(uint16_t microseconds);
// Delay for N x 10 mill sec
NOINLINE void delay10ms(byte ms) { while (ms--) delayUs(10000); }

// Pulse generation for oscillator calibration
void pulse(uint16_t duration);

void calibration() {
	//masterMode = true;
	//PORTB = PORTB & ~_BV(MASTER_PIN);              // Remove pull-up from iButton pin
	pulse(480); pulse(410); pulse(70); pulse(65); pulse(55); pulse(53); pulse(10); pulse(5); pulse(3); delay10ms(100);
}

NOINLINE void Emulate(byte emulRetry = REPEAT);
// Power down sleep mode
NOINLINE void powerDown();
// OneWire functions (based on this project: http://www.technoblogy.com/show?2G8A)
// Configure OneWire pin as OUTPUT (low level was pre-configured in setup())
NOINLINE void master_mode(/*bool button*/);
// Transmit inverted 1-0 sequence
NOINLINE void pinLowRelease(uint16_t low, uint16_t high = 0);
// Initialize OneWire
bool oneWireReset();
// Write bit in Master mode
void masterWriteBit(bool bit);
// Write byte in Master mode
void masterWriteByte(byte data);
// Read bytes into the buffer
void masterReadData(byte buf[8]);
// Read byte in Master mode
byte masterReadByte();
// Reset timer counter and flags
void resetTimer();
// Wait for high level in Slave mode
NOINLINE void waitTimeSlot();

bool error_get() { return TIFR0; } // 120 * timeslot_divider //

bool error_high() {  return TIFR0 & _BV(TOV0);/*_BV(OCF0A);*/ } //overflow timer0 256 * timeslot_divider // 3413.2 ms if F_CPU 4.8mhz
// Read OneWire pin
bool pinRead() { return PINREG & _BV(ONEWIRE_PIN); }
// Wait for high level in Slave mode
void waitForHigh();				// Reset INT0 Interrupt Flag
// Wait for high level in Slave mode
void waitForLow();						// Detect falling edge

NOINLINE void waitReset();
// Write bit in Slave mode
NOINLINE void slaveWriteBit(bool bit);
// Read bit in Slave mode
NOINLINE bool slaveReadBit();
// Calculate CRC over the buffer - 0x00 is correct
NOINLINE byte CRC8(const byte buf[], byte len = 7);
// EEPROM functions
// Write byte to EEPROM
NOINLINE void memWrite(byte ucAddress, byte ucData);
// Read byte from EEPROM
NOINLINE byte memRead(byte ucAddress);
// Other functions
// Blink N times
NOINLINE void blink(byte count);
