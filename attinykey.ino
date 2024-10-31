/*
 Name:		attinykey.ino
 Created:	16-Feb-24 03:14:21
 Author:	sQueezy
*/
#include "attinykey.h"

void loop() {
	static byte rom_addr = 0xFF;
	byte repeats = repeats_init, data, i, bitmask;  // Address of the current key ROM
	//sei();                          // Enable interrupts  
	//DDRB = _BV(BATTERY_PIN) | _BV(LED_PIN);   //now led, battery output (0)	// Invert pull-ups (1 for inputs, 0 for outputs - as far as I could measure, this configuration consumes the least energy)
	//DDRB |= _BV(BUTTON_PIN);					// Set battery pin as output (gnd)				// configure button pin as output (gnd)
	//PORTB = ~DDRB;	
	powerDown();	// Sleep and accumulate energy
	//DDRB |= _BV(ONEWIRE_PIN); MCUCR = _BV(SE) | _BV(SM1);  while (true){__asm__ __volatile__("sleep");} 
WaitForResetPulse:
	//if(!waitReset()) return;
	waitReset();
	delayUs(20);
	//pinLowRelease(90);
	DDRB |= _BV(ONEWIRE_PIN); //Send Presence pulse
	//resetTimer();
	if (rom_addr > EEPROM_SIZE) {
		dataBytes[0] = 0x1;
	}
	else {
		for (i = 0; i < 7; i++) dataBytes[i] = memRead(rom_addr + i);			// Get the current ROM bit //137 ticks * (1/4.8Mhz) ~ 28.5 mcs
	}
	dataBytes[7] = CRC8(dataBytes);		//~387 ticks * (1/4.8Mhz) ~ 80.6 mcs
	//while (!(TIFR0 & _BV(OCF0B))) {};
	DDRB &= ~(_BV(ONEWIRE_PIN)); // END Presence Configure OneWire pin as INPUT
	//TCNT0 = OCR0A = 120 / timeslot_divider; //Skip first timeslot check (now we have 2048 /*(1706.66)*/microseconds until the next timeout)
	for (bitmask = 1, data = 0; bitmask; bitmask <<= 1) {
		if (slaveReadBit()) data |= bitmask;
		if (error_get()) goto WaitForResetPulse;
	}
	if (data == SEARCH_CMD /*|| data == ALARM_CMD*/) {
		bool bit_send, bit_recv;
		for (i = 0; i < 8; i++) {
			for (bitmask = 1; bitmask; bitmask <<= 1) {
				bit_send = dataBytes[i] & bitmask;
				slaveWriteBit(bit_send); if (error_high()) return;//goto Loop;	// Write the bit into the bus and check timeslot error
				slaveWriteBit(!bit_send); if (error_high()) return;		// Write the inverted bit into the bus and check timeslot error
				bit_recv = slaveReadBit(); if (error_high()) return;
				if (bit_send != bit_recv) goto WaitForResetPulse;	// Check if the master has chosen another slave
			}
		}
	}
	else if (data & 0b11 ) { // Answer with the current iButton ROM
		for (i = 0; i < 8; i++) {
			for (bitmask = 1; bitmask; bitmask <<= 1) {
				slaveWriteBit(dataBytes[i] & bitmask);
				if (error_get()) goto WaitForResetPulse;//goto Loop; // Check timeslot error
			}
		}
	}
	if (repeats) { --repeats; goto WaitForResetPulse; }
	if ((rom_addr >= (EEPROM_SIZE - 1 - 7)) /*|| (memRead(rom_addr) == 0xFF)*/) { 
		//if (rom_addr == 0xFF) {
		//	for (i = 1; i < 7; i++)  dataBytes[i] = 0xFF;
		//	--rom_addr;
		//}
		//else 
			rom_addr = 0;
		return;
	} 
	rom_addr += 7;// Switch to the next saved ROM
}

int main() {
#ifndef __LGT8FX8P__
	TCCR0B = 0b011; //prescaler(clk / 64) = 150khz // prescaler(clk / 64) = 125khz  
#else
	TCCR0B = 0b100 /*| _BV(CS00)*/; //prescaler(clk / 256) = 125khz
#endif // !__LGT8FX8P__
	PRR = 0xFF;                     // Disable timer and ADC
	ACSR = _BV(ACD);                // Disable Analog Comparator
	PORTB = _BV(BUTTON_PIN);		// Pull up only button pin
	if (PINB & _BV(BATTERY_PIN)) {              // Check if the emulator powered by battery
		master_mode();
	}
	if (PINB & _BV(BUTTON_PIN)) repeats_init = REPEAT - 1; // The number of repeats for one key
	PORTB = 0;	//Disable pullup
	DDRB = (_BV(LED_PIN) | _BV(BUTTON_PIN) | _BV(BATTERY_PIN));
	sei();
	powerDown();						// First sleep after powering on to stabilize the processes //USING (SUT 0b10) FUSE BITS FOR 64MS DELAY STARTUP
	//OSCCAL = 60;						// 0-127 Oscillator Frequency calibration value: set the frequency to ~8 MHz (should be calibrated manually)
	for (;;) {
		loop();
	}
}

// Power down sleep mode
void powerDown() {
	DIDR0 = 0xFF;											// Disabling digital input circuits.
	__asm__ __volatile__("wdr");												// Reset Watchdog timer
	WDTREG = _BV(WDCE);
	//if ((MCUSR & _BV(PORF))) {							// Check if just powered on 
	//	MCUSR = 0;
	//}
	//else {
		//      WDTCR = _BV(WDTIE) | _BV(WDP2);				// Sleep for 0.25s before processing the next key ROM
#ifndef __LGT8FX8P__
	WDTREG = _BV(WDTIE) | 0b101 /* 0b110*/;				// Sleep for 0.5s before processing the next key ROM //0b110 (1 sec)
#else
	WDTREG = _BV(WDTIE) | 0b011 /*| 0b110*/;
#endif // __LGT8FX8P__
	//}
	MCUCR = _BV(SE) | _BV(SM1);                             // Sleep Enable, power-down mode.
	__asm__ __volatile__("sleep");
	DIDR0 = 0;
}

void waitReset() {
	do {
		PRR = 0xFF;                     // Disable timer and ADC
		IFREG = 0xFF;					//Reset interrupt flags
		MCUCR = _BV(SE) | _BV(SM1);     // Sleep Enable, Power-down mode, INT0 Low level detection mode
		INTREG = _BV(INT0);				// Enable INT0 interrupts
		WDTREG = _BV(WDCE);
		WDTREG = _BV(WDTIE);			//16ms
		__asm__ __volatile__("sleep");
		/*
		if (MCUSR & _BV(WDRF)) {							// Check if just powered on
			MCUSR = 0;
			Emulate();
			return false;
		}
		*/
		//MCUSR = 0; //WDRF
		INTREG = 0;						// Disable INT0 interrupts
		MCUCR = _BV(ISC01);				// INT0 falling edge detection mode
		PRR = ~_BV(PRTIM0);				// Enable Timer			
		// Setup timer to control timeslots. To check whether the pulse fits into the timeslot, it is necessary that channel B triggers, but channel A does not trigger.
		OCR0A = 720 / timeslot_divider; // 960us is the maximum pulse duration, 240 is the minimum.
		OCR0B = 360 / timeslot_divider; // We would also need to subtract the ~22 clock cycles that are spent on waking up from sleep, but the one clock cycle of the timer is equal to 64 clock cycles of the CPU, so we will not do this unless absolutely necessary.  
		resetTimer();					// Reset timer counter and flags
		while (!pinRead()) { /*line low*/ };					// Wait for the end of the Reset pulse
	} //while (!(TIFR0 & _BV(OCF0B)) || (TIFR0 & _BV(OCF0A)));// Check timeslot error
	while (TIFR0 != _BV(OCF0B));
	//OCR0A = 1013 / (timeslot_divider);
	OCR0B = 120 / (timeslot_divider);
	//return true;
}

void delayUs(uint16_t microseconds) {
	// Small cycle for less than 10 microseconds
	delayMicroseconds(microseconds);// return;
	//asm volatile ("nop");
}

void resetTimer() {
	TCNT0 = 0;			// Reset timer counter 
	TIFR0 = 0xFF;		// And flags
}

// Transmit inverted 1-0 sequence
void pinLowRelease(uint16_t low, uint16_t high) {
	DDRB |= _BV(ONEWIRE_PIN); // Configure OneWire pin as OUTPUT (low level was pre-configured in setup())
	delayUs(low);
	DDRB &= ~_BV(ONEWIRE_PIN); // Configure OneWire pin as INPUT
	delayUs(high);
}

// Initialize OneWire
bool oneWireReset() {
	pinLowRelease(480, 69);
	bool presence = !pinRead();
	delayUs(230);
	presence &= pinRead();
	delayUs(180);
	return presence;
}

// Write byte in Master mode
void masterWriteByte(const byte data) {
	for (byte bitmask = 1; bitmask; bitmask <<= 1) {
		masterWriteBit(data & bitmask);
	}
}

// Read bytes into the buffer
void masterReadData(byte buf[]) {
	for (byte i = 0; i < 8; i++) {
		buf[i] = masterReadByte();
	}
}

// Read byte in Master mode
byte masterReadByte() {
	byte data = 0;
	for (byte bitMask = 1; bitMask; bitMask <<= 1) {
		pinLowRelease(3, 15);
		if (pinRead()) data |= bitMask;
		delayUs(50);
	}
	return data;
}

// Write bit in Master mode
void masterWriteBit(bool bit) {
	byte low, high;
	if (bit) {
		low = 5; high = 60;
	}
	else {
		low = 50; high = 15;
	}
	pinLowRelease(low, high);
}

// Wait for high level in Slave mode
void waitTimeSlot() {
	resetTimer();
	waitForHigh();
	if (error_get()) return;
	resetTimer();
	waitForLow();
	if (error_high()) return;
}

// Read bit in Slave mode
bool slaveReadBit() {
	waitTimeSlot();
	if (error_get()) return false;
	delayUs(15);
	return pinRead();					// Merge bit whith timeslot check result
}

// Write bit in Slave mode
void slaveWriteBit(bool bit) {
	waitTimeSlot();
	if (error_get()) return;
	if (bit == false) {
		pinLowRelease(20);
	}
}

// Write byte to EEPROM
void memWrite(byte ucAddress, byte ucData) {
	while (EECR & _BV(EEPE)) {};		// Wait for completion of previous write
	//byte store = memRead(1020);
	//if (store != ucData) {
	//	if ((store | (ucData ^ 0xFF)) == 0xFF) EECR = _BV(EEPM1);
	//	else EECR = 0;
	EEAR = ucAddress;				// Set up address and bit registers
	EEDR = ucData;
	EECR |= _BV(EEMPE);				// Master bit program enable
	EECR |= _BV(EEPE);              // Program enable 
//}
}

// Read byte from EEPROM
byte memRead(byte ucAddress) {
	while (EECR & _BV(EEPE)) {};		// Wait for completion of previous read
	EEAR = ucAddress;					// Set up address register
	EECR |= _BV(EERE);					// Read Enable
	return EEDR;						// Return bit from bit register
}

byte CRC8(const byte buf[], byte len) {
	byte crc = 0;
	while (len--) { crc = _crc_ibutton_update(crc, *buf++); };
	return crc; // Calculate CRC over the buffer: (true) is correct
}

// Other functions
// Blink N times
void blink(byte count) {
	while (count--) {
		PORTB &= ~_BV(LED_PIN);        // Turn LED off
		//delay()
		delay10ms(50);                 // 0.5s
		PORTB |= _BV(LED_PIN);         // Turn LED on
		delay10ms(50);                 // 0.5s
	}
}

bool find_key(byte& addr, const byte buf[]) {
	for (byte store, i;;) { // Search for the ROM in the saved list
		for (i = 0;; ++i) {
			store = memRead(addr + i);
			// if ((~val | j) == 0) {										// Beware of integer promotion when performing bitwise operations on integer types smaller than int:
																			//  https://wiki.sei.cmu.edu/confluence/display/c/EXP14-C.+Beware+of+integer+promotion+when+performing+bitwise+operations+on+integer+types+smaller+than+int
			//if (((store ^ 0xFF) | i) == 0) return false;					// Check first byte for 0xFF (End of list reached)
			if (i == 0 && store == 0xFF) return false;
			if (store != buf[i]) break;										// Doesn't match. Go to the next one.
			if (i == 7) return true;										//key is finded
		}
		if ((addr += 8) >= EEPROM_SIZE) return true;
	}
}

void master_mode() {
	byte addr, i;
	PORTB = DDRB = _BV(LED_PIN) | _BV(MASTER_PIN);		// LED and OneWire output 1
	if (!(PINB & _BV(BUTTON_PIN))) { //
		blink(1);
		EECR |= _BV(EEPM0); //only erase
		addr = EEPROM_SIZE - 7 - 1;
		while (!(PINB & _BV(BUTTON_PIN)) && addr) {
			addr -= 7; blink(1);
			if (memRead(addr) != 0xFF) {
				//PORTB &= ~_BV(LED_PIN);
				for (i = 0; i < EEPROM_SIZE; i++) memWrite(i, 0);    // Erase EEPROM before new programming
				//PORTB |= _BV(LED_PIN);
			}
		}
		EECR = 0;
	}
	for (addr = 0;;) {
		//delay10ms(100);
		if (oneWireReset()) {                      // Reset OneWire
			masterWriteByte(READ_CMD);          // Read iButton ROM
			masterReadData(dataBytes);
			if (dataBytes[7] == (CRC8(dataBytes))) {// Check ROM CRC
				if (find_key(addr, dataBytes)) blink(2);		// ROM was found or EEPROM is full
				else {
					blink(1);									// If the ROM is not found and we have not reached the EEPROM limits
					for (i = 0; i < 8; i++) {
						memWrite(addr + i, dataBytes[i]);		// Save new ROM to the list
					}
				}
			}//else blink(3);								// ROM CRC Error	
		}
	}
}

void writeBitMetakom(bool bit, const byte& Ti, const byte& Tj) {
	DDRB &= ~(_BV(ONEWIRE_PIN));		// End high current consumption
	delayUs(bit ? Ti : Tj);
	DDRB |= _BV(ONEWIRE_PIN);		// Start high current consumption
	delayUs(bit ? Tj : Ti);
}

void Emulate(byte emulRetry) {
	const byte Tlong = 120, Tshort = 60;
	static byte nibble = 0xFF;
	byte bitmask, i;
	while (emulRetry--) {
		if (nibble == 0xFF) {
		METAKOM:
			DDRB |= _BV(ONEWIRE_PIN);
			delayUs(((0xFF & 1) ? Tshort : Tlong) + (Tlong + Tshort));			//sending synchronise bit log 0
			for (bitmask = 0b100; bitmask; bitmask >>= 1) {
				writeBitMetakom(0x2 & bitmask, Tlong, Tshort);					//sending start nibble
			}
			for (i = 0; i < 4; i++) {
				for (bitmask = 128; bitmask; bitmask >>= 1) {
					writeBitMetakom(nibble & bitmask, Tlong, Tshort);
				}
			}
			DDRB &= ~_BV(ONEWIRE_PIN);
		}
		else {
		CYFRAL: //0xEE
			for (bitmask = 0b1000; bitmask; bitmask >>= 1) {
				0x1 & bitmask ? pinLowRelease(Tshort, Tlong) : pinLowRelease(Tlong, Tshort); //sending start nibble
			}
			for (i = 0; i < 4; i++) {
				for (bitmask = 128; bitmask; bitmask >>= 1) {			//reading nibble from MSB
					nibble& bitmask ? pinLowRelease(Tshort, Tlong) : pinLowRelease(Tlong, Tshort);
				}
			}
		}
	}
	nibble ^= 0x11;
}

// END-OF-FILE
