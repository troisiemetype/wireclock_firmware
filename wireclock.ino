/*
 * This Arduino sketch is an Attiny 1616 based digital clock
 * Copyright (C) 2022  Pierre-Loup Martin
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *	This project is a digital clock running on an Attiny 1616, featuring :
 *		Partly PCB, partly freeform circuit made of bent and brased brass wire.
 *		A 7-segments display to display time.
 *		A piezo pad for alarm, hour beep, etc.
 *		A photodiode for light sensing, so the brightness of the display adapts to the environment.
 *		Three buttons for setting the time, etc.
 *		(optionnal) two 4x4 led matrix for pimping effects.
 *
 *		Power supply is from an USB C plug.
 *		A 3V cell is provisionned to ensure keeping time if main power is unplugged.
 *
 *		The Attiny will run on its internal RC oscillator at 20MHz.
 *		A 15ppm 32.768 kHz crystal is used as an external clock for the on-chip RTC.
 *		The buttons could be either physical switchs, brass-wire-made (a button-wire that can be pushed against a ground-wire), or capacitive pads.
 */

/*
 *	The main difficulty of this project will be to have a reliable time source. That means precision, and no drift.
 *		Precision is not really a problem, as the internal clock can be adjusted to correct itself on regular basis if it's too slow or too fast.
 *		Drifting is another problem, and the crystal is choosen so as it is stable over time.
 *		A tuning algorithm is provided to adjust this time with a known reference :
 *			Easilly made is a program where the user pushes a button at a known interval, i.e. 24h, so the user imprecision is minored.
 *				This technique has the drawback of relying on the user
 *
 *			A script running on a computer for time reference, that send ticks to the clock via serial at regular intervals
 *				This is far more precise, and thus faster to proceed, but there could still be imprecision coming from peripherals like USB, etc.
 *
 *			The ideal solution would be an embbeded system with reference clock, e.g. an Arduino with GPS.
 *				This solutions ensures that there is no uncertainty linked to USB, serial, or other peripherals. Just level-changing-pin, a few clock ticks.
 */

// We use SPI instead of bit banging to control the shift registers. 
#include <SPI.h>
#include <EEPROM.h>

#include "PushButton.h"

// Pin declaration

const uint8_t PIN_SER = 14;  			// MOSI pin, so SPI is used for clock and data
const uint8_t PIN_OE = 9; 	 			// On a PWM output
const uint8_t PIN_RCLK = 4; 			// Latch pin
const uint8_t PIN_SRCLK = 16; 			// SCK, SPI clock
const uint8_t PIN_SRCLR = 5; 			// Clear register
const uint8_t PIN_PIEZO = 0; 			// Piezo sound

const uint8_t PIN_LIGHT_SENSE = 15;

const uint8_t PIN_NO_USE_1 = 10;
const uint8_t PIN_NO_USE_2 = 11;

const uint8_t PIN_BTN_PLUS = 1;
const uint8_t PIN_BTN_MODE = 2;
const uint8_t PIN_BTN_MINUS = 3;

const uint8_t PIN_SYNC = 17; 			// PA0, UPDI — just for prototype, because of bodging on pins 10, 11, 15
//const uint8_t PIN_SYNC = 10; 			// PC0

const uint8_t eeAddrInit = 0;
const uint8_t eeAddrCal = 2;
const uint8_t eeAddrPER = 4;
const uint8_t eeAddrCompDelay = 6;
const uint8_t eeAddrCompValue = 8;


// The bytes representing every digit on the 7-segment display
const uint8_t digits[] = {
	0b00111111,			// 0
	0b00000110,			// 1
	0b01011011,			// 2
	0b01001111,			// 3
	0b01100110,			// 4
	0b01101101,			// 5
	0b01111101,			// 6
	0b00000111,			// 7
	0b01111111,			// 8
	0b01101111,			// 9
};

// Time variables.
struct time_t{
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	volatile uint32_t ticks;		// Keeps count of the number of ticks. kinda millis(), but slower.
	uint32_t nextComp;
	bool	tick : 1;				// The blinking dots which display seconds.
	bool	twelveHoursFormat : 1;
	bool	dst : 1;				// Daylight saving time. Don't know if it will be implemented, as it varies from countries to countries.
	volatile bool	update : 1;		// This is access by the RTC timer interrupt to signal a change.
} time;

// TODO : some of this struct members should be local variable in the calibrate function.
struct calibration_t{
	uint16_t compDelay;				// This holds the delay after which a compensation has to be applied.
	int16_t compValue;				// This holds the value to compensate for (positive or negative).
	volatile uint32_t current;		// This is the current counter value on tick.
	uint32_t last;					// This is the previous counter value on tick.
	int32_t drift;					// This is the cumulated drift between two subsequent ticks.
	uint32_t lastTick;				// The total tick number when last sync pulse occurs. Used to keep track of the calibration process.
	uint16_t ticks;					// The number of ticks since the beginning of calibration.
	bool calibrate : 1;				// Whether the calibration is complete or not.
	uint8_t calPhase :2;
	volatile bool update : 1; 		// If interrupt has fired.
} sync;

// Status update.
// This is probably not needed as testing for a value to be updated takes more time as updated it anyway.
// We'll see if there are more comples cases.
/*
struct update_t{
	bool second : 1;
	bool minute : 1;
	bool hour : 1;
} update;
*/
// Display values (kind of buffer)
struct display_t{
	uint8_t data[8];
	uint8_t digit;				// Use 2 bits for HH:MM display, 3 bits for HH:MM:SS display on 8x8 matrix.
} display;

uint32_t timeUpdate = 0;
bool piezo = 0;
uint8_t test = 0;

uint16_t lightLevel = 1;
uint8_t lightSmoothingRatio = 6;

enum{
	DISPLAY_TIME = 0,
	SET_MINUTE,
	SET_HOUR,
};

uint8_t mode;

PushButton buttonMode;
PushButton buttonMinus;
PushButton buttonPlus;


// ISR for RTC
ISR(RTC_CNT_vect){
	RTC.INTFLAGS = RTC_OVF_bm;
	time.update = 1;
	++time.ticks;
}

// ISR for sync pin
ISR(PORTA_PORT_vect){
	if(!(PORTA.INTFLAGS & (1 << 0)))
		return;

	PORTA.INTFLAGS |= (1 << 0);
	sync.current = RTC.CNT;
	sync.update = 1;
}

// Functions to increment time.
// Increment seconds.
void updateSeconds(){
	if(++time.second > 59){
		time.second = 0;
//		update.second = true;
		updateMinutes();
	}
}

// increment minutes.
void updateMinutes(){
	if(++time.minute > 59){
		time.minute = 0;
//		update.minute = true;
		updateHours();
	}
}

// increment hours.
void updateHours(){
	if(++time.hour > 23){
		time.hour = 0;
//		update.hour = true;
	}
}

// Functions to set time.
// set minutes.
void setMinutes(int8_t inc){
	if(inc > 0){
		if(time.minute < 59) time.minute++;
		else time.minute = 0;
	} else if(inc < 0){
		if(time.minute > 0) time.minute--;
		else time.minute = 59;
	}
}

// set hours.
void setHours(int8_t inc){
	if(inc > 0){
		if(time.hour <23) time.hour++;
		else time.hour = 0;
	} else if(inc < 0){
		if(time.hour > 0) time.hour--;
		else time.hour = 23;
	}
}

// Display linked functions.
// Update the display buffer with new values.
void bufferDisplay(){
	// Update time

	// HH:MM update	
	display.data[0] = digits[time.minute % 10];
	display.data[1] = digits[time.minute / 10];
	display.data[2] = digits[time.hour % 10];
	display.data[3] = (time.hour / 10)?digits[(time.hour / 10)]:0;
/*
	// HH:MM:SS update
	display.data[0] = digits[time.second % 10];
	display.data[1] = digits[time.second / 10];
	display.data[2] = 0;
	display.data[3] = digits[time.minute % 10];
	display.data[4] = digits[time.minute / 10];
	display.data[5] = 0;
	display.data[6] = digits[time.hour % 10];
	display.data[7] = (time.hour / 10)?digits[(time.hour / 10)]:0;
*/
//	display.data[3] = digits[time.hour / 10];

/*
	// Temp values, seconds + minutes, for debug.
	display.data[0] = digits[time.second % 10];
	display.data[1] = digits[time.second / 10];
	display.data[2] = digits[time.minute % 10];
	display.data[3] = (time.minute / 10)?digits[(time.minute / 10)]:0;
*/
/*	
	uint16_t light = analogRead(PIN_LIGHT_SENSE);

	display.data[0] = digits[light % 10];
	display.data[1] = ((light / 10) % 10)?digits[(light / 10) % 10]:0;
	display.data[2] = ((light / 100) % 10)?digits[(light / 100) % 10]:0;
	display.data[3] = (light / 1000)?digits[(light / 1000)]:0;
*/

	// update blinking dots
	// Note : not every 7-segment have their two central dots wired on the same digit.
	// Anyway, once the circuit is assembled it doesn't change, so we hard-code it.
	if(time.tick){
		if(mode == DISPLAY_TIME){
			if(sync.calibrate){
				display.data[0] |= 0x80;
				display.data[1] |= 0x80;				
			}
			// Uncomment for 8x8 matrix :
//			display.data[3] |= 0x80;
//			display.data[4] |= 0x80;
		} else if(mode == SET_MINUTE){
			display.data[0] |= 0x80;
		} else if(mode == SET_HOUR){
			display.data[1] |= 0x80;
		}

	}

	// Update matrix
}

// update display routine, called about every milliseconds.
// Above two milliseconds, there is a visible flickr.
void updateDisplay(){
	// We send the data trough SPI.
	// First byte is the active digit / matrix col. leds are active high, so it has to be pullued low to be active.
	// Second byte is the active segment / matrix row.

	// This is the update for HH:MM display

	digitalWrite(PIN_RCLK, 0);
	SPI.transfer(display.data[display.digit]);
	SPI.transfer(~(1 << (3 - display.digit)));
	digitalWrite(PIN_RCLK, 1);

	//We increment the digit counter. It's a two bits unsigned number, so it rolls back from 3 to 0 by itself.
	if(++display.digit > 3) display.digit = 0;

	// This is the update for HH:MM:SS 8x8 display
/*
	digitalWrite(PIN_RCLK, 0);
	SPI.transfer((1 << (display.digit)));
	SPI.transfer(~(display.data[display.digit]));
	digitalWrite(PIN_RCLK, 1);

	//We increment the digit counter. It's a three bits unsigned number, so it rolls back from 7 to 0 by itself.
	if(++display.digit > 7) display.digit = 0;
*/
}

// dimming function, called by the main loop on every tick.
void dimDisplay(){
//	TCA0.SINGLE.CMP0 = analogRead(PIN_LIGHT_SENSE);
//	return;

	uint16_t light = analogRead(PIN_LIGHT_SENSE);
	light = light * lightSmoothingRatio + lightLevel * (64 - lightSmoothingRatio);
	light /= 64;

	if(lightLevel != light){
		lightLevel = light;
		if(lightLevel == 0) lightLevel = 1;
		TCA0.SINGLE.CMP0 = lightLevel;
	}
}

// Button handling
void readButtons(){
	if(buttonMode.update()){
		if(buttonMode.isLongPressed() && mode == DISPLAY_TIME){
			mode = SET_MINUTE;
		} else if(buttonMode.justPressed()){
			if(++mode > SET_HOUR){
				mode = DISPLAY_TIME;
				time.second = 0;
				while(RTC.STATUS & (1 << 1));
				RTC.CNT = 0;
			}
		}
	}

	buttonMinus.update();
	buttonPlus.update();
}

// Greater common divisor
int16_t GCD(int16_t a, int16_t b){
	if(b == 0) return a;
	return GCD(b, a % b);
}

// Update sync routine. Called whenever there is an interrupt on sync pin.
void updateSync(){
/*
 * For memory, sync structure :

struct calibration_t{
	uint32_t compDelay;				// This holds the delay after which a compensation has to be applied.
	volatile uint32_t current;		// This is the current counter value on tick.
	uint32_t last;					// This is the previous counter value on tick.
	int32_t drift;					// This is the cumulated drift between two subsequent ticks.
	uint32_t lastTick;				// The total tick number when last sync pulse occurs. Used to keep track of the 
	uint16_t ticks;					// The number of ticks since the beginning of calibration.
	bool compDirection : 1; 		// In which way will the compensation will be applied.
	bool calibrate : 1; 			// If we are calibrating.
	volatile bool update : 1; 		// If interrupt has fired.
} sync;

*/	
	sync.calibrate = 0;
	sync.ticks = 0;
	sync.drift = 0;
	sync.calPhase = 0;

	sync.lastTick = time.ticks;

	sync.last = sync.current;
	sync.update = 0;

	// We now enter an endless loop, 
	for(;;){
		// If the interrupt has fired, we check the new value against the older, and compute instant and cumulated drift.
		if(sync.update){
			sync.update = 0;
			// Compute instant drift.
			int32_t drift = sync.current - sync.last;
			sync.drift -= drift;
			// Hold this new value.
			sync.last = sync.current;

			// Store the current tick value so we know the GPS module doesn't stall.
			sync.lastTick = time.ticks;

			// Increment the calibration tick counter so we know we have long enough time to be accurate.
			sync.ticks++;

			// Here we want to display the current drift

			bool polarity = 0;
			if(sync.drift < 0) polarity = 1;

			drift = abs(sync.drift);

			display.data[0] = digits[drift % 10];
			drift /= 10;
			display.data[1] = (drift)?digits[(drift % 10)]:0;
			drift /= 10;
			display.data[2] = (drift)?digits[(drift % 10)]:0;
			display.data[3] = (polarity > 0)?0:0b01000000;

			(sync.calPhase > 0) ? display.data[0] |= 0x80 : 0;
			(sync.calPhase > 1) ? display.data[1] |= 0x80 : 0;

			if(sync.calPhase == 0){
				if(sync.ticks > 10){
					// Wait for RTC.PER to be writable.
					while(RTC.STATUS & (1 << 2));
					// Load new value
					RTC.PER -=  sync.drift / (sync.ticks * 2);

					// Re-init values for fine calibrating
					sync.ticks = 0;
					sync.drift = 0;
					sync.calPhase = 1;

					sync.last = sync.current;
					sync.update = 0;
				}

			} else if(sync.calPhase == 1 ){
				if(sync.ticks > 20){
					// Compute the number of ticks between corrections.
					int16_t gcd = GCD(sync.drift, sync.ticks);
					// We have to multiply by 2, as clocks ticks are half a second, whereas GPS is one pulse per second.
					sync.compDelay = (sync.ticks * 2) / gcd;
					sync.compValue = sync.drift / gcd;

					// Save the new values to eeprom.
					EEPROM.put(eeAddrCompDelay, sync.compDelay);
					EEPROM.put(eeAddrCompValue, sync.compValue);

					// Also save the new RTC.PER value to eeprom as well.
					uint16_t per = RTC.PER;
					EEPROM.put(eeAddrPER, per);

					// And save flag that says calibration has been done.
					EEPROM.update(eeAddrCal, 0b00000011);


					sync.calibrate = 1;
					sync.calPhase = 2;
				}
			} else {
				// Nothing
			}

			// Eventually, if enough time has elapsed since the beginning of calibration, we can compute the final value.
			// It will be taken into account when the loop exits
			// Calibration time is arbitrary. Given GPS precision, a few seconds are enough.
/*			if((sync.ticks > 30)){
				// Display a dot to let know that enough calibration time is elapsed.
				display.data[0] |= 0x80;
				// Compute the number of ticks between corrections.
				int16_t gcd = GCD(sync.drift, sync.ticks);
				// We have to multiply by 2, as clocks ticks are half a second, whereas GPS is one pulse per second.
				sync.compDelay = (sync.ticks * 2) / gcd;
				sync.compValue = sync.drift / gcd;

				sync.calibrate = 1;
			}
*/
		}

		// We also want to keep track of the time, so it's still good when the calibration is done.
		if(time.update){
			time.update = 0;
			time.tick ^= 1;
			if (time.tick == 0) updateSeconds();
		}

		updateDisplay();

		// Test how much ticks have past since last interrupt. If more than 3, the GPS module was stalled or disconnected.
		if((time.ticks - sync.lastTick) > 3){
			// If we're leaving with enough calibration data, we save it in memory.
			if(sync.calibrate){
				time.nextComp = time.ticks + sync.compDelay;
			}
			return;
		}
	}
}

// Timers and interrupts definitions

// initialisation of the timer used for display dimming.
void initDimmingTimer(){
	// Set the top value for the timer.
	TCA0.SINGLE.PER = 0x3ff;
	// Enable the timer. (and no prescaler)
	TCA0.SINGLE.CTRLA = 0x1;
	// Enable single-slope PWM
	TCA0.SINGLE.CTRLB = 0;
	TCA0.SINGLE.CTRLB |= 0b011;
	// Set output on WO0 (PB0, pin OE)
//	TCA0.SINGLE.CTRLB |= (1 << 4);
	// invert output on the port pin, shift register OE is active low.
	PORTB.PIN0CTRL |= (1 << 7);
	// Set default value for dimming.
	TCA0.SINGLE.CMP0 = 1023;
}

// initialisation of the RTC timer (which runs on external 32.768kHz external crystal)
void initRTC(){
	uint8_t temp = 0;
	// Configure the oscillator in Clock Controller Peripheral CLKCTRL
	// (1 << RUNSTDBY) | (1 << ENABLE)
	//	CLKCTRL.XOSC32KCTRLA = (1 << 1) | (1 << 0);
	temp = CLKCTRL.XOSC32KCTRLA;
	temp &= ~CLKCTRL_ENABLE_bm;
	_PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, temp);
//	CCP = 0xd8;
//	CLKCTRL.XOSC32KCTRLA = temp;

	while(CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm);

	temp = CLKCTRL.XOSC32KCTRLA;
	temp |= (0x3 << 4);
	temp &= ~CLKCTRL_SEL_bm;
	temp |= CLKCTRL_RUNSTDBY_bm;
	_PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, temp);
//	CCP = 0xd8;
//	CLKCTRL.XOSC32KCTRLA = temp;

	temp = CLKCTRL.XOSC32KCTRLA;
	temp |= CLKCTRL_ENABLE_bm;
	_PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, temp);
//	CCP = 0xd8;
//	CLKCTRL.XOSC32KCTRLA = temp;

	while(RTC.STATUS > 0);

	// Set the overflow value in RTC.PER
	uint16_t per = 0;
	EEPROM.get(eeAddrPER, per);
	cli();
	RTC.PER = per;
//	RTC.PER = 0x3fff;
//	RTC.PERL = 0xff;
//	RTC.PERH = 0x3f;

	// Write the clock selections bits in the clock selection register CKSEL in RTC.CKSEL
	// Select external oscillator as a source.
	RTC.CLKSEL = 0x02;
//	RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc;
	// For debug purpose : extract 32KHz from main clock.
//	RTC.CLKSEL = 0x00;

	// configure RTC internal prescaler in RTC.CTRLA
	// Run in standby mode, nor prescaller, RTC enabled.
	// (1 << RUNSTDBY)
//	RTC.CTRLA = (1 << 7);
	RTC.CTRLA = RTC_PRESCALER_DIV1_gc | RTC_RTCEN_bm | RTC_RUNSTDBY_bm;
	// Enable interrupts (CMP, OVF) in RTC.INTCTRL
	// (1 << CMP)
//	RTC.INTCTRL = (1 << 1);
	RTC.INTCTRL |= RTC_OVF_bm;
	// enable clock by writing 1 to RTCEN in RTC.CTRLA
	// (1 << RTCEN)
//	RTC.CTRLA = (1 << 0);

	sei();
}

// initialisation of the pin used for sync.
void initSyncPin(){
	// sync pin is PC0, except for prototype where this pin is unusable which uses PA0 instead.
	// Enable rising edge interrupt, not inverted, no pull-up, on sync pin
//	PORTC.PIN0CTRL = 0x02;
	PORTA.PIN0CTRL = 0x02;
}

void deinitSyncPin(){

}

void setup(){
	// Init pins direction and state.
	pinMode(PIN_SER, OUTPUT);
	pinMode(PIN_OE, OUTPUT);
	pinMode(PIN_RCLK, OUTPUT);
	pinMode(PIN_SRCLK, OUTPUT);
	pinMode(PIN_SRCLR, OUTPUT);

	pinMode(PIN_PIEZO, OUTPUT);

	pinMode(PIN_LIGHT_SENSE, INPUT);

	pinMode(PIN_BTN_PLUS, INPUT_PULLUP);
	pinMode(PIN_BTN_MODE, INPUT_PULLUP);
	pinMode(PIN_BTN_MINUS, INPUT_PULLUP);

	digitalWrite(PIN_SER, 0);
	digitalWrite(PIN_OE, 0);
	digitalWrite(PIN_RCLK, 0);
	digitalWrite(PIN_SRCLK, 0);
	digitalWrite(PIN_SRCLR, 1);

	digitalWrite(PIN_PIEZO, piezo);

	// Init SPI
	SPI.begin();
	SPI.beginTransaction(SPISettings(8000000L, MSBFIRST, SPI_MODE0));

	// Init global variables values
	memset(&time, 0, sizeof(time_t));
	memset(&sync, 0, sizeof(time_t));
	memset(&display, 0, sizeof(display_t));

	time.tick = 1;

	// Eeprom init / loading
	uint16_t per = 0x3fff;
	uint16_t compDelay = 0;
	int16_t compValue = 0;
	uint8_t calibrated = 0;

	// Write default values to eeprom. Only used on very first launch
	if(EEPROM.read(eeAddrInit) != 'e' || EEPROM.read(eeAddrInit + 1) != 'i'){
		EEPROM.put(eeAddrPER, per);
		EEPROM.put(eeAddrCompDelay, compDelay);
		EEPROM.put(eeAddrCompValue, compValue);

		EEPROM.write(eeAddrInit, 'e');
		EEPROM.write(eeAddrInit + 1, 'i');
		EEPROM.write(eeAddrCal, 0);
	}

	// load compensation values from eeprom. PER is loaded from the initRTC function.
	EEPROM.get(eeAddrCompDelay, sync.compDelay);
	EEPROM.get(eeAddrCompValue, sync.compValue);

	EEPROM.get(eeAddrCal, calibrated);

	if(calibrated == 0b11) sync.calibrate = 1;

	// Init timers
	initDimmingTimer();
	initRTC();
	initSyncPin();

	// Debug : use the compilation time to preset time.
	char ct[] = __TIME__;
	time.hour = (ct[0] - '0') * 10 + (ct[1] - '0');
	time.minute = (ct[3] - '0') * 10 + (ct[4] - '0');
	time.second = (ct[6] - '0') * 10 + (ct[7] - '0');
//	time.minute = 8;
//	time.hour = 23;

	mode = DISPLAY_TIME;

	buttonMode.begin(PIN_BTN_MODE, INPUT_PULLUP);
	buttonMinus.begin(PIN_BTN_MINUS, INPUT_PULLUP);
	buttonPlus.begin(PIN_BTN_PLUS, INPUT_PULLUP);

	bufferDisplay();
	updateDisplay();
}

void loop(){

	readButtons();

	if(time.update){
		time.update = 0;
		digitalWrite(PIN_PIEZO, time.tick ^= 1);
//		time.tick ^= 1;
		if (time.tick == 1 && mode == DISPLAY_TIME) updateSeconds();
		bufferDisplay();

		// update light reading
//		dimDisplay();
	}	

	if(mode == SET_MINUTE){
		if(buttonMinus.justPressed())
			setMinutes(-1);
		else if(buttonPlus.justPressed())
			setMinutes(1);
	} else if(mode == SET_HOUR){
		if(buttonMinus.justPressed())
			setHours(-1);
		else if(buttonPlus.justPressed())
			setHours(1);
	}

	if(sync.update){
		updateSync();
	}

	if(sync.calibrate && (time.ticks == time.nextComp)){
		time.nextComp += sync.compDelay;
		RTC.CNT += sync.compValue;
	}

/*
	uint32_t now = millis();
	if((now - timeUpdate) > 1000){
		timeUpdate = now;
		updateSeconds();
		bufferDisplay();
		time.tick ^= 1;
//		digitalWrite(PIN_PIEZO, time.tick ^= 1);
	}
*/
	updateDisplay();

/*
	digitalWrite(PIN_RCLK, 0);
	SPI.transfer(test++);
	digitalWrite(PIN_RCLK, 1);
	delay(10);
*/
}