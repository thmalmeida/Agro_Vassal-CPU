/*
Used ports

PG5 --> k1
PE5 --> k2
PE4	--> k3

PC2 --> s11
PC1 --> s10
PC0 --> s09
PD7 --> s08
PG2 --> s07
PG1 --> s06
PG0 --> s05
PL7 --> s04
PL6 --> s03
PL5 --> s02
PL4 --> s01
PB4 --> f01
PB5 --> f02

PH3 --> k1_read
PE3 --> k3_read
PH4 --> Th_read

PF0 --> (A0) Damage!!!
PF1 --> (A1) ADC Irms
PF2	-->	(A2) ADC Pressure Sensor
PF3 -->	(A3) ADC Reservoir Level (useful)
PD1 --> SDA DS1307
PD0 --> SDL DS1307

PD3 --> TX1 BT
PD2 --> RX1 BT

Errors:
0x00: No error detected;
0x01: AND OP: no valves opened!
0x02: Thermal safe!
0x03: Im sensor down!
0x04: PRessure DOWN!
0x05: PRessure HIGH!
0x06: k3 locked on starting
0x07: Time after green flag
0x08: Bluetooth serial first error
0x09: Bluetooth serial second error
0x10: Mission Acomplished!
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include <string.h>
#include <stdlib.h>

#include <Arduino.h>

#include "RTC/Time.h"
#include "RTC/DS1307RTC.h"

#include "Comm/Wire.h"
#include "Comm/SoftwareSerial.h"

//#include "MemoryFree.h"

tmElements_t tm;
SoftwareSerial SerialSIM900 =  SoftwareSerial(2, 3);

#define k3_on()		PORTD |=  (1<<4);	// Triac 3 is enabled
#define k3_off()	PORTD &= ~(1<<4);	// Triac 3 is disabled
#define k2_on()		PORTD |=  (1<<5);	// Triac 2 is enabled
#define k2_off()	PORTD &= ~(1<<5);	// Triac 2 is disabled
#define k1_on()		PORTD |=  (1<<6);	// Triac 1 is enabled
#define k1_off()	PORTD &= ~(1<<6);	// Triac 1 is disabled

// Contactors input
#define k1_readPin	bit_is_clear(PIND, 7)
#define k3_readPin 	bit_is_clear(PINB, 0)
#define Th_readPin	bit_is_clear(PINB, 1)

//#define k1_readPinD	bit_is_set(PIND, 7)
//#define k3_readPinD	bit_is_set(PINB, 0)

// Time decision variables
#define HourOn	21
#define MinOn 	30
#define HourOff	6
#define MinOff	0

// PRessure
#define valveAlloc	11

// -------------------------------------------------------- //

// Global
// EEPROM memory Addresses
const uint8_t addr_stateMode   	= 0;		// 1  byte
const uint8_t addr_lastError	= 1;		// 1  byte
const uint8_t addr_minPRess		= 2;		// 1  byte
const uint8_t addr_maxPRess		= 3;		// 1  byte
const uint8_t addr_minIs		= 4;		// 2  bytes
const uint8_t addr_maxIs		= 6;		// 2  bytes
const uint8_t addr_timePipeB	= 8;		// 2  byte
const uint8_t addr_timeSector   = 10;		// 11 bytes allocated
const uint8_t addr_valveSequence= 30;		// 11 bytes allocated
const uint8_t addr_HourOnTM 	= 42;		// nTM (9) byte(s)
const uint8_t addr_MinOnTM 		= 52;		// nTM (9) byte(s)
const uint8_t addr_nTM 			= 62;		// 1 byte
const uint8_t addr_motorTimerStart1 = 65;	// 2 bytes
const uint8_t addr_motorTimerStart2 = 67;	// 2 bytes

uint16_t timePipeB = 200;						// time delay in seconds
volatile uint16_t count_timePipeB = timePipeB;
volatile uint8_t flag_BrokenPipeVerify = 0;

//const uint8_t addr_standBy_min 	= 5;		// 2 bytes
//const uint8_t addr_motorTimerE 	= 7;			// 1 byte
//const uint8_t addr_Iper				= 32;		// 1 byte only
//const uint8_t addr_iValveCheck		= 33;		// 1 byte only
//const uint8_t addr_LevelRef 	= 2;			// 4 bytes

// Times
uint8_t nTM;
uint8_t HourOnTM[9];
uint8_t MinOnTM[9];

// Motor timers in milliseconds
uint8_t motorTimerStart1 = 55;
uint16_t motorTimerStart2 = 200;

// Logs
const int nLog = 12;
uint8_t loadLog[nLog];
uint8_t loadStatus[nLog];
uint8_t hourLog[nLog];
uint8_t minuteLog[nLog];

// Wake up interrupts
uint8_t flag_WDRF = 0;			// Watchdog System Reset Flag
uint8_t flag_BORF = 0;			// Brown-out Reset Flag
uint8_t flag_EXTRF = 0;			// External Reset Flag
uint8_t flag_PORF = 0;			// Power-on Reset Flag

uint8_t motorStatus = 0;
uint8_t periodState = 0;
uint8_t stateMode = 0;

uint8_t minPRess = 0;
uint8_t maxPRess = 0;
uint16_t minIs = 0;
uint16_t maxIs = 0;
uint16_t nodeValve_maxIs = 0;

uint8_t flag_iValveCheck = 1;
uint8_t flag_waitPowerOn = 0;	// Minutes before start motor after power line ocasionally down
uint8_t powerOn_min_Standy = 0;
uint8_t powerOn_min = 0;
uint8_t powerOn_sec = 0;

volatile uint16_t timeSector = 0;
uint8_t lastError = 0;
uint8_t timeSectorVectorMin[valveAlloc];
uint8_t valveSequence[valveAlloc] 	= {0,0,0,0,0,0,0,0,0,0,0};
uint8_t valveStatus[valveAlloc]		= {0,0,0,0,0,0,0,0,0,0,0};
uint16_t valveIs					= 0;						// Current from sensor
uint8_t sectorCurrently 			= 0;
uint8_t sectorNext					= 0;
uint8_t sectorChanged 				= 0;

uint8_t flag_1s = 0;
uint8_t flag_timeOVF = 0;
uint8_t flag_timeMatch = 0;
uint8_t flag01 = 0;
uint8_t flag02 = 0;
uint8_t flag03 = 0;
uint8_t flag04 = 0;
uint8_t flag05 = 0;
uint8_t flag_frameStartBT = 0;
uint8_t flag_debug = 0;
uint8_t flag_Started = 0;

uint8_t debugCommand = 0;

// Communicaton variables
char inChar, aux[3], aux2[5], sInstr[15];
uint8_t rLength;
char sInstrBluetooth[15];
uint8_t j2 = 0;
uint8_t enableTranslate_Bluetooth = 0;
uint8_t enableDecode = 0;

// SIM900 Communication variables
uint8_t jSIM900 = 0;
uint8_t flag_frameStartSIM900 = 0;
char sInstrSIM900[15];
uint8_t rLengthSIM900 = 0;
uint8_t enableTranslate_SIM900 = 0;
uint8_t count_SIM900_timeout = 0;
uint8_t count_30s = 0;
uint8_t flag_SIM900_checkAlive = 0;
uint8_t enableSIM900_Send = 0;

int PRess;
int Pdig=0;

enum states01 {
	redTime,
	greenTime
};
enum states01 periodo = redTime;

//void init_SIM900()
//{
//	DDRH |= (1<<PH5);	// Reset pin
//	DDRH |= (1<<PH6);	// Power pin
//
//	PORTH &= ~(1<<PH5);
//	PORTH &= ~(1<<PH6);
//}

void init_contactors()
{
	DDRD |=  (1<<PD4);	// k1
	DDRD |=  (1<<PD5);	// k2
	DDRD |=  (1<<PD6);	// k3

	DDRD &= ~(1<<PD7);	// K1 NO input
	DDRB &= ~(1<<PB0);	// K3 NO input
	DDRB &= ~(1<<PB1);	// Thermal device protection

}
void init_ADC()
{
//	ADCSRA ==> ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);	// Set 128 division clock
	ADCSRA |= (1<<ADEN); 				// Enable module

//	ADCSRB ==>	–	ACME	–	–	MUX5	ADTS2	ADTS1	ADTS0
	ADCSRB &= ~(1<<ADTS2);				// Free running mode.
	ADCSRB &= ~(1<<ADTS1);
	ADCSRB &= ~(1<<ADTS0);

//	ADCSRB &= ~(1<<MUX5);				// To select ADC0;

//	ADMUX ==> REFS1 REFS0 ADLAR MUX4 MUX3 MUX2 MUX1 MUX0
//	ADMUX &=  ~(1<<REFS1);				// AREF, Internal Vref turned off
//	ADMUX &=  ~(1<<REFS0);

	ADMUX &=  ~(1<<REFS1);				// AVCC with external capacitor at AREF pin
	ADMUX |=   (1<<REFS0);

//	ADMUX |=   (1<<REFS1);				// Reserved
//	ADMUX &=  ~(1<<REFS0);

//	ADMUX |=  (1<<REFS0);				// Internal 1.1V Voltage Reference with external capacitor at AREF pin
//	ADMUX |=  (1<<REFS1);


//	ADMUX |=  (1<<ADLAR);				// Left Adjustment. To ADCH register.
//										// Using 8 bits. Get ADCH only.
	ADMUX &= ~(1<<ADLAR);				// Right Adjustment. To ADCL register.
										// Using 10 bits
//	ADMUX &= ~(1<<MUX4);				// Select ADC1
	ADMUX &= ~(1<<MUX3);
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX1);
	ADMUX |=  (1<<MUX0);
}
void init_Timer1_1Hz()
{
	// Timer 1 with 16 bit time counter. On a Fast PWM
	// TCCR1A <==	COM1A1	COM1A0	COM1B1	COM1B0	COM1C1	COM1C0	WGM11	WGM10
	TCCR1A = 0b00000010;

	// TCCR1B <==	ICNC1	ICES1	–		WGM13	WGM12	CS12	CS11	CS10
	TCCR1B = 0b00011101;	// Start timer at Fcpu/1024

	// TIMSK1 <==	–		–		ICIE1	–		OCIE1C	OCIE1B	OCIE1A	TOIE1
//	TIMSK1 |= (1 << OCIE1A);
	TIMSK1 = 0b00000010;

	ICR1 = 15624;	// To obtain 1Hz clock.
}
void init_WDT()
{
	// Configuring to enable only Reset System if occurs 4 s timeout
//	WDTCSR <== WDIF WDIE WDP3 WDCE WDE WDP2 WDP1 WDP0
//	WDTCSR |=  (1<<WDCE) | (1<<WDE);	// Enable Watchdog Timer
//	WDTCSR &= ~(1<<WDIE);				// Disable interrupt
//
//	WDTCSR |=  (1<<WDP3);				// 512k (524288) Cycles, 4.0s
//	WDTCSR &= ~(1<<WDP2);
//	WDTCSR &= ~(1<<WDP1);
//	WDTCSR &= ~(1<<WDP0);

//	WDTCSR |=  (1<<WDCE);
//	WDTCSR = 0b00111000;

//	wdt_enable(WDTO_8S);
	// WDT enable

	wdt_enable(WDTO_8S);
}
void stop_WDT()
{
	cli();
//	__watchdog_reset();
	/* Clear WDRF in MCUSR */
	MCUSR &= ~(1<<WDRF);
	/* Write logical one to WDCE and WDE */
	/* Keep old prescaler setting to prevent unintentional time-out */
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	/* Turn off WDT */
	WDTCSR = 0x00;
	sei();
}

double get_Pressure()
{
	/*
	Sensor details

    Thread size : G 1/4" (BSP)
    Sensor material:  Carbon steel alloy
    Working voltage: 5 VDC
    Output voltage: 0.5 to 4.5 VDC
    Working Current: <= 10 mA
    Working pressure range: 0 to  1.2 MPa
    Maxi pressure: 2.4 MPa
    Working temperature range: 0 to 100 graus C
    Accuracy: ± 1.0%
    Response time: <= 2.0 ms
    Package include: 1 pc pressure sensor
    Wires : Red---Power (+5V)  Black---Power (0V) - blue ---Pulse singal output


    4.5 V___	   922___	1.2 MPa___	 12 Bar___	 120 m.c.a.___
	  	  |				|			|			|				|
	 	  |				|			|			|				|
	 	  |				|			|			|				|
	  out_|			Pd__|		  __|			|			Pa__|
	 	  |				|			|			|				|
	 	  |				|			|			|				|
	 	  |				|			|			|				|
		 _|_		   _|_		   _|_		   _|_			   _|_
	0.5 V			103			0 MPa		0 Bar		0 m.c.a.

	(out-0.5)/(4.5-0.5) = 1024

	(out-0.0)/(5-0) = (x-0)/(1024-0)

	(Pd - 103)/(922-103) = (Pa - 0)/(120 - 0)
	Pa = 120.0*Pd/(1024.0);

	(xs - 0) = temp - (0)
	(255 - 0)  +50 - (0)

	Direct Conversion
	xs = 255*(temp+0)/51
	tempNow_XS = (uint8_t) 255.0*(tempNow+0.0)/51.0;

	Inverse Conversion
	temp = (TempMax*xs/255) - TempMin
	tempNow = (uint8_t) ((sTempMax*tempNow_XS)/255.0 - sTempMin);
    */

	uint8_t low, high;
	int Pd;
	double Pa;

	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX1);				// Select ADC0
	ADMUX &= ~(1<<MUX0);

	ADCSRA |= (1<<ADSC);				// Start conversion;
	while (bit_is_set(ADCSRA, ADSC));	// wait until conversion done;

//		Serial.println((ADCH << 8) | ADCL);
	low  = ADCL;
	high = ADCH;

	Pd = (high << 8) | low;

	Pdig = Pd;
	Pa = (120.0)*(Pd-102.4)/(921.6-102.4);
//	(Pd - 103)/(922-103) = (Pa - 0)/(120 - 0);
	return Pa;
}

void logMessage(uint8_t loadType, uint8_t newStatus)
{
	int i;

//	for(i=(nLog-1);i>0;i--)
	for(i=1; i<nLog; i++)
	{
		hourLog[i-1] = hourLog[i];
		minuteLog[i-1] = minuteLog[i];

		loadLog[i-1] = loadLog[i];
		loadStatus[i-1] = loadStatus[i];
	}
	loadLog[nLog-1] = loadType;
	loadStatus[nLog-1] = newStatus;

	hourLog[nLog-1] = tm.Hour;
	minuteLog[nLog-1] = tm.Minute;


//	for(i=(nLog-1);i>0;i--)
//	{
//		hourLog[i] = hourLog[i-1];
//		minuteLog[i] = minuteLog[i-1];
//
//		loadLog[i] = loadLog[i-1];
//		loadStatus[i] = loadStatus[i-1];
//	}
//	loadLog[0] = loadType;
//	loadStatus[0] = newStatus;
//
//	hourLog[0] = tm.Hour;
//	minuteLog[0] = tm.Minute;
}

void nodeValve_setInstr(uint8_t sectorPrivate, uint8_t status, uint8_t makeLog)
{
	uint8_t type = 0;
	uint8_t size = 4;
	if((!sectorPrivate) && (!status) &&  (!makeLog))
	{
		type = 3;

		Wire.beginTransmission(8);

		Wire.write(type);			// 0
		Wire.write(size);			// 1
		Wire.write(0x00);			// 2
		Wire.write(0x00);			// 3

		Wire.endTransmission();
		_delay_ms(500);								// wait a little bit for sector get stable
	}
	else
	{
		type = 1;

		Wire.beginTransmission(8);

		Wire.write(type);			// 0
		Wire.write(size);			// 1
		Wire.write(sectorPrivate);	// 2
		Wire.write(status);			// 3

		Wire.endTransmission();
		_delay_ms(500);								// wait a little bit for sector get stable

		if(flag_debug)
		{
			Serial.println("");
			Serial.print("type: ");
			Serial.println(type);
			Serial.print("size: ");
			Serial.println(size);
			Serial.print("sectorPrivate: ");
			Serial.println(sectorPrivate);
			Serial.print("status");
			Serial.println(status);
		}
	}

	if(makeLog)
	{
		logMessage(sectorPrivate, status);
	}
}
void nodeValve_setIs(uint16_t maxIs)
{
	uint8_t type = 2;
	uint8_t size = 4;

	uint8_t lbyte_valveSensor = maxIs;
	uint8_t hbyte_valveSensor = maxIs >> 8;

	Wire.beginTransmission(8);

	Wire.write(type);				// 0
	Wire.write(size);				// 1
	Wire.write(lbyte_valveSensor);	// 2
	Wire.write(hbyte_valveSensor);	// 3

	Wire.endTransmission();
	_delay_ms(500);								// wait a little bit for sector get stable
}
void nodeValve_getSettings()
{
	Wire.requestFrom(8, 15);		// request 13 bytes from slave device 8
	int k = 0;
	uint8_t cur[4];

	while(Wire.available())	// slave may send less than requested
	{
		if(k < 11)
			valveStatus[k] = Wire.read();
		else
			cur[k-11] = Wire.read();
		k++;
	}
	_delay_ms(500);

	valveIs = (cur[1] << 8) | cur[0];
	nodeValve_maxIs = (cur[3] << 8) | cur[2];

}

void motor_stop()
{
	k1_off();
	k2_off();
	k3_off();
	logMessage(12, 0);
//	motorStatus = 0;
}

void turnAll_OFF_init()
{
	motor_stop();
	nodeValve_setInstr(0, 0, 0);

	flag_timeMatch = 0;
	timeSector = 0;
	sectorCurrently = 0;
	flag_Started = 0;
}
void turnAll_OFF(uint8_t error)
{
	motor_stop();
	nodeValve_setInstr(0, 0, 0);

	flag_timeMatch = 0;
	flag_Started = 0;
	sectorCurrently = 0;
	sectorNext = 0;
	timeSector = 0;

	stateMode = 0;	// Manual
	eeprom_write_byte(( uint8_t *)(addr_stateMode), stateMode);

	lastError = error;
	Serial.print("er:");
	Serial.println(error);
	eeprom_write_byte((uint8_t *)(addr_lastError), lastError);
}
void motor_start()
{
//	switch (modeS)
//	{
//		case 0:
//			break;
//
//		case 1:
//			k1_on();
//			k3_on();
//			logMessage(12, 1);
//			break;
//
//		case 2:
			k1_on();
			k3_on();
			wdt_reset();
			_delay_ms(((double) 100.0*motorTimerStart1));
			wdt_reset();

			k3_off();
			uint32_t count = 0;
			while(k3_readPin)
			{
				count++;
				if(count>=120000)
				{
					k1_off();
					k2_off();
					k3_off();
					turnAll_OFF(0x06);
					return;
				}
			}
//			Serial.print("FuCK: ");
//			Serial.println(count);
			_delay_ms(motorTimerStart2);
			k2_on();

			// carga, estado;
			logMessage(12, 1);
		//	Serial.print("Count = ");
		//	Serial.println(count);
//			break;
//
//		default:
//			break;
//	}
}

void summary_Print(uint8_t opt)
{
	char buffer[80];
	int i;

	switch (opt)
	{
		case 0:		// main data. Date, clock, state, mode, period
			sprintf(buffer,"%.2d:%.2d:%.2d", tm.Hour, tm.Minute, tm.Second);
			Serial.println(buffer);
			sprintf(buffer," %.2d/%.2d/%d ", tm.Day, tm.Month, tmYearToCalendar(tm.Year));
			Serial.println(buffer);
			sprintf(buffer,"m:%d p:%d tm:%d", stateMode, periodState, flag_timeMatch);
			Serial.println(buffer);

			sprintf(buffer," d:%d", (day()-1));
			Serial.println(buffer);
			sprintf(buffer,"%.2d:%.2d:%.2d", hour(), minute(), second());
			Serial.println(buffer);

//			Serial.print("Uptime: ");
//			sprintf(buffer,"%.2d:%.2d:%.2d", hour(), minute(), second());
//			Serial.println(buffer);
//			sprintf(buffer,"d:%d, m:%d, y:%d", (day()-1), (month()-1), (year()-1970));
//			Serial.println(buffer);
			break;

		case 1:		// time of sectors
			for(i=0;i<11;i++)
			{
				sprintf(buffer,"t%d:%d, ",i+1, timeSectorVectorMin[i]);
				Serial.print(buffer);
			}
			Serial.println("");

			for(i=0;i<11;i++)
			{
				sprintf(buffer,"s%d:%d, ",i+1, valveSequence[i]);
				Serial.print(buffer);
			}
			Serial.println("");
			for(i=0;i<11;i++)
			{
				sprintf(buffer,"v%d:%d, ",i+1, valveStatus[i]);
				Serial.print(buffer);
			}
			Serial.println("");
			Serial.println("");
//			sprintf(buffer,"t1:%d, t2:%d, t3:%d", timeSectorVectorMin[0], timeSectorVectorMin[1], timeSectorVectorMin[2]);
//			Serial.println(buffer);
//			sprintf(buffer,"t4:%d, t5:%d, t6:%d", timeSectorVectorMin[3], timeSectorVectorMin[4], timeSectorVectorMin[5]);
//			Serial.println(buffer);
//			sprintf(buffer,"t7:%d, t8:%d, t9:%d", timeSectorVectorMin[6], timeSectorVectorMin[7], timeSectorVectorMin[8]);
//			Serial.println(buffer);
//			sprintf(buffer,"t10:%d, t11:%d", timeSectorVectorMin[9],timeSectorVectorMin[10]);
//			Serial.println(buffer);
			break;

		case 2:		// wich valve will open in the process
			sprintf(buffer,"le:%d s%d Pr:%d Is:%d ts:%d Ep:%d", lastError, sectorCurrently, PRess, valveIs, timeSector, count_timePipeB);
			Serial.println(buffer);

//			Serial.println(buffer);
//			sprintf(buffer,"s5:%d, s6:%d, s7:%d, s8:%d", valveStatus[4], valveStatus[5], valveStatus[6], valveStatus[7]);
//			Serial.println(buffer);
//			sprintf(buffer,"s9:%d, s10:%d, s11:%d", valveStatus[8], valveStatus[9],valveStatus[10]);
//			Serial.println(buffer);
			break;

		case 3:		// motor variables
			sprintf(buffer,"M:%d Pr:%d Rth:%d Th%d Ka%d Kc%d", motorStatus, PRess, Th_readPin, Th_readPin, k1_readPin, k3_readPin);
			Serial.println(buffer);
			break;

		case 4:		// Valves info
			sprintf(buffer,"s%d Is%d c%d ft:%d", sectorCurrently, valveIs, sectorChanged, flag_timeOVF);
			Serial.println(buffer);
			break;

		case 5:		// Time clock to match process
			for(i=0;i<nTM;i++)
			{
				sprintf(buffer,"h%d: %.2d:%.2d",i+1, HourOnTM[i], MinOnTM[i]);
				Serial.println(buffer);
			}
			break;

		case 6:		// History
			for(i=(nLog-1); i>=0; i--)
			{
				sprintf(buffer,"%.2d-L%.2d|%d|%.2d:%.2d ",i+1, loadLog[i], loadStatus[i], hourLog[i], minuteLog[i]);
				Serial.print(buffer);
				Serial.println("");
			}
			break;

		case 7: 	// Reference variables
			sprintf(buffer,"le%d Pmin:%d Pmax:%d Imin:%d Imax:%d ImaxN:%d tPB:%d m1:%d m2:%d", lastError, minPRess, maxPRess, minIs, maxIs, nodeValve_maxIs, timePipeB, motorTimerStart1, motorTimerStart2);
			Serial.println(buffer);
			break;

		case 8:		// Reset flags
			sprintf(buffer,"W%d B%d E%d P%d", flag_WDRF, flag_BORF, flag_EXTRF, flag_PORF);
			Serial.println(buffer);
			break;

		case 21:
			Serial.print("0x0");
			Serial.println(1,HEX);
			break;

		case 22:
			Serial.print("0x0");
			Serial.println(2,HEX);
			break;

		case 23:
			Serial.print("0x0");
			Serial.println(3,HEX);
			break;

		case 24:
			Serial.print("0x0");
			Serial.println(4,HEX);
			break;

		case 25:
			Serial.print("0x0");
			Serial.println(5,HEX);
			break;

		case 26:
			Serial.print("0x0");
			Serial.println(6,HEX);
			break;

		case 27:
			Serial.print("0x0");
			Serial.println(7,HEX);
			break;

		case 28:
			Serial.print("0x0");
			Serial.println(8,HEX);
			break;

		case 29:
			Serial.print("0x0");
			Serial.println(9,HEX);
			break;

		case 30:
			Serial.print("Done");
			break;

		default:
			break;
	}
	memset(buffer,0,sizeof(char));
}

uint16_t timeSectorMemory(uint8_t sectorPrivate)
{
	return 60*timeSectorVectorMin[sectorPrivate-1];
}

void valveIs_check()
{
	if(valveIs < minIs)
	{
		turnAll_OFF(0x03);
	}
}
void valveOpenVerify()
{
	wdt_reset();
	int i, valveWorking = 0;

	for(i=0; i<11; i++)
	{
		valveWorking |= valveStatus[i];
	}

	if(!valveWorking)
	{
		turnAll_OFF(0x01);
	}
}
void thermalSafe()
{
	if(Th_readPin)
	{
		uint16_t countThermal = 50000;
//		Serial.println("A");
		while(Th_readPin && countThermal)
		{
			countThermal--;
		}
//		Serial.println("B");
		if(Th_readPin && !countThermal)
		{
			turnAll_OFF(0x02);
//			summary_Print(22);
		}
	}
}
void pipeBrokenSafe()
{
	if(flag_BrokenPipeVerify)
	{
		if(PRess < minPRess)
		{
			turnAll_OFF(0x04);
	//		summary_Print(24);
	//		Serial.println("PRessure Down!");
		}
	}
}
void pipePRessSafe()
{
	if(PRess>=maxPRess)
	{
		turnAll_OFF(0x05);
//		summary_Print(25);
//		Serial.println("PRessure HIGH!");
	}
}

// 20160204 - rever essa lógica!
void process_Working()
{
	if(flag_timeOVF)	// down counter on TIMER 1 interrupt function
	{
		uint8_t flag_sectorActiveFound = 0;
//		uint8_t sectorNext = (sectorCurrently + 1);
		sectorNext++;

		uint8_t A = 0;
		uint8_t B = 0;
		uint8_t C = 0;

		if(flag_debug)
		{
			Serial.print("sectorNext: ");
			Serial.println(sectorNext);
		}

		// encontrar proximo setor no vetor AND funcionando;
		if(sectorNext <= valveAlloc)	//
		{
			do{
				if(flag_debug)
					Serial.println("do");

				wdt_reset();
				if(valveSequence[sectorNext-1])
				{
					float I1 = (float) valveIs;			// get I1

					nodeValve_setInstr(sectorNext, 1, 1);		// turn on the next sector
					nodeValve_getSettings();					// ask for new I2 current
					float I2 = (float) valveIs;
					float I3 = I1*1.4;

					if(flag_debug)
					{
						Serial.print("I1:");
						Serial.print(I1);
						Serial.print(" I2:");
						Serial.print(I2);
						Serial.print(" I3:");
						Serial.println(I3);
					}

					if((I2 > I3) && (I2 > minIs))
					{
						if(flag_debug)
							Serial.println("Found!");

						flag_sectorActiveFound = 1;

						nodeValve_setInstr(sectorCurrently, 0, 1);
						sectorCurrently = sectorNext;
						timeSector = timeSectorMemory(sectorCurrently);
						count_timePipeB = timePipeB;
						flag_BrokenPipeVerify = 0;
						flag_timeOVF = 0;

//						if(flag_debug)
//						{
							summary_Print(4);
//						}

						if(!k1_readPin)
						{
							motor_start();
						}
					}
					else
					{
						nodeValve_setInstr(sectorNext, 0, 1);		// turn off the next sector
						flag_sectorActiveFound = 1;
						if(flag_debug)
						{
							Serial.print("sNext1: ");
							Serial.println(sectorNext);
						}

//						sectorNext++;
//						if(sectorNext > valveAlloc)
//						{
//						c	sectorCurrently = 12;
////							flag_sectorActiveFound = 1;
////							sectorCurrently = valveAlloc + 1;
//						}

						if(flag_debug)
						{
							Serial.print("sNext2: ");
							Serial.println(sectorNext);

							Serial.print("sCur2: ");
							Serial.println(sectorCurrently);
						}
					}
				}
				else
				{
					flag_sectorActiveFound = 1;
				}
//				else
//				{
//					nodeValve_setInstr(sectorNext, 0, 1); // should not have this line here
//					sectorNext++;

//					if(sectorNext > valveAlloc)
//					{
//						sectorCurrently = 12;
////						flag_sectorActiveFound = 1;
////						sectorCurrently = valveAlloc + 1;
//					}
//					if(flag_debug)
//					{
//						Serial.print("sNext3: ");
//						Serial.println(sectorNext);
//
//						Serial.print("sCur3: ");
//						Serial.println(sectorCurrently);
//					}
//				}

				// Sai do loop se flag_sectorActiveFound = 1
				// OU
				// se atinge setor maior do que valveAlloc;

				A = !flag_sectorActiveFound;
				B = !(sectorNext > valveAlloc);
				C = A && B;

//				A = !(flag_sectorActiveFound && valveSequence[sectorNext-1]);
//				B = (sectorNext <= valveAlloc);
//				C = A && B;

	//			Serial.print("SectorC: ");
	//			Serial.println(sectorCurrently);
	//			Serial.print("SectorN: ");
	//			Serial.println(sectorNext);
	//			Serial.print("A: ");
	//			Serial.println(A);
	//			Serial.print("B: ");
	//			Serial.println(B);
	//			Serial.print("C: ");
	//			Serial.println(C);

			}while(C);
//			Serial.print("sectorCurrently: ");
//			Serial.println(sectorCurrently);
	//		}while(!(flag_sectorActiveFound && valveSequence[sectorNext]) || (sectorNext <= valveAlloc));	// keep here while not find sector or max sextor allocated
		}
		else
		{
			turnAll_OFF(0x10);
			uint32_t count = 0;
			while(k1_readPin)
			{
				count++;
				if(count>=120000)
				{
					k1_off();
					k2_off();
					k3_off();
					turnAll_OFF(0x06);
					return;
				}
			}

			if(flag_debug)
			{
				Serial.println("Done!");
			}
		}
	}
}
void process_Programmed()
{
	if(flag_timeMatch)
	{
		flag_timeMatch = 0;//^= flag_timeMatch;
		lastError = 0x00;
		flag_Started = 1;
	}

	if(flag_Started)
		process_Working();
}
void process_valveTest()
{
	wdt_reset();
	nodeValve_setInstr(sectorCurrently, 1, 0);
	_delay_ms(4000);
	wdt_reset();
	nodeValve_setInstr(sectorCurrently, 0, 0);
	_delay_ms(4000);
}

void process_Mode()
{
	switch(stateMode)
	{
		case 0:	// Manual Process
//			process_Manual();
			break;

		case 1: // nightMode
			process_Programmed();
			break;

		case 9:
			process_valveTest();
			break;

		default:
			stateMode = 0;
			break;

//		case 2:
//			process_Working();
//			break;

//		case 3: //onlyOneSector. Turn on at night only one sector
//			process_Programmed();
//			break;

//		case 4: //valveTesting
			// Automatic Process
//			process_valveTest();
//			break;
	}
}

void check_period()				// Season time verify
{
	if(((tm.Hour == HourOn) && (tm.Minute == MinOn)) || (tm.Hour > HourOn) || (tm.Hour < HourOff) || ((tm.Hour == HourOff) && (tm.Minute < MinOff)))
	{
		periodo = greenTime;

		if(flag01)
		{
			flag01 = 0;
		}
	}

	if (((tm.Hour == HourOff) && (tm.Minute >= MinOff))	|| ((tm.Hour > HourOff) && (tm.Hour < HourOn))	|| ((tm.Hour == HourOn) && (tm.Minute < MinOn)))
	{
		periodo = redTime;

		flag01 = 1;
	}
}
void check_timeMatch()			// matching time verify
{
	uint8_t i, nTM_var=0;
	if(!motorStatus)
	{
//		Serial.println("A");
		if(stateMode)
		{
//			Serial.println("B");
			switch (stateMode)
			{
			case 1:
//				Serial.println("C");
				nTM_var = 1;
				break;

			case 2:
				nTM_var = nTM;
				break;
			}

			for(i=0;i<nTM_var;i++)
			{
//				Serial.println("D");
				if((tm.Hour == HourOnTM[i]) && (tm.Minute == MinOnTM[i]))
				{
//					Serial.println("E");
					flag_timeMatch = 1;
				}
			}
		}
	}
}

void refreshVariables()
{
	if (flag_1s)
	{
		flag_1s = 0;

		motorStatus = k1_readPin;		// Motor status refresh
		nodeValve_getSettings();		// Get valves info
		PRess = get_Pressure();			// Get pipe pressure

		if(k1_readPin)
		{
			thermalSafe();				// thermal safe check;
			valveOpenVerify();			// AND op with all output valves;
			pipePRessSafe();			// Verify maximum pressure;
			valveIs_check();			// check if there is current on Is sensor;
			pipeBrokenSafe();			// Verify pipe low pressure;
		}

		RTC.read(tm);				// RTC fetch;
		check_period();
		check_timeMatch();

		if(flag_debug)
		{
			summary_Print(debugCommand);
		}
	}
}
void refreshSectorSetup()
{
	int i;
	for(i=0;i<11;i++)
		timeSectorVectorMin[i] = eeprom_read_byte((uint8_t *)(i+addr_timeSector));

	for(i=0;i<11;i++)
		valveSequence[i] = eeprom_read_byte((uint8_t *)(i+addr_valveSequence));
}
void refreshStoredData()
{
	refreshSectorSetup();				// Refresh variables

	stateMode = eeprom_read_byte((uint8_t *)(addr_stateMode));
	lastError = eeprom_read_byte((uint8_t *)(addr_lastError));

//	flag_iValveCheck = eeprom_read_byte((uint8_t *)(addr_iValveCheck));
//	powerOn_min_Standy = eeprom_read_byte((uint8_t *)(addr_standBy_min));
//	powerOn_min = powerOn_min_Standy;
//	Iconst = (eeprom_read_byte((uint8_t *)(addr_Iper)))/100.0 + 1.0;

	minPRess = eeprom_read_byte((uint8_t *)(addr_minPRess));
	maxPRess = eeprom_read_byte((uint8_t *)(addr_maxPRess));

	uint8_t lbyte, hbyte;
	hbyte = eeprom_read_byte((uint8_t *)(addr_minIs+1));
	lbyte = eeprom_read_byte((uint8_t *)(addr_minIs));
	minIs = ((hbyte << 8) | lbyte);

	hbyte = eeprom_read_byte((uint8_t *)(addr_maxIs+1));
	lbyte = eeprom_read_byte((uint8_t *)(addr_maxIs));
	maxIs = ((hbyte << 8) | lbyte);

	hbyte = eeprom_read_byte((uint8_t *)(addr_timePipeB+1));
	lbyte = eeprom_read_byte((uint8_t *)(addr_timePipeB));
	timePipeB = ((hbyte << 8) | lbyte);

//	hbyte = eeprom_read_byte((uint8_t *)(addr_motorTimerStart1+1));
//	lbyte = eeprom_read_byte((uint8_t *)(addr_motorTimerStart1));
//	motorTimerStart1 = ((hbyte << 8) | lbyte);
	motorTimerStart1 = eeprom_read_byte((uint8_t *)(addr_motorTimerStart1));

	hbyte = eeprom_read_byte((uint8_t *)(addr_motorTimerStart2+1));
	lbyte = eeprom_read_byte((uint8_t *)(addr_motorTimerStart2));
	motorTimerStart2 = ((hbyte << 8) | lbyte);

	nTM = eeprom_read_byte((uint8_t *)(addr_nTM));
	uint8_t i;
	for(i=0;i<9;i++)
	{
		HourOnTM[i] = eeprom_read_byte((uint8_t *)(addr_HourOnTM+i));
		MinOnTM[i] = eeprom_read_byte((uint8_t *)(addr_MinOnTM+i));
	}
}

void comm_Bluetooth()
{
	// Rx - Always listening
//	uint8_t j2 =0;
	while((Serial.available()>0))	// Reading from serial
	{
		inChar = Serial.read();

		if(inChar=='$')
		{
			j2 = 0;
			flag_frameStartBT = 1;
//			Serial.println("Frame Start!");
		}

		if(flag_frameStartBT)
			sInstrBluetooth[j2] = inChar;

//		sprintf(buffer,"J= %d",j2);
//		Serial.println(buffer);

		j2++;

		if(j2>=sizeof(sInstrBluetooth))
		{
			memset(sInstrBluetooth,0,sizeof(sInstrBluetooth));
			j2=0;
			summary_Print(28);
		}

		if(inChar==';')
		{
//			Serial.println("Encontrou ; !");
			if(flag_frameStartBT)
			{
//				Serial.println("Frame Stop!");
				flag_frameStartBT = 0;
				rLength = j2;
				j2 = 0;
				enableTranslate_Bluetooth = 1;
			}
		}
	}
//	flag_frameStart = 0;

	if(enableTranslate_Bluetooth)
	{
//		Serial.println("enableTranslate_Bluetooth");
		enableTranslate_Bluetooth = 0;

		char *pi0, *pf0;
		pi0 = strchr(sInstrBluetooth,'$');
		pf0 = strchr(sInstrBluetooth,';');

		if(pi0!=NULL)
		{
			uint8_t l0=0;
			l0 = pf0 - pi0;

			int i;
			for(i=1;i<=l0;i++)
			{
				sInstr[i-1] = pi0[i];
//				Serial.write(sInstr[i-1]);
			}
//			memset(sInstrBluetooth,0,sizeof(sInstrBluetooth));
	//		Serial.println(sInstr);

			enableDecode = 1;
		}
		else
		{
			summary_Print(29);
			Serial.write(pi0[0]);
			Serial.write(pf0[0]);
		}
		memset(sInstrBluetooth,0,sizeof(sInstrBluetooth));
	}
}
void comm_SIM900()
{
	// Rx - Always listening
//	uint8_t j1 =0;
	while((Serial.available()>0))	// Reading from serial
	{
		inChar = Serial.read();
		Serial.write(inChar);

		if(inChar=='$')
		{
			jSIM900 = 0;
			memset(sInstrSIM900,0,sizeof(sInstrSIM900));
			flag_frameStartSIM900 = 1;
//			Serial.println("Frame Start!");
		}

		if(flag_frameStartSIM900)
		{
			sInstrSIM900[jSIM900] = inChar;
//			Serial.write(sInstrSIM900[j1]);
		}

		jSIM900++;

		if(jSIM900>=sizeof(sInstrSIM900))
		{
			memset(sInstrSIM900,0,sizeof(sInstrSIM900));
			jSIM900=0;
			Serial.println("ZEROU! sIntr SIM900 Buffer!");
		}

		if(inChar==';')
		{
			if(flag_frameStartSIM900)
			{
//				Serial.println("Frame Stop!");
				flag_frameStartSIM900 = 0;
				rLengthSIM900 = jSIM900;
//				j1 = 0;
				enableTranslate_SIM900 = 1;
			}
		}


		// Variables for check if SIM900 is alive.
		flag_SIM900_checkAlive = 0;
		count_SIM900_timeout = 0;
		count_30s = 0;

//		if(flag_SIM900_checkAlive)
//		{
//			if(inChar=='K')
//			{
//				enableSIM900_checkAliveCompare = 1;
//				flag_SIM900_checkAlive = 0;
//				count_SIM900_timeout = 0;
//				j1 = 0;
//			}
//		}
	}

	// PC to SIM900
	while(Serial.available() > 0)
		Serial.write(Serial.read());

	if(enableTranslate_SIM900)
	{
		jSIM900 = 0;
		enableTranslate_SIM900 = 0;

		char *pi1, *pf1;
		pi1 = strchr(sInstrSIM900,'$');
		pf1 = strchr(sInstrSIM900,';');

		if(pi1!=NULL)
		{
//			Serial.println("pi!=NULL");
			uint8_t l1=0;
			l1 = pf1 - pi1;

			int i;
			for(i=1;i<=l1;i++)
			{
				sInstr[i-1] = pi1[i];
//				Serial.write(sInstr[i-1]);
			}
			memset(sInstrSIM900,0,sizeof(sInstrSIM900));
			Serial.println(sInstr);

			enableDecode = 1;
			enableSIM900_Send = 1;
		}
		else
		{
			Serial.println("Error 404!");
			Serial.write(pi1[0]);
			Serial.write(pf1[0]);
		}
	}

	// Special Functions  CHECK ALIVE!
//	if(flag_SIM900_checkAlive)
//	{
//		if(count_SIM900_timeout > 5)
//		{
//			flag_SIM900_checkAlive = 0;
//			count_SIM900_timeout = 0;
//			Serial.println("SIM900 Check Alive TIMEOUT!");
//
//			SIM900_power();
//			if((minute()*60 + second())<90)
//			{
//				sprintf(buffer,"- Vassal Controller Started! -");
//				SIM900_sendSMS(buffer);
//			}
//		}
//	}

//	if(enableSIM900_checkAliveCompare)
//	{
//		enableSIM900_checkAliveCompare = 0;
//		j1 = 0;
//
//		char *p;
//		p = strchr(sInstrSIM900,'O');
//
//		if(p[0] == 'O' && p[1] == 'K')
//		{
//			flag_SIM900_died = 0;
//			Serial.println("Alive!");
//		}
//		else
//		{
//			Serial.println("Is DEAD??");
////			flag_SIM900_died = 1;
//		}
//	}
}
void comm_SerialPC()
{

}
void comm_SIM900_SerialPC()
{
	// SIM900 to PC
	while(Serial.available() > 0)
		Serial.write(Serial.read());

	// PC to SIM900
	while(Serial.available() > 0)
		Serial.write(Serial.read());
}
void comm_SIM900_Bluetooth()
{
	while(Serial.available() > 0)
		Serial.write(Serial.read());

	while(Serial.available() > 0)
		Serial.write(Serial.read());
}
void comm_SerialPC_BluetoothMOD()
{
	while(Serial.available() > 0)
		Serial.write(Serial.read());

	while(Serial.available() > 0)
		Serial.write(Serial.read());
}

void handleMessage()
{
/*
Comandos do VASSALO

Ínicio do comando sempre com dólar $
Fim do comando sempre com ponto vírgula ;

$0X;				Verificar detalhes - Detalhes simples (tempo).
	$00;			- Mostra o tempo ajustado de todos setores.
		$00:0;		- flag_sendContinuously = 0
		$00:1;		- flag_sendContinuously = 1
	$01;			- Mostra o tempo de cada setor;
	$02;			- Mostra qual setor entra na sequencia de irrigacao;
	$03;			- Variáveis do motor;
	$04;			- Setor atual, mudança de estado, tempo restante no setor;
	$05;			- Motivo da reinicialização;
	$06;			- Ultimo erro, leitura de contatores e rele termico;
	$07;			- Histórico Liga/Desliga das cargas;
	$08;			- Corrente no sensor das valvulas;
		$08:15;		- 15%. Valor de referência do sensor de corrente.
		$08:00;		- Não verifica sensor de corrente das válvulas;
	$09;			- Reinicia o sistema.

$1HHMMSS;		Ajusta o horário do sistema;
	$1123040;		Ajustar a hora para 12:30:40

$2DDMMAAAA;  		Ajusta a data do sistema no formato dia/mês/ano(4 dígitos);
	$201042014;	Ajusta a data para 01 de abril de 2014;

$3X;			Acionamento do motor;
	$31;		liga (CUIDADO! Verifique se há válvula aberta antes de acionar o motor!);
	$30;		desliga;

$4sNN:V;		Modo seguro de acionamento (somente para válvulas dos piquetes);
	$4c01:1;	- Liga setor 1 e desliga o anterior ligado;
	$4f02:1;	- Liga setor 2 (mesmo que tenha outro ligado);
	$4t00:30;	- Configura todos setores para 30 minutos;
	$4t02:47;	- Configura tempo do setor 2 para 47 minutos;
	$4s02:1;	- Configura setor 2 para ligar;
	$4s00:1;	- Configura todos setores para ligar;
	$4s00:0;	- Desconfigura todos setores;
	$4s03:0;	- Configura setor 3 para não ligar;

	$4z:0123;	- Configura a variável timeSector para 123 segundos;

$5sNN:MM;		Ativa o setor para sequencia de irrigação;
	$5:h1:2130;		- configura primeiro horario de acionamento (h1) para às 21:30 horas;
	$5:h1:0530;		- configura primeiro horario de acionamento (h1) para às 05:30 horas;
	$5:n:2;			- habilita somente os 2 primeiros horários programados;

$6X;			Modo de funcionamento
	$60; 		- Coloca no modo manual (desligado). DESLIGA TODAS AS CARGAS!;
	$61;		- Programa para ligar às X horas;

	$69:0:01;	- Funcao
	$69:1:02


	$69:s03;	- Testa o setor 3 se está funcionando e retorna SMS;

$7:				Parâmetros
	$7:i1:050;		- Corrente [mA] mínima para funcionamento em regime permanente;
	$7:i2:200;		- Corrente [mA] máxima aceitável no solenoide;
	$7:p1:050;		- Pressão [m.c.a.] mínima na tubulação em regime permanente;
	$7:p2:070;		- Pressão [m.c.a.] máxima instantânea na tubulação;
	$7:t1:180;		- Tempo para ligar alarme de baixa pressão [s];
	$7:m1:56;		- Tempo partida estrela triangulo sendo 56*100 ms ou 5,6 s. Aceitando no máximo 6,5 segundos;
	$7:m1:65;		- Máximo tempo de troca dos contatores devido máxima variável para função _delay_ms();
	$7:m2:250;		- Tempo partida estrela triangulo [ms];


$727988081875;		Troca número de telefone
*/

	// Tx - Transmitter
	if(enableDecode)
	{
		enableDecode = 0;

//		int i;
//		for(i=0;i<rLength;i++)
//		{
//			Serial.println(sInstr[i]);
//		}
//		for(i=0;i<rLength;i++)
//		{
//			Serial.println(sInstr[i],HEX);
//		}

		// Getting the opcode
		aux[0] = '0';
		aux[1] = sInstr[0];
		aux[2] = '\0';
		uint8_t opcode = (uint8_t) atoi(aux);

		switch (opcode)
		{
// --------------------------------------------------------------------------------------
			case 0:	// Check status
			{
				aux[0] = '0';
				aux[1] = sInstr[1];
				aux[2] = '\0';
				uint8_t statusCommand = (uint8_t) atoi(aux);

				if(sInstr[2] == ';')
				{
					switch (statusCommand)
					{
//						case 7:
//							SIM900_power();
//							Serial.println("SIM900 Power!");
//						break;

						case 9:	// Reset system
//								Serial.println("Rebooting system...");
								wdt_enable(WDTO_15MS);
								_delay_ms(100);
							break;

						default:
							summary_Print(statusCommand);
							break;
					}
				}
				else if(sInstr[2] == ':' && sInstr[4] == ';')	// $00:1;
				{
					aux[0] = '0';
					aux[1] = sInstr[3];
					aux[2] = '\0';
					flag_debug = (uint8_t) atoi(aux);
					debugCommand = statusCommand;
//					switch (statusCommand)
//					{
//						case 8:
//							aux[0] = sInstr[3];
//							aux[1] = sInstr[4];
//							aux[2] = '\0';
//							uint8_t Iper = (uint8_t) atoi(aux);
//
//							if(Iper)
//							{
//								Iconst = Iper/100.0 + 1.0;
//								Serial.println(Iconst);
//								eeprom_write_byte((uint8_t *)(addr_Iper), Iper);
//
//								flag_iValveCheck = 1;
//								eeprom_write_byte((uint8_t *)(addr_iValveCheck), flag_iValveCheck);
//							}
//							else
//							{
//								flag_iValveCheck = 0;
//								eeprom_write_byte((uint8_t *)(addr_iValveCheck), flag_iValveCheck);
//							}
//
//							break;
//					}

				}
			}
			break;
// --------------------------------------------------------------------------------------
			case 1:	// Set-up clock
			{
				// Getting the parameters
				aux[0] = sInstr[1];
				aux[1] = sInstr[2];
				aux[2] = '\0';
				tm.Hour = (uint8_t) atoi(aux);

				aux[0] = sInstr[3];
				aux[1] = sInstr[4];
				aux[2] = '\0';
				tm.Minute = (uint8_t) atoi(aux);

				aux[0] = sInstr[5];
				aux[1] = sInstr[6];
				aux[2] = '\0';
				tm.Second = (uint8_t) atoi(aux);

				RTC.write(tm);

				summary_Print(0);
			}
				break;
// -----------------------------------------------------------------
			case 2:	// Set-up date
				// Getting the parameters
				aux[0] = sInstr[1];
				aux[1] = sInstr[2];
				aux[2] = '\0';
				tm.Day = (uint8_t) atoi(aux);

				aux[0] = sInstr[3];
				aux[1] = sInstr[4];
				aux[2] = '\0';
				tm.Month = (uint8_t) atoi(aux);

				char aux2[5];
				aux2[0] = sInstr[5];
				aux2[1] = sInstr[6];
				aux2[2] = sInstr[7];
				aux2[3] = sInstr[8];
				aux2[4] = '\0';
				tm.Year = (uint8_t) (atoi(aux2)-1970);

				RTC.write(tm);

				summary_Print(0);

				break;
// -----------------------------------------------------------------
			case 3:	// Set motor ON/OFF
				uint8_t motorCommand;

				aux[0] = '0';
				aux[1] = sInstr[1];
				aux[2] = '\0';
				motorCommand = (uint8_t) atoi(aux);

				if (motorCommand&&(!motorStatus))
				{
					motor_start();
				}
				else
					motor_stop();

				summary_Print(3);
				break;
// -----------------------------------------------------------------
			case 4:	// ON OFF sectors
				if(sInstr[1] == 'f' && sInstr[4]==':' && sInstr[6]==';')	// $4f02:1; (turn ON sector 2)
				{
					aux[0] = sInstr[2];
					aux[1] = sInstr[3];
					aux[2] = '\0';
					uint8_t sector = (uint8_t) atoi(aux);

					aux[0] = '0';
					aux[1] = sInstr[5];
					aux[2] = '\0';
					uint8_t sectorCommand = (uint8_t) atoi(aux);

					nodeValve_setInstr(sector, sectorCommand, 1);
					_delay_ms(250);
					nodeValve_getSettings();
					sectorCurrently = sector;
					count_timePipeB = timePipeB;
					flag_BrokenPipeVerify = 0;

					summary_Print(2);
				}
				if(sInstr[1] == 'c' && sInstr[4]==':' && sInstr[6]==';')	// $4c03:1; (turn ON sector 3)
				{
					aux[0] = sInstr[2];
					aux[1] = sInstr[3];
					aux[2] = '\0';
					uint8_t sector = (uint8_t) atoi(aux);

					aux[0] = '0';
					aux[1] = sInstr[5];
					aux[2] = '\0';
					uint8_t sectorCommand = (uint8_t) atoi(aux);

					nodeValve_setInstr(sector, sectorCommand, 1);
					if(sectorCommand)
					{
						nodeValve_setInstr(sectorCurrently, 0, 1);
						sectorCurrently = sector;
						count_timePipeB = timePipeB;
						flag_BrokenPipeVerify = 0;
					}
					_delay_ms(250);
					nodeValve_getSettings();
					summary_Print(2);
				}
				if(sInstr[1] == 'z' && sInstr[2]==':' && sInstr[7]==';')	// 4z:0123;
				{
					aux2[0] = sInstr[3];
					aux2[1] = sInstr[4];
					aux2[2] = sInstr[5];
					aux2[3] = sInstr[6];
					aux2[4] = '\0';

					timeSector = (uint16_t) atoi(aux2);
				}

				if(sInstr[1] == 't' && sInstr[4]==':' && sInstr[7]==';')	// 4t01:23;
				{
					// Get sector number
					aux[0] = sInstr[2];
					aux[1] = sInstr[3];
					aux[2] = '\0';
					uint8_t sector = (uint8_t) atoi(aux);

					// get sector time
					uint8_t sectorTimeChange;
					aux[0] = sInstr[5];
					aux[1] = sInstr[6];
					aux[2] = '\0';
					sectorTimeChange = (uint8_t) atoi(aux);

					if(!sector)
					{
						int i;
						for(i=0;i<valveAlloc;i++)
						{
							eeprom_write_byte((uint8_t *)(i+addr_timeSector), sectorTimeChange);
						}
					}
					else
					{
						eeprom_write_byte(( uint8_t *)(sector-1+addr_timeSector), sectorTimeChange);
					}
					refreshSectorSetup();
					summary_Print(1);
				}
				if(sInstr[1] == 's' && sInstr[4]==':' && sInstr[6]==';')	//	4s02:0;
				{
					// Get sector number
					aux[0] = sInstr[2];	//0
					aux[1] = sInstr[3];	//2
					aux[2] = '\0';
					uint8_t sector = (uint8_t) atoi(aux);

					// get status command (set ON=1 or Set off=0)
					aux[0] = '0';
					aux[1] = sInstr[5];	//0
					aux[2] = '\0';
					uint8_t sectorSequenceChange = (uint8_t) atoi(aux);

					if(!sector)	// If sector num =0, then turn OFF all sectors
					{
						int i;
						for(i=0;i<valveAlloc;i++)
						{
							eeprom_write_byte((uint8_t *)(i+addr_valveSequence), sectorSequenceChange);
						}
					}
					else		// Else, set or clear the specified sector;
					{
						eeprom_write_byte((uint8_t *)(sector-1+addr_valveSequence), sectorSequenceChange);
					}
					refreshSectorSetup();
					summary_Print(1);
				}
				break;
// -----------------------------------------------------------------
			case 5: // Command is $5:h1:2130;
			{
				if(sInstr[1]==':' && sInstr[2]=='h' && sInstr[4]==':' && sInstr[9]==';')
				{
					aux[0] = '0';
					aux[1] = sInstr[3];
					aux[2] = '\0';
					uint8_t indexV = (uint8_t) atoi(aux);

					aux[0] = sInstr[5];
					aux[1] = sInstr[6];
					aux[2] = '\0';
					HourOnTM[indexV-1] = (uint8_t) atoi(aux);
					eeprom_write_byte(( uint8_t *)(addr_HourOnTM+indexV-1), HourOnTM[indexV-1]);

					aux[0] = sInstr[7];
					aux[1] = sInstr[8];
					aux[2] = '\0';
					MinOnTM[indexV-1] = (uint8_t) atoi(aux);
					eeprom_write_byte(( uint8_t *)(addr_MinOnTM+indexV-1), MinOnTM[indexV-1]);

					summary_Print(5);
				}
				else if(sInstr[1]==':' && sInstr[2]=='n' && sInstr[3]==':' && sInstr[5]==';')
				{
					aux[0] = '0';
					aux[1] = sInstr[4];
					aux[2] = '\0';

					nTM = (uint8_t) atoi(aux);
					eeprom_write_byte(( uint8_t *)(addr_nTM), nTM);

					summary_Print(5);
				}
				else if(sInstr[1]==';')
				{
					summary_Print(5);
				}
			}
			break;
// -----------------------------------------------------------------
			case 6: // Program set
			{
				// 6x;
				// 63:sxx;
				aux[0] = '0';
				aux[1] = sInstr[1];
				aux[2] = '\0';
				uint8_t setCommand = (uint8_t) atoi(aux);

				switch (setCommand)
				{
					case 0:
						stateMode = 0;
						Serial.println("rstCommand");
						turnAll_OFF(0x00);
						break;

					case 1:
						stateMode = 1; //Programed mode;
						eeprom_write_byte(( uint8_t *)(addr_stateMode), stateMode);
						break;

					case 9:
//						if((sInstr[2] == ':') && (sInstr[4] == ':') && (sInstr[7] == ';'))
						if((sInstr[2] == ':')  && (sInstr[5] == ';'))
						{
//							aux[0] = '0';
//							aux[1] = sInstr[3];
//							aux[2] = '\0';
//							uint8_t testCommand = (uint8_t) atoi(aux);

							aux[0] = sInstr[3];
							aux[1] = sInstr[4];
							aux[2] = '\0';
							uint8_t sector = (uint8_t) atoi(aux);

							stateMode = 9;
							sectorCurrently = sector;
//							switch (testCommand)
//							{
//								case 0:
//
//									break;
//
//								case 1:
//									break;
//
//								default:
//									stateMode = 0;
//									break;
//							}
						}
				}
				summary_Print(0);
			}
			break;
// -----------------------------------------------------------------
			case 7:	// $7:i1:050;  $7:i2:200;  $7:p1:050;  $7:p2:070; Parameters
			{         //012345678
				if(sInstr[1]==':' && sInstr[2]=='i' && sInstr[3]=='1' && sInstr[4]==':' && sInstr[8]==';')
				{
//					aux[0] = '0';
//					aux[1] = sInstr[3];
//					aux[2] = '\0';
//					uint8_t indexV = (uint8_t) atoi(aux);

					aux2[0] = '0';
					aux2[1] = sInstr[5];
					aux2[2] = sInstr[6];
					aux2[3] = sInstr[7];
					aux2[4] = '\0';

					uint16_t i1wordRef = 0;
					i1wordRef = (uint16_t) atoi(aux2);
					minIs  = i1wordRef;

					uint8_t lbyteRef = 0, hbyteRef = 0;
					lbyteRef = i1wordRef;
					hbyteRef = (i1wordRef >> 8);

					eeprom_write_byte((uint8_t *)(addr_minIs+1), hbyteRef);
					eeprom_write_byte((uint8_t *)(addr_minIs), lbyteRef);

					summary_Print(7);
				}
				else if(sInstr[1]==':' && sInstr[2]=='i' && sInstr[3]=='2' && sInstr[4]==':' && sInstr[8]==';')
				{
					aux2[0] = '0';
					aux2[1] = sInstr[5];
					aux2[2] = sInstr[6];
					aux2[3] = sInstr[7];
					aux2[4] = '\0';

					maxIs = (uint16_t) atoi(aux2);

					uint8_t lbyteRef = 0, hbyteRef = 0;
					lbyteRef = maxIs;
					hbyteRef = (maxIs >> 8);

					eeprom_write_byte((uint8_t *)(addr_maxIs+1), hbyteRef);
					eeprom_write_byte((uint8_t *)(addr_maxIs), lbyteRef);

					nodeValve_setIs(maxIs);

					summary_Print(7);
				}
				else if(sInstr[1]==':' && sInstr[2]=='p' && sInstr[3]=='1' && sInstr[4]==':' && sInstr[8]==';')
				{
					aux2[0] = '0';
					aux2[1] = sInstr[5];
					aux2[2] = sInstr[6];
					aux2[3] = sInstr[7];
					aux2[4] = '\0';

					minPRess = (uint16_t) atoi(aux2);

					uint8_t lbyteRef = 0, hbyteRef = 0;
					lbyteRef = minPRess;
					hbyteRef = (minPRess >> 8);

					eeprom_write_byte((uint8_t *)(addr_minPRess+1), hbyteRef);
					eeprom_write_byte((uint8_t *)(addr_minPRess), lbyteRef);

					summary_Print(7);
				}
				else if(sInstr[1]==':' && sInstr[2]=='p' && sInstr[3]=='2' && sInstr[4]==':' && sInstr[8]==';')
				{
					aux2[0] = '0';
					aux2[1] = sInstr[5];
					aux2[2] = sInstr[6];
					aux2[3] = sInstr[7];
					aux2[4] = '\0';

					maxPRess = (uint16_t) atoi(aux2);

					uint8_t lbyteRef = 0, hbyteRef = 0;
					lbyteRef = maxPRess;
					hbyteRef = (maxPRess >> 8);

					eeprom_write_byte((uint8_t *)(addr_maxPRess+1), hbyteRef);
					eeprom_write_byte((uint8_t *)(addr_maxPRess), lbyteRef);

					summary_Print(7);
				}
				else if(sInstr[1]==':' && sInstr[2]=='t' && sInstr[3]=='1' && sInstr[4]==':' && sInstr[8]==';') // timePipeB set
				{
					aux2[0] = '0';
					aux2[1] = sInstr[5];
					aux2[2] = sInstr[6];
					aux2[3] = sInstr[7];
					aux2[4] = '\0';

					timePipeB = (uint16_t) atoi(aux2);

					uint8_t lbyteRef = 0, hbyteRef = 0;
					lbyteRef = timePipeB;
					hbyteRef = (timePipeB >> 8);

					eeprom_write_byte((uint8_t *)(addr_timePipeB+1), hbyteRef);
					eeprom_write_byte((uint8_t *)(addr_timePipeB), lbyteRef);

					summary_Print(7);
				}
				else if(sInstr[1]==':' && sInstr[2]=='m' && sInstr[3]=='1' && sInstr[4]==':' && sInstr[7]==';') // motor start timer 1
				{	//$7:m1:xx;
//					aux2[0] = sInstr[5];
//					aux2[1] = sInstr[6];
//					aux2[2] = sInstr[7];
//					aux2[3] = sInstr[8];
//					aux2[4] = '\0';

					aux[0] = sInstr[5];
					aux[1] = sInstr[6];
					aux[2] = '\0';

					motorTimerStart1 = (uint8_t) atoi(aux);

//					uint8_t lbyteRef = 0, hbyteRef = 0;
//					lbyteRef = motorTimerStart1;
//					hbyteRef = (motorTimerStart1 >> 8);

//					eeprom_write_byte((uint8_t *)(addr_motorTimerStart1+1), hbyteRef);
//					eeprom_write_byte((uint8_t *)(addr_motorTimerStart1), lbyteRef);
					eeprom_write_byte((uint8_t *)(addr_motorTimerStart1), motorTimerStart1);

					Serial.print("motorTimer: ");
					Serial.println(motorTimerStart1);

					summary_Print(7);
				}
				else if(sInstr[1]==':' && sInstr[2]=='m' && sInstr[3]=='2' && sInstr[4]==':' && sInstr[8]==';') // motor start timer 2
				{
					aux2[0] = '0';
					aux2[1] = sInstr[5];
					aux2[2] = sInstr[6];
					aux2[3] = sInstr[7];
					aux2[4] = '\0';

					motorTimerStart2 = (uint16_t) atoi(aux2);

					uint8_t lbyteRef = 0, hbyteRef = 0;
					lbyteRef = motorTimerStart2;
					hbyteRef = (motorTimerStart2 >> 8);

					eeprom_write_byte((uint8_t *)(addr_motorTimerStart2+1), hbyteRef);
					eeprom_write_byte((uint8_t *)(addr_motorTimerStart2), lbyteRef);

					summary_Print(7);
				}

//				7:27988081875;
//
////				if(sInstr[1] == ':')
////				{
//				int i;
//				uint8_t flag_numError;
//				flag_numError = 0;
//
//				for(i=0;i<11;i++)
//				{
//					aux[0] = '0';
//					aux[1] = sInstr[i+1];
//					aux[2] = '\0';
//					celPhoneNumber[i] = (uint8_t) atoi(aux);
//					if(celPhoneNumber[i] >9)
//					{
//						flag_numError = 1;
//					}
//				}
//
//				if(!flag_numError)
//				{
//					for(i=0;i<11;i++)
//					{
//						eeprom_write_byte(( uint8_t *)(i+addr_celNumber), celPhoneNumber[i]);
//					}
//
////					refreshCelPhoneNumber();
////					summary_Print(2);
//				}
//				else
//				{
////					summary_Print(9);
//				}
			}
			break;
// -----------------------------------------------------------------
			case 8:
//				GLCD.Init();
			break;
// -----------------------------------------------------------------
			case 9: // internet stuffs
				// 9x;
				uint8_t setCommandConnection;
				aux[0] = '0';
				aux[1] = sInstr[1];
				aux[2] = '\0';
				setCommandConnection = (uint8_t) atoi(aux);

				switch(setCommandConnection)
				{
					case 0:
//						Serial.println("Starting GPRS Conn...!");
//						SIM900_GPRS_Connect();
						break;

					case 1:
//						SIM900_getIpAddress();
						break;

					case 2:
//						Serial.println("Starting TCP Server...");
//						SIM900_TCP_Server_Start();
						break;

					case 3:
//						Serial.println("Stoping GPRS Conn...!");
//						SIM900_GPRS_Diconnect();

						break;

					case 4:
//						SIM900_sendmail();
						break;

					default:
//						Serial.println("N4");
						break;
				}
				break;
// -----------------------------------------------------------------
			default:
				summary_Print(10);
				break;
		}
		memset(sInstr,0,sizeof(sInstr));
	}
}

ISR(TIMER1_COMPA_vect)
{
	if(!count_timePipeB)
		flag_BrokenPipeVerify = 1;
	else
		count_timePipeB--;

	if(stateMode)
	{
		if(timeSector == 0)
			flag_timeOVF = 1;
		else
			timeSector--;
	}

	flag_1s = 1;
}

// Test functions
void get_adc()
{

//	ADCSRB &= ~(1<<MUX5);				// To select ADC0;

//	ADMUX &= ~(1<<MUX4);				// Select ADC0
	ADMUX &= ~(1<<MUX3);
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX1);
	ADMUX |=  (1<<MUX0);

	ADCSRA |= (1<<ADSC);				// Start conversion;
	while (bit_is_set(ADCSRA, ADSC));	// wait until conversion done;

	uint8_t low, high;
	low  = ADCL;
	high = ADCH;

//	int value = (high << 8) | low;
//	Serial.println(value);
	_delay_ms(200);


//	return ((high << 8) | low);
}

int main()
{
	cli();								// Clear all interruptions
	flag_WDRF 	= ((1<<WDRF)  & MCUSR);	// PowerOFF / Reset verification
	flag_BORF 	= ((1<<BORF)  & MCUSR);
	flag_EXTRF 	= ((1<<EXTRF) & MCUSR);
	flag_PORF 	= ((1<<PORF)  & MCUSR);
	MCUSR = 0x00;
	sei();								// System enable interruptions

	init();								// Initialize arduino hardware requirements.
	init_contactors();
	init_ADC();
//	init_SIM900();
	init_Timer1_1Hz();
	init_WDT();
	Wire.begin();

	Serial.begin(38400);				// Debug
	Serial.println("_V_");				// Welcome!

//	SerialSIM900.begin(9600);

	turnAll_OFF_init();
	refreshStoredData();

	while (1)
	{
		// Refresh all variables to compare and take decisions;
		wdt_reset();
		refreshVariables();

		// Bluetooth communication
		wdt_reset();
		comm_Bluetooth();

		wdt_reset();
//		comm_SIM900();

		// Message Manipulation
		wdt_reset();
		handleMessage();

		// Main process.
		wdt_reset();
		process_Mode();

//		if(freeMemory()<50)
//		{
//			Serial.println("Down!");
//		}
//		char buffer[15];
//		sprintf(buffer,"RAM: %d",freeMemory());
//		Serial.println(buffer);
////		SIM900 <--> uC
//		wdt_reset();
//		comm_SIM900();
	}

//	// Use only this while to setup bluetooth
//	while(1)
//	{
//		wdt_reset();
//		comm_SerialPC_BluetoothMOD();
//	}
}

