/*
||
|| @file   Teensy_NeoPixel.ino

|| @description
||
*/

#include "FiniteStateMachine.h"
#include <TimeLib.h>

#include "FBD.h"
#include <FastLED.h>

#define NUM_STRIPS 22
#define PlayerSerial Serial1

#define NUM_LEDS_PER_STRIP 16
#define NUM_LEDS (NUM_LEDS_PER_STRIP * NUM_STRIPS + 8)
CRGB leds[NUM_LEDS];


/*
* This is part for color setting.
* H, S, V range.
*
* 42, 45, 255 is (60, 17.85, 100) in colorizer.org.
* So Active color is (42, 45, 100)
* Standbycolor is (42, 45, 0) = (60, 17.85, 0) in colorizer.org
*/
#define HUE 42
#define SAT 45
#define STBYBRIGHTNESS 0
#define ACTIVEBRIGHTNESS 255

// Standby to Active transition time
#define STBYTOACTIVETIME 2000

// Active to Standby transition time
#define ACTIVETOSTBYTIME 2000

// Active to Trigger transition time
#define ACTIVETOTRIGTIME 1000

// Trigger to Active transition time
#define TRIGTOACTIVETIME 1000

// Trigger holding time,
#define TRIGGERHOLDINGTIME 15000

// Trigger Ignore time,
#define TRIGGERIGNORETIME 30000

/* Ports definition */
const uint8_t POWERBUTTON = 24;

// Light sensor modules
#define NUM_TRIGGERS 22
const uint8_t Toggles[] = { 25, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23 };
/*
const uint8_t Toggle01 = 25;
const uint8_t Toggle02 = 29;
const uint8_t Toggle03 = 30;
const uint8_t Toggle04 = 31;
const uint8_t Toggle05 = 32;
const uint8_t Toggle06 = 33;
const uint8_t Toggle07 = 34;
const uint8_t Toggle08 = 35;
const uint8_t Toggle09 = 36;
const uint8_t Toggle10 = 37;
const uint8_t Toggle11 = 38;
const uint8_t Toggle12 = 39;

const uint8_t Toggle13 = 21;
const uint8_t Toggle14 = 22;
const uint8_t Toggle15 = 23;
const uint8_t Toggle16 = 24;
const uint8_t Toggle17 = 25;
const uint8_t Toggle18 = 26;
const uint8_t Toggle19 = 27;
const uint8_t Toggle20 = 28;
const uint8_t Toggle21 = 29;
const uint8_t Toggle22 = 23;

*/

// Neopixel LED Strips
const uint8_t Strip1 = 2;
const uint8_t Strip2 = 3;
const uint8_t Strip3 = 4;
const uint8_t Strip4 = 6;
const uint8_t Strip5 = 7;
const uint8_t Strip6 = 8;
const uint8_t Strip7 = 9;
const uint8_t Strip8 = 10;

// Define Order of Strips
const uint16_t STRIP_A1 = 33;
const uint16_t STRIP_A2 = 17;
const uint16_t STRIP_A3 = 82;
const uint16_t STRIP_A4 = 66;
const uint16_t STRIP_A5 = 131;
const uint16_t STRIP_A6 = 1;
const uint16_t STRIP_A7 = 50;
const uint16_t STRIP_A8 = 115;
const uint16_t STRIP_A9 = 99;
const uint16_t STRIP_A10 = 164;
const uint16_t STRIP_A11 = 148;

const uint16_t STRIP_B1 = 181;
const uint16_t STRIP_B2 = 197;
const uint16_t STRIP_B3 = 214;
const uint16_t STRIP_B4 = 312;
const uint16_t STRIP_B5 = 263;
const uint16_t STRIP_B6 = 230;
const uint16_t STRIP_B7 = 328;
const uint16_t STRIP_B8 = 344;
const uint16_t STRIP_B9 = 279;
const uint16_t STRIP_B10 = 295;
const uint16_t STRIP_B11 = 246;

const uint16_t DriverLED[8] = { 0, 49, 98, 147, 180, 213, 262, 311 };
const uint16_t STRIP_NONE = 0x00;

const uint16_t STRIP_PTR[23] = {
	STRIP_A1 , STRIP_A2, STRIP_A3, STRIP_A4, STRIP_A5, STRIP_A6, STRIP_A7, STRIP_A8, STRIP_A9, STRIP_A10, STRIP_A11,
	STRIP_B1 , STRIP_B2, STRIP_B3, STRIP_B4, STRIP_B5, STRIP_B6, STRIP_B7, STRIP_B8, STRIP_B9, STRIP_B10, STRIP_B11,
	STRIP_NONE
};


const uint16_t nLEDInterval = 25;

const uint16_t PreDetermine[NUM_TRIGGERS][3] = {
	{ STRIP_A6, STRIP_A9, STRIP_A5 },{ STRIP_NONE, STRIP_NONE, STRIP_NONE },{ STRIP_A5, STRIP_A6, STRIP_A9 },{ STRIP_NONE, STRIP_NONE, STRIP_NONE },
	{ STRIP_A1, STRIP_A3, STRIP_A6 },{ STRIP_A1, STRIP_A5, STRIP_A10 },{ STRIP_A2, STRIP_A4, STRIP_A8 },{ STRIP_NONE, STRIP_NONE, STRIP_NONE },
	{ STRIP_A6, STRIP_A10, STRIP_A1 },{ STRIP_NONE, STRIP_NONE, STRIP_NONE },{ STRIP_NONE, STRIP_NONE, STRIP_NONE },
	{ STRIP_B4, STRIP_B6, STRIP_B9 },{ STRIP_NONE, STRIP_NONE, STRIP_NONE },{ STRIP_NONE, STRIP_NONE, STRIP_NONE },{ STRIP_B9, STRIP_B6, STRIP_B1 },
	{ STRIP_NONE, STRIP_NONE, STRIP_NONE },{ STRIP_B4, STRIP_B9, STRIP_B10 },{ STRIP_NONE, STRIP_NONE, STRIP_NONE },{ STRIP_NONE, STRIP_NONE, STRIP_NONE },
	{ STRIP_B4, STRIP_B6, STRIP_B10 },{ STRIP_B9, STRIP_B1, STRIP_B6 },{ STRIP_NONE, STRIP_NONE, STRIP_NONE }
};

static TON TriggerTON[NUM_TRIGGERS];
static TP TriggerTP[NUM_TRIGGERS];
static Rtrg TriggerRtrg[NUM_TRIGGERS];

static TON PowerTON;
static Rtrg PowerTrg;

static uint64_t nTimeMs = 0;
static uint8_t nTrgNumber = 0;

void Standby2ActiveUpdate();
void Active2StandbyUpdate();

void ActiveUpdate();
void TrigerEnter();
void Active2TriggerUpdate();
void TrigerUpdate();
void Trigger2ActiveUpdate();
//void stand
void TrigerUpdate();
void StandbyUpdate();
void ActiveEnter();

void TrigerEnd();

/** the state machine controls which of the states get attention and execution time */
/** this is the definitions of the states that our program uses */
State Standby2Active = State(Standby2ActiveUpdate);
State Active2Standby = State(Active2StandbyUpdate);
State Active = State(ActiveEnter, ActiveUpdate, NULL);
State Active2Trigger = State(Active2TriggerUpdate);
State Triggered = State(TrigerEnter, TrigerUpdate, TrigerEnd);
State Trigger2Active = State(Trigger2ActiveUpdate);
State Standby = State(StandbyUpdate);  //no operation

FSM stateMachine = FSM(Standby); //initialize state machine, start in state: noop

const uint16_t DEBOUNCE = 200; // 200ms

void InitVars()
{

	fill_solid(leds, NUM_LEDS, CRGB::White);

	for (uint8_t idx = 0; idx < NUM_TRIGGERS; idx++)
	{
		TriggerTON[idx].ET = 0;
		TriggerTON[idx].IN = 0;
		TriggerTON[idx].PRE = 0;
		TriggerTON[idx].Q = 0;
		TriggerTON[idx].PT = DEBOUNCE;

		TriggerTP[idx].PT = TRIGGERIGNORETIME;
		TriggerTP[idx].IN = 0;
		TriggerTP[idx].PRE = 0;
		TriggerTP[idx].Q = 0;
		TriggerTP[idx].ET = 0;

		TriggerRtrg[idx].Q = 0;
		TriggerRtrg[idx].IN = 0;
		TriggerRtrg[idx].PRE = 0;
	}

	PowerTrg.IN = 0;
	PowerTrg.PRE = 0;
	PowerTrg.Q = 0;

	PowerTON.ET = 0;
	PowerTON.PT = DEBOUNCE;
	PowerTON.IN = 0;
	PowerTON.PRE = 0;
	PowerTON.Q = 0;

}

void setup()
{
	// Console init
	Serial.begin(115200);
	while (!Serial);
	Serial.println("Program started!!!");
	PlayerSerial.begin(115200);
	//SendVideoCommand(0);

	// 19mm Power Button Init
	digitalWrite(POWERBUTTON, HIGH);
	pinMode(POWERBUTTON, INPUT_PULLUP);

	// Light Sensors
	for (uint8_t idx = 0; idx < NUM_TRIGGERS; idx++)
	{
		digitalWrite(Toggles[idx], HIGH);
		pinMode(Toggles[idx], INPUT_PULLUP);
	}

	// LED Strip Pins for LED Drive
	pinMode(Strip1, OUTPUT);
	pinMode(Strip2, OUTPUT);
	pinMode(Strip3, OUTPUT);
	pinMode(Strip4, OUTPUT);
	pinMode(Strip5, OUTPUT);
	pinMode(Strip6, OUTPUT);
	pinMode(Strip7, OUTPUT);
	pinMode(Strip8, OUTPUT);

	// Configure LED Object
	FastLED.addLeds<NEOPIXEL, Strip1>(leds + (NUM_LEDS_PER_STRIP * 0 + 0), NUM_LEDS_PER_STRIP * 3 + 1);

	FastLED.addLeds<NEOPIXEL, Strip2>(leds + (NUM_LEDS_PER_STRIP * 3 + 1), NUM_LEDS_PER_STRIP * 3 + 1);

	FastLED.addLeds<NEOPIXEL, Strip3>(leds + (NUM_LEDS_PER_STRIP * 6 + 2), NUM_LEDS_PER_STRIP * 3 + 1);

	FastLED.addLeds<NEOPIXEL, Strip4>(leds + (NUM_LEDS_PER_STRIP * 9 + 3), NUM_LEDS_PER_STRIP * 2 + 1);

	FastLED.addLeds<NEOPIXEL, Strip5>(leds + (NUM_LEDS_PER_STRIP * 11 + 4), NUM_LEDS_PER_STRIP * 2 + 1);

	FastLED.addLeds<NEOPIXEL, Strip6>(leds + (NUM_LEDS_PER_STRIP * 13 + 5), NUM_LEDS_PER_STRIP * 3 + 1);

	FastLED.addLeds<NEOPIXEL, Strip7>(leds + (NUM_LEDS_PER_STRIP * 16 + 6), NUM_LEDS_PER_STRIP * 3 + 1);

	FastLED.addLeds<NEOPIXEL, Strip8>(leds + (NUM_LEDS_PER_STRIP * 19 + 7), NUM_LEDS_PER_STRIP * 3 + 1);

	//
	InitVars();
}


//poor example, but then again; it's just an example
void loop()
{

	// Toggle Processing
	for (uint8_t idx = 0; idx < NUM_TRIGGERS; idx++)
	{
		TriggerTON[idx].IN = digitalRead(Toggles[idx]) == LOW;// && stateMachine.isInState(Active);
		TONFunc(&TriggerTON[idx]);

		//TriggerTP[idx].IN = TriggerTON[idx].Q;
		//TPFunc(&TriggerTP[idx]);
		TriggerRtrg[idx].IN = TriggerTON[idx].Q;
		RTrgFunc(&TriggerRtrg[idx]);
		if (TriggerRtrg[idx].Q && stateMachine.isInState(Active))
		{
			Serial.print("Light sensor ");
			Serial.print(idx + 1);
			Serial.println(" is Triggered!");

			switch (idx)
			{
			case 0:
			case 2:
			case 4:
			case 5:
			case 6:
			case 8:
			case 11:
			case 14:
			case 16:
			case 19:
			case 20:
			{
				nTrgNumber = idx;
				stateMachine.transitionTo(Active2Trigger);
			}
			break;
			}
		}
	}

	// Power Button Process
	PowerTON.IN = digitalRead(POWERBUTTON) == LOW && stateMachine.isInState(Standby);
	TONFunc(&PowerTON);
	PowerTrg.IN = PowerTON.Q;
	RTrgFunc(&PowerTrg);
	if (PowerTrg.Q)
	{
		Serial.println("Power Button Pressed, I have entered into Active Status.");
		stateMachine.transitionTo(Standby2Active);
	}

	// State transition
	uint32_t ActiveTimeOut = 60000; // 1 minutes
	if (stateMachine.isInState(Active) && stateMachine.timeInCurrentState() >= ActiveTimeOut)
	{

		stateMachine.transitionTo(Active2Standby);
		Serial.println("I have switched into Active2Standby Status");
	}

	//
	if (stateMachine.isInState(Standby2Active) && stateMachine.timeInCurrentState() >= STBYTOACTIVETIME)
	{
		stateMachine.transitionTo(Active);
		Serial.println("I have switched into Active Status");
	}

	if (stateMachine.isInState(Active2Standby) && stateMachine.timeInCurrentState() >= ACTIVETOSTBYTIME)
	{

		Serial.println("I have switched into Standby Status");
		stateMachine.transitionTo(Standby);

	}

	if (stateMachine.isInState(Active2Trigger) && stateMachine.timeInCurrentState() >= ACTIVETOTRIGTIME)
	{
		stateMachine.transitionTo(Triggered);
		Serial.println("I have switched into Trigger Status");
	}

	if (stateMachine.isInState(Triggered) && stateMachine.timeInCurrentState() >= TRIGGERHOLDINGTIME)
	{
		if (digitalRead(Toggles[nTrgNumber]) == HIGH)
		{
			Serial.println("I have switched into Trigger2Active Status");
			stateMachine.transitionTo(Trigger2Active);
		}
		else if (stateMachine.timeInCurrentState() > TRIGGERHOLDINGTIME * 3)
		{
			Serial.println("Three cycles ignored, I have switched into Trigger2Active Status");
			stateMachine.transitionTo(Trigger2Active);
		}
	}

	if (stateMachine.isInState(Trigger2Active) && stateMachine.timeInCurrentState() >= TRIGTOACTIVETIME)
	{
		Serial.println("I have switched into Active Status");

		stateMachine.transitionTo(Active);
	}

	// FSM management
	stateMachine.update();
}

void StandbyUpdate()
{
	if ((nTimeMs + nLEDInterval) < millis())
	{
		nTimeMs = millis();
		fill_solid(leds, NUM_LEDS, CHSV(HUE, SAT, STBYBRIGHTNESS));
		for (uint8_t idx = 0; idx < 8; idx++)
			leds[DriverLED[idx]] = CRGB(0x00, 0x00, 0x00);
		FastLED.show();
	}
}

void ActiveEnter()
{
	uint8_t nBrightness = 0;
	fill_solid(leds, NUM_LEDS, CHSV(HUE, SAT, ACTIVEBRIGHTNESS));
	for (uint8_t idx = 0; idx < 8; idx++)
		leds[DriverLED[idx]] = CRGB(0x00, 0x00, 0x00);
	FastLED.show();
}

void ActiveUpdate()
{
	if ((nTimeMs + nLEDInterval) < millis())
	{
		nTimeMs = millis();
		fill_solid(leds, NUM_LEDS, CRGB(255, 255, 209));
		//fill_solid(leds, NUM_LEDS, CHSV(HUE, SAT, ACTIVEBRIGHTNESS));
		for (uint8_t idx = 0; idx < 8; idx++)
			leds[DriverLED[idx]] = CRGB(0x00, 0x00, 0x00);
		FastLED.show();
	}
}

void TrigerEnter()
{
	// This part is when trigger happens, it will play video as fit to triger number
	SendVideoCommand(nTrgNumber + 1);
	for (uint8_t idx = 0; idx < NUM_STRIPS; idx++)
	{
		uint16_t ntemp = STRIP_PTR[idx];
		if ((PreDetermine[nTrgNumber][0] == ntemp || PreDetermine[nTrgNumber][1] == ntemp) || PreDetermine[nTrgNumber][2] == ntemp)
		{
			fill_solid(leds + ntemp, NUM_LEDS_PER_STRIP, CHSV(HUE, SAT, ACTIVEBRIGHTNESS));
		}
		else
		{
			fill_solid(leds + ntemp, NUM_LEDS_PER_STRIP, CHSV(HUE, SAT, STBYBRIGHTNESS));
		}
	}
	for (uint8_t idx = 0; idx < 8; idx++)
		leds[DriverLED[idx]] = CRGB(0x00, 0x00, 0x00);

	FastLED.show();

}

void TrigerUpdate()
{

}

void SendVideoCommand(uint8_t nVideoIndex)
{
	uint8_t nPacket[6];
	// Start Character
	nPacket[0] = 0xFF;
	// Packet start byte
	nPacket[1] = 0xAA;
	// Data length
	nPacket[2] = 0x01;
	// Video index
	nPacket[3] = nVideoIndex + 1;

	// checksum
	uint8_t nSum = 0;
	nSum += nPacket[2];
	nSum += nPacket[3];
	nPacket[4] = nSum % 100;
	// End Character
	nPacket[5] = 0xFE;

	for (uint8_t idx = 0; idx < 6; idx++)
	{
		PlayerSerial.write(nPacket[idx]);
	}

	Serial.println("I sent video command!, Command is ");
	Serial.print("0x");
	for (uint8_t idx = 0; idx < 6; idx++)
	{
		if (nPacket[idx] < 16)
			Serial.print("0");
		Serial.print(nPacket[idx], HEX);

	}
	Serial.println();
}

void Standby2ActiveUpdate()
{
	uint8_t nBrightness = 0;

	nBrightness = map(stateMachine.timeInCurrentState(), 0, STBYTOACTIVETIME, STBYBRIGHTNESS, ACTIVEBRIGHTNESS);
	if ((nTimeMs + nLEDInterval) < millis())
	{
		nTimeMs = millis();
		for (uint8_t idx = 0; idx < NUM_STRIPS; idx++)
		{
			uint16_t ntemp = STRIP_PTR[idx];
			fill_solid(leds + ntemp, NUM_LEDS_PER_STRIP, CHSV(HUE, SAT, nBrightness));
		}

		for (uint8_t idx = 0; idx < 8; idx++)
			leds[DriverLED[idx]] = CRGB(0x00, 0x00, 0x00);
		FastLED.show();
	}
}

void Active2StandbyUpdate()
{
	uint8_t nBrightness = 0;
	nBrightness = map(stateMachine.timeInCurrentState(), 0, ACTIVETOSTBYTIME, ACTIVEBRIGHTNESS, STBYBRIGHTNESS);
	if ((nTimeMs + nLEDInterval) < millis())
	{
		nTimeMs = millis();
		for (uint8_t idx = 0; idx < NUM_STRIPS; idx++)
		{
			uint16_t ntemp = STRIP_PTR[idx];
			fill_solid(leds + ntemp, NUM_LEDS_PER_STRIP, CHSV(HUE, SAT, nBrightness));
		}
		for (uint8_t idx = 0; idx < 8; idx++)
			leds[DriverLED[idx]] = CRGB(0x00, 0x00, 0x00);
		FastLED.show();
	}
}

void Active2TriggerUpdate()
{
	uint8_t nBrightFadeDown;
	nBrightFadeDown = map(stateMachine.timeInCurrentState(), 0, ACTIVETOTRIGTIME, ACTIVEBRIGHTNESS, STBYBRIGHTNESS);
	if ((nTimeMs + nLEDInterval) < millis())
	{
		nTimeMs = millis();
		Serial.println("Active to Trigger");
		for (uint8_t idx = 0; idx < NUM_STRIPS; idx++)
		{
			uint16_t ntemp = STRIP_PTR[idx];
			if ((PreDetermine[nTrgNumber][0] == ntemp || PreDetermine[nTrgNumber][1] == ntemp) || PreDetermine[nTrgNumber][2] == ntemp)
			{
				fill_solid(leds + ntemp, NUM_LEDS_PER_STRIP, CHSV(HUE, SAT, ACTIVEBRIGHTNESS));
			}
			else
			{
				fill_solid(leds + ntemp, NUM_LEDS_PER_STRIP, CHSV(HUE, SAT, nBrightFadeDown));
			}
		}
		for (uint8_t idx = 0; idx < 8; idx++)
			leds[DriverLED[idx]] = CRGB(0x00, 0x00, 0x00);

		FastLED.show();
	}
}

void Trigger2ActiveUpdate()
{
	uint8_t nBrightFadeUp;
	nBrightFadeUp = map(stateMachine.timeInCurrentState(), 0, TRIGTOACTIVETIME, STBYBRIGHTNESS, ACTIVEBRIGHTNESS);
	if ((nTimeMs + nLEDInterval) < millis())
	{
		nTimeMs = millis();
		Serial.println("Trigger to activate");
		for (uint8_t idx = 0; idx < NUM_STRIPS; idx++)
		{
			uint16_t ntemp = STRIP_PTR[idx];
			if ((PreDetermine[nTrgNumber][0] == ntemp || PreDetermine[nTrgNumber][1] == ntemp) || PreDetermine[nTrgNumber][2] == ntemp)
			{
				fill_solid(leds + ntemp, NUM_LEDS_PER_STRIP, CHSV(HUE, SAT, ACTIVEBRIGHTNESS));
			}
			else
			{
				fill_solid(leds + ntemp, NUM_LEDS_PER_STRIP, CHSV(HUE, SAT, nBrightFadeUp));
			}
		}
		for (uint8_t idx = 0; idx < 8; idx++)
			leds[DriverLED[idx]] = CRGB(0x00, 0x00, 0x00);
		FastLED.show();
	}
}

void TrigerEnd()
{
	SendVideoCommand(0);
}

void pauseVideo()
{
	SendVideoCommand(23);
}

void stopVideo()
{
	SendVideoCommand(25);
}
