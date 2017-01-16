/*
This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
http://creativecommons.org/licenses/by-nc-sa/4.0/legalcode
Initial credits go to Nick Gammon for serial state machine

Version 1.4

Philosophy:
1- decode serial communication or I2C messages sent by another device (ESP8266 with nodemcu for example :p ) using state machine 
2- Manage update of global value including controls and math if needed
3- Update display using handlePreviousState and config in EEPROM
4- Manage buttons to alter config
*/

/*
Serial/I2C syntax: [command letter][decimal number]
// Yx      : Start module with intensity set to x (1-7)
// Sx      : Speed update (decimal notation)
// Gx      : Gear update (decimal notation)
// Rx      : RPM update (decimal notation)
// Tx      : Max RPM
// Ox      : oil temp
// Wx      : water temp
// Fx      : fuel
// Lx      : lap
// Px      : position
// Exxxx   : lap time. 4 bytes representing  float according to IEEE
// Z0A     : Stop and clear module

The command will be processed when a new one starts. For example, RPM will be updated on the module once a new command such as S will start
Serial data to start with max luminosity set (5), speed=95, RPM=4250, gear=2, lap=3, position=12 (last caracter is to finish processing of RPM, can be anything except a digit)
Y5S95G2R4250T5000L3P12R

If gear is negative, the reverse is assumed and 'r' is displayed
If gear is 0, neutral is assumed and 'n' is displayed

Usage
button 1 : cycle values for left dashboard part
button 2 : cycle values for right dashboard part
button 3 : toggle internal speed multiplier by 3.6 to convert m/s to km/h
button 4 : toggle between RPM leds showing full range (mode 0) or 75%-100% (mode 1)
button 5 : number of red LEDS to display (0-8)
button 6 : Reset maxrpm. Usefull when rpm goes to hig values with no reason and rpm LEDs are then less usefull.
button 8 : show current settings and save to EEPROM. Second digit shown indicates if speed multiplier is on (1) or off (0), third digit indicates RPM led mode


							+----[PWR]-------------------| USB |--+
							|                            +-----+  |
							|         GND/RST2  [ ][ ]            |
							|       MOSI2/SCK2  [ ][ ]  A5/SCL[ ] |   C5
							|          5V/MISO2 [ ][ ]  A4/SDA[ ] |   C4
							|                             AREF[ ] |
							|                              GND[ ] |
							| [ ]N/C                    SCK/13[ ] |   B5
							| [ ]IOREF                 MISO/12[ ] |   .
							| [ ]RST                   MOSI/11[ ]~|   .
							| [ ]3V3    +---+               10[ ]~|   .
							| [ ]5v    -| A |-               9[ ]~|   .
							| [ ]GND   -| R |-               8[ ] |   B0
							| [ ]GND   -| D |-                    |
							| [ ]Vin   -| U |-               7[ ] |   D7
							|          -| I |-               6[ ]~|   .
							| [ ]A0    -| N |-               5[ ]~|   .
							| [ ]A1    -| O |-               4[ ] |   .
							| [ ]A2     +---+           INT1/3[ ]~|   .
							| [ ]A3                     INT0/2[ ] |   .
							| [ ]A4/SDA  RST SCK MISO     TX>1[ ] |   .
							| [ ]A5/SCL  [ ] [ ] [ ]      RX<0[ ] |   D0
							|            [ ] [ ] [ ]              |
							|  UNO_R3    GND MOSI 5V  ____________/
							\_______________________/

http://busyducks.com/ascii-art-arduinos

								  +-----+
					 +------------| USB |------------+
					 |            +-----+            |
				B5   | [ ]D13/SCK        MISO/D12[ ] |   B4
					 | [ ]3.3V           MOSI/D11[ ] |   B3
					 | [ ]V.ref     ___    SS/D10[ ] |   B2
				C0   | [ ]A0       / N \       D9[ ] |   B1
				C1   | [ ]A1      /  A  \      D8[ ] |   B0
				C2   | [ ]A2      \  N  /      D7[ ] |   D7
				C3   | [ ]A3       \_0_/       D6[ ] |   D6
			ESP-C4   | [X]A4/SDA               D5[X] |   D5 - TM1638 DA
			ESP-C5   | [X]A5/SCL               D4[X] |   D4 - TM1638 CLOCK
					 | [ ]A6              INT1/D3[X] |   D3 - TM1638 STROBE
					 | [ ]A7              INT0/D2[ ] |   D2
					 | [ ]5V                  GND[ ] |     
				C6   | [ ]RST                 RST[ ] |   C6
					 | [ ]GND   5V MOSI GND   TX1[ ] |   D0
					 | [ ]Vin   [ ] [ ] [ ]   RX1[ ] |   D1
					 |          [ ] [ ] [ ]          |
					 |          MISO SCK RST         |
					 | NANO-V3                       |
					 +-------------------------------+
		 
					 http://busyducks.com/ascii-art-arduinos

*/

//#define DEBUG
#include <Boards.h>
#include <TM1638.h>
#include <EEPROMex.h>
#include <SoftwareSerial.h>

//#include <Wire.h>
#undef _EEPROMEX_DEBUG //to prevent false alarms on EEPROM write execeeded
#define LEFT 1
#define RIGHT 2
unsigned int carspeed;         // holds the speed data (0-65535 size)
unsigned int cargear;          // holds gear value data (0-65535 size)
unsigned int carrpm;           // holds the rpm data (0-65535 size)
unsigned int caroil;           // holds the oil temp data (0-65535 size)
unsigned int carwater;         // holds the water temp data (0-65535 size)
unsigned int carfuel;          // holds the fuel data (0-65535 size)
unsigned int carlap;           // holds the lap data (0-65535 size)
unsigned int carpos;           // holds the position data (0-65535 size)
long rpmleds;          // holds the 8 leds values, both MSB=green LSB=red
unsigned int rpmmax = 1000;    // autocalibrating RPM max val
bool speedmultiplier = 0;      //speed multiplier by 3.6 (usefull for some games such as kart racing pro)
// buttons TM1638 (left to right)
#define btn1 0b00000001 // 1
#define btn2 0b00000010 // 2
#define btn3 0b00000100 // 4
#define btn4 0b00001000 // 8
#define btn5 0b00010000 // 16
#define btn6 0b00100000 // 32
#define btn7 0b01000000 // 64
#define btn8 0b10000000 // 128

const byte indicator[] = { 'S','G','R','O','W','F','L' }; //possible indicators show on module
byte leftindicator = 0; //speed on left by default
byte rightindicator = 1; //gear on right by default
byte rpmstyle = 1; // RPM style 1: 0-100% range; 2:75%-100%
byte redleds = 3; // number of leds that will be red
byte buttons;
bool reverse;
String text;
int offset;
// the possible states of the state-machine
typedef enum { NONE, GOT_R, GOT_S, GOT_G, GOT_O, GOT_W, GOT_F, GOT_L, GOT_P, GOT_STOP, GOT_START, GOT_MAXRPM } states;

// current state-machine state
states state = NONE;
// current partial number
signed int currentValue;
//Data Clock Strobe
TM1638 module1(5, 4, 3);

void processMaxRPM(const unsigned int value)
{
#if defined DEBUG
	Serial.print(F("MaxRPM = ")); Serial.println(value);
#endif
	rpmmax = value;
}

void processRPM(const unsigned int value)
{
	// do something with RPM
#if defined DEBUG
	Serial.print(F("RPM = ")); Serial.println(value);
#endif
	carrpm = value;
	if (carrpm > rpmmax && carrpm < 50000) {
		rpmmax = carrpm;
	}
}

void processSpeed(const unsigned int value)
{
#if defined DEBUG
	Serial.print(F("Speed = ")); Serial.println(value);
#endif
	if (speedmultiplier) carspeed = value * 3.6;
	else carspeed = value;
}

void processGear(const unsigned int value)
{
	// do something with gear
#if defined DEBUG
	Serial.print(F("Gear = ")); Serial.println(value);
#endif
	cargear = value;
}

void processOil(const unsigned int value)
{
#if defined DEBUG
	Serial.print(F("Oil = ")); Serial.println(value);
#endif
	caroil = value;
}

void processWater(const unsigned int value)
{
#if defined DEBUG
	Serial.print(F("Water = ")); Serial.println(value);
#endif
	carwater = value;
}

void processFuel(const unsigned int value)
{
#if defined DEBUG
	Serial.print(F("Fuel = ")); Serial.println(value);
#endif
	carfuel = value;
}

void processLap(const unsigned int value)
{
#if defined DEBUG
	Serial.print(F("Lap = ")); Serial.println(value);
#endif
	carlap = value;
}

void processPos(const unsigned int value)
{
#if defined DEBUG
	Serial.print(F("Pos = ")); Serial.println(value);
#endif
	carpos = value;
}

void processStop(const unsigned int value)
{
#if defined DEBUG
	Serial.print(F("Stop = ")); Serial.println(value);
#endif
	module1.clearDisplay();
	module1.setLEDs(0);
	module1.setupDisplay(false, 7);
}

void processStart(const unsigned int value)
{
#if defined DEBUG
	Serial.print(F("Start = ")); Serial.println(value);
#endif
	module1.setupDisplay(true, value);
	module1.clearDisplay();
	module1.setLEDs(0);
	rpmmax = 1000;
}

void printModule(unsigned int value, int pad, int side) {
	text = String(value);
	while (text.length() < pad) {
		text = "0" + text;
	}
	if (side == LEFT) {// we print on the left side of the module
#if defined DEBUG
		Serial.print(F("Change left to: ")); Serial.print(text); Serial.println("");
#endif
		offset = 0;
	}
	else {// we print on the right side of the module
#if defined DEBUG
		Serial.print(F("Change right to: ")); Serial.print(text); Serial.println("");
#endif
		offset = 8 - text.length();
	}
	module1.setDisplayToString(text, 0, offset);
}

void handlePreviousState()
{
	unsigned int i;
	word leds=0;
	switch (state)
	{
	case GOT_MAXRPM:
		processMaxRPM(currentValue);
		break;
	case GOT_R:
		processRPM(currentValue);
		if (indicator[leftindicator] == 'R') {
			printModule(carrpm, 4, LEFT);
		}
		if (indicator[rightindicator] == 'R') {
			printModule(carrpm, 4, RIGHT);
		}
		//Update LEDS
		switch (rpmstyle) {
		case 0:
			rpmleds = map(carrpm, 0, rpmmax * 1.1, 0, 10);        // distributes the rpm level to the 8 leds + 1 for shift change
			break;
		case 1:
			rpmleds = map(carrpm, rpmmax * 0.75, rpmmax * 1.0, 0, 10); // display only 75%-100% RPM range for more accurate  gear change
			break;
		}
		//LED coloring: two bytes RED/GREEN
		// 80 40 20 10 08 04 02 01
		for (i = 0;i < rpmleds;i++) {
			if (i>=(8-redleds) || rpmleds>=9)
			{ //LED is red
				leds = leds + (1 << i+8);
			}
			else 
			{ // LED is green
				leds = leds + (1 << i);
			}
#if defined DEBUG
			//Serial.print("LED value: ");
			//Serial.println(leds);
#endif
		}
		module1.setLEDs(leds);
		break;
	case GOT_W:
		processWater(currentValue);
		if (indicator[leftindicator] == 'W') {
			printModule(carwater, 3, LEFT);
		}
		if (indicator[rightindicator] == 'W') {
			printModule(carwater, 3, RIGHT);
		}
		break;
	case GOT_F:
		processFuel(currentValue);
		if (indicator[leftindicator] == 'F') {
			printModule(carfuel, 3, LEFT);
		}
		if (indicator[rightindicator] == 'F') {
			printModule(carfuel, 3, RIGHT);
		}
		break;
	case GOT_O:
		processOil(currentValue);
		if (indicator[leftindicator] == 'O') {
			printModule(caroil, 3, LEFT);
		}
		if (indicator[rightindicator] == 'O') {
			printModule(caroil, 3, RIGHT);
		}
		break;
	case GOT_S:
		processSpeed(currentValue);
		if (indicator[leftindicator] == 'S') {
			printModule(carspeed, 3, LEFT);
		}
		if (indicator[rightindicator] == 'S') {
			printModule(carspeed, 3, RIGHT);
		}
		break;
	case GOT_G:
		processGear(currentValue);
		if (indicator[leftindicator] == 'G') {
			if (reverse == false) {
				if (cargear == 0) {
#ifdef DEBUG
					Serial.println(F("Neutral left"));
#endif
					module1.setDisplayToString("n", 0, 0);
				}
				else { printModule(cargear, 1, LEFT); }
			}
			else {
#ifdef DEBUG
				Serial.println(F("Reverse left"));
#endif
				module1.setDisplayToString("R", 0, 0);
			}
		}
		if (indicator[rightindicator] == 'G') {
			if (reverse == false) {
				if (cargear == 0) {
#ifdef DEBUG
					Serial.println(F("Neutral right"));
#endif
					module1.setDisplayToString("n", 0, 7);
				}
				else { printModule(cargear, 1, RIGHT); }
			}
			else {
#ifdef DEBUG
				Serial.println(F("Reverse right"));
#endif
				module1.setDisplayToString("R", 0, 7);
			}
		}
		reverse = false;
		break;
	case GOT_L:
		processLap(currentValue);
		if (indicator[leftindicator] == 'L') {
			printModule(carlap, 2, LEFT);
		}
		if (indicator[rightindicator] == 'L') {
			printModule(carlap, 2, RIGHT);
		}
		break;
	case GOT_P:
		processPos(currentValue);
		if (indicator[leftindicator] == 'P') {
			printModule(carpos, 2, LEFT);
		}
		if (indicator[rightindicator] == 'P') {
			printModule(carpos, 2, RIGHT);
		}
		break;
	case GOT_START:
		processStart(currentValue);
		break;
	case GOT_STOP:
		processStop(currentValue);
		break;
	}  // end of switch
	currentValue = 0;
}  // end of handlePreviousState

void processIncomingByte(const byte c)
{
	if (isdigit(c))
	{
		currentValue *= 10;
		currentValue += c - '0';
	}  // end of digit

	else {
		// The end of the number signals a state change
		if (c != '-') { handlePreviousState(); } //we need to manage minus sign for negative gear, meaning reverse speed
	  // set the new state, if we recognize it
		switch (c)
		{
		case '-':
#ifdef DEBUG
			Serial.println(F("Minus car for reverse gear"));
#endif
			reverse = true;
			state = GOT_G;
			break;
		case 'T':
			state = GOT_MAXRPM;
			break;
		case 'R':
			state = GOT_R;
			break;
		case 'S':
			state = GOT_S;
			break;
		case 'G':
			state = GOT_G;
			break;
		case 'O':
			state = GOT_O;
			break;
		case 'W':
			state = GOT_W;
			break;
		case 'F':
			state = GOT_F;
			break;
		case 'L':
			state = GOT_L;
			break;
		case 'P':
			state = GOT_P;
			break;
		case 'Z':
			state = GOT_STOP;
			break;
		case 'Y':
			state = GOT_START;
			break;
		default:
			state = NONE;
			break;
		}  // end of switch on incoming byte
	} // end of not digit

} // end of processIncomingByte

void dispIndicators()
{
	char l;
	char r;
	l = (char)(indicator[leftindicator]);
	r = (char)(indicator[rightindicator]);
	text = String(l) + " " + String(speedmultiplier) + " " + String(rpmstyle) + String(redleds)+" " + String(r);
#if defined DEBUG
	Serial.print(F("Indicators:")); Serial.println(text);
#endif
	module1.clearDisplay();
	module1.setDisplayToString(text);
	delay(200);
}

float byte2float(byte binary[]) {
	typedef union {
		float val;
		byte binary[4];
	} binaryFloat;
	binaryFloat unionvar;
	unionvar.binary[0] = binary[0];
	unionvar.binary[1] = binary[1];
	unionvar.binary[2] = binary[2];
	unionvar.binary[3] = binary[3];
#if defined DEBUG
	Serial.print("byte2float: ");
	for (char i = 0; i <4; i++) {
		Serial.print(unionvar.binary[i], HEX);Serial.print(" ");
	}
	Serial.print("= ");Serial.println(unionvar.val,4);
#endif
	return unionvar.val;
}

unsigned int byte2uint(byte binary[]) {
	word val;
	val=word(binary[1],binary[0]);
#if defined DEBUG
	Serial.print("byte2uint: ");
	for (char i = 0; i <2; i++) {
		Serial.print(binary[i], HEX);Serial.print(" ");
	}
	Serial.print("= ");Serial.println(val);
#endif
	return (unsigned int)val;
}

int byte2int(byte binary[]) {
	int val;
	val = binary[1] << 8 | binary[0];
#if defined DEBUG
	Serial.print("byte2int: ");
	for (char i = 0; i <2; i++) {
		Serial.print(binary[i], HEX);Serial.print(" ");
	}
	Serial.print("= ");Serial.println(val);
#endif
	return val;
}

void setup()
{
	Serial.begin(115200);
	state = NONE;
	module1.setupDisplay(true, 7);
	module1.clearDisplay();
	module1.setDisplayToString(F("CarDashX"));
	module1.setLEDs(0x00ff);
	delay(1000);
	leftindicator = EEPROM.readByte(5);
	rightindicator = EEPROM.readByte(7);
	speedmultiplier = EEPROM.readByte(9);
	rpmstyle = EEPROM.readByte(11);
	redleds = EEPROM.readByte(13);
	dispIndicators();
	module1.setLEDs(0x00);
	reverse = false;
/*
	Wire.begin(8);                // join i2c bus with address #8
	Wire.onReceive(receiveEvent); // register event
*/
/*
	byte vBuffer[4];
	byte sBuffer[2];
	float floatspeed;
	unsigned int uintspeed;
	int intspeed;
	vBuffer[0] = 0x41;
	vBuffer[1] = 0xA0;
	vBuffer[2] = 0x30;
	vBuffer[3] = 0x40;
	floatspeed=byte2float(vBuffer);
	vBuffer[0] = 0x40;
	vBuffer[1] = 0x30;
	vBuffer[2] = 0xA0;
	vBuffer[3] = 0x41;
	floatspeed = byte2float(vBuffer);
	sBuffer[0] = 0xFE;
	sBuffer[1] = 0x00;
	intspeed = byte2int(sBuffer);
	uintspeed = byte2uint(sBuffer);
	sBuffer[0] = 0xff;
	sBuffer[1] = 0xFf;
	intspeed = byte2int(sBuffer);
	uintspeed = byte2uint(sBuffer);
	*/
	

}  // end of setup

void loop()
{
	//  buttons = btn4; //For testing purpose
	while (1 == 1) {
		while (Serial.available()) processIncomingByte(Serial.read());
		buttons = module1.getButtons();
		if (buttons != 0 && buttons != 255) {
			//      Serial.print ("Button = "); Serial.println (buttons);
			switch (buttons)
			{
			case btn1:
				if (leftindicator + 1 < sizeof(indicator)) leftindicator++;
				else leftindicator = 0;
#if defined DEBUG
				Serial.print(F("Change left to ")); Serial.println((char)indicator[leftindicator]);
#endif
				dispIndicators();
				delay(1000);
				module1.clearDisplay();
				//buttons = btn2; //For testing purpose
				break;
			case btn2:
				if (rightindicator + 1 < sizeof(indicator)) rightindicator++;
				else rightindicator = 0;
#if defined DEBUG
				Serial.print(F("Change right to ")); Serial.println((char)indicator[rightindicator]);
#endif
				dispIndicators();
				delay(1000);
				module1.clearDisplay();
				//buttons = btn3; //For testing purpose
				break;
			case btn3:
				speedmultiplier = !speedmultiplier;
				dispIndicators();
				delay(1000);
				module1.clearDisplay();
#if defined DEBUG
				Serial.print(F("Speed multiplier set ")); Serial.println(speedmultiplier);
#endif
				//buttons = btn4;
				break;
			case btn4:
				rpmstyle = (rpmstyle + 1) % 2;
				dispIndicators();
				delay(1000);
				module1.clearDisplay();
				break;
			case btn5:
				redleds = (redleds + 1) % 9;
				dispIndicators();
				delay(1000);
				module1.clearDisplay();
				break;
			case btn6:
				rpmmax = 1000;
				dispIndicators();
				delay(1000);
				module1.clearDisplay();
				break;
			case btn8:
				dispIndicators();
				EEPROM.updateByte(5, leftindicator);
				EEPROM.updateByte(7, rightindicator);
				EEPROM.updateByte(9, speedmultiplier);
				EEPROM.updateByte(11, rpmstyle);
				EEPROM.updateByte(13, redleds);
#if defined DEBUG

				Serial.print(F("EEPROM updated ")); Serial.print(EEPROM.readByte(5)); Serial.print(EEPROM.readByte(7)); Serial.print(EEPROM.readByte(9));
#endif
				//buttons = 0;
				delay(2000);
				module1.clearDisplay();
				break;
			}
			delay(200);//debounce
		}
	}
}

/*
void receiveEvent(int howMany) {
	while (1 <= Wire.available()) { 
		char c = Wire.read(); // receive byte as a character
#if defined DEBUG
		Serial.print(c);         // print the character
#endif
		switch (c) {
			case 'S': {
				state = GOT_S;
				byte speedbyte[4];
				int i = 0;
				while (Wire.available()) {
					speedbyte[i++] = (byte)Wire.read();
	#if defined DEBUG
					Serial.print(speedbyte[i-1],HEX);         // print the character
	#endif
					  }
#if defined DEBUG
				Serial.println("");
#endif
					currentValue = (int)byte2float(speedbyte);
					//processSpeed(currentValue);
	#if defined DEBUG
					Serial.print("I2C Speed:");Serial.println(currentValue);
	#endif
					handlePreviousState();
					state = NONE;
					break;
			}
			case 'R': {
				state = GOT_R;
				byte rpmbyte[2];
				int i = 0;
				while (Wire.available()) {
					rpmbyte[i++] = (byte)Wire.read();
	#if defined DEBUG
					Serial.print(rpmbyte[i - 1],HEX);         // print the character
	#endif
				}
#if defined DEBUG
				Serial.println("");
#endif
				currentValue = byte2uint(rpmbyte);
				//processRPM((unsigned int)currentValue);
	#if defined DEBUG
				Serial.print("I2C RPM:");Serial.println(currentValue);
	#endif
				handlePreviousState();
				state = NONE;
				break;
			}
			case 'G': {
				state = GOT_G;
				handlePreviousState();
				state = NONE;
				break;
			}
		}

	}
	//int x = Wire.read();    // receive byte as an integer
	// char c = Wire.read();
	//Serial.println(c);         // print the integer
}
*/
