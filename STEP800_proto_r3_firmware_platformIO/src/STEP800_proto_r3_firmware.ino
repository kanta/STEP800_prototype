/*
 Name:		STEP800_proto_r3_firmware.ino

 target:    Arduino Zero
 Created:	2020/06/15 16:23:30
 Modified:  2021/02/18
 Author:	kanta / Yuske
*/

#include <Arduino.h>
#include "wiring_private.h" // pinPeripheral() function
//#include "samd.h" // for code sense @ Atmel Studio
#include <SPI.h>
#include <OSCMessage.h>
#include <Ethernet.h>
#include <SparkFundSPINConstants.h>
#include <SparkFunAutoDriver.h>

#include "utils/Debug.h"
#include "def.h"
Debug debug(&SerialUSB, DEBUG_LEVEL);

#define COMPILE_DATE __DATE__
#define COMPILE_TIME __TIME__
constexpr auto PROJECT_NAME = "STEP800proto_r3.1";

#define NUM_OF_MOTOR (8)

#define MOTOR_ID_ALL 255
#define MOTOR_ID_FIRST 1
#define MOTOR_ID_LAST 8

#define ledPin	13
byte mac[] = { 0x60, 0x95, 0xCE, 0x10, 0x03, 0x00 },
myId = 0;
#define IP_OFFSET 100
// IPAddress myIp(10, 0, 0, IP_OFFSET);
// IPAddress destIp(10, 0, 0, 10);
IPAddress myIp(192, 168, 0, IP_OFFSET);
IPAddress destIp(192, 168, 0, 11);
unsigned int outPort = 50100;
unsigned int inPort = 50000;

EthernetUDP Udp;
boolean isDestIpSet = false;
#define W5500_RESET_PIN A3

// STATUS register polling
#define STATUS_POLL_PERIOD   10	//[ms]

// these values will be initialized at setup()
bool busy[NUM_OF_MOTOR];
bool flag[NUM_OF_MOTOR];
bool HiZ[NUM_OF_MOTOR];
bool homeSwState[NUM_OF_MOTOR];
bool dir[NUM_OF_MOTOR];
bool uvloStatus[NUM_OF_MOTOR];
uint8_t motorStatus[NUM_OF_MOTOR];
uint8_t thermalStatus[NUM_OF_MOTOR];

bool reportBUSY[NUM_OF_MOTOR];
bool reportFLAG[NUM_OF_MOTOR];
bool reportHiZ[NUM_OF_MOTOR];
bool reportHomeSwStatus[NUM_OF_MOTOR];
bool reportDir[NUM_OF_MOTOR];
bool reportMotorStatus[NUM_OF_MOTOR];

bool reportSwEvn[NUM_OF_MOTOR];
bool reportCommandError[NUM_OF_MOTOR];
bool reportUVLO[NUM_OF_MOTOR];
bool reportThermalStatus[NUM_OF_MOTOR];
bool reportOCD[NUM_OF_MOTOR];
bool reportStall[NUM_OF_MOTOR];

// 74HC165 shift register for dip switch and busy/flag input
#define MISO3	3 // SERCOM2/PAD[1]
#define MOSI3	4 // SERCOM2/PAD[0] // dummy
#define SCK3	0 // SERCOM2/PAD[3]
#define LATCH3	A5
SPIClass SPI3(&sercom2, MISO3, SCK3, MOSI3, SPI_PAD_0_SCK_3, SERCOM_RX_PAD_1);
byte shiftResistorValue[3];

// L6470
#define L6470_MISO	6	// D6 /SERCOM3/PAD[2] miso
#define L6470_MOSI	11	// D11/SERCOM3/PAD[0] mosi
#define L6470_SCK	12	// D12/SERCOM3/PAD[3] sck
//SPIClass altSPI (&sercom1, 12, 13, 11, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);
SPIClass L6470SPI(&sercom3, L6470_MISO, L6470_SCK, L6470_MOSI, SPI_PAD_0_SCK_3, SERCOM_RX_PAD_2);// MISO/SCK/MOSI pins

#define L6470_CS_PIN A0
#define L6470_RESET_PIN A2

AutoDriver stepper[] = {
	AutoDriver(7, L6470_CS_PIN, L6470_RESET_PIN),
	AutoDriver(6, L6470_CS_PIN, L6470_RESET_PIN),
	AutoDriver(5, L6470_CS_PIN, L6470_RESET_PIN),
	AutoDriver(4, L6470_CS_PIN, L6470_RESET_PIN),
	AutoDriver(3, L6470_CS_PIN, L6470_RESET_PIN),
	AutoDriver(2, L6470_CS_PIN, L6470_RESET_PIN),
	AutoDriver(1, L6470_CS_PIN, L6470_RESET_PIN),
	AutoDriver(0, L6470_CS_PIN, L6470_RESET_PIN)
};

// microSD slot
#define SD_CS_PIN	4u
#define SD_DETECT_PIN	A4

#define SETUP_SW_PIN	5u

// servo mode
uint32_t lastServoUpdateTime;
int32_t targetPosition[NUM_OF_MOTOR]; // these values will be initialized at setup()
float kP[NUM_OF_MOTOR], kI[NUM_OF_MOTOR], kD[NUM_OF_MOTOR];
boolean isServoMode[NUM_OF_MOTOR];
constexpr auto position_tolerance = 20; // steps

// Tx, Rx LED
bool rxLedEnabled = false, txLedEnabled = false;
uint32_t RXL_blinkStartTime, TXL_blinkStartTime;
#define RXL_TXL_BLINK_DURATION	100 // ms

void setUSBPriority()
{
	const auto irqn = USB_IRQn;
	NVIC_DisableIRQ(irqn);
	NVIC_SetPriority(irqn, 2);
	NVIC_EnableIRQ(irqn);
}

void setup()
{
	//setUSBPriority();
	pinMode(ledPin, OUTPUT);
	pinMode(SD_CS_PIN, OUTPUT);
	pinMode(LATCH3, OUTPUT);
	pinMode(SD_DETECT_PIN, INPUT_PULLUP);
	pinMode(SETUP_SW_PIN, INPUT_PULLUP);

	SerialUSB.begin(115200);

	pinMode(W5500_RESET_PIN, OUTPUT);

	// Prepare pins
	pinMode(L6470_RESET_PIN, OUTPUT);
	pinMode(L6470_CS_PIN, OUTPUT);
	pinMode(L6470_MOSI, OUTPUT);
	pinMode(L6470_MISO, INPUT);
	pinMode(L6470_SCK, OUTPUT);

	// Shift registers
	SPI3.begin();
	pinPeripheral(MISO3, PIO_SERCOM_ALT);	// MISO
	//pinPeripheral(MOSI3, PIO_SERCOM_ALT);	// MOSI, not in use
	pinPeripheral(SCK3, PIO_SERCOM_ALT);		// SCK
	SPI3.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
	pinMode(LATCH3, OUTPUT);
	digitalWrite(LATCH3, HIGH);

	// Reset powerSTEP and set CS
	digitalWrite(L6470_RESET_PIN, HIGH);
	digitalWrite(L6470_RESET_PIN, LOW);
	delay(10);
	digitalWrite(L6470_RESET_PIN, HIGH);
	digitalWrite(L6470_CS_PIN, HIGH);

	// Start SPI for L6470
	L6470SPI.begin();
	pinPeripheral(L6470_MOSI, PIO_SERCOM_ALT);
	pinPeripheral(L6470_SCK, PIO_SERCOM_ALT);
	pinPeripheral(L6470_MISO, PIO_SERCOM_ALT);
	L6470SPI.setDataMode(SPI_MODE3);
	delay(10);

	// Configure L6470 and some variables
	for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
	{
		busy[i] = false;
		flag[i] = false;
		HiZ[i] = false;
		homeSwState[i] = false;
		dir[i] = false;
		uvloStatus[i] = false;
		motorStatus[i] = 0;
		thermalStatus[i] = 0;

		reportBUSY[i] = false;
		reportFLAG[i] = false;
		reportHiZ[i] = false;
		reportHomeSwStatus[i] = false;
		reportDir[i] = false;
		reportMotorStatus[i] = false;
		reportSwEvn[i] = false;
		reportCommandError[i] = false;
		reportUVLO[i] = false;
		reportThermalStatus[i] = false;
		reportOCD[i] = false;
		reportStall[i] = false;

		targetPosition[i] = 0;
		kP[i] = SERVO_KP;
		kI[i] = SERVO_KI;
		kD[i] = SERVO_KD;
		isServoMode[i] = false;

		stepper[i].SPIPortConnect(&L6470SPI);
		resetMotorDriver(i + MOTOR_ID_FIRST);
		digitalWrite(ledPin, HIGH);
		delay(20);
		digitalWrite(ledPin, LOW);
		delay(20);
	}

	// Configure W5500
	digitalWrite(W5500_RESET_PIN, HIGH);
	hasShiftRegisterUpdated();
	myId = shiftResistorValue[2];// shiftResistorValue[2];
	delay(1);
	resetEthernet();
}

void resetEthernet() {
	digitalWrite(W5500_RESET_PIN, LOW);
	digitalWrite(ledPin, HIGH);
	delay(10); // This delay is necessary to refresh the network connection.
	digitalWrite(W5500_RESET_PIN, HIGH);
	digitalWrite(ledPin, LOW);
	delay(1);
	mac[5] = IP_OFFSET + myId;
	myIp[3] = IP_OFFSET + myId;
	outPort = 50000 + IP_OFFSET + myId;
	Ethernet.begin(mac, myIp);
	Udp.begin(inPort);
}

void resetMotorDriver(uint8_t deviceID) {
	if (MOTOR_ID_FIRST <= deviceID && deviceID <= MOTOR_ID_LAST) {
		deviceID -= MOTOR_ID_FIRST;
		stepper[deviceID].resetDev();
		stepper[deviceID].configStepMode(STEP_FS_128);
		stepper[deviceID].setMaxSpeed(650.);
		stepper[deviceID].setLoSpdOpt(true);
		stepper[deviceID].setFullSpeed(15000.);
		stepper[deviceID].setAcc(2000.);
		stepper[deviceID].setDec(2000.);
		stepper[deviceID].setSlewRate(SR_530V_us);
		stepper[deviceID].setOCThreshold(OC_1500mA);
		stepper[deviceID].setOCShutdown(OC_SD_ENABLE);
		stepper[deviceID].setPWMFreq(PWM_DIV_1, PWM_MUL_0_75);
		stepper[deviceID].setVoltageComp(VS_COMP_DISABLE);
		stepper[deviceID].hardHiZ();
		stepper[deviceID].setSwitchMode(SW_USER);
		stepper[deviceID].setOscMode(EXT_16MHZ_OSCOUT_INVERT);
		stepper[deviceID].setRunKVAL(INITIAL_RUN_KVAL);
		stepper[deviceID].setAccKVAL(INITIAL_ACC_KVAL);
		stepper[deviceID].setDecKVAL(INITIAL_DEC_KVAL);
		stepper[deviceID].setHoldKVAL(INITIAL_HOLD_KVAL);
		stepper[deviceID].setParam(STALL_TH, 0x1F);
		stepper[deviceID].setParam(ALARM_EN, 0xFF);
		delay(1);
		stepper[deviceID].getStatus(); // clears error flags
	}
}

void turnOnRXL() {
	digitalWrite(PIN_LED_RXL, LOW); // turn on
	RXL_blinkStartTime = millis();
	rxLedEnabled = true;
}

void turnOnTXL() {
	digitalWrite(PIN_LED_TXL, LOW); // turn on
	TXL_blinkStartTime = millis();
	txLedEnabled = true;
}

/************************* float -> int cast helper for OSC message ****************************/
int getInt(OSCMessage &msg, uint8_t offset)
{
	int msgVal = 0;
	if (msg.isFloat(offset))
	{
		msgVal = (int) msg.getFloat(offset);
	}
	else
	{
		msgVal = msg.getInt(offset);
	}
	return msgVal;
}


void sendOneInt(char* address, int32_t data) {
	if (!isDestIpSet) { return; }
	OSCMessage newMes(address);
	newMes.add((int32_t)data);
	Udp.beginPacket(destIp, outPort);
	newMes.send(Udp);
	Udp.endPacket();
	newMes.empty();
	turnOnTXL();
}

void sendTwoInt(char* address, int32_t data1, int32_t data2) {
	if (!isDestIpSet) { return; }
	OSCMessage newMes(address);
	newMes.add((int32_t)data1).add((int32_t)data2);
	Udp.beginPacket(destIp, outPort);
	newMes.send(Udp);
	Udp.endPacket();
	newMes.empty();
	turnOnTXL();
}

void sendIdFloat(char* address, int32_t id, float data) {
	if (!isDestIpSet) { return; }
	OSCMessage newMes(address);
	newMes.add(id).add(data);
	Udp.beginPacket(destIp, outPort);
	newMes.send(Udp);
	Udp.endPacket();
	newMes.empty();
	turnOnTXL();
}

void sendOneString(char* address, const char* data) {
	OSCMessage newMes(address);
	newMes.add(data);
	Udp.beginPacket(destIp, outPort);
	newMes.send(Udp);
	Udp.endPacket();
	newMes.empty();
	turnOnTXL();
}

#pragma region config_commands_osc_listener
void setDestIp(OSCMessage& msg, int addrOffset) {
	bool bIpUpdated = false;
	OSCMessage newMes("/destIp");
	for (auto i = 0; i < 4; i++)
	{
		bIpUpdated |= (destIp[i] != Udp.remoteIP()[i]);
		newMes.add(Udp.remoteIP()[i]);
	}
	destIp = Udp.remoteIP();
	newMes.add(bIpUpdated);
	Udp.beginPacket(destIp, outPort);
	newMes.send(Udp);
	Udp.endPacket();
	newMes.empty();
	isDestIpSet = true;
	turnOnTXL();
	debug.println(F("setDestIp"), DEBUG_OSC);
}

void getVersion(OSCMessage& msg, int addrOffset) {
	String version = COMPILE_DATE;
	version += String(" ") + String(COMPILE_TIME) + String(" ") + String(PROJECT_NAME);
	sendOneString("/version", version.c_str());
}

// reset the motor driver chip and setup it
void resetMotorDriver(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		resetMotorDriver(motorID);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			resetMotorDriver(i);
		}
	}
}

// simply send reset command to the driverchip via SPI
void resetDev(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].resetDev();
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].resetDev();
		}
	}
}

void enableFlagReport(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	bool bEnable = getInt(msg, 1) > 0;
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		reportFLAG[motorID - 1] = bEnable;
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			reportFLAG[i] = bEnable;
		}
	}
}

void enableBusyReport(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	bool bEnable = getInt(msg, 1) > 0;
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		reportBUSY[motorID - 1] = bEnable;
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			reportBUSY[i] = bEnable;
		}
	}
}

void enableHizReport(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	bool bEnable = getInt(msg, 1) > 0;
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		reportHiZ[motorID - 1] = bEnable;
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			reportHiZ[i] = bEnable;
		}
	}
}
void enableHomeSwReport(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	bool bEnable = getInt(msg, 1) > 0;
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		reportHomeSwStatus[motorID - 1] = bEnable;
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			reportHomeSwStatus[i] = bEnable;
		}
	}
}
void enableDirReport(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	bool bEnable = getInt(msg, 1) > 0;
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		reportDir[motorID - 1] = bEnable;
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			reportDir[i] = bEnable;
		}
	}
}
void enableMotorStatusReport(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	bool bEnable = getInt(msg, 1) > 0;
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		reportMotorStatus[motorID - 1] = bEnable;
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			reportMotorStatus[i] = bEnable;
		}
	}
}
void enableSwEventReport(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	bool bEnable = getInt(msg, 1) > 0;
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		reportSwEvn[motorID - 1] = bEnable;
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			reportSwEvn[i] = bEnable;
		}
	}
}
void enableCommandErrorReport(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	bool bEnable = getInt(msg, 1) > 0;
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		reportCommandError[motorID - 1] = bEnable;
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			reportCommandError[i] = bEnable;
		}
	}
}
void enableUvloReport(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	bool bEnable = getInt(msg, 1) > 0;
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		reportUVLO[motorID - 1] = bEnable;
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			reportUVLO[i] = bEnable;
		}
	}
}
void enableThermalStatusReport(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	bool bEnable = getInt(msg, 1) > 0;
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		reportThermalStatus[motorID - 1] = bEnable;
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			reportThermalStatus[i] = bEnable;
		}
	}
}
void enableOverCurrentReport(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	bool bEnable = getInt(msg, 1) > 0;
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		reportOCD[motorID - 1] = bEnable;
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			reportOCD[i] = bEnable;
		}
	}
}
void enableStallReport(OSCMessage& msg, int addrOffset) {
	// uint8_t motorID = msg.getInt(0);
	// bool bEnable = msg.getInt(1) > 0;
	uint8_t motorID = getInt(msg, 0);
	bool bEnable = getInt(msg, 1) > 0;
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST)
	{
		reportStall[motorID - 1] = bEnable;
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			reportStall[i] = bEnable;
		}
	}
}

void getHomeSw(OSCMessage& msg, int addrOffset) {
	if (!isDestIpSet) { return; }
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		getHomeSw(motorID);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			getHomeSw(i + 1);
		}
	}
}
void getHomeSw(uint8_t motorID) {
	if (!isDestIpSet) { return; }
	OSCMessage newMes("/homeSw");
	newMes.add(motorID).add(homeSwState[motorID - MOTOR_ID_FIRST]).add(dir[motorID - MOTOR_ID_FIRST]);
	Udp.beginPacket(destIp, outPort);
	newMes.send(Udp);
	Udp.endPacket();
	newMes.empty();
	turnOnTXL();
}

void getBusy(OSCMessage& msg, int addrOffset) {
	if (!isDestIpSet) { return; }
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST)
	{
		sendTwoInt("/busy", motorID, busy[motorID - MOTOR_ID_FIRST]);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			sendTwoInt("/busy", i + MOTOR_ID_FIRST, busy[i]);
		}
	}
}
void getUvlo(OSCMessage& msg, int addrOffset) {
	if (!isDestIpSet) { return; }
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		sendTwoInt("/uvlo", motorID, uvloStatus[motorID - MOTOR_ID_FIRST]);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			sendTwoInt("/uvlo", i + 1, uvloStatus[i]);
		}
	}
}

void getMotorStatus(OSCMessage& msg, int addrOffset) {
	if (!isDestIpSet) { return; }
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST)
	{
		sendTwoInt("/motorStatus", motorID, motorStatus[motorID - MOTOR_ID_FIRST]);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			sendTwoInt("/motorStatus", i + 1, motorStatus[i]);
		}
	}
}

void getThermalStatus(OSCMessage& msg, int addrOffset) {
	if (!isDestIpSet) { return; }
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		sendTwoInt("/thermalStatus", motorID, thermalStatus[motorID - MOTOR_ID_FIRST]);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			sendTwoInt("/thermalStatus", i + 1, thermalStatus[i]);
		}
	}
}

void getStatus(OSCMessage& msg, int addrOffset) {
	if (!isDestIpSet) { return; }
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		sendTwoInt("/status", motorID, stepper[motorID - MOTOR_ID_FIRST].getStatus());
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			sendTwoInt("/status", i + MOTOR_ID_FIRST, stepper[i].getStatus());
		}
	}
}

void getStatusList(OSCMessage& msg, int addrOffset) {
	if (!isDestIpSet) { return; }
	OSCMessage newMes("/statusList");
	for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
		newMes.add((int32_t)stepper[i].getStatus());
	}

	Udp.beginPacket(destIp, outPort);
	newMes.send(Udp);
	Udp.endPacket();
	newMes.empty();
	turnOnTXL();
}

void setMicrostepMode(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	uint8_t microstepMode = constrain(getInt(msg, 1), STEP_FS, STEP_FS_128); // 0-7

	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].configStepMode(microstepMode);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].configStepMode(microstepMode);
		}
	}
}

void getMicrostepMode(OSCMessage& msg, int addrOffset) {
	if (!isDestIpSet) { return; }
	uint8_t motorID = getInt(msg, 0);

	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		sendTwoInt("/microstepMode", motorID, stepper[motorID - MOTOR_ID_FIRST].getStepMode());
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			sendTwoInt("/microstepMode", i + MOTOR_ID_FIRST, stepper[i].getStepMode());
		}
	}
}

void getHomeSwMode(OSCMessage& msg, int addrOffset) {
	if (!isDestIpSet) { return; }
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		sendTwoInt("/homeSwMode", motorID, stepper[motorID - MOTOR_ID_FIRST].getSwitchMode() > 0);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			sendTwoInt("/homeSwMode", i + MOTOR_ID_FIRST, stepper[i].getSwitchMode() > 0);
		}
	}
}

void setHomeSwMode(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	uint16_t switchMode = (getInt(msg, 1) > 0) ? SW_USER : SW_HARD_STOP;

	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].setSwitchMode(switchMode);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].setSwitchMode(switchMode);
		}
	}
}

void setStallThreshold(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	uint8_t threshold = getInt(msg, 1) & 0x7F; // 7bit

	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].setParam(STALL_TH, threshold);
		getStallThreshold(motorID);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].setParam(STALL_TH, threshold);
			getStallThreshold(i + 1);
		}
	}
	
}
void getStallThreshold(uint8_t motorId) {
	if (!isDestIpSet) { return; }
	uint8_t stall_th_raw = stepper[motorId - MOTOR_ID_FIRST].getParam(STALL_TH) & 0x7F;
	float threshold = (stall_th_raw + 1) * 31.25;
	sendIdFloat("/stallThreshold", motorId, threshold);
}
void getStallThreshold(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		getStallThreshold(motorID);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			getStallThreshold(i + MOTOR_ID_FIRST);
		}
	}
}

void setOverCurrentThreshold(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	uint8_t threshold = getInt(msg, 1) & 0x0F; // 4bit

	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].setParam(OCD_TH, threshold);
		getOverCurrentThreshold(motorID);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].setParam(OCD_TH, threshold);
			getOverCurrentThreshold(i + 1);
		}
	}
}
void getOverCurrentThreshold(uint8_t motorId) {
	if (!isDestIpSet) { return; }
	uint8_t ocd_th_raw = stepper[motorId - MOTOR_ID_FIRST].getParam(OCD_TH) & 0x0F;
	float threshold = (ocd_th_raw + 1) * 375;
	sendIdFloat("/overCurrentThreshold", motorId, threshold);
}
void getOverCurrentThreshold(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		getOverCurrentThreshold(motorID);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			getOverCurrentThreshold(i + MOTOR_ID_FIRST);
		}
	}
}

void setLowSpeedOptimizeThreshold(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	float stepsPerSecond = msg.getFloat(1);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].setMinSpeed(stepsPerSecond, true);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].setMinSpeed(stepsPerSecond, true);
		}
	}
}
void getLowSpeedOptimizeThreshold(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		getLowSpeedOptimizeThreshold(motorID);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			getLowSpeedOptimizeThreshold(i + 1);
		}
	}
}
void getLowSpeedOptimizeThreshold(uint8_t motorID) {
	if (!isDestIpSet) { return; }
	bool optimizationEnabled = (stepper[motorID - MOTOR_ID_FIRST].getParam(MIN_SPEED) & (1 << 12)) > 0;
	OSCMessage newMes("/lowSpeedOptimizeThreshold");
	newMes.add((int32_t)motorID);
	newMes.add(stepper[motorID - MOTOR_ID_FIRST].getMinSpeed());
	newMes.add(optimizationEnabled);
	Udp.beginPacket(destIp, outPort);
	newMes.send(Udp);
	Udp.endPacket();
	newMes.empty();
	turnOnTXL();
}

#pragma endregion config_commands_osc_listener

#pragma region KVAL_commands_osc_listener

void setKval(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);

	int hold = constrain(getInt(msg, 1), 0, 255);
	int run = constrain(getInt(msg, 2), 0, 255);
	int acc = constrain(getInt(msg, 3), 0, 255);
	int dec = constrain(getInt(msg, 4), 0, 255);

	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].setHoldKVAL(hold);
		stepper[motorID - MOTOR_ID_FIRST].setRunKVAL(run);
		stepper[motorID - MOTOR_ID_FIRST].setAccKVAL(acc);
		stepper[motorID - MOTOR_ID_FIRST].setDecKVAL(dec);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].setHoldKVAL(hold);
			stepper[i].setRunKVAL(run);
			stepper[i].setAccKVAL(acc);
			stepper[i].setDecKVAL(dec);
		}
	}
	debug.print(F("KVAL set to: "), DEBUG_OSC);
	debug.println(motorID, DEBUG_OSC);
}
void setHoldKval(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	uint8_t kvalInput = constrain(getInt(msg, 1), 0, 255);

	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].setHoldKVAL(kvalInput);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].setHoldKVAL(kvalInput);
		}
	}
}
void setRunKval(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	uint8_t kvalInput = constrain(getInt(msg, 1), 0, 255);

	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].setRunKVAL(kvalInput);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].setRunKVAL(kvalInput);
		}
	}
}
void setAccKval(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	uint8_t kvalInput = constrain(getInt(msg, 1), 0, 255);

	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].setAccKVAL(kvalInput);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].setAccKVAL(kvalInput);
		}
	}
}
void setDecKval(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	uint8_t kvalInput = constrain(getInt(msg, 1), 0, 255);

	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].setDecKVAL(kvalInput);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].setDecKVAL(kvalInput);
		}
	}
}

void getKval(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		getKval(motorID);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			getKval(i + 1);
		}
	}
}
void getKval(uint8_t motorID) {
	if (!isDestIpSet) { return; }
	OSCMessage newMes("/kval");
	newMes.add((int32_t)motorID);
	newMes.add((int32_t)stepper[motorID - MOTOR_ID_FIRST].getHoldKVAL());
	newMes.add((int32_t)stepper[motorID - MOTOR_ID_FIRST].getRunKVAL());
	newMes.add((int32_t)stepper[motorID - MOTOR_ID_FIRST].getAccKVAL());
	newMes.add((int32_t)stepper[motorID - MOTOR_ID_FIRST].getDecKVAL());
	Udp.beginPacket(destIp, outPort);
	newMes.send(Udp);
	Udp.endPacket();
	newMes.empty();
	turnOnTXL();
}

#pragma endregion KVAL_commands_osc_listener

#pragma region speed_commands_osc_listener

void setSpeedProfile(OSCMessage& msg, int addrOffset) {
	uint8_t target = getInt(msg, 0);

	float acc = msg.getFloat(1);
	float dec = msg.getFloat(2);
	float maxSpeed = msg.getFloat(3);

	if (MOTOR_ID_FIRST <= target && target <= MOTOR_ID_LAST) {
		stepper[target - MOTOR_ID_FIRST].setAcc(acc);
		stepper[target - MOTOR_ID_FIRST].setDec(dec);
		stepper[target - MOTOR_ID_FIRST].setMaxSpeed(maxSpeed);

	}
	else if (target == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].setAcc(acc);
			stepper[i].setDec(dec);
			stepper[i].setMaxSpeed(maxSpeed);
		}
	}
}

void setMaxSpeed(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	float stepsPerSecond = msg.getFloat(1);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].setMaxSpeed(stepsPerSecond);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].setMaxSpeed(stepsPerSecond);
		}
	}
}
// MIN_SPEED register is set by setLowSpeedOptimizeThreshold function.

void setFullstepSpeed(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	float stepsPerSecond = msg.getFloat(1);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].setFullSpeed(stepsPerSecond);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].setFullSpeed(stepsPerSecond);
		}
	}
}
void setAcc(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	float stepsPerSecondPerSecond = msg.getFloat(1);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].setAcc(stepsPerSecondPerSecond);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].setAcc(stepsPerSecondPerSecond);
		}
	}
}
void setDec(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	float stepsPerSecondPerSecond = msg.getFloat(1);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].setDec(stepsPerSecondPerSecond);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].setDec(stepsPerSecondPerSecond);
		}
	}
}

void getSpeedProfile(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		motorID -= MOTOR_ID_FIRST;
		getSpeedProfile(motorID);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			getSpeedProfile(i);
		}
	}
}
void getSpeedProfile(uint8_t motorID) {
	if (!isDestIpSet) { return; }
	OSCMessage newMes("/speedProfile");
	newMes.add((int32_t)motorID);
	newMes.add((float)stepper[motorID].getAcc());
	newMes.add((float)stepper[motorID].getDec());
	newMes.add((float)stepper[motorID].getMaxSpeed());
	Udp.beginPacket(destIp, outPort);
	newMes.send(Udp);
	Udp.endPacket();
	newMes.empty();
	turnOnTXL();
}

void getSpeed(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	float s;
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		s = stepper[motorID - MOTOR_ID_FIRST].getSpeed();
		if (dir[motorID - MOTOR_ID_FIRST] == REV) { s *= -1.0; }
		sendIdFloat("/speed", motorID, s);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			s = stepper[i].getSpeed();
			if (dir[i] == REV) { s *= -1.0; }
			sendIdFloat("/speed", i + 1, s);
		}
	}
}

#pragma endregion speed_commands_osc_listener

#pragma region motion_commands_osc_listener

void getPosition(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		sendTwoInt("/position", motorID, stepper[motorID - MOTOR_ID_FIRST].getPos());
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			sendTwoInt("/position", i + MOTOR_ID_FIRST, stepper[i].getPos());
		}
	}
}
void getMark(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		sendTwoInt("/mark", motorID, stepper[motorID - MOTOR_ID_FIRST].getMark());
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			sendTwoInt("/mark", i + MOTOR_ID_FIRST, stepper[i].getMark());
		}
	}
}

void run(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);

	float stepsPerSec = 0;
	if ( msg.isFloat(1) ) { stepsPerSec = msg.getFloat(1); }
	else if ( msg.isInt(1) ) { stepsPerSec = (float)getInt(msg, 1); }
	
	boolean dir = stepsPerSec > 0;
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].run(dir, abs(stepsPerSec));
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].run(dir, abs(stepsPerSec));
		}
	}
}

void move(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	int32_t numSteps = getInt(msg, 1);
	boolean dir = numSteps > 0;
	numSteps = abs(numSteps);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].move(dir, numSteps);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].move(dir, numSteps);
		}
	}
}
void goTo(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);

	int32_t pos = getInt(msg, 1);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].goTo(pos);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].goTo(pos);
		}
	}
}
void goToDir(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	boolean dir = getInt(msg, 1) > 0;
	int32_t pos = getInt(msg, 2);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].goToDir(dir, pos);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].goToDir(dir, pos);
		}
	}
}

void goUntil(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	boolean action = getInt(msg, 1) > 0;
	float stepsPerSec = 0;
	if (msg.isFloat(2)) { stepsPerSec = msg.getFloat(2); }
	else if (msg.isInt(2)) { stepsPerSec = (float)getInt(msg, 2); }
	boolean dir = stepsPerSec > 0.;
	stepsPerSec = abs(stepsPerSec);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		motorID -= MOTOR_ID_FIRST;
		if (!homeSwState[motorID]) {
			stepper[motorID].goUntil(action, dir, stepsPerSec);
		}
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			if (!homeSwState[i]) {
				stepper[i].goUntil(action, dir, stepsPerSec);
			}
		}
	}
}

void releaseSw(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	boolean action = getInt(msg, 1) > 0;
	boolean dir = getInt(msg, 2) > 0;
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].releaseSw(action, dir);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].releaseSw(action, dir);
		}
	}
}
void goHome(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].goHome();
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].goHome();
		}
	}
}
void goMark(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].goMark();
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].goMark();
		}
	}
}
void setMark(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	int32_t newMark = getInt(msg, 1);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].setMark(newMark);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].setMark(newMark);
		}
	}
}
void setPosition(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	int32_t newPos = getInt(msg, 1);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].setPos(newPos);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].setPos(newPos);
		}
	}
}
void resetPos(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		stepper[motorID - MOTOR_ID_FIRST].resetPos();
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			stepper[i].resetPos();
		}
	}
}

void softStop(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		isServoMode[motorID - MOTOR_ID_FIRST] = false;
		stepper[motorID - MOTOR_ID_FIRST].softStop();
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			isServoMode[i] = false;
			stepper[i].softStop();
		}
	}
}
void hardStop(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		isServoMode[motorID - MOTOR_ID_FIRST] = false;
		stepper[motorID - MOTOR_ID_FIRST].hardStop();
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			isServoMode[i] = false;
			stepper[i].hardStop();
		}
	}
}
void softHiZ(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		isServoMode[motorID - MOTOR_ID_FIRST] = false;
		stepper[motorID - MOTOR_ID_FIRST].softHiZ();
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			isServoMode[i] = false;
			stepper[i].softHiZ();
		}
	}
}
void hardHiZ(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		isServoMode[motorID - MOTOR_ID_FIRST] = false;
		stepper[motorID - MOTOR_ID_FIRST].hardHiZ();
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			isServoMode[i] = false;
			stepper[i].hardHiZ();
		}
	}
}

#pragma endregion motion_commands_osc_listener

#pragma region servo_commands_osc_listener

void setTargetPosition(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	int32_t position = getInt(msg, 1);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		targetPosition[motorID - MOTOR_ID_FIRST] = position;
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			targetPosition[i] = position;
		}
	}
	debug.print(F("setTargetPosition: "), DEBUG_OSC);
	debug.print(motorID, DEBUG_OSC);
	debug.print(F(", "), DEBUG_OSC);
	debug.println(position, DEBUG_OSC);
}

void setTargetPositionList(OSCMessage& msg, int addrOffset) {
	for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
		targetPosition[i] = getInt(msg, i);
	}
}

void enableServoMode(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	bool bEnable = getInt(msg, 1) > 0;
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		if (bEnable) {
			targetPosition[motorID - MOTOR_ID_FIRST] = stepper[motorID - MOTOR_ID_FIRST].getPos();
			reportBUSY[motorID - MOTOR_ID_FIRST] = false;
			reportMotorStatus[motorID - MOTOR_ID_FIRST] = false;
			reportDir[motorID - MOTOR_ID_FIRST] = false;
			stepper[motorID - MOTOR_ID_FIRST].hardStop();
		}
		else {
			stepper[motorID - MOTOR_ID_FIRST].softStop();
		}
		isServoMode[motorID - MOTOR_ID_FIRST] = bEnable;
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			if (bEnable) {
				targetPosition[i] = stepper[i].getPos();
				reportBUSY[i] = false;
				reportMotorStatus[i] = false;
				reportDir[i] = false;
				stepper[i].hardStop();
			}
			else {
				stepper[i].softStop();
			}
			isServoMode[i] = bEnable;
		}
	}
	debug.print(F("enableServoMode: "), DEBUG_OSC);
	debug.print(motorID, DEBUG_OSC);
	debug.print(", ", DEBUG_OSC);
	debug.println((uint8_t)bEnable, DEBUG_OSC);
}

void setServoParam(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	float _kp = msg.getFloat(1), _ki = msg.getFloat(2), _kd = msg.getFloat(3);
	if (_kp <= 0.0) _kp = 0;
	if (_ki <= 0.0) _ki = 0;
	if (_kd <= 0.0) _kd = 0;
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		kP[motorID - MOTOR_ID_FIRST] = _kp;
		kI[motorID - MOTOR_ID_FIRST] = _ki;
		kD[motorID - MOTOR_ID_FIRST] = _kd;
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			kP[i] = _kp;
			kI[i] = _ki;
			kD[i] = _kd;
		}
	}
}

void getServoParam(uint8_t motorID) {
	if (!isDestIpSet) { return; }
	OSCMessage newMes("/servoParam");
	newMes.add(motorID).add(kP[motorID-1]).add(kI[motorID-1]).add(kD[motorID-1]);
	Udp.beginPacket(destIp, outPort);
	newMes.send(Udp);
	Udp.endPacket();
	newMes.empty();
	turnOnTXL();
}

void getServoParam(OSCMessage& msg, int addrOffset) {
	uint8_t motorID = getInt(msg, 0);
	if (MOTOR_ID_FIRST <= motorID && motorID <= MOTOR_ID_LAST) {
		getServoParam(motorID);
	}
	else if (motorID == MOTOR_ID_ALL) {
		for (uint8_t i = 0; i < NUM_OF_MOTOR; i++) {
			getServoParam(i + 1);
		}
	}
}

#pragma endregion servo_commands_osc_listener

void OSCMsgReceive() {

	OSCMessage msgIN;
	int size;
	if ((size = Udp.parsePacket()) > 0) {
		while (size--)
			msgIN.fill(Udp.read());

		if (!msgIN.hasError()) {
			// some possible frequent messeages
			msgIN.route("/setTargetPosition", setTargetPosition);
			msgIN.route("/setTargetPositionList", setTargetPositionList);
			msgIN.route("/getPosition", getPosition);
			msgIN.route("/getSpeed", getSpeed);
			msgIN.route("/run", run);
			//msgIN.route("/runRaw", runRaw);

			// motion
			msgIN.route("/move", move);
			msgIN.route("/goTo", goTo);
			msgIN.route("/goToDir", goToDir);
			msgIN.route("/goUntil", goUntil);
			//msgIN.route("/goUntilRaw", goUntilRaw);
			msgIN.route("/releaseSw", releaseSw);
			msgIN.route("/goHome", goHome);
			msgIN.route("/goMark", goMark);
			msgIN.route("/setMark", setMark);
			msgIN.route("/getMark", getMark);
			msgIN.route("/setPosition", setPosition);
			msgIN.route("/resetPos", resetPos);
			msgIN.route("/resetDev", resetDev);
			msgIN.route("/softStop", softStop);
			msgIN.route("/hardStop", hardStop);
			msgIN.route("/softHiZ", softHiZ);
			msgIN.route("/hardHiZ", hardHiZ);

			// servo mode
			msgIN.route("/enableServoMode", enableServoMode);
			msgIN.route("/setServoParam", setServoParam);
			msgIN.route("/getServoParam", getServoParam);

			// speed
			msgIN.route("/setSpeedProfile", setSpeedProfile);
			msgIN.route("/setMaxSpeed", setMaxSpeed);
			msgIN.route("/setFullstepSpeed", setFullstepSpeed);
			msgIN.route("/setAcc", setAcc);
			msgIN.route("/setDec", setDec);
			msgIN.route("/getSpeedProfile", getSpeedProfile);

			// KVAL
			msgIN.route("/setKval", setKval);
			msgIN.route("/setAccKval", setAccKval);
			msgIN.route("/setDecKval", setDecKval);
			msgIN.route("/setRunKval", setRunKval);
			msgIN.route("/setHoldKval", setHoldKval);
			msgIN.route("/getKval", getKval);

			// config
			msgIN.route("/setDestIp", setDestIp);
			msgIN.route("/getVersion", getVersion);
			msgIN.route("/getStatus", getStatus);
			msgIN.route("/getStatusList", getStatusList);
			msgIN.route("/getHomeSw", getHomeSw);
			msgIN.route("/getBusy", getBusy);
			msgIN.route("/getUvlo", getUvlo);
			msgIN.route("/getMotorStatus", getMotorStatus);
			msgIN.route("/getThermalStatus", getThermalStatus);
			msgIN.route("/resetMotorDriver", resetMotorDriver);
			msgIN.route("/enableFlagReport", enableFlagReport);
			msgIN.route("/enableBusyReport", enableBusyReport);
			msgIN.route("/enableHizReport", enableHizReport);
			msgIN.route("/enableHomeSwReport", enableHomeSwReport);
			msgIN.route("/enableDirReport", enableDirReport);
			msgIN.route("/enableMotorStatusReport", enableMotorStatusReport);
			msgIN.route("/enableSwEventReport", enableSwEventReport);
			msgIN.route("/enableCommandErrorReport", enableCommandErrorReport);
			msgIN.route("/enableUvloReport", enableUvloReport);
			msgIN.route("/enableThermalStatusReport", enableThermalStatusReport);
			msgIN.route("/enableOverCurrentReport", enableOverCurrentReport);
			msgIN.route("/enableStallReport", enableStallReport);
			//msgIN.route("/getDir", getDir);

			msgIN.route("/setMicrostepMode", setMicrostepMode);
			msgIN.route("/getMicrostepMode", getMicrostepMode);
			msgIN.route("/getHomeSwMode", getHomeSwMode);
			msgIN.route("/setHomeSwMode", setHomeSwMode);
			msgIN.route("/setStallThreshold", setStallThreshold);
			msgIN.route("/getStallThreshold", getStallThreshold);
			msgIN.route("/setOverCurrentThreshold", setOverCurrentThreshold);
			msgIN.route("/getOverCurrentThreshold", getOverCurrentThreshold);
			msgIN.route("/setLowSpeedOptimizeThreshold", setLowSpeedOptimizeThreshold);
			msgIN.route("/getLowSpeedOptimizeThreshold", getLowSpeedOptimizeThreshold);


			//msgIN.route("/setSpdProfileRaw",setSpdProfileRaw);
			//msgIN.route("/setMaxSpeedRaw", setMaxSpeedRaw);
			//msgIN.route("/setMinSpeedRaw", setMinSpeedRaw);
			//msgIN.route("/setFullSpeedRaw", setFullSpeedRaw);
			//msgIN.route("/setAccRaw", setAccRaw);
			//msgIN.route("/setDecRaw", setDecRaw);
			//msgIN.route("/getSpdProfileRaw",getSpdProfileRaw);
		}
		turnOnRXL();
	}
}

bool hasShiftRegisterUpdated() {
	byte t[3];
	bool changed = false;
	digitalWrite(LATCH3, LOW);
	//delayMicroseconds(4);
	digitalWrite(LATCH3, HIGH);
	//byte t = shiftIn(dataPin, clockPin, MSBFIRST);
	//delayMicroseconds(4);
	for (uint8_t i = 0; i < 3; i++) {
		t[i] = SPI3.transfer(0);
		if (shiftResistorValue[i] != t[i]) {
			shiftResistorValue[i] = t[i];
			changed = true;
		}
	}
	return changed;
}

void updateFlagBusy() {
	for (uint8_t i = 1; i < 2; i--) {
		for (uint8_t j = 0; j < 4; j++) {
			uint8_t index = j;
			if (i == 0) {
				index = j + 4;
			}
			bool isBUSY = byteToBool(shiftResistorValue[i], j * 2);
			bool isFLAG = byteToBool(shiftResistorValue[i], j * 2 + 1);
			if (busy[index] != isBUSY) {
				busy[index] = isBUSY;
				if (reportBUSY[index]) {
					sendTwoInt("/busy", index + 1, isBUSY);
				}
			}
			if (flag[index] != isFLAG) {
				flag[index] = isFLAG;
				if (reportFLAG[index]) {
					sendTwoInt("/flag", index + 1, isFLAG);
				}
			}
		}
	}
}
bool byteToBool(byte val, uint8_t index) {
	return (val >> index) & 1;
}

void updateMyId() {
	if ( shiftResistorValue[2] != myId )
	{
		myId = shiftResistorValue[2];
		resetEthernet();
	}
}

void sendStatusDebug(char* address, int32_t data1, int32_t data2, int32_t data3) {
	if (!isDestIpSet) { return; }
	OSCMessage newMes(address);
	newMes.add(data1).add(data2).add(data3);
	Udp.beginPacket(destIp, outPort);
	newMes.send(Udp);
	Udp.endPacket();
	newMes.empty();
	turnOnTXL();
}

uint8_t convertThermalStatus(uint8_t input) {
	uint8_t output = 0;
	input &= 0x03;
	if ( (input >> 1) == LOW ) { output = 2; } // TH_SD flag, active low
	else { output = ((input & 0x01) == LOW); } // TH_WRN flag, active low
	return output;
}

void checkStatus() {
	uint32_t t;
	for (uint8_t i = 0; i < NUM_OF_MOTOR; i++)
	{
		const auto status = stepper[i].getStatus();
		// HiZ, high for HiZ
		t = (status & STATUS_HIZ) > 0;
		if (HiZ[i] != t)
		{
			HiZ[i] = t;
			if ( reportHiZ[i] ) sendTwoInt("/HiZ", i + 1, t);
		}
		// BUSY
		t = (status & STATUS_BUSY) == 0;
		if (busy[i] != t)
		{
			busy[i] = t;
			if ( reportBUSY[i] ) sendTwoInt("/busy", i + 1, t);
		}

		// DIR
		t = (status & STATUS_DIR) > 0;
		if (dir[i] != t)
		{
			dir[i] = t;
			if (reportDir[i]) sendTwoInt("/dir", i + 1, t);
		}
		// SW_F, low for open, high for close
		t = (status & STATUS_SW_F) > 0;
		if (homeSwState[i] != t)
		{
			homeSwState[i] = t;
			if (reportHomeSwStatus[i]) getHomeSw(i + 1);
		}
		// SW_EVN, active high, latched
		t = (status & STATUS_SW_EVN) > 0;
		if (t && reportSwEvn[i] ) sendOneInt("/swEvent", i + 1);
		// MOT_STATUS
		t = (status & STATUS_MOT_STATUS) >> 5;
		if (motorStatus[i] != t) {
			motorStatus[i] = t;
			if (reportMotorStatus[i]) sendTwoInt("/motorStatus", i + 1, motorStatus[i]);
		}
		// NOTPREF_CMD + WRONG_CMD, both active high
		t = (status & (STATUS_NOTPERF_CMD | STATUS_WRONG_CMD)) >> 7;
		if ((t != 0) && (reportCommandError[i])) sendTwoInt("/commandError", i + 1, t);
		// UVLO, active low
		t = (status & STATUS_UVLO) == 0;
		if ( t != uvloStatus[i] )
		{
			uvloStatus[i] = !uvloStatus[i];
			if ( reportUVLO[i] ) sendTwoInt("/uvlo", i + 1, uvloStatus[i]);
		}
		// TH_WRN & TH_SD, both active low
		t = convertThermalStatus((status & (STATUS_TH_WRN | STATUS_TH_SD)) >> 10);	
		if (thermalStatus[i] != t) {
			thermalStatus[i] = t;
			if (reportThermalStatus[i]) sendTwoInt("/thermalStatus", i + 1, thermalStatus[i]);
		}
		// OCD, active low, latched
		t = (status & STATUS_OCD) == 0;
		if (t && reportOCD[i]) sendOneInt("/overCurrent", i + 1);

		// STEP_LOSS_A&B, active low, latched
		t = (status & (STATUS_STEP_LOSS_A | STATUS_STEP_LOSS_B)) >> 13;
		if ((t !=  3) && reportStall[i]) sendOneInt("/stall", i + 1);
	}
}

void checkLED(uint32_t _currentTimeMillis) {
	if ( rxLedEnabled )
	{
		if ((uint32_t)(_currentTimeMillis - RXL_blinkStartTime) >= RXL_TXL_BLINK_DURATION)
		{
			rxLedEnabled = false;
			digitalWrite(PIN_LED_RXL, HIGH); // turn off
		}
	}
	if (txLedEnabled)
	{
		if ((uint32_t)(_currentTimeMillis - TXL_blinkStartTime) >= RXL_TXL_BLINK_DURATION)
		{
			txLedEnabled = false;
			digitalWrite(PIN_LED_TXL, HIGH); // turn off
		}
	}
}

void updateServo(uint32_t currentTimeMicros) {
	static float eZ1[NUM_OF_MOTOR] = { 0,0,0,0,0,0,0,0 },
		eZ2[NUM_OF_MOTOR] = { 0,0,0,0,0,0,0,0 },
		integral[NUM_OF_MOTOR] = { 0,0,0,0,0,0,0,0 };
	float spd = 0.0;
	if ((uint32_t)(currentTimeMicros - lastServoUpdateTime) >= 100) {
		for (uint8_t i = 0; i < 8; i++) {
			if (isServoMode[i]) {
				int32_t error = targetPosition[i] - stepper[i].getPos();
				integral[i] += ((error + eZ1[i]) / 2.0f);
				if (integral[i] > 1500.0) integral[i] = 1500.0;
				else if (integral[i] < -1500.0) integral[i] = -1500.0;
				if (abs(error) > position_tolerance) {
					double diff = error - eZ1[i];

					spd = error * kP[i] + integral[i] * kI[i] + diff * kD[i];
				}
				eZ2[i] = eZ1[i];
				eZ1[i] = error;
				float absSpd = abs(spd);
				if (absSpd < 1.) {
					spd = 0.0;
				}
				stepper[i].run((spd > 0), absSpd);
			}
		}
		lastServoUpdateTime = currentTimeMicros;
	}
}

void loop() {
	uint32_t currentTimeMillis = millis(),
		currentTimeMicros = micros();
	static uint32_t lastPollTime = 0;

	if ((uint32_t)(currentTimeMillis - lastPollTime) >= STATUS_POLL_PERIOD)
	{
		checkStatus();
		checkLED(currentTimeMillis);
		if (hasShiftRegisterUpdated()) {
			updateFlagBusy();
			updateMyId();
		}
		lastPollTime = currentTimeMillis;
	}
	OSCMsgReceive();
	updateServo(currentTimeMicros);
}
