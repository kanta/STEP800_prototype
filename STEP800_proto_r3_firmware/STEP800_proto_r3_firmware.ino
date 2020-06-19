/*
 Name:		STEP800_proto_r3_firmware.ino
 Created:	2020/06/15 16:23:30
 Author:	kanta
*/

#include <Arduino.h>
#include "wiring_private.h" // pinPeripheral() function
#include "samd.h" // for code sense @ Atmel Studio
#include <SPI.h>
#include <OSCMessage.h>
#include <Ethernet.h>
#include <SparkFundSPINConstants.h>
#include <SparkFunAutoDriver.h>
#include "delay.h"

#define NUM_L6470 (8)

#define TARGET_ALL 255
#define TARGET_FIRST 1
#define TARGET_LAST 8

#define ledPin	13
byte mac[] = { 0x60, 0x95, 0xCE, 0x10, 0x03, 0x00 },
myId = 0;
IPAddress myIp(10, 0, 0, 100);
IPAddress destIp(10, 0, 0, 10);
unsigned int outPort = 20100;
unsigned int inPort = 20000;

EthernetUDP Udp;
boolean isDestIpSet = false;
#define W5500_RESET A3

#define POLL_DURATION   10
bool swState[NUM_L6470] = { 0,0,0,0,0,0,0,0 };
bool busy[NUM_L6470] = { 0,0,0,0,0,0,0,0 };
bool dir[NUM_L6470] = { 0,0,0,0,0,0,0,0 };
uint8_t motorStatus[NUM_L6470] = { 0,0,0,0,0,0,0,0 };

// 74HC165 shift register switch input
#define MISO3	3 // SERCOM2/PAD[1]
#define MOSI3	4 // SERCOM2/PAD[0] // dummy
#define SCK3	0 // SERCOM2/PAD[3]
#define LATCH3	A5
SPIClass SPI3(&sercom2, MISO3, SCK3, MOSI3, SPI_PAD_0_SCK_3, SERCOM_RX_PAD_1);

// L6470
#define L6470_MISO	6	// D6 /SERCOM3/PAD[2] miso
#define L6470_MOSI	11	// D11/SERCOM3/PAD[0] mosi
#define L6470_SCK	12	// D12/SERCOM3/PAD[3] sck
//SPIClass altSPI (&sercom1, 12, 13, 11, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);
SPIClass L6470SPI(&sercom3, L6470_MISO, L6470_SCK, L6470_MOSI, SPI_PAD_0_SCK_3, SERCOM_RX_PAD_2);// MISO/SCK/MOSI pins

#define L6470_CS_PIN A0
#define L6470_RESET_PIN A2


AutoDriver L6470[] = {
	AutoDriver(7, L6470_CS_PIN, L6470_RESET_PIN),
	AutoDriver(6, L6470_CS_PIN, L6470_RESET_PIN),
	AutoDriver(5, L6470_CS_PIN, L6470_RESET_PIN),
	AutoDriver(4, L6470_CS_PIN, L6470_RESET_PIN),
	AutoDriver(3, L6470_CS_PIN, L6470_RESET_PIN),
	AutoDriver(2, L6470_CS_PIN, L6470_RESET_PIN),
	AutoDriver(1, L6470_CS_PIN, L6470_RESET_PIN),
	AutoDriver(0, L6470_CS_PIN, L6470_RESET_PIN)
};

#define SD_CS	4u
#define SD_DETECT	A4

bool isOriginReturn[NUM_L6470];
bool isSendBusy[NUM_L6470];
bool isSendFlag[NUM_L6470];

uint8_t lastDir[NUM_L6470];
uint8_t lastSw[NUM_L6470];
uint8_t lastBusy[NUM_L6470];
uint8_t lastFlag[NUM_L6470];

byte shiftResistorValue[3];

// pid
uint32_t lastPidTime;
int32_t targetPosition[8] = { 0,0,0,0,0,0,0,0 };
float kP[8] = { 0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06 };
boolean isPositionFeedback[8] = { false, false, false, false, false, false, false, false };

void setup()
{

	pinMode(ledPin, OUTPUT);
	pinMode(SD_CS, OUTPUT);
	pinMode(LATCH3, OUTPUT);
	pinMode(SD_DETECT, INPUT_PULLUP);

	SerialUSB.begin(115200);

	pinMode(W5500_RESET, OUTPUT);

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
	// L6470SPI.setClockDivider(SPI_CLOCK_DIV128); // default 4
	pinPeripheral(L6470_MOSI, PIO_SERCOM_ALT);
	pinPeripheral(L6470_SCK, PIO_SERCOM_ALT);
	pinPeripheral(L6470_MISO, PIO_SERCOM_ALT);
	L6470SPI.setDataMode(SPI_MODE3);
	delay(10);

	// Configure L6470
	for (uint8_t i = 0; i < NUM_L6470; i++)
	{
		L6470[i].SPIPortConnect(&L6470SPI);
		L6470[i].configStepMode(STEP_FS_128);

		L6470[i].setMaxSpeed(10000);
		L6470[i].setFullSpeed(2000);
		L6470[i].setAcc(2000);
		L6470[i].setDec(2000);
		L6470[i].setSlewRate(SR_530V_us);//
		L6470[i].setOCThreshold(OC_3375mA);
		L6470[i].setOCShutdown(OC_SD_ENABLE);
		L6470[i].setPWMFreq(PWM_DIV_1, PWM_MUL_0_75);
		L6470[i].setVoltageComp(VS_COMP_DISABLE);
		L6470[i].hardHiZ();
		L6470[i].setSwitchMode(SW_USER);
		L6470[i].setOscMode(EXT_16MHZ_OSCOUT_INVERT);
		L6470[i].setRunKVAL(64);
		L6470[i].setAccKVAL(64);
		L6470[i].setDecKVAL(64);
		L6470[i].setHoldKVAL(32);
		L6470[i].setParam(ALARM_EN, 0xFF); 
		delay(1);
		L6470[i].getStatus(); // clears error flags
		digitalWrite(ledPin, HIGH);
		delay(20);
		digitalWrite(ledPin, LOW);
		delay(20);
	}

	digitalWrite(W5500_RESET, HIGH);
	delay(1);
	digitalWrite(W5500_RESET, LOW);
	delay(1);
	digitalWrite(W5500_RESET, HIGH);

	updateShiftResistor();
	uint8_t id = shiftResistorValue[2];
	SerialUSB.print("id: ");
	SerialUSB.println(id);
	mac[5] += id;
	myIp[3] += id;
	outPort += id;
	Ethernet.begin(mac, myIp);
	Udp.begin(inPort);

}

void sendOneDatum(char* address, int32_t data) {
	if (!isDestIpSet) { return; }
	OSCMessage newMes(address);
	newMes.add((int32_t)data);
	Udp.beginPacket(destIp, outPort);
	newMes.send(Udp);
	Udp.endPacket();
	newMes.empty();
}

void sendTwoData(char* address, int32_t target, int32_t data) {
	if (!isDestIpSet) { return; }
	OSCMessage newMes(address);
	newMes.add((int32_t)target);
	newMes.add((int32_t)data);
	Udp.beginPacket(destIp, outPort);
	newMes.send(Udp);
	Udp.endPacket();
	newMes.empty();
}

void sendIdFloat(char* address, int32_t id, float data) {
	if (!isDestIpSet) { return; }
	OSCMessage newMes(address);
	newMes.add(id).add(data);
	Udp.beginPacket(destIp, outPort);
	newMes.send(Udp);
	Udp.endPacket();
	newMes.empty();
}


void setDestIp(OSCMessage& msg, int addrOffset) {
	bool bIpUpdated = (destIp[3] != Udp.remoteIP()[3]);
	destIp = Udp.remoteIP();
	isDestIpSet = true;
	sendTwoData("/newDestIp", destIp[3], bIpUpdated);
	//Watchdog.reset();
}

#pragma region config

void setIsSendFlag(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	uint8_t flag = constrain(msg.getInt(1), 0, 7);
	if (target != TARGET_ALL) {
		setIsSendFlag(target, flag);
	}
	else {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			setIsSendFlag(i, flag);
		}
	}
}
void setIsSendFlag(uint8_t target, uint8_t val) {
	if (val == 0) {
		isSendFlag[target] = false;
	}
	else {
		isSendFlag[target] = true;
	}
}
void setIsSendBusy(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	uint8_t flag = constrain(msg.getInt(1), 0, 7);
	if (target != TARGET_ALL) {
		setIsSendBusy(target, flag);
	}
	else {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			setIsSendBusy(i, flag);
		}
	}
}
void setIsSendBusy(uint8_t target, uint8_t val) {
	if (val == 0) {
		isSendBusy[target] = false;
	}
	else {
		isSendBusy[target] = true;
	}
}
void getSw(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		sendTwoData("/getSw", target, lastSw[target - TARGET_FIRST]);
	}
	else if (target == TARGET_ALL) {
		OSCMessage newMes("/getSw");
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			newMes.add((int32_t)lastSw[i]);
		}
		Udp.beginPacket(destIp, outPort);
		newMes.send(Udp);
		Udp.endPacket();
		newMes.empty();
	}
}


void getBusy(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		sendTwoData("/getBusy", target, lastBusy[target - TARGET_FIRST]);
	}
	else if (target == TARGET_ALL) {
		OSCMessage newMes("/getBusy");
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			newMes.add((int32_t)lastBusy[i]);
		}
		Udp.beginPacket(destIp, outPort);
		newMes.send(Udp);
		Udp.endPacket();
		newMes.empty();
	}
}

void getStatus(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		sendTwoData("/status", target, L6470[target - TARGET_FIRST].getStatus());
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			sendTwoData("/status", i + TARGET_FIRST, L6470[i].getStatus());
		}
	}
}

void getStatusList(OSCMessage& msg, int addrOffset) {
	OSCMessage newMes("/statusList");
	for (uint8_t i = 0; i < NUM_L6470; i++) {
		newMes.add((int32_t)L6470[i].getStatus());
	}

	Udp.beginPacket(destIp, outPort);
	newMes.send(Udp);
	Udp.endPacket();
	newMes.empty();
}

void configStepMode(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	uint8_t stepMode = constrain(msg.getInt(1), STEP_FS, STEP_FS_128);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].configStepMode(stepMode);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].configStepMode(stepMode);
		}
	}
}

void getStepMode(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		sendTwoData("/stepMode", target, L6470[target - TARGET_FIRST].getStepMode());
	}
	else if (target == TARGET_ALL) {
		OSCMessage newMes("/stepMode");
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			newMes.add((int32_t)L6470[i].getStepMode());
		}
		Udp.beginPacket(destIp, outPort);
		newMes.send(Udp);
		Udp.endPacket();
		newMes.empty();
	}
}

void getSwMode(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		sendTwoData("/swMode", target, L6470[target - TARGET_FIRST].getSwitchMode());
	}
	else if (target == TARGET_ALL) {
		OSCMessage newMes("/swMode");
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			newMes.add((int32_t)L6470[i].getSwitchMode());
		}
		Udp.beginPacket(destIp, outPort);
		newMes.send(Udp);
		Udp.endPacket();
		newMes.empty();
	}
}

void setSwMode(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	uint8_t switchMode = (msg.getInt(1) > 0) ? SW_USER : SW_HARD_STOP;
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].setSwitchMode(switchMode);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].setSwitchMode(switchMode);
		}
	}
}

void setStallThreshold(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	uint8_t threshold = msg.getInt(1) & 0x7F;
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].setParam(STALL_TH, threshold);
		getStallThreshold(target);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].setParam(STALL_TH, threshold);
		}
	}
	
}
void getStallThreshold(uint8_t motorId) {
	uint8_t stall_th_raw = L6470[motorId - TARGET_FIRST].getParam(STALL_TH) & 0x7F;
	float threshold = (stall_th_raw + 1) * 31.25;
	sendIdFloat("/stallThreshold", motorId, threshold);
}
void getStallThreshold(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	getStallThreshold(target);
}

#pragma endregion config

#pragma region KVAL

void setKVAL(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	int hold = constrain(msg.getInt(1), 0, 255);
	int run = constrain(msg.getInt(2), 0, 255);
	int acc = constrain(msg.getInt(3), 0, 255);
	int dec = constrain(msg.getInt(4), 0, 255);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].setHoldKVAL(hold);
		L6470[target - TARGET_FIRST].setRunKVAL(run);
		L6470[target - TARGET_FIRST].setAccKVAL(acc);
		L6470[target - TARGET_FIRST].setDecKVAL(dec);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].setHoldKVAL(hold);
			L6470[i].setRunKVAL(run);
			L6470[i].setAccKVAL(acc);
			L6470[i].setDecKVAL(dec);
		}
	}
}
void setHoldKVAL(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	uint8_t kvalInput = constrain(msg.getInt(1), 0, 255);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].setHoldKVAL(kvalInput);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].setHoldKVAL(kvalInput);
		}
	}
}
void setRunKVAL(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	uint8_t kvalInput = constrain(msg.getInt(1), 0, 255);

	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].setRunKVAL(kvalInput);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].setRunKVAL(kvalInput);
		}
	}
}
void setAccKVAL(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	uint8_t kvalInput = constrain(msg.getInt(1), 0, 255);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].setAccKVAL(kvalInput);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].setAccKVAL(kvalInput);
		}
	}
}
void setDecKVAL(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	uint8_t kvalInput = constrain(msg.getInt(1), 0, 255);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].setDecKVAL(kvalInput);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].setDecKVAL(kvalInput);
		}
	}
}

void getKVAL(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		getKVAL(target);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			getKVAL(i + 1);
		}
	}
}
void getKVAL(uint8_t target) {
	OSCMessage newMes("/kval");
	newMes.add((int32_t)target);
	newMes.add((int32_t)L6470[target - TARGET_FIRST].getHoldKVAL());
	newMes.add((int32_t)L6470[target - TARGET_FIRST].getRunKVAL());
	newMes.add((int32_t)L6470[target - TARGET_FIRST].getAccKVAL());
	newMes.add((int32_t)L6470[target - TARGET_FIRST].getDecKVAL());
	Udp.beginPacket(destIp, outPort);
	newMes.send(Udp);
	Udp.endPacket();
	newMes.empty();
}

#pragma endregion

#pragma region speed

void setSpdProfile(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	float max = msg.getFloat(1);
	float min = msg.getFloat(2);
	float acc = msg.getFloat(3);
	float dec = msg.getFloat(4);

	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].setMaxSpeed(max);
		L6470[target - TARGET_FIRST].setMinSpeed(min);
		L6470[target - TARGET_FIRST].setAcc(acc);
		L6470[target - TARGET_FIRST].setDec(dec);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].setMaxSpeed(max);
			L6470[i].setMinSpeed(min);
			L6470[i].setAcc(acc);
			L6470[i].setDec(dec);
		}
	}
}

void setMaxSpeed(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	float stepsPerSecond = msg.getFloat(1);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].setMaxSpeed(stepsPerSecond);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].setMaxSpeed(stepsPerSecond);
		}
	}
}
void setMinSpeed(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	float stepsPerSecond = msg.getFloat(1);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].setMinSpeed(stepsPerSecond);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].setMinSpeed(stepsPerSecond);
		}
	}
}
void setFullstepSpeed(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	float stepsPerSecond = msg.getFloat(1);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].setFullSpeed(stepsPerSecond);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].setFullSpeed(stepsPerSecond);
		}
	}
}
void setAcc(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	float stepsPerSecondPerSecond = msg.getFloat(1);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].setAcc(stepsPerSecondPerSecond);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].setAcc(stepsPerSecondPerSecond);
		}
	}
}
void setDec(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	float stepsPerSecondPerSecond = msg.getFloat(1);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].setDec(stepsPerSecondPerSecond);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].setDec(stepsPerSecondPerSecond);
		}
	}
}

void getSpdProfile(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		getSpdProfile(target);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			getSpdProfile(i);
		}
	}
}
void getSpdProfile(uint8_t target) {
	OSCMessage newMes("/spd");
	newMes.add((int32_t)target);
	newMes.add((float)L6470[target - TARGET_FIRST].getMaxSpeed());
	newMes.add((float)L6470[target - TARGET_FIRST].getMinSpeed());
	newMes.add((float)L6470[target - TARGET_FIRST].getAcc());
	newMes.add((float)L6470[target - TARGET_FIRST].getDec());
	Udp.beginPacket(destIp, outPort);
	newMes.send(Udp);
	Udp.endPacket();
	newMes.empty();
}

#pragma endregion speed


#pragma region motion

void getPos(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		sendTwoData("/pos", target, L6470[target - TARGET_FIRST].getPos());
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			sendTwoData("/pos", i + TARGET_FIRST, L6470[i].getPos());
		}
	}
}
void getMark(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		sendTwoData("/mark", target, L6470[target - TARGET_FIRST].getMark());
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			sendTwoData("/mark", i + TARGET_FIRST, L6470[i].getMark());
		}
	}
}

void run(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);

	float stepsPerSec = msg.getFloat(1);
	boolean dir = stepsPerSec > 0;
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].run(dir, abs(stepsPerSec));
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].run(dir, abs(stepsPerSec));
		}
	}
}

void move(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	int32_t numSteps = msg.getInt(1);
	boolean dir = numSteps > 0;
	numSteps = abs(numSteps);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].move(dir, numSteps);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].move(dir, numSteps);
		}
	}
}
void goTo(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);

	int32_t pos = msg.getInt(1);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].goTo(pos);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].goTo(pos);
		}
	}
}
void goToDir(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);

	uint8_t dir = constrain(msg.getInt(1), 0, 1);
	int32_t pos = msg.getInt(2);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].goToDir(dir, pos);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].goToDir(dir, pos);
		}
	}
}
// todo: action??????
void goUntil(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);

	uint8_t action = msg.getInt(1);
	float stepsPerSec = msg.getFloat(2);
	boolean dir = stepsPerSec > 0.;
	stepsPerSec = abs(stepsPerSec);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].goUntil(action, dir, stepsPerSec);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].goUntil(action, dir, stepsPerSec);
		}
	}
}

void releaseSw(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);

	uint8_t action = msg.getInt(1);
	uint8_t dir = constrain(msg.getInt(2), 0, 1);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].releaseSw(action, dir);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].releaseSw(action, dir);
		}
	}
}
void goHome(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].goHome();
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].goHome();
		}
	}
}
void goMark(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].goMark();
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].goMark();
		}
	}
}
void setMark(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	unsigned long newMark = msg.getInt(1);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].setMark(newMark);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].setMark(newMark);
		}
	}
}
void setPos(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);

	int32_t newPos = msg.getInt(1);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].setPos(newPos);
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].setPos(newPos);
		}
	}
}
void resetPos(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].resetPos();
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].resetPos();
		}
	}
}
void resetDev(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].resetDev();
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].resetDev();
		}
	}
}
void softStop(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].softStop();
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].softStop();
		}
	}
}
void hardStop(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].hardStop();
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].hardStop();
		}
	}
}
void softHiZ(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].softHiZ();
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].softHiZ();
		}
	}
}
void hardHiZ(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		L6470[target - TARGET_FIRST].hardHiZ();
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			L6470[i].hardHiZ();
		}
	}
}

#pragma endregion motion

#pragma region pid


void setTargetPosition(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	int32_t position = msg.getInt(1);
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		targetPosition[target - TARGET_FIRST] = position;
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			targetPosition[i] = position;
		}
	}
}

void setTargetPositionList(OSCMessage& msg, int addrOffset) {
	for (uint8_t i = 0; i < NUM_L6470; i++) {
		targetPosition[i] = msg.getInt(i);
	}
}

void enablePositionFeedback(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	bool b = msg.getInt(1) > 0;
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		if (b) {
			targetPosition[target - TARGET_FIRST] = L6470[target - TARGET_FIRST].getPos();
		}
		else {
			L6470[target - TARGET_FIRST].softStop();
		}
		isPositionFeedback[target - TARGET_FIRST] = b;
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			if (b) {
				targetPosition[i] = L6470[i].getPos();
			}
			else {
				L6470[i].softStop();
			}
			isPositionFeedback[i] = b;
		}
	}
	SerialUSB.print("enablePositionFeedback: ");
	SerialUSB.print(target);
	SerialUSB.print(", ");
	SerialUSB.print(b);
	SerialUSB.println();
}

void setKp(OSCMessage& msg, int addrOffset) {
	uint8_t target = msg.getInt(0);
	float t = msg.getFloat(1);
	if (t <= 0.0) {
		t = 0;
	}
	if (TARGET_FIRST <= target && target <= TARGET_LAST) {
		kP[target - TARGET_FIRST] = t;
	}
	else if (target == TARGET_ALL) {
		for (uint8_t i = 0; i < NUM_L6470; i++) {
			kP[i] = t;
		}
	}
}
#pragma endregion pid

void OSCMsgReceive() {

	OSCMessage msgIN;
	int size;
	if ((size = Udp.parsePacket()) > 0) {
		while (size--)
			msgIN.fill(Udp.read());

		if (!msgIN.hasError()) {
			msgIN.route("/setDestIp", setDestIp);
			msgIN.route("/setIsSendFlag", setIsSendFlag);
			msgIN.route("/setIsSendBusy", setIsSendBusy);

			msgIN.route("/getStatus", getStatus);
			msgIN.route("/getStatusList", getStatusList);
			msgIN.route("/getSw", getSw);
			msgIN.route("/getBusy", getBusy);
			//msgIN.route("/getDir", getDir);

			msgIN.route("/configStepMode", configStepMode);
			msgIN.route("/getStepMode", getStepMode);
			msgIN.route("/getSwMode", getSwMode);
			msgIN.route("/setSwMode", setSwMode);
			msgIN.route("/setStallThreshold", setStallThreshold);
			msgIN.route("/getStallThreshold", getStallThreshold);

			msgIN.route("/setSpdProfile", setSpdProfile);
			msgIN.route("/setMaxSpeed", setMaxSpeed);
			msgIN.route("/setMinSpeed", setMinSpeed);
			msgIN.route("/setFullstepSpeed", setFullstepSpeed);
			msgIN.route("/setAcc", setAcc);
			msgIN.route("/setDec", setDec);
			msgIN.route("/getSpdProfile", getSpdProfile);

			//msgIN.route("/setSpdProfileRaw",setSpdProfileRaw);
			//msgIN.route("/setMaxSpeedRaw", setMaxSpeedRaw);
			//msgIN.route("/setMinSpeedRaw", setMinSpeedRaw);
			//msgIN.route("/setFullSpeedRaw", setFullSpeedRaw);
			//msgIN.route("/setAccRaw", setAccRaw);
			//msgIN.route("/setDecRaw", setDecRaw);
			//msgIN.route("/getSpdProfileRaw",getSpdProfileRaw);

			msgIN.route("/setKVAL", setKVAL);
			msgIN.route("/setAccKVAL", setAccKVAL);
			msgIN.route("/setDecKVAL", setDecKVAL);
			msgIN.route("/setRunKVAL", setRunKVAL);
			msgIN.route("/setHoldKVAL", setHoldKVAL);
			msgIN.route("/getKVAL", getKVAL);

			msgIN.route("/getPos", getPos);
			msgIN.route("/getMark", getMark);
			msgIN.route("/run", run);
			//msgIN.route("/runRaw", runRaw);
			msgIN.route("/move", move);
			msgIN.route("/goTo", goTo);
			msgIN.route("/goToDir", goToDir);
			msgIN.route("/goUntil", goUntil);
			//msgIN.route("/goUntilRaw", goUntilRaw);
			msgIN.route("/releaseSw", releaseSw);
			msgIN.route("/goHome", goHome);
			msgIN.route("/goMark", goMark);
			msgIN.route("/setMark", setMark);
			msgIN.route("/setPos", setPos);
			msgIN.route("/resetPos", resetPos);
			msgIN.route("/resetDev", resetDev);
			msgIN.route("/softStop", softStop);
			msgIN.route("/hardStop", hardStop);
			msgIN.route("/softHiZ", softHiZ);
			msgIN.route("/hardHiZ", hardHiZ);

			msgIN.route("/setTargetPosition", setTargetPosition);
			msgIN.route("/setTargetPositionList", setTargetPositionList);
			msgIN.route("/enablePositionFeedback", enablePositionFeedback);
			msgIN.route("/setKp", setKp);
		}
	}
}

bool updateShiftResistor() {

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

			bool isBusy = byteToBool(shiftResistorValue[i], j * 2);
			bool isFlag = byteToBool(shiftResistorValue[i], j * 2 + 1);

			if (lastBusy[index] != isBusy) {
				lastBusy[index] = isBusy;
				if (isSendBusy[index]) {
					sendTwoData("/busy", index + 1, isBusy);
				}
			}

			if (lastFlag[index] != isFlag) {
				lastFlag[index] = isFlag;
				if (isSendFlag[index]) {
					sendTwoData("/flag", index + 1, isFlag);
				}
			}
		}

	}
}
bool byteToBool(byte val, uint8_t index) {
	return (val >> index) & 1;
}

void updatePid(uint32_t currentTimeMicros) {
	if ((uint32_t)(currentTimeMicros - lastPidTime) >= 100) {
		for (uint8_t i = 0; i < 8; i++) {
			if (isPositionFeedback[i]) {
				float spd = (targetPosition[i] - L6470[i].getPos()) * kP[i];
				float absSpd = abs(spd);
				if (absSpd < 1.) {
					spd = 0.0;
				}
				L6470[i].run((spd > 0), absSpd);
			}
		}
		lastPidTime = currentTimeMicros;
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
}

void checkStatus() {
	for (uint8_t i = 0; i < NUM_L6470; i++)
	{
		const auto status = L6470[i].getStatus();
		uint32_t t = (status & STATUS_SW_F) > 0;
		if (swState[i] != t)
		{
			swState[i] = t;
			//sendTwoData("/sw", i + 1, t);
			sendStatusDebug("/sw", i + 1, t, status);
		}

		t = (status & STATUS_BUSY) > 0;
		if (busy[i] != t)
		{
			busy[i] = t;
			//sendTwoData("/busy", i + 1, t);
			sendStatusDebug("/busy", i + 1, t, status);
		}

		t = (status & STATUS_DIR) > 0;
		if (dir[i] != t)
		{
			dir[i] = t;
			//sendTwoData("/dir", i + 1, t);
			sendStatusDebug("/dir", i + 1, t, status);
		}

		t = (status & STATUS_MOT_STATUS) >> 5;
		if (motorStatus[i] != t) {
			motorStatus[i] = t;
			sendStatusDebug("/motorStatus", i + 1, t, status);
		}

		t = (status & (STATUS_STEP_LOSS_A | STATUS_STEP_LOSS_B)) >> 13;
		if (t != 3)
		{
			sendStatusDebug("/stall", i + 1, t, status);
		}

	}
}

void loop() {
	uint32_t currentTimeMillis = millis(),
		currentTimeMicros = micros();
	static uint32_t lastPollTime = 0;

	if ((uint32_t)(currentTimeMillis - lastPollTime) >= POLL_DURATION)
	{
		checkStatus();
		lastPollTime = currentTimeMillis;
	}
	OSCMsgReceive();

	if (updateShiftResistor()) {
		updateFlagBusy();
	}
	updatePid(currentTimeMicros);
}
