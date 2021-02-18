#ifndef _DEBUG_H_
#define _DEBUG_H_

#include <Arduino.h>

#define DEBUG_GENERAL 0x01
#define DEBUG_SENSOR 0x02
#define DEBUG_ACTION 0x04
#define DEBUG_OSC 0x08

class Debug
{

public:
    uint8_t outputLevel = 0;

    Debug(Stream *s, uint8_t d);
    int getFreeRam();
    void print(uint8_t, uint8_t level);
    void println(uint8_t, uint8_t level);
    void print(int8_t, uint8_t level);
    void println(int8_t, uint8_t level);
    void print(int16_t, uint8_t level);
    void println(int16_t, uint8_t level);
    void print(uint16_t, uint8_t level);
    void println(uint16_t, uint8_t level);
    void print(int32_t, uint8_t level);
    void println(int32_t, uint8_t level);
    void print(float, uint8_t level);
    void println(float, uint8_t level);
    void println(const __FlashStringHelper*, uint8_t level);
    void print(const __FlashStringHelper*, uint8_t level);
    void println(String, uint8_t level);
    void print(String, uint8_t level);
    void println(char, uint8_t level);
    void print(char, uint8_t level);
    void printlnNumHex(uint8_t, uint8_t level);
    void printNumHex(uint8_t, uint8_t level);
    void ledOn();
    void ledOff();

private:
    Stream *stream;
    uint32_t loopTimer;
    uint8_t devLevel;
};

#endif