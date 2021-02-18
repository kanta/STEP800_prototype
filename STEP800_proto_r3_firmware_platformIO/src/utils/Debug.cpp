#include "Debug.h"

#define DEBUG

Debug::Debug(Stream *s, uint8_t d) : stream(s), devLevel(d)
{
    outputLevel = devLevel;
    // outputLevel |= DEBUG_GENERAL;
    // outputLevel |= DEBUG_CONTROL;
    // outputLevel |= DEBUG_SENSOR;
    // outputLevel |= DEBUG_ACTION;
    // outputLevel |= DEBUG_UART;
};

int Debug::getFreeRam()
{
    extern int __heap_start, *__brkval;
    int v;

    v = (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);

#ifdef DEBUG
    if (DEBUG_GENERAL & outputLevel)
    {
        stream->print(F("Free RAM = "));
        stream->println(v, DEC);
    }
#endif
    return v;
}

void Debug::print(uint8_t val, uint8_t level)
{
#ifdef DEBUG
    if (level & outputLevel)
    {
        stream->print(val);
    }
#endif
}

void Debug::println(uint8_t val, uint8_t level)
{
#ifdef DEBUG
    if (level & outputLevel)
    {
        stream->println(val);
    }
#endif
}

void Debug::print(int8_t val, uint8_t level)
{
#ifdef DEBUG
    if (level & outputLevel)
    {
        stream->print(val);
    }
#endif
}

void Debug::println(int8_t val, uint8_t level)
{
#ifdef DEBUG
    if (level & outputLevel)
    {
        stream->println(val);
    }
#endif
}

void Debug::print(int16_t val, uint8_t level)
{
#ifdef DEBUG
    if (level & outputLevel)
    {
        stream->print(val);
    }
#endif
}

void Debug::println(int16_t val, uint8_t level)
{
#ifdef DEBUG
    if (level & outputLevel)
    {
        stream->println(val);
    }
#endif
}

void Debug::print(uint16_t val, uint8_t level)
{
#ifdef DEBUG
    if (level & outputLevel)
    {
        stream->print(val);
    }
#endif
}

void Debug::println(uint16_t val, uint8_t level)
{
#ifdef DEBUG
    if (level & outputLevel)
    {
        stream->println(val);
    }
#endif
}

void Debug::print(int32_t val, uint8_t level)
{
#ifdef DEBUG
    if (level & outputLevel)
    {
        stream->print(val);
    }
#endif
}

void Debug::println(int32_t val, uint8_t level)
{
#ifdef DEBUG
    if (level & outputLevel)
    {
        stream->println(val);
    }
#endif
}

void Debug::print(float val, uint8_t level)
{
#ifdef DEBUG
    if (level & outputLevel)
    {
        stream->print(val);
    }
#endif
}

void Debug::println(float val, uint8_t level)
{
#ifdef DEBUG
    if (level & outputLevel)
    {
        stream->println(val);
    }
#endif
}

void Debug::println(const __FlashStringHelper *str, uint8_t level)
{
#ifdef DEBUG
    if (level & outputLevel)
    {
        stream->println(str);
    }
#endif
}

void Debug::print(const __FlashStringHelper *str, uint8_t level)
{
#ifdef DEBUG
    if (level & outputLevel)
    {
        stream->print(str);
    }
#endif
}

void Debug::println(String str, uint8_t level)
{
#ifdef DEBUG
    if (level & outputLevel)
    {
        stream->println(str);
    }
#endif
}

void Debug::print(String str, uint8_t level)
{
#ifdef DEBUG
    if (level & outputLevel)
    {
        stream->print(str);
    }
#endif
}

void Debug::println(char chr, uint8_t level)
{
#ifdef DEBUG
    if (level & outputLevel)
    {
        stream->println(chr);
    }
#endif
}

void Debug::print(char chr, uint8_t level)
{
#ifdef DEBUG
    if (level & outputLevel)
    {
        stream->print(chr);
    }
#endif
}

void Debug::printlnNumHex(uint8_t val, uint8_t level)
{
#ifdef DEBUG
    if (level & outputLevel)
    {
        char bufChar[5];
        int ret = snprintf(bufChar, sizeof(bufChar), "0x%02x", val);
        if (ret < 0)
        {
            return;
        }
        stream->println(bufChar);
    }
#endif
}
void Debug::printNumHex(uint8_t val, uint8_t level)
{
#ifdef DEBUG
    if (level & outputLevel)
    {
        char bufChar[5];
        int ret = snprintf(bufChar, sizeof(bufChar), "0x%02x", val);
        if (ret < 0)
        {
            return;
        }
        stream->print(bufChar);
    }
#endif
}
/****************************************************************************************
*	@brief      just debug indicator
*	@return     none
****************************************************************************************/
void Debug::ledOn()
{
    digitalWrite(LED_BUILTIN, HIGH); // turn the indicator on
}
void Debug::ledOff()
{
    digitalWrite(LED_BUILTIN, LOW); // turn the indicator off
}