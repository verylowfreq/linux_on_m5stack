#pragma once

#include <cstring>
#include <M5Stack.h>


#define FILENAME (strchr(__FILENAME__, '/') ? strrchr(__FILENAME__, '/') : __FILENAME__)

#define PANIC(...) do { Serial.printf("[PANIC] %s:%d %s() ", FILENAME, __LINE__, __func__); Serial.printf("" __VA_ARGS__); Serial.println(); \
                        M5.Lcd.printf("[PANIC] %s:%d %s() ", FILENAME, __LINE__, __func__); M5.Lcd.printf("" __VA_ARGS__); M5.Lcd.println(); \
                        while (true) { delay(100); }\
                   } while (false)

#define ERROR(...) do { Serial.printf("[ERROR] %s:%d %s() ", FILENAME, __LINE__, __func__); Serial.printf("" __VA_ARGS__); Serial.println(); \
                        M5.Lcd.printf("[ERROR] %s:%d %s() ", FILENAME, __LINE__, __func__); M5.Lcd.printf("" __VA_ARGS__); M5.Lcd.println(); \
                   } while (false)

#define DEBUG(...) do { Serial.printf("[DEBUG] %s:%d %s() ", FILENAME, __LINE__, __func__); Serial.printf("" __VA_ARGS__); Serial.println(); \
                        M5.Lcd.printf("[DEBUG] %s:%d %s() ", FILENAME, __LINE__, __func__); M5.Lcd.printf("" __VA_ARGS__); M5.Lcd.println(); \
                   } while (false)
