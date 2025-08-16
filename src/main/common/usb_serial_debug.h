#pragma once

void usbSerialDebugInit(void);
void usbSerialDebugLog(const char *fmt, ...);
void usbSerialDebugLogFloat(const char *label, float value);
