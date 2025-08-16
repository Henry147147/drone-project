/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>
#include <string.h>

extern "C" {
#include <stdarg.h>
#include <stdio.h>
#include "platform.h"
#include "drivers/serial.h"
#include "drivers/serial_usb_vcp.h"
#include "common/usb_serial_debug.h"
}

static serialPort_t testPort;
static serialPort_t *configuredPort;
static char writeBuffer[256];
static int writePos;

class UsbSerialDebugTest : public ::testing::Test {
protected:
    void SetUp() override {
        configuredPort = nullptr;
        writePos = 0;
        memset(writeBuffer, 0, sizeof(writeBuffer));
        usbSerialDebugInit();
    }
};

TEST_F(UsbSerialDebugTest, InitializesPort) {
    EXPECT_EQ(&testPort, configuredPort);
}

TEST_F(UsbSerialDebugTest, LogsMessageWithNewline) {
    usbSerialDebugLog("count %d", 7);
    writeBuffer[writePos] = '\0';
    EXPECT_STREQ("count 7\r\n", writeBuffer);
}

TEST_F(UsbSerialDebugTest, LogsTruncatedFloat) {
    usbSerialDebugLogFloat("temp", 3.14159f);
    writeBuffer[writePos] = '\0';
    EXPECT_STREQ("temp 3.14\r\n", writeBuffer);
}

extern "C" {

serialPort_t *usbVcpOpen(void) {
    return &testPort;
}

void serialWrite(serialPort_t *instance, uint8_t ch) {
    (void)instance;
    if (writePos < (int)sizeof(writeBuffer)) {
        writeBuffer[writePos++] = (char)ch;
    }
}

bool isSerialTransmitBufferEmpty(const serialPort_t *instance) {
    (void)instance;
    return true;
}

static serialPort_t *printfPort;

void setPrintfSerialPort(serialPort_t *port) {
    printfPort = port;
    configuredPort = port;
}

void *stdout_putp;
void (*stdout_putf)(void *, char);

void init_printf(void *putp, void (*putf)(void *, char)) {
    stdout_putp = putp;
    stdout_putf = putf;
}

static void stub_putc(void *p, char c) {
    (void)p;
    serialWrite(printfPort, (uint8_t)c);
}

void printfSerialInit(void) {
    init_printf(nullptr, stub_putc);
}

int tfp_format(void *putp, void (*putf)(void *, char), const char *fmt, va_list va) {
    char buf[128];
    int len = vsnprintf(buf, sizeof(buf), fmt, va);
    for (int i = 0; i < len; i++) {
        putf(putp, buf[i]);
    }
    return len;
}

int tfp_printf(const char *fmt, ...) {
    char buf[128];
    va_list va;
    va_start(va, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, va);
    va_end(va);
    for (int i = 0; i < len; i++) {
        stdout_putf(stdout_putp, buf[i]);
    }
    return len;
}

}
