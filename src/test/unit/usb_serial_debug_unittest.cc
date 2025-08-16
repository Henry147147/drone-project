/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option),
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern "C" {
    #include "platform.h"
    #include "common/printf.h"
    #include "common/usb_serial_debug.h"
    #include "drivers/serial.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static char outputBuf[256];
static size_t outputPos;
static bool openSucceeds = true;
static serialPort_t dummyPort;

static void resetBuffer(void)
{
    memset(outputBuf, 0, sizeof(outputBuf));
    outputPos = 0;
}

TEST(UsbSerialDebugTest, LogFormatsMessage)
{
    openSucceeds = true;
    resetBuffer();
    usbSerialDebugInit();
    usbSerialDebugLog("hello %d", 42);
    EXPECT_STREQ("hello 42\r\n", outputBuf);
}

TEST(UsbSerialDebugTest, LogFloatTruncates)
{
    openSucceeds = true;
    resetBuffer();
    usbSerialDebugInit();
    usbSerialDebugLogFloat("val", 3.1415f);
    EXPECT_STREQ("val 3.14\r\n", outputBuf);
}

TEST(UsbSerialDebugTest, NoOutputWithoutPort)
{
    openSucceeds = false;
    resetBuffer();
    usbSerialDebugInit();
    usbSerialDebugLog("fail");
    usbSerialDebugLogFloat("v", 1.23f);
    EXPECT_STREQ("", outputBuf);
}

// STUBS
extern "C" {
    void *stdout_putp;
    putcf stdout_putf;

    serialPort_t *usbVcpOpen(void)
    {
        return openSucceeds ? &dummyPort : NULL;
    }

    void setPrintfSerialPort(serialPort_t *p) { UNUSED(p); }
    void printfSerialInit(void) {}

    int tfp_format(void *putp, void (*putf)(void *, char), const char *fmt, va_list va)
    {
        UNUSED(putp); UNUSED(putf);
        vsnprintf(outputBuf + outputPos, sizeof(outputBuf) - outputPos, fmt, va);
        outputPos = strlen(outputBuf);
        return 0;
    }

    int tfp_printf(const char *fmt, ...)
    {
        va_list va;
        va_start(va, fmt);
        vsnprintf(outputBuf + outputPos, sizeof(outputBuf) - outputPos, fmt, va);
        va_end(va);
        outputPos = strlen(outputBuf);
        return 0;
    }

    void serialWrite(serialPort_t *instance, uint8_t ch)
    {
        UNUSED(instance);
        if (outputPos < sizeof(outputBuf) - 1) {
            outputBuf[outputPos++] = ch;
            outputBuf[outputPos] = '\0';
        }
    }
}

