/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>

extern "C" {
    #include "platform.h"
    #include "common/usb_serial_debug.h"
    #include "common/printf.h"
    #include "drivers/serial.h"

    static char logBuffer[64];
    static int logPos;
    static serialPort_t dummyPort;
    static serialPort_t *configuredPort;

    void serialWrite(serialPort_t *port, uint8_t ch);

    static void stubPutc(void *p, char c)
    {
        serialWrite((serialPort_t *)p, (uint8_t)c);
    }

    serialPort_t *usbVcpOpen(void)
    {
        return &dummyPort;
    }

    void setPrintfSerialPort(serialPort_t *port)
    {
        configuredPort = port;
    }

    void printfSerialInit(void)
    {
        init_printf(configuredPort, stubPutc);
    }

    void serialWrite(serialPort_t *port, uint8_t ch)
    {
        (void)port;
        logBuffer[logPos++] = ch;
    }

    int tfp_printf(const char *fmt, ...)
    {
        va_list va;
        va_start(va, fmt);
        int written = tfp_format(stdout_putp, stdout_putf, fmt, va);
        va_end(va);
        return written;
    }
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

class UsbSerialDebugTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        logPos = 0;
        memset(logBuffer, 0, sizeof(logBuffer));
    }
};

TEST_F(UsbSerialDebugTest, LogsTruncatedFloat)
{
    usbSerialDebugInit();
    usbSerialDebugLogFloat("val", 3.141f);
    logBuffer[logPos] = '\0';
    EXPECT_STREQ("val 3.14\r\n", logBuffer);
}

TEST_F(UsbSerialDebugTest, LogsFormattedMessage)
{
    usbSerialDebugInit();
    usbSerialDebugLog("x=%d", 42);
    logBuffer[logPos] = '\0';
    EXPECT_STREQ("x=42\r\n", logBuffer);
}

