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
#include "common/usb_serial_debug.h"
#include "common/printf.h"
#include "drivers/serial.h"
}

#include "gtest/gtest.h"
#include "unittest_macros.h"

// stub implementations -------------------------------------------------------

static serialPort_t stubPort;

typedef struct {
    uint8_t buffer[256];
    int pos;
} serialPortStub_t;

static serialPortStub_t serialWriteStub;

extern "C" {
void *stdout_putp;
putcf stdout_putf;

static void stdout_putc(void *p, char c)
{
    serialWrite((serialPort_t *)p, (uint8_t)c);
}

serialPort_t *usbVcpOpen(void)
{
    return &stubPort;
}

void setPrintfSerialPort(serialPort_t *port)
{
    stdout_putp = port;
}

void printfSerialInit(void)
{
    stdout_putf = stdout_putc;
}

void serialWrite(serialPort_t *instance, uint8_t ch)
{
    EXPECT_EQ(instance, &stubPort);
    EXPECT_LT(serialWriteStub.pos, (int)sizeof(serialWriteStub.buffer));
    serialWriteStub.buffer[serialWriteStub.pos++] = ch;
}

int tfp_format(void *putp, void (*putf)(void *, char), const char *fmt, va_list va)
{
    char buf[128];
    int len = vsnprintf(buf, sizeof(buf), fmt, va);
    for (int i = 0; i < len; i++) {
        putf(putp, buf[i]);
    }
    return len;
}

int tfp_printf(const char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    char buf[128];
    int len = vsnprintf(buf, sizeof(buf), fmt, va);
    va_end(va);
    for (int i = 0; i < len; i++) {
        stdout_putf(stdout_putp, buf[i]);
    }
    return len;
}
} // extern "C"

// tests ----------------------------------------------------------------------

class UsbSerialDebugTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        serialWriteStub.pos = 0;
        usbSerialDebugInit();
        serialWriteStub.pos = 0;
    }
};

TEST_F(UsbSerialDebugTest, LogPrintsMessageWithNewline)
{
    usbSerialDebugLog("val %d", 7);
    serialWriteStub.buffer[serialWriteStub.pos] = '\0';
    EXPECT_STREQ("val 7\r\n", (char *)serialWriteStub.buffer);
}

TEST_F(UsbSerialDebugTest, LogFloatFormatsValue)
{
    usbSerialDebugLogFloat("t", 3.14159f);
    serialWriteStub.buffer[serialWriteStub.pos] = '\0';
    EXPECT_STREQ("t 3.14\r\n", (char *)serialWriteStub.buffer);
}

