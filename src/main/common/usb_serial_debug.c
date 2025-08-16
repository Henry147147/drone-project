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

#include <stdarg.h>
#include <stdlib.h>
#include "platform.h"

#include "common/printf.h"
#include "common/printf_serial.h"

#include "drivers/serial.h"
#include "drivers/serial_usb_vcp.h"

#include "usb_serial_debug.h"

static serialPort_t *usbDebugPort;

void usbSerialDebugInit(void)
{
    usbDebugPort = usbVcpOpen();
    if (!usbDebugPort) {
        return;
    }
    setPrintfSerialPort(usbDebugPort);
    printfSerialInit();
}

void usbSerialDebugLog(const char *fmt, ...)
{
    if (!usbDebugPort) {
        return;
    }
    va_list va;
    va_start(va, fmt);
    tfp_format(stdout_putp, stdout_putf, fmt, va);
    va_end(va);
    tfp_printf("\r\n");
}

void usbSerialDebugLogFloat(const char *label, float value)
{
    if (!usbDebugPort) {
        return;
    }
    int scaled = (int)(value * 100); // two decimal places
    int whole = scaled / 100;
    int frac = abs(scaled % 100);
    tfp_printf("%s %d.%02d\r\n", label, whole, frac);
}
