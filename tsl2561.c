/*_____________________________________________________________________________
 │                                                                            |
 │ COPYRIGHT (C) 2022 Mihai Baneu                                             |
 │                                                                            |
 | Permission is hereby  granted,  free of charge,  to any person obtaining a |
 | copy of this software and associated documentation files (the "Software"), |
 | to deal in the Software without restriction,  including without limitation |
 | the rights to  use, copy, modify, merge, publish, distribute,  sublicense, |
 | and/or sell copies  of  the Software, and to permit  persons to  whom  the |
 | Software is furnished to do so, subject to the following conditions:       |
 |                                                                            |
 | The above  copyright notice  and this permission notice  shall be included |
 | in all copies or substantial portions of the Software.                     |
 |                                                                            |
 | THE SOFTWARE IS PROVIDED  "AS IS",  WITHOUT WARRANTY OF ANY KIND,  EXPRESS |
 | OR   IMPLIED,   INCLUDING   BUT   NOT   LIMITED   TO   THE  WARRANTIES  OF |
 | MERCHANTABILITY,  FITNESS FOR  A  PARTICULAR  PURPOSE AND NONINFRINGEMENT. |
 | IN NO  EVENT SHALL  THE AUTHORS  OR  COPYRIGHT  HOLDERS  BE LIABLE FOR ANY |
 | CLAIM, DAMAGES OR OTHER LIABILITY,  WHETHER IN AN ACTION OF CONTRACT, TORT |
 | OR OTHERWISE, ARISING FROM,  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR  |
 | THE USE OR OTHER DEALINGS IN THE SOFTWARE.                                 |
 |____________________________________________________________________________|
 |                                                                            |
 |  Author: Mihai Baneu                           Last modified: 13.Oct.2022  |
 |                                                                            |
 |___________________________________________________________________________*/

/*!
 * Original file: Adafruit_TSL2561_U.h
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 */

#include <stdint.h>
#include "tsl2561.h"

static tsl2561_hw_control_t hw;

static uint8_t read8(const uint8_t addr, const uint8_t reg)
{
    uint8_t data;

    hw.data_wr(addr, &reg, 1);
    hw.data_rd(addr, &data, 1);

    return data;
}

static uint16_t read16(const uint8_t addr, const uint8_t reg)
{
    uint16_t data;

    hw.data_wr(addr, &reg, 1);
    hw.data_rd(addr, (uint8_t *)&data, sizeof(data));

    return data;
}

static void write8(const uint8_t addr, const uint8_t reg, const uint8_t value)
{
    const uint8_t data[] = { reg, value };
    hw.data_wr(addr, data, sizeof(data));
}

static void enable(const uint8_t addr)
{
    /* Enable the device by setting the control bit to 0x03 */
    write8(addr, TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWERON);
}

static void disable(const uint8_t addr)
{
    /* Turn the device off to save power */
    write8(addr, TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF);
}

void tsl2561_init(tsl2561_hw_control_t hw_control)
{
    hw = hw_control;

    /* Make sure we're actually connected */
    uint8_t x = read8(hw.address, TSL2561_REGISTER_ID);
    if (x & 0x05) { // ID code for TSL2561
        //TODO trigger error return false;
    }

    /* Set default integration time and gain */
    tsl2561_config(TSL2561_INTEGRATIONTIME_101MS, TSL2561_GAIN_1X);
}

void tsl2561_config(tsl2561_integration_time_t time, tsl2561_gain_t gain)
{
    /* Enable the device by setting the control bit to 0x03 */
    enable(hw.address);

    /* Update the timing register */
    write8(hw.address, TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING, time | gain);

    /* Turn the device off to save power */
    disable(hw.address);
}

void tsl2561_start()
{
    /* Enable the device by setting the control bit to 0x03 */
    enable(hw.address);
}

void tsl2561_stop()
{
    /* Turn the device off to save power */
    disable(hw.address);
}

void tsl2561_read(uint16_t *broadband,uint16_t *ir)
{
    /* Reads a two byte value from channel 0 (visible + infrared) */
    *broadband = read16(hw.address, TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW);

    /* Reads a two byte value from channel 1 (infrared) */
    *ir = read16(hw.address, TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW);
}

uint32_t tsl2561_calculate_lux(uint16_t broadband, uint16_t ir, tsl2561_integration_time_t time, tsl2561_gain_t gain)
{
    uint32_t chScale;
    uint32_t channel1;
    uint32_t channel0;

    /* Make sure the sensor isn't saturated! */
    uint16_t clipThreshold;
    switch (time) {
        case TSL2561_INTEGRATIONTIME_13MS:
            clipThreshold = TSL2561_CLIPPING_13MS;
            break;
        case TSL2561_INTEGRATIONTIME_101MS:
            clipThreshold = TSL2561_CLIPPING_101MS;
            break;
        default:
            clipThreshold = TSL2561_CLIPPING_402MS;
            break;
    }

    /* Return 65536 lux if the sensor is saturated */
    if ((broadband > clipThreshold) || (ir > clipThreshold)) {
        return 65536;
    }

    /* Get the correct scale depending on the intergration time */
    switch (time) {
        case TSL2561_INTEGRATIONTIME_13MS:
            chScale = TSL2561_LUX_CHSCALE_TINT0;
            break;
        case TSL2561_INTEGRATIONTIME_101MS:
            chScale = TSL2561_LUX_CHSCALE_TINT1;
            break;
        default: /* No scaling ... integration time = 402ms */
            chScale = (1 << TSL2561_LUX_CHSCALE);
            break;
    }

    /* Scale for gain (1x or 16x) */
    if (!gain) chScale = chScale << 4;

    /* Scale the channel values */
    channel0 = (broadband * chScale) >> TSL2561_LUX_CHSCALE;
    channel1 = (ir * chScale) >> TSL2561_LUX_CHSCALE;

    /* Find the ratio of the channel values (Channel1/Channel0) */
    uint32_t ratio1 = 0;
    if (channel0 != 0) ratio1 = (channel1 << (TSL2561_LUX_RATIOSCALE+1)) / channel0;

    /* round the ratio value */
    uint32_t ratio = (ratio1 + 1) >> 1;

    uint32_t b, m;

#ifdef TSL2561_PACKAGE_CS
    if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1C))
    {b=TSL2561_LUX_B1C; m=TSL2561_LUX_M1C;}
    else if (ratio <= TSL2561_LUX_K2C)
    {b=TSL2561_LUX_B2C; m=TSL2561_LUX_M2C;}
    else if (ratio <= TSL2561_LUX_K3C)
    {b=TSL2561_LUX_B3C; m=TSL2561_LUX_M3C;}
    else if (ratio <= TSL2561_LUX_K4C)
    {b=TSL2561_LUX_B4C; m=TSL2561_LUX_M4C;}
    else if (ratio <= TSL2561_LUX_K5C)
    {b=TSL2561_LUX_B5C; m=TSL2561_LUX_M5C;}
    else if (ratio <= TSL2561_LUX_K6C)
    {b=TSL2561_LUX_B6C; m=TSL2561_LUX_M6C;}
    else if (ratio <= TSL2561_LUX_K7C)
    {b=TSL2561_LUX_B7C; m=TSL2561_LUX_M7C;}
    else if (ratio > TSL2561_LUX_K8C)
    {b=TSL2561_LUX_B8C; m=TSL2561_LUX_M8C;}
#else
    if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1T))
    {b=TSL2561_LUX_B1T; m=TSL2561_LUX_M1T;}
    else if (ratio <= TSL2561_LUX_K2T)
    {b=TSL2561_LUX_B2T; m=TSL2561_LUX_M2T;}
    else if (ratio <= TSL2561_LUX_K3T)
    {b=TSL2561_LUX_B3T; m=TSL2561_LUX_M3T;}
    else if (ratio <= TSL2561_LUX_K4T)
    {b=TSL2561_LUX_B4T; m=TSL2561_LUX_M4T;}
    else if (ratio <= TSL2561_LUX_K5T)
    {b=TSL2561_LUX_B5T; m=TSL2561_LUX_M5T;}
    else if (ratio <= TSL2561_LUX_K6T)
    {b=TSL2561_LUX_B6T; m=TSL2561_LUX_M6T;}
    else if (ratio <= TSL2561_LUX_K7T)
    {b=TSL2561_LUX_B7T; m=TSL2561_LUX_M7T;}
    else if (ratio > TSL2561_LUX_K8T)
    {b=TSL2561_LUX_B8T; m=TSL2561_LUX_M8T;}
#endif

    uint32_t temp;
    temp = ((channel0 * b) - (channel1 * m));

    /* Do not allow negative lux value */
    if (temp < 0) temp = 0;

    /* Round lsb (2^(LUX_SCALE-1)) */
    temp += (1 << (TSL2561_LUX_LUXSCALE-1));

    /* Strip off fractional portion */
    uint32_t lux = temp >> TSL2561_LUX_LUXSCALE;

    /* Signal I2C had no errors */
    return lux;
}
