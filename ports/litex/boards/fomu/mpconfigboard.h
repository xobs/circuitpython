/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Scott Shawcroft for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

//Micropython setup

#define MICROPY_HW_BOARD_NAME       "Fomu"
#define MICROPY_HW_MCU_NAME         "VexRiscv"

#define FLASH_SIZE                  (0x100000)
#define FLASH_PAGE_SIZE             (0x4000)

#define AUTORESET_DELAY_MS 500
#define BOARD_FLASH_SIZE (FLASH_SIZE - 0x4000)

// On-board flash
#define SPI_FLASH_MOSI_PIN          &pin_PB05
#define SPI_FLASH_MISO_PIN          &pin_PB04
#define SPI_FLASH_SCK_PIN           &pin_PB03
#define SPI_FLASH_CS_PIN            &pin_PA15

#define DEFAULT_I2C_BUS_SCL (&pin_PB06)
#define DEFAULT_I2C_BUS_SDA (&pin_PB07)

#define DEFAULT_SPI_BUS_SCK (&pin_PB13)
#define DEFAULT_SPI_BUS_MOSI (&pin_PB15)
#define DEFAULT_SPI_BUS_MISO (&pin_PB14)

#define DEFAULT_UART_BUS_RX (&pin_PB11)
#define DEFAULT_UART_BUS_TX (&pin_PB10)
