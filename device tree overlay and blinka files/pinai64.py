# https://github.com/adafruit/Adafruit_Blinka/issues/687

# pinai
# SPDX-FileCopyrightText: 2021 Melissa LeBlanc-Williams for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""DRA74x pin names"""
from adafruit_blinka.microcontroller.generic_linux.libgpiod_pin import Pin

# BeagleBone AI64
# P8_1 = DGND           # DGND
# P8_2 = DGND           # DGND
P8_3 = Pin((1, 20))  # GPIO1_24 - GPIO_24
P8_4 = Pin((1, 48))  # GPIO1_25 - GPIO_25
P8_5 = Pin((1, 33))  # GPIO7_1 - GPIO_193
P8_6 = Pin((1, 34))  # GPIO7_2 - GPIO_194
P8_7 = Pin((1, 15))  # TIMER4 - GPIO_165
P8_8 = Pin((1, 14))  # TIMER7 - GPIO_166
P8_9 = Pin((1, 17))  # TIMER5 - GPIO_178
P8_10 = Pin((1, 16))  # TIMER6 - GPIO_164
P8_11 = Pin((1, 60))  # GPIO3_11 - GPIO_75
P8_12 = Pin((1, 59))  # GPIO3_10 - GPIO_74
P8_13 = Pin((1, 89))  # EHRPWM2B - GPIO_107
P8_14 = Pin((1, 75))  # GPIO4_13 - GPIO_109
P8_15 = Pin((1, 61))  # GPIO4_3 - GPIO_99
P8_16 = Pin((1, 62))  # GPIO4_29 - GPIO_125
P8_17 = Pin((1, 3))  # GPIO8_18 - GPIO_242
P8_18 = Pin((1, 4))  # GPIO4_9 - GPIO_105
P8_19 = Pin((1, 88))  # EHRPWM2A - GPIO_106
P8_20 = Pin((1, 76))  # GPIO6_30 - GPIO_190
P8_21 = Pin((1, 30))  # GPIO6_29 - GPIO_189
P8_22 = Pin((1, 5))  # GPIO1_23 - GPIO_23
P8_23 = Pin((1, 31))  # GPIO1_22 - GPIO_22
P8_24 = Pin((1, 6))  # GPIO7_0 - GPIO_192
P8_25 = Pin((1, 35))  # GPIO6_31 - GPIO_191
P8_26 = Pin((1, 51))  # GPIO4_28 - GPIO_124
P8_27 = Pin((1, 71))  # GPIO4_23 - GPIO_119
P8_28 = Pin((1, 72))  # GPIO4_19 - GPIO_115
P8_29 = Pin((1, 73))  # GPIO4_22 - GPIO_118
P8_30 = Pin((1, 74))  # GPIO4_20 - GPIO_116
P8_31 = Pin((1, 32))  # UART5_CTSN - GPIO_238
P8_32 = Pin((1, 26))  # UART5_RTSN - GPIO_239
P8_33 = Pin((1, 25))  # UART4_RTSN - GPIO_237
P8_34 = Pin((1, 7))  # UART3_RTSN - GPIO_235
P8_35 = Pin((1, 69))  # UART4_CTSN - GPIO_236
P8_36 = Pin((1, 8))  # UART3_CTSN - GPIO_234
P8_37 = Pin((1, 106))  # UART5_TXD - GPIO_232
P8_38 = Pin((1, 105))  # UART5_RXD - GPIO_233
P8_39 = Pin((1, 69))  # GPIO8_6 - GPIO_230
P8_40 = Pin((1, 70))  # GPIO8_7 - GPIO_231
P8_41 = Pin((1, 67))  # GPIO8_4 - GPIO_228
P8_42 = Pin((1, 68))  # GPIO8_5 - GPIO_229
P8_43 = Pin((1, 65))  # GPIO8_2 - GPIO_226
P8_44 = Pin((1, 66))  # GPIO8_3 - GPIO_227
P8_45 = Pin((1, 79))  # GPIO8_0 - GPIO_224
P8_46 = Pin((1, 80))  # GPIO8_1 - GPIO_225

# P9_1 = DGND           # DGND - GPIO_0
# P9_2 = DGND           # DGND - GPIO_0
# P9_3 = VDD_3V3        # VDD_3V3 - GPIO_0
# P9_4 = VDD_3V3        # VDD_3V3 - GPIO_0
# P9_5 = VDD_5V         # VDD_5V - GPIO_0
# P9_6 = VDD_5V         # VDD_5V - GPIO_0
# P9_7 = SYS_5V         # SYS_5V - GPIO_0
# P9_8 = SYS_5V         # SYS_5V - GPIO_0
# P9_9 = PWR_BUT        # PWR_BUT - GPIO_0
# P9_10 = SYS_RESETN    # SYS_RESETn - GPIO_0
P9_11 = Pin((1, 11))  # UART4_RXD - GPIO_241
P9_12 = Pin((1, 45))  # GPIO5_0 - GPIO_128
P9_13 = Pin((1, 2))  # UART4_TXD - GPIO_172
P9_14 = Pin((1, 93))  # EHRPWM1A - GPIO_121
P9_15 = Pin((1, 47))  # GPIO3_12 - GPIO_76
P9_16 = Pin((1, 94))  # EHRPWM1B - GPIO_122
P9_17 = Pin((1, 28))  # I2C1_SCL - GPIO_209
P9_18 = Pin((1, 40))  # I2C1_SDA - GPIO_208
P9_19 = Pin((2, 1))  # I2C2_SCL - GPIO_195
P9_20 = Pin((2, 2))  # I2C2_SDA - GPIO_196
P9_21 = Pin((1, 39))  # UART3_TXD - GPIO_67
P9_22 = Pin((1, 38))  # UART3_RXD - GPIO_179
P9_23 = Pin((1, 10))  # GPIO7_11 - GPIO_203
P9_24 = Pin((1, 119))  # UART1_TXD - GPIO_175
P9_25 = Pin((1, 127))  # GPIO6_17 - GPIO_177
P9_26 = Pin((1, 118))  # UART1_RXD - GPIO_174
P9_27 = Pin((1, 46))  # GPIO4_15 - GPIO_111
P9_28 = Pin((2, 11))  # SPI1_CS0 - GPIO_113
P9_29 = Pin((2, 14))  # SPI1_D0 - GPIO_139
P9_30 = Pin((2, 13))  # SPI1_D1 - GPIO_140
P9_31 = Pin((2, 12))  # SPI1_SCLK - GPIO_138
# P9_32 = VDD_ADC       # VDD_ADC - GPIO_0
# P9_33 = AIN4          # AIN4 - GPIO_0
# P9_34 = GNDA_ADC      # GNDA_ADC - GPIO_0
# P9_35 = AIN6          # AIN6 - GPIO_0
# P9_36 = AIN5          # AIN5 - GPIO_0
# P9_37 = AIN2          # AIN2 - GPIO_0
# P9_38 = AIN3          # AIN3 - GPIO_0
# P9_39 = AIN0          # AIN0 - GPIO_0
# P9_40 = AIN1          # AIN1 - GPIO_0
P9_41 = Pin((2, 0))  # CLKOUT2 - GPIO_180
P9_42 = Pin((1, 123))  # GPIO4_18 - GPIO_114
# P9_43 = DGND          # DGND - GPIO_0
# P9_44 = DGND          # DGND - GPIO_0
# P9_45 = DGND          # DGND - GPIO_0
# P9_46 = DGND          # DGND - GPIO_0


##########################################
# User LEDs
#USR0 = Pin((2, 17))  # USR0 - GPIO3_17
#USR1 = Pin((4, 5))  # USR1 - GPIO5_5
#USR2 = Pin((2, 15))  # USR2 - GPIO3_15
#USR3 = Pin((2, 14))  # USR3 - GPIO3_14
#USR4 = Pin((2, 7))  # USR4 - GPIO3_7

# I2C2
I2C2_SCL = P9_19  # i2c2_scl
I2C2_SDA = P9_20  # i2c2_sda

# I2C5
I2C5_SCL = P9_17  # i2c5_scl
I2C5_SDA = P9_18  # i2c5_sda

# SPI0
SPI0_CS0 = P9_17
SPI0_D1 = P9_18
SPI0_D0 = P9_21
SPI0_SCLK = P9_22

# SPI1
SPI1_CS0 = P9_28
SPI1_CS1 = P9_42
SPI1_SCLK = P9_31
SPI1_D0 = P9_30
SPI1_D1 = P9_29

# UART0
UART0_TXD = P8_44
UART0_RXD = P8_36
UART0_RTSn = P8_34
UART0_CTSn = P8_45

# UART3
UART3_TXD = P9_21
UART3_RXD = P9_22
UART3_RTSn = P9_17
UART3_CTSn = P9_18

# UART5
UART5_TXD = P9_13
UART5_RXD = P9_11
UART5_RTSn = P8_6
UART5_CTSn = P8_5

# UART8
UART8_TXD = P8_37
UART8_RXD = P8_38
UART8_RTSn = P8_32
UART8_CTSn = P8_31

# UART10
UART10_TXD = P9_24
UART10_RXD = P9_26
UART10_RTSn = P8_4
UART10_CTSn = P8_3

# PWM
TIMER10 = P8_10
TIMER11 = P8_7
TIMER12 = P8_8
TIMER14 = P8_9

# ordered as i2cId, SCL, SDA
i2cPorts = (
    (5, I2C2_SCL, I2C2_SDA),  # default config
    (5, I2C2_SCL, I2C2_SDA),  # roboticscape config
    (3, I2C5_SCL, I2C5_SDA),  # roboticscape config
)

# ordered as spiId, sckId, mosiId, misoId
spiPorts = (
    (0, SPI0_SCLK, SPI0_D0, SPI0_D1),
    (1, SPI1_SCLK, SPI1_D1, SPI1_D0),
)

# ordered as uartId, txId, rxId
uartPorts = (
    (0, UART0_TXD, UART0_RXD),
    (3, UART3_TXD, UART3_RXD),
    (5, UART5_TXD, UART5_RXD),
    (8, UART8_TXD, UART8_RXD),
    (10, UART10_TXD, UART10_RXD),
)


