datasheet = """0 High SDA0 SA5 PCLK - - -
1 High SCL0 SA4 DE - - -
2 High SDA1 SA3 LCD_VSYNC - - -
3 High SCL1 SA2 LCD_HSYNC - - -
4 High GPCLK0 SA1 DPI_D0 - - ARM_TDI
5 High GPCLK1 SA0 DPI_D1 - - ARM_TDO
6 High GPCLK2 SOE_N DPI_D2 - - ARM_RTCK
7 High SPI0_CE1_N SWE_N DPI_D3 - - -
8 High SPI0_CE0_N SD0 DPI_D4 - - -
9 Low SPI0_MISO SD1 DPI_D5 - - -
10 Low SPI0_MOSI SD2 DPI_D6 - - -
11 Low SPI0_SCLK SD3 DPI_D7 - - -
12 Low PWM0 SD4 DPI_D8 - - ARM_TMS
13 Low PWM1 SD5 DPI_D9 - - ARM_TCK
14 Low TXD0 SD6 DPI_D10 - - TXD1
15 Low RXD0 SD7 DPI_D11 - - RXD1
16 Low FL0 SD8 DPI_D12 CTS0 SPI1_CE2_N CTS1
17 Low FL1 SD9 DPI_D13 RTS0 SPI1_CE1_N RTS1
18 Low PCM_CLK SD10 DPI_D14 - SPI1_CE0_N PWM0
19 Low PCM_FS SD11 DPI_D15 - SPI1_MISO PWM1
20 Low PCM_DIN SD12 DPI_D16 - SPI1_MOSI GPCLK0
21 Low PCM_DOUT SD13 DPI_D17 - SPI1_SCLK GPCLK1
22 Low SD0_CLK SD14 DPI_D18 SD1_CLK ARM_TRST -
23 Low SD0_CMD SD15 DPI_D19 SD1_CMD ARM_RTCK -
24 Low SD0_DAT0 SD16 DPI_D20 SD1_DAT0 ARM_TDO -
25 Low SD0_DAT1 SD17 DPI_D21 SD1_DAT1 ARM_TCK -
26 Low SD0_DAT2 TE0 DPI_D22 SD1_DAT2 ARM_TDI -
27 Low SD0_DAT3 TE1 DPI_D23 SD1_DAT3 ARM_TMS -
28 None SDA0 SA5 PCM_CLK FL0 - -
29 None SCL0 SA4 PCM_FS FL1 - -
30 Low TE0 SA3 PCM_DIN CTS0 - CTS1
31 Low FL0 SA2 PCM_DOUT RTS0 - RTS1
32 Low GPCLK0 SA1 RING_OCLK TXD0 - TXD1
33 Low FL1 SA0 TE1 RXD0 - RXD1
34 High GPCLK0 SOE_N TE2 SD1_CLK - -
35 High SPI0_CE1_N SWE_N - SD1_CMD - -
36 High SPI0_CE0_N SD0 TXD0 SD1_DAT0 - -
37 Low SPI0_MISO SD1 RXD0 SD1_DAT1 - -
38 Low SPI0_MOSI SD2 RTS0 SD1_DAT2 - -
39 Low SPI0_SCLK SD3 CTS0 SD1_DAT3 - -
40 Low PWM0 SD4 - SD1_DAT4 SPI2_MISO TXD1
41 Low PWM1 SD5 TE0 SD1_DAT5 SPI2_MOSI RXD1
42 Low GPCLK1 SD6 TE1 SD1_DAT6 SPI2_SCLK RTS1
43 Low GPCLK2 SD7 TE2 SD1_DAT7 SPI2_CE0_N CTS1
44 None GPCLK1 SDA0 SDA1 TE0 SPI2_CE1_N -
45 None PWM1 SCL0 SCL1 TE1 SPI2_CE2_N -
"""
funcs = {}
for line in datasheet.strip().split('\n'):
    pin,defpull,alt0,alt1,alt2,alt3,alt4,alt5 = line.split()
    funcs[pin] = {'alt0': alt0, 'alt1': alt1, 'alt2': alt2, 'alt3': alt3, 'alt4': alt4,
                  'alt5': alt5, 'gpio_in': 'IN', 'gpio_out': 'OUT'}

from subprocess import *
pins = check_output("ssh o 'cat /sys/kernel/debug/pinctrl/3f200000.gpio/pins'",shell=True)
for line in pins.split('\n')[1:]:
    if not line: continue
    _,pin,_,_,fn,_ = line.split(' ',5)
    if int(pin) <= 45:
        print(pin,funcs[pin][fn])
    
