EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "AGRIOT sensor node"
Date "2020-09-10"
Rev "1.0"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L RF_Module:RFM95W-868S2 U1
U 1 1 5DA76F79
P 3350 2200
F 0 "U1" H 3350 2881 50  0000 C CNN
F 1 "RFM95W-868S2" H 3350 2790 50  0000 C CNN
F 2 "RF_Module:HOPERF_RFM9XW_SMD" H 50  3850 50  0001 C CNN
F 3 "https://www.hoperf.com/data/upload/portal/20181127/5bfcbea20e9ef.pdf" H 50  3850 50  0001 C CNN
	1    3350 2200
	1    0    0    -1  
$EndComp
$Comp
L Arduino:ARDUINO_Pro-MiniALL-Aliexpress U2
U 1 1 5DA794B6
P 7700 2900
F 0 "U2" H 7700 4393 60  0000 C CNN
F 1 "ARDUINO_Pro-MiniALL" H 7700 4287 60  0000 C CNB
F 2 "arduino_shields:ARDUINO_Pro-MiniALLB_Aliexpress" H 7750 1900 60  0001 L CNN
F 3 "Arduino/Arduino-Pro-Mini-v13.pdf" H 7700 4181 60  0000 C CNN
	1    7700 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 1900 2500 1900
Wire Wire Line
	2850 2000 2500 2000
Wire Wire Line
	2850 2100 2500 2100
Wire Wire Line
	2850 2200 2500 2200
Wire Wire Line
	2850 2400 2500 2400
$Comp
L power:GND #PWR04
U 1 1 5DB5F32F
P 3450 3000
F 0 "#PWR04" H 3450 2750 50  0001 C CNN
F 1 "GND" H 3455 2827 50  0000 C CNN
F 2 "" H 3450 3000 50  0001 C CNN
F 3 "" H 3450 3000 50  0001 C CNN
	1    3450 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 2800 3450 2900
Wire Wire Line
	3250 2800 3250 2900
Wire Wire Line
	3250 2900 3350 2900
Connection ~ 3450 2900
Wire Wire Line
	3450 2900 3450 3000
Wire Wire Line
	3350 2800 3350 2900
Connection ~ 3350 2900
Wire Wire Line
	3350 2900 3450 2900
Wire Wire Line
	3350 1400 3350 1500
$Comp
L power:+3.3V #PWR03
U 1 1 5DB60116
P 3350 1400
F 0 "#PWR03" H 3350 1250 50  0001 C CNN
F 1 "+3.3V" H 3365 1573 50  0000 C CNN
F 2 "" H 3350 1400 50  0001 C CNN
F 3 "" H 3350 1400 50  0001 C CNN
	1    3350 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 3000 8750 3000
Wire Wire Line
	8500 3100 8750 3100
Text GLabel 2500 1900 0    50   Input ~ 0
SPI_SCK
Text GLabel 2500 2000 0    50   Input ~ 0
SPI_MOSI
Text GLabel 2500 2100 0    50   Input ~ 0
SPI_MISO
Text GLabel 2500 2200 0    50   Input ~ 0
RFM95_NSS
Text GLabel 2500 2400 0    50   Input ~ 0
RFM95_RST
Wire Wire Line
	8500 3200 8750 3200
Text GLabel 8750 3200 2    50   Input ~ 0
SPI_MISO
Text GLabel 8750 3100 2    50   Output ~ 0
SPI_MOSI
Text GLabel 8750 3000 2    50   Output ~ 0
RFM95_NSS
Wire Wire Line
	8500 3300 8750 3300
Text GLabel 8750 3300 2    50   Output ~ 0
SPI_SCK
Wire Wire Line
	8500 2900 8750 2900
Text GLabel 8750 2900 2    50   Output ~ 0
RFM95_RST
Wire Wire Line
	6900 3500 6700 3500
Wire Wire Line
	6700 3500 6700 3600
Wire Wire Line
	6700 3600 6900 3600
Wire Wire Line
	6700 3600 6700 3750
Connection ~ 6700 3600
Wire Wire Line
	6900 2200 6700 2200
Wire Wire Line
	6700 2200 6700 2300
Wire Wire Line
	6900 2300 6700 2300
Connection ~ 6700 2300
Wire Wire Line
	6700 2300 6700 2400
$Comp
L power:GND #PWR011
U 1 1 5DB6E03F
P 6700 2400
F 0 "#PWR011" H 6700 2150 50  0001 C CNN
F 1 "GND" H 6705 2227 50  0000 C CNN
F 2 "" H 6700 2400 50  0001 C CNN
F 3 "" H 6700 2400 50  0001 C CNN
	1    6700 2400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5DB6E457
P 6700 3750
F 0 "#PWR013" H 6700 3500 50  0001 C CNN
F 1 "GND" H 6705 3577 50  0000 C CNN
F 2 "" H 6700 3750 50  0001 C CNN
F 3 "" H 6700 3750 50  0001 C CNN
	1    6700 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 2100 6450 2100
$Comp
L power:+3.3V #PWR010
U 1 1 5DB6FE13
P 6450 2100
F 0 "#PWR010" H 6450 1950 50  0001 C CNN
F 1 "+3.3V" H 6465 2273 50  0000 C CNN
F 2 "" H 6450 2100 50  0001 C CNN
F 3 "" H 6450 2100 50  0001 C CNN
	1    6450 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 3100 6700 3100
Wire Wire Line
	6900 3200 6700 3200
Wire Wire Line
	6900 3300 6700 3300
Wire Wire Line
	6900 3400 6700 3400
Text GLabel 6700 3100 0    50   Input ~ 0
DTR
Text GLabel 6700 3200 0    50   Input ~ 0
TX
Text GLabel 6700 3300 0    50   Input ~ 0
RX
$Comp
L power:+3.3V #PWR012
U 1 1 5DB7F4B5
P 6700 3400
F 0 "#PWR012" H 6700 3250 50  0001 C CNN
F 1 "+3.3V" V 6715 3528 50  0000 L CNN
F 2 "" H 6700 3400 50  0001 C CNN
F 3 "" H 6700 3400 50  0001 C CNN
	1    6700 3400
	0    -1   -1   0   
$EndComp
Connection ~ 3350 1500
Wire Wire Line
	3350 1500 3350 1700
$Comp
L power:GND #PWR06
U 1 1 5DB84504
P 4150 1500
F 0 "#PWR06" H 4150 1250 50  0001 C CNN
F 1 "GND" V 4155 1372 50  0000 R CNN
F 2 "" H 4150 1500 50  0001 C CNN
F 3 "" H 4150 1500 50  0001 C CNN
	1    4150 1500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4000 1500 4150 1500
Wire Wire Line
	3700 1500 3350 1500
$Comp
L Device:C C1
U 1 1 5DB82B2B
P 3850 1500
F 0 "C1" V 3598 1500 50  0000 C CNN
F 1 "4.7uF/6.3v" V 3689 1500 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3888 1350 50  0001 C CNN
F 3 "~" H 3850 1500 50  0001 C CNN
	1    3850 1500
	0    1    1    0   
$EndComp
$Comp
L Jumper:SolderJumper_2_Open DIO1
U 1 1 5DB5C962
P 4100 2500
F 0 "DIO1" H 4300 2550 50  0000 C CNN
F 1 "SolderJumper_2_Open" H 5400 2500 50  0001 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm" H 4100 2500 50  0001 C CNN
F 3 "~" H 4100 2500 50  0001 C CNN
	1    4100 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 2200 8750 2200
Text GLabel 8750 2500 2    50   Input ~ 0
RFM95_DIO5
Text GLabel 8750 2200 2    50   Input ~ 0
RFM95_DIO0
Text GLabel 4400 2600 2    50   Output ~ 0
RFM95_DIO0
Text GLabel 4400 2500 2    50   Output ~ 0
RFM95_DIO1
$Comp
L Connector:Conn_Coaxial J1
U 1 1 5DB96F37
P 5300 1900
F 0 "J1" H 5400 1875 50  0000 L CNN
F 1 "Conn_Coaxial" H 5400 1784 50  0000 L CNN
F 2 "Connector_Coaxial:U.FL_Molex_MCRF_73412-0110_Vertical" H 5300 1900 50  0001 C CNN
F 3 " ~" H 5300 1900 50  0001 C CNN
	1    5300 1900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5DB98905
P 5300 2100
F 0 "#PWR07" H 5300 1850 50  0001 C CNN
F 1 "GND" H 5305 1927 50  0000 C CNN
F 2 "" H 5300 2100 50  0001 C CNN
F 3 "" H 5300 2100 50  0001 C CNN
	1    5300 2100
	1    0    0    -1  
$EndComp
$Comp
L Jumper:SolderJumper_2_Open DIO5
U 1 1 5DBC6685
P 4100 2100
F 0 "DIO5" H 4300 2150 50  0000 C CNN
F 1 "SolderJumper_2_Open" H 5400 2100 50  0001 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm" H 4100 2100 50  0001 C CNN
F 3 "~" H 4100 2100 50  0001 C CNN
	1    4100 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 1900 5100 1900
$Comp
L Jumper:SolderJumper_2_Open DIO2
U 1 1 5DBD387E
P 4100 2400
F 0 "DIO2" H 4300 2450 50  0000 C CNN
F 1 "SolderJumper_2_Open" H 5400 2400 50  0001 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm" H 4100 2400 50  0001 C CNN
F 3 "~" H 4100 2400 50  0001 C CNN
	1    4100 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 2500 3850 2500
Wire Wire Line
	3950 2400 3850 2400
Wire Wire Line
	3950 2100 3850 2100
Wire Wire Line
	4250 2500 4400 2500
Text GLabel 4400 2400 2    50   Output ~ 0
RFM95_DIO2
Text GLabel 4400 2100 2    50   Output ~ 0
RFM95_DIO5
Wire Wire Line
	4250 2100 4400 2100
Wire Wire Line
	4250 2400 4400 2400
$Comp
L Jumper:SolderJumper_2_Open DIO3
U 1 1 5DBE4978
P 4100 2300
F 0 "DIO3" H 4300 2350 50  0000 C CNN
F 1 "SolderJumper_2_Open" H 5400 2300 50  0001 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm" H 4100 2300 50  0001 C CNN
F 3 "~" H 4100 2300 50  0001 C CNN
	1    4100 2300
	1    0    0    -1  
$EndComp
Text GLabel 4400 2300 2    50   Output ~ 0
RFM95_DIO3
Wire Wire Line
	3950 2300 3850 2300
Wire Wire Line
	4250 2300 4400 2300
Wire Wire Line
	3850 2600 4400 2600
Text GLabel 8750 2600 2    50   Input ~ 0
RFM95_DIO3
Text GLabel 8750 2800 2    50   Input ~ 0
RFM95_DIO2
Text GLabel 8750 2700 2    50   Input ~ 0
RFM95_DIO1
Wire Wire Line
	6900 2400 6700 2400
Connection ~ 6700 2400
$Comp
L LoRaModule-rescue:DC-DC_Buck_Step_Module-dc-dc_module M1
U 1 1 5DC23DCA
P 4550 4750
F 0 "M1" H 4550 5065 50  0000 C CNN
F 1 "DC-DC_Buck_Step_Module" H 4550 4974 50  0000 C CNN
F 2 "library:DC-DC_Buck_Step_Module" H 4750 4350 50  0001 C CNN
F 3 "https://es.aliexpress.com/item/32742116421.html?spm=a2g0s.9042311.0.0.292863c06eUDtc" H 4750 4350 50  0001 C CNN
	1    4550 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 4700 3750 4700
Wire Wire Line
	5150 4700 5300 4700
Wire Wire Line
	4550 5250 4550 5350
$Comp
L power:+12V #PWR01
U 1 1 5DC2AD25
P 3750 4700
F 0 "#PWR01" H 3750 4550 50  0001 C CNN
F 1 "+12V" V 3765 4828 50  0000 L CNN
F 2 "" H 3750 4700 50  0001 C CNN
F 3 "" H 3750 4700 50  0001 C CNN
	1    3750 4700
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR05
U 1 1 5DC2B68E
P 5300 4700
F 0 "#PWR05" H 5300 4550 50  0001 C CNN
F 1 "+3.3V" V 5315 4828 50  0000 L CNN
F 2 "" H 5300 4700 50  0001 C CNN
F 3 "" H 5300 4700 50  0001 C CNN
	1    5300 4700
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5DC2FBA4
P 4550 5350
F 0 "#PWR02" H 4550 5100 50  0001 C CNN
F 1 "GND" H 4555 5177 50  0000 C CNN
F 2 "" H 4550 5350 50  0001 C CNN
F 3 "" H 4550 5350 50  0001 C CNN
	1    4550 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 4650 2000 4750
Wire Wire Line
	2000 4550 2000 4450
$Comp
L power:GND #PWR0101
U 1 1 5DC41414
P 2000 4750
F 0 "#PWR0101" H 2000 4500 50  0001 C CNN
F 1 "GND" H 2005 4577 50  0000 C CNN
F 2 "" H 2000 4750 50  0001 C CNN
F 3 "" H 2000 4750 50  0001 C CNN
	1    2000 4750
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0102
U 1 1 5DC41A80
P 2000 4450
F 0 "#PWR0102" H 2000 4300 50  0001 C CNN
F 1 "+12V" H 2015 4623 50  0000 C CNN
F 2 "" H 2000 4450 50  0001 C CNN
F 3 "" H 2000 4450 50  0001 C CNN
	1    2000 4450
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x03 J4
U 1 1 5DC5DFD8
P 1550 5700
F 0 "J4" H 1630 5742 50  0000 L CNN
F 1 "Screw_Terminal_01x03" H 1630 5651 50  0000 L CNN
F 2 "TerminalBlock_4Ucon:TerminalBlock_4Ucon_1x03_P3.50mm_Horizontal" H 1550 5700 50  0001 C CNN
F 3 "~" H 1550 5700 50  0001 C CNN
	1    1550 5700
	-1   0    0    1   
$EndComp
Wire Wire Line
	1750 5800 2000 5800
Wire Wire Line
	1750 5600 2000 5600
Wire Wire Line
	2000 5600 2000 5500
Wire Wire Line
	8500 2300 8750 2300
Text GLabel 8750 2300 2    50   BiDi ~ 0
SENSOR_DATA
Text GLabel 2400 5700 2    50   BiDi ~ 0
SENSOR_DATA
Wire Wire Line
	7800 4200 7800 4450
Wire Wire Line
	7900 4200 7900 4450
Text GLabel 7800 4450 3    50   Output ~ 0
I2C_SCL
Text GLabel 7900 4450 3    50   BiDi ~ 0
I2C_SDA
Wire Wire Line
	2400 5700 2250 5700
Connection ~ 2250 5700
Wire Wire Line
	2250 5700 1750 5700
$Comp
L power:+3.3V #PWR0105
U 1 1 5DC435A8
P 2250 5300
F 0 "#PWR0105" H 2250 5150 50  0001 C CNN
F 1 "+3.3V" H 2265 5473 50  0000 C CNN
F 2 "" H 2250 5300 50  0001 C CNN
F 3 "" H 2250 5300 50  0001 C CNN
	1    2250 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 5400 2250 5300
$Comp
L Device:R R1
U 1 1 5DC42ED8
P 2250 5550
F 0 "R1" H 2320 5596 50  0000 L CNN
F 1 "4.7k" H 2320 5505 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2180 5550 50  0001 C CNN
F 3 "~" H 2250 5550 50  0001 C CNN
	1    2250 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 6100 2000 5800
$Comp
L Connector:Screw_Terminal_01x03 J6
U 1 1 5DC62F44
P 1550 6900
F 0 "J6" H 1630 6942 50  0000 L CNN
F 1 "Screw_Terminal_01x03" H 1630 6851 50  0000 L CNN
F 2 "TerminalBlock_4Ucon:TerminalBlock_4Ucon_1x03_P3.50mm_Horizontal" H 1550 6900 50  0001 C CNN
F 3 "~" H 1550 6900 50  0001 C CNN
	1    1550 6900
	-1   0    0    1   
$EndComp
Wire Wire Line
	1750 7000 2000 7000
Wire Wire Line
	1750 6800 2000 6800
Wire Wire Line
	2000 6800 2000 6700
Text GLabel 2400 6900 2    50   BiDi ~ 0
SENSOR_AUX
Wire Wire Line
	2400 6900 2250 6900
Connection ~ 2250 6900
Wire Wire Line
	2250 6900 1750 6900
$Comp
L power:+3.3V #PWR0108
U 1 1 5DC62F5D
P 2250 6500
F 0 "#PWR0108" H 2250 6350 50  0001 C CNN
F 1 "+3.3V" H 2265 6673 50  0000 C CNN
F 2 "" H 2250 6500 50  0001 C CNN
F 3 "" H 2250 6500 50  0001 C CNN
	1    2250 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 6600 2250 6500
$Comp
L Device:R R2
U 1 1 5DC62F64
P 2250 6750
F 0 "R2" H 2320 6796 50  0000 L CNN
F 1 "4.7k" H 2320 6705 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2180 6750 50  0001 C CNN
F 3 "~" H 2250 6750 50  0001 C CNN
	1    2250 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 7300 2000 7000
Wire Wire Line
	8500 2400 8750 2400
Text GLabel 8750 2400 2    50   BiDi ~ 0
SENSOR_AUX
$Comp
L power:+3.3V #PWR0103
U 1 1 5DCA6333
P 2000 5500
F 0 "#PWR0103" H 2000 5350 50  0001 C CNN
F 1 "+3.3V" H 2015 5673 50  0000 C CNN
F 2 "" H 2000 5500 50  0001 C CNN
F 3 "" H 2000 5500 50  0001 C CNN
	1    2000 5500
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0104
U 1 1 5DCA684D
P 2000 6700
F 0 "#PWR0104" H 2000 6550 50  0001 C CNN
F 1 "+3.3V" H 2015 6873 50  0000 C CNN
F 2 "" H 2000 6700 50  0001 C CNN
F 3 "" H 2000 6700 50  0001 C CNN
	1    2000 6700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 5DCA81AF
P 2000 6100
F 0 "#PWR0106" H 2000 5850 50  0001 C CNN
F 1 "GND" V 2005 5972 50  0000 R CNN
F 2 "" H 2000 6100 50  0001 C CNN
F 3 "" H 2000 6100 50  0001 C CNN
	1    2000 6100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 5DCA9E30
P 2000 7300
F 0 "#PWR0107" H 2000 7050 50  0001 C CNN
F 1 "GND" V 2005 7172 50  0000 R CNN
F 2 "" H 2000 7300 50  0001 C CNN
F 3 "" H 2000 7300 50  0001 C CNN
	1    2000 7300
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Flag +3V3
U 1 1 5DCB3D94
P 9600 5550
F 0 "+3V3" H 9860 5598 50  0000 L CNN
F 1 "+3v3" H 9860 5553 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 9800 5550 50  0001 C CNN
F 3 "~" H 9800 5550 50  0001 C CNN
	1    9600 5550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5DCB8CEC
P 4550 6500
F 0 "R4" H 4620 6546 50  0000 L CNN
F 1 "4.7k" H 4620 6455 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4480 6500 50  0001 C CNN
F 3 "~" H 4550 6500 50  0001 C CNN
	1    4550 6500
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0109
U 1 1 5DCBB9A8
P 4550 6200
F 0 "#PWR0109" H 4550 6050 50  0001 C CNN
F 1 "+3.3V" H 4565 6373 50  0000 C CNN
F 2 "" H 4550 6200 50  0001 C CNN
F 3 "" H 4550 6200 50  0001 C CNN
	1    4550 6200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 5DCBE487
P 5350 7050
F 0 "#PWR0110" H 5350 6800 50  0001 C CNN
F 1 "GND" V 5355 6922 50  0000 R CNN
F 2 "" H 5350 7050 50  0001 C CNN
F 3 "" H 5350 7050 50  0001 C CNN
	1    5350 7050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 6200 4550 6350
Wire Wire Line
	4550 6650 4550 6850
Wire Wire Line
	4550 6850 4650 6850
Wire Wire Line
	5250 6850 5350 6850
Wire Wire Line
	5350 6850 5350 7050
$Comp
L Switch:SW_DIP_x01 SW1
U 1 1 5DCB6802
P 4950 6850
F 0 "SW1" H 4950 7117 50  0000 C CNN
F 1 "SW_DIP_x01" H 4950 7026 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_Push_1P1T_NO_CK_KSC6xxJ" H 4950 6850 50  0001 C CNN
F 3 "~" H 4950 6850 50  0001 C CNN
	1    4950 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 6850 4400 6850
Connection ~ 4550 6850
Text GLabel 4400 6850 0    50   Input ~ 0
SWITCH_CFG
Text GLabel 8750 3600 2    50   Input ~ 0
SWITCH_CFG
$Comp
L Connector:TestPoint_Flag SCL1
U 1 1 5DD2A2D1
P 9600 5700
F 0 "SCL1" H 9860 5748 50  0000 L CNN
F 1 "SCL" H 9860 5703 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 9800 5700 50  0001 C CNN
F 3 "~" H 9800 5700 50  0001 C CNN
	1    9600 5700
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Flag SDA1
U 1 1 5DD2CBA5
P 9600 5850
F 0 "SDA1" H 9860 5898 50  0000 L CNN
F 1 "SDA" H 9860 5853 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 9800 5850 50  0001 C CNN
F 3 "~" H 9800 5850 50  0001 C CNN
	1    9600 5850
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Flag GND1
U 1 1 5DD2F4CE
P 9600 6000
F 0 "GND1" H 9860 6048 50  0000 L CNN
F 1 "GND" H 9860 6003 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 9800 6000 50  0001 C CNN
F 3 "~" H 9800 6000 50  0001 C CNN
	1    9600 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 5550 9350 5550
Wire Wire Line
	9600 5700 9350 5700
Wire Wire Line
	9600 5850 9350 5850
Wire Wire Line
	9600 6000 9350 6000
$Comp
L power:+3.3V #PWR0111
U 1 1 5DD3E2E6
P 9350 5550
F 0 "#PWR0111" H 9350 5400 50  0001 C CNN
F 1 "+3.3V" V 9365 5678 50  0000 L CNN
F 2 "" H 9350 5550 50  0001 C CNN
F 3 "" H 9350 5550 50  0001 C CNN
	1    9350 5550
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5DD3EBFE
P 9350 6000
F 0 "#PWR0112" H 9350 5750 50  0001 C CNN
F 1 "GND" V 9355 5872 50  0000 R CNN
F 2 "" H 9350 6000 50  0001 C CNN
F 3 "" H 9350 6000 50  0001 C CNN
	1    9350 6000
	0    1    1    0   
$EndComp
Text GLabel 9350 5700 0    50   BiDi ~ 0
I2C_SCL
Text GLabel 9350 5850 0    50   BiDi ~ 0
I2C_SDA
$Comp
L Device:LED D1
U 1 1 5DD5469F
P 10150 2550
F 0 "D1" V 10189 2433 50  0000 R CNN
F 1 "LED" V 10098 2433 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 10150 2550 50  0001 C CNN
F 3 "~" H 10150 2550 50  0001 C CNN
	1    10150 2550
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R3
U 1 1 5DD592B1
P 10150 3000
F 0 "R3" H 10220 3046 50  0000 L CNN
F 1 "2.2kR" H 10220 2955 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 10080 3000 50  0001 C CNN
F 3 "~" H 10150 3000 50  0001 C CNN
	1    10150 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	10150 2700 10150 2850
Wire Wire Line
	10150 2400 10150 2250
Wire Wire Line
	10150 3150 10150 3350
$Comp
L power:+3V3 #PWR0113
U 1 1 5DD64DA8
P 10150 2250
F 0 "#PWR0113" H 10150 2100 50  0001 C CNN
F 1 "+3V3" H 10165 2423 50  0000 C CNN
F 2 "" H 10150 2250 50  0001 C CNN
F 3 "" H 10150 2250 50  0001 C CNN
	1    10150 2250
	1    0    0    -1  
$EndComp
Text GLabel 8750 3500 2    50   Output ~ 0
LED
Text GLabel 10150 3350 3    50   Input ~ 0
LED
Wire Wire Line
	8500 3700 8750 3700
Wire Wire Line
	8500 3800 8750 3800
Wire Wire Line
	7700 4200 7700 4450
Wire Wire Line
	7600 4200 7600 4450
Text GLabel 8750 3700 2    50   BiDi ~ 0
A2
Text GLabel 8750 3800 2    50   BiDi ~ 0
A3
Text GLabel 7700 4450 3    50   BiDi ~ 0
A6
Text GLabel 7600 4450 3    50   BiDi ~ 0
A7
$Comp
L Connector:TestPoint_Flag A2
U 1 1 5DD7D960
P 8200 5600
F 0 "A2" H 8460 5648 50  0000 L CNN
F 1 "+3v3" H 8460 5603 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 8400 5600 50  0001 C CNN
F 3 "~" H 8400 5600 50  0001 C CNN
	1    8200 5600
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Flag A3
U 1 1 5DD7D966
P 8200 5750
F 0 "A3" H 8460 5798 50  0000 L CNN
F 1 "SCL" H 8460 5753 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 8400 5750 50  0001 C CNN
F 3 "~" H 8400 5750 50  0001 C CNN
	1    8200 5750
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Flag A6
U 1 1 5DD7D96C
P 8200 5900
F 0 "A6" H 8460 5948 50  0000 L CNN
F 1 "SDA" H 8460 5903 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 8400 5900 50  0001 C CNN
F 3 "~" H 8400 5900 50  0001 C CNN
	1    8200 5900
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Flag A7
U 1 1 5DD7D972
P 8200 6050
F 0 "A7" H 8460 6098 50  0000 L CNN
F 1 "GND" H 8460 6053 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 8400 6050 50  0001 C CNN
F 3 "~" H 8400 6050 50  0001 C CNN
	1    8200 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 5600 7950 5600
Wire Wire Line
	8200 5750 7950 5750
Wire Wire Line
	8200 5900 7950 5900
Wire Wire Line
	8200 6050 7950 6050
Text GLabel 7950 5600 0    50   BiDi ~ 0
A2
Text GLabel 7950 5750 0    50   BiDi ~ 0
A3
Text GLabel 7950 5900 0    50   BiDi ~ 0
A6
Text GLabel 7950 6050 0    50   BiDi ~ 0
A7
$Comp
L Mechanical:MountingHole H1
U 1 1 5DD882F6
P 9300 900
F 0 "H1" H 9400 946 50  0000 L CNN
F 1 "MountingHole" H 9400 855 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 9300 900 50  0001 C CNN
F 3 "~" H 9300 900 50  0001 C CNN
	1    9300 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 3500 8750 3500
Wire Wire Line
	8500 3600 8750 3600
Wire Wire Line
	8500 2800 8750 2800
Wire Wire Line
	8750 2700 8500 2700
Wire Wire Line
	8500 2600 8750 2600
Wire Wire Line
	8500 2500 8750 2500
$Comp
L Mechanical:MountingHole H2
U 1 1 5DE2A18B
P 9300 1200
F 0 "H2" H 9400 1246 50  0000 L CNN
F 1 "MountingHole" H 9400 1155 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 9300 1200 50  0001 C CNN
F 3 "~" H 9300 1200 50  0001 C CNN
	1    9300 1200
	1    0    0    -1  
$EndComp
$Comp
L Graphic:SYM_ESD_Small #SYM1
U 1 1 5DE49BD1
P 7500 6800
F 0 "#SYM1" H 7500 6940 50  0001 C CNN
F 1 "SYM_ESD_Small" H 7500 6675 50  0001 C CNN
F 2 "Symbol:ESD-Logo_8.9x8mm_SilkScreen" H 7500 6810 50  0001 C CNN
F 3 "~" H 7500 6810 50  0001 C CNN
	1    7500 6800
	1    0    0    -1  
$EndComp
$Comp
L Graphic:Logo_Open_Hardware_Small #LOGO1
U 1 1 5DE4E7BA
P 1200 1150
F 0 "#LOGO1" H 1200 1425 50  0001 C CNN
F 1 "Logo_Open_Hardware_Small" H 1200 925 50  0001 C CNN
F 2 "Symbol:OSHW-Logo_7.5x8mm_SilkScreen" H 1200 1150 50  0001 C CNN
F 3 "~" H 1200 1150 50  0001 C CNN
	1    1200 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 4650 2000 4650
Wire Wire Line
	1650 4550 2000 4550
$Comp
L Connector:Screw_Terminal_01x02 J3
U 1 1 5DC3238B
P 1450 4650
F 0 "J3" H 1368 4325 50  0000 C CNN
F 1 "Screw_Terminal_01x02" H 1368 4416 50  0000 C CNN
F 2 "TerminalBlock_4Ucon:TerminalBlock_4Ucon_1x02_P3.50mm_Horizontal" H 1450 4650 50  0001 C CNN
F 3 "~" H 1450 4650 50  0001 C CNN
	1    1450 4650
	-1   0    0    1   
$EndComp
$EndSCHEMATC