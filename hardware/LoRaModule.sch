EESchema Schematic File Version 4
LIBS:LoRaModule-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L RF_Module:RFM95W-868S2 U1
U 1 1 5DA76F79
P 3450 2500
F 0 "U1" H 3450 3181 50  0000 C CNN
F 1 "RFM95W-868S2" H 3450 3090 50  0000 C CNN
F 2 "RF_Module:HOPERF_RFM9XW_SMD" H 150 4150 50  0001 C CNN
F 3 "https://www.hoperf.com/data/upload/portal/20181127/5bfcbea20e9ef.pdf" H 150 4150 50  0001 C CNN
	1    3450 2500
	1    0    0    -1  
$EndComp
$Comp
L Arduino:ARDUINO_Pro-MiniALL-Aliexpress U2
U 1 1 5DA794B6
P 7400 2800
F 0 "U2" H 7400 4293 60  0000 C CNN
F 1 "ARDUINO_Pro-MiniALL" H 7400 4187 60  0000 C CNB
F 2 "arduino_shields:ARDUINO_Pro-MiniALLB_Aliexpress" H 7450 1800 60  0001 L CNN
F 3 "Arduino/Arduino-Pro-Mini-v13.pdf" H 7400 4081 60  0000 C CNN
	1    7400 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 2200 2600 2200
Wire Wire Line
	2950 2300 2600 2300
Wire Wire Line
	2950 2400 2600 2400
Wire Wire Line
	2950 2500 2600 2500
Wire Wire Line
	2950 2700 2600 2700
$Comp
L power:GND #PWR04
U 1 1 5DB5F32F
P 3550 3300
F 0 "#PWR04" H 3550 3050 50  0001 C CNN
F 1 "GND" H 3555 3127 50  0000 C CNN
F 2 "" H 3550 3300 50  0001 C CNN
F 3 "" H 3550 3300 50  0001 C CNN
	1    3550 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 3100 3550 3200
Wire Wire Line
	3350 3100 3350 3200
Wire Wire Line
	3350 3200 3450 3200
Connection ~ 3550 3200
Wire Wire Line
	3550 3200 3550 3300
Wire Wire Line
	3450 3100 3450 3200
Connection ~ 3450 3200
Wire Wire Line
	3450 3200 3550 3200
Wire Wire Line
	3450 1700 3450 1800
$Comp
L power:+3.3V #PWR03
U 1 1 5DB60116
P 3450 1700
F 0 "#PWR03" H 3450 1550 50  0001 C CNN
F 1 "+3.3V" H 3465 1873 50  0000 C CNN
F 2 "" H 3450 1700 50  0001 C CNN
F 3 "" H 3450 1700 50  0001 C CNN
	1    3450 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 2900 8450 2900
Wire Wire Line
	8200 3000 8450 3000
Text GLabel 2600 2200 0    50   Input ~ 0
SPI_SCK
Text GLabel 2600 2300 0    50   Input ~ 0
SPI_MOSI
Text GLabel 2600 2400 0    50   Input ~ 0
SPI_MISO
Text GLabel 2600 2500 0    50   Input ~ 0
RFM95_NSS
Text GLabel 2600 2700 0    50   Input ~ 0
RFM95_RST
Wire Wire Line
	8200 3100 8450 3100
Text GLabel 8450 3100 2    50   Input ~ 0
SPI_MISO
Text GLabel 8450 3000 2    50   Output ~ 0
SPI_MOSI
Text GLabel 8450 2900 2    50   Output ~ 0
RFM95_NSS
Wire Wire Line
	8200 3200 8450 3200
Text GLabel 8450 3200 2    50   Output ~ 0
SPI_SCK
Wire Wire Line
	8200 2800 8450 2800
Text GLabel 8450 2800 2    50   Output ~ 0
RFM95_RST
Wire Wire Line
	6600 3400 6400 3400
Wire Wire Line
	6400 3400 6400 3500
Wire Wire Line
	6400 3500 6600 3500
Wire Wire Line
	6400 3500 6400 3650
Connection ~ 6400 3500
Wire Wire Line
	6600 2100 6400 2100
Wire Wire Line
	6400 2100 6400 2200
Wire Wire Line
	6600 2200 6400 2200
Connection ~ 6400 2200
Wire Wire Line
	6400 2200 6400 2300
$Comp
L power:GND #PWR011
U 1 1 5DB6E03F
P 6400 2300
F 0 "#PWR011" H 6400 2050 50  0001 C CNN
F 1 "GND" H 6405 2127 50  0000 C CNN
F 2 "" H 6400 2300 50  0001 C CNN
F 3 "" H 6400 2300 50  0001 C CNN
	1    6400 2300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5DB6E457
P 6400 3650
F 0 "#PWR013" H 6400 3400 50  0001 C CNN
F 1 "GND" H 6405 3477 50  0000 C CNN
F 2 "" H 6400 3650 50  0001 C CNN
F 3 "" H 6400 3650 50  0001 C CNN
	1    6400 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 2000 6150 2000
$Comp
L power:+3.3V #PWR010
U 1 1 5DB6FE13
P 6150 2000
F 0 "#PWR010" H 6150 1850 50  0001 C CNN
F 1 "+3.3V" H 6165 2173 50  0000 C CNN
F 2 "" H 6150 2000 50  0001 C CNN
F 3 "" H 6150 2000 50  0001 C CNN
	1    6150 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 3000 6400 3000
Wire Wire Line
	6600 3100 6400 3100
Wire Wire Line
	6600 3200 6400 3200
Wire Wire Line
	6600 3300 6400 3300
Text GLabel 6400 3000 0    50   Input ~ 0
DTR
Text GLabel 6400 3100 0    50   Input ~ 0
TX
Text GLabel 6400 3200 0    50   Input ~ 0
RX
$Comp
L power:+3.3V #PWR012
U 1 1 5DB7F4B5
P 6400 3300
F 0 "#PWR012" H 6400 3150 50  0001 C CNN
F 1 "+3.3V" V 6415 3428 50  0000 L CNN
F 2 "" H 6400 3300 50  0001 C CNN
F 3 "" H 6400 3300 50  0001 C CNN
	1    6400 3300
	0    -1   -1   0   
$EndComp
Connection ~ 3450 1800
Wire Wire Line
	3450 1800 3450 2000
$Comp
L power:GND #PWR06
U 1 1 5DB84504
P 4250 1800
F 0 "#PWR06" H 4250 1550 50  0001 C CNN
F 1 "GND" V 4255 1672 50  0000 R CNN
F 2 "" H 4250 1800 50  0001 C CNN
F 3 "" H 4250 1800 50  0001 C CNN
	1    4250 1800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4100 1800 4250 1800
Wire Wire Line
	3800 1800 3450 1800
$Comp
L Device:C C1
U 1 1 5DB82B2B
P 3950 1800
F 0 "C1" V 3698 1800 50  0000 C CNN
F 1 "4.7uF/6.3v" V 3789 1800 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3988 1650 50  0001 C CNN
F 3 "~" H 3950 1800 50  0001 C CNN
	1    3950 1800
	0    1    1    0   
$EndComp
$Comp
L Jumper:SolderJumper_2_Open DIO1
U 1 1 5DB5C962
P 4200 2800
F 0 "DIO1" H 4400 2850 50  0000 C CNN
F 1 "SolderJumper_2_Open" H 5500 2800 50  0001 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm" H 4200 2800 50  0001 C CNN
F 3 "~" H 4200 2800 50  0001 C CNN
	1    4200 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 2100 8450 2100
Text GLabel 8450 2400 2    50   Input ~ 0
RFM95_DIO5
Text GLabel 8450 2100 2    50   Input ~ 0
RFM95_DIO0
Text GLabel 4500 2900 2    50   Output ~ 0
RFM95_DIO0
Text GLabel 4500 2800 2    50   Output ~ 0
RFM95_DIO1
$Comp
L Connector:Conn_Coaxial J1
U 1 1 5DB96F37
P 5400 2200
F 0 "J1" H 5500 2175 50  0000 L CNN
F 1 "Conn_Coaxial" H 5500 2084 50  0000 L CNN
F 2 "Connector_Coaxial:U.FL_Molex_MCRF_73412-0110_Vertical" H 5400 2200 50  0001 C CNN
F 3 " ~" H 5400 2200 50  0001 C CNN
	1    5400 2200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5DB98905
P 5400 2400
F 0 "#PWR07" H 5400 2150 50  0001 C CNN
F 1 "GND" H 5405 2227 50  0000 C CNN
F 2 "" H 5400 2400 50  0001 C CNN
F 3 "" H 5400 2400 50  0001 C CNN
	1    5400 2400
	1    0    0    -1  
$EndComp
$Comp
L Jumper:SolderJumper_2_Open DIO5
U 1 1 5DBC6685
P 4200 2400
F 0 "DIO5" H 4400 2450 50  0000 C CNN
F 1 "SolderJumper_2_Open" H 5500 2400 50  0001 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm" H 4200 2400 50  0001 C CNN
F 3 "~" H 4200 2400 50  0001 C CNN
	1    4200 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 2200 5200 2200
$Comp
L Jumper:SolderJumper_2_Open DIO2
U 1 1 5DBD387E
P 4200 2700
F 0 "DIO2" H 4400 2750 50  0000 C CNN
F 1 "SolderJumper_2_Open" H 5500 2700 50  0001 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm" H 4200 2700 50  0001 C CNN
F 3 "~" H 4200 2700 50  0001 C CNN
	1    4200 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 2800 3950 2800
Wire Wire Line
	4050 2700 3950 2700
Wire Wire Line
	4050 2400 3950 2400
Wire Wire Line
	4350 2800 4500 2800
Text GLabel 4500 2700 2    50   Output ~ 0
RFM95_DIO2
Text GLabel 4500 2400 2    50   Output ~ 0
RFM95_DIO5
Wire Wire Line
	4350 2400 4500 2400
Wire Wire Line
	4350 2700 4500 2700
$Comp
L Jumper:SolderJumper_2_Open DIO3
U 1 1 5DBE4978
P 4200 2600
F 0 "DIO3" H 4400 2650 50  0000 C CNN
F 1 "SolderJumper_2_Open" H 5500 2600 50  0001 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm" H 4200 2600 50  0001 C CNN
F 3 "~" H 4200 2600 50  0001 C CNN
	1    4200 2600
	1    0    0    -1  
$EndComp
Text GLabel 4500 2600 2    50   Output ~ 0
RFM95_DIO3
Wire Wire Line
	4050 2600 3950 2600
Wire Wire Line
	4350 2600 4500 2600
Wire Wire Line
	3950 2900 4500 2900
Text GLabel 8450 2500 2    50   Input ~ 0
RFM95_DIO3
Text GLabel 8450 2700 2    50   Input ~ 0
RFM95_DIO2
Text GLabel 8450 2600 2    50   Input ~ 0
RFM95_DIO1
Wire Wire Line
	6600 2300 6400 2300
Connection ~ 6400 2300
$Comp
L LoRaModule-rescue:DC-DC_Buck_Step_Module-dc-dc_module M1
U 1 1 5DC23DCA
P 2900 4550
F 0 "M1" H 2900 4865 50  0000 C CNN
F 1 "DC-DC_Buck_Step_Module" H 2900 4774 50  0000 C CNN
F 2 "library:DC-DC_Buck_Step_Module" H 3100 4150 50  0001 C CNN
F 3 "https://es.aliexpress.com/item/32742116421.html?spm=a2g0s.9042311.0.0.292863c06eUDtc" H 3100 4150 50  0001 C CNN
	1    2900 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 4500 2100 4500
Wire Wire Line
	3500 4500 3650 4500
Wire Wire Line
	2900 5050 2900 5150
$Comp
L power:+12V #PWR01
U 1 1 5DC2AD25
P 2100 4500
F 0 "#PWR01" H 2100 4350 50  0001 C CNN
F 1 "+12V" V 2115 4628 50  0000 L CNN
F 2 "" H 2100 4500 50  0001 C CNN
F 3 "" H 2100 4500 50  0001 C CNN
	1    2100 4500
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR05
U 1 1 5DC2B68E
P 3650 4500
F 0 "#PWR05" H 3650 4350 50  0001 C CNN
F 1 "+3.3V" V 3665 4628 50  0000 L CNN
F 2 "" H 3650 4500 50  0001 C CNN
F 3 "" H 3650 4500 50  0001 C CNN
	1    3650 4500
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5DC2FBA4
P 2900 5150
F 0 "#PWR02" H 2900 4900 50  0001 C CNN
F 1 "GND" H 2905 4977 50  0000 C CNN
F 2 "" H 2900 5150 50  0001 C CNN
F 3 "" H 2900 5150 50  0001 C CNN
	1    2900 5150
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J3
U 1 1 5DC3238B
P 1500 5050
F 0 "J3" H 1418 4725 50  0000 C CNN
F 1 "Screw_Terminal_01x02" H 1418 4816 50  0000 C CNN
F 2 "TerminalBlock_4Ucon:TerminalBlock_4Ucon_1x02_P3.50mm_Horizontal" H 1500 5050 50  0001 C CNN
F 3 "~" H 1500 5050 50  0001 C CNN
	1    1500 5050
	-1   0    0    1   
$EndComp
Wire Wire Line
	1700 4950 2050 4950
Wire Wire Line
	1700 5050 2050 5050
Wire Wire Line
	2050 5050 2050 5150
Wire Wire Line
	2050 4950 2050 4850
$Comp
L power:GND #PWR0101
U 1 1 5DC41414
P 2050 5150
F 0 "#PWR0101" H 2050 4900 50  0001 C CNN
F 1 "GND" H 2055 4977 50  0000 C CNN
F 2 "" H 2050 5150 50  0001 C CNN
F 3 "" H 2050 5150 50  0001 C CNN
	1    2050 5150
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0102
U 1 1 5DC41A80
P 2050 4850
F 0 "#PWR0102" H 2050 4700 50  0001 C CNN
F 1 "+12V" H 2065 5023 50  0000 C CNN
F 2 "" H 2050 4850 50  0001 C CNN
F 3 "" H 2050 4850 50  0001 C CNN
	1    2050 4850
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x03 J4
U 1 1 5DC5DFD8
P 9400 3850
F 0 "J4" H 9480 3892 50  0000 L CNN
F 1 "Screw_Terminal_01x03" H 9480 3801 50  0000 L CNN
F 2 "TerminalBlock_4Ucon:TerminalBlock_4Ucon_1x03_P3.50mm_Horizontal" H 9400 3850 50  0001 C CNN
F 3 "~" H 9400 3850 50  0001 C CNN
	1    9400 3850
	-1   0    0    1   
$EndComp
Wire Wire Line
	9600 3950 9850 3950
Wire Wire Line
	9600 3750 9850 3750
Wire Wire Line
	9850 3750 9850 3650
Wire Wire Line
	8200 2200 8450 2200
Text GLabel 8450 2200 2    50   BiDi ~ 0
SENSOR_DATA
Text GLabel 10250 3850 2    50   BiDi ~ 0
SENSOR_DATA
Text Notes 6200 5150 2    50   ~ 0
Añadir botón de reset y configuration mode
Wire Wire Line
	7500 4100 7500 4350
Wire Wire Line
	7600 4100 7600 4350
Text GLabel 7500 4350 3    50   Output ~ 0
I2C_SCL
Text GLabel 7600 4350 3    50   BiDi ~ 0
I2C_SDA
Text Notes 5750 5300 2    50   ~ 0
Falta pull-up en bus de sensor
Text Notes 5750 5450 2    50   ~ 0
¿pull-up en chip select de spi?
Text Notes 6050 5550 2    50   ~ 0
Alimentación arduino , es necesario más de 3.3v por raw
Wire Wire Line
	10250 3850 10100 3850
Connection ~ 10100 3850
Wire Wire Line
	10100 3850 9600 3850
$Comp
L power:+3.3V #PWR0105
U 1 1 5DC435A8
P 10100 3450
F 0 "#PWR0105" H 10100 3300 50  0001 C CNN
F 1 "+3.3V" H 10115 3623 50  0000 C CNN
F 2 "" H 10100 3450 50  0001 C CNN
F 3 "" H 10100 3450 50  0001 C CNN
	1    10100 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	10100 3550 10100 3450
$Comp
L Device:R R1
U 1 1 5DC42ED8
P 10100 3700
F 0 "R1" H 10170 3746 50  0000 L CNN
F 1 "4.7k" H 10170 3655 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 10030 3700 50  0001 C CNN
F 3 "~" H 10100 3700 50  0001 C CNN
	1    10100 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 4250 9850 3950
$Comp
L Connector:Screw_Terminal_01x03 J6
U 1 1 5DC62F44
P 9400 5050
F 0 "J6" H 9480 5092 50  0000 L CNN
F 1 "Screw_Terminal_01x03" H 9480 5001 50  0000 L CNN
F 2 "TerminalBlock_4Ucon:TerminalBlock_4Ucon_1x03_P3.50mm_Horizontal" H 9400 5050 50  0001 C CNN
F 3 "~" H 9400 5050 50  0001 C CNN
	1    9400 5050
	-1   0    0    1   
$EndComp
Wire Wire Line
	9600 5150 9850 5150
Wire Wire Line
	9600 4950 9850 4950
Wire Wire Line
	9850 4950 9850 4850
Text GLabel 10250 5050 2    50   BiDi ~ 0
SENSOR_AUX
Wire Wire Line
	10250 5050 10100 5050
Connection ~ 10100 5050
Wire Wire Line
	10100 5050 9600 5050
$Comp
L power:+3.3V #PWR0108
U 1 1 5DC62F5D
P 10100 4650
F 0 "#PWR0108" H 10100 4500 50  0001 C CNN
F 1 "+3.3V" H 10115 4823 50  0000 C CNN
F 2 "" H 10100 4650 50  0001 C CNN
F 3 "" H 10100 4650 50  0001 C CNN
	1    10100 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	10100 4750 10100 4650
$Comp
L Device:R R2
U 1 1 5DC62F64
P 10100 4900
F 0 "R2" H 10170 4946 50  0000 L CNN
F 1 "4.7k" H 10170 4855 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 10030 4900 50  0001 C CNN
F 3 "~" H 10100 4900 50  0001 C CNN
	1    10100 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 5450 9850 5150
Wire Wire Line
	8200 2300 8450 2300
Text GLabel 8450 2300 2    50   BiDi ~ 0
SENSOR_AUX
$Comp
L power:+3.3V #PWR0103
U 1 1 5DCA6333
P 9850 3650
F 0 "#PWR0103" H 9850 3500 50  0001 C CNN
F 1 "+3.3V" H 9865 3823 50  0000 C CNN
F 2 "" H 9850 3650 50  0001 C CNN
F 3 "" H 9850 3650 50  0001 C CNN
	1    9850 3650
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0104
U 1 1 5DCA684D
P 9850 4850
F 0 "#PWR0104" H 9850 4700 50  0001 C CNN
F 1 "+3.3V" H 9865 5023 50  0000 C CNN
F 2 "" H 9850 4850 50  0001 C CNN
F 3 "" H 9850 4850 50  0001 C CNN
	1    9850 4850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 5DCA81AF
P 9850 4250
F 0 "#PWR0106" H 9850 4000 50  0001 C CNN
F 1 "GND" V 9855 4122 50  0000 R CNN
F 2 "" H 9850 4250 50  0001 C CNN
F 3 "" H 9850 4250 50  0001 C CNN
	1    9850 4250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 5DCA9E30
P 9850 5450
F 0 "#PWR0107" H 9850 5200 50  0001 C CNN
F 1 "GND" V 9855 5322 50  0000 R CNN
F 2 "" H 9850 5450 50  0001 C CNN
F 3 "" H 9850 5450 50  0001 C CNN
	1    9850 5450
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Flag +3V3
U 1 1 5DCB3D94
P 8250 5850
F 0 "+3V3" H 8510 5898 50  0000 L CNN
F 1 "+3v3" H 8510 5853 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 8450 5850 50  0001 C CNN
F 3 "~" H 8450 5850 50  0001 C CNN
	1    8250 5850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5DCB8CEC
P 4800 6050
F 0 "R4" H 4870 6096 50  0000 L CNN
F 1 "4.7k" H 4870 6005 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4730 6050 50  0001 C CNN
F 3 "~" H 4800 6050 50  0001 C CNN
	1    4800 6050
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0109
U 1 1 5DCBB9A8
P 4800 5750
F 0 "#PWR0109" H 4800 5600 50  0001 C CNN
F 1 "+3.3V" H 4815 5923 50  0000 C CNN
F 2 "" H 4800 5750 50  0001 C CNN
F 3 "" H 4800 5750 50  0001 C CNN
	1    4800 5750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 5DCBE487
P 5600 6600
F 0 "#PWR0110" H 5600 6350 50  0001 C CNN
F 1 "GND" V 5605 6472 50  0000 R CNN
F 2 "" H 5600 6600 50  0001 C CNN
F 3 "" H 5600 6600 50  0001 C CNN
	1    5600 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 5750 4800 5900
Wire Wire Line
	4800 6200 4800 6400
Wire Wire Line
	4800 6400 4900 6400
Wire Wire Line
	5500 6400 5600 6400
Wire Wire Line
	5600 6400 5600 6600
$Comp
L Switch:SW_DIP_x01 SW1
U 1 1 5DCB6802
P 5200 6400
F 0 "SW1" H 5200 6667 50  0000 C CNN
F 1 "SW_DIP_x01" H 5200 6576 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_Push_1P1T_NO_CK_KSC6xxJ" H 5200 6400 50  0001 C CNN
F 3 "~" H 5200 6400 50  0001 C CNN
	1    5200 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 6400 4650 6400
Connection ~ 4800 6400
Text GLabel 4650 6400 0    50   Input ~ 0
SWITCH_CFG
Text GLabel 8450 3500 2    50   Input ~ 0
SWITCH_CFG
$Comp
L Connector:TestPoint_Flag SCL1
U 1 1 5DD2A2D1
P 8250 6000
F 0 "SCL1" H 8510 6048 50  0000 L CNN
F 1 "SCL" H 8510 6003 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 8450 6000 50  0001 C CNN
F 3 "~" H 8450 6000 50  0001 C CNN
	1    8250 6000
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Flag SDA1
U 1 1 5DD2CBA5
P 8250 6150
F 0 "SDA1" H 8510 6198 50  0000 L CNN
F 1 "SDA" H 8510 6153 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 8450 6150 50  0001 C CNN
F 3 "~" H 8450 6150 50  0001 C CNN
	1    8250 6150
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Flag GND1
U 1 1 5DD2F4CE
P 8250 6300
F 0 "GND1" H 8510 6348 50  0000 L CNN
F 1 "GND" H 8510 6303 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 8450 6300 50  0001 C CNN
F 3 "~" H 8450 6300 50  0001 C CNN
	1    8250 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 5850 8000 5850
Wire Wire Line
	8250 6000 8000 6000
Wire Wire Line
	8250 6150 8000 6150
Wire Wire Line
	8250 6300 8000 6300
$Comp
L power:+3.3V #PWR0111
U 1 1 5DD3E2E6
P 8000 5850
F 0 "#PWR0111" H 8000 5700 50  0001 C CNN
F 1 "+3.3V" V 8015 5978 50  0000 L CNN
F 2 "" H 8000 5850 50  0001 C CNN
F 3 "" H 8000 5850 50  0001 C CNN
	1    8000 5850
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5DD3EBFE
P 8000 6300
F 0 "#PWR0112" H 8000 6050 50  0001 C CNN
F 1 "GND" V 8005 6172 50  0000 R CNN
F 2 "" H 8000 6300 50  0001 C CNN
F 3 "" H 8000 6300 50  0001 C CNN
	1    8000 6300
	0    1    1    0   
$EndComp
Text GLabel 8000 6000 0    50   BiDi ~ 0
I2C_SCL
Text GLabel 8000 6150 0    50   BiDi ~ 0
I2C_SDA
$Comp
L Device:LED D1
U 1 1 5DD5469F
P 10150 1800
F 0 "D1" V 10189 1683 50  0000 R CNN
F 1 "LED" V 10098 1683 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 10150 1800 50  0001 C CNN
F 3 "~" H 10150 1800 50  0001 C CNN
	1    10150 1800
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R3
U 1 1 5DD592B1
P 10150 2250
F 0 "R3" H 10220 2296 50  0000 L CNN
F 1 "2.2kR" H 10220 2205 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 10080 2250 50  0001 C CNN
F 3 "~" H 10150 2250 50  0001 C CNN
	1    10150 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	10150 1950 10150 2100
Wire Wire Line
	10150 1650 10150 1500
Wire Wire Line
	10150 2400 10150 2600
$Comp
L power:+3V3 #PWR0113
U 1 1 5DD64DA8
P 10150 1500
F 0 "#PWR0113" H 10150 1350 50  0001 C CNN
F 1 "+3V3" H 10165 1673 50  0000 C CNN
F 2 "" H 10150 1500 50  0001 C CNN
F 3 "" H 10150 1500 50  0001 C CNN
	1    10150 1500
	1    0    0    -1  
$EndComp
Text GLabel 8450 3400 2    50   Output ~ 0
LED
Text GLabel 10150 2600 3    50   Input ~ 0
LED
Wire Wire Line
	8200 3600 8450 3600
Wire Wire Line
	8200 3700 8450 3700
Wire Wire Line
	7400 4100 7400 4350
Wire Wire Line
	7300 4100 7300 4350
Text GLabel 8450 3600 2    50   BiDi ~ 0
A2
Text GLabel 8450 3700 2    50   BiDi ~ 0
A3
Text GLabel 7400 4350 3    50   BiDi ~ 0
A6
Text GLabel 7300 4350 3    50   BiDi ~ 0
A7
$Comp
L Connector:TestPoint_Flag A2
U 1 1 5DD7D960
P 6850 5900
F 0 "A2" H 7110 5948 50  0000 L CNN
F 1 "+3v3" H 7110 5903 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 7050 5900 50  0001 C CNN
F 3 "~" H 7050 5900 50  0001 C CNN
	1    6850 5900
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Flag A3
U 1 1 5DD7D966
P 6850 6050
F 0 "A3" H 7110 6098 50  0000 L CNN
F 1 "SCL" H 7110 6053 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 7050 6050 50  0001 C CNN
F 3 "~" H 7050 6050 50  0001 C CNN
	1    6850 6050
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Flag A6
U 1 1 5DD7D96C
P 6850 6200
F 0 "A6" H 7110 6248 50  0000 L CNN
F 1 "SDA" H 7110 6203 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 7050 6200 50  0001 C CNN
F 3 "~" H 7050 6200 50  0001 C CNN
	1    6850 6200
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint_Flag A7
U 1 1 5DD7D972
P 6850 6350
F 0 "A7" H 7110 6398 50  0000 L CNN
F 1 "GND" H 7110 6353 50  0001 L CNN
F 2 "TestPoint:TestPoint_Pad_1.5x1.5mm" H 7050 6350 50  0001 C CNN
F 3 "~" H 7050 6350 50  0001 C CNN
	1    6850 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 5900 6600 5900
Wire Wire Line
	6850 6050 6600 6050
Wire Wire Line
	6850 6200 6600 6200
Wire Wire Line
	6850 6350 6600 6350
Text GLabel 6600 5900 0    50   BiDi ~ 0
A2
Text GLabel 6600 6050 0    50   BiDi ~ 0
A3
Text GLabel 6600 6200 0    50   BiDi ~ 0
A6
Text GLabel 6600 6350 0    50   BiDi ~ 0
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
	8200 3400 8450 3400
Wire Wire Line
	8200 3500 8450 3500
Wire Wire Line
	8200 2700 8450 2700
Wire Wire Line
	8450 2600 8200 2600
Wire Wire Line
	8200 2500 8450 2500
Wire Wire Line
	8200 2400 8450 2400
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
$EndSCHEMATC