EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:Weather
LIBS:Weather Balloon-cache
EELAYER 25 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 1 4
Title "HAB Control Unit"
Date "2016-09-26"
Rev "v0.6"
Comp "Robert Susmilch"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 12350 500  9450 6900
U 57DC68E1
F0 "Heater" 250
F1 "Heater.sch" 250
$EndSheet
Text GLabel 1750 5600 0    50   Output ~ 0
Main_Heat
Text GLabel 1750 5800 0    50   Output ~ 0
GPS_Heat
Text GLabel 1750 5700 0    50   Output ~ 0
UV_Heat
Text GLabel 4550 5600 2    50   Output ~ 0
TSL2561_Heat
Text GLabel 1750 5500 0    50   Output ~ 0
Custom_Heat
$Comp
L ATMEGA2560-A IC2
U 1 1 57DCCF9A
P 3150 4200
F 0 "IC2" H 2600 6400 50  0000 C CNN
F 1 "ATMEGA2560-A" H 2800 6300 50  0000 C CNN
F 2 "Housings_QFP:TQFP-100_14x14mm_Pitch0.5mm" H 2700 6200 50  0001 C CIN
F 3 "" H 3150 4200 50  0000 C CNN
	1    3150 4200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 57DCDDF7
P 3100 7200
F 0 "#PWR01" H 3100 6950 50  0001 C CNN
F 1 "GND" H 3105 7027 50  0000 C CNN
F 2 "" H 3100 7200 50  0000 C CNN
F 3 "" H 3100 7200 50  0000 C CNN
	1    3100 7200
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 57DCF331
P 3550 700
F 0 "C1" H 3665 746 50  0000 L CNN
F 1 "22uF" H 3665 655 50  0000 L CNN
F 2 "Capacitors_Tantalum_SMD:TantalC_SizeC_EIA-6032_HandSoldering" H 3588 550 50  0001 C CNN
F 3 "" H 3550 700 50  0000 C CNN
	1    3550 700 
	0    1    1    0   
$EndComp
$Comp
L C C3
U 1 1 57DCF337
P 3550 1100
F 0 "C3" H 3665 1146 50  0000 L CNN
F 1 "10pF" H 3665 1055 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 3588 950 50  0001 C CNN
F 3 "" H 3550 1100 50  0000 C CNN
	1    3550 1100
	0    1    1    0   
$EndComp
$Comp
L GND #PWR02
U 1 1 57DCF344
P 3800 900
F 0 "#PWR02" H 3800 650 50  0001 C CNN
F 1 "GND" H 3805 727 50  0000 C CNN
F 2 "" H 3800 900 50  0000 C CNN
F 3 "" H 3800 900 50  0000 C CNN
	1    3800 900 
	0    -1   -1   0   
$EndComp
Text GLabel 1650 6800 0    60   Input ~ 0
AREF
$Comp
L Crystal Y1
U 1 1 57DCF599
P 1550 2050
F 0 "Y1" V 1596 1919 50  0000 R CNN
F 1 "16Mhz" V 1500 1900 50  0000 R CNN
F 2 "Personal:HC-49V" H 1550 2050 50  0001 C CNN
F 3 "" H 1550 2050 50  0000 C CNN
	1    1550 2050
	0    -1   -1   0   
$EndComp
$Comp
L C_Small C7
U 1 1 57DCF6E8
P 1400 2200
F 0 "C7" V 1300 2300 50  0000 C CNN
F 1 "22pF" V 1200 2250 50  0000 C CNN
F 2 "Capacitors_ThroughHole:C_Disc_D3_P2.5" H 1400 2200 50  0001 C CNN
F 3 "" H 1400 2200 50  0000 C CNN
	1    1400 2200
	0    -1   -1   0   
$EndComp
$Comp
L C_Small C6
U 1 1 57DCF87D
P 1400 1900
F 0 "C6" V 1600 2000 50  0000 C CNN
F 1 "22pF" V 1500 1950 50  0000 C CNN
F 2 "Capacitors_ThroughHole:C_Disc_D3_P2.5" H 1400 1900 50  0001 C CNN
F 3 "" H 1400 1900 50  0000 C CNN
	1    1400 1900
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR03
U 1 1 57DCFA65
P 1200 2050
F 0 "#PWR03" H 1200 1800 50  0001 C CNN
F 1 "GND" H 1205 1877 50  0000 C CNN
F 2 "" H 1200 2050 50  0000 C CNN
F 3 "" H 1200 2050 50  0000 C CNN
	1    1200 2050
	0    1    1    0   
$EndComp
$Comp
L R R1
U 1 1 57E79EE4
P 3100 700
F 0 "R1" V 3000 700 50  0000 C CNN
F 1 "100" V 3100 700 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 3030 700 50  0001 C CNN
F 3 "" H 3100 700 50  0000 C CNN
	1    3100 700 
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR04
U 1 1 57E7BF3D
P 2900 700
F 0 "#PWR04" H 2900 550 50  0001 C CNN
F 1 "+5V" H 2915 873 50  0000 C CNN
F 2 "" H 2900 700 50  0000 C CNN
F 3 "" H 2900 700 50  0000 C CNN
	1    2900 700 
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X02 P4
U 1 1 57E7C6AE
P 1550 1550
F 0 "P4" H 1550 1400 50  0000 C CNN
F 1 "2560_RESET" H 1850 1550 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 1550 1550 50  0001 C CNN
F 3 "" H 1550 1550 50  0000 C CNN
	1    1550 1550
	-1   0    0    1   
$EndComp
$Comp
L R R2
U 1 1 57E7C8B8
P 1550 900
F 0 "R2" V 1450 900 50  0000 C CNN
F 1 "10k" V 1550 900 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 1480 900 50  0001 C CNN
F 3 "" H 1550 900 50  0000 C CNN
	1    1550 900 
	0    1    1    0   
$EndComp
$Comp
L C C2
U 1 1 57E7CA4F
P 1950 900
F 0 "C2" V 1800 850 50  0000 L CNN
F 1 "0.1uF" V 2100 800 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 1988 750 50  0001 C CNN
F 3 "" H 1950 900 50  0000 C CNN
	1    1950 900 
	0    1    1    0   
$EndComp
$Comp
L GND #PWR05
U 1 1 57E7CF28
P 2150 1000
F 0 "#PWR05" H 2150 750 50  0001 C CNN
F 1 "GND" H 2155 827 50  0000 C CNN
F 2 "" H 2150 1000 50  0000 C CNN
F 3 "" H 2150 1000 50  0000 C CNN
	1    2150 1000
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR06
U 1 1 57E7CFC8
P 1350 850
F 0 "#PWR06" H 1350 700 50  0001 C CNN
F 1 "+5V" H 1365 1023 50  0000 C CNN
F 2 "" H 1350 850 50  0000 C CNN
F 3 "" H 1350 850 50  0000 C CNN
	1    1350 850 
	1    0    0    -1  
$EndComp
Text GLabel 1650 1150 0    60   Input ~ 0
2560_RESET
$Comp
L BILEVEL_SHIFT U2
U 1 1 57E7E9E3
P 6600 4650
F 0 "U2" H 6600 5287 60  0000 C CNN
F 1 "BILEVEL_SHIFT" H 6600 5181 60  0000 C CNN
F 2 "Personal:DIP-12_W7.62mm_LongPads_ALL" H 6750 4500 60  0001 C CNN
F 3 "" H 6750 4500 60  0001 C CNN
	1    6600 4650
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR07
U 1 1 57E7EB01
P 6000 3800
F 0 "#PWR07" H 6000 3650 50  0001 C CNN
F 1 "+5V" V 6015 3928 50  0000 L CNN
F 2 "" H 6000 3800 50  0000 C CNN
F 3 "" H 6000 3800 50  0000 C CNN
	1    6000 3800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 57E7ECEB
P 7300 4750
F 0 "#PWR08" H 7300 4500 50  0001 C CNN
F 1 "GND" V 7305 4622 50  0000 R CNN
F 2 "" H 7300 4750 50  0000 C CNN
F 3 "" H 7300 4750 50  0000 C CNN
	1    7300 4750
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR09
U 1 1 57E7EE18
P 5600 4750
F 0 "#PWR09" H 5600 4500 50  0001 C CNN
F 1 "GND" V 5605 4622 50  0000 R CNN
F 2 "" H 5600 4750 50  0000 C CNN
F 3 "" H 5600 4750 50  0000 C CNN
	1    5600 4750
	0    1    1    0   
$EndComp
$Comp
L CONN_01X02 P10
U 1 1 57E7EFD2
P 4700 5250
F 0 "P10" H 4777 5291 50  0000 L CNN
F 1 "SERIAL" H 4777 5200 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 4700 5250 50  0001 C CNN
F 3 "" H 4700 5250 50  0000 C CNN
	1    4700 5250
	1    0    0    -1  
$EndComp
Text GLabel 8050 4300 2    60   Output ~ 0
SCL_3.3
Text GLabel 8050 4450 2    60   BiDi ~ 0
SDA_3.3
Text GLabel 7350 4900 2    60   Input ~ 0
2560_RX_GPS_TX
Text GLabel 7350 5050 2    60   Output ~ 0
2560_TX_GPS_RX
Text GLabel 4900 3750 1    50   Output ~ 0
SCL_5
Text GLabel 5000 3750 1    50   BiDi ~ 0
SDA_5
$Comp
L CONN_01X02 P5
U 1 1 57E803B7
P 5050 2250
F 0 "P5" H 5100 2400 50  0000 R CNN
F 1 "2560_SCK" H 5100 2100 50  0000 R CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 5050 2250 50  0001 C CNN
F 3 "" H 5050 2250 50  0000 C CNN
	1    5050 2250
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X02 P6
U 1 1 57E804C0
P 5550 2250
F 0 "P6" H 5600 2400 50  0000 R CNN
F 1 "2560_MOSI" H 5650 2100 50  0000 R CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 5550 2250 50  0001 C CNN
F 3 "" H 5550 2250 50  0000 C CNN
	1    5550 2250
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X02 P7
U 1 1 57E805B9
P 6050 2250
F 0 "P7" H 6100 2400 50  0000 R CNN
F 1 "2560_MISO" H 6150 2100 50  0000 R CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 6050 2250 50  0001 C CNN
F 3 "" H 6050 2250 50  0000 C CNN
	1    6050 2250
	0    -1   -1   0   
$EndComp
Text GLabel 7250 2800 2    50   Input ~ 0
MISO
Text GLabel 7250 2700 2    50   Output ~ 0
MOSI
Text GLabel 7250 2600 2    50   Output ~ 0
SCK
Text GLabel 4550 5500 2    50   Output ~ 0
SHT31_Heat
$Sheet
S 3850 -3950 4900 3600
U 57E85DBE
F0 "Aux" 100
F1 "Aux.sch" 100
$EndSheet
Text GLabel 4550 6100 2    60   Input ~ 0
UV_VOLTAGE
Text GLabel 1750 2500 0    50   Output ~ 0
UV_POWER
Text GLabel 2750 1250 0    60   Input ~ 0
Vcc_1
Text GLabel 2750 1100 0    60   Input ~ 0
Vcc_2
Text GLabel 2750 950  0    60   Input ~ 0
Vcc_3
$Comp
L ATTINY85-P IC1
U 1 1 57E8934A
P 8400 1350
F 0 "IC1" H 8400 1957 50  0000 C CNN
F 1 "ATTINY85-P" H 8400 1866 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 8400 1775 50  0000 C CIN
F 3 "" H 8400 1350 50  0000 C CNN
	1    8400 1350
	1    0    0    -1  
$EndComp
Text GLabel 4750 1800 2    50   Output ~ 0
PIEZO_PWR
Text GLabel 4750 1900 2    50   Output ~ 0
PIEZO
Text GLabel 4750 2000 2    50   Output ~ 0
LANDING_LED
Text GLabel 4750 1600 2    50   Output ~ 0
DS18B20_PWR
Text GLabel 4750 1700 2    50   BiDi ~ 0
DS18B20_DATA
$Comp
L C C4
U 1 1 57E8BF07
P 9950 1350
F 0 "C4" H 10065 1396 50  0000 L CNN
F 1 "22uF" H 10065 1305 50  0000 L CNN
F 2 "Capacitors_Tantalum_SMD:TantalC_SizeC_EIA-6032_HandSoldering" H 9988 1200 50  0001 C CNN
F 3 "" H 9950 1350 50  0000 C CNN
	1    9950 1350
	-1   0    0    1   
$EndComp
$Comp
L C C5
U 1 1 57E8D76E
P 10300 1350
F 0 "C5" H 10415 1396 50  0000 L CNN
F 1 "10pF" H 10415 1305 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 10338 1200 50  0001 C CNN
F 3 "" H 10300 1350 50  0000 C CNN
	1    10300 1350
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR010
U 1 1 57E8DECC
P 9950 1650
F 0 "#PWR010" H 9950 1400 50  0001 C CNN
F 1 "GND" H 9955 1477 50  0000 C CNN
F 2 "" H 9950 1650 50  0000 C CNN
F 3 "" H 9950 1650 50  0000 C CNN
	1    9950 1650
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR011
U 1 1 57E8E314
P 9950 950
F 0 "#PWR011" H 9950 800 50  0001 C CNN
F 1 "+5V" H 9965 1123 50  0000 C CNN
F 2 "" H 9950 950 50  0000 C CNN
F 3 "" H 9950 950 50  0000 C CNN
	1    9950 950 
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 57E90FED
P 8000 2200
F 0 "R3" H 8070 2246 50  0000 L CNN
F 1 "10k" H 8070 2155 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 7930 2200 50  0001 C CNN
F 3 "" H 8000 2200 50  0000 C CNN
	1    8000 2200
	1    0    0    -1  
$EndComp
$Comp
L C C8
U 1 1 57E910E1
P 8000 2600
F 0 "C8" H 8115 2646 50  0000 L CNN
F 1 "0.1uF" H 8115 2555 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 8038 2450 50  0001 C CNN
F 3 "" H 8000 2600 50  0000 C CNN
	1    8000 2600
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR012
U 1 1 57E9141B
P 8000 2000
F 0 "#PWR012" H 8000 1850 50  0001 C CNN
F 1 "+5V" H 8015 2173 50  0000 C CNN
F 2 "" H 8000 2000 50  0000 C CNN
F 3 "" H 8000 2000 50  0000 C CNN
	1    8000 2000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 57E91548
P 8000 2800
F 0 "#PWR013" H 8000 2550 50  0001 C CNN
F 1 "GND" H 8005 2627 50  0000 C CNN
F 2 "" H 8000 2800 50  0000 C CNN
F 3 "" H 8000 2800 50  0000 C CNN
	1    8000 2800
	1    0    0    -1  
$EndComp
Text GLabel 6900 1400 0    50   Output ~ 0
2560_RESET
Text GLabel 6900 1500 0    50   Input ~ 0
2560_HEARTBEAT
Text GLabel 4550 2900 2    50   Output ~ 0
2560_HEARTBEAT
$Comp
L R R9
U 1 1 57E928EF
P 5350 6100
F 0 "R9" V 5250 6000 50  0000 L CNN
F 1 "10k" V 5350 6050 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 5280 6100 50  0001 C CNN
F 3 "" H 5350 6100 50  0000 C CNN
	1    5350 6100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR014
U 1 1 57E92A30
P 5350 6300
F 0 "#PWR014" H 5350 6050 50  0001 C CNN
F 1 "GND" H 5355 6127 50  0000 C CNN
F 2 "" H 5350 6300 50  0000 C CNN
F 3 "" H 5350 6300 50  0000 C CNN
	1    5350 6300
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR015
U 1 1 57E92DFD
P 5850 5800
F 0 "#PWR015" H 5850 5650 50  0001 C CNN
F 1 "+5V" H 5750 5950 50  0000 L CNN
F 2 "" H 5850 5800 50  0000 C CNN
F 3 "" H 5850 5800 50  0000 C CNN
	1    5850 5800
	1    0    0    -1  
$EndComp
Text GLabel 1750 2600 0    50   Output ~ 0
SHT31_RESET
Text GLabel 1750 2700 0    50   Output ~ 0
DOF_POWER
Text GLabel 1750 2800 0    50   Output ~ 0
MS5607_POWER
Text GLabel 1750 2900 0    50   Output ~ 0
TSL2561_POWER
Text GLabel 1750 3000 0    50   Output ~ 0
SD_POWER
Text GLabel 1750 3100 0    50   Output ~ 0
GPS_POWER
$Comp
L R R5
U 1 1 57E96F25
P 6000 2900
F 0 "R5" V 5950 3050 50  0000 C CNN
F 1 "150" V 6000 2900 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 5930 2900 50  0001 C CNN
F 3 "" H 6000 2900 50  0000 C CNN
	1    6000 2900
	0    1    1    0   
$EndComp
$Comp
L +5V #PWR016
U 1 1 57E9700C
P 6350 2900
F 0 "#PWR016" H 6350 2750 50  0001 C CNN
F 1 "+5V" V 6365 3073 50  0000 C CNN
F 2 "" H 6350 2900 50  0000 C CNN
F 3 "" H 6350 2900 50  0000 C CNN
	1    6350 2900
	0    1    1    0   
$EndComp
$Comp
L R R7
U 1 1 57E980CF
P 6000 3050
F 0 "R7" V 5950 3200 50  0000 C CNN
F 1 "150" V 6000 3050 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 5930 3050 50  0001 C CNN
F 3 "" H 6000 3050 50  0000 C CNN
	1    6000 3050
	0    1    1    0   
$EndComp
$Comp
L R R8
U 1 1 57E98FFD
P 6000 3200
F 0 "R8" V 5950 3350 50  0000 C CNN
F 1 "150" V 6000 3200 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 5930 3200 50  0001 C CNN
F 3 "" H 6000 3200 50  0000 C CNN
	1    6000 3200
	0    1    1    0   
$EndComp
$Comp
L Led_Small D1
U 1 1 57E9930D
P 5600 2900
F 0 "D1" H 5500 2950 50  0000 C CNN
F 1 "Green" H 5750 2950 50  0000 C CNN
F 2 "LEDs:LED_1206" V 5600 2900 50  0001 C CNN
F 3 "" V 5600 2900 50  0000 C CNN
	1    5600 2900
	1    0    0    -1  
$EndComp
$Comp
L Led_Small D2
U 1 1 57E99533
P 5600 3050
F 0 "D2" H 5500 3100 50  0000 C CNN
F 1 "Yellow" H 5750 3100 50  0000 C CNN
F 2 "LEDs:LED_1206" V 5600 3050 50  0001 C CNN
F 3 "" V 5600 3050 50  0000 C CNN
	1    5600 3050
	1    0    0    -1  
$EndComp
$Comp
L Led_Small D3
U 1 1 57E99627
P 5600 3200
F 0 "D3" H 5500 3250 50  0000 C CNN
F 1 "Red" H 5750 3250 50  0000 C CNN
F 2 "LEDs:LED_1206" V 5600 3200 50  0001 C CNN
F 3 "" V 5600 3200 50  0000 C CNN
	1    5600 3200
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH_SMALL_H SW1
U 1 1 57E9C671
P 5650 5900
F 0 "SW1" H 5650 5750 50  0000 C CNN
F 1 "SILENCE" H 5650 5850 50  0000 C CNN
F 2 "Buttons_Switches_ThroughHole:SW_PUSH-12mm" H 5650 6100 50  0001 C CNN
F 3 "" H 5650 6100 50  0000 C CNN
	1    5650 5900
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 57E9EB4E
P 5700 4100
F 0 "C10" H 5815 4146 50  0000 L CNN
F 1 "10pF" H 5815 4055 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 5738 3950 50  0001 C CNN
F 3 "" H 5700 4100 50  0000 C CNN
	1    5700 4100
	1    0    0    -1  
$EndComp
$Comp
L C C11
U 1 1 57E9F8D2
P 7250 4100
F 0 "C11" H 7365 4146 50  0000 L CNN
F 1 "10pF" H 7365 4055 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 7288 3950 50  0001 C CNN
F 3 "" H 7250 4100 50  0000 C CNN
	1    7250 4100
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR017
U 1 1 57E7EF03
P 7100 3800
F 0 "#PWR017" H 7100 3650 50  0001 C CNN
F 1 "+3.3V" V 7115 3928 50  0000 L CNN
F 2 "" H 7100 3800 50  0000 C CNN
F 3 "" H 7100 3800 50  0000 C CNN
	1    7100 3800
	1    0    0    -1  
$EndComp
Text GLabel 1750 3200 0    50   Output ~ 0
GEIGER_POWER
$Comp
L PWR_FLAG #FLG018
U 1 1 57EB331A
P 3250 1100
F 0 "#FLG018" H 3250 1195 50  0001 C CNN
F 1 "PWR_FLAG" H 3050 1300 50  0000 L CNN
F 2 "" H 3250 1100 50  0000 C CNN
F 3 "" H 3250 1100 50  0000 C CNN
	1    3250 1100
	0    -1   -1   0   
$EndComp
NoConn ~ 1850 6600
NoConn ~ 1850 4900
NoConn ~ 1850 4800
NoConn ~ 1850 4700
NoConn ~ 1850 4600
NoConn ~ 4450 2200
NoConn ~ 4450 2300
NoConn ~ 4450 5000
NoConn ~ 4450 4900
NoConn ~ 4450 4800
NoConn ~ 4450 4700
NoConn ~ 4450 5400
NoConn ~ 4450 5800
NoConn ~ 4450 5700
NoConn ~ 4450 6800
NoConn ~ 4450 6700
NoConn ~ 4450 6600
NoConn ~ 4450 6500
NoConn ~ 4450 6400
NoConn ~ 4450 6300
NoConn ~ 4450 6200
Text GLabel 7250 2500 2    50   Output ~ 0
SS
$Comp
L +5V #PWR019
U 1 1 57EC3307
P 9150 4000
F 0 "#PWR019" H 9150 3850 50  0001 C CNN
F 1 "+5V" H 9165 4173 50  0000 C CNN
F 2 "" H 9150 4000 50  0000 C CNN
F 3 "" H 9150 4000 50  0000 C CNN
	1    9150 4000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR020
U 1 1 57EC340D
P 10050 4500
F 0 "#PWR020" H 10050 4250 50  0001 C CNN
F 1 "GND" H 10055 4327 50  0000 C CNN
F 2 "" H 10050 4500 50  0000 C CNN
F 3 "" H 10050 4500 50  0000 C CNN
	1    10050 4500
	1    0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 57EC3E32
P 10050 4250
F 0 "C9" H 10165 4296 50  0000 L CNN
F 1 "0.1uF" H 10165 4205 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 10088 4100 50  0001 C CNN
F 3 "" H 10050 4250 50  0000 C CNN
	1    10050 4250
	-1   0    0    -1  
$EndComp
Text GLabel 8950 4500 0    60   Output ~ 0
AREF
$Sheet
S 9650 -3950 3300 3600
U 57EC94A0
F0 "AT25_EEPROM" 50
F1 "AT25_EEPROM.sch" 50
$EndSheet
Text GLabel 1750 6100 0    50   Output ~ 0
AT25_CS_1
Text GLabel 1750 6200 0    50   Output ~ 0
AT25_CS_2
Text GLabel 1750 6300 0    50   Output ~ 0
AT25_CS_3
Text GLabel 1750 6400 0    50   Output ~ 0
AT25_CS_4
Text GLabel 1750 6500 0    50   Output ~ 0
AT25_CS_5
Text GLabel 1750 5900 0    50   Output ~ 0
AT25_CS_6
Text GLabel 1750 5400 0    50   Output ~ 0
AT25_CS_7
Text GLabel 1750 5300 0    50   Output ~ 0
AT25_CS_8
Text GLabel 1750 5200 0    50   Output ~ 0
AT25_CS_9
Text GLabel 1750 5000 0    50   Output ~ 0
AT25_CS_10
$Comp
L MAX6107EUR U1
U 1 1 57EC3289
P 9600 4450
F 0 "U1" H 9450 4600 50  0000 C CNN
F 1 "MAX6107EUR" H 9600 4300 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 9575 4200 50  0001 C CNN
F 3 "" H 9575 4200 50  0001 C CNN
	1    9600 4450
	1    0    0    -1  
$EndComp
Text GLabel 1700 4300 0    50   Input ~ 0
GEIGER_X
Text GLabel 1700 4400 0    50   Input ~ 0
GEIGER_Y
Text GLabel 1700 4500 0    50   Input ~ 0
GEIGER_Z
$Comp
L R R4
U 1 1 57EDBF8D
P 6350 2300
F 0 "R4" V 6450 2250 50  0000 L CNN
F 1 "4.7k" V 6350 2200 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 6280 2300 50  0001 C CNN
F 3 "" H 6350 2300 50  0000 C CNN
	1    6350 2300
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR021
U 1 1 57EDC8DE
P 6550 2050
F 0 "#PWR021" H 6550 1900 50  0001 C CNN
F 1 "+5V" H 6565 2223 50  0000 C CNN
F 2 "" H 6550 2050 50  0000 C CNN
F 3 "" H 6550 2050 50  0000 C CNN
	1    6550 2050
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 57EDD700
P 6550 2300
F 0 "R6" V 6650 2250 50  0000 L CNN
F 1 "4.7k" V 6550 2200 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 6480 2300 50  0001 C CNN
F 3 "" H 6550 2300 50  0000 C CNN
	1    6550 2300
	1    0    0    -1  
$EndComp
$Comp
L R R43
U 1 1 57EDD8AA
P 6750 2300
F 0 "R43" V 6850 2250 50  0000 L CNN
F 1 "4.7k" V 6750 2200 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 6680 2300 50  0001 C CNN
F 3 "" H 6750 2300 50  0000 C CNN
	1    6750 2300
	1    0    0    -1  
$EndComp
$Comp
L R R46
U 1 1 57EDE349
P 7750 4050
F 0 "R46" V 7850 4000 50  0000 L CNN
F 1 "4.7k" V 7750 3950 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 7680 4050 50  0001 C CNN
F 3 "" H 7750 4050 50  0000 C CNN
	1    7750 4050
	1    0    0    -1  
$EndComp
$Comp
L R R47
U 1 1 57EDE35F
P 7950 4050
F 0 "R47" V 8050 4000 50  0000 L CNN
F 1 "4.7k" V 7950 3950 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 7880 4050 50  0001 C CNN
F 3 "" H 7950 4050 50  0000 C CNN
	1    7950 4050
	1    0    0    -1  
$EndComp
$Comp
L R R44
U 1 1 57EDF187
P 5200 4050
F 0 "R44" V 5300 4000 50  0000 L CNN
F 1 "4.7k" V 5200 3950 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 5130 4050 50  0001 C CNN
F 3 "" H 5200 4050 50  0000 C CNN
	1    5200 4050
	1    0    0    -1  
$EndComp
$Comp
L R R45
U 1 1 57EDF251
P 5400 4050
F 0 "R45" V 5500 4000 50  0000 L CNN
F 1 "4.7k" V 5400 3950 50  0000 L CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 5330 4050 50  0001 C CNN
F 3 "" H 5400 4050 50  0000 C CNN
	1    5400 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 7100 3000 7150
Wire Wire Line
	3000 7150 3300 7150
Wire Wire Line
	3300 7150 3300 7100
Wire Wire Line
	3200 7100 3200 7150
Connection ~ 3200 7150
Wire Wire Line
	3100 7100 3100 7200
Connection ~ 3100 7150
Wire Wire Line
	1650 6800 1850 6800
Wire Wire Line
	1500 1900 1850 1900
Wire Wire Line
	1500 2200 1850 2200
Connection ~ 1550 1900
Connection ~ 1550 2200
Wire Wire Line
	1250 2200 1300 2200
Wire Wire Line
	1250 1900 1250 2200
Wire Wire Line
	1250 1900 1300 1900
Wire Wire Line
	1200 2050 1250 2050
Connection ~ 1250 2050
Wire Wire Line
	1750 1600 1850 1600
Wire Wire Line
	1700 900  1800 900 
Wire Wire Line
	2150 1000 2150 900 
Wire Wire Line
	2150 900  2100 900 
Wire Wire Line
	1350 850  1350 900 
Wire Wire Line
	1350 900  1400 900 
Wire Wire Line
	1750 900  1750 1500
Connection ~ 1750 900 
Wire Wire Line
	1650 1150 1750 1150
Connection ~ 1750 1150
Wire Wire Line
	6150 4450 5350 4450
Wire Wire Line
	5350 4450 5350 4400
Wire Wire Line
	7050 4750 7300 4750
Wire Wire Line
	5600 4750 6150 4750
Wire Wire Line
	4500 5200 4450 5200
Wire Wire Line
	4500 5300 4450 5300
Wire Wire Line
	4450 4500 5050 4500
Wire Wire Line
	5050 4500 5050 4900
Wire Wire Line
	5050 4900 6150 4900
Wire Wire Line
	4450 4600 4950 4600
Wire Wire Line
	4950 4600 4950 5050
Wire Wire Line
	4950 5050 6150 5050
Wire Wire Line
	7350 4900 7050 4900
Wire Wire Line
	7350 5050 7050 5050
Connection ~ 5000 4400
Wire Wire Line
	4900 3750 4900 4300
Connection ~ 4900 4300
Wire Wire Line
	5000 2600 5000 2450
Wire Wire Line
	5500 2450 5500 2700
Wire Wire Line
	6000 2450 6000 2800
Wire Wire Line
	6000 2800 4450 2800
Wire Wire Line
	6100 2450 6100 2450
Wire Wire Line
	5600 2450 5600 2450
Wire Wire Line
	5100 2450 5100 2450
Wire Wire Line
	1700 4300 1850 4300
Wire Wire Line
	1700 4500 1850 4500
Wire Wire Line
	3300 700  3300 1300
Wire Wire Line
	3250 1100 3400 1100
Wire Wire Line
	3250 700  3400 700 
Wire Wire Line
	3750 1100 3700 1100
Wire Wire Line
	3750 700  3750 1100
Wire Wire Line
	3750 700  3700 700 
Wire Wire Line
	3800 900  3750 900 
Connection ~ 3750 900 
Wire Wire Line
	2950 700  2900 700 
Connection ~ 3300 1100
Connection ~ 3300 700 
Wire Wire Line
	4450 6100 4550 6100
Wire Wire Line
	2800 1300 2800 1250
Wire Wire Line
	2800 1250 2750 1250
Wire Wire Line
	2900 1300 2900 1100
Wire Wire Line
	2900 1100 2750 1100
Wire Wire Line
	3000 1300 3000 950 
Wire Wire Line
	3000 950  2750 950 
Wire Wire Line
	9750 1600 10300 1600
Wire Wire Line
	9950 1500 9950 1650
Connection ~ 9950 1600
Wire Wire Line
	10300 1600 10300 1500
Wire Wire Line
	9750 1100 10300 1100
Wire Wire Line
	10300 1100 10300 1200
Wire Wire Line
	9950 950  9950 1200
Connection ~ 9950 1100
Wire Wire Line
	7000 1600 7050 1600
Wire Wire Line
	7050 1100 6700 1100
Wire Wire Line
	7050 1200 6700 1200
Wire Wire Line
	7050 1300 6700 1300
Wire Wire Line
	8000 2350 8000 2450
Connection ~ 8000 2400
Wire Wire Line
	8000 2000 8000 2050
Wire Wire Line
	8000 2800 8000 2750
Wire Wire Line
	6900 1400 7050 1400
Wire Wire Line
	4450 5900 5500 5900
Wire Wire Line
	5350 5900 5350 5950
Wire Wire Line
	5350 6250 5350 6300
Connection ~ 5350 5900
Wire Wire Line
	5850 5800 5850 5900
Wire Wire Line
	5800 5900 6100 5900
Wire Wire Line
	1750 2500 1850 2500
Wire Wire Line
	1750 2600 1850 2600
Wire Wire Line
	1750 3100 1850 3100
Wire Wire Line
	1750 3000 1850 3000
Wire Wire Line
	1750 2900 1850 2900
Wire Wire Line
	1750 2800 1850 2800
Wire Wire Line
	1750 2700 1850 2700
Wire Wire Line
	4750 1600 4450 1600
Wire Wire Line
	4450 1700 4750 1700
Wire Wire Line
	4750 1800 4450 1800
Wire Wire Line
	4450 1900 4750 1900
Wire Wire Line
	4750 2000 4450 2000
Wire Wire Line
	6900 1500 7050 1500
Wire Wire Line
	1750 5600 1850 5600
Wire Wire Line
	1750 5700 1850 5700
Wire Wire Line
	1750 5800 1850 5800
Wire Wire Line
	1750 5500 1850 5500
Wire Wire Line
	5500 3200 4450 3200
Wire Wire Line
	5500 3050 5400 3050
Wire Wire Line
	5400 3050 5400 3100
Wire Wire Line
	5400 3100 4450 3100
Wire Wire Line
	5500 2900 5350 2900
Wire Wire Line
	5350 2900 5350 3000
Wire Wire Line
	5350 3000 4450 3000
Wire Wire Line
	5700 3050 5850 3050
Wire Wire Line
	5700 3200 5850 3200
Wire Wire Line
	5700 2900 5850 2900
Wire Wire Line
	6250 3200 6150 3200
Wire Wire Line
	6150 3050 6250 3050
Connection ~ 6250 3050
Wire Wire Line
	6150 2900 6350 2900
Connection ~ 6250 2900
Wire Wire Line
	4550 5600 4450 5600
Wire Wire Line
	4550 5500 4450 5500
Wire Wire Line
	6000 4600 6150 4600
Wire Wire Line
	6000 3800 6000 4600
Wire Wire Line
	5700 3850 5700 3950
Wire Wire Line
	5200 3850 6000 3850
Connection ~ 6000 3850
Wire Wire Line
	5700 4250 5700 4750
Connection ~ 5700 4750
Wire Wire Line
	7100 3800 7100 4600
Wire Wire Line
	7100 4600 7050 4600
Wire Wire Line
	7250 4250 7250 4750
Connection ~ 7250 4750
Wire Wire Line
	7250 3950 7250 3850
Wire Wire Line
	7100 3850 7950 3850
Connection ~ 7100 3850
Wire Wire Line
	4900 2500 7250 2500
Wire Wire Line
	10050 4400 10050 4500
Wire Wire Line
	10050 4450 10000 4450
Wire Wire Line
	9150 4000 9150 4400
Wire Wire Line
	9150 4400 9200 4400
Wire Wire Line
	10050 4100 10050 4050
Wire Wire Line
	10050 4050 9150 4050
Connection ~ 9150 4050
Connection ~ 10050 4450
Wire Wire Line
	8950 4500 9200 4500
Wire Wire Line
	1850 5400 1750 5400
Wire Wire Line
	1750 5300 1850 5300
Wire Wire Line
	1750 5200 1850 5200
Wire Wire Line
	1750 5000 1850 5000
Wire Wire Line
	1750 5900 1850 5900
Wire Wire Line
	1750 6100 1850 6100
Wire Wire Line
	1750 6200 1850 6200
Wire Wire Line
	1750 6300 1850 6300
Wire Wire Line
	1750 6400 1850 6400
Wire Wire Line
	1750 6500 1850 6500
Wire Wire Line
	1850 3200 1750 3200
Wire Wire Line
	1700 4400 1850 4400
Wire Wire Line
	6100 2800 7250 2800
Wire Wire Line
	6100 2800 6100 2450
Wire Wire Line
	5600 2450 5600 2700
Wire Wire Line
	5600 2700 7250 2700
Wire Wire Line
	5100 2600 7250 2600
Wire Wire Line
	5100 2600 5100 2450
Wire Wire Line
	6350 2150 6350 2100
Wire Wire Line
	6350 2450 6350 2800
Connection ~ 6350 2800
Wire Wire Line
	6350 2100 6750 2100
Wire Wire Line
	6750 2100 6750 2150
Wire Wire Line
	6550 2050 6550 2150
Connection ~ 6550 2100
Wire Wire Line
	6550 2450 6550 2700
Connection ~ 6550 2700
Wire Wire Line
	7950 3850 7950 3900
Connection ~ 7250 3850
Wire Wire Line
	7750 3900 7750 3850
Connection ~ 7750 3850
Wire Wire Line
	7750 4200 7750 4450
Connection ~ 7750 4450
Wire Wire Line
	7950 4200 7950 4300
Connection ~ 7950 4300
Wire Wire Line
	5000 3750 5000 4400
Wire Wire Line
	5200 3900 5200 3850
Connection ~ 5700 3850
Wire Wire Line
	5400 3900 5400 3850
Connection ~ 5400 3850
Wire Wire Line
	5200 4200 5200 4400
Connection ~ 5200 4400
Wire Wire Line
	5400 4300 5400 4200
Connection ~ 5400 4300
Wire Wire Line
	6250 2900 6250 3200
Text Label 4450 3500 0    50   ~ 0
RADIO_1
Text Label 4450 3600 0    50   ~ 0
RADIO_2
Text Label 4450 3700 0    50   ~ 0
RADIO_3
Text Label 4450 3800 0    50   ~ 0
RADIO_4
Text Label 4450 3900 0    50   ~ 0
RADIO_5
Text Label 4450 4000 0    50   ~ 0
RADIO_6
Text Label 4450 4100 0    50   ~ 0
RADIO_7
Text Label 4450 3400 0    50   ~ 0
RADIO_0
$Comp
L CONN_01X10 P22
U 1 1 57E90246
P 9600 2950
F 0 "P22" H 9678 2991 50  0000 L CNN
F 1 "RADIO" H 9678 2900 50  0000 L CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x10" H 9600 2950 50  0001 C CNN
F 3 "" H 9600 2950 50  0000 C CNN
	1    9600 2950
	1    0    0    -1  
$EndComp
Text Label 8800 2600 2    50   ~ 0
RADIO_0
Text Label 8800 2700 2    50   ~ 0
RADIO_1
Text Label 8800 2800 2    50   ~ 0
RADIO_2
Text Label 8800 2900 2    50   ~ 0
RADIO_3
Text Label 8800 3000 2    50   ~ 0
RADIO_4
Text Label 8800 3100 2    50   ~ 0
RADIO_5
Text Label 8800 3200 2    50   ~ 0
RADIO_6
Text Label 8800 3300 2    50   ~ 0
RADIO_7
$Comp
L GND #PWR022
U 1 1 57E908EF
P 9350 3450
F 0 "#PWR022" H 9350 3200 50  0001 C CNN
F 1 "GND" H 9355 3277 50  0000 C CNN
F 2 "" H 9350 3450 50  0000 C CNN
F 3 "" H 9350 3450 50  0000 C CNN
	1    9350 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9350 3450 9350 3400
Wire Wire Line
	9350 3400 9400 3400
$Comp
L +5V #PWR023
U 1 1 57E90D72
P 9350 2450
F 0 "#PWR023" H 9350 2300 50  0001 C CNN
F 1 "+5V" H 9365 2623 50  0000 C CNN
F 2 "" H 9350 2450 50  0000 C CNN
F 3 "" H 9350 2450 50  0000 C CNN
	1    9350 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	9350 2450 9350 2500
Wire Wire Line
	9350 2500 9400 2500
Wire Wire Line
	6750 2500 6750 2450
Connection ~ 6750 2500
$Comp
L PWR_FLAG #FLG024
U 1 1 57EA2C82
P 6100 5800
F 0 "#FLG024" H 6100 5895 50  0001 C CNN
F 1 "PWR_FLAG" H 6400 5900 50  0000 C CNN
F 2 "" H 6100 5800 50  0000 C CNN
F 3 "" H 6100 5800 50  0000 C CNN
	1    6100 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 5900 6100 5800
Connection ~ 5850 5900
$Comp
L R R53
U 1 1 57EA783F
P 4750 2500
F 0 "R53" V 4700 2300 50  0000 C CNN
F 1 "22" V 4750 2500 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 4680 2500 50  0001 C CNN
F 3 "" H 4750 2500 50  0000 C CNN
	1    4750 2500
	0    1    1    0   
$EndComp
$Comp
L R R54
U 1 1 57EA7D87
P 4750 2600
F 0 "R54" V 4700 2400 50  0000 C CNN
F 1 "22" V 4750 2600 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 4680 2600 50  0001 C CNN
F 3 "" H 4750 2600 50  0000 C CNN
	1    4750 2600
	0    1    1    0   
$EndComp
$Comp
L R R56
U 1 1 57EA7E21
P 4750 2700
F 0 "R56" V 4700 2500 50  0000 C CNN
F 1 "22" V 4750 2700 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 4680 2700 50  0001 C CNN
F 3 "" H 4750 2700 50  0000 C CNN
	1    4750 2700
	0    1    1    0   
$EndComp
Wire Wire Line
	4600 2600 4450 2600
Wire Wire Line
	4600 2500 4450 2500
Wire Wire Line
	4600 2700 4450 2700
Wire Wire Line
	4900 2600 5000 2600
Wire Wire Line
	5500 2700 4900 2700
$Comp
L R R64
U 1 1 57EA873D
P 4650 4300
F 0 "R64" V 4700 4500 50  0000 C CNN
F 1 "22" V 4650 4300 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 4580 4300 50  0001 C CNN
F 3 "" H 4650 4300 50  0000 C CNN
	1    4650 4300
	0    1    1    0   
$EndComp
$Comp
L R R66
U 1 1 57EA8913
P 4650 4400
F 0 "R66" V 4700 4600 50  0000 C CNN
F 1 "22" V 4650 4400 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 4580 4400 50  0001 C CNN
F 3 "" H 4650 4400 50  0000 C CNN
	1    4650 4400
	0    1    1    0   
$EndComp
Wire Wire Line
	4450 4400 4500 4400
Wire Wire Line
	4450 4300 4500 4300
Wire Wire Line
	5350 4400 4800 4400
Wire Wire Line
	4800 4300 6150 4300
$Comp
L R R65
U 1 1 57EA9048
P 7550 4300
F 0 "R65" V 7500 4100 50  0000 C CNN
F 1 "22" V 7550 4300 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 7480 4300 50  0001 C CNN
F 3 "" H 7550 4300 50  0000 C CNN
	1    7550 4300
	0    1    1    0   
$EndComp
$Comp
L R R67
U 1 1 57EA9138
P 7550 4450
F 0 "R67" V 7500 4250 50  0000 C CNN
F 1 "22" V 7550 4450 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 7480 4450 50  0001 C CNN
F 3 "" H 7550 4450 50  0000 C CNN
	1    7550 4450
	0    1    1    0   
$EndComp
Wire Wire Line
	7400 4450 7050 4450
Wire Wire Line
	7050 4300 7400 4300
Wire Wire Line
	7700 4300 8050 4300
Wire Wire Line
	7700 4450 8050 4450
$Comp
L R R55
U 1 1 57EA9C7A
P 9200 2600
F 0 "R55" V 9150 2400 50  0000 C CNN
F 1 "22" V 9200 2600 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 9130 2600 50  0001 C CNN
F 3 "" H 9200 2600 50  0000 C CNN
	1    9200 2600
	0    1    1    0   
$EndComp
$Comp
L R R57
U 1 1 57EA9DDC
P 9200 2700
F 0 "R57" V 9150 2500 50  0000 C CNN
F 1 "22" V 9200 2700 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 9130 2700 50  0001 C CNN
F 3 "" H 9200 2700 50  0000 C CNN
	1    9200 2700
	0    1    1    0   
$EndComp
$Comp
L R R58
U 1 1 57EA9E8B
P 9200 2800
F 0 "R58" V 9150 2600 50  0000 C CNN
F 1 "22" V 9200 2800 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 9130 2800 50  0001 C CNN
F 3 "" H 9200 2800 50  0000 C CNN
	1    9200 2800
	0    1    1    0   
$EndComp
$Comp
L R R59
U 1 1 57EA9F3D
P 9200 2900
F 0 "R59" V 9150 2700 50  0000 C CNN
F 1 "22" V 9200 2900 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 9130 2900 50  0001 C CNN
F 3 "" H 9200 2900 50  0000 C CNN
	1    9200 2900
	0    1    1    0   
$EndComp
$Comp
L R R60
U 1 1 57EA9FF2
P 9200 3000
F 0 "R60" V 9150 2800 50  0000 C CNN
F 1 "22" V 9200 3000 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 9130 3000 50  0001 C CNN
F 3 "" H 9200 3000 50  0000 C CNN
	1    9200 3000
	0    1    1    0   
$EndComp
$Comp
L R R61
U 1 1 57EAA0AA
P 9200 3100
F 0 "R61" V 9150 2900 50  0000 C CNN
F 1 "22" V 9200 3100 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 9130 3100 50  0001 C CNN
F 3 "" H 9200 3100 50  0000 C CNN
	1    9200 3100
	0    1    1    0   
$EndComp
$Comp
L R R62
U 1 1 57EAA165
P 9200 3200
F 0 "R62" V 9150 3000 50  0000 C CNN
F 1 "22" V 9200 3200 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 9130 3200 50  0001 C CNN
F 3 "" H 9200 3200 50  0000 C CNN
	1    9200 3200
	0    1    1    0   
$EndComp
$Comp
L R R63
U 1 1 57EAA223
P 9200 3300
F 0 "R63" V 9150 3100 50  0000 C CNN
F 1 "22" V 9200 3300 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 9130 3300 50  0001 C CNN
F 3 "" H 9200 3300 50  0000 C CNN
	1    9200 3300
	0    1    1    0   
$EndComp
Wire Wire Line
	9350 3200 9400 3200
Wire Wire Line
	9400 3300 9350 3300
Wire Wire Line
	9050 3300 8800 3300
Wire Wire Line
	8800 3200 9050 3200
Wire Wire Line
	8800 3100 9050 3100
Wire Wire Line
	8800 3000 9050 3000
Wire Wire Line
	8800 2900 9050 2900
Wire Wire Line
	8800 2800 9050 2800
Wire Wire Line
	8800 2700 9050 2700
Wire Wire Line
	8800 2600 9050 2600
Wire Wire Line
	9350 2600 9400 2600
Wire Wire Line
	9400 2700 9350 2700
Wire Wire Line
	9350 2800 9400 2800
Wire Wire Line
	9400 2900 9350 2900
Wire Wire Line
	9350 3000 9400 3000
Wire Wire Line
	9400 3100 9350 3100
Wire Wire Line
	7000 2150 7000 1600
Wire Wire Line
	7850 2150 7000 2150
Wire Wire Line
	8000 2400 7800 2400
Wire Wire Line
	7850 2150 7850 2300
Wire Wire Line
	7850 2300 7800 2300
$Comp
L CONN_01X02 P8
U 1 1 57E8EC26
P 7600 2350
F 0 "P8" V 7550 2150 50  0000 C CNN
F 1 "85_RESET" V 7700 2300 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 7600 2350 50  0001 C CNN
F 3 "" H 7600 2350 50  0000 C CNN
	1    7600 2350
	-1   0    0    -1  
$EndComp
$Comp
L CONN_01X05 P1
U 1 1 57EB4E99
P 6500 1100
F 0 "P1" H 6419 675 50  0000 C CNN
F 1 "85_PRGM" H 6419 766 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05" H 6500 1100 50  0001 C CNN
F 3 "" H 6500 1100 50  0000 C CNN
	1    6500 1100
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR025
U 1 1 57EB51E6
P 6800 1000
F 0 "#PWR025" H 6800 750 50  0001 C CNN
F 1 "GND" V 6805 872 50  0000 R CNN
F 2 "" H 6800 1000 50  0000 C CNN
F 3 "" H 6800 1000 50  0000 C CNN
	1    6800 1000
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR026
U 1 1 57EB5317
P 6800 900
F 0 "#PWR026" H 6800 750 50  0001 C CNN
F 1 "+5V" V 6815 1028 50  0000 L CNN
F 2 "" H 6800 900 50  0000 C CNN
F 3 "" H 6800 900 50  0000 C CNN
	1    6800 900 
	0    1    1    0   
$EndComp
Wire Wire Line
	6800 900  6700 900 
Wire Wire Line
	6800 1000 6700 1000
$Comp
L CONN_01X10 P2
U 1 1 57EC1197
P 750 3750
F 0 "P2" H 669 3075 50  0000 C CNN
F 1 "ADC_AUX" H 669 3166 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x10" H 750 3750 50  0001 C CNN
F 3 "" H 750 3750 50  0000 C CNN
	1    750  3750
	-1   0    0    1   
$EndComp
Wire Wire Line
	1850 4100 950  4100
Wire Wire Line
	1850 4000 950  4000
Wire Wire Line
	950  3900 1850 3900
Wire Wire Line
	1850 3800 950  3800
Wire Wire Line
	950  3700 1850 3700
Wire Wire Line
	1850 3600 950  3600
Wire Wire Line
	950  3500 1850 3500
Wire Wire Line
	1850 3400 950  3400
$Comp
L GND #PWR027
U 1 1 57EC1FB5
P 1000 4250
F 0 "#PWR027" H 1000 4000 50  0001 C CNN
F 1 "GND" H 1005 4077 50  0000 C CNN
F 2 "" H 1000 4250 50  0000 C CNN
F 3 "" H 1000 4250 50  0000 C CNN
	1    1000 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 4250 1000 4200
Wire Wire Line
	1000 4200 950  4200
$Comp
L +5V #PWR028
U 1 1 57EC22D1
P 1000 3300
F 0 "#PWR028" H 1000 3150 50  0001 C CNN
F 1 "+5V" V 1015 3428 50  0000 L CNN
F 2 "" H 1000 3300 50  0000 C CNN
F 3 "" H 1000 3300 50  0000 C CNN
	1    1000 3300
	0    1    1    0   
$EndComp
Wire Wire Line
	1000 3300 950  3300
Wire Wire Line
	4550 2900 4450 2900
NoConn ~ 4450 2100
$EndSCHEMATC
