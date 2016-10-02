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
Sheet 4 4
Title "Serial EEPROM Backup"
Date "2016-09-26"
Rev "v0.6"
Comp "Robert Susmilch"
Comment1 "To be used as backup to SD card for critical data."
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L AT25_EEPROM U6
U 1 1 57EC95EB
P 1550 1250
F 0 "U6" H 1300 1500 50  0000 C CNN
F 1 "AT25_EEPROM" H 1200 1000 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 1200 1200 50  0001 C CNN
F 3 "" H 1200 1200 50  0000 C CNN
	1    1550 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 1150 1100 1150
Wire Wire Line
	1100 900  1100 1250
Wire Wire Line
	1550 850  1550 950 
$Comp
L +5V #PWR088
U 1 1 57EC95F6
P 1550 850
F 0 "#PWR088" H 1550 700 50  0001 C CNN
F 1 "+5V" H 1565 1023 50  0000 C CNN
F 2 "" H 1550 850 50  0000 C CNN
F 3 "" H 1550 850 50  0000 C CNN
	1    1550 850 
	1    0    0    -1  
$EndComp
Connection ~ 1550 900 
$Comp
L GND #PWR089
U 1 1 57EC95FD
P 1550 1650
F 0 "#PWR089" H 1550 1400 50  0001 C CNN
F 1 "GND" H 1555 1477 50  0000 C CNN
F 2 "" H 1550 1650 50  0000 C CNN
F 3 "" H 1550 1650 50  0000 C CNN
	1    1550 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 1650 1550 1550
Text GLabel 2000 1150 2    50   Input ~ 0
SCK
Wire Wire Line
	2000 1150 1950 1150
Text GLabel 2000 1250 2    50   Input ~ 0
MOSI
Text GLabel 2000 1350 2    50   Output ~ 0
MISO
Wire Wire Line
	2000 1350 1950 1350
Wire Wire Line
	2000 1250 1950 1250
Wire Wire Line
	1100 1250 1150 1250
Connection ~ 1100 1150
Text GLabel 1100 1350 0    50   Input ~ 0
AT25_CS_1
Wire Wire Line
	1100 1350 1150 1350
$Comp
L AT25_EEPROM U8
U 1 1 57EC968A
P 1550 2650
F 0 "U8" H 1300 2900 50  0000 C CNN
F 1 "AT25_EEPROM" H 1200 2400 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 1200 2600 50  0001 C CNN
F 3 "" H 1200 2600 50  0000 C CNN
	1    1550 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 2550 1100 2550
Wire Wire Line
	1100 2300 1100 2650
Wire Wire Line
	1100 2300 1550 2300
Wire Wire Line
	1550 2250 1550 2350
$Comp
L +5V #PWR090
U 1 1 57EC9695
P 1550 2250
F 0 "#PWR090" H 1550 2100 50  0001 C CNN
F 1 "+5V" H 1565 2423 50  0000 C CNN
F 2 "" H 1550 2250 50  0000 C CNN
F 3 "" H 1550 2250 50  0000 C CNN
	1    1550 2250
	1    0    0    -1  
$EndComp
Connection ~ 1550 2300
$Comp
L GND #PWR091
U 1 1 57EC969C
P 1550 3050
F 0 "#PWR091" H 1550 2800 50  0001 C CNN
F 1 "GND" H 1555 2877 50  0000 C CNN
F 2 "" H 1550 3050 50  0000 C CNN
F 3 "" H 1550 3050 50  0000 C CNN
	1    1550 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 3050 1550 2950
Text GLabel 2000 2550 2    50   Input ~ 0
SCK
Wire Wire Line
	2000 2550 1950 2550
Text GLabel 2000 2650 2    50   Input ~ 0
MOSI
Text GLabel 2000 2750 2    50   Output ~ 0
MISO
Wire Wire Line
	2000 2750 1950 2750
Wire Wire Line
	2000 2650 1950 2650
Wire Wire Line
	1100 2650 1150 2650
Connection ~ 1100 2550
Text GLabel 1100 2750 0    50   Input ~ 0
AT25_CS_2
Wire Wire Line
	1100 2750 1150 2750
$Comp
L AT25_EEPROM U10
U 1 1 57EC974B
P 1550 4100
F 0 "U10" H 1300 4350 50  0000 C CNN
F 1 "AT25_EEPROM" H 1200 3850 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 1200 4050 50  0001 C CNN
F 3 "" H 1200 4050 50  0000 C CNN
	1    1550 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 4000 1100 4000
Wire Wire Line
	1100 3750 1100 4100
Wire Wire Line
	1100 3750 1550 3750
Wire Wire Line
	1550 3700 1550 3800
$Comp
L +5V #PWR092
U 1 1 57EC9756
P 1550 3700
F 0 "#PWR092" H 1550 3550 50  0001 C CNN
F 1 "+5V" H 1565 3873 50  0000 C CNN
F 2 "" H 1550 3700 50  0000 C CNN
F 3 "" H 1550 3700 50  0000 C CNN
	1    1550 3700
	1    0    0    -1  
$EndComp
Connection ~ 1550 3750
$Comp
L GND #PWR093
U 1 1 57EC975D
P 1550 4500
F 0 "#PWR093" H 1550 4250 50  0001 C CNN
F 1 "GND" H 1555 4327 50  0000 C CNN
F 2 "" H 1550 4500 50  0000 C CNN
F 3 "" H 1550 4500 50  0000 C CNN
	1    1550 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 4500 1550 4400
Text GLabel 2000 4000 2    50   Input ~ 0
SCK
Wire Wire Line
	2000 4000 1950 4000
Text GLabel 2000 4100 2    50   Input ~ 0
MOSI
Text GLabel 2000 4200 2    50   Output ~ 0
MISO
Wire Wire Line
	2000 4200 1950 4200
Wire Wire Line
	2000 4100 1950 4100
Wire Wire Line
	1100 4100 1150 4100
Connection ~ 1100 4000
Text GLabel 1100 4200 0    50   Input ~ 0
AT25_CS_3
Wire Wire Line
	1100 4200 1150 4200
$Comp
L AT25_EEPROM U12
U 1 1 57EC981E
P 1550 5550
F 0 "U12" H 1300 5800 50  0000 C CNN
F 1 "AT25_EEPROM" H 1200 5300 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 1200 5500 50  0001 C CNN
F 3 "" H 1200 5500 50  0000 C CNN
	1    1550 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 5450 1100 5450
Wire Wire Line
	1100 5200 1100 5550
Wire Wire Line
	1100 5200 1650 5200
Wire Wire Line
	1550 5150 1550 5250
$Comp
L +5V #PWR094
U 1 1 57EC9829
P 1550 5150
F 0 "#PWR094" H 1550 5000 50  0001 C CNN
F 1 "+5V" H 1565 5323 50  0000 C CNN
F 2 "" H 1550 5150 50  0000 C CNN
F 3 "" H 1550 5150 50  0000 C CNN
	1    1550 5150
	1    0    0    -1  
$EndComp
Connection ~ 1550 5200
$Comp
L GND #PWR095
U 1 1 57EC9830
P 1550 5950
F 0 "#PWR095" H 1550 5700 50  0001 C CNN
F 1 "GND" H 1555 5777 50  0000 C CNN
F 2 "" H 1550 5950 50  0000 C CNN
F 3 "" H 1550 5950 50  0000 C CNN
	1    1550 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 5950 1550 5850
Text GLabel 2000 5450 2    50   Input ~ 0
SCK
Wire Wire Line
	2000 5450 1950 5450
Text GLabel 2000 5550 2    50   Input ~ 0
MOSI
Text GLabel 2000 5650 2    50   Output ~ 0
MISO
Wire Wire Line
	2000 5650 1950 5650
Wire Wire Line
	2000 5550 1950 5550
Wire Wire Line
	1100 5550 1150 5550
Connection ~ 1100 5450
Text GLabel 1100 5650 0    50   Input ~ 0
AT25_CS_4
Wire Wire Line
	1100 5650 1150 5650
$Comp
L C C41
U 1 1 57EC9841
P 1800 5200
F 0 "C41" H 1915 5246 50  0000 L CNN
F 1 "4.7uF" H 1915 5155 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 1838 5050 50  0001 C CNN
F 3 "" H 1800 5200 50  0000 C CNN
	1    1800 5200
	0    1    -1   0   
$EndComp
$Comp
L GND #PWR096
U 1 1 57EC9848
P 2000 5200
F 0 "#PWR096" H 2000 4950 50  0001 C CNN
F 1 "GND" V 2005 5072 50  0000 R CNN
F 2 "" H 2000 5200 50  0000 C CNN
F 3 "" H 2000 5200 50  0000 C CNN
	1    2000 5200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2000 5200 1950 5200
$Comp
L AT25_EEPROM U14
U 1 1 57EC992F
P 1550 7000
F 0 "U14" H 1300 7250 50  0000 C CNN
F 1 "AT25_EEPROM" H 1200 6750 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 1200 6950 50  0001 C CNN
F 3 "" H 1200 6950 50  0000 C CNN
	1    1550 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 6900 1100 6900
Wire Wire Line
	1100 6650 1100 7000
Wire Wire Line
	1100 6650 1650 6650
Wire Wire Line
	1550 6600 1550 6700
$Comp
L +5V #PWR097
U 1 1 57EC993A
P 1550 6600
F 0 "#PWR097" H 1550 6450 50  0001 C CNN
F 1 "+5V" H 1565 6773 50  0000 C CNN
F 2 "" H 1550 6600 50  0000 C CNN
F 3 "" H 1550 6600 50  0000 C CNN
	1    1550 6600
	1    0    0    -1  
$EndComp
Connection ~ 1550 6650
$Comp
L GND #PWR098
U 1 1 57EC9941
P 1550 7400
F 0 "#PWR098" H 1550 7150 50  0001 C CNN
F 1 "GND" H 1555 7227 50  0000 C CNN
F 2 "" H 1550 7400 50  0000 C CNN
F 3 "" H 1550 7400 50  0000 C CNN
	1    1550 7400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 7400 1550 7300
Text GLabel 2000 6900 2    50   Input ~ 0
SCK
Wire Wire Line
	2000 6900 1950 6900
Text GLabel 2000 7000 2    50   Input ~ 0
MOSI
Text GLabel 2000 7100 2    50   Output ~ 0
MISO
Wire Wire Line
	2000 7100 1950 7100
Wire Wire Line
	2000 7000 1950 7000
Wire Wire Line
	1100 7000 1150 7000
Connection ~ 1100 6900
Text GLabel 1100 7100 0    50   Input ~ 0
AT25_CS_5
Wire Wire Line
	1100 7100 1150 7100
$Comp
L C C43
U 1 1 57EC9952
P 1800 6650
F 0 "C43" H 1915 6696 50  0000 L CNN
F 1 "4.7uF" H 1915 6605 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 1838 6500 50  0001 C CNN
F 3 "" H 1800 6650 50  0000 C CNN
	1    1800 6650
	0    1    -1   0   
$EndComp
$Comp
L GND #PWR099
U 1 1 57EC9959
P 2000 6650
F 0 "#PWR099" H 2000 6400 50  0001 C CNN
F 1 "GND" V 2005 6522 50  0000 R CNN
F 2 "" H 2000 6650 50  0000 C CNN
F 3 "" H 2000 6650 50  0000 C CNN
	1    2000 6650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2000 6650 1950 6650
$Comp
L AT25_EEPROM U7
U 1 1 57EC9A2A
P 3400 1250
F 0 "U7" H 3150 1500 50  0000 C CNN
F 1 "AT25_EEPROM" H 3050 1000 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 3050 1200 50  0001 C CNN
F 3 "" H 3050 1200 50  0000 C CNN
	1    3400 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 1150 2950 1150
Wire Wire Line
	2950 900  2950 1250
Wire Wire Line
	2950 900  3400 900 
Wire Wire Line
	3400 850  3400 950 
$Comp
L +5V #PWR0100
U 1 1 57EC9A35
P 3400 850
F 0 "#PWR0100" H 3400 700 50  0001 C CNN
F 1 "+5V" H 3415 1023 50  0000 C CNN
F 2 "" H 3400 850 50  0000 C CNN
F 3 "" H 3400 850 50  0000 C CNN
	1    3400 850 
	1    0    0    -1  
$EndComp
Connection ~ 3400 900 
$Comp
L GND #PWR0101
U 1 1 57EC9A3C
P 3400 1650
F 0 "#PWR0101" H 3400 1400 50  0001 C CNN
F 1 "GND" H 3405 1477 50  0000 C CNN
F 2 "" H 3400 1650 50  0000 C CNN
F 3 "" H 3400 1650 50  0000 C CNN
	1    3400 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 1650 3400 1550
Text GLabel 3850 1150 2    50   Input ~ 0
SCK
Wire Wire Line
	3850 1150 3800 1150
Text GLabel 3850 1250 2    50   Input ~ 0
MOSI
Text GLabel 3850 1350 2    50   Output ~ 0
MISO
Wire Wire Line
	3850 1350 3800 1350
Wire Wire Line
	3850 1250 3800 1250
Wire Wire Line
	2950 1250 3000 1250
Connection ~ 2950 1150
Text GLabel 2950 1350 0    50   Input ~ 0
AT25_CS_6
Wire Wire Line
	2950 1350 3000 1350
$Comp
L AT25_EEPROM U9
U 1 1 57EC9B1B
P 3400 2650
F 0 "U9" H 3150 2900 50  0000 C CNN
F 1 "AT25_EEPROM" H 3050 2400 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 3050 2600 50  0001 C CNN
F 3 "" H 3050 2600 50  0000 C CNN
	1    3400 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 2550 2950 2550
Wire Wire Line
	2950 2300 2950 2650
Wire Wire Line
	2950 2300 3400 2300
Wire Wire Line
	3400 2250 3400 2350
$Comp
L +5V #PWR0102
U 1 1 57EC9B26
P 3400 2250
F 0 "#PWR0102" H 3400 2100 50  0001 C CNN
F 1 "+5V" H 3415 2423 50  0000 C CNN
F 2 "" H 3400 2250 50  0000 C CNN
F 3 "" H 3400 2250 50  0000 C CNN
	1    3400 2250
	1    0    0    -1  
$EndComp
Connection ~ 3400 2300
$Comp
L GND #PWR0103
U 1 1 57EC9B2D
P 3400 3050
F 0 "#PWR0103" H 3400 2800 50  0001 C CNN
F 1 "GND" H 3405 2877 50  0000 C CNN
F 2 "" H 3400 3050 50  0000 C CNN
F 3 "" H 3400 3050 50  0000 C CNN
	1    3400 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 3050 3400 2950
Text GLabel 3850 2550 2    50   Input ~ 0
SCK
Wire Wire Line
	3850 2550 3800 2550
Text GLabel 3850 2650 2    50   Input ~ 0
MOSI
Text GLabel 3850 2750 2    50   Output ~ 0
MISO
Wire Wire Line
	3850 2750 3800 2750
Wire Wire Line
	3850 2650 3800 2650
Wire Wire Line
	2950 2650 3000 2650
Connection ~ 2950 2550
Text GLabel 2950 2750 0    50   Input ~ 0
AT25_CS_7
Wire Wire Line
	2950 2750 3000 2750
$Comp
L AT25_EEPROM U11
U 1 1 57EC9C12
P 3400 4100
F 0 "U11" H 3150 4350 50  0000 C CNN
F 1 "AT25_EEPROM" H 3050 3850 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 3050 4050 50  0001 C CNN
F 3 "" H 3050 4050 50  0000 C CNN
	1    3400 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 4000 2950 4000
Wire Wire Line
	2950 3750 2950 4100
Wire Wire Line
	2950 3750 3400 3750
Wire Wire Line
	3400 3700 3400 3800
$Comp
L +5V #PWR0104
U 1 1 57EC9C1D
P 3400 3700
F 0 "#PWR0104" H 3400 3550 50  0001 C CNN
F 1 "+5V" H 3415 3873 50  0000 C CNN
F 2 "" H 3400 3700 50  0000 C CNN
F 3 "" H 3400 3700 50  0000 C CNN
	1    3400 3700
	1    0    0    -1  
$EndComp
Connection ~ 3400 3750
$Comp
L GND #PWR0105
U 1 1 57EC9C24
P 3400 4500
F 0 "#PWR0105" H 3400 4250 50  0001 C CNN
F 1 "GND" H 3405 4327 50  0000 C CNN
F 2 "" H 3400 4500 50  0000 C CNN
F 3 "" H 3400 4500 50  0000 C CNN
	1    3400 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 4500 3400 4400
Text GLabel 3850 4000 2    50   Input ~ 0
SCK
Wire Wire Line
	3850 4000 3800 4000
Text GLabel 3850 4100 2    50   Input ~ 0
MOSI
Text GLabel 3850 4200 2    50   Output ~ 0
MISO
Wire Wire Line
	3850 4200 3800 4200
Wire Wire Line
	3850 4100 3800 4100
Wire Wire Line
	2950 4100 3000 4100
Connection ~ 2950 4000
Text GLabel 2950 4200 0    50   Input ~ 0
AT25_CS_8
Wire Wire Line
	2950 4200 3000 4200
$Comp
L AT25_EEPROM U13
U 1 1 57EC9D27
P 3400 5550
F 0 "U13" H 3150 5800 50  0000 C CNN
F 1 "AT25_EEPROM" H 3050 5300 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 3050 5500 50  0001 C CNN
F 3 "" H 3050 5500 50  0000 C CNN
	1    3400 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 5450 2950 5450
Wire Wire Line
	2950 5200 2950 5550
Wire Wire Line
	2950 5200 3500 5200
Wire Wire Line
	3400 5150 3400 5250
$Comp
L +5V #PWR0106
U 1 1 57EC9D32
P 3400 5150
F 0 "#PWR0106" H 3400 5000 50  0001 C CNN
F 1 "+5V" H 3415 5323 50  0000 C CNN
F 2 "" H 3400 5150 50  0000 C CNN
F 3 "" H 3400 5150 50  0000 C CNN
	1    3400 5150
	1    0    0    -1  
$EndComp
Connection ~ 3400 5200
$Comp
L GND #PWR0107
U 1 1 57EC9D39
P 3400 5950
F 0 "#PWR0107" H 3400 5700 50  0001 C CNN
F 1 "GND" H 3405 5777 50  0000 C CNN
F 2 "" H 3400 5950 50  0000 C CNN
F 3 "" H 3400 5950 50  0000 C CNN
	1    3400 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 5950 3400 5850
Text GLabel 3850 5450 2    50   Input ~ 0
SCK
Wire Wire Line
	3850 5450 3800 5450
Text GLabel 3850 5550 2    50   Input ~ 0
MOSI
Text GLabel 3850 5650 2    50   Output ~ 0
MISO
Wire Wire Line
	3850 5650 3800 5650
Wire Wire Line
	3850 5550 3800 5550
Wire Wire Line
	2950 5550 3000 5550
Connection ~ 2950 5450
Text GLabel 2950 5650 0    50   Input ~ 0
AT25_CS_9
Wire Wire Line
	2950 5650 3000 5650
$Comp
L C C42
U 1 1 57EC9D4A
P 3650 5200
F 0 "C42" H 3765 5246 50  0000 L CNN
F 1 "4.7uF" H 3765 5155 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 3688 5050 50  0001 C CNN
F 3 "" H 3650 5200 50  0000 C CNN
	1    3650 5200
	0    1    -1   0   
$EndComp
$Comp
L GND #PWR0108
U 1 1 57EC9D51
P 3850 5200
F 0 "#PWR0108" H 3850 4950 50  0001 C CNN
F 1 "GND" V 3855 5072 50  0000 R CNN
F 2 "" H 3850 5200 50  0000 C CNN
F 3 "" H 3850 5200 50  0000 C CNN
	1    3850 5200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3850 5200 3800 5200
$Comp
L AT25_EEPROM U15
U 1 1 57EC9E1A
P 3400 7000
F 0 "U15" H 3150 7250 50  0000 C CNN
F 1 "AT25_EEPROM" H 3050 6750 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 3050 6950 50  0001 C CNN
F 3 "" H 3050 6950 50  0000 C CNN
	1    3400 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 6900 2950 6900
Wire Wire Line
	2950 6650 2950 7000
Wire Wire Line
	2950 6650 3400 6650
Wire Wire Line
	3400 6600 3400 6700
$Comp
L +5V #PWR0109
U 1 1 57EC9E25
P 3400 6600
F 0 "#PWR0109" H 3400 6450 50  0001 C CNN
F 1 "+5V" H 3415 6773 50  0000 C CNN
F 2 "" H 3400 6600 50  0000 C CNN
F 3 "" H 3400 6600 50  0000 C CNN
	1    3400 6600
	1    0    0    -1  
$EndComp
Connection ~ 3400 6650
$Comp
L GND #PWR0110
U 1 1 57EC9E2C
P 3400 7400
F 0 "#PWR0110" H 3400 7150 50  0001 C CNN
F 1 "GND" H 3405 7227 50  0000 C CNN
F 2 "" H 3400 7400 50  0000 C CNN
F 3 "" H 3400 7400 50  0000 C CNN
	1    3400 7400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 7400 3400 7300
Text GLabel 3850 6900 2    50   Input ~ 0
SCK
Wire Wire Line
	3850 6900 3800 6900
Text GLabel 3850 7000 2    50   Input ~ 0
MOSI
Text GLabel 3850 7100 2    50   Output ~ 0
MISO
Wire Wire Line
	3850 7100 3800 7100
Wire Wire Line
	3850 7000 3800 7000
Wire Wire Line
	2950 7000 3000 7000
Connection ~ 2950 6900
Text GLabel 2950 7100 0    50   Input ~ 0
AT25_CS_10
Wire Wire Line
	2950 7100 3000 7100
Wire Wire Line
	1100 900  1550 900 
$EndSCHEMATC
