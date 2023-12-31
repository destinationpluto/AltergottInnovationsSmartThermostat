EESchema Schematic File Version 4
LIBS:THERMOSTAT-cache
EELAYER 26 0
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
L CUSTOM:DHT22 DHT1
U 1 1 5BDC7DEF
P 1800 1050
F 0 "DHT1" H 2008 1190 50  0000 L CNN
F 1 "DHT22" H 2008 1099 50  0000 L CNN
F 2 "CUSTOMDEVICES:DHT22FOOT" H 1800 1650 50  0001 C CNN
F 3 "" H 1800 1100 50  0001 C CNN
	1    1800 1050
	1    0    0    -1  
$EndComp
$Comp
L CUSTOM:CAPTOUCH TouchUp1
U 1 1 5BDC7E6F
P 4500 4500
F 0 "TouchUp1" H 4778 4490 50  0000 L CNN
F 1 "CAPTOUCH" H 4778 4399 50  0000 L CNN
F 2 "CUSTOMDEVICES:CAPTOUCHFOOT" H 4400 4800 50  0001 C CNN
F 3 "" H 4500 4500 50  0001 C CNN
	1    4500 4500
	-1   0    0    1   
$EndComp
$Comp
L CUSTOM:CAPTOUCH TouchDown1
U 1 1 5BDC7ED6
P 3050 4500
F 0 "TouchDown1" H 3328 4490 50  0000 L CNN
F 1 "CAPTOUCH" H 3328 4399 50  0000 L CNN
F 2 "CUSTOMDEVICES:CAPTOUCHFOOT" H 2950 4800 50  0001 C CNN
F 3 "" H 3050 4500 50  0001 C CNN
	1    3050 4500
	-1   0    0    1   
$EndComp
$Comp
L CUSTOM:ILI9341TFT TFT1
U 1 1 5BDE4A2A
P 6050 2500
F 0 "TFT1" H 5541 2429 50  0000 R CNN
F 1 "ILI9341TFT" H 5541 2520 50  0000 R CNN
F 2 "CUSTOMDEVICES:ILI9341TFTFOOT" H 5800 3000 50  0001 C CNN
F 3 "" H 6050 2500 50  0001 C CNN
	1    6050 2500
	-1   0    0    1   
$EndComp
Wire Wire Line
	3200 4100 3200 3550
Connection ~ 3200 3550
Wire Wire Line
	4650 4100 4650 3550
Connection ~ 4650 3550
Wire Wire Line
	2900 4100 2900 3750
Connection ~ 2900 3750
Wire Wire Line
	2900 3750 3500 3750
Wire Wire Line
	4350 4100 4350 3750
Connection ~ 4350 3750
Wire Wire Line
	2750 3200 3050 3200
Wire Wire Line
	3050 3200 3050 4100
Wire Wire Line
	5700 1800 5100 1800
Wire Wire Line
	5100 1800 5100 3550
Wire Wire Line
	5700 1900 5200 1900
Wire Wire Line
	5200 1900 5200 3750
Wire Wire Line
	5300 2000 5700 2000
Wire Wire Line
	5700 2200 5400 2200
Wire Wire Line
	5400 2200 5400 1700
Wire Wire Line
	5700 2500 5500 2500
Wire Wire Line
	5500 2500 5500 1200
Wire Wire Line
	1650 1300 1650 1350
Wire Wire Line
	1650 3050 2000 3050
Wire Wire Line
	2000 3050 2000 3550
Connection ~ 2000 3550
Wire Wire Line
	2000 3550 3200 3550
Wire Wire Line
	1950 1300 1950 1750
Wire Wire Line
	1950 2900 2200 2900
Wire Wire Line
	2200 2900 2200 3750
Connection ~ 2200 3750
Wire Wire Line
	2200 3750 2900 3750
Wire Wire Line
	1750 1300 1750 1500
Wire Wire Line
	5700 2400 2550 2400
Wire Wire Line
	5700 2300 2850 2300
Wire Wire Line
	2850 2300 2850 1650
Wire Wire Line
	5700 2100 5600 2100
Wire Wire Line
	5600 2100 5600 750 
Wire Wire Line
	5600 750  3600 750 
$Comp
L CUSTOM:WEMOSD1MINI ESP1
U 1 1 5BE01E75
P 3500 1450
F 0 "ESP1" H 3500 2228 50  0000 C CNN
F 1 "WEMOSD1MINI" H 3500 2137 50  0000 C CNN
F 2 "CUSTOMDEVICES:WeMosD1Mini" H 3500 2250 50  0001 C CNN
F 3 "" H 3500 1450 50  0001 C CNN
	1    3500 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 3550 5100 3550
Wire Wire Line
	4350 3750 5200 3750
Wire Wire Line
	3650 2150 3650 3550
Connection ~ 3650 3550
Wire Wire Line
	3650 3550 4650 3550
Wire Wire Line
	3950 1800 5000 1800
Wire Wire Line
	5000 1800 5000 1700
Wire Wire Line
	5000 1700 5400 1700
Wire Wire Line
	3950 1350 5300 1350
Wire Wire Line
	5300 1350 5300 2000
Wire Wire Line
	3950 1200 5500 1200
Wire Wire Line
	4500 4100 4500 1500
Wire Wire Line
	4500 1500 3950 1500
Wire Wire Line
	3600 850  3600 750 
Wire Wire Line
	2750 1050 3050 1050
Wire Wire Line
	2750 1050 2750 3200
Wire Wire Line
	2550 1200 3050 1200
Wire Wire Line
	2550 1200 2550 2400
Wire Wire Line
	2850 1650 3050 1650
Wire Wire Line
	2650 1800 3050 1800
Wire Wire Line
	3200 3550 3650 3550
Wire Wire Line
	3500 2150 3500 3750
Connection ~ 3500 3750
Wire Wire Line
	3500 3750 4350 3750
$Comp
L Mechanical:MountingHole Hole1
U 1 1 5BE258DF
P 6400 1050
F 0 "Hole1" H 6500 1096 50  0000 L CNN
F 1 "MountingHole" H 6500 1005 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 6400 1050 50  0001 C CNN
F 3 "~" H 6400 1050 50  0001 C CNN
	1    6400 1050
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole Hole4
U 1 1 5BE259D8
P 6950 1400
F 0 "Hole4" H 7050 1446 50  0000 L CNN
F 1 "MountingHole" H 7050 1355 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 6950 1400 50  0001 C CNN
F 3 "~" H 6950 1400 50  0001 C CNN
	1    6950 1400
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole Hole2
U 1 1 5BE25A15
P 7350 1050
F 0 "Hole2" H 7450 1096 50  0000 L CNN
F 1 "MountingHole" H 7450 1005 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 7350 1050 50  0001 C CNN
F 3 "~" H 7350 1050 50  0001 C CNN
	1    7350 1050
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole Hole3
U 1 1 5BE25A71
P 8300 1100
F 0 "Hole3" H 8400 1146 50  0000 L CNN
F 1 "MountingHole" H 8400 1055 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 8300 1100 50  0001 C CNN
F 3 "~" H 8300 1100 50  0001 C CNN
	1    8300 1100
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole Hole5
U 1 1 5BE6F82C
P 8100 1550
F 0 "Hole5" H 8200 1596 50  0000 L CNN
F 1 "MountingHole" H 8200 1505 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 8100 1550 50  0001 C CNN
F 3 "~" H 8100 1550 50  0001 C CNN
	1    8100 1550
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole Hole6
U 1 1 5BE6F87E
P 9100 1550
F 0 "Hole6" H 9200 1596 50  0000 L CNN
F 1 "MountingHole" H 9200 1505 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 9100 1550 50  0001 C CNN
F 3 "~" H 9100 1550 50  0001 C CNN
	1    9100 1550
	1    0    0    -1  
$EndComp
$Comp
L CUSTOM:TERMINAL3PIN TERMINAL1
U 1 1 5C15ABDB
P 5000 6800
F 0 "TERMINAL1" V 4949 6927 50  0000 L CNN
F 1 "TERMINAL3PIN" V 5040 6927 50  0000 L CNN
F 2 "CUSTOMDEVICES:TERMINAL3PINFOOT" H 4900 6700 50  0001 C CNN
F 3 "" H 5000 6800 50  0001 C CNN
	1    5000 6800
	0    1    1    0   
$EndComp
$Comp
L CUSTOM:TERMINAL3PIN DHTEXT1
U 1 1 5C164B07
P 900 900
F 0 "DHTEXT1" H 1128 865 50  0000 L CNN
F 1 "TERMINAL3PIN" H 1128 774 50  0000 L CNN
F 2 "CUSTOMDEVICES:TERMINAL3PINFOOT" H 800 800 50  0001 C CNN
F 3 "" H 900 900 50  0001 C CNN
	1    900  900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  1150 750  1350
Wire Wire Line
	750  1350 1650 1350
Connection ~ 1650 1350
Wire Wire Line
	1650 1350 1650 3050
Wire Wire Line
	900  1150 900  1500
Wire Wire Line
	900  1500 1750 1500
Connection ~ 1750 1500
Wire Wire Line
	1750 1500 1750 1650
Wire Wire Line
	1050 1150 1050 1750
Wire Wire Line
	1050 1750 1950 1750
Connection ~ 1950 1750
Wire Wire Line
	1950 1750 1950 2900
$Comp
L CUSTOM:HLKPM01 HLKPM01
U 1 1 5C68E636
P 1800 6800
F 0 "HLKPM01" H 1825 5685 50  0000 C CNN
F 1 "HLKPM01" H 1825 5776 50  0000 C CNN
F 2 "CUSTOMDEVICES:HLKPM01FOOT" H 1800 7350 50  0001 C CNN
F 3 "" H 1800 7250 50  0001 C CNN
	1    1800 6800
	-1   0    0    1   
$EndComp
Wire Wire Line
	2400 6950 2850 6950
Wire Wire Line
	2400 6650 4600 6650
Wire Wire Line
	4750 6800 2850 6800
Wire Wire Line
	2850 6800 2850 6950
Wire Wire Line
	1150 6000 900  6000
Wire Wire Line
	900  6000 900  3550
Wire Wire Line
	900  3550 2000 3550
Wire Wire Line
	1150 7450 650  7450
Wire Wire Line
	650  7450 650  3750
Wire Wire Line
	650  3750 2200 3750
Wire Wire Line
	3050 1350 2350 1350
Wire Wire Line
	2350 1350 2350 1650
Wire Wire Line
	2350 1650 1750 1650
$Comp
L CUSTOM:MOC3021 MOC1
U 1 1 5C6B126C
P 6150 5500
F 0 "MOC1" H 6150 5825 50  0000 C CNN
F 1 "MOC3021" H 6150 5734 50  0000 C CNN
F 2 "CUSTOMDEVICES:MOC3021" H 6150 5900 50  0001 C CNN
F 3 "" H 6150 5500 50  0001 C CNN
	1    6150 5500
	1    0    0    -1  
$EndComp
$Comp
L CUSTOM:RESISTOR 220_OHM1
U 1 1 5C6B137C
P 5500 5400
F 0 "220_OHM1" V 5275 5400 50  0000 C CNN
F 1 "RESISTOR" V 5366 5400 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" H 5250 5250 50  0001 C CNN
F 3 "" H 5500 5400 50  0001 C CNN
	1    5500 5400
	0    1    1    0   
$EndComp
$Comp
L CUSTOM:RESISTOR 1K_OHM1
U 1 1 5C6B140C
P 6700 5400
F 0 "1K_OHM1" V 6475 5400 50  0000 C CNN
F 1 "RESISTOR" V 6566 5400 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0414_L11.9mm_D4.5mm_P15.24mm_Horizontal" H 6450 5250 50  0001 C CNN
F 3 "" H 6700 5400 50  0001 C CNN
	1    6700 5400
	0    1    1    0   
$EndComp
$Comp
L CUSTOM:BTA16_600 TRIAC1
U 1 1 5C6B14A0
P 7050 5800
F 0 "TRIAC1" H 7179 5846 50  0000 L CNN
F 1 "BTA16_600" H 7179 5755 50  0000 L CNN
F 2 "CUSTOMDEVICES:BTA16FOOT" H 6950 6200 50  0001 C CNN
F 3 "" H 7050 5800 50  0001 C CNN
	1    7050 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 5400 5850 5400
Wire Wire Line
	5850 5600 5750 5600
Wire Wire Line
	5750 5600 5750 3750
Wire Wire Line
	5750 3750 5200 3750
Connection ~ 5200 3750
Wire Wire Line
	6550 5400 6450 5400
Wire Wire Line
	6850 5400 7050 5400
Wire Wire Line
	7050 5400 7050 5650
Wire Wire Line
	4600 6650 4600 6300
Wire Wire Line
	4600 6300 7750 6300
Wire Wire Line
	7750 6300 7750 5400
Wire Wire Line
	7750 5400 7050 5400
Connection ~ 4600 6650
Wire Wire Line
	4600 6650 4750 6650
Connection ~ 7050 5400
Wire Wire Line
	6450 5600 6600 5600
Wire Wire Line
	6600 5600 6600 5900
Wire Wire Line
	6600 5900 6900 5900
Wire Wire Line
	7050 5950 4300 5950
Wire Wire Line
	4300 5950 4300 6950
Wire Wire Line
	4300 6950 4750 6950
Wire Wire Line
	5350 5400 2650 5400
Wire Wire Line
	2650 1800 2650 5400
$EndSCHEMATC
