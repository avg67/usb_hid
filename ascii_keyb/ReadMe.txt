*** ACHTUNG ***

Auf der KEYr4 Platine muss ein zusätzliche Leitung gezogen werden!
Von Pin 8 des ATTiny2313 zum Pin 8 des 74SL374.


ATTiny2313

PIN		KEYr4		KEY4/Maus
1  Reset
2  PD0/RxD	NC		NC
3  PD1/TxD	PS/2-Data	NC
4  PA1		NC		NKC-Reset
5  PA0		NC		#NKC-INT
6  PD2/INT0	PS/2-Clk	PS/2-Clk
7  PD3/INT1	KEY-Strobe	KEY4-Ready
8  PD4		*KEY4-Ready	PS/2-Data
9  PD5		NC		#Keyboard activ
10 GND
11 PD6		NKC-Reset	NC
12 PB0		KEY-D0		KEY-D0
13 PB1		KEY-D1		KEY-D1
14 PB2		KEY-D2		KEY-D2
15 PB3		KEY-D3		KEY-D3
16 PB3		KEY-D4		KEY-D4
17 PB5		KEY-D5		KEY-D5
18 PB5		KEY-D6		KEY-D6
19 PB7		NC		KEY-Strobe
20 Vcc

* = neue Verbindung
# = entfällt