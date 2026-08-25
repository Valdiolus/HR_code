Modbus Protocol

(1) Communication Parameters
Data bit: 8
Stop bit: 1
Parity Check: No.
Baud Rate: 115200bps

(2) Modbus Protocol Paramters
Mode: Modbus-RTU
Verificaiton: CRC-16/Modbus
Sensor Address: 0x01 by default (can be changed).
Read Code: 0x03
Write Code: 0x06

(3) Modbus Protocol Format

CRC=Cyclic Redundancy Check

(4.1) RS485 Modbus Protocl Format
Host Sends (TX, Read):
Name:
Address
Read Code
0x03
Register Address
Number of Registers
CRC16
Verification
Length (Byte)
1
1
2
2
2
Name:
Address
Read Code
0x03
Number of Data Bytes
| Data
| CRC16
Verification
Length (Byte)
1
1
1
N
2
Name:
| Address
Read Code
0x06
Register
Address
| Data
TCRC16
Verification
Length (Byte)
1
1
2
2
2
Client Replies (RX, Read):
Host Sends (TX, Write):
Client Replies (RX, Write):
aus
Name:
Address
Read Code
0x06
Register Address
Data
CRC16
Verification
Length (Byte)
1
1
2
2
2

(4.2)
Status
Address
Function
Data Type
Explanation

Read Only 0x0100
Processed
Value
Unsigned Int,
16 bits
As soon as the command is received, the sensor performs several measurements and calculates the results. After processed, a distance is output. Unit: mm.
Response Time: 100 ~ 300ms. (The longer distance, the longer time.)

Read Only 0x0101
Real-Time
Value
Unsigned Int, J a16 bits
As soon as the command is received, the sensor performs a measurement once and outputs a distance.
Unit: mm. Response Time: 15 ~ 140ms

Read Only 0x0102
Temperature
bits
Signed Int, 16 Unit 0.1°C. Resolution: 0.5°C. Response Time:
50~100ms.

Read Only 0x010A
Time to Receive
Echoed Wave
Unsigned Int,
16 bits
Time period from trigger to echoed wave received. Unit: u
s. This time divided by 5.75 will be the distance value in mm. Response time: 5~140ms.

(4.3)
Read/Write 0x0200
Slave
Address
Unsigned Int,
16 bits
Unsigned Int, Range: 0x01 ~ 0xFE, default: 0x01.
OxFFis broadcast address.

Read/Write 0x0201
Baud Rate
16 bits
Default: 0x09, 115200bps.
Unsigned Int, 0x01-2400, 0x02-4800, 0x03-9600,
0x04-14400, 0x05-19200, 0x06-38400, 0x07-57600, 0x08-76800, 0x09-115200

Read/Write 0x0205
Polarity of Switch
Output
Unsigned Int,
16 bits
Only applicable for Switch output.
Ox00: If the distance is less than threshold, TX terminal outputs low level.
0x01 (default): If the distance is less than threshold, TX
terminal outputs high level.

Read/Write 0x0206
Switch
Threshold for Unsigned Int,
16 bits
Only applicable for Switch output.
Set the threshold for switch output. Range: 30 ~
3000mm.
E.g. 0x03E8 = 1000mm.

Read/Write 0x0208
Detection
Angle Grade 16 bits
Unsigned Int,
4 grades available: 0x01 ~ 0x04. Default: 0x04.
The higher grade, the wider detection angle.
101 - 30°
02 - 40°
03 - 50°
04 - 60°

Read/Write 0x0209
Distance
Value Unit
Unsigned Int,
16 bits
Only applicable for UART Auto and Controlled output.
0x00 - mm (default), 0x01 - us (if divided by 5.75, it will be distance in mm).

(4.4)
Read/Write 0x021A
Grade
Power Noise Unsigned Int,
16 bits
5 grades available: 0x01 ~ 0x05. Default: 0x01.
The higher grade, the more resistant to noise but the higher impact on the detection angle.
01-applicable for battery power.
02 - for USBpowersupply with some high frequency noise.
03 - for longer distance USB power supply.
04 - for switch power
05 - for switch power with complicated interference in environment. Not recommended.

Read/Write 0x021F
Detection
Range Grade 16 bits
Unsigned Int,
4 grades available: 0x01 ~ 0x04. Default: 0x04.
The higher grade, the bigger range but longer response time.
01 - Range up to 50cm, response time of real value
15~110ms, response time of processed value
100~150ms;
02 - Range up to 150cm, response time of real value
15~120ms, response time of processed value
100~200ms;
03- Range up to 250cm, response time of real value
15~130ms, response time of processed value
100-250ms;


04 - Range up to 300cm, response time of real value
15~140ms, response time of processed value
100~300ms

(5) RS485 Modbus Communication Examples:
- To read processed value:
Host (TX): 01 03 01 00 00 01 85 F6
Client (RX): 01 03 02 02 F2 38 A1
Explanation: Address of sensor is 0x01. Prcessed distance value is 0x	02F2, of which decimal distance is 754mm.

- To read read time value:
Host (TX): 01 03 01 01 00 01 D4 36 
Client (RX): 01 03 02 02 EF F8 A8
Explanation: Address of sensor is 0x01. Read-time distance value is 0x02EF, of which decimal distance is 751mm.

- To read temperature:
Host (TX): 01 03 01 02 00 01 24 36
Client (RX): 01 03 02 01 2C B8 09
Explanation: Address of sensor is 0x01. Real-time temperature value is 0x012C, of which decimal temperature is 30.0℃.

- To change the address of sensor:
Host (TX): 01 06 02 00 00 05 48 71
Client (RX): 01 06 02 00 00 05 48 71
Explanation: Address of sensor is changed from 0x01to 0x05.

If the address of sensor is forgoten, its address can be checked in this way:
Host (TX): FF 03 02 00 00 01 90 6C
The sensor replies:
(RX): 01 03 02 00 01 79 84 
It shows the address of sensor is 0x01.

To read/check the current Baud Rate:
Host (TX): 01 03 02 01 00 01 D4 72
Client (RX): 01 03 02 00 03 F8 45
0x0003 indicates the current Baud Rate is 9600bps.

To set the Baud Rate at 2400bps.
Host (TX): 01 06 02 01 00 01 99 B3
Client (RX): 01 06 02 01 00 01 99 B3

(7) Long Distance Transmission of RS485

The adddress change code is useful when a host machine controls more than 1 sensor module and each sensor should be assigned with a unique address.