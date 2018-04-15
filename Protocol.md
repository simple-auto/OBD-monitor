=====================================================================
=====================================================================
=====================================================================
== Data Transfer Protocol for Terralink Hardware to Innobec Server ==
=====================================================================
=====================================================(Draft #1)======
=====================================================================
Revo Project,
Terralink SpA, Chile
revoproject@terralink.cl

1) Server address
=================
IP:TCP_Port

2) Identification
=================
IMEI (International Mobile Equipment Identifier)
Structure:
xx xx xx xx SS SS SS v
x refer to manufacturer
S = Serial Number (6 digit)
v = verification number
Devices will be identified by their 6-digit serial no.
(Los dispositivos se identificarán por su número de serie de 6 dígitos.)
3) Packets
==========
Types of packets
----------------
0 = DTCs (Diagnostic Trouble Codes)
1 = Preprocessed data (decimal)
2 = Raw OBD data (Hexadecimal)
3...9 Future Use
Structure
---------
SSSSSS A P...P \n (newline)
S (6 digit) = Serial number
A (1 digit) = Type of packet
P (D digit) = PayloadType 0 Packet Structure
-----------------------
(SSSSSS A=0 P) P = NN DDDDD_1 ... DDDDD_N
N
D_i
(2 digit)
(5 digit)
=
=
Number of DTCs
i-th Diagnostic Trouble Code (i in 1 to N)
Type 0 Packet Example:
SSSSSSAP...
\n
256358003P0106P0104U1101
Serial Number = S = 256358
Type of Packet = A = 0
Payload = NN DDDDD_1 ... DDDDD_N = 03 P0106 P0104 U1101
Number of DTCs = N = 3
DTC 1 = D_1 = P0106
DTC 2 = D_2 = P0104
DTC 3 = D_3 = U1101
Type 1 Packet Structure
-----------------------
(SSSSSS A=1 P) P = VVV_1 ... VVV_30 NNN_1 ... NNN_30 BBB CCC T...T, D...D
V_i
N_i
C
B
T
D
(3
(3
(3
(3
digit)
digit)
digit)
digit)
(***)
(***)
=
=
=
=
=
=
i-th Velocity [km/h] (i in 1 to 30)
i-th Revolutions per minute *10[rpm] (i in 1 to 30)
Coolant Temperature [Celcius deg.]
Battery Voltage *0.1[V]
Timestamp [s] (initial timestamp)
Distance traveled during the packet collection [m] (delta)
*** = Variable length. Comma separated.
Type 1 Packet Example:
SSSSSSAP...
.....
2563581120...120120120120500...0050050003012039,1914
Serial Number = S = 256358
Type of Packet = A = 1
Payload = V (1 to 30) N (1 to 30) B C, T, D
V = 120 [km/h]
N = 500*10 [rpm] = 5000 [rpm]
C = 30 [Celsius degrees]
B = 120 * 0.1 [V] = 12.0 [V]
T = 39 [s]
D = 1914 [m] = 1.914 [km]
\n
