
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2
create_project: 2

00:00:052

00:00:052	
508.0082	
200.105Z17-268h px� 
�
Command: %s
1870*	planAhead2�
�read_checkpoint -auto_incremental -incremental C:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/utils_1/imports/synth_1/Nexys_A7.dcpZ12-2866h px� 
�
;Read reference checkpoint from %s for incremental synthesis3154*	planAhead2`
^C:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/utils_1/imports/synth_1/Nexys_A7.dcpZ12-5825h px� 
T
-Please ensure there are no constraint changes3725*	planAheadZ12-7989h px� 
d
Command: %s
53*	vivadotcl23
1synth_design -top Nexys_A7 -part xc7a100tcsg324-1Z4-113h px� 
:
Starting synth_design
149*	vivadotclZ4-321h px� 
{
@Attempting to get a license for feature '%s' and/or device '%s'
308*common2
	Synthesis2

xc7a100tZ17-347h px� 
k
0Got license for feature '%s' and/or device '%s'
310*common2
	Synthesis2

xc7a100tZ17-349h px� 
E
Loading part %s157*device2
xc7a100tcsg324-1Z21-403h px� 
[
$Part: %s does not have CEAM library.966*device2
xc7a100tcsg324-1Z21-9227h px� 

VNo compile time benefit to using incremental synthesis; A full resynthesis will be run2353*designutilsZ20-5440h px� 
�
�Flow is switching to default flow due to incremental criteria not met. If you would like to alter this behaviour and have the flow terminate instead, please set the following parameter config_implementation {autoIncr.Synth.RejectBehavior Terminate}2229*designutilsZ20-4379h px� 
o
HMultithreading enabled for synth_design using a maximum of %s processes.4828*oasys2
2Z8-7079h px� 
a
?Launching helper process for spawning children vivado processes4827*oasysZ8-7078h px� 
N
#Helper process launched with PID %s4824*oasys2
12112Z8-7075h px� 
�
%s*synth2{
yStarting RTL Elaboration : Time (s): cpu = 00:00:03 ; elapsed = 00:00:03 . Memory (MB): peak = 1381.785 ; gain = 448.367
h px� 
�
synthesizing module '%s'%s4497*oasys2

Nexys_A72
 2�
}C:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Project_top_module_exp3.v2
38@Z8-6157h px� 
�
synthesizing module '%s'%s4497*oasys2
MSSD2
 2n
jC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/MSSD.v2
38@Z8-6157h px� 
�
synthesizing module '%s'%s4497*oasys2
SSD2
 2m
iC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/SSD.v2
38@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
SSD2
 2
02
12m
iC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/SSD.v2
38@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
MSSD2
 2
02
12n
jC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/MSSD.v2
38@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
	debouncer2
 2s
oC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/debouncer.v2
38@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
	debouncer2
 2
02
12s
oC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/debouncer.v2
38@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
Multi_Cycle_Computer2
 2~
zC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Multi_Cycle_Computer.v2
18@Z8-6157h px� 
�
synthesizing module '%s'%s4497*oasys2
my_datapath2
 2u
qC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/my_datapath.v2
18@Z8-6157h px� 
�
synthesizing module '%s'%s4497*oasys2
Register_rsten2
 2x
tC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Register_rsten.v2
18@Z8-6157h px� 
I
%s
*synth21
/	Parameter WIDTH bound to: 32 - type: integer 
h p
x
� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
Register_rsten2
 2
02
12x
tC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Register_rsten.v2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2

Mux_2to12
 2r
nC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Mux_2to1.v2
18@Z8-6157h px� 
I
%s
*synth21
/	Parameter WIDTH bound to: 32 - type: integer 
h p
x
� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2

Mux_2to12
 2
02
12r
nC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Mux_2to1.v2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
	ID_memory2
 2s
oC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/ID_memory.v2
18@Z8-6157h px� 
M
%s
*synth25
3	Parameter BYTE_SIZE bound to: 32 - type: integer 
h p
x
� 
N
%s
*synth26
4	Parameter ADDR_WIDTH bound to: 32 - type: integer 
h p
x
� 
�
,$readmem data file '%s' is read successfully3426*oasys2
Instructions.hex2s
oC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/ID_memory.v2
118@Z8-3876h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
	ID_memory2
 2
02
12s
oC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/ID_memory.v2
18@Z8-6155h px� 
�
Pwidth (%s) of port connection '%s' does not match port width (%s) of module '%s'689*oasys2
322
WD2
2562
	ID_memory2u
qC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/my_datapath.v2
368@Z8-689h px� 
�
Pwidth (%s) of port connection '%s' does not match port width (%s) of module '%s'689*oasys2
322
RD2
2562
	ID_memory2u
qC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/my_datapath.v2
378@Z8-689h px� 
�
synthesizing module '%s'%s4497*oasys2
Register_en2
 2u
qC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Register_en.v2
18@Z8-6157h px� 
I
%s
*synth21
/	Parameter WIDTH bound to: 32 - type: integer 
h p
x
� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
Register_en2
 2
02
12u
qC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Register_en.v2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
Register_simple2
 2y
uC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Register_simple.v2
18@Z8-6157h px� 
I
%s
*synth21
/	Parameter WIDTH bound to: 32 - type: integer 
h p
x
� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
Register_simple2
 2
02
12y
uC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Register_simple.v2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
Mux_2to1__parameterized02
 2r
nC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Mux_2to1.v2
18@Z8-6157h px� 
H
%s
*synth20
.	Parameter WIDTH bound to: 4 - type: integer 
h p
x
� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
Mux_2to1__parameterized02
 2
02
12r
nC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Mux_2to1.v2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
Register_file2
 2w
sC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Register_file.v2
18@Z8-6157h px� 
I
%s
*synth21
/	Parameter WIDTH bound to: 32 - type: integer 
h p
x
� 
�
synthesizing module '%s'%s4497*oasys2
Register_rsten_neg2
 2|
xC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Register_rsten_neg.v2
18@Z8-6157h px� 
I
%s
*synth21
/	Parameter WIDTH bound to: 32 - type: integer 
h p
x
� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
Register_rsten_neg2
 2
02
12|
xC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Register_rsten_neg.v2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
Decoder_4to162
 2w
sC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Decoder_4to16.v2
18@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
Decoder_4to162
 2
02
12w
sC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Decoder_4to16.v2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
	Mux_16to12
 2s
oC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Mux_16to1.v2
18@Z8-6157h px� 
I
%s
*synth21
/	Parameter WIDTH bound to: 32 - type: integer 
h p
x
� 
�
default block is never used226*oasys2s
oC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Mux_16to1.v2
98@Z8-226h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
	Mux_16to12
 2
02
12s
oC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Mux_16to1.v2
18@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
Register_file2
 2
02
12w
sC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Register_file.v2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2

Extender2
 2r
nC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Extender.v2
18@Z8-6157h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2

Extender2
 2
02
12r
nC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Extender.v2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2

Mux_4to12
 2r
nC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Mux_4to1.v2
18@Z8-6157h px� 
I
%s
*synth21
/	Parameter WIDTH bound to: 32 - type: integer 
h p
x
� 
�
default block is never used226*oasys2r
nC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Mux_4to1.v2
98@Z8-226h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2

Mux_4to12
 2
02
12r
nC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Mux_4to1.v2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
Mux_2to1__parameterized12
 2r
nC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Mux_2to1.v2
18@Z8-6157h px� 
H
%s
*synth20
.	Parameter WIDTH bound to: 5 - type: integer 
h p
x
� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
Mux_2to1__parameterized12
 2
02
12r
nC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Mux_2to1.v2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
Mux_2to1__parameterized22
 2r
nC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Mux_2to1.v2
18@Z8-6157h px� 
H
%s
*synth20
.	Parameter WIDTH bound to: 2 - type: integer 
h p
x
� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
Mux_2to1__parameterized22
 2
02
12r
nC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Mux_2to1.v2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2	
shifter2
 2q
mC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/shifter.v2
18@Z8-6157h px� 
I
%s
*synth21
/	Parameter WIDTH bound to: 32 - type: integer 
h p
x
� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2	
shifter2
 2
02
12q
mC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/shifter.v2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
ALU2
 2m
iC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/ALU.v2
18@Z8-6157h px� 
I
%s
*synth21
/	Parameter WIDTH bound to: 32 - type: integer 
h p
x
� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
ALU2
 2
02
12m
iC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/ALU.v2
18@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
my_datapath2
 2
02
12u
qC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/my_datapath.v2
18@Z8-6155h px� 
�
synthesizing module '%s'%s4497*oasys2
my_controller2
 2w
sC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/my_controller.v2
18@Z8-6157h px� 
�
synthesizing module '%s'%s4497*oasys2 
Register_rsten__parameterized02
 2x
tC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Register_rsten.v2
18@Z8-6157h px� 
H
%s
*synth20
.	Parameter WIDTH bound to: 1 - type: integer 
h p
x
� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2 
Register_rsten__parameterized02
 2
02
12x
tC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Register_rsten.v2
18@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
my_controller2
 2
02
12w
sC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/my_controller.v2
18@Z8-6155h px� 
�
&Input port '%s' has an internal driver4442*oasys2
	fsm_state2~
zC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Multi_Cycle_Computer.v2
548@Z8-6104h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
PCS2
my_controller2
my_controller2~
zC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Multi_Cycle_Computer.v2
368@Z8-7071h px� 
�
Kinstance '%s' of module '%s' has %s connections declared, but only %s given4757*oasys2
my_controller2
my_controller2
192
182~
zC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Multi_Cycle_Computer.v2
368@Z8-7023h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
Multi_Cycle_Computer2
 2
02
12~
zC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Multi_Cycle_Computer.v2
18@Z8-6155h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2

Nexys_A72
 2
02
12�
}C:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Project_top_module_exp3.v2
38@Z8-6155h px� 
�
qTrying to implement RAM '%s' in registers. Block RAM or DRAM implementation is not possible; see log for reasons.3901*oasys2	
mem_regZ8-4767h px� 
C
%s
*synth2+
)Reason is one or more of the following :
h p
x
� 
�
%s
*synth2�
~	1: RAM has multiple writes via different ports in same process. If RAM inferencing intended, write to one port per process. 
h p
x
� 
X
%s
*synth2@
>	2: Unable to determine number of words or word size in RAM. 
h p
x
� 
B
%s
*synth2*
(	3: No valid read/write found for RAM. 
h p
x
� 
A
%s
*synth2)
'RAM "mem_reg" dissolved into registers
h p
x
� 
�
0Net %s in module/entity %s does not have driver.3422*oasys2
	fsm_state2

Nexys_A72�
}C:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/Project_top_module_exp3.v2
218@Z8-3848h px� 
~
9Port %s in module %s is either unconnected or has no load4866*oasys2
INSTRUCTION[3]2
my_controllerZ8-7129h px� 
~
9Port %s in module %s is either unconnected or has no load4866*oasys2
INSTRUCTION[2]2
my_controllerZ8-7129h px� 
~
9Port %s in module %s is either unconnected or has no load4866*oasys2
INSTRUCTION[1]2
my_controllerZ8-7129h px� 
~
9Port %s in module %s is either unconnected or has no load4866*oasys2
INSTRUCTION[0]2
my_controllerZ8-7129h px� 
�
%s*synth2{
yFinished RTL Elaboration : Time (s): cpu = 00:00:09 ; elapsed = 00:00:09 . Memory (MB): peak = 1912.895 ; gain = 979.477
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
;
%s
*synth2#
!Start Handling Custom Attributes
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Handling Custom Attributes : Time (s): cpu = 00:00:09 ; elapsed = 00:00:10 . Memory (MB): peak = 1912.895 ; gain = 979.477
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished RTL Optimization Phase 1 : Time (s): cpu = 00:00:09 ; elapsed = 00:00:10 . Memory (MB): peak = 1912.895 ; gain = 979.477
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2
Netlist sorting complete. 2

00:00:002
00:00:00.2972

1912.8952
0.000Z17-268h px� 
K
)Preparing netlist for logic optimization
349*projectZ1-570h px� 
>

Processing XDC Constraints
244*projectZ1-262h px� 
=
Initializing timing engine
348*projectZ1-569h px� 
�
Parsing XDC File [%s]
179*designutils2n
jC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/constrs_1/imports/EE446/Nexys-A7-100T-Master.xdc8Z20-179h px� 
�
Finished Parsing XDC File [%s]
178*designutils2n
jC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/constrs_1/imports/EE446/Nexys-A7-100T-Master.xdc8Z20-178h px� 
�
�Implementation specific constraints were found while reading constraint file [%s]. These constraints will be ignored for synthesis but will be used in implementation. Impacted constraints are listed in the file [%s].
233*project2l
jC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/constrs_1/imports/EE446/Nexys-A7-100T-Master.xdc2
.Xil/Nexys_A7_propImpl.xdcZ1-236h px� 
H
&Completed Processing XDC Constraints

245*projectZ1-263h px� 
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2
Netlist sorting complete. 2

00:00:002
00:00:00.0022

2021.9142
0.000Z17-268h px� 
l
!Unisim Transformation Summary:
%s111*project2'
%No Unisim elements were transformed.
Z1-111h px� 
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2"
 Constraint Validation Runtime : 2

00:00:002
00:00:00.0522

2021.9142
0.000Z17-268h px� 

VNo compile time benefit to using incremental synthesis; A full resynthesis will be run2353*designutilsZ20-5440h px� 
�
�Flow is switching to default flow due to incremental criteria not met. If you would like to alter this behaviour and have the flow terminate instead, please set the following parameter config_implementation {autoIncr.Synth.RejectBehavior Terminate}2229*designutilsZ20-4379h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Constraint Validation : Time (s): cpu = 00:00:18 ; elapsed = 00:00:19 . Memory (MB): peak = 2021.914 ; gain = 1088.496
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
D
%s
*synth2,
*Start Loading Part and Timing Information
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
9
%s
*synth2!
Loading part: xc7a100tcsg324-1
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Loading Part and Timing Information : Time (s): cpu = 00:00:18 ; elapsed = 00:00:19 . Memory (MB): peak = 2021.914 ; gain = 1088.496
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
H
%s
*synth20
.Start Applying 'set_property' XDC Constraints
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished applying 'set_property' XDC Constraints : Time (s): cpu = 00:00:18 ; elapsed = 00:00:19 . Memory (MB): peak = 2021.914 ; gain = 1088.496
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
q
3inferred FSM for state register '%s' in module '%s'802*oasys2
	state_reg2
my_controllerZ8-802h px� 
�
!inferring latch for variable '%s'327*oasys2
FSM_sequential_nextstate_reg2w
sC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/my_controller.v2
728@Z8-327h px� 
�
!inferring latch for variable '%s'327*oasys2
FSM_onehot_nextstate_reg2w
sC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/my_controller.v2
728@Z8-327h px� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
z
%s
*synth2b
`                   State |                     New Encoding |                Previous Encoding 
h p
x
� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
y
%s
*synth2a
_                      S0 |                      00000000100 |                             0000
h p
x
� 
y
%s
*synth2a
_                      S1 |                      00001000000 |                             0001
h p
x
� 
y
%s
*synth2a
_                     S10 |                      00000100000 |                             1010
h p
x
� 
y
%s
*synth2a
_                      S2 |                      00000001000 |                             0010
h p
x
� 
y
%s
*synth2a
_                      S3 |                      00000000001 |                             0011
h p
x
� 
y
%s
*synth2a
_                      S4 |                      00000000010 |                             0100
h p
x
� 
y
%s
*synth2a
_                      S5 |                      00000010000 |                             0101
h p
x
� 
y
%s
*synth2a
_                      S6 |                      00010000000 |                             0110
h p
x
� 
y
%s
*synth2a
_                      S7 |                      00100000000 |                             0111
h p
x
� 
y
%s
*synth2a
_                      S8 |                      10000000000 |                             1000
h p
x
� 
y
%s
*synth2a
_                      S9 |                      01000000000 |                             1001
h p
x
� 
~
%s
*synth2f
d---------------------------------------------------------------------------------------------------
h p
x
� 
�
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys2
	state_reg2	
one-hot2
my_controllerZ8-3354h px� 
�
!inferring latch for variable '%s'327*oasys2
FSM_onehot_nextstate_reg2w
sC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.srcs/sources_1/imports/EE446_EXP2_2515583_Code/my_controller.v2
728@Z8-327h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished RTL Optimization Phase 2 : Time (s): cpu = 00:00:35 ; elapsed = 00:00:38 . Memory (MB): peak = 2021.914 ; gain = 1088.496
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
:
%s
*synth2"
 Start RTL Component Statistics 
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
9
%s
*synth2!
Detailed RTL Component Info : 
h p
x
� 
(
%s
*synth2
+---Adders : 
h p
x
� 
F
%s
*synth2.
,	   3 Input   33 Bit       Adders := 6     
h p
x
� 
F
%s
*synth2.
,	   2 Input   32 Bit       Adders := 1     
h p
x
� 
F
%s
*synth2.
,	   2 Input    3 Bit       Adders := 1     
h p
x
� 
&
%s
*synth2
+---XORs : 
h p
x
� 
H
%s
*synth20
.	   2 Input     32 Bit         XORs := 2     
h p
x
� 
H
%s
*synth20
.	   2 Input      1 Bit         XORs := 1     
h p
x
� 
+
%s
*synth2
+---Registers : 
h p
x
� 
H
%s
*synth20
.	               32 Bit    Registers := 22    
h p
x
� 
H
%s
*synth20
.	                8 Bit    Registers := 256   
h p
x
� 
H
%s
*synth20
.	                3 Bit    Registers := 1     
h p
x
� 
H
%s
*synth20
.	                1 Bit    Registers := 10    
h p
x
� 
'
%s
*synth2
+---Muxes : 
h p
x
� 
F
%s
*synth2.
,	   2 Input   32 Bit        Muxes := 4     
h p
x
� 
F
%s
*synth2.
,	   4 Input   32 Bit        Muxes := 4     
h p
x
� 
F
%s
*synth2.
,	  17 Input   11 Bit        Muxes := 1     
h p
x
� 
F
%s
*synth2.
,	   2 Input    8 Bit        Muxes := 196   
h p
x
� 
F
%s
*synth2.
,	   2 Input    5 Bit        Muxes := 1     
h p
x
� 
F
%s
*synth2.
,	   2 Input    4 Bit        Muxes := 4     
h p
x
� 
F
%s
*synth2.
,	   3 Input    4 Bit        Muxes := 1     
h p
x
� 
F
%s
*synth2.
,	  11 Input    4 Bit        Muxes := 2     
h p
x
� 
F
%s
*synth2.
,	   2 Input    2 Bit        Muxes := 1     
h p
x
� 
F
%s
*synth2.
,	  11 Input    2 Bit        Muxes := 2     
h p
x
� 
F
%s
*synth2.
,	   4 Input    2 Bit        Muxes := 1     
h p
x
� 
F
%s
*synth2.
,	   2 Input    1 Bit        Muxes := 205   
h p
x
� 
F
%s
*synth2.
,	  33 Input    1 Bit        Muxes := 124   
h p
x
� 
F
%s
*synth2.
,	  32 Input    1 Bit        Muxes := 8     
h p
x
� 
F
%s
*synth2.
,	   6 Input    1 Bit        Muxes := 1     
h p
x
� 
F
%s
*synth2.
,	  11 Input    1 Bit        Muxes := 5     
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
=
%s
*synth2%
#Finished RTL Component Statistics 
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
6
%s
*synth2
Start Part Resource Summary
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
q
%s
*synth2Y
WPart Resources:
DSPs: 240 (col length:80)
BRAMs: 270 (col length: RAMB18 80 RAMB36 40)
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
9
%s
*synth2!
Finished Part Resource Summary
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
E
%s
*synth2-
+Start Cross Boundary and Area Optimization
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
e
8ROM "%s" won't be mapped to RAM because it is too sparse3998*oasys2	
p_0_outZ8-5546h px� 
�
�Message '%s' appears more than %s times and has been disabled. User can change this message limit to see more message instances.
14*common2
Synth 8-55462
100Z17-14h px� 
�
�Message '%s' appears more than %s times and has been disabled. User can change this message limit to see more message instances.
14*common2
Synth 8-55462
100Z17-14h px� 
�
�Message '%s' appears more than %s times and has been disabled. User can change this message limit to see more message instances.
14*common2
Synth 8-55462
100Z17-14h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Cross Boundary and Area Optimization : Time (s): cpu = 00:01:00 ; elapsed = 00:01:23 . Memory (MB): peak = 2021.914 ; gain = 1088.496
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
@
%s
*synth2(
&Start Applying XDC Timing Constraints
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Applying XDC Timing Constraints : Time (s): cpu = 00:01:05 ; elapsed = 00:01:28 . Memory (MB): peak = 2021.914 ; gain = 1088.496
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
4
%s
*synth2
Start Timing Optimization
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
~Finished Timing Optimization : Time (s): cpu = 00:01:06 ; elapsed = 00:01:29 . Memory (MB): peak = 2021.914 ; gain = 1088.496
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
3
%s
*synth2
Start Technology Mapping
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
H
&Parallel synthesis criteria is not met4829*oasysZ8-7080h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2
}Finished Technology Mapping : Time (s): cpu = 00:01:09 ; elapsed = 00:01:31 . Memory (MB): peak = 2021.914 ; gain = 1088.496
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
-
%s
*synth2
Start IO Insertion
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
?
%s
*synth2'
%Start Flattening Before IO Insertion
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
B
%s
*synth2*
(Finished Flattening Before IO Insertion
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
6
%s
*synth2
Start Final Netlist Cleanup
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
9
%s
*synth2!
Finished Final Netlist Cleanup
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2y
wFinished IO Insertion : Time (s): cpu = 00:01:12 ; elapsed = 00:01:35 . Memory (MB): peak = 2021.914 ; gain = 1088.496
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
=
%s
*synth2%
#Start Renaming Generated Instances
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Renaming Generated Instances : Time (s): cpu = 00:01:12 ; elapsed = 00:01:35 . Memory (MB): peak = 2021.914 ; gain = 1088.496
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
:
%s
*synth2"
 Start Rebuilding User Hierarchy
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Rebuilding User Hierarchy : Time (s): cpu = 00:01:13 ; elapsed = 00:01:35 . Memory (MB): peak = 2021.914 ; gain = 1088.496
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
9
%s
*synth2!
Start Renaming Generated Ports
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Renaming Generated Ports : Time (s): cpu = 00:01:13 ; elapsed = 00:01:36 . Memory (MB): peak = 2021.914 ; gain = 1088.496
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
;
%s
*synth2#
!Start Handling Custom Attributes
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Handling Custom Attributes : Time (s): cpu = 00:01:13 ; elapsed = 00:01:36 . Memory (MB): peak = 2021.914 ; gain = 1088.496
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
8
%s
*synth2 
Start Renaming Generated Nets
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Renaming Generated Nets : Time (s): cpu = 00:01:13 ; elapsed = 00:01:36 . Memory (MB): peak = 2021.914 ; gain = 1088.496
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
9
%s
*synth2!
Start Writing Synthesis Report
h p
x
� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
/
%s
*synth2

Report BlackBoxes: 
h p
x
� 
8
%s
*synth2 
+-+--------------+----------+
h p
x
� 
8
%s
*synth2 
| |BlackBox name |Instances |
h p
x
� 
8
%s
*synth2 
+-+--------------+----------+
h p
x
� 
8
%s
*synth2 
+-+--------------+----------+
h p
x
� 
/
%s*synth2

Report Cell Usage: 
h px� 
2
%s*synth2
+------+-------+------+
h px� 
2
%s*synth2
|      |Cell   |Count |
h px� 
2
%s*synth2
+------+-------+------+
h px� 
2
%s*synth2
|1     |BUFG   |     2|
h px� 
2
%s*synth2
|2     |CARRY4 |    78|
h px� 
2
%s*synth2
|3     |LUT1   |     3|
h px� 
2
%s*synth2
|4     |LUT2   |   179|
h px� 
2
%s*synth2
|5     |LUT3   |   726|
h px� 
2
%s*synth2
|6     |LUT4   |  1459|
h px� 
2
%s*synth2
|7     |LUT5   |   781|
h px� 
2
%s*synth2
|8     |LUT6   |  5662|
h px� 
2
%s*synth2
|9     |MUXF7  |  1358|
h px� 
2
%s*synth2
|10    |MUXF8  |   628|
h px� 
2
%s*synth2
|11    |FDRE   |  2858|
h px� 
2
%s*synth2
|12    |FDSE   |     1|
h px� 
2
%s*synth2
|13    |LD     |    11|
h px� 
2
%s*synth2
|14    |IBUF   |    19|
h px� 
2
%s*synth2
|15    |OBUF   |    32|
h px� 
2
%s*synth2
+------+-------+------+
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
�
%s*synth2�
�Finished Writing Synthesis Report : Time (s): cpu = 00:01:13 ; elapsed = 00:01:36 . Memory (MB): peak = 2021.914 ; gain = 1088.496
h px� 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
� 
`
%s
*synth2H
FSynthesis finished with 0 errors, 0 critical warnings and 4 warnings.
h p
x
� 
�
%s
*synth2�
Synthesis Optimization Runtime : Time (s): cpu = 00:01:00 ; elapsed = 00:01:32 . Memory (MB): peak = 2021.914 ; gain = 979.477
h p
x
� 
�
%s
*synth2�
�Synthesis Optimization Complete : Time (s): cpu = 00:01:14 ; elapsed = 00:01:37 . Memory (MB): peak = 2021.914 ; gain = 1088.496
h p
x
� 
B
 Translating synthesized netlist
350*projectZ1-571h px� 
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2
Netlist sorting complete. 2

00:00:002
00:00:00.0842

2021.9142
0.000Z17-268h px� 
V
-Analyzing %s Unisim elements for replacement
17*netlist2
2075Z29-17h px� 
X
2Unisim Transformation completed in %s CPU seconds
28*netlist2
0Z29-28h px� 
K
)Preparing netlist for logic optimization
349*projectZ1-570h px� 
Q
)Pushed %s inverter(s) to %s load pin(s).
98*opt2
02
0Z31-138h px� 
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2
Netlist sorting complete. 2

00:00:002
00:00:00.0022

2021.9142
0.000Z17-268h px� 
�
!Unisim Transformation Summary:
%s111*project2I
G  A total of 11 instances were transformed.
  LD => LDCE: 11 instances
Z1-111h px� 
V
%Synth Design complete | Checksum: %s
562*	vivadotcl2

34ca62e9Z4-1430h px� 
C
Releasing license: %s
83*common2
	SynthesisZ17-83h px� 
�
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
1782
152
02
0Z4-41h px� 
L
%s completed successfully
29*	vivadotcl2
synth_designZ4-42h px� 
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2
synth_design: 2

00:01:182

00:01:422

2021.9142

1505.188Z17-268h px� 
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2
Write ShapeDB Complete: 2

00:00:002
00:00:00.0252

2021.9142
0.000Z17-268h px� 
�
 The %s '%s' has been generated.
621*common2

checkpoint2P
NC:/Users/bahar/EE446_EXP3_2515583/EE446_EXP3_2515583.runs/synth_1/Nexys_A7.dcpZ17-1381h px� 
�
Executing command : %s
56330*	planAhead2[
Yreport_utilization -file Nexys_A7_utilization_synth.rpt -pb Nexys_A7_utilization_synth.pbZ12-24828h px� 
\
Exiting %s at %s...
206*common2
Vivado2
Tue Apr  8 10:57:45 2025Z17-206h px� 


End Record