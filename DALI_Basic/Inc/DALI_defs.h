                                             /**************************************************************************************************
* File: Dali_defs.h
* File Type: C - Header File
* Project Name: DALI definitions
* Company: (c) mikroElektronika, 2012
* Revision History:
*       - initial release;
* Description:
*     This project contains definitions used by API functions for DALI protocol4.
*
* Target:
*       MCU:             STM32F407VG
*       Dev.Board:       EasyMx PRO v7 for STM32 ARM

*       Oscillator:      168MHz
*       SW:              mikroC PRO for ARM

* NOTES:
*     This project will be used for test Dali
**************************************************************************************************/

#define DALI_EXTENDED_COMMAND           0x03
#define DALI_COMMAND                    0x02
#define DALI_BACKCHANNEL                0x01

#define SHORT_ADDRESS                   0x00
#define GROUP_ADDRESS                   0x80

//Broadcast
#define BROADCAST_DIRECT                0xFE
#define BROADCAST_CMD                   0xFF

//for address byte
#define FOLLOWING_DIRECT_ARC_POWER_LVL  0x00
#define FOLLOWING_COMMAND               0x01


//Group addresses. Max 16 groups
#define GROUP01_DIRECT_ADDR             0x81
#define GROUP02_DIRECT_ADDR             0x83
#define GROUP03_DIRECT_ADDR             0x85
#define GROUP04_DIRECT_ADDR             0x87
#define GROUP05_DIRECT_ADDR             0x89
#define GROUP06_DIRECT_ADDR             0x8B
#define GROUP07_DIRECT_ADDR             0x8D
#define GROUP08_DIRECT_ADDR             0x8F
#define GROUP09_DIRECT_ADDR             0x91
#define GROUP10_DIRECT_ADDR             0x93
#define GROUP11_DIRECT_ADDR             0x95
#define GROUP12_DIRECT_ADDR             0x97
#define GROUP13_DIRECT_ADDR             0x99
#define GROUP14_DIRECT_ADDR             0x9B
#define GROUP15_DIRECT_ADDR             0x9D
#define GROUP16_DIRECT_ADDR             0x9F

//available addresses 0-63
#define ADDRESS01                       0x00
#define ADDRESS02                       0x01
#define ADDRESS03                       0x02
#define ADDRESS04                       0x03
#define ADDRESS05                       0x04
#define ADDRESS06                       0x05
#define ADDRESS07                       0x06
#define ADDRESS08                       0x07
#define ADDRESS09                       0x08
#define ADDRESS10                       0x09
#define ADDRESS11                       0x0A
#define ADDRESS12                       0x0B
#define ADDRESS13                       0x0C
#define ADDRESS14                       0x0D
#define ADDRESS15                       0x0E
#define ADDRESS16                       0x0F
#define ADDRESS17                       0x10
#define ADDRESS18                       0x11
#define ADDRESS19                       0x12
#define ADDRESS20                       0x13
#define ADDRESS21                       0x14
#define ADDRESS22                       0x15
#define ADDRESS23                       0x16
#define ADDRESS24                       0x17
#define ADDRESS25                       0x18
#define ADDRESS26                       0x19
#define ADDRESS27                       0x1A
#define ADDRESS28                       0x1B
#define ADDRESS29                       0x1C
#define ADDRESS30                       0x1D
#define ADDRESS31                       0x1E
#define ADDRESS32                       0x1F
#define ADDRESS33                       0x20
#define ADDRESS34                       0x21
#define ADDRESS35                       0x22
#define ADDRESS36                       0x23
#define ADDRESS37                       0x24
#define ADDRESS38                       0x25
#define ADDRESS39                       0x26
#define ADDRESS40                       0x27
#define ADDRESS41                       0x28
#define ADDRESS42                       0x29
#define ADDRESS43                       0x2A
#define ADDRESS44                       0x2B
#define ADDRESS45                       0x2C
#define ADDRESS46                       0x2D
#define ADDRESS47                       0x2E
#define ADDRESS48                       0x2F
#define ADDRESS49                       0x30
#define ADDRESS50                       0x31
#define ADDRESS51                       0x32
#define ADDRESS52                       0x33
#define ADDRESS53                       0x34
#define ADDRESS54                       0x35
#define ADDRESS55                       0x36
#define ADDRESS56                       0x37
#define ADDRESS57                       0x38
#define ADDRESS58                       0x39
#define ADDRESS59                       0x3A
#define ADDRESS60                       0x3B
#define ADDRESS61                       0x3C
#define ADDRESS62                       0x3D
#define ADDRESS63                       0x3E
#define ADDRESS64                       0x3F

/*****************************Commands*****************************************/
//Indirect Arc Power Commands

#define OFF                             0x00
#define UP                              0x01
#define DOWN                            0x02
#define STEP_UP                         0x03
#define STEP_DOWN                       0x04
#define RECALL_MAX_LEVEL                0x05
#define RECALL_MIN_LEVEL                0x06
#define STEP_DOWN_AND_OFF               0x07
#define ON_AND_STEP_UP                  0x08

#define GO_TO_SCENE01                   0x10
#define GO_TO_SCENE02                   0x11
#define GO_TO_SCENE03                   0x12
#define GO_TO_SCENE04                   0x13
#define GO_TO_SCENE05                   0x14
#define GO_TO_SCENE06                   0x15
#define GO_TO_SCENE07                   0x16
#define GO_TO_SCENE08                   0x17
#define GO_TO_SCENE09                   0x18
#define GO_TO_SCENE10                   0x19
#define GO_TO_SCENE11                   0x1A
#define GO_TO_SCENE12                   0x1B
#define GO_TO_SCENE13                   0x1C
#define GO_TO_SCENE14                   0x1D
#define GO_TO_SCENE15                   0x1E
#define GO_TO_SCENE16                   0x1F


//General Configuration Commands

#define RESET                           0x20
#define STORE_ACTUAL_LEVEL_IN_THE_DTR   0x21


//Arc Power Parameters Settings

#define STORE_THE_DTR_AS_MAX_LEVEL                 0x22
#define STORE_THE_DTR_AS_MIN_LEVEL                 0x23
#define STORE_THE_DTR_AS_SYSTEM_FAILURE_LEVEL      0x2C
#define STORE_THE_DTR_AS_POWER_ON_LEVEL            0x2D
#define STORE_THE_DTR_AS_FADE_TIME                 0x2E
#define STORE_THE_DTR_AS_FADE_RATE                 0x2F

#define STORE_THE_DTRAS_SCENE01                    0x40
#define STORE_THE_DTRAS_SCENE02                    0x41
#define STORE_THE_DTRAS_SCENE03                    0x42
#define STORE_THE_DTRAS_SCENE04                    0x43
#define STORE_THE_DTRAS_SCENE05                    0x44
#define STORE_THE_DTRAS_SCENE06                    0x45
#define STORE_THE_DTRAS_SCENE07                    0x46
#define STORE_THE_DTRAS_SCENE08                    0x47
#define STORE_THE_DTRAS_SCENE09                    0x48
#define STORE_THE_DTRAS_SCENE10                    0x49
#define STORE_THE_DTRAS_SCENE11                    0x4A
#define STORE_THE_DTRAS_SCENE12                    0x4B
#define STORE_THE_DTRAS_SCENE13                    0x4C
#define STORE_THE_DTRAS_SCENE14                    0x4D
#define STORE_THE_DTRAS_SCENE15                    0x4E
#define STORE_THE_DTRAS_SCENE16                    0x4F

//System Parameters Settings

#define REMOVE_FROM_SCENE01                        0x50
#define REMOVE_FROM_SCENE02                        0x51
#define REMOVE_FROM_SCENE03                        0x52
#define REMOVE_FROM_SCENE04                        0x53
#define REMOVE_FROM_SCENE05                        0x54
#define REMOVE_FROM_SCENE06                        0x55
#define REMOVE_FROM_SCENE07                        0x56
#define REMOVE_FROM_SCENE08                        0x57
#define REMOVE_FROM_SCENE09                        0x58
#define REMOVE_FROM_SCENE10                        0x59
#define REMOVE_FROM_SCENE11                        0x5A
#define REMOVE_FROM_SCENE12                        0x5B
#define REMOVE_FROM_SCENE13                        0x5C
#define REMOVE_FROM_SCENE14                        0x5D
#define REMOVE_FROM_SCENE15                        0x5E
#define REMOVE_FROM_SCENE16                        0x5F

#define ADD_TO_GROUP01                             0x60
#define ADD_TO_GROUP02                             0x61
#define ADD_TO_GROUP03                             0x62
#define ADD_TO_GROUP04                             0x63
#define ADD_TO_GROUP05                             0x64
#define ADD_TO_GROUP06                             0x65
#define ADD_TO_GROUP07                             0x66
#define ADD_TO_GROUP08                             0x67
#define ADD_TO_GROUP09                             0x68
#define ADD_TO_GROUP10                             0x69
#define ADD_TO_GROUP11                             0x6A
#define ADD_TO_GROUP12                             0x6B
#define ADD_TO_GROUP13                             0x6C
#define ADD_TO_GROUP14                             0x6D
#define ADD_TO_GROUP15                             0x6E
#define ADD_TO_GROUP16                             0x6F

#define REMOVE_FROM_GROUP01                        0x70
#define REMOVE_FROM_GROUP02                        0x71
#define REMOVE_FROM_GROUP03                        0x72
#define REMOVE_FROM_GROUP04                        0x73
#define REMOVE_FROM_GROUP05                        0x74
#define REMOVE_FROM_GROUP06                        0x75
#define REMOVE_FROM_GROUP07                        0x76
#define REMOVE_FROM_GROUP08                        0x77
#define REMOVE_FROM_GROUP09                        0x78
#define REMOVE_FROM_GROUP10                        0x79
#define REMOVE_FROM_GROUP11                        0x7A
#define REMOVE_FROM_GROUP12                        0x7B
#define REMOVE_FROM_GROUP13                        0x7C
#define REMOVE_FROM_GROUP14                        0x7D
#define REMOVE_FROM_GROUP15                        0x7E
#define REMOVE_FROM_GROUP16                        0x7F

#define STORE_DTR_AS_SHORT_ADDRESS                 0x80

//Status Information Commands

#define QUERY_STATUS                               0x90
#define QUERY_BALLAST                              0x91
#define QUERY_LAMP_FAILURE                         0x92
#define QUERY_LAMP_POWER_ON                        0x93
#define QUERY_LIMIT_ERROR                          0x94
#define QUERY_RESET_STATE                          0x95
#define QUERY_MISSING_SHORT_ADDRESS                0x96
#define QUERY_VERSION_NUMBER                       0x97
#define QUERY_CONTENT_DTR                          0x98
#define QUERY_DEVICE_TYPE                          0x99
#define QUERY_PHYSICAL_MINIMUM_LEVEL               0x9A
#define QUERY_POWER_FAILURE                        0x9B

//Arc Power Parameter Settings

#define QUERY_ACTUAL_LEVEL                         0xA0
#define QUERY_MAX_LEVEL                            0xA1
#define QUERY_MIN_LEVEL                            0xA2
#define QUERY_POWER_ON_LEVEL                       0xA3
#define QUERY_SYSTEM_FAILURE_LEVEL                 0xA4
#define QUERY_FADE_TIME_FADE_RATE                  0xA5

//Queries Related To The System Parameter Settings

#define QUERY_SCENE_LEVEL01                        0xB0
#define QUERY_SCENE_LEVEL02                        0xB1
#define QUERY_SCENE_LEVEL03                        0xB2
#define QUERY_SCENE_LEVEL04                        0xB3
#define QUERY_SCENE_LEVEL05                        0xB4
#define QUERY_SCENE_LEVEL06                        0xB5
#define QUERY_SCENE_LEVEL07                        0xB6
#define QUERY_SCENE_LEVEL08                        0xB7
#define QUERY_SCENE_LEVEL09                        0xB8
#define QUERY_SCENE_LEVEL10                        0xB9
#define QUERY_SCENE_LEVEL11                        0xBA
#define QUERY_SCENE_LEVEL12                        0xBB
#define QUERY_SCENE_LEVEL13                        0xBC
#define QUERY_SCENE_LEVEL14                        0xBD
#define QUERY_SCENE_LEVEL15                        0xBE
#define QUERY_SCENE_LEVEL16                        0xBF

#define QUERY_GROUPS_0_7                           0xC0
#define QUERY_GROUPS_8_15                          0xC1

#define QUERY_RANDOM_ADDRESS_H                     0xC2
#define QUERY_RANDOM_ADDERSS_M                     0xC3
#define QUERY_RANDOM_ADDRESS_L                     0xC4

//Special Commands

#define TERMINATE_H_BITS                           0xA1    //Address: 0xA100
#define TERMINATE_L_BITS                           0x00    //1010 0001 0000 0000

#define DTR                                        0xA3    //1010 0011 XXXX XXXX

#define INITIALISE                                 0xA5    //1010 0101 XXXX XXXX

#define INITIALISE_ALL_BALLAST_H                   0xA5
#define INITIALISE_ALL_BALLAST_L                   0x00

#define INITIALISE_BALLAST_WITHOUT_SHORT_ADDR_H    0xA5
#define INITIALISE_BALLAST_WITHOUT_SHORT_ADDR_L    0xFF

#define RANDOMISE_H                                0xA7
#define RANDOMISE_L                                0x00

#define COMPARE_H                                  0xA9
#define COMPARE_L                                  0x00

#define WITHDRAW_H                                 0xAB
#define WITHDRAW_L                                 0x00

#define SEARCHADDRH                                0xB1    //1011 0001 XXXX XXXX
#define SEARCHADDRM                                0xB3    //1011 0011 XXXX XXXX
#define SEARCHADDRL                                0xB5    //1011 1001 XXXX XXXX

#define PROGRAM_SHORT_ADDRESS                      0xB7    //1011 0111 0AAA AAA1
#define VERIFY_SHORT_ADDRESS                       0xB9    //1011 1001 0AAA AAA1

#define QUERY_SHORT_ADDRESS_H                      0xBB    //1011 1011 0000 0000
#define QUERY_SHORT_ADDRESS_L                      0x00

#define PHYSICAL_SELECTION_H                       0xBD    //1011 1101 0000 0000
#define PHYSICAL_SELECTION_L                       0x00

#define ENABLE_STANDARD_DEVICE_H                   0xC1    //1100 0001 0000 0000
#define ENABLE_STANDARD_DEVICE_L                   0x00

#define ENABLE_EMERGENCY_LIGHTS_H                  0xC1    //1100 0001 0000 0001
#define ENABLE_EMERGENCY_LIGHTS_L                  0x01

#define ENABLE_HID_LAMPS_H                         0xC1    //1100 0001 0000 0010
#define ENABLE_HID_LAMPS_L                         0x02

#define ENABLE_LOW_V_HALOGEN_LAMPS_H               0xC1    //1100 0001 0000 0011
#define ENABLE_LOW_V_HALOGEN_LAMPS_L               0x03

#define ENABLE_DIM_INCANDESCENT_LAMPS_H            0xC1    //1100 0001 0000 0100
#define ENABLE_DIM_INCANDESCENT_LAMPS_L            0x04

#define ENABLE_LED_MODULES_H                       0xC1    //1100 0001 0000 0110
#define ENABLE_LED_MODULES_L                       0x00
/*
        0          Fluorescent lamps IEC 62386-201
        1          Self-contained emergency lighting IEC 62386-202
        2          Discharge lamps (excluding fluorescent lamps) IEC 62386-203
        3          Low voltage halogen lamps IEC 62386-204
        4          Supply Voltage controller for incandescent lamps IEC 62386-205
        5          Conversion from digital into D.C. voltage IEC 62386-206
        6          LED modules IEC 62386-207
        7          Switching function IEC 62386-208
        8          Colour control IEC 62386-209
        9          Sequencer IEC 62386-210
        10         Optical control IEC 62386-211
        11  - 127  Not yet defined
        128 - 254  Reserved for control devices
        255        Control gear supports more than one device type
*/
