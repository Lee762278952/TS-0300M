/*
 * File:           E:\Work\Work_2019\Work_20191017_第三代会议主机API（1061+DSP）\第三代系统资料\TS-0300M-ADAU1466-20200409\default-20191231\ADAU1466-1\adau1466_1_IC_1_REG.h
 *
 * Created:        Thursday, April 09, 2020 11:02:13 AM
 * Description:    20191108:IC 1 control register definitions.
 *
 * This software is distributed in the hope that it will be useful,
 * but is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * This software may only be used to program products purchased from
 * Analog Devices for incorporation by you into audio products that
 * are intended for resale to audio product end users. This software
 * may not be distributed whole or in any part to third parties.
 *
 * Copyright ©2020 Analog Devices, Inc. All rights reserved.
 */
#ifndef __ADAU1466_1_IC_1_REG_H__
#define __ADAU1466_1_IC_1_REG_H__


/* PLL_CTRL0  - Registers (IC 1) */
#define DSP1_REG_PLL_CTRL0_IC_1_ADDR                   0xF000
#define DSP1_REG_PLL_CTRL0_IC_1_BYTE                   2
#define DSP1_REG_PLL_CTRL0_IC_1_VALUE                  0x60

/* PLL_CTRL1  - Registers (IC 1) */
#define DSP1_REG_PLL_CTRL1_IC_1_ADDR                   0xF001
#define DSP1_REG_PLL_CTRL1_IC_1_BYTE                   2
#define DSP1_REG_PLL_CTRL1_IC_1_VALUE                  0x2

/* PLL_CLK_SRC  - Registers (IC 1) */
#define DSP1_REG_PLL_CLK_SRC_IC_1_ADDR                 0xF002
#define DSP1_REG_PLL_CLK_SRC_IC_1_BYTE                 2
#define DSP1_REG_PLL_CLK_SRC_IC_1_VALUE                0x1

/* PLL_ENABLE  - Registers (IC 1) */
#define DSP1_REG_PLL_ENABLE_IC_1_ADDR                  0xF003
#define DSP1_REG_PLL_ENABLE_IC_1_BYTE                  2
#define DSP1_REG_PLL_ENABLE_IC_1_VALUE                 0x1

/* PLL_LOCK  - Registers (IC 1) */
#define DSP1_REG_PLL_LOCK_IC_1_ADDR                    0xF004
#define DSP1_REG_PLL_LOCK_IC_1_BYTE                    0
#define DSP1_REG_PLL_LOCK_IC_1_VALUE                   0x0

/* MCLK_OUT  - Registers (IC 1) */
#define DSP1_REG_MCLK_OUT_IC_1_ADDR                    0xF005
#define DSP1_REG_MCLK_OUT_IC_1_BYTE                    2
#define DSP1_REG_MCLK_OUT_IC_1_VALUE                   0x5

/* PLL_WATCHDOG  - Registers (IC 1) */
#define DSP1_REG_PLL_WATCHDOG_IC_1_ADDR                0xF006
#define DSP1_REG_PLL_WATCHDOG_IC_1_BYTE                2
#define DSP1_REG_PLL_WATCHDOG_IC_1_VALUE               0x1

/* CLK_GEN1_M  - Registers (IC 1) */
#define DSP1_REG_CLK_GEN1_M_IC_1_ADDR                  0xF020
#define DSP1_REG_CLK_GEN1_M_IC_1_BYTE                  2
#define DSP1_REG_CLK_GEN1_M_IC_1_VALUE                 0x6

/* CLK_GEN1_N  - Registers (IC 1) */
#define DSP1_REG_CLK_GEN1_N_IC_1_ADDR                  0xF021
#define DSP1_REG_CLK_GEN1_N_IC_1_BYTE                  2
#define DSP1_REG_CLK_GEN1_N_IC_1_VALUE                 0x1

/* CLK_GEN2_M  - Registers (IC 1) */
#define DSP1_REG_CLK_GEN2_M_IC_1_ADDR                  0xF022
#define DSP1_REG_CLK_GEN2_M_IC_1_BYTE                  2
#define DSP1_REG_CLK_GEN2_M_IC_1_VALUE                 0x6

/* CLK_GEN2_N  - Registers (IC 1) */
#define DSP1_REG_CLK_GEN2_N_IC_1_ADDR                  0xF023
#define DSP1_REG_CLK_GEN2_N_IC_1_BYTE                  2
#define DSP1_REG_CLK_GEN2_N_IC_1_VALUE                 0x1

/* CLK_GEN3_M  - Registers (IC 1) */
#define DSP1_REG_CLK_GEN3_M_IC_1_ADDR                  0xF024
#define DSP1_REG_CLK_GEN3_M_IC_1_BYTE                  2
#define DSP1_REG_CLK_GEN3_M_IC_1_VALUE                 0x6

/* CLK_GEN3_N  - Registers (IC 1) */
#define DSP1_REG_CLK_GEN3_N_IC_1_ADDR                  0xF025
#define DSP1_REG_CLK_GEN3_N_IC_1_BYTE                  2
#define DSP1_REG_CLK_GEN3_N_IC_1_VALUE                 0x1

/* CLK_GEN3_SRC  - Registers (IC 1) */
#define DSP1_REG_CLK_GEN3_SRC_IC_1_ADDR                0xF026
#define DSP1_REG_CLK_GEN3_SRC_IC_1_BYTE                2
#define DSP1_REG_CLK_GEN3_SRC_IC_1_VALUE               0xE

/* CLK_GEN3_LOCK  - Registers (IC 1) */
#define DSP1_REG_CLK_GEN3_LOCK_IC_1_ADDR               0xF027
#define DSP1_REG_CLK_GEN3_LOCK_IC_1_BYTE               0
#define DSP1_REG_CLK_GEN3_LOCK_IC_1_VALUE              0x0

/* POWER_ENABLE0  - Registers (IC 1) */
#define DSP1_REG_POWER_ENABLE0_IC_1_ADDR               0xF050
#define DSP1_REG_POWER_ENABLE0_IC_1_BYTE               2
#define DSP1_REG_POWER_ENABLE0_IC_1_VALUE              0x1FFF

/* POWER_ENABLE1  - Registers (IC 1) */
#define DSP1_REG_POWER_ENABLE1_IC_1_ADDR               0xF051
#define DSP1_REG_POWER_ENABLE1_IC_1_BYTE               2
#define DSP1_REG_POWER_ENABLE1_IC_1_VALUE              0x1F

/* ASRC_INPUT0  - Registers (IC 1) */
#define DSP1_REG_ASRC_INPUT0_IC_1_ADDR                 0xF100
#define DSP1_REG_ASRC_INPUT0_IC_1_BYTE                 2
#define DSP1_REG_ASRC_INPUT0_IC_1_VALUE                0x0

/* ASRC_INPUT1  - Registers (IC 1) */
#define DSP1_REG_ASRC_INPUT1_IC_1_ADDR                 0xF101
#define DSP1_REG_ASRC_INPUT1_IC_1_BYTE                 2
#define DSP1_REG_ASRC_INPUT1_IC_1_VALUE                0x0

/* ASRC_INPUT2  - Registers (IC 1) */
#define DSP1_REG_ASRC_INPUT2_IC_1_ADDR                 0xF102
#define DSP1_REG_ASRC_INPUT2_IC_1_BYTE                 2
#define DSP1_REG_ASRC_INPUT2_IC_1_VALUE                0x0

/* ASRC_INPUT3  - Registers (IC 1) */
#define DSP1_REG_ASRC_INPUT3_IC_1_ADDR                 0xF103
#define DSP1_REG_ASRC_INPUT3_IC_1_BYTE                 2
#define DSP1_REG_ASRC_INPUT3_IC_1_VALUE                0x0

/* ASRC_INPUT4  - Registers (IC 1) */
#define DSP1_REG_ASRC_INPUT4_IC_1_ADDR                 0xF104
#define DSP1_REG_ASRC_INPUT4_IC_1_BYTE                 2
#define DSP1_REG_ASRC_INPUT4_IC_1_VALUE                0x0

/* ASRC_INPUT5  - Registers (IC 1) */
#define DSP1_REG_ASRC_INPUT5_IC_1_ADDR                 0xF105
#define DSP1_REG_ASRC_INPUT5_IC_1_BYTE                 2
#define DSP1_REG_ASRC_INPUT5_IC_1_VALUE                0x0

/* ASRC_INPUT6  - Registers (IC 1) */
#define DSP1_REG_ASRC_INPUT6_IC_1_ADDR                 0xF106
#define DSP1_REG_ASRC_INPUT6_IC_1_BYTE                 2
#define DSP1_REG_ASRC_INPUT6_IC_1_VALUE                0x0

/* ASRC_INPUT7  - Registers (IC 1) */
#define DSP1_REG_ASRC_INPUT7_IC_1_ADDR                 0xF107
#define DSP1_REG_ASRC_INPUT7_IC_1_BYTE                 2
#define DSP1_REG_ASRC_INPUT7_IC_1_VALUE                0x0

/* ASRC_OUT_RATE0  - Registers (IC 1) */
#define DSP1_REG_ASRC_OUT_RATE0_IC_1_ADDR              0xF140
#define DSP1_REG_ASRC_OUT_RATE0_IC_1_BYTE              2
#define DSP1_REG_ASRC_OUT_RATE0_IC_1_VALUE             0x0

/* ASRC_OUT_RATE1  - Registers (IC 1) */
#define DSP1_REG_ASRC_OUT_RATE1_IC_1_ADDR              0xF141
#define DSP1_REG_ASRC_OUT_RATE1_IC_1_BYTE              2
#define DSP1_REG_ASRC_OUT_RATE1_IC_1_VALUE             0x0

/* ASRC_OUT_RATE2  - Registers (IC 1) */
#define DSP1_REG_ASRC_OUT_RATE2_IC_1_ADDR              0xF142
#define DSP1_REG_ASRC_OUT_RATE2_IC_1_BYTE              2
#define DSP1_REG_ASRC_OUT_RATE2_IC_1_VALUE             0x0

/* ASRC_OUT_RATE3  - Registers (IC 1) */
#define DSP1_REG_ASRC_OUT_RATE3_IC_1_ADDR              0xF143
#define DSP1_REG_ASRC_OUT_RATE3_IC_1_BYTE              2
#define DSP1_REG_ASRC_OUT_RATE3_IC_1_VALUE             0x0

/* ASRC_OUT_RATE4  - Registers (IC 1) */
#define DSP1_REG_ASRC_OUT_RATE4_IC_1_ADDR              0xF144
#define DSP1_REG_ASRC_OUT_RATE4_IC_1_BYTE              2
#define DSP1_REG_ASRC_OUT_RATE4_IC_1_VALUE             0x0

/* ASRC_OUT_RATE5  - Registers (IC 1) */
#define DSP1_REG_ASRC_OUT_RATE5_IC_1_ADDR              0xF145
#define DSP1_REG_ASRC_OUT_RATE5_IC_1_BYTE              2
#define DSP1_REG_ASRC_OUT_RATE5_IC_1_VALUE             0x0

/* ASRC_OUT_RATE6  - Registers (IC 1) */
#define DSP1_REG_ASRC_OUT_RATE6_IC_1_ADDR              0xF146
#define DSP1_REG_ASRC_OUT_RATE6_IC_1_BYTE              2
#define DSP1_REG_ASRC_OUT_RATE6_IC_1_VALUE             0x0

/* ASRC_OUT_RATE7  - Registers (IC 1) */
#define DSP1_REG_ASRC_OUT_RATE7_IC_1_ADDR              0xF147
#define DSP1_REG_ASRC_OUT_RATE7_IC_1_BYTE              2
#define DSP1_REG_ASRC_OUT_RATE7_IC_1_VALUE             0x0

/* SOUT_SOURCE0  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE0_IC_1_ADDR                0xF180
#define DSP1_REG_SOUT_SOURCE0_IC_1_BYTE                2
#define DSP1_REG_SOUT_SOURCE0_IC_1_VALUE               0x2

/* SOUT_SOURCE1  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE1_IC_1_ADDR                0xF181
#define DSP1_REG_SOUT_SOURCE1_IC_1_BYTE                2
#define DSP1_REG_SOUT_SOURCE1_IC_1_VALUE               0x2

/* SOUT_SOURCE2  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE2_IC_1_ADDR                0xF182
#define DSP1_REG_SOUT_SOURCE2_IC_1_BYTE                2
#define DSP1_REG_SOUT_SOURCE2_IC_1_VALUE               0x2

/* SOUT_SOURCE3  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE3_IC_1_ADDR                0xF183
#define DSP1_REG_SOUT_SOURCE3_IC_1_BYTE                2
#define DSP1_REG_SOUT_SOURCE3_IC_1_VALUE               0x2

/* SOUT_SOURCE4  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE4_IC_1_ADDR                0xF184
#define DSP1_REG_SOUT_SOURCE4_IC_1_BYTE                2
#define DSP1_REG_SOUT_SOURCE4_IC_1_VALUE               0x2

/* SOUT_SOURCE5  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE5_IC_1_ADDR                0xF185
#define DSP1_REG_SOUT_SOURCE5_IC_1_BYTE                2
#define DSP1_REG_SOUT_SOURCE5_IC_1_VALUE               0x2

/* SOUT_SOURCE6  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE6_IC_1_ADDR                0xF186
#define DSP1_REG_SOUT_SOURCE6_IC_1_BYTE                2
#define DSP1_REG_SOUT_SOURCE6_IC_1_VALUE               0x2

/* SOUT_SOURCE7  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE7_IC_1_ADDR                0xF187
#define DSP1_REG_SOUT_SOURCE7_IC_1_BYTE                2
#define DSP1_REG_SOUT_SOURCE7_IC_1_VALUE               0x2

/* SOUT_SOURCE8  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE8_IC_1_ADDR                0xF188
#define DSP1_REG_SOUT_SOURCE8_IC_1_BYTE                2
#define DSP1_REG_SOUT_SOURCE8_IC_1_VALUE               0x2

/* SOUT_SOURCE9  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE9_IC_1_ADDR                0xF189
#define DSP1_REG_SOUT_SOURCE9_IC_1_BYTE                2
#define DSP1_REG_SOUT_SOURCE9_IC_1_VALUE               0x2

/* SOUT_SOURCE10  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE10_IC_1_ADDR               0xF18A
#define DSP1_REG_SOUT_SOURCE10_IC_1_BYTE               2
#define DSP1_REG_SOUT_SOURCE10_IC_1_VALUE              0x2

/* SOUT_SOURCE11  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE11_IC_1_ADDR               0xF18B
#define DSP1_REG_SOUT_SOURCE11_IC_1_BYTE               2
#define DSP1_REG_SOUT_SOURCE11_IC_1_VALUE              0x2

/* SOUT_SOURCE12  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE12_IC_1_ADDR               0xF18C
#define DSP1_REG_SOUT_SOURCE12_IC_1_BYTE               2
#define DSP1_REG_SOUT_SOURCE12_IC_1_VALUE              0x2

/* SOUT_SOURCE13  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE13_IC_1_ADDR               0xF18D
#define DSP1_REG_SOUT_SOURCE13_IC_1_BYTE               2
#define DSP1_REG_SOUT_SOURCE13_IC_1_VALUE              0x2

/* SOUT_SOURCE14  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE14_IC_1_ADDR               0xF18E
#define DSP1_REG_SOUT_SOURCE14_IC_1_BYTE               2
#define DSP1_REG_SOUT_SOURCE14_IC_1_VALUE              0x2

/* SOUT_SOURCE15  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE15_IC_1_ADDR               0xF18F
#define DSP1_REG_SOUT_SOURCE15_IC_1_BYTE               2
#define DSP1_REG_SOUT_SOURCE15_IC_1_VALUE              0x2

/* SOUT_SOURCE16  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE16_IC_1_ADDR               0xF190
#define DSP1_REG_SOUT_SOURCE16_IC_1_BYTE               2
#define DSP1_REG_SOUT_SOURCE16_IC_1_VALUE              0x2

/* SOUT_SOURCE17  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE17_IC_1_ADDR               0xF191
#define DSP1_REG_SOUT_SOURCE17_IC_1_BYTE               2
#define DSP1_REG_SOUT_SOURCE17_IC_1_VALUE              0x2

/* SOUT_SOURCE18  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE18_IC_1_ADDR               0xF192
#define DSP1_REG_SOUT_SOURCE18_IC_1_BYTE               2
#define DSP1_REG_SOUT_SOURCE18_IC_1_VALUE              0x2

/* SOUT_SOURCE19  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE19_IC_1_ADDR               0xF193
#define DSP1_REG_SOUT_SOURCE19_IC_1_BYTE               2
#define DSP1_REG_SOUT_SOURCE19_IC_1_VALUE              0x2

/* SOUT_SOURCE20  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE20_IC_1_ADDR               0xF194
#define DSP1_REG_SOUT_SOURCE20_IC_1_BYTE               2
#define DSP1_REG_SOUT_SOURCE20_IC_1_VALUE              0x2

/* SOUT_SOURCE21  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE21_IC_1_ADDR               0xF195
#define DSP1_REG_SOUT_SOURCE21_IC_1_BYTE               2
#define DSP1_REG_SOUT_SOURCE21_IC_1_VALUE              0x2

/* SOUT_SOURCE22  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE22_IC_1_ADDR               0xF196
#define DSP1_REG_SOUT_SOURCE22_IC_1_BYTE               2
#define DSP1_REG_SOUT_SOURCE22_IC_1_VALUE              0x2

/* SOUT_SOURCE23  - Registers (IC 1) */
#define DSP1_REG_SOUT_SOURCE23_IC_1_ADDR               0xF197
#define DSP1_REG_SOUT_SOURCE23_IC_1_BYTE               2
#define DSP1_REG_SOUT_SOURCE23_IC_1_VALUE              0x2

/* SPDIFTX_INPUT  - Registers (IC 1) */
#define DSP1_REG_SPDIFTX_INPUT_IC_1_ADDR               0xF1C0
#define DSP1_REG_SPDIFTX_INPUT_IC_1_BYTE               2
#define DSP1_REG_SPDIFTX_INPUT_IC_1_VALUE              0x0

/* SERIAL_BYTE_0_0  - Registers (IC 1) */
#define DSP1_REG_SERIAL_BYTE_0_0_IC_1_ADDR             0xF200
#define DSP1_REG_SERIAL_BYTE_0_0_IC_1_BYTE             2
#define DSP1_REG_SERIAL_BYTE_0_0_IC_1_VALUE            0x93AB

/* SERIAL_BYTE_0_1  - Registers (IC 1) */
#define DSP1_REG_SERIAL_BYTE_0_1_IC_1_ADDR             0xF201
#define DSP1_REG_SERIAL_BYTE_0_1_IC_1_BYTE             2
#define DSP1_REG_SERIAL_BYTE_0_1_IC_1_VALUE            0x2

/* SERIAL_BYTE_1_0  - Registers (IC 1) */
#define DSP1_REG_SERIAL_BYTE_1_0_IC_1_ADDR             0xF204
#define DSP1_REG_SERIAL_BYTE_1_0_IC_1_BYTE             2
#define DSP1_REG_SERIAL_BYTE_1_0_IC_1_VALUE            0x93AB

/* SERIAL_BYTE_1_1  - Registers (IC 1) */
#define DSP1_REG_SERIAL_BYTE_1_1_IC_1_ADDR             0xF205
#define DSP1_REG_SERIAL_BYTE_1_1_IC_1_BYTE             2
#define DSP1_REG_SERIAL_BYTE_1_1_IC_1_VALUE            0x2

/* SERIAL_BYTE_2_0  - Registers (IC 1) */
#define DSP1_REG_SERIAL_BYTE_2_0_IC_1_ADDR             0xF208
#define DSP1_REG_SERIAL_BYTE_2_0_IC_1_BYTE             2
#define DSP1_REG_SERIAL_BYTE_2_0_IC_1_VALUE            0x93AA

/* SERIAL_BYTE_2_1  - Registers (IC 1) */
#define DSP1_REG_SERIAL_BYTE_2_1_IC_1_ADDR             0xF209
#define DSP1_REG_SERIAL_BYTE_2_1_IC_1_BYTE             2
#define DSP1_REG_SERIAL_BYTE_2_1_IC_1_VALUE            0x2

/* SERIAL_BYTE_3_0  - Registers (IC 1) */
#define DSP1_REG_SERIAL_BYTE_3_0_IC_1_ADDR             0xF20C
#define DSP1_REG_SERIAL_BYTE_3_0_IC_1_BYTE             2
#define DSP1_REG_SERIAL_BYTE_3_0_IC_1_VALUE            0x9322

/* SERIAL_BYTE_3_1  - Registers (IC 1) */
#define DSP1_REG_SERIAL_BYTE_3_1_IC_1_ADDR             0xF20D
#define DSP1_REG_SERIAL_BYTE_3_1_IC_1_BYTE             2
#define DSP1_REG_SERIAL_BYTE_3_1_IC_1_VALUE            0x2

/* SERIAL_BYTE_4_0  - Registers (IC 1) */
#define DSP1_REG_SERIAL_BYTE_4_0_IC_1_ADDR             0xF210
#define DSP1_REG_SERIAL_BYTE_4_0_IC_1_BYTE             2
#define DSP1_REG_SERIAL_BYTE_4_0_IC_1_VALUE            0x9323

/* SERIAL_BYTE_4_1  - Registers (IC 1) */
#define DSP1_REG_SERIAL_BYTE_4_1_IC_1_ADDR             0xF211
#define DSP1_REG_SERIAL_BYTE_4_1_IC_1_BYTE             2
#define DSP1_REG_SERIAL_BYTE_4_1_IC_1_VALUE            0x2

/* SERIAL_BYTE_5_0  - Registers (IC 1) */
#define DSP1_REG_SERIAL_BYTE_5_0_IC_1_ADDR             0xF214
#define DSP1_REG_SERIAL_BYTE_5_0_IC_1_BYTE             2
#define DSP1_REG_SERIAL_BYTE_5_0_IC_1_VALUE            0x27AB

/* SERIAL_BYTE_5_1  - Registers (IC 1) */
#define DSP1_REG_SERIAL_BYTE_5_1_IC_1_ADDR             0xF215
#define DSP1_REG_SERIAL_BYTE_5_1_IC_1_BYTE             2
#define DSP1_REG_SERIAL_BYTE_5_1_IC_1_VALUE            0x2

/* SERIAL_BYTE_6_0  - Registers (IC 1) */
#define DSP1_REG_SERIAL_BYTE_6_0_IC_1_ADDR             0xF218
#define DSP1_REG_SERIAL_BYTE_6_0_IC_1_BYTE             2
#define DSP1_REG_SERIAL_BYTE_6_0_IC_1_VALUE            0x4BAA

/* SERIAL_BYTE_6_1  - Registers (IC 1) */
#define DSP1_REG_SERIAL_BYTE_6_1_IC_1_ADDR             0xF219
#define DSP1_REG_SERIAL_BYTE_6_1_IC_1_BYTE             2
#define DSP1_REG_SERIAL_BYTE_6_1_IC_1_VALUE            0x2

/* SERIAL_BYTE_7_0  - Registers (IC 1) */
#define DSP1_REG_SERIAL_BYTE_7_0_IC_1_ADDR             0xF21C
#define DSP1_REG_SERIAL_BYTE_7_0_IC_1_BYTE             2
#define DSP1_REG_SERIAL_BYTE_7_0_IC_1_VALUE            0x9322

/* SERIAL_BYTE_7_1  - Registers (IC 1) */
#define DSP1_REG_SERIAL_BYTE_7_1_IC_1_ADDR             0xF21D
#define DSP1_REG_SERIAL_BYTE_7_1_IC_1_BYTE             2
#define DSP1_REG_SERIAL_BYTE_7_1_IC_1_VALUE            0x2

/* FTDM_IN0  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN0_IC_1_ADDR                    0xF300
#define DSP1_REG_FTDM_IN0_IC_1_BYTE                    2
#define DSP1_REG_FTDM_IN0_IC_1_VALUE                   0x0

/* FTDM_IN1  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN1_IC_1_ADDR                    0xF301
#define DSP1_REG_FTDM_IN1_IC_1_BYTE                    2
#define DSP1_REG_FTDM_IN1_IC_1_VALUE                   0x0

/* FTDM_IN2  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN2_IC_1_ADDR                    0xF302
#define DSP1_REG_FTDM_IN2_IC_1_BYTE                    2
#define DSP1_REG_FTDM_IN2_IC_1_VALUE                   0x0

/* FTDM_IN3  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN3_IC_1_ADDR                    0xF303
#define DSP1_REG_FTDM_IN3_IC_1_BYTE                    2
#define DSP1_REG_FTDM_IN3_IC_1_VALUE                   0x0

/* FTDM_IN4  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN4_IC_1_ADDR                    0xF304
#define DSP1_REG_FTDM_IN4_IC_1_BYTE                    2
#define DSP1_REG_FTDM_IN4_IC_1_VALUE                   0x0

/* FTDM_IN5  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN5_IC_1_ADDR                    0xF305
#define DSP1_REG_FTDM_IN5_IC_1_BYTE                    2
#define DSP1_REG_FTDM_IN5_IC_1_VALUE                   0x0

/* FTDM_IN6  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN6_IC_1_ADDR                    0xF306
#define DSP1_REG_FTDM_IN6_IC_1_BYTE                    2
#define DSP1_REG_FTDM_IN6_IC_1_VALUE                   0x0

/* FTDM_IN7  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN7_IC_1_ADDR                    0xF307
#define DSP1_REG_FTDM_IN7_IC_1_BYTE                    2
#define DSP1_REG_FTDM_IN7_IC_1_VALUE                   0x0

/* FTDM_IN8  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN8_IC_1_ADDR                    0xF308
#define DSP1_REG_FTDM_IN8_IC_1_BYTE                    2
#define DSP1_REG_FTDM_IN8_IC_1_VALUE                   0x0

/* FTDM_IN9  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN9_IC_1_ADDR                    0xF309
#define DSP1_REG_FTDM_IN9_IC_1_BYTE                    2
#define DSP1_REG_FTDM_IN9_IC_1_VALUE                   0x0

/* FTDM_IN10  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN10_IC_1_ADDR                   0xF30A
#define DSP1_REG_FTDM_IN10_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN10_IC_1_VALUE                  0x0

/* FTDM_IN11  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN11_IC_1_ADDR                   0xF30B
#define DSP1_REG_FTDM_IN11_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN11_IC_1_VALUE                  0x0

/* FTDM_IN12  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN12_IC_1_ADDR                   0xF30C
#define DSP1_REG_FTDM_IN12_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN12_IC_1_VALUE                  0x0

/* FTDM_IN13  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN13_IC_1_ADDR                   0xF30D
#define DSP1_REG_FTDM_IN13_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN13_IC_1_VALUE                  0x0

/* FTDM_IN14  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN14_IC_1_ADDR                   0xF30E
#define DSP1_REG_FTDM_IN14_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN14_IC_1_VALUE                  0x0

/* FTDM_IN15  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN15_IC_1_ADDR                   0xF30F
#define DSP1_REG_FTDM_IN15_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN15_IC_1_VALUE                  0x0

/* FTDM_IN16  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN16_IC_1_ADDR                   0xF310
#define DSP1_REG_FTDM_IN16_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN16_IC_1_VALUE                  0x0

/* FTDM_IN17  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN17_IC_1_ADDR                   0xF311
#define DSP1_REG_FTDM_IN17_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN17_IC_1_VALUE                  0x0

/* FTDM_IN18  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN18_IC_1_ADDR                   0xF312
#define DSP1_REG_FTDM_IN18_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN18_IC_1_VALUE                  0x0

/* FTDM_IN19  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN19_IC_1_ADDR                   0xF313
#define DSP1_REG_FTDM_IN19_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN19_IC_1_VALUE                  0x0

/* FTDM_IN20  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN20_IC_1_ADDR                   0xF314
#define DSP1_REG_FTDM_IN20_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN20_IC_1_VALUE                  0x0

/* FTDM_IN21  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN21_IC_1_ADDR                   0xF315
#define DSP1_REG_FTDM_IN21_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN21_IC_1_VALUE                  0x0

/* FTDM_IN22  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN22_IC_1_ADDR                   0xF316
#define DSP1_REG_FTDM_IN22_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN22_IC_1_VALUE                  0x0

/* FTDM_IN23  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN23_IC_1_ADDR                   0xF317
#define DSP1_REG_FTDM_IN23_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN23_IC_1_VALUE                  0x0

/* FTDM_IN24  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN24_IC_1_ADDR                   0xF318
#define DSP1_REG_FTDM_IN24_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN24_IC_1_VALUE                  0x0

/* FTDM_IN25  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN25_IC_1_ADDR                   0xF319
#define DSP1_REG_FTDM_IN25_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN25_IC_1_VALUE                  0x0

/* FTDM_IN26  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN26_IC_1_ADDR                   0xF31A
#define DSP1_REG_FTDM_IN26_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN26_IC_1_VALUE                  0x0

/* FTDM_IN27  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN27_IC_1_ADDR                   0xF31B
#define DSP1_REG_FTDM_IN27_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN27_IC_1_VALUE                  0x0

/* FTDM_IN28  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN28_IC_1_ADDR                   0xF31C
#define DSP1_REG_FTDM_IN28_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN28_IC_1_VALUE                  0x0

/* FTDM_IN29  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN29_IC_1_ADDR                   0xF31D
#define DSP1_REG_FTDM_IN29_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN29_IC_1_VALUE                  0x0

/* FTDM_IN30  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN30_IC_1_ADDR                   0xF31E
#define DSP1_REG_FTDM_IN30_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN30_IC_1_VALUE                  0x0

/* FTDM_IN31  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN31_IC_1_ADDR                   0xF31F
#define DSP1_REG_FTDM_IN31_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN31_IC_1_VALUE                  0x0

/* FTDM_IN32  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN32_IC_1_ADDR                   0xF320
#define DSP1_REG_FTDM_IN32_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN32_IC_1_VALUE                  0x0

/* FTDM_IN33  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN33_IC_1_ADDR                   0xF321
#define DSP1_REG_FTDM_IN33_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN33_IC_1_VALUE                  0x0

/* FTDM_IN34  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN34_IC_1_ADDR                   0xF322
#define DSP1_REG_FTDM_IN34_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN34_IC_1_VALUE                  0x0

/* FTDM_IN35  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN35_IC_1_ADDR                   0xF323
#define DSP1_REG_FTDM_IN35_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN35_IC_1_VALUE                  0x0

/* FTDM_IN36  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN36_IC_1_ADDR                   0xF324
#define DSP1_REG_FTDM_IN36_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN36_IC_1_VALUE                  0x0

/* FTDM_IN37  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN37_IC_1_ADDR                   0xF325
#define DSP1_REG_FTDM_IN37_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN37_IC_1_VALUE                  0x0

/* FTDM_IN38  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN38_IC_1_ADDR                   0xF326
#define DSP1_REG_FTDM_IN38_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN38_IC_1_VALUE                  0x0

/* FTDM_IN39  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN39_IC_1_ADDR                   0xF327
#define DSP1_REG_FTDM_IN39_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN39_IC_1_VALUE                  0x0

/* FTDM_IN40  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN40_IC_1_ADDR                   0xF328
#define DSP1_REG_FTDM_IN40_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN40_IC_1_VALUE                  0x0

/* FTDM_IN41  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN41_IC_1_ADDR                   0xF329
#define DSP1_REG_FTDM_IN41_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN41_IC_1_VALUE                  0x0

/* FTDM_IN42  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN42_IC_1_ADDR                   0xF32A
#define DSP1_REG_FTDM_IN42_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN42_IC_1_VALUE                  0x0

/* FTDM_IN43  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN43_IC_1_ADDR                   0xF32B
#define DSP1_REG_FTDM_IN43_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN43_IC_1_VALUE                  0x0

/* FTDM_IN44  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN44_IC_1_ADDR                   0xF32C
#define DSP1_REG_FTDM_IN44_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN44_IC_1_VALUE                  0x0

/* FTDM_IN45  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN45_IC_1_ADDR                   0xF32D
#define DSP1_REG_FTDM_IN45_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN45_IC_1_VALUE                  0x0

/* FTDM_IN46  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN46_IC_1_ADDR                   0xF32E
#define DSP1_REG_FTDM_IN46_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN46_IC_1_VALUE                  0x0

/* FTDM_IN47  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN47_IC_1_ADDR                   0xF32F
#define DSP1_REG_FTDM_IN47_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN47_IC_1_VALUE                  0x0

/* FTDM_IN48  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN48_IC_1_ADDR                   0xF330
#define DSP1_REG_FTDM_IN48_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN48_IC_1_VALUE                  0x0

/* FTDM_IN49  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN49_IC_1_ADDR                   0xF331
#define DSP1_REG_FTDM_IN49_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN49_IC_1_VALUE                  0x0

/* FTDM_IN50  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN50_IC_1_ADDR                   0xF332
#define DSP1_REG_FTDM_IN50_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN50_IC_1_VALUE                  0x0

/* FTDM_IN51  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN51_IC_1_ADDR                   0xF333
#define DSP1_REG_FTDM_IN51_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN51_IC_1_VALUE                  0x0

/* FTDM_IN52  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN52_IC_1_ADDR                   0xF334
#define DSP1_REG_FTDM_IN52_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN52_IC_1_VALUE                  0x0

/* FTDM_IN53  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN53_IC_1_ADDR                   0xF335
#define DSP1_REG_FTDM_IN53_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN53_IC_1_VALUE                  0x0

/* FTDM_IN54  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN54_IC_1_ADDR                   0xF336
#define DSP1_REG_FTDM_IN54_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN54_IC_1_VALUE                  0x0

/* FTDM_IN55  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN55_IC_1_ADDR                   0xF337
#define DSP1_REG_FTDM_IN55_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN55_IC_1_VALUE                  0x0

/* FTDM_IN56  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN56_IC_1_ADDR                   0xF338
#define DSP1_REG_FTDM_IN56_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN56_IC_1_VALUE                  0x0

/* FTDM_IN57  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN57_IC_1_ADDR                   0xF339
#define DSP1_REG_FTDM_IN57_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN57_IC_1_VALUE                  0x0

/* FTDM_IN58  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN58_IC_1_ADDR                   0xF33A
#define DSP1_REG_FTDM_IN58_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN58_IC_1_VALUE                  0x0

/* FTDM_IN59  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN59_IC_1_ADDR                   0xF33B
#define DSP1_REG_FTDM_IN59_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN59_IC_1_VALUE                  0x0

/* FTDM_IN60  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN60_IC_1_ADDR                   0xF33C
#define DSP1_REG_FTDM_IN60_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN60_IC_1_VALUE                  0x0

/* FTDM_IN61  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN61_IC_1_ADDR                   0xF33D
#define DSP1_REG_FTDM_IN61_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN61_IC_1_VALUE                  0x0

/* FTDM_IN62  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN62_IC_1_ADDR                   0xF33E
#define DSP1_REG_FTDM_IN62_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN62_IC_1_VALUE                  0x0

/* FTDM_IN63  - Registers (IC 1) */
#define DSP1_REG_FTDM_IN63_IC_1_ADDR                   0xF33F
#define DSP1_REG_FTDM_IN63_IC_1_BYTE                   2
#define DSP1_REG_FTDM_IN63_IC_1_VALUE                  0x0

/* FTDM_OUT0  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT0_IC_1_ADDR                   0xF380
#define DSP1_REG_FTDM_OUT0_IC_1_BYTE                   2
#define DSP1_REG_FTDM_OUT0_IC_1_VALUE                  0x0

/* FTDM_OUT1  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT1_IC_1_ADDR                   0xF381
#define DSP1_REG_FTDM_OUT1_IC_1_BYTE                   2
#define DSP1_REG_FTDM_OUT1_IC_1_VALUE                  0x0

/* FTDM_OUT2  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT2_IC_1_ADDR                   0xF382
#define DSP1_REG_FTDM_OUT2_IC_1_BYTE                   2
#define DSP1_REG_FTDM_OUT2_IC_1_VALUE                  0x0

/* FTDM_OUT3  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT3_IC_1_ADDR                   0xF383
#define DSP1_REG_FTDM_OUT3_IC_1_BYTE                   2
#define DSP1_REG_FTDM_OUT3_IC_1_VALUE                  0x0

/* FTDM_OUT4  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT4_IC_1_ADDR                   0xF384
#define DSP1_REG_FTDM_OUT4_IC_1_BYTE                   2
#define DSP1_REG_FTDM_OUT4_IC_1_VALUE                  0x0

/* FTDM_OUT5  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT5_IC_1_ADDR                   0xF385
#define DSP1_REG_FTDM_OUT5_IC_1_BYTE                   2
#define DSP1_REG_FTDM_OUT5_IC_1_VALUE                  0x0

/* FTDM_OUT6  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT6_IC_1_ADDR                   0xF386
#define DSP1_REG_FTDM_OUT6_IC_1_BYTE                   2
#define DSP1_REG_FTDM_OUT6_IC_1_VALUE                  0x0

/* FTDM_OUT7  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT7_IC_1_ADDR                   0xF387
#define DSP1_REG_FTDM_OUT7_IC_1_BYTE                   2
#define DSP1_REG_FTDM_OUT7_IC_1_VALUE                  0x0

/* FTDM_OUT8  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT8_IC_1_ADDR                   0xF388
#define DSP1_REG_FTDM_OUT8_IC_1_BYTE                   2
#define DSP1_REG_FTDM_OUT8_IC_1_VALUE                  0x0

/* FTDM_OUT9  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT9_IC_1_ADDR                   0xF389
#define DSP1_REG_FTDM_OUT9_IC_1_BYTE                   2
#define DSP1_REG_FTDM_OUT9_IC_1_VALUE                  0x0

/* FTDM_OUT10  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT10_IC_1_ADDR                  0xF38A
#define DSP1_REG_FTDM_OUT10_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT10_IC_1_VALUE                 0x0

/* FTDM_OUT11  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT11_IC_1_ADDR                  0xF38B
#define DSP1_REG_FTDM_OUT11_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT11_IC_1_VALUE                 0x0

/* FTDM_OUT12  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT12_IC_1_ADDR                  0xF38C
#define DSP1_REG_FTDM_OUT12_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT12_IC_1_VALUE                 0x0

/* FTDM_OUT13  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT13_IC_1_ADDR                  0xF38D
#define DSP1_REG_FTDM_OUT13_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT13_IC_1_VALUE                 0x0

/* FTDM_OUT14  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT14_IC_1_ADDR                  0xF38E
#define DSP1_REG_FTDM_OUT14_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT14_IC_1_VALUE                 0x0

/* FTDM_OUT15  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT15_IC_1_ADDR                  0xF38F
#define DSP1_REG_FTDM_OUT15_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT15_IC_1_VALUE                 0x0

/* FTDM_OUT16  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT16_IC_1_ADDR                  0xF390
#define DSP1_REG_FTDM_OUT16_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT16_IC_1_VALUE                 0x0

/* FTDM_OUT17  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT17_IC_1_ADDR                  0xF391
#define DSP1_REG_FTDM_OUT17_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT17_IC_1_VALUE                 0x0

/* FTDM_OUT18  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT18_IC_1_ADDR                  0xF392
#define DSP1_REG_FTDM_OUT18_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT18_IC_1_VALUE                 0x0

/* FTDM_OUT19  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT19_IC_1_ADDR                  0xF393
#define DSP1_REG_FTDM_OUT19_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT19_IC_1_VALUE                 0x0

/* FTDM_OUT20  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT20_IC_1_ADDR                  0xF394
#define DSP1_REG_FTDM_OUT20_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT20_IC_1_VALUE                 0x0

/* FTDM_OUT21  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT21_IC_1_ADDR                  0xF395
#define DSP1_REG_FTDM_OUT21_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT21_IC_1_VALUE                 0x0

/* FTDM_OUT22  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT22_IC_1_ADDR                  0xF396
#define DSP1_REG_FTDM_OUT22_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT22_IC_1_VALUE                 0x0

/* FTDM_OUT23  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT23_IC_1_ADDR                  0xF397
#define DSP1_REG_FTDM_OUT23_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT23_IC_1_VALUE                 0x0

/* FTDM_OUT24  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT24_IC_1_ADDR                  0xF398
#define DSP1_REG_FTDM_OUT24_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT24_IC_1_VALUE                 0x0

/* FTDM_OUT25  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT25_IC_1_ADDR                  0xF399
#define DSP1_REG_FTDM_OUT25_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT25_IC_1_VALUE                 0x0

/* FTDM_OUT26  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT26_IC_1_ADDR                  0xF39A
#define DSP1_REG_FTDM_OUT26_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT26_IC_1_VALUE                 0x0

/* FTDM_OUT27  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT27_IC_1_ADDR                  0xF39B
#define DSP1_REG_FTDM_OUT27_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT27_IC_1_VALUE                 0x0

/* FTDM_OUT28  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT28_IC_1_ADDR                  0xF39C
#define DSP1_REG_FTDM_OUT28_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT28_IC_1_VALUE                 0x0

/* FTDM_OUT29  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT29_IC_1_ADDR                  0xF39D
#define DSP1_REG_FTDM_OUT29_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT29_IC_1_VALUE                 0x0

/* FTDM_OUT30  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT30_IC_1_ADDR                  0xF39E
#define DSP1_REG_FTDM_OUT30_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT30_IC_1_VALUE                 0x0

/* FTDM_OUT31  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT31_IC_1_ADDR                  0xF39F
#define DSP1_REG_FTDM_OUT31_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT31_IC_1_VALUE                 0x0

/* FTDM_OUT32  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT32_IC_1_ADDR                  0xF3A0
#define DSP1_REG_FTDM_OUT32_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT32_IC_1_VALUE                 0x0

/* FTDM_OUT33  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT33_IC_1_ADDR                  0xF3A1
#define DSP1_REG_FTDM_OUT33_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT33_IC_1_VALUE                 0x0

/* FTDM_OUT34  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT34_IC_1_ADDR                  0xF3A2
#define DSP1_REG_FTDM_OUT34_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT34_IC_1_VALUE                 0x0

/* FTDM_OUT35  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT35_IC_1_ADDR                  0xF3A3
#define DSP1_REG_FTDM_OUT35_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT35_IC_1_VALUE                 0x0

/* FTDM_OUT36  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT36_IC_1_ADDR                  0xF3A4
#define DSP1_REG_FTDM_OUT36_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT36_IC_1_VALUE                 0x0

/* FTDM_OUT37  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT37_IC_1_ADDR                  0xF3A5
#define DSP1_REG_FTDM_OUT37_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT37_IC_1_VALUE                 0x0

/* FTDM_OUT38  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT38_IC_1_ADDR                  0xF3A6
#define DSP1_REG_FTDM_OUT38_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT38_IC_1_VALUE                 0x0

/* FTDM_OUT39  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT39_IC_1_ADDR                  0xF3A7
#define DSP1_REG_FTDM_OUT39_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT39_IC_1_VALUE                 0x0

/* FTDM_OUT40  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT40_IC_1_ADDR                  0xF3A8
#define DSP1_REG_FTDM_OUT40_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT40_IC_1_VALUE                 0x0

/* FTDM_OUT41  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT41_IC_1_ADDR                  0xF3A9
#define DSP1_REG_FTDM_OUT41_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT41_IC_1_VALUE                 0x0

/* FTDM_OUT42  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT42_IC_1_ADDR                  0xF3AA
#define DSP1_REG_FTDM_OUT42_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT42_IC_1_VALUE                 0x0

/* FTDM_OUT43  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT43_IC_1_ADDR                  0xF3AB
#define DSP1_REG_FTDM_OUT43_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT43_IC_1_VALUE                 0x0

/* FTDM_OUT44  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT44_IC_1_ADDR                  0xF3AC
#define DSP1_REG_FTDM_OUT44_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT44_IC_1_VALUE                 0x0

/* FTDM_OUT45  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT45_IC_1_ADDR                  0xF3AD
#define DSP1_REG_FTDM_OUT45_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT45_IC_1_VALUE                 0x0

/* FTDM_OUT46  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT46_IC_1_ADDR                  0xF3AE
#define DSP1_REG_FTDM_OUT46_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT46_IC_1_VALUE                 0x0

/* FTDM_OUT47  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT47_IC_1_ADDR                  0xF3AF
#define DSP1_REG_FTDM_OUT47_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT47_IC_1_VALUE                 0x0

/* FTDM_OUT48  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT48_IC_1_ADDR                  0xF3B0
#define DSP1_REG_FTDM_OUT48_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT48_IC_1_VALUE                 0x0

/* FTDM_OUT49  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT49_IC_1_ADDR                  0xF3B1
#define DSP1_REG_FTDM_OUT49_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT49_IC_1_VALUE                 0x0

/* FTDM_OUT50  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT50_IC_1_ADDR                  0xF3B2
#define DSP1_REG_FTDM_OUT50_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT50_IC_1_VALUE                 0x0

/* FTDM_OUT51  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT51_IC_1_ADDR                  0xF3B3
#define DSP1_REG_FTDM_OUT51_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT51_IC_1_VALUE                 0x0

/* FTDM_OUT52  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT52_IC_1_ADDR                  0xF3B4
#define DSP1_REG_FTDM_OUT52_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT52_IC_1_VALUE                 0x0

/* FTDM_OUT53  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT53_IC_1_ADDR                  0xF3B5
#define DSP1_REG_FTDM_OUT53_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT53_IC_1_VALUE                 0x0

/* FTDM_OUT54  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT54_IC_1_ADDR                  0xF3B6
#define DSP1_REG_FTDM_OUT54_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT54_IC_1_VALUE                 0x0

/* FTDM_OUT55  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT55_IC_1_ADDR                  0xF3B7
#define DSP1_REG_FTDM_OUT55_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT55_IC_1_VALUE                 0x0

/* FTDM_OUT56  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT56_IC_1_ADDR                  0xF3B8
#define DSP1_REG_FTDM_OUT56_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT56_IC_1_VALUE                 0x0

/* FTDM_OUT57  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT57_IC_1_ADDR                  0xF3B9
#define DSP1_REG_FTDM_OUT57_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT57_IC_1_VALUE                 0x0

/* FTDM_OUT58  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT58_IC_1_ADDR                  0xF3BA
#define DSP1_REG_FTDM_OUT58_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT58_IC_1_VALUE                 0x0

/* FTDM_OUT59  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT59_IC_1_ADDR                  0xF3BB
#define DSP1_REG_FTDM_OUT59_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT59_IC_1_VALUE                 0x0

/* FTDM_OUT60  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT60_IC_1_ADDR                  0xF3BC
#define DSP1_REG_FTDM_OUT60_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT60_IC_1_VALUE                 0x0

/* FTDM_OUT61  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT61_IC_1_ADDR                  0xF3BD
#define DSP1_REG_FTDM_OUT61_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT61_IC_1_VALUE                 0x0

/* FTDM_OUT62  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT62_IC_1_ADDR                  0xF3BE
#define DSP1_REG_FTDM_OUT62_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT62_IC_1_VALUE                 0x0

/* FTDM_OUT63  - Registers (IC 1) */
#define DSP1_REG_FTDM_OUT63_IC_1_ADDR                  0xF3BF
#define DSP1_REG_FTDM_OUT63_IC_1_BYTE                  2
#define DSP1_REG_FTDM_OUT63_IC_1_VALUE                 0x0

/* HIBERNATE  - Registers (IC 1) */
#define DSP1_REG_HIBERNATE_IC_1_ADDR                   0xF400
#define DSP1_REG_HIBERNATE_IC_1_BYTE                   2
#define DSP1_REG_HIBERNATE_IC_1_VALUE                  0x0

/* START_PULSE  - Registers (IC 1) */
#define DSP1_REG_START_PULSE_IC_1_ADDR                 0xF401
#define DSP1_REG_START_PULSE_IC_1_BYTE                 2
#define DSP1_REG_START_PULSE_IC_1_VALUE                0x2

/* START_CORE  - Registers (IC 1) */
#define DSP1_REG_START_CORE_IC_1_ADDR                  0xF402
#define DSP1_REG_START_CORE_IC_1_BYTE                  2
#define DSP1_REG_START_CORE_IC_1_VALUE                 0x1

/* KILL_CORE  - Registers (IC 1) */
#define DSP1_REG_KILL_CORE_IC_1_ADDR                   0xF403
#define DSP1_REG_KILL_CORE_IC_1_BYTE                   2
#define DSP1_REG_KILL_CORE_IC_1_VALUE                  0x0

/* START_ADDRESS  - Registers (IC 1) */
#define DSP1_REG_START_ADDRESS_IC_1_ADDR               0xF404
#define DSP1_REG_START_ADDRESS_IC_1_BYTE               2
#define DSP1_REG_START_ADDRESS_IC_1_VALUE              0x0

/* CORE_STATUS  - Registers (IC 1) */
#define DSP1_REG_CORE_STATUS_IC_1_ADDR                 0xF405
#define DSP1_REG_CORE_STATUS_IC_1_BYTE                 0
#define DSP1_REG_CORE_STATUS_IC_1_VALUE                0x0

/* PANIC_CLEAR  - Registers (IC 1) */
#define DSP1_REG_PANIC_CLEAR_IC_1_ADDR                 0xF421
#define DSP1_REG_PANIC_CLEAR_IC_1_BYTE                 2
#define DSP1_REG_PANIC_CLEAR_IC_1_VALUE                0x0

/* PANIC_PARITY_MASK  - Registers (IC 1) */
#define DSP1_REG_PANIC_PARITY_MASK_IC_1_ADDR           0xF422
#define DSP1_REG_PANIC_PARITY_MASK_IC_1_BYTE           2
#define DSP1_REG_PANIC_PARITY_MASK_IC_1_VALUE          0x3

/* PANIC_SOFTWARE_MASK  - Registers (IC 1) */
#define DSP1_REG_PANIC_SOFTWARE_MASK_IC_1_ADDR         0xF423
#define DSP1_REG_PANIC_SOFTWARE_MASK_IC_1_BYTE         2
#define DSP1_REG_PANIC_SOFTWARE_MASK_IC_1_VALUE        0x0

/* PANIC_WD_MASK  - Registers (IC 1) */
#define DSP1_REG_PANIC_WD_MASK_IC_1_ADDR               0xF424
#define DSP1_REG_PANIC_WD_MASK_IC_1_BYTE               2
#define DSP1_REG_PANIC_WD_MASK_IC_1_VALUE              0x0

/* PANIC_STACK_MASK  - Registers (IC 1) */
#define DSP1_REG_PANIC_STACK_MASK_IC_1_ADDR            0xF425
#define DSP1_REG_PANIC_STACK_MASK_IC_1_BYTE            2
#define DSP1_REG_PANIC_STACK_MASK_IC_1_VALUE           0x0

/* PANIC_LOOP_MASK  - Registers (IC 1) */
#define DSP1_REG_PANIC_LOOP_MASK_IC_1_ADDR             0xF426
#define DSP1_REG_PANIC_LOOP_MASK_IC_1_BYTE             2
#define DSP1_REG_PANIC_LOOP_MASK_IC_1_VALUE            0x0

/* PANIC_FLAG  - Registers (IC 1) */
#define DSP1_REG_PANIC_FLAG_IC_1_ADDR                  0xF427
#define DSP1_REG_PANIC_FLAG_IC_1_BYTE                  0
#define DSP1_REG_PANIC_FLAG_IC_1_VALUE                 0x0

/* PANIC_CODE  - Registers (IC 1) */
#define DSP1_REG_PANIC_CODE_IC_1_ADDR                  0xF428
#define DSP1_REG_PANIC_CODE_IC_1_BYTE                  0
#define DSP1_REG_PANIC_CODE_IC_1_VALUE                 0x0

/* EXECUTE_COUNT  - Registers (IC 1) */
#define DSP1_REG_EXECUTE_COUNT_IC_1_ADDR               0xF432
#define DSP1_REG_EXECUTE_COUNT_IC_1_BYTE               0
#define DSP1_REG_EXECUTE_COUNT_IC_1_VALUE              0x0

/* WATCHDOG_MAXCOUNT  - Registers (IC 1) */
#define DSP1_REG_WATCHDOG_MAXCOUNT_IC_1_ADDR           0xF443
#define DSP1_REG_WATCHDOG_MAXCOUNT_IC_1_BYTE           2
#define DSP1_REG_WATCHDOG_MAXCOUNT_IC_1_VALUE          0x0

/* WATCHDOG_PRESCALE  - Registers (IC 1) */
#define DSP1_REG_WATCHDOG_PRESCALE_IC_1_ADDR           0xF444
#define DSP1_REG_WATCHDOG_PRESCALE_IC_1_BYTE           2
#define DSP1_REG_WATCHDOG_PRESCALE_IC_1_VALUE          0x0

/* BLOCKINT_EN  - Registers (IC 1) */
#define DSP1_REG_BLOCKINT_EN_IC_1_ADDR                 0xF450
#define DSP1_REG_BLOCKINT_EN_IC_1_BYTE                 2
#define DSP1_REG_BLOCKINT_EN_IC_1_VALUE                0x0

/* BLOCKINT_VALUE  - Registers (IC 1) */
#define DSP1_REG_BLOCKINT_VALUE_IC_1_ADDR              0xF451
#define DSP1_REG_BLOCKINT_VALUE_IC_1_BYTE              2
#define DSP1_REG_BLOCKINT_VALUE_IC_1_VALUE             0x0

/* PROG_CNTR0  - Registers (IC 1) */
#define DSP1_REG_PROG_CNTR0_IC_1_ADDR                  0xF460
#define DSP1_REG_PROG_CNTR0_IC_1_BYTE                  0
#define DSP1_REG_PROG_CNTR0_IC_1_VALUE                 0x0

/* PROG_CNTR1  - Registers (IC 1) */
#define DSP1_REG_PROG_CNTR1_IC_1_ADDR                  0xF461
#define DSP1_REG_PROG_CNTR1_IC_1_BYTE                  0
#define DSP1_REG_PROG_CNTR1_IC_1_VALUE                 0x0

/* PROG_CNTR_CLEAR  - Registers (IC 1) */
#define DSP1_REG_PROG_CNTR_CLEAR_IC_1_ADDR             0xF462
#define DSP1_REG_PROG_CNTR_CLEAR_IC_1_BYTE             2
#define DSP1_REG_PROG_CNTR_CLEAR_IC_1_VALUE            0x0

/* PROG_CNTR_LENGTH0  - Registers (IC 1) */
#define DSP1_REG_PROG_CNTR_LENGTH0_IC_1_ADDR           0xF463
#define DSP1_REG_PROG_CNTR_LENGTH0_IC_1_BYTE           0
#define DSP1_REG_PROG_CNTR_LENGTH0_IC_1_VALUE          0x0

/* PROG_CNTR_LENGTH1  - Registers (IC 1) */
#define DSP1_REG_PROG_CNTR_LENGTH1_IC_1_ADDR           0xF464
#define DSP1_REG_PROG_CNTR_LENGTH1_IC_1_BYTE           0
#define DSP1_REG_PROG_CNTR_LENGTH1_IC_1_VALUE          0x0

/* PROG_CNTR_MAXLENGTH0  - Registers (IC 1) */
#define DSP1_REG_PROG_CNTR_MAXLENGTH0_IC_1_ADDR        0xF465
#define DSP1_REG_PROG_CNTR_MAXLENGTH0_IC_1_BYTE        0
#define DSP1_REG_PROG_CNTR_MAXLENGTH0_IC_1_VALUE       0x0

/* PROG_CNTR_MAXLENGTH1  - Registers (IC 1) */
#define DSP1_REG_PROG_CNTR_MAXLENGTH1_IC_1_ADDR        0xF466
#define DSP1_REG_PROG_CNTR_MAXLENGTH1_IC_1_BYTE        0
#define DSP1_REG_PROG_CNTR_MAXLENGTH1_IC_1_VALUE       0x0

/* PANIC_PARITY_MASK1  - Registers (IC 1) */
#define DSP1_REG_PANIC_PARITY_MASK1_IC_1_ADDR          0xF467
#define DSP1_REG_PANIC_PARITY_MASK1_IC_1_BYTE          2
#define DSP1_REG_PANIC_PARITY_MASK1_IC_1_VALUE         0x0

/* PANIC_PARITY_MASK2  - Registers (IC 1) */
#define DSP1_REG_PANIC_PARITY_MASK2_IC_1_ADDR          0xF468
#define DSP1_REG_PANIC_PARITY_MASK2_IC_1_BYTE          2
#define DSP1_REG_PANIC_PARITY_MASK2_IC_1_VALUE         0x0

/* PANIC_PARITY_MASK3  - Registers (IC 1) */
#define DSP1_REG_PANIC_PARITY_MASK3_IC_1_ADDR          0xF469
#define DSP1_REG_PANIC_PARITY_MASK3_IC_1_BYTE          2
#define DSP1_REG_PANIC_PARITY_MASK3_IC_1_VALUE         0x0

/* PANIC_PARITY_MASK4  - Registers (IC 1) */
#define DSP1_REG_PANIC_PARITY_MASK4_IC_1_ADDR          0xF46A
#define DSP1_REG_PANIC_PARITY_MASK4_IC_1_BYTE          2
#define DSP1_REG_PANIC_PARITY_MASK4_IC_1_VALUE         0x0

/* PANIC_PARITY_MASK5  - Registers (IC 1) */
#define DSP1_REG_PANIC_PARITY_MASK5_IC_1_ADDR          0xF46B
#define DSP1_REG_PANIC_PARITY_MASK5_IC_1_BYTE          2
#define DSP1_REG_PANIC_PARITY_MASK5_IC_1_VALUE         0x0

/* PANIC_CODE1  - Registers (IC 1) */
#define DSP1_REG_PANIC_CODE1_IC_1_ADDR                 0xF46C
#define DSP1_REG_PANIC_CODE1_IC_1_BYTE                 0
#define DSP1_REG_PANIC_CODE1_IC_1_VALUE                0x0

/* PANIC_CODE2  - Registers (IC 1) */
#define DSP1_REG_PANIC_CODE2_IC_1_ADDR                 0xF46D
#define DSP1_REG_PANIC_CODE2_IC_1_BYTE                 0
#define DSP1_REG_PANIC_CODE2_IC_1_VALUE                0x0

/* PANIC_CODE3  - Registers (IC 1) */
#define DSP1_REG_PANIC_CODE3_IC_1_ADDR                 0xF46E
#define DSP1_REG_PANIC_CODE3_IC_1_BYTE                 0
#define DSP1_REG_PANIC_CODE3_IC_1_VALUE                0x0

/* PANIC_CODE4  - Registers (IC 1) */
#define DSP1_REG_PANIC_CODE4_IC_1_ADDR                 0xF46F
#define DSP1_REG_PANIC_CODE4_IC_1_BYTE                 0
#define DSP1_REG_PANIC_CODE4_IC_1_VALUE                0x0

/* PANIC_CODE5  - Registers (IC 1) */
#define DSP1_REG_PANIC_CODE5_IC_1_ADDR                 0xF470
#define DSP1_REG_PANIC_CODE5_IC_1_BYTE                 0
#define DSP1_REG_PANIC_CODE5_IC_1_VALUE                0x0

/* MP0_MODE  - Registers (IC 1) */
#define DSP1_REG_MP0_MODE_IC_1_ADDR                    0xF510
#define DSP1_REG_MP0_MODE_IC_1_BYTE                    2
#define DSP1_REG_MP0_MODE_IC_1_VALUE                   0x0

/* MP1_MODE  - Registers (IC 1) */
#define DSP1_REG_MP1_MODE_IC_1_ADDR                    0xF511
#define DSP1_REG_MP1_MODE_IC_1_BYTE                    2
#define DSP1_REG_MP1_MODE_IC_1_VALUE                   0x0

/* MP2_MODE  - Registers (IC 1) */
#define DSP1_REG_MP2_MODE_IC_1_ADDR                    0xF512
#define DSP1_REG_MP2_MODE_IC_1_BYTE                    2
#define DSP1_REG_MP2_MODE_IC_1_VALUE                   0x0

/* MP3_MODE  - Registers (IC 1) */
#define DSP1_REG_MP3_MODE_IC_1_ADDR                    0xF513
#define DSP1_REG_MP3_MODE_IC_1_BYTE                    2
#define DSP1_REG_MP3_MODE_IC_1_VALUE                   0x0

/* MP4_MODE  - Registers (IC 1) */
#define DSP1_REG_MP4_MODE_IC_1_ADDR                    0xF514
#define DSP1_REG_MP4_MODE_IC_1_BYTE                    2
#define DSP1_REG_MP4_MODE_IC_1_VALUE                   0x0

/* MP5_MODE  - Registers (IC 1) */
#define DSP1_REG_MP5_MODE_IC_1_ADDR                    0xF515
#define DSP1_REG_MP5_MODE_IC_1_BYTE                    2
#define DSP1_REG_MP5_MODE_IC_1_VALUE                   0x0

/* MP6_MODE  - Registers (IC 1) */
#define DSP1_REG_MP6_MODE_IC_1_ADDR                    0xF516
#define DSP1_REG_MP6_MODE_IC_1_BYTE                    2
#define DSP1_REG_MP6_MODE_IC_1_VALUE                   0x0

/* MP7_MODE  - Registers (IC 1) */
#define DSP1_REG_MP7_MODE_IC_1_ADDR                    0xF517
#define DSP1_REG_MP7_MODE_IC_1_BYTE                    2
#define DSP1_REG_MP7_MODE_IC_1_VALUE                   0x0

/* MP8_MODE  - Registers (IC 1) */
#define DSP1_REG_MP8_MODE_IC_1_ADDR                    0xF518
#define DSP1_REG_MP8_MODE_IC_1_BYTE                    2
#define DSP1_REG_MP8_MODE_IC_1_VALUE                   0x0

/* MP9_MODE  - Registers (IC 1) */
#define DSP1_REG_MP9_MODE_IC_1_ADDR                    0xF519
#define DSP1_REG_MP9_MODE_IC_1_BYTE                    2
#define DSP1_REG_MP9_MODE_IC_1_VALUE                   0x0

/* MP10_MODE  - Registers (IC 1) */
#define DSP1_REG_MP10_MODE_IC_1_ADDR                   0xF51A
#define DSP1_REG_MP10_MODE_IC_1_BYTE                   2
#define DSP1_REG_MP10_MODE_IC_1_VALUE                  0x0

/* MP11_MODE  - Registers (IC 1) */
#define DSP1_REG_MP11_MODE_IC_1_ADDR                   0xF51B
#define DSP1_REG_MP11_MODE_IC_1_BYTE                   2
#define DSP1_REG_MP11_MODE_IC_1_VALUE                  0x0

/* MP12_MODE  - Registers (IC 1) */
#define DSP1_REG_MP12_MODE_IC_1_ADDR                   0xF51C
#define DSP1_REG_MP12_MODE_IC_1_BYTE                   2
#define DSP1_REG_MP12_MODE_IC_1_VALUE                  0x0

/* MP13_MODE  - Registers (IC 1) */
#define DSP1_REG_MP13_MODE_IC_1_ADDR                   0xF51D
#define DSP1_REG_MP13_MODE_IC_1_BYTE                   2
#define DSP1_REG_MP13_MODE_IC_1_VALUE                  0x0

/* MP0_WRITE  - Registers (IC 1) */
#define DSP1_REG_MP0_WRITE_IC_1_ADDR                   0xF520
#define DSP1_REG_MP0_WRITE_IC_1_BYTE                   2
#define DSP1_REG_MP0_WRITE_IC_1_VALUE                  0x0

/* MP1_WRITE  - Registers (IC 1) */
#define DSP1_REG_MP1_WRITE_IC_1_ADDR                   0xF521
#define DSP1_REG_MP1_WRITE_IC_1_BYTE                   2
#define DSP1_REG_MP1_WRITE_IC_1_VALUE                  0x0

/* MP2_WRITE  - Registers (IC 1) */
#define DSP1_REG_MP2_WRITE_IC_1_ADDR                   0xF522
#define DSP1_REG_MP2_WRITE_IC_1_BYTE                   2
#define DSP1_REG_MP2_WRITE_IC_1_VALUE                  0x0

/* MP3_WRITE  - Registers (IC 1) */
#define DSP1_REG_MP3_WRITE_IC_1_ADDR                   0xF523
#define DSP1_REG_MP3_WRITE_IC_1_BYTE                   2
#define DSP1_REG_MP3_WRITE_IC_1_VALUE                  0x0

/* MP4_WRITE  - Registers (IC 1) */
#define DSP1_REG_MP4_WRITE_IC_1_ADDR                   0xF524
#define DSP1_REG_MP4_WRITE_IC_1_BYTE                   2
#define DSP1_REG_MP4_WRITE_IC_1_VALUE                  0x0

/* MP5_WRITE  - Registers (IC 1) */
#define DSP1_REG_MP5_WRITE_IC_1_ADDR                   0xF525
#define DSP1_REG_MP5_WRITE_IC_1_BYTE                   2
#define DSP1_REG_MP5_WRITE_IC_1_VALUE                  0x0

/* MP6_WRITE  - Registers (IC 1) */
#define DSP1_REG_MP6_WRITE_IC_1_ADDR                   0xF526
#define DSP1_REG_MP6_WRITE_IC_1_BYTE                   2
#define DSP1_REG_MP6_WRITE_IC_1_VALUE                  0x0

/* MP7_WRITE  - Registers (IC 1) */
#define DSP1_REG_MP7_WRITE_IC_1_ADDR                   0xF527
#define DSP1_REG_MP7_WRITE_IC_1_BYTE                   2
#define DSP1_REG_MP7_WRITE_IC_1_VALUE                  0x0

/* MP8_WRITE  - Registers (IC 1) */
#define DSP1_REG_MP8_WRITE_IC_1_ADDR                   0xF528
#define DSP1_REG_MP8_WRITE_IC_1_BYTE                   2
#define DSP1_REG_MP8_WRITE_IC_1_VALUE                  0x0

/* MP9_WRITE  - Registers (IC 1) */
#define DSP1_REG_MP9_WRITE_IC_1_ADDR                   0xF529
#define DSP1_REG_MP9_WRITE_IC_1_BYTE                   2
#define DSP1_REG_MP9_WRITE_IC_1_VALUE                  0x0

/* MP10_WRITE  - Registers (IC 1) */
#define DSP1_REG_MP10_WRITE_IC_1_ADDR                  0xF52A
#define DSP1_REG_MP10_WRITE_IC_1_BYTE                  2
#define DSP1_REG_MP10_WRITE_IC_1_VALUE                 0x0

/* MP11_WRITE  - Registers (IC 1) */
#define DSP1_REG_MP11_WRITE_IC_1_ADDR                  0xF52B
#define DSP1_REG_MP11_WRITE_IC_1_BYTE                  2
#define DSP1_REG_MP11_WRITE_IC_1_VALUE                 0x0

/* MP12_WRITE  - Registers (IC 1) */
#define DSP1_REG_MP12_WRITE_IC_1_ADDR                  0xF52C
#define DSP1_REG_MP12_WRITE_IC_1_BYTE                  2
#define DSP1_REG_MP12_WRITE_IC_1_VALUE                 0x0

/* MP13_WRITE  - Registers (IC 1) */
#define DSP1_REG_MP13_WRITE_IC_1_ADDR                  0xF52D
#define DSP1_REG_MP13_WRITE_IC_1_BYTE                  2
#define DSP1_REG_MP13_WRITE_IC_1_VALUE                 0x0

/* MP0_READ  - Registers (IC 1) */
#define DSP1_REG_MP0_READ_IC_1_ADDR                    0xF530
#define DSP1_REG_MP0_READ_IC_1_BYTE                    0
#define DSP1_REG_MP0_READ_IC_1_VALUE                   0x0

/* MP1_READ  - Registers (IC 1) */
#define DSP1_REG_MP1_READ_IC_1_ADDR                    0xF531
#define DSP1_REG_MP1_READ_IC_1_BYTE                    0
#define DSP1_REG_MP1_READ_IC_1_VALUE                   0x0

/* MP2_READ  - Registers (IC 1) */
#define DSP1_REG_MP2_READ_IC_1_ADDR                    0xF532
#define DSP1_REG_MP2_READ_IC_1_BYTE                    0
#define DSP1_REG_MP2_READ_IC_1_VALUE                   0x0

/* MP3_READ  - Registers (IC 1) */
#define DSP1_REG_MP3_READ_IC_1_ADDR                    0xF533
#define DSP1_REG_MP3_READ_IC_1_BYTE                    0
#define DSP1_REG_MP3_READ_IC_1_VALUE                   0x0

/* MP4_READ  - Registers (IC 1) */
#define DSP1_REG_MP4_READ_IC_1_ADDR                    0xF534
#define DSP1_REG_MP4_READ_IC_1_BYTE                    0
#define DSP1_REG_MP4_READ_IC_1_VALUE                   0x0

/* MP5_READ  - Registers (IC 1) */
#define DSP1_REG_MP5_READ_IC_1_ADDR                    0xF535
#define DSP1_REG_MP5_READ_IC_1_BYTE                    0
#define DSP1_REG_MP5_READ_IC_1_VALUE                   0x0

/* MP6_READ  - Registers (IC 1) */
#define DSP1_REG_MP6_READ_IC_1_ADDR                    0xF536
#define DSP1_REG_MP6_READ_IC_1_BYTE                    0
#define DSP1_REG_MP6_READ_IC_1_VALUE                   0x0

/* MP7_READ  - Registers (IC 1) */
#define DSP1_REG_MP7_READ_IC_1_ADDR                    0xF537
#define DSP1_REG_MP7_READ_IC_1_BYTE                    0
#define DSP1_REG_MP7_READ_IC_1_VALUE                   0x0

/* MP8_READ  - Registers (IC 1) */
#define DSP1_REG_MP8_READ_IC_1_ADDR                    0xF538
#define DSP1_REG_MP8_READ_IC_1_BYTE                    0
#define DSP1_REG_MP8_READ_IC_1_VALUE                   0x0

/* MP9_READ  - Registers (IC 1) */
#define DSP1_REG_MP9_READ_IC_1_ADDR                    0xF539
#define DSP1_REG_MP9_READ_IC_1_BYTE                    0
#define DSP1_REG_MP9_READ_IC_1_VALUE                   0x0

/* MP10_READ  - Registers (IC 1) */
#define DSP1_REG_MP10_READ_IC_1_ADDR                   0xF53A
#define DSP1_REG_MP10_READ_IC_1_BYTE                   0
#define DSP1_REG_MP10_READ_IC_1_VALUE                  0x0

/* MP11_READ  - Registers (IC 1) */
#define DSP1_REG_MP11_READ_IC_1_ADDR                   0xF53B
#define DSP1_REG_MP11_READ_IC_1_BYTE                   0
#define DSP1_REG_MP11_READ_IC_1_VALUE                  0x0

/* MP12_READ  - Registers (IC 1) */
#define DSP1_REG_MP12_READ_IC_1_ADDR                   0xF53C
#define DSP1_REG_MP12_READ_IC_1_BYTE                   0
#define DSP1_REG_MP12_READ_IC_1_VALUE                  0x0

/* MP13_READ  - Registers (IC 1) */
#define DSP1_REG_MP13_READ_IC_1_ADDR                   0xF53D
#define DSP1_REG_MP13_READ_IC_1_BYTE                   0
#define DSP1_REG_MP13_READ_IC_1_VALUE                  0x0

/* DMIC_CTRL0  - Registers (IC 1) */
#define DSP1_REG_DMIC_CTRL0_IC_1_ADDR                  0xF560
#define DSP1_REG_DMIC_CTRL0_IC_1_BYTE                  2
#define DSP1_REG_DMIC_CTRL0_IC_1_VALUE                 0x4000

/* DMIC_CTRL1  - Registers (IC 1) */
#define DSP1_REG_DMIC_CTRL1_IC_1_ADDR                  0xF561
#define DSP1_REG_DMIC_CTRL1_IC_1_BYTE                  2
#define DSP1_REG_DMIC_CTRL1_IC_1_VALUE                 0x4000

/* ASRC_LOCK  - Registers (IC 1) */
#define DSP1_REG_ASRC_LOCK_IC_1_ADDR                   0xF580
#define DSP1_REG_ASRC_LOCK_IC_1_BYTE                   0
#define DSP1_REG_ASRC_LOCK_IC_1_VALUE                  0x0

/* ASRC_MUTE  - Registers (IC 1) */
#define DSP1_REG_ASRC_MUTE_IC_1_ADDR                   0xF581
#define DSP1_REG_ASRC_MUTE_IC_1_BYTE                   2
#define DSP1_REG_ASRC_MUTE_IC_1_VALUE                  0x0

/* ASRC0_RATIO  - Registers (IC 1) */
#define DSP1_REG_ASRC0_RATIO_IC_1_ADDR                 0xF582
#define DSP1_REG_ASRC0_RATIO_IC_1_BYTE                 0
#define DSP1_REG_ASRC0_RATIO_IC_1_VALUE                0x0

/* ASRC1_RATIO  - Registers (IC 1) */
#define DSP1_REG_ASRC1_RATIO_IC_1_ADDR                 0xF583
#define DSP1_REG_ASRC1_RATIO_IC_1_BYTE                 0
#define DSP1_REG_ASRC1_RATIO_IC_1_VALUE                0x0

/* ASRC2_RATIO  - Registers (IC 1) */
#define DSP1_REG_ASRC2_RATIO_IC_1_ADDR                 0xF584
#define DSP1_REG_ASRC2_RATIO_IC_1_BYTE                 0
#define DSP1_REG_ASRC2_RATIO_IC_1_VALUE                0x0

/* ASRC3_RATIO  - Registers (IC 1) */
#define DSP1_REG_ASRC3_RATIO_IC_1_ADDR                 0xF585
#define DSP1_REG_ASRC3_RATIO_IC_1_BYTE                 0
#define DSP1_REG_ASRC3_RATIO_IC_1_VALUE                0x0

/* ASRC4_RATIO  - Registers (IC 1) */
#define DSP1_REG_ASRC4_RATIO_IC_1_ADDR                 0xF586
#define DSP1_REG_ASRC4_RATIO_IC_1_BYTE                 0
#define DSP1_REG_ASRC4_RATIO_IC_1_VALUE                0x0

/* ASRC5_RATIO  - Registers (IC 1) */
#define DSP1_REG_ASRC5_RATIO_IC_1_ADDR                 0xF587
#define DSP1_REG_ASRC5_RATIO_IC_1_BYTE                 0
#define DSP1_REG_ASRC5_RATIO_IC_1_VALUE                0x0

/* ASRC6_RATIO  - Registers (IC 1) */
#define DSP1_REG_ASRC6_RATIO_IC_1_ADDR                 0xF588
#define DSP1_REG_ASRC6_RATIO_IC_1_BYTE                 0
#define DSP1_REG_ASRC6_RATIO_IC_1_VALUE                0x0

/* ASRC7_RATIO  - Registers (IC 1) */
#define DSP1_REG_ASRC7_RATIO_IC_1_ADDR                 0xF589
#define DSP1_REG_ASRC7_RATIO_IC_1_BYTE                 0
#define DSP1_REG_ASRC7_RATIO_IC_1_VALUE                0x0

/* ASRC_RAMPMAX_OVR  - Registers (IC 1) */
#define DSP1_REG_ASRC_RAMPMAX_OVR_IC_1_ADDR            0xF590
#define DSP1_REG_ASRC_RAMPMAX_OVR_IC_1_BYTE            2
#define DSP1_REG_ASRC_RAMPMAX_OVR_IC_1_VALUE           0x7FF

/* ASRC0_RAMPMAX  - Registers (IC 1) */
#define DSP1_REG_ASRC0_RAMPMAX_IC_1_ADDR               0xF591
#define DSP1_REG_ASRC0_RAMPMAX_IC_1_BYTE               2
#define DSP1_REG_ASRC0_RAMPMAX_IC_1_VALUE              0x7FF

/* ASRC1_RAMPMAX  - Registers (IC 1) */
#define DSP1_REG_ASRC1_RAMPMAX_IC_1_ADDR               0xF592
#define DSP1_REG_ASRC1_RAMPMAX_IC_1_BYTE               2
#define DSP1_REG_ASRC1_RAMPMAX_IC_1_VALUE              0x7FF

/* ASRC2_RAMPMAX  - Registers (IC 1) */
#define DSP1_REG_ASRC2_RAMPMAX_IC_1_ADDR               0xF593
#define DSP1_REG_ASRC2_RAMPMAX_IC_1_BYTE               2
#define DSP1_REG_ASRC2_RAMPMAX_IC_1_VALUE              0x7FF

/* ASRC3_RAMPMAX  - Registers (IC 1) */
#define DSP1_REG_ASRC3_RAMPMAX_IC_1_ADDR               0xF594
#define DSP1_REG_ASRC3_RAMPMAX_IC_1_BYTE               2
#define DSP1_REG_ASRC3_RAMPMAX_IC_1_VALUE              0x7FF

/* ASRC4_RAMPMAX  - Registers (IC 1) */
#define DSP1_REG_ASRC4_RAMPMAX_IC_1_ADDR               0xF595
#define DSP1_REG_ASRC4_RAMPMAX_IC_1_BYTE               2
#define DSP1_REG_ASRC4_RAMPMAX_IC_1_VALUE              0x7FF

/* ASRC5_RAMPMAX  - Registers (IC 1) */
#define DSP1_REG_ASRC5_RAMPMAX_IC_1_ADDR               0xF596
#define DSP1_REG_ASRC5_RAMPMAX_IC_1_BYTE               2
#define DSP1_REG_ASRC5_RAMPMAX_IC_1_VALUE              0x7FF

/* ASRC6_RAMPMAX  - Registers (IC 1) */
#define DSP1_REG_ASRC6_RAMPMAX_IC_1_ADDR               0xF597
#define DSP1_REG_ASRC6_RAMPMAX_IC_1_BYTE               2
#define DSP1_REG_ASRC6_RAMPMAX_IC_1_VALUE              0x7FF

/* ASRC7_RAMPMAX  - Registers (IC 1) */
#define DSP1_REG_ASRC7_RAMPMAX_IC_1_ADDR               0xF598
#define DSP1_REG_ASRC7_RAMPMAX_IC_1_BYTE               2
#define DSP1_REG_ASRC7_RAMPMAX_IC_1_VALUE              0x7FF

/* ADC_READ0  - Registers (IC 1) */
#define DSP1_REG_ADC_READ0_IC_1_ADDR                   0xF5A0
#define DSP1_REG_ADC_READ0_IC_1_BYTE                   0
#define DSP1_REG_ADC_READ0_IC_1_VALUE                  0x0

/* ADC_READ1  - Registers (IC 1) */
#define DSP1_REG_ADC_READ1_IC_1_ADDR                   0xF5A1
#define DSP1_REG_ADC_READ1_IC_1_BYTE                   0
#define DSP1_REG_ADC_READ1_IC_1_VALUE                  0x0

/* ADC_READ2  - Registers (IC 1) */
#define DSP1_REG_ADC_READ2_IC_1_ADDR                   0xF5A2
#define DSP1_REG_ADC_READ2_IC_1_BYTE                   0
#define DSP1_REG_ADC_READ2_IC_1_VALUE                  0x0

/* ADC_READ3  - Registers (IC 1) */
#define DSP1_REG_ADC_READ3_IC_1_ADDR                   0xF5A3
#define DSP1_REG_ADC_READ3_IC_1_BYTE                   0
#define DSP1_REG_ADC_READ3_IC_1_VALUE                  0x0

/* ADC_READ4  - Registers (IC 1) */
#define DSP1_REG_ADC_READ4_IC_1_ADDR                   0xF5A4
#define DSP1_REG_ADC_READ4_IC_1_BYTE                   0
#define DSP1_REG_ADC_READ4_IC_1_VALUE                  0x0

/* ADC_READ5  - Registers (IC 1) */
#define DSP1_REG_ADC_READ5_IC_1_ADDR                   0xF5A5
#define DSP1_REG_ADC_READ5_IC_1_BYTE                   0
#define DSP1_REG_ADC_READ5_IC_1_VALUE                  0x0

/* MP0_MODE1  - Registers (IC 1) */
#define DSP1_REG_MP0_MODE1_IC_1_ADDR                   0xF5C0
#define DSP1_REG_MP0_MODE1_IC_1_BYTE                   2
#define DSP1_REG_MP0_MODE1_IC_1_VALUE                  0x0

/* MP1_MODE1  - Registers (IC 1) */
#define DSP1_REG_MP1_MODE1_IC_1_ADDR                   0xF5C1
#define DSP1_REG_MP1_MODE1_IC_1_BYTE                   2
#define DSP1_REG_MP1_MODE1_IC_1_VALUE                  0x0

/* MP2_MODE1  - Registers (IC 1) */
#define DSP1_REG_MP2_MODE1_IC_1_ADDR                   0xF5C2
#define DSP1_REG_MP2_MODE1_IC_1_BYTE                   2
#define DSP1_REG_MP2_MODE1_IC_1_VALUE                  0x0

/* MP3_MODE1  - Registers (IC 1) */
#define DSP1_REG_MP3_MODE1_IC_1_ADDR                   0xF5C3
#define DSP1_REG_MP3_MODE1_IC_1_BYTE                   2
#define DSP1_REG_MP3_MODE1_IC_1_VALUE                  0x0

/* MP4_MODE1  - Registers (IC 1) */
#define DSP1_REG_MP4_MODE1_IC_1_ADDR                   0xF5C4
#define DSP1_REG_MP4_MODE1_IC_1_BYTE                   2
#define DSP1_REG_MP4_MODE1_IC_1_VALUE                  0x0

/* MP5_MODE1  - Registers (IC 1) */
#define DSP1_REG_MP5_MODE1_IC_1_ADDR                   0xF5C5
#define DSP1_REG_MP5_MODE1_IC_1_BYTE                   2
#define DSP1_REG_MP5_MODE1_IC_1_VALUE                  0x0

/* MP6_MODE1  - Registers (IC 1) */
#define DSP1_REG_MP6_MODE1_IC_1_ADDR                   0xF5C6
#define DSP1_REG_MP6_MODE1_IC_1_BYTE                   2
#define DSP1_REG_MP6_MODE1_IC_1_VALUE                  0x0

/* MP7_MODE1  - Registers (IC 1) */
#define DSP1_REG_MP7_MODE1_IC_1_ADDR                   0xF5C7
#define DSP1_REG_MP7_MODE1_IC_1_BYTE                   2
#define DSP1_REG_MP7_MODE1_IC_1_VALUE                  0x0

/* MP8_MODE1  - Registers (IC 1) */
#define DSP1_REG_MP8_MODE1_IC_1_ADDR                   0xF5C8
#define DSP1_REG_MP8_MODE1_IC_1_BYTE                   2
#define DSP1_REG_MP8_MODE1_IC_1_VALUE                  0x0

/* MP9_MODE1  - Registers (IC 1) */
#define DSP1_REG_MP9_MODE1_IC_1_ADDR                   0xF5C9
#define DSP1_REG_MP9_MODE1_IC_1_BYTE                   2
#define DSP1_REG_MP9_MODE1_IC_1_VALUE                  0x0

/* MP10_MODE1  - Registers (IC 1) */
#define DSP1_REG_MP10_MODE1_IC_1_ADDR                  0xF5CA
#define DSP1_REG_MP10_MODE1_IC_1_BYTE                  2
#define DSP1_REG_MP10_MODE1_IC_1_VALUE                 0x0

/* MP11_MODE1  - Registers (IC 1) */
#define DSP1_REG_MP11_MODE1_IC_1_ADDR                  0xF5CB
#define DSP1_REG_MP11_MODE1_IC_1_BYTE                  2
#define DSP1_REG_MP11_MODE1_IC_1_VALUE                 0x0

/* MP0_WRITE1  - Registers (IC 1) */
#define DSP1_REG_MP0_WRITE1_IC_1_ADDR                  0xF5D0
#define DSP1_REG_MP0_WRITE1_IC_1_BYTE                  2
#define DSP1_REG_MP0_WRITE1_IC_1_VALUE                 0x0

/* MP1_WRITE1  - Registers (IC 1) */
#define DSP1_REG_MP1_WRITE1_IC_1_ADDR                  0xF5D1
#define DSP1_REG_MP1_WRITE1_IC_1_BYTE                  2
#define DSP1_REG_MP1_WRITE1_IC_1_VALUE                 0x0

/* MP2_WRITE1  - Registers (IC 1) */
#define DSP1_REG_MP2_WRITE1_IC_1_ADDR                  0xF5D2
#define DSP1_REG_MP2_WRITE1_IC_1_BYTE                  2
#define DSP1_REG_MP2_WRITE1_IC_1_VALUE                 0x0

/* MP3_WRITE1  - Registers (IC 1) */
#define DSP1_REG_MP3_WRITE1_IC_1_ADDR                  0xF5D3
#define DSP1_REG_MP3_WRITE1_IC_1_BYTE                  2
#define DSP1_REG_MP3_WRITE1_IC_1_VALUE                 0x0

/* MP4_WRITE1  - Registers (IC 1) */
#define DSP1_REG_MP4_WRITE1_IC_1_ADDR                  0xF5D4
#define DSP1_REG_MP4_WRITE1_IC_1_BYTE                  2
#define DSP1_REG_MP4_WRITE1_IC_1_VALUE                 0x0

/* MP5_WRITE1  - Registers (IC 1) */
#define DSP1_REG_MP5_WRITE1_IC_1_ADDR                  0xF5D5
#define DSP1_REG_MP5_WRITE1_IC_1_BYTE                  2
#define DSP1_REG_MP5_WRITE1_IC_1_VALUE                 0x0

/* MP6_WRITE1  - Registers (IC 1) */
#define DSP1_REG_MP6_WRITE1_IC_1_ADDR                  0xF5D6
#define DSP1_REG_MP6_WRITE1_IC_1_BYTE                  2
#define DSP1_REG_MP6_WRITE1_IC_1_VALUE                 0x0

/* MP7_WRITE1  - Registers (IC 1) */
#define DSP1_REG_MP7_WRITE1_IC_1_ADDR                  0xF5D7
#define DSP1_REG_MP7_WRITE1_IC_1_BYTE                  2
#define DSP1_REG_MP7_WRITE1_IC_1_VALUE                 0x0

/* MP8_WRITE1  - Registers (IC 1) */
#define DSP1_REG_MP8_WRITE1_IC_1_ADDR                  0xF5D8
#define DSP1_REG_MP8_WRITE1_IC_1_BYTE                  2
#define DSP1_REG_MP8_WRITE1_IC_1_VALUE                 0x0

/* MP9_WRITE1  - Registers (IC 1) */
#define DSP1_REG_MP9_WRITE1_IC_1_ADDR                  0xF5D9
#define DSP1_REG_MP9_WRITE1_IC_1_BYTE                  2
#define DSP1_REG_MP9_WRITE1_IC_1_VALUE                 0x0

/* MP10_WRITE1  - Registers (IC 1) */
#define DSP1_REG_MP10_WRITE1_IC_1_ADDR                 0xF5DA
#define DSP1_REG_MP10_WRITE1_IC_1_BYTE                 2
#define DSP1_REG_MP10_WRITE1_IC_1_VALUE                0x0

/* MP11_WRITE1  - Registers (IC 1) */
#define DSP1_REG_MP11_WRITE1_IC_1_ADDR                 0xF5DB
#define DSP1_REG_MP11_WRITE1_IC_1_BYTE                 2
#define DSP1_REG_MP11_WRITE1_IC_1_VALUE                0x0

/* MP0_READ1  - Registers (IC 1) */
#define DSP1_REG_MP0_READ1_IC_1_ADDR                   0xF5E0
#define DSP1_REG_MP0_READ1_IC_1_BYTE                   0
#define DSP1_REG_MP0_READ1_IC_1_VALUE                  0x0

/* MP1_READ1  - Registers (IC 1) */
#define DSP1_REG_MP1_READ1_IC_1_ADDR                   0xF5E1
#define DSP1_REG_MP1_READ1_IC_1_BYTE                   0
#define DSP1_REG_MP1_READ1_IC_1_VALUE                  0x0

/* MP2_READ1  - Registers (IC 1) */
#define DSP1_REG_MP2_READ1_IC_1_ADDR                   0xF5E2
#define DSP1_REG_MP2_READ1_IC_1_BYTE                   0
#define DSP1_REG_MP2_READ1_IC_1_VALUE                  0x0

/* MP3_READ1  - Registers (IC 1) */
#define DSP1_REG_MP3_READ1_IC_1_ADDR                   0xF5E3
#define DSP1_REG_MP3_READ1_IC_1_BYTE                   0
#define DSP1_REG_MP3_READ1_IC_1_VALUE                  0x0

/* MP4_READ1  - Registers (IC 1) */
#define DSP1_REG_MP4_READ1_IC_1_ADDR                   0xF5E4
#define DSP1_REG_MP4_READ1_IC_1_BYTE                   0
#define DSP1_REG_MP4_READ1_IC_1_VALUE                  0x0

/* MP5_READ1  - Registers (IC 1) */
#define DSP1_REG_MP5_READ1_IC_1_ADDR                   0xF5E5
#define DSP1_REG_MP5_READ1_IC_1_BYTE                   0
#define DSP1_REG_MP5_READ1_IC_1_VALUE                  0x0

/* MP6_READ1  - Registers (IC 1) */
#define DSP1_REG_MP6_READ1_IC_1_ADDR                   0xF5E6
#define DSP1_REG_MP6_READ1_IC_1_BYTE                   0
#define DSP1_REG_MP6_READ1_IC_1_VALUE                  0x0

/* MP7_READ1  - Registers (IC 1) */
#define DSP1_REG_MP7_READ1_IC_1_ADDR                   0xF5E7
#define DSP1_REG_MP7_READ1_IC_1_BYTE                   0
#define DSP1_REG_MP7_READ1_IC_1_VALUE                  0x0

/* MP8_READ1  - Registers (IC 1) */
#define DSP1_REG_MP8_READ1_IC_1_ADDR                   0xF5E8
#define DSP1_REG_MP8_READ1_IC_1_BYTE                   0
#define DSP1_REG_MP8_READ1_IC_1_VALUE                  0x0

/* MP9_READ1  - Registers (IC 1) */
#define DSP1_REG_MP9_READ1_IC_1_ADDR                   0xF5E9
#define DSP1_REG_MP9_READ1_IC_1_BYTE                   0
#define DSP1_REG_MP9_READ1_IC_1_VALUE                  0x0

/* MP10_READ1  - Registers (IC 1) */
#define DSP1_REG_MP10_READ1_IC_1_ADDR                  0xF5EA
#define DSP1_REG_MP10_READ1_IC_1_BYTE                  0
#define DSP1_REG_MP10_READ1_IC_1_VALUE                 0x0

/* MP11_READ1  - Registers (IC 1) */
#define DSP1_REG_MP11_READ1_IC_1_ADDR                  0xF5EB
#define DSP1_REG_MP11_READ1_IC_1_BYTE                  0
#define DSP1_REG_MP11_READ1_IC_1_VALUE                 0x0

/* SPDIF_LOCK_DET  - Registers (IC 1) */
#define DSP1_REG_SPDIF_LOCK_DET_IC_1_ADDR              0xF600
#define DSP1_REG_SPDIF_LOCK_DET_IC_1_BYTE              0
#define DSP1_REG_SPDIF_LOCK_DET_IC_1_VALUE             0x0

/* SPDIF_RX_CTRL  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CTRL_IC_1_ADDR               0xF601
#define DSP1_REG_SPDIF_RX_CTRL_IC_1_BYTE               2
#define DSP1_REG_SPDIF_RX_CTRL_IC_1_VALUE              0x0

/* SPDIF_RX_DECODE  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_DECODE_IC_1_ADDR             0xF602
#define DSP1_REG_SPDIF_RX_DECODE_IC_1_BYTE             0
#define DSP1_REG_SPDIF_RX_DECODE_IC_1_VALUE            0x88

/* SPDIF_RX_COMPRMODE  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_COMPRMODE_IC_1_ADDR          0xF603
#define DSP1_REG_SPDIF_RX_COMPRMODE_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_COMPRMODE_IC_1_VALUE         0x0

/* SPDIF_RESTART  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RESTART_IC_1_ADDR               0xF604
#define DSP1_REG_SPDIF_RESTART_IC_1_BYTE               2
#define DSP1_REG_SPDIF_RESTART_IC_1_VALUE              0x0

/* SPDIF_LOSS_OF_LOCK  - Registers (IC 1) */
#define DSP1_REG_SPDIF_LOSS_OF_LOCK_IC_1_ADDR          0xF605
#define DSP1_REG_SPDIF_LOSS_OF_LOCK_IC_1_BYTE          0
#define DSP1_REG_SPDIF_LOSS_OF_LOCK_IC_1_VALUE         0x0

/* SPDIF_RX_MCLKSPEED  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_MCLKSPEED_IC_1_ADDR          0xF606
#define DSP1_REG_SPDIF_RX_MCLKSPEED_IC_1_BYTE          2
#define DSP1_REG_SPDIF_RX_MCLKSPEED_IC_1_VALUE         0x1

/* SPDIF_TX_MCLKSPEED  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_MCLKSPEED_IC_1_ADDR          0xF607
#define DSP1_REG_SPDIF_TX_MCLKSPEED_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_MCLKSPEED_IC_1_VALUE         0x1

/* SPDIF_AUX_EN  - Registers (IC 1) */
#define DSP1_REG_SPDIF_AUX_EN_IC_1_ADDR                0xF608
#define DSP1_REG_SPDIF_AUX_EN_IC_1_BYTE                2
#define DSP1_REG_SPDIF_AUX_EN_IC_1_VALUE               0x0

/* SPDIF_RX_AUXBIT_READY  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_AUXBIT_READY_IC_1_ADDR       0xF60F
#define DSP1_REG_SPDIF_RX_AUXBIT_READY_IC_1_BYTE       0
#define DSP1_REG_SPDIF_RX_AUXBIT_READY_IC_1_VALUE      0x0

/* SPDIF_RX_CS_LEFT_0  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_LEFT_0_IC_1_ADDR          0xF610
#define DSP1_REG_SPDIF_RX_CS_LEFT_0_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_CS_LEFT_0_IC_1_VALUE         0x0

/* SPDIF_RX_CS_LEFT_1  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_LEFT_1_IC_1_ADDR          0xF611
#define DSP1_REG_SPDIF_RX_CS_LEFT_1_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_CS_LEFT_1_IC_1_VALUE         0x0

/* SPDIF_RX_CS_LEFT_2  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_LEFT_2_IC_1_ADDR          0xF612
#define DSP1_REG_SPDIF_RX_CS_LEFT_2_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_CS_LEFT_2_IC_1_VALUE         0x0

/* SPDIF_RX_CS_LEFT_3  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_LEFT_3_IC_1_ADDR          0xF613
#define DSP1_REG_SPDIF_RX_CS_LEFT_3_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_CS_LEFT_3_IC_1_VALUE         0x0

/* SPDIF_RX_CS_LEFT_4  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_LEFT_4_IC_1_ADDR          0xF614
#define DSP1_REG_SPDIF_RX_CS_LEFT_4_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_CS_LEFT_4_IC_1_VALUE         0x0

/* SPDIF_RX_CS_LEFT_5  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_LEFT_5_IC_1_ADDR          0xF615
#define DSP1_REG_SPDIF_RX_CS_LEFT_5_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_CS_LEFT_5_IC_1_VALUE         0x0

/* SPDIF_RX_CS_LEFT_6  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_LEFT_6_IC_1_ADDR          0xF616
#define DSP1_REG_SPDIF_RX_CS_LEFT_6_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_CS_LEFT_6_IC_1_VALUE         0x0

/* SPDIF_RX_CS_LEFT_7  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_LEFT_7_IC_1_ADDR          0xF617
#define DSP1_REG_SPDIF_RX_CS_LEFT_7_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_CS_LEFT_7_IC_1_VALUE         0x0

/* SPDIF_RX_CS_LEFT_8  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_LEFT_8_IC_1_ADDR          0xF618
#define DSP1_REG_SPDIF_RX_CS_LEFT_8_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_CS_LEFT_8_IC_1_VALUE         0x0

/* SPDIF_RX_CS_LEFT_9  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_LEFT_9_IC_1_ADDR          0xF619
#define DSP1_REG_SPDIF_RX_CS_LEFT_9_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_CS_LEFT_9_IC_1_VALUE         0x0

/* SPDIF_RX_CS_LEFT_10  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_LEFT_10_IC_1_ADDR         0xF61A
#define DSP1_REG_SPDIF_RX_CS_LEFT_10_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_CS_LEFT_10_IC_1_VALUE        0x0

/* SPDIF_RX_CS_LEFT_11  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_LEFT_11_IC_1_ADDR         0xF61B
#define DSP1_REG_SPDIF_RX_CS_LEFT_11_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_CS_LEFT_11_IC_1_VALUE        0x0

/* SPDIF_RX_CS_RIGHT_0  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_RIGHT_0_IC_1_ADDR         0xF620
#define DSP1_REG_SPDIF_RX_CS_RIGHT_0_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_CS_RIGHT_0_IC_1_VALUE        0x0

/* SPDIF_RX_CS_RIGHT_1  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_RIGHT_1_IC_1_ADDR         0xF621
#define DSP1_REG_SPDIF_RX_CS_RIGHT_1_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_CS_RIGHT_1_IC_1_VALUE        0x0

/* SPDIF_RX_CS_RIGHT_2  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_RIGHT_2_IC_1_ADDR         0xF622
#define DSP1_REG_SPDIF_RX_CS_RIGHT_2_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_CS_RIGHT_2_IC_1_VALUE        0x0

/* SPDIF_RX_CS_RIGHT_3  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_RIGHT_3_IC_1_ADDR         0xF623
#define DSP1_REG_SPDIF_RX_CS_RIGHT_3_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_CS_RIGHT_3_IC_1_VALUE        0x0

/* SPDIF_RX_CS_RIGHT_4  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_RIGHT_4_IC_1_ADDR         0xF624
#define DSP1_REG_SPDIF_RX_CS_RIGHT_4_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_CS_RIGHT_4_IC_1_VALUE        0x0

/* SPDIF_RX_CS_RIGHT_5  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_RIGHT_5_IC_1_ADDR         0xF625
#define DSP1_REG_SPDIF_RX_CS_RIGHT_5_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_CS_RIGHT_5_IC_1_VALUE        0x0

/* SPDIF_RX_CS_RIGHT_6  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_RIGHT_6_IC_1_ADDR         0xF626
#define DSP1_REG_SPDIF_RX_CS_RIGHT_6_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_CS_RIGHT_6_IC_1_VALUE        0x0

/* SPDIF_RX_CS_RIGHT_7  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_RIGHT_7_IC_1_ADDR         0xF627
#define DSP1_REG_SPDIF_RX_CS_RIGHT_7_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_CS_RIGHT_7_IC_1_VALUE        0x0

/* SPDIF_RX_CS_RIGHT_8  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_RIGHT_8_IC_1_ADDR         0xF628
#define DSP1_REG_SPDIF_RX_CS_RIGHT_8_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_CS_RIGHT_8_IC_1_VALUE        0x0

/* SPDIF_RX_CS_RIGHT_9  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_RIGHT_9_IC_1_ADDR         0xF629
#define DSP1_REG_SPDIF_RX_CS_RIGHT_9_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_CS_RIGHT_9_IC_1_VALUE        0x0

/* SPDIF_RX_CS_RIGHT_10  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_RIGHT_10_IC_1_ADDR        0xF62A
#define DSP1_REG_SPDIF_RX_CS_RIGHT_10_IC_1_BYTE        0
#define DSP1_REG_SPDIF_RX_CS_RIGHT_10_IC_1_VALUE       0x0

/* SPDIF_RX_CS_RIGHT_11  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_CS_RIGHT_11_IC_1_ADDR        0xF62B
#define DSP1_REG_SPDIF_RX_CS_RIGHT_11_IC_1_BYTE        0
#define DSP1_REG_SPDIF_RX_CS_RIGHT_11_IC_1_VALUE       0x0

/* SPDIF_RX_UD_LEFT_0  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_LEFT_0_IC_1_ADDR          0xF630
#define DSP1_REG_SPDIF_RX_UD_LEFT_0_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_UD_LEFT_0_IC_1_VALUE         0x0

/* SPDIF_RX_UD_LEFT_1  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_LEFT_1_IC_1_ADDR          0xF631
#define DSP1_REG_SPDIF_RX_UD_LEFT_1_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_UD_LEFT_1_IC_1_VALUE         0x0

/* SPDIF_RX_UD_LEFT_2  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_LEFT_2_IC_1_ADDR          0xF632
#define DSP1_REG_SPDIF_RX_UD_LEFT_2_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_UD_LEFT_2_IC_1_VALUE         0x0

/* SPDIF_RX_UD_LEFT_3  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_LEFT_3_IC_1_ADDR          0xF633
#define DSP1_REG_SPDIF_RX_UD_LEFT_3_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_UD_LEFT_3_IC_1_VALUE         0x0

/* SPDIF_RX_UD_LEFT_4  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_LEFT_4_IC_1_ADDR          0xF634
#define DSP1_REG_SPDIF_RX_UD_LEFT_4_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_UD_LEFT_4_IC_1_VALUE         0x0

/* SPDIF_RX_UD_LEFT_5  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_LEFT_5_IC_1_ADDR          0xF635
#define DSP1_REG_SPDIF_RX_UD_LEFT_5_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_UD_LEFT_5_IC_1_VALUE         0x0

/* SPDIF_RX_UD_LEFT_6  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_LEFT_6_IC_1_ADDR          0xF636
#define DSP1_REG_SPDIF_RX_UD_LEFT_6_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_UD_LEFT_6_IC_1_VALUE         0x0

/* SPDIF_RX_UD_LEFT_7  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_LEFT_7_IC_1_ADDR          0xF637
#define DSP1_REG_SPDIF_RX_UD_LEFT_7_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_UD_LEFT_7_IC_1_VALUE         0x0

/* SPDIF_RX_UD_LEFT_8  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_LEFT_8_IC_1_ADDR          0xF638
#define DSP1_REG_SPDIF_RX_UD_LEFT_8_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_UD_LEFT_8_IC_1_VALUE         0x0

/* SPDIF_RX_UD_LEFT_9  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_LEFT_9_IC_1_ADDR          0xF639
#define DSP1_REG_SPDIF_RX_UD_LEFT_9_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_UD_LEFT_9_IC_1_VALUE         0x0

/* SPDIF_RX_UD_LEFT_10  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_LEFT_10_IC_1_ADDR         0xF63A
#define DSP1_REG_SPDIF_RX_UD_LEFT_10_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_UD_LEFT_10_IC_1_VALUE        0x0

/* SPDIF_RX_UD_LEFT_11  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_LEFT_11_IC_1_ADDR         0xF63B
#define DSP1_REG_SPDIF_RX_UD_LEFT_11_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_UD_LEFT_11_IC_1_VALUE        0x0

/* SPDIF_RX_UD_RIGHT_0  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_RIGHT_0_IC_1_ADDR         0xF640
#define DSP1_REG_SPDIF_RX_UD_RIGHT_0_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_UD_RIGHT_0_IC_1_VALUE        0x0

/* SPDIF_RX_UD_RIGHT_1  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_RIGHT_1_IC_1_ADDR         0xF641
#define DSP1_REG_SPDIF_RX_UD_RIGHT_1_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_UD_RIGHT_1_IC_1_VALUE        0x0

/* SPDIF_RX_UD_RIGHT_2  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_RIGHT_2_IC_1_ADDR         0xF642
#define DSP1_REG_SPDIF_RX_UD_RIGHT_2_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_UD_RIGHT_2_IC_1_VALUE        0x0

/* SPDIF_RX_UD_RIGHT_3  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_RIGHT_3_IC_1_ADDR         0xF643
#define DSP1_REG_SPDIF_RX_UD_RIGHT_3_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_UD_RIGHT_3_IC_1_VALUE        0x0

/* SPDIF_RX_UD_RIGHT_4  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_RIGHT_4_IC_1_ADDR         0xF644
#define DSP1_REG_SPDIF_RX_UD_RIGHT_4_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_UD_RIGHT_4_IC_1_VALUE        0x0

/* SPDIF_RX_UD_RIGHT_5  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_RIGHT_5_IC_1_ADDR         0xF645
#define DSP1_REG_SPDIF_RX_UD_RIGHT_5_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_UD_RIGHT_5_IC_1_VALUE        0x0

/* SPDIF_RX_UD_RIGHT_6  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_RIGHT_6_IC_1_ADDR         0xF646
#define DSP1_REG_SPDIF_RX_UD_RIGHT_6_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_UD_RIGHT_6_IC_1_VALUE        0x0

/* SPDIF_RX_UD_RIGHT_7  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_RIGHT_7_IC_1_ADDR         0xF647
#define DSP1_REG_SPDIF_RX_UD_RIGHT_7_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_UD_RIGHT_7_IC_1_VALUE        0x0

/* SPDIF_RX_UD_RIGHT_8  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_RIGHT_8_IC_1_ADDR         0xF648
#define DSP1_REG_SPDIF_RX_UD_RIGHT_8_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_UD_RIGHT_8_IC_1_VALUE        0x0

/* SPDIF_RX_UD_RIGHT_9  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_RIGHT_9_IC_1_ADDR         0xF649
#define DSP1_REG_SPDIF_RX_UD_RIGHT_9_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_UD_RIGHT_9_IC_1_VALUE        0x0

/* SPDIF_RX_UD_RIGHT_10  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_RIGHT_10_IC_1_ADDR        0xF64A
#define DSP1_REG_SPDIF_RX_UD_RIGHT_10_IC_1_BYTE        0
#define DSP1_REG_SPDIF_RX_UD_RIGHT_10_IC_1_VALUE       0x0

/* SPDIF_RX_UD_RIGHT_11  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_UD_RIGHT_11_IC_1_ADDR        0xF64B
#define DSP1_REG_SPDIF_RX_UD_RIGHT_11_IC_1_BYTE        0
#define DSP1_REG_SPDIF_RX_UD_RIGHT_11_IC_1_VALUE       0x0

/* SPDIF_RX_VB_LEFT_0  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_LEFT_0_IC_1_ADDR          0xF650
#define DSP1_REG_SPDIF_RX_VB_LEFT_0_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_VB_LEFT_0_IC_1_VALUE         0x0

/* SPDIF_RX_VB_LEFT_1  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_LEFT_1_IC_1_ADDR          0xF651
#define DSP1_REG_SPDIF_RX_VB_LEFT_1_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_VB_LEFT_1_IC_1_VALUE         0x0

/* SPDIF_RX_VB_LEFT_2  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_LEFT_2_IC_1_ADDR          0xF652
#define DSP1_REG_SPDIF_RX_VB_LEFT_2_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_VB_LEFT_2_IC_1_VALUE         0x0

/* SPDIF_RX_VB_LEFT_3  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_LEFT_3_IC_1_ADDR          0xF653
#define DSP1_REG_SPDIF_RX_VB_LEFT_3_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_VB_LEFT_3_IC_1_VALUE         0x0

/* SPDIF_RX_VB_LEFT_4  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_LEFT_4_IC_1_ADDR          0xF654
#define DSP1_REG_SPDIF_RX_VB_LEFT_4_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_VB_LEFT_4_IC_1_VALUE         0x0

/* SPDIF_RX_VB_LEFT_5  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_LEFT_5_IC_1_ADDR          0xF655
#define DSP1_REG_SPDIF_RX_VB_LEFT_5_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_VB_LEFT_5_IC_1_VALUE         0x0

/* SPDIF_RX_VB_LEFT_6  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_LEFT_6_IC_1_ADDR          0xF656
#define DSP1_REG_SPDIF_RX_VB_LEFT_6_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_VB_LEFT_6_IC_1_VALUE         0x0

/* SPDIF_RX_VB_LEFT_7  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_LEFT_7_IC_1_ADDR          0xF657
#define DSP1_REG_SPDIF_RX_VB_LEFT_7_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_VB_LEFT_7_IC_1_VALUE         0x0

/* SPDIF_RX_VB_LEFT_8  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_LEFT_8_IC_1_ADDR          0xF658
#define DSP1_REG_SPDIF_RX_VB_LEFT_8_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_VB_LEFT_8_IC_1_VALUE         0x0

/* SPDIF_RX_VB_LEFT_9  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_LEFT_9_IC_1_ADDR          0xF659
#define DSP1_REG_SPDIF_RX_VB_LEFT_9_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_VB_LEFT_9_IC_1_VALUE         0x0

/* SPDIF_RX_VB_LEFT_10  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_LEFT_10_IC_1_ADDR         0xF65A
#define DSP1_REG_SPDIF_RX_VB_LEFT_10_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_VB_LEFT_10_IC_1_VALUE        0x0

/* SPDIF_RX_VB_LEFT_11  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_LEFT_11_IC_1_ADDR         0xF65B
#define DSP1_REG_SPDIF_RX_VB_LEFT_11_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_VB_LEFT_11_IC_1_VALUE        0x0

/* SPDIF_RX_VB_RIGHT_0  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_RIGHT_0_IC_1_ADDR         0xF660
#define DSP1_REG_SPDIF_RX_VB_RIGHT_0_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_VB_RIGHT_0_IC_1_VALUE        0x0

/* SPDIF_RX_VB_RIGHT_1  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_RIGHT_1_IC_1_ADDR         0xF661
#define DSP1_REG_SPDIF_RX_VB_RIGHT_1_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_VB_RIGHT_1_IC_1_VALUE        0x0

/* SPDIF_RX_VB_RIGHT_2  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_RIGHT_2_IC_1_ADDR         0xF662
#define DSP1_REG_SPDIF_RX_VB_RIGHT_2_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_VB_RIGHT_2_IC_1_VALUE        0x0

/* SPDIF_RX_VB_RIGHT_3  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_RIGHT_3_IC_1_ADDR         0xF663
#define DSP1_REG_SPDIF_RX_VB_RIGHT_3_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_VB_RIGHT_3_IC_1_VALUE        0x0

/* SPDIF_RX_VB_RIGHT_4  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_RIGHT_4_IC_1_ADDR         0xF664
#define DSP1_REG_SPDIF_RX_VB_RIGHT_4_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_VB_RIGHT_4_IC_1_VALUE        0x0

/* SPDIF_RX_VB_RIGHT_5  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_RIGHT_5_IC_1_ADDR         0xF665
#define DSP1_REG_SPDIF_RX_VB_RIGHT_5_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_VB_RIGHT_5_IC_1_VALUE        0x0

/* SPDIF_RX_VB_RIGHT_6  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_RIGHT_6_IC_1_ADDR         0xF666
#define DSP1_REG_SPDIF_RX_VB_RIGHT_6_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_VB_RIGHT_6_IC_1_VALUE        0x0

/* SPDIF_RX_VB_RIGHT_7  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_RIGHT_7_IC_1_ADDR         0xF667
#define DSP1_REG_SPDIF_RX_VB_RIGHT_7_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_VB_RIGHT_7_IC_1_VALUE        0x0

/* SPDIF_RX_VB_RIGHT_8  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_RIGHT_8_IC_1_ADDR         0xF668
#define DSP1_REG_SPDIF_RX_VB_RIGHT_8_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_VB_RIGHT_8_IC_1_VALUE        0x0

/* SPDIF_RX_VB_RIGHT_9  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_RIGHT_9_IC_1_ADDR         0xF669
#define DSP1_REG_SPDIF_RX_VB_RIGHT_9_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_VB_RIGHT_9_IC_1_VALUE        0x0

/* SPDIF_RX_VB_RIGHT_10  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_RIGHT_10_IC_1_ADDR        0xF66A
#define DSP1_REG_SPDIF_RX_VB_RIGHT_10_IC_1_BYTE        0
#define DSP1_REG_SPDIF_RX_VB_RIGHT_10_IC_1_VALUE       0x0

/* SPDIF_RX_VB_RIGHT_11  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_VB_RIGHT_11_IC_1_ADDR        0xF66B
#define DSP1_REG_SPDIF_RX_VB_RIGHT_11_IC_1_BYTE        0
#define DSP1_REG_SPDIF_RX_VB_RIGHT_11_IC_1_VALUE       0x0

/* SPDIF_RX_PB_LEFT_0  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_LEFT_0_IC_1_ADDR          0xF670
#define DSP1_REG_SPDIF_RX_PB_LEFT_0_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_PB_LEFT_0_IC_1_VALUE         0x0

/* SPDIF_RX_PB_LEFT_1  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_LEFT_1_IC_1_ADDR          0xF671
#define DSP1_REG_SPDIF_RX_PB_LEFT_1_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_PB_LEFT_1_IC_1_VALUE         0x0

/* SPDIF_RX_PB_LEFT_2  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_LEFT_2_IC_1_ADDR          0xF672
#define DSP1_REG_SPDIF_RX_PB_LEFT_2_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_PB_LEFT_2_IC_1_VALUE         0x0

/* SPDIF_RX_PB_LEFT_3  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_LEFT_3_IC_1_ADDR          0xF673
#define DSP1_REG_SPDIF_RX_PB_LEFT_3_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_PB_LEFT_3_IC_1_VALUE         0x0

/* SPDIF_RX_PB_LEFT_4  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_LEFT_4_IC_1_ADDR          0xF674
#define DSP1_REG_SPDIF_RX_PB_LEFT_4_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_PB_LEFT_4_IC_1_VALUE         0x0

/* SPDIF_RX_PB_LEFT_5  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_LEFT_5_IC_1_ADDR          0xF675
#define DSP1_REG_SPDIF_RX_PB_LEFT_5_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_PB_LEFT_5_IC_1_VALUE         0x0

/* SPDIF_RX_PB_LEFT_6  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_LEFT_6_IC_1_ADDR          0xF676
#define DSP1_REG_SPDIF_RX_PB_LEFT_6_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_PB_LEFT_6_IC_1_VALUE         0x0

/* SPDIF_RX_PB_LEFT_7  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_LEFT_7_IC_1_ADDR          0xF677
#define DSP1_REG_SPDIF_RX_PB_LEFT_7_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_PB_LEFT_7_IC_1_VALUE         0x0

/* SPDIF_RX_PB_LEFT_8  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_LEFT_8_IC_1_ADDR          0xF678
#define DSP1_REG_SPDIF_RX_PB_LEFT_8_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_PB_LEFT_8_IC_1_VALUE         0x0

/* SPDIF_RX_PB_LEFT_9  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_LEFT_9_IC_1_ADDR          0xF679
#define DSP1_REG_SPDIF_RX_PB_LEFT_9_IC_1_BYTE          0
#define DSP1_REG_SPDIF_RX_PB_LEFT_9_IC_1_VALUE         0x0

/* SPDIF_RX_PB_LEFT_10  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_LEFT_10_IC_1_ADDR         0xF67A
#define DSP1_REG_SPDIF_RX_PB_LEFT_10_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_PB_LEFT_10_IC_1_VALUE        0x0

/* SPDIF_RX_PB_LEFT_11  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_LEFT_11_IC_1_ADDR         0xF67B
#define DSP1_REG_SPDIF_RX_PB_LEFT_11_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_PB_LEFT_11_IC_1_VALUE        0x0

/* SPDIF_RX_PB_RIGHT_0  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_RIGHT_0_IC_1_ADDR         0xF680
#define DSP1_REG_SPDIF_RX_PB_RIGHT_0_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_PB_RIGHT_0_IC_1_VALUE        0x0

/* SPDIF_RX_PB_RIGHT_1  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_RIGHT_1_IC_1_ADDR         0xF681
#define DSP1_REG_SPDIF_RX_PB_RIGHT_1_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_PB_RIGHT_1_IC_1_VALUE        0x0

/* SPDIF_RX_PB_RIGHT_2  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_RIGHT_2_IC_1_ADDR         0xF682
#define DSP1_REG_SPDIF_RX_PB_RIGHT_2_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_PB_RIGHT_2_IC_1_VALUE        0x0

/* SPDIF_RX_PB_RIGHT_3  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_RIGHT_3_IC_1_ADDR         0xF683
#define DSP1_REG_SPDIF_RX_PB_RIGHT_3_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_PB_RIGHT_3_IC_1_VALUE        0x0

/* SPDIF_RX_PB_RIGHT_4  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_RIGHT_4_IC_1_ADDR         0xF684
#define DSP1_REG_SPDIF_RX_PB_RIGHT_4_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_PB_RIGHT_4_IC_1_VALUE        0x0

/* SPDIF_RX_PB_RIGHT_5  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_RIGHT_5_IC_1_ADDR         0xF685
#define DSP1_REG_SPDIF_RX_PB_RIGHT_5_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_PB_RIGHT_5_IC_1_VALUE        0x0

/* SPDIF_RX_PB_RIGHT_6  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_RIGHT_6_IC_1_ADDR         0xF686
#define DSP1_REG_SPDIF_RX_PB_RIGHT_6_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_PB_RIGHT_6_IC_1_VALUE        0x0

/* SPDIF_RX_PB_RIGHT_7  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_RIGHT_7_IC_1_ADDR         0xF687
#define DSP1_REG_SPDIF_RX_PB_RIGHT_7_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_PB_RIGHT_7_IC_1_VALUE        0x0

/* SPDIF_RX_PB_RIGHT_8  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_RIGHT_8_IC_1_ADDR         0xF688
#define DSP1_REG_SPDIF_RX_PB_RIGHT_8_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_PB_RIGHT_8_IC_1_VALUE        0x0

/* SPDIF_RX_PB_RIGHT_9  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_RIGHT_9_IC_1_ADDR         0xF689
#define DSP1_REG_SPDIF_RX_PB_RIGHT_9_IC_1_BYTE         0
#define DSP1_REG_SPDIF_RX_PB_RIGHT_9_IC_1_VALUE        0x0

/* SPDIF_RX_PB_RIGHT_10  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_RIGHT_10_IC_1_ADDR        0xF68A
#define DSP1_REG_SPDIF_RX_PB_RIGHT_10_IC_1_BYTE        0
#define DSP1_REG_SPDIF_RX_PB_RIGHT_10_IC_1_VALUE       0x0

/* SPDIF_RX_PB_RIGHT_11  - Registers (IC 1) */
#define DSP1_REG_SPDIF_RX_PB_RIGHT_11_IC_1_ADDR        0xF68B
#define DSP1_REG_SPDIF_RX_PB_RIGHT_11_IC_1_BYTE        0
#define DSP1_REG_SPDIF_RX_PB_RIGHT_11_IC_1_VALUE       0x0

/* SPDIF_TX_EN  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_EN_IC_1_ADDR                 0xF690
#define DSP1_REG_SPDIF_TX_EN_IC_1_BYTE                 2
#define DSP1_REG_SPDIF_TX_EN_IC_1_VALUE                0x0

/* SPDIF_TX_CTRL  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CTRL_IC_1_ADDR               0xF691
#define DSP1_REG_SPDIF_TX_CTRL_IC_1_BYTE               2
#define DSP1_REG_SPDIF_TX_CTRL_IC_1_VALUE              0x0

/* SPDIF_TX_AUXBIT_SOURCE  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_AUXBIT_SOURCE_IC_1_ADDR      0xF69F
#define DSP1_REG_SPDIF_TX_AUXBIT_SOURCE_IC_1_BYTE      2
#define DSP1_REG_SPDIF_TX_AUXBIT_SOURCE_IC_1_VALUE     0x0

/* SPDIF_TX_CS_LEFT_0  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_LEFT_0_IC_1_ADDR          0xF6A0
#define DSP1_REG_SPDIF_TX_CS_LEFT_0_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_CS_LEFT_0_IC_1_VALUE         0x0

/* SPDIF_TX_CS_LEFT_1  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_LEFT_1_IC_1_ADDR          0xF6A1
#define DSP1_REG_SPDIF_TX_CS_LEFT_1_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_CS_LEFT_1_IC_1_VALUE         0x0

/* SPDIF_TX_CS_LEFT_2  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_LEFT_2_IC_1_ADDR          0xF6A2
#define DSP1_REG_SPDIF_TX_CS_LEFT_2_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_CS_LEFT_2_IC_1_VALUE         0x0

/* SPDIF_TX_CS_LEFT_3  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_LEFT_3_IC_1_ADDR          0xF6A3
#define DSP1_REG_SPDIF_TX_CS_LEFT_3_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_CS_LEFT_3_IC_1_VALUE         0x0

/* SPDIF_TX_CS_LEFT_4  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_LEFT_4_IC_1_ADDR          0xF6A4
#define DSP1_REG_SPDIF_TX_CS_LEFT_4_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_CS_LEFT_4_IC_1_VALUE         0x0

/* SPDIF_TX_CS_LEFT_5  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_LEFT_5_IC_1_ADDR          0xF6A5
#define DSP1_REG_SPDIF_TX_CS_LEFT_5_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_CS_LEFT_5_IC_1_VALUE         0x0

/* SPDIF_TX_CS_LEFT_6  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_LEFT_6_IC_1_ADDR          0xF6A6
#define DSP1_REG_SPDIF_TX_CS_LEFT_6_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_CS_LEFT_6_IC_1_VALUE         0x0

/* SPDIF_TX_CS_LEFT_7  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_LEFT_7_IC_1_ADDR          0xF6A7
#define DSP1_REG_SPDIF_TX_CS_LEFT_7_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_CS_LEFT_7_IC_1_VALUE         0x0

/* SPDIF_TX_CS_LEFT_8  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_LEFT_8_IC_1_ADDR          0xF6A8
#define DSP1_REG_SPDIF_TX_CS_LEFT_8_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_CS_LEFT_8_IC_1_VALUE         0x0

/* SPDIF_TX_CS_LEFT_9  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_LEFT_9_IC_1_ADDR          0xF6A9
#define DSP1_REG_SPDIF_TX_CS_LEFT_9_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_CS_LEFT_9_IC_1_VALUE         0x0

/* SPDIF_TX_CS_LEFT_10  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_LEFT_10_IC_1_ADDR         0xF6AA
#define DSP1_REG_SPDIF_TX_CS_LEFT_10_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_CS_LEFT_10_IC_1_VALUE        0x0

/* SPDIF_TX_CS_LEFT_11  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_LEFT_11_IC_1_ADDR         0xF6AB
#define DSP1_REG_SPDIF_TX_CS_LEFT_11_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_CS_LEFT_11_IC_1_VALUE        0x0

/* SPDIF_TX_CS_RIGHT_0  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_RIGHT_0_IC_1_ADDR         0xF6B0
#define DSP1_REG_SPDIF_TX_CS_RIGHT_0_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_CS_RIGHT_0_IC_1_VALUE        0x0

/* SPDIF_TX_CS_RIGHT_1  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_RIGHT_1_IC_1_ADDR         0xF6B1
#define DSP1_REG_SPDIF_TX_CS_RIGHT_1_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_CS_RIGHT_1_IC_1_VALUE        0x0

/* SPDIF_TX_CS_RIGHT_2  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_RIGHT_2_IC_1_ADDR         0xF6B2
#define DSP1_REG_SPDIF_TX_CS_RIGHT_2_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_CS_RIGHT_2_IC_1_VALUE        0x0

/* SPDIF_TX_CS_RIGHT_3  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_RIGHT_3_IC_1_ADDR         0xF6B3
#define DSP1_REG_SPDIF_TX_CS_RIGHT_3_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_CS_RIGHT_3_IC_1_VALUE        0x0

/* SPDIF_TX_CS_RIGHT_4  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_RIGHT_4_IC_1_ADDR         0xF6B4
#define DSP1_REG_SPDIF_TX_CS_RIGHT_4_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_CS_RIGHT_4_IC_1_VALUE        0x0

/* SPDIF_TX_CS_RIGHT_5  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_RIGHT_5_IC_1_ADDR         0xF6B5
#define DSP1_REG_SPDIF_TX_CS_RIGHT_5_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_CS_RIGHT_5_IC_1_VALUE        0x0

/* SPDIF_TX_CS_RIGHT_6  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_RIGHT_6_IC_1_ADDR         0xF6B6
#define DSP1_REG_SPDIF_TX_CS_RIGHT_6_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_CS_RIGHT_6_IC_1_VALUE        0x0

/* SPDIF_TX_CS_RIGHT_7  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_RIGHT_7_IC_1_ADDR         0xF6B7
#define DSP1_REG_SPDIF_TX_CS_RIGHT_7_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_CS_RIGHT_7_IC_1_VALUE        0x0

/* SPDIF_TX_CS_RIGHT_8  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_RIGHT_8_IC_1_ADDR         0xF6B8
#define DSP1_REG_SPDIF_TX_CS_RIGHT_8_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_CS_RIGHT_8_IC_1_VALUE        0x0

/* SPDIF_TX_CS_RIGHT_9  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_RIGHT_9_IC_1_ADDR         0xF6B9
#define DSP1_REG_SPDIF_TX_CS_RIGHT_9_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_CS_RIGHT_9_IC_1_VALUE        0x0

/* SPDIF_TX_CS_RIGHT_10  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_RIGHT_10_IC_1_ADDR        0xF6BA
#define DSP1_REG_SPDIF_TX_CS_RIGHT_10_IC_1_BYTE        2
#define DSP1_REG_SPDIF_TX_CS_RIGHT_10_IC_1_VALUE       0x0

/* SPDIF_TX_CS_RIGHT_11  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_CS_RIGHT_11_IC_1_ADDR        0xF6BB
#define DSP1_REG_SPDIF_TX_CS_RIGHT_11_IC_1_BYTE        2
#define DSP1_REG_SPDIF_TX_CS_RIGHT_11_IC_1_VALUE       0x0

/* SPDIF_TX_UD_LEFT_0  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_LEFT_0_IC_1_ADDR          0xF6C0
#define DSP1_REG_SPDIF_TX_UD_LEFT_0_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_UD_LEFT_0_IC_1_VALUE         0x0

/* SPDIF_TX_UD_LEFT_1  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_LEFT_1_IC_1_ADDR          0xF6C1
#define DSP1_REG_SPDIF_TX_UD_LEFT_1_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_UD_LEFT_1_IC_1_VALUE         0x0

/* SPDIF_TX_UD_LEFT_2  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_LEFT_2_IC_1_ADDR          0xF6C2
#define DSP1_REG_SPDIF_TX_UD_LEFT_2_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_UD_LEFT_2_IC_1_VALUE         0x0

/* SPDIF_TX_UD_LEFT_3  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_LEFT_3_IC_1_ADDR          0xF6C3
#define DSP1_REG_SPDIF_TX_UD_LEFT_3_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_UD_LEFT_3_IC_1_VALUE         0x0

/* SPDIF_TX_UD_LEFT_4  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_LEFT_4_IC_1_ADDR          0xF6C4
#define DSP1_REG_SPDIF_TX_UD_LEFT_4_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_UD_LEFT_4_IC_1_VALUE         0x0

/* SPDIF_TX_UD_LEFT_5  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_LEFT_5_IC_1_ADDR          0xF6C5
#define DSP1_REG_SPDIF_TX_UD_LEFT_5_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_UD_LEFT_5_IC_1_VALUE         0x0

/* SPDIF_TX_UD_LEFT_6  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_LEFT_6_IC_1_ADDR          0xF6C6
#define DSP1_REG_SPDIF_TX_UD_LEFT_6_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_UD_LEFT_6_IC_1_VALUE         0x0

/* SPDIF_TX_UD_LEFT_7  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_LEFT_7_IC_1_ADDR          0xF6C7
#define DSP1_REG_SPDIF_TX_UD_LEFT_7_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_UD_LEFT_7_IC_1_VALUE         0x0

/* SPDIF_TX_UD_LEFT_8  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_LEFT_8_IC_1_ADDR          0xF6C8
#define DSP1_REG_SPDIF_TX_UD_LEFT_8_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_UD_LEFT_8_IC_1_VALUE         0x0

/* SPDIF_TX_UD_LEFT_9  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_LEFT_9_IC_1_ADDR          0xF6C9
#define DSP1_REG_SPDIF_TX_UD_LEFT_9_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_UD_LEFT_9_IC_1_VALUE         0x0

/* SPDIF_TX_UD_LEFT_10  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_LEFT_10_IC_1_ADDR         0xF6CA
#define DSP1_REG_SPDIF_TX_UD_LEFT_10_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_UD_LEFT_10_IC_1_VALUE        0x0

/* SPDIF_TX_UD_LEFT_11  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_LEFT_11_IC_1_ADDR         0xF6CB
#define DSP1_REG_SPDIF_TX_UD_LEFT_11_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_UD_LEFT_11_IC_1_VALUE        0x0

/* SPDIF_TX_UD_RIGHT_0  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_RIGHT_0_IC_1_ADDR         0xF6D0
#define DSP1_REG_SPDIF_TX_UD_RIGHT_0_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_UD_RIGHT_0_IC_1_VALUE        0x0

/* SPDIF_TX_UD_RIGHT_1  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_RIGHT_1_IC_1_ADDR         0xF6D1
#define DSP1_REG_SPDIF_TX_UD_RIGHT_1_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_UD_RIGHT_1_IC_1_VALUE        0x0

/* SPDIF_TX_UD_RIGHT_2  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_RIGHT_2_IC_1_ADDR         0xF6D2
#define DSP1_REG_SPDIF_TX_UD_RIGHT_2_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_UD_RIGHT_2_IC_1_VALUE        0x0

/* SPDIF_TX_UD_RIGHT_3  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_RIGHT_3_IC_1_ADDR         0xF6D3
#define DSP1_REG_SPDIF_TX_UD_RIGHT_3_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_UD_RIGHT_3_IC_1_VALUE        0x0

/* SPDIF_TX_UD_RIGHT_4  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_RIGHT_4_IC_1_ADDR         0xF6D4
#define DSP1_REG_SPDIF_TX_UD_RIGHT_4_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_UD_RIGHT_4_IC_1_VALUE        0x0

/* SPDIF_TX_UD_RIGHT_5  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_RIGHT_5_IC_1_ADDR         0xF6D5
#define DSP1_REG_SPDIF_TX_UD_RIGHT_5_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_UD_RIGHT_5_IC_1_VALUE        0x0

/* SPDIF_TX_UD_RIGHT_6  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_RIGHT_6_IC_1_ADDR         0xF6D6
#define DSP1_REG_SPDIF_TX_UD_RIGHT_6_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_UD_RIGHT_6_IC_1_VALUE        0x0

/* SPDIF_TX_UD_RIGHT_7  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_RIGHT_7_IC_1_ADDR         0xF6D7
#define DSP1_REG_SPDIF_TX_UD_RIGHT_7_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_UD_RIGHT_7_IC_1_VALUE        0x0

/* SPDIF_TX_UD_RIGHT_8  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_RIGHT_8_IC_1_ADDR         0xF6D8
#define DSP1_REG_SPDIF_TX_UD_RIGHT_8_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_UD_RIGHT_8_IC_1_VALUE        0x0

/* SPDIF_TX_UD_RIGHT_9  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_RIGHT_9_IC_1_ADDR         0xF6D9
#define DSP1_REG_SPDIF_TX_UD_RIGHT_9_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_UD_RIGHT_9_IC_1_VALUE        0x0

/* SPDIF_TX_UD_RIGHT_10  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_RIGHT_10_IC_1_ADDR        0xF6DA
#define DSP1_REG_SPDIF_TX_UD_RIGHT_10_IC_1_BYTE        2
#define DSP1_REG_SPDIF_TX_UD_RIGHT_10_IC_1_VALUE       0x0

/* SPDIF_TX_UD_RIGHT_11  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_UD_RIGHT_11_IC_1_ADDR        0xF6DB
#define DSP1_REG_SPDIF_TX_UD_RIGHT_11_IC_1_BYTE        2
#define DSP1_REG_SPDIF_TX_UD_RIGHT_11_IC_1_VALUE       0x0

/* SPDIF_TX_VB_LEFT_0  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_LEFT_0_IC_1_ADDR          0xF6E0
#define DSP1_REG_SPDIF_TX_VB_LEFT_0_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_VB_LEFT_0_IC_1_VALUE         0x0

/* SPDIF_TX_VB_LEFT_1  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_LEFT_1_IC_1_ADDR          0xF6E1
#define DSP1_REG_SPDIF_TX_VB_LEFT_1_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_VB_LEFT_1_IC_1_VALUE         0x0

/* SPDIF_TX_VB_LEFT_2  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_LEFT_2_IC_1_ADDR          0xF6E2
#define DSP1_REG_SPDIF_TX_VB_LEFT_2_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_VB_LEFT_2_IC_1_VALUE         0x0

/* SPDIF_TX_VB_LEFT_3  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_LEFT_3_IC_1_ADDR          0xF6E3
#define DSP1_REG_SPDIF_TX_VB_LEFT_3_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_VB_LEFT_3_IC_1_VALUE         0x0

/* SPDIF_TX_VB_LEFT_4  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_LEFT_4_IC_1_ADDR          0xF6E4
#define DSP1_REG_SPDIF_TX_VB_LEFT_4_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_VB_LEFT_4_IC_1_VALUE         0x0

/* SPDIF_TX_VB_LEFT_5  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_LEFT_5_IC_1_ADDR          0xF6E5
#define DSP1_REG_SPDIF_TX_VB_LEFT_5_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_VB_LEFT_5_IC_1_VALUE         0x0

/* SPDIF_TX_VB_LEFT_6  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_LEFT_6_IC_1_ADDR          0xF6E6
#define DSP1_REG_SPDIF_TX_VB_LEFT_6_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_VB_LEFT_6_IC_1_VALUE         0x0

/* SPDIF_TX_VB_LEFT_7  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_LEFT_7_IC_1_ADDR          0xF6E7
#define DSP1_REG_SPDIF_TX_VB_LEFT_7_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_VB_LEFT_7_IC_1_VALUE         0x0

/* SPDIF_TX_VB_LEFT_8  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_LEFT_8_IC_1_ADDR          0xF6E8
#define DSP1_REG_SPDIF_TX_VB_LEFT_8_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_VB_LEFT_8_IC_1_VALUE         0x0

/* SPDIF_TX_VB_LEFT_9  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_LEFT_9_IC_1_ADDR          0xF6E9
#define DSP1_REG_SPDIF_TX_VB_LEFT_9_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_VB_LEFT_9_IC_1_VALUE         0x0

/* SPDIF_TX_VB_LEFT_10  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_LEFT_10_IC_1_ADDR         0xF6EA
#define DSP1_REG_SPDIF_TX_VB_LEFT_10_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_VB_LEFT_10_IC_1_VALUE        0x0

/* SPDIF_TX_VB_LEFT_11  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_LEFT_11_IC_1_ADDR         0xF6EB
#define DSP1_REG_SPDIF_TX_VB_LEFT_11_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_VB_LEFT_11_IC_1_VALUE        0x0

/* SPDIF_TX_VB_RIGHT_0  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_RIGHT_0_IC_1_ADDR         0xF6F0
#define DSP1_REG_SPDIF_TX_VB_RIGHT_0_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_VB_RIGHT_0_IC_1_VALUE        0x0

/* SPDIF_TX_VB_RIGHT_1  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_RIGHT_1_IC_1_ADDR         0xF6F1
#define DSP1_REG_SPDIF_TX_VB_RIGHT_1_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_VB_RIGHT_1_IC_1_VALUE        0x0

/* SPDIF_TX_VB_RIGHT_2  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_RIGHT_2_IC_1_ADDR         0xF6F2
#define DSP1_REG_SPDIF_TX_VB_RIGHT_2_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_VB_RIGHT_2_IC_1_VALUE        0x0

/* SPDIF_TX_VB_RIGHT_3  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_RIGHT_3_IC_1_ADDR         0xF6F3
#define DSP1_REG_SPDIF_TX_VB_RIGHT_3_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_VB_RIGHT_3_IC_1_VALUE        0x0

/* SPDIF_TX_VB_RIGHT_4  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_RIGHT_4_IC_1_ADDR         0xF6F4
#define DSP1_REG_SPDIF_TX_VB_RIGHT_4_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_VB_RIGHT_4_IC_1_VALUE        0x0

/* SPDIF_TX_VB_RIGHT_5  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_RIGHT_5_IC_1_ADDR         0xF6F5
#define DSP1_REG_SPDIF_TX_VB_RIGHT_5_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_VB_RIGHT_5_IC_1_VALUE        0x0

/* SPDIF_TX_VB_RIGHT_6  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_RIGHT_6_IC_1_ADDR         0xF6F6
#define DSP1_REG_SPDIF_TX_VB_RIGHT_6_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_VB_RIGHT_6_IC_1_VALUE        0x0

/* SPDIF_TX_VB_RIGHT_7  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_RIGHT_7_IC_1_ADDR         0xF6F7
#define DSP1_REG_SPDIF_TX_VB_RIGHT_7_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_VB_RIGHT_7_IC_1_VALUE        0x0

/* SPDIF_TX_VB_RIGHT_8  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_RIGHT_8_IC_1_ADDR         0xF6F8
#define DSP1_REG_SPDIF_TX_VB_RIGHT_8_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_VB_RIGHT_8_IC_1_VALUE        0x0

/* SPDIF_TX_VB_RIGHT_9  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_RIGHT_9_IC_1_ADDR         0xF6F9
#define DSP1_REG_SPDIF_TX_VB_RIGHT_9_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_VB_RIGHT_9_IC_1_VALUE        0x0

/* SPDIF_TX_VB_RIGHT_10  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_RIGHT_10_IC_1_ADDR        0xF6FA
#define DSP1_REG_SPDIF_TX_VB_RIGHT_10_IC_1_BYTE        2
#define DSP1_REG_SPDIF_TX_VB_RIGHT_10_IC_1_VALUE       0x0

/* SPDIF_TX_VB_RIGHT_11  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_VB_RIGHT_11_IC_1_ADDR        0xF6FB
#define DSP1_REG_SPDIF_TX_VB_RIGHT_11_IC_1_BYTE        2
#define DSP1_REG_SPDIF_TX_VB_RIGHT_11_IC_1_VALUE       0x0

/* SPDIF_TX_PB_LEFT_0  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_LEFT_0_IC_1_ADDR          0xF700
#define DSP1_REG_SPDIF_TX_PB_LEFT_0_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_PB_LEFT_0_IC_1_VALUE         0x0

/* SPDIF_TX_PB_LEFT_1  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_LEFT_1_IC_1_ADDR          0xF701
#define DSP1_REG_SPDIF_TX_PB_LEFT_1_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_PB_LEFT_1_IC_1_VALUE         0x0

/* SPDIF_TX_PB_LEFT_2  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_LEFT_2_IC_1_ADDR          0xF702
#define DSP1_REG_SPDIF_TX_PB_LEFT_2_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_PB_LEFT_2_IC_1_VALUE         0x0

/* SPDIF_TX_PB_LEFT_3  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_LEFT_3_IC_1_ADDR          0xF703
#define DSP1_REG_SPDIF_TX_PB_LEFT_3_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_PB_LEFT_3_IC_1_VALUE         0x0

/* SPDIF_TX_PB_LEFT_4  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_LEFT_4_IC_1_ADDR          0xF704
#define DSP1_REG_SPDIF_TX_PB_LEFT_4_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_PB_LEFT_4_IC_1_VALUE         0x0

/* SPDIF_TX_PB_LEFT_5  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_LEFT_5_IC_1_ADDR          0xF705
#define DSP1_REG_SPDIF_TX_PB_LEFT_5_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_PB_LEFT_5_IC_1_VALUE         0x0

/* SPDIF_TX_PB_LEFT_6  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_LEFT_6_IC_1_ADDR          0xF706
#define DSP1_REG_SPDIF_TX_PB_LEFT_6_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_PB_LEFT_6_IC_1_VALUE         0x0

/* SPDIF_TX_PB_LEFT_7  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_LEFT_7_IC_1_ADDR          0xF707
#define DSP1_REG_SPDIF_TX_PB_LEFT_7_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_PB_LEFT_7_IC_1_VALUE         0x0

/* SPDIF_TX_PB_LEFT_8  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_LEFT_8_IC_1_ADDR          0xF708
#define DSP1_REG_SPDIF_TX_PB_LEFT_8_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_PB_LEFT_8_IC_1_VALUE         0x0

/* SPDIF_TX_PB_LEFT_9  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_LEFT_9_IC_1_ADDR          0xF709
#define DSP1_REG_SPDIF_TX_PB_LEFT_9_IC_1_BYTE          2
#define DSP1_REG_SPDIF_TX_PB_LEFT_9_IC_1_VALUE         0x0

/* SPDIF_TX_PB_LEFT_10  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_LEFT_10_IC_1_ADDR         0xF70A
#define DSP1_REG_SPDIF_TX_PB_LEFT_10_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_PB_LEFT_10_IC_1_VALUE        0x0

/* SPDIF_TX_PB_LEFT_11  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_LEFT_11_IC_1_ADDR         0xF70B
#define DSP1_REG_SPDIF_TX_PB_LEFT_11_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_PB_LEFT_11_IC_1_VALUE        0x0

/* SPDIF_TX_PB_RIGHT_0  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_RIGHT_0_IC_1_ADDR         0xF710
#define DSP1_REG_SPDIF_TX_PB_RIGHT_0_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_PB_RIGHT_0_IC_1_VALUE        0x0

/* SPDIF_TX_PB_RIGHT_1  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_RIGHT_1_IC_1_ADDR         0xF711
#define DSP1_REG_SPDIF_TX_PB_RIGHT_1_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_PB_RIGHT_1_IC_1_VALUE        0x0

/* SPDIF_TX_PB_RIGHT_2  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_RIGHT_2_IC_1_ADDR         0xF712
#define DSP1_REG_SPDIF_TX_PB_RIGHT_2_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_PB_RIGHT_2_IC_1_VALUE        0x0

/* SPDIF_TX_PB_RIGHT_3  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_RIGHT_3_IC_1_ADDR         0xF713
#define DSP1_REG_SPDIF_TX_PB_RIGHT_3_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_PB_RIGHT_3_IC_1_VALUE        0x0

/* SPDIF_TX_PB_RIGHT_4  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_RIGHT_4_IC_1_ADDR         0xF714
#define DSP1_REG_SPDIF_TX_PB_RIGHT_4_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_PB_RIGHT_4_IC_1_VALUE        0x0

/* SPDIF_TX_PB_RIGHT_5  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_RIGHT_5_IC_1_ADDR         0xF715
#define DSP1_REG_SPDIF_TX_PB_RIGHT_5_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_PB_RIGHT_5_IC_1_VALUE        0x0

/* SPDIF_TX_PB_RIGHT_6  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_RIGHT_6_IC_1_ADDR         0xF716
#define DSP1_REG_SPDIF_TX_PB_RIGHT_6_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_PB_RIGHT_6_IC_1_VALUE        0x0

/* SPDIF_TX_PB_RIGHT_7  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_RIGHT_7_IC_1_ADDR         0xF717
#define DSP1_REG_SPDIF_TX_PB_RIGHT_7_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_PB_RIGHT_7_IC_1_VALUE        0x0

/* SPDIF_TX_PB_RIGHT_8  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_RIGHT_8_IC_1_ADDR         0xF718
#define DSP1_REG_SPDIF_TX_PB_RIGHT_8_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_PB_RIGHT_8_IC_1_VALUE        0x0

/* SPDIF_TX_PB_RIGHT_9  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_RIGHT_9_IC_1_ADDR         0xF719
#define DSP1_REG_SPDIF_TX_PB_RIGHT_9_IC_1_BYTE         2
#define DSP1_REG_SPDIF_TX_PB_RIGHT_9_IC_1_VALUE        0x0

/* SPDIF_TX_PB_RIGHT_10  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_RIGHT_10_IC_1_ADDR        0xF71A
#define DSP1_REG_SPDIF_TX_PB_RIGHT_10_IC_1_BYTE        2
#define DSP1_REG_SPDIF_TX_PB_RIGHT_10_IC_1_VALUE       0x0

/* SPDIF_TX_PB_RIGHT_11  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PB_RIGHT_11_IC_1_ADDR        0xF71B
#define DSP1_REG_SPDIF_TX_PB_RIGHT_11_IC_1_BYTE        2
#define DSP1_REG_SPDIF_TX_PB_RIGHT_11_IC_1_VALUE       0x0

/* BCLK_IN0_PIN  - Registers (IC 1) */
#define DSP1_REG_BCLK_IN0_PIN_IC_1_ADDR                0xF780
#define DSP1_REG_BCLK_IN0_PIN_IC_1_BYTE                2
#define DSP1_REG_BCLK_IN0_PIN_IC_1_VALUE               0x18

/* BCLK_IN1_PIN  - Registers (IC 1) */
#define DSP1_REG_BCLK_IN1_PIN_IC_1_ADDR                0xF781
#define DSP1_REG_BCLK_IN1_PIN_IC_1_BYTE                2
#define DSP1_REG_BCLK_IN1_PIN_IC_1_VALUE               0x18

/* BCLK_IN2_PIN  - Registers (IC 1) */
#define DSP1_REG_BCLK_IN2_PIN_IC_1_ADDR                0xF782
#define DSP1_REG_BCLK_IN2_PIN_IC_1_BYTE                2
#define DSP1_REG_BCLK_IN2_PIN_IC_1_VALUE               0x18

/* BCLK_IN3_PIN  - Registers (IC 1) */
#define DSP1_REG_BCLK_IN3_PIN_IC_1_ADDR                0xF783
#define DSP1_REG_BCLK_IN3_PIN_IC_1_BYTE                2
#define DSP1_REG_BCLK_IN3_PIN_IC_1_VALUE               0x18

/* BCLK_OUT0_PIN  - Registers (IC 1) */
#define DSP1_REG_BCLK_OUT0_PIN_IC_1_ADDR               0xF784
#define DSP1_REG_BCLK_OUT0_PIN_IC_1_BYTE               2
#define DSP1_REG_BCLK_OUT0_PIN_IC_1_VALUE              0x18

/* BCLK_OUT1_PIN  - Registers (IC 1) */
#define DSP1_REG_BCLK_OUT1_PIN_IC_1_ADDR               0xF785
#define DSP1_REG_BCLK_OUT1_PIN_IC_1_BYTE               2
#define DSP1_REG_BCLK_OUT1_PIN_IC_1_VALUE              0x18

/* BCLK_OUT2_PIN  - Registers (IC 1) */
#define DSP1_REG_BCLK_OUT2_PIN_IC_1_ADDR               0xF786
#define DSP1_REG_BCLK_OUT2_PIN_IC_1_BYTE               2
#define DSP1_REG_BCLK_OUT2_PIN_IC_1_VALUE              0x18

/* BCLK_OUT3_PIN  - Registers (IC 1) */
#define DSP1_REG_BCLK_OUT3_PIN_IC_1_ADDR               0xF787
#define DSP1_REG_BCLK_OUT3_PIN_IC_1_BYTE               2
#define DSP1_REG_BCLK_OUT3_PIN_IC_1_VALUE              0x18

/* LRCLK_IN0_PIN  - Registers (IC 1) */
#define DSP1_REG_LRCLK_IN0_PIN_IC_1_ADDR               0xF788
#define DSP1_REG_LRCLK_IN0_PIN_IC_1_BYTE               2
#define DSP1_REG_LRCLK_IN0_PIN_IC_1_VALUE              0x18

/* LRCLK_IN1_PIN  - Registers (IC 1) */
#define DSP1_REG_LRCLK_IN1_PIN_IC_1_ADDR               0xF789
#define DSP1_REG_LRCLK_IN1_PIN_IC_1_BYTE               2
#define DSP1_REG_LRCLK_IN1_PIN_IC_1_VALUE              0x18

/* LRCLK_IN2_PIN  - Registers (IC 1) */
#define DSP1_REG_LRCLK_IN2_PIN_IC_1_ADDR               0xF78A
#define DSP1_REG_LRCLK_IN2_PIN_IC_1_BYTE               2
#define DSP1_REG_LRCLK_IN2_PIN_IC_1_VALUE              0x18

/* LRCLK_IN3_PIN  - Registers (IC 1) */
#define DSP1_REG_LRCLK_IN3_PIN_IC_1_ADDR               0xF78B
#define DSP1_REG_LRCLK_IN3_PIN_IC_1_BYTE               2
#define DSP1_REG_LRCLK_IN3_PIN_IC_1_VALUE              0x18

/* LRCLK_OUT0_PIN  - Registers (IC 1) */
#define DSP1_REG_LRCLK_OUT0_PIN_IC_1_ADDR              0xF78C
#define DSP1_REG_LRCLK_OUT0_PIN_IC_1_BYTE              2
#define DSP1_REG_LRCLK_OUT0_PIN_IC_1_VALUE             0x18

/* LRCLK_OUT1_PIN  - Registers (IC 1) */
#define DSP1_REG_LRCLK_OUT1_PIN_IC_1_ADDR              0xF78D
#define DSP1_REG_LRCLK_OUT1_PIN_IC_1_BYTE              2
#define DSP1_REG_LRCLK_OUT1_PIN_IC_1_VALUE             0x18

/* LRCLK_OUT2_PIN  - Registers (IC 1) */
#define DSP1_REG_LRCLK_OUT2_PIN_IC_1_ADDR              0xF78E
#define DSP1_REG_LRCLK_OUT2_PIN_IC_1_BYTE              2
#define DSP1_REG_LRCLK_OUT2_PIN_IC_1_VALUE             0x18

/* LRCLK_OUT3_PIN  - Registers (IC 1) */
#define DSP1_REG_LRCLK_OUT3_PIN_IC_1_ADDR              0xF78F
#define DSP1_REG_LRCLK_OUT3_PIN_IC_1_BYTE              2
#define DSP1_REG_LRCLK_OUT3_PIN_IC_1_VALUE             0x18

/* SDATA_IN0_PIN  - Registers (IC 1) */
#define DSP1_REG_SDATA_IN0_PIN_IC_1_ADDR               0xF790
#define DSP1_REG_SDATA_IN0_PIN_IC_1_BYTE               2
#define DSP1_REG_SDATA_IN0_PIN_IC_1_VALUE              0x18

/* SDATA_IN1_PIN  - Registers (IC 1) */
#define DSP1_REG_SDATA_IN1_PIN_IC_1_ADDR               0xF791
#define DSP1_REG_SDATA_IN1_PIN_IC_1_BYTE               2
#define DSP1_REG_SDATA_IN1_PIN_IC_1_VALUE              0x18

/* SDATA_IN2_PIN  - Registers (IC 1) */
#define DSP1_REG_SDATA_IN2_PIN_IC_1_ADDR               0xF792
#define DSP1_REG_SDATA_IN2_PIN_IC_1_BYTE               2
#define DSP1_REG_SDATA_IN2_PIN_IC_1_VALUE              0x18

/* SDATA_IN3_PIN  - Registers (IC 1) */
#define DSP1_REG_SDATA_IN3_PIN_IC_1_ADDR               0xF793
#define DSP1_REG_SDATA_IN3_PIN_IC_1_BYTE               2
#define DSP1_REG_SDATA_IN3_PIN_IC_1_VALUE              0x18

/* SDATA_OUT0_PIN  - Registers (IC 1) */
#define DSP1_REG_SDATA_OUT0_PIN_IC_1_ADDR              0xF794
#define DSP1_REG_SDATA_OUT0_PIN_IC_1_BYTE              2
#define DSP1_REG_SDATA_OUT0_PIN_IC_1_VALUE             0x8

/* SDATA_OUT1_PIN  - Registers (IC 1) */
#define DSP1_REG_SDATA_OUT1_PIN_IC_1_ADDR              0xF795
#define DSP1_REG_SDATA_OUT1_PIN_IC_1_BYTE              2
#define DSP1_REG_SDATA_OUT1_PIN_IC_1_VALUE             0x8

/* SDATA_OUT2_PIN  - Registers (IC 1) */
#define DSP1_REG_SDATA_OUT2_PIN_IC_1_ADDR              0xF796
#define DSP1_REG_SDATA_OUT2_PIN_IC_1_BYTE              2
#define DSP1_REG_SDATA_OUT2_PIN_IC_1_VALUE             0x8

/* SDATA_OUT3_PIN  - Registers (IC 1) */
#define DSP1_REG_SDATA_OUT3_PIN_IC_1_ADDR              0xF797
#define DSP1_REG_SDATA_OUT3_PIN_IC_1_BYTE              2
#define DSP1_REG_SDATA_OUT3_PIN_IC_1_VALUE             0x8

/* SPDIF_TX_PIN  - Registers (IC 1) */
#define DSP1_REG_SPDIF_TX_PIN_IC_1_ADDR                0xF798
#define DSP1_REG_SPDIF_TX_PIN_IC_1_BYTE                2
#define DSP1_REG_SPDIF_TX_PIN_IC_1_VALUE               0x8

/* SCLK_SCL_PIN  - Registers (IC 1) */
#define DSP1_REG_SCLK_SCL_PIN_IC_1_ADDR                0xF799
#define DSP1_REG_SCLK_SCL_PIN_IC_1_BYTE                2
#define DSP1_REG_SCLK_SCL_PIN_IC_1_VALUE               0x8

/* MISO_SDA_PIN  - Registers (IC 1) */
#define DSP1_REG_MISO_SDA_PIN_IC_1_ADDR                0xF79A
#define DSP1_REG_MISO_SDA_PIN_IC_1_BYTE                2
#define DSP1_REG_MISO_SDA_PIN_IC_1_VALUE               0x8

/* SS_PIN  - Registers (IC 1) */
#define DSP1_REG_SS_PIN_IC_1_ADDR                      0xF79B
#define DSP1_REG_SS_PIN_IC_1_BYTE                      2
#define DSP1_REG_SS_PIN_IC_1_VALUE                     0x18

/* MOSI_ADDR1_PIN  - Registers (IC 1) */
#define DSP1_REG_MOSI_ADDR1_PIN_IC_1_ADDR              0xF79C
#define DSP1_REG_MOSI_ADDR1_PIN_IC_1_BYTE              2
#define DSP1_REG_MOSI_ADDR1_PIN_IC_1_VALUE             0x18

/* SCLK_SCL_M_PIN  - Registers (IC 1) */
#define DSP1_REG_SCLK_SCL_M_PIN_IC_1_ADDR              0xF79D
#define DSP1_REG_SCLK_SCL_M_PIN_IC_1_BYTE              2
#define DSP1_REG_SCLK_SCL_M_PIN_IC_1_VALUE             0x8

/* MISO_SDA_M_PIN  - Registers (IC 1) */
#define DSP1_REG_MISO_SDA_M_PIN_IC_1_ADDR              0xF79E
#define DSP1_REG_MISO_SDA_M_PIN_IC_1_BYTE              2
#define DSP1_REG_MISO_SDA_M_PIN_IC_1_VALUE             0x8

/* SS_M_PIN  - Registers (IC 1) */
#define DSP1_REG_SS_M_PIN_IC_1_ADDR                    0xF79F
#define DSP1_REG_SS_M_PIN_IC_1_BYTE                    2
#define DSP1_REG_SS_M_PIN_IC_1_VALUE                   0x18

/* MOSI_M_PIN  - Registers (IC 1) */
#define DSP1_REG_MOSI_M_PIN_IC_1_ADDR                  0xF7A0
#define DSP1_REG_MOSI_M_PIN_IC_1_BYTE                  2
#define DSP1_REG_MOSI_M_PIN_IC_1_VALUE                 0x18

/* MP6_PIN  - Registers (IC 1) */
#define DSP1_REG_MP6_PIN_IC_1_ADDR                     0xF7A1
#define DSP1_REG_MP6_PIN_IC_1_BYTE                     2
#define DSP1_REG_MP6_PIN_IC_1_VALUE                    0x18

/* MP7_PIN  - Registers (IC 1) */
#define DSP1_REG_MP7_PIN_IC_1_ADDR                     0xF7A2
#define DSP1_REG_MP7_PIN_IC_1_BYTE                     2
#define DSP1_REG_MP7_PIN_IC_1_VALUE                    0x18

/* CLKOUT_PIN  - Registers (IC 1) */
#define DSP1_REG_CLKOUT_PIN_IC_1_ADDR                  0xF7A3
#define DSP1_REG_CLKOUT_PIN_IC_1_BYTE                  2
#define DSP1_REG_CLKOUT_PIN_IC_1_VALUE                 0x8

/* SOFT_RESET  - Registers (IC 1) */
#define DSP1_REG_SOFT_RESET_IC_1_ADDR                  0xF890
#define DSP1_REG_SOFT_RESET_IC_1_BYTE                  2
#define DSP1_REG_SOFT_RESET_IC_1_VALUE                 0x1

/* SECOND_PAGE_ENABLE  - Registers (IC 1) */
#define DSP1_REG_SECOND_PAGE_ENABLE_IC_1_ADDR          0xF899
#define DSP1_REG_SECOND_PAGE_ENABLE_IC_1_BYTE          2
#define DSP1_REG_SECOND_PAGE_ENABLE_IC_1_VALUE         0x0


/*
 *
 * Control register's field descriptions
 *
 */

/* PLL_CTRL0 (IC 1) */
#define DSP1_R0_PLL_CTRL0_PLL_FBDIVIDER_IC_1           0x60   /* 1100000b	[6:0] */
#define DSP1_R0_PLL_CTRL0_PLL_FBDIVIDER_IC_1_MASK      0x7F
#define DSP1_R0_PLL_CTRL0_PLL_FBDIVIDER_IC_1_SHIFT     0

/* PLL_CTRL1 (IC 1) */
#define DSP1_R1_PLL_CTRL1_PLL_DIV_IC_1                 0x2    /* 10b	[1:0] */
#define DSP1_R1_PLL_CTRL1_PLL_DIV_IC_1_MASK            0x3
#define DSP1_R1_PLL_CTRL1_PLL_DIV_IC_1_SHIFT           0

/* PLL_CLK_SRC (IC 1) */
#define DSP1_R2_PLL_CLK_SRC_CLKSRC_IC_1                0x1    /* 1b	[0] */
#define DSP1_R2_PLL_CLK_SRC_CLKSRC_IC_1_MASK           0x1
#define DSP1_R2_PLL_CLK_SRC_CLKSRC_IC_1_SHIFT          0

/* PLL_ENABLE (IC 1) */
#define DSP1_R3_PLL_ENABLE_PLL_ENABLE_IC_1             0x1    /* 1b	[0] */
#define DSP1_R3_PLL_ENABLE_PLL_ENABLE_IC_1_MASK        0x1
#define DSP1_R3_PLL_ENABLE_PLL_ENABLE_IC_1_SHIFT       0

/* PLL_LOCK (IC 1) */
#define DSP1_R4_PLL_LOCK_PLL_LOCK_IC_1                 0x0    /* 0b	[0] */
#define DSP1_R4_PLL_LOCK_PLL_LOCK_IC_1_MASK            0x1
#define DSP1_R4_PLL_LOCK_PLL_LOCK_IC_1_SHIFT           0

/* MCLK_OUT (IC 1) */
#define DSP1_R5_MCLK_OUT_CLKOUT_ENABLE_IC_1            0x1    /* 1b	[0] */
#define DSP1_R5_MCLK_OUT_CLKOUT_RATE_IC_1              0x2    /* 10b	[2:1] */
#define DSP1_R5_MCLK_OUT_CLKOUT_ENABLE_IC_1_MASK       0x1
#define DSP1_R5_MCLK_OUT_CLKOUT_ENABLE_IC_1_SHIFT      0
#define DSP1_R5_MCLK_OUT_CLKOUT_RATE_IC_1_MASK         0x6
#define DSP1_R5_MCLK_OUT_CLKOUT_RATE_IC_1_SHIFT        1

/* PLL_WATCHDOG (IC 1) */
#define DSP1_R6_PLL_WATCHDOG_PLL_WATCHDOG_IC_1         0x1    /* 1b	[0] */
#define DSP1_R6_PLL_WATCHDOG_PLL_WATCHDOG_IC_1_MASK    0x1
#define DSP1_R6_PLL_WATCHDOG_PLL_WATCHDOG_IC_1_SHIFT   0

/* CLK_GEN1_M (IC 1) */
#define DSP1_R7_CLK_GEN1_M_CLOCKGEN1_M_IC_1            0x06   /* 000000110b	[8:0] */
#define DSP1_R7_CLK_GEN1_M_CLOCKGEN1_M_IC_1_MASK       0x1FF
#define DSP1_R7_CLK_GEN1_M_CLOCKGEN1_M_IC_1_SHIFT      0

/* CLK_GEN1_N (IC 1) */
#define DSP1_R8_CLK_GEN1_N_CLOCKGEN1_N_IC_1            0x01   /* 000000001b	[8:0] */
#define DSP1_R8_CLK_GEN1_N_CLOCKGEN1_N_IC_1_MASK       0x1FF
#define DSP1_R8_CLK_GEN1_N_CLOCKGEN1_N_IC_1_SHIFT      0

/* CLK_GEN2_M (IC 1) */
#define DSP1_R9_CLK_GEN2_M_CLOCKGEN2_M_IC_1            0x06   /* 000000110b	[8:0] */
#define DSP1_R9_CLK_GEN2_M_CLOCKGEN2_M_IC_1_MASK       0x1FF
#define DSP1_R9_CLK_GEN2_M_CLOCKGEN2_M_IC_1_SHIFT      0

/* CLK_GEN2_N (IC 1) */
#define DSP1_R10_CLK_GEN2_N_CLOCKGEN2_N_IC_1           0x01   /* 000000001b	[8:0] */
#define DSP1_R10_CLK_GEN2_N_CLOCKGEN2_N_IC_1_MASK      0x1FF
#define DSP1_R10_CLK_GEN2_N_CLOCKGEN2_N_IC_1_SHIFT     0

/* CLK_GEN3_M (IC 1) */
#define DSP1_R11_CLK_GEN3_M_CLOCKGEN3_M_IC_1           0x0006 /* 0000000000000110b	[15:0] */
#define DSP1_R11_CLK_GEN3_M_CLOCKGEN3_M_IC_1_MASK      0xFFFF
#define DSP1_R11_CLK_GEN3_M_CLOCKGEN3_M_IC_1_SHIFT     0

/* CLK_GEN3_N (IC 1) */
#define DSP1_R12_CLK_GEN3_N_CLOCKGEN3_N_IC_1           0x0001 /* 0000000000000001b	[15:0] */
#define DSP1_R12_CLK_GEN3_N_CLOCKGEN3_N_IC_1_MASK      0xFFFF
#define DSP1_R12_CLK_GEN3_N_CLOCKGEN3_N_IC_1_SHIFT     0

/* CLK_GEN3_SRC (IC 1) */
#define DSP1_R13_CLK_GEN3_SRC_FREF_PIN_IC_1            0xE    /* 1110b	[3:0] */
#define DSP1_R13_CLK_GEN3_SRC_CLK_GEN3_SRC_IC_1        0x0    /* 0b	[4] */
#define DSP1_R13_CLK_GEN3_SRC_FREF_PIN_IC_1_MASK       0xF
#define DSP1_R13_CLK_GEN3_SRC_FREF_PIN_IC_1_SHIFT      0
#define DSP1_R13_CLK_GEN3_SRC_CLK_GEN3_SRC_IC_1_MASK   0x10
#define DSP1_R13_CLK_GEN3_SRC_CLK_GEN3_SRC_IC_1_SHIFT  4

/* CLK_GEN3_LOCK (IC 1) */
#define DSP1_R14_CLK_GEN3_LOCK_GEN3_LOCK_IC_1          0x0    /* 0b	[0] */
#define DSP1_R14_CLK_GEN3_LOCK_GEN3_LOCK_IC_1_MASK     0x1
#define DSP1_R14_CLK_GEN3_LOCK_GEN3_LOCK_IC_1_SHIFT    0

/* POWER_ENABLE0 (IC 1) */
#define DSP1_R15_POWER_ENABLE0_SIN0_PWR_IC_1           0x1    /* 1b	[0] */
#define DSP1_R15_POWER_ENABLE0_SIN1_PWR_IC_1           0x1    /* 1b	[1] */
#define DSP1_R15_POWER_ENABLE0_SIN2_PWR_IC_1           0x1    /* 1b	[2] */
#define DSP1_R15_POWER_ENABLE0_SIN3_PWR_IC_1           0x1    /* 1b	[3] */
#define DSP1_R15_POWER_ENABLE0_SOUT0_PWR_IC_1          0x1    /* 1b	[4] */
#define DSP1_R15_POWER_ENABLE0_SOUT1_PWR_IC_1          0x1    /* 1b	[5] */
#define DSP1_R15_POWER_ENABLE0_SOUT2_PWR_IC_1          0x1    /* 1b	[6] */
#define DSP1_R15_POWER_ENABLE0_SOUT3_PWR_IC_1          0x1    /* 1b	[7] */
#define DSP1_R15_POWER_ENABLE0_ASRCBANK0_PWR_IC_1      0x1    /* 1b	[8] */
#define DSP1_R15_POWER_ENABLE0_ASRCBANK1_PWR_IC_1      0x1    /* 1b	[9] */
#define DSP1_R15_POWER_ENABLE0_CLK_GEN1_PWR_IC_1       0x1    /* 1b	[10] */
#define DSP1_R15_POWER_ENABLE0_CLK_GEN2_PWR_IC_1       0x1    /* 1b	[11] */
#define DSP1_R15_POWER_ENABLE0_CLK_GEN3_PWR_IC_1       0x1    /* 1b	[12] */
#define DSP1_R15_POWER_ENABLE0_SIN0_PWR_IC_1_MASK      0x1
#define DSP1_R15_POWER_ENABLE0_SIN0_PWR_IC_1_SHIFT     0
#define DSP1_R15_POWER_ENABLE0_SIN1_PWR_IC_1_MASK      0x2
#define DSP1_R15_POWER_ENABLE0_SIN1_PWR_IC_1_SHIFT     1
#define DSP1_R15_POWER_ENABLE0_SIN2_PWR_IC_1_MASK      0x4
#define DSP1_R15_POWER_ENABLE0_SIN2_PWR_IC_1_SHIFT     2
#define DSP1_R15_POWER_ENABLE0_SIN3_PWR_IC_1_MASK      0x8
#define DSP1_R15_POWER_ENABLE0_SIN3_PWR_IC_1_SHIFT     3
#define DSP1_R15_POWER_ENABLE0_SOUT0_PWR_IC_1_MASK     0x10
#define DSP1_R15_POWER_ENABLE0_SOUT0_PWR_IC_1_SHIFT    4
#define DSP1_R15_POWER_ENABLE0_SOUT1_PWR_IC_1_MASK     0x20
#define DSP1_R15_POWER_ENABLE0_SOUT1_PWR_IC_1_SHIFT    5
#define DSP1_R15_POWER_ENABLE0_SOUT2_PWR_IC_1_MASK     0x40
#define DSP1_R15_POWER_ENABLE0_SOUT2_PWR_IC_1_SHIFT    6
#define DSP1_R15_POWER_ENABLE0_SOUT3_PWR_IC_1_MASK     0x80
#define DSP1_R15_POWER_ENABLE0_SOUT3_PWR_IC_1_SHIFT    7
#define DSP1_R15_POWER_ENABLE0_ASRCBANK0_PWR_IC_1_MASK 0x100
#define DSP1_R15_POWER_ENABLE0_ASRCBANK0_PWR_IC_1_SHIFT 8
#define DSP1_R15_POWER_ENABLE0_ASRCBANK1_PWR_IC_1_MASK 0x200
#define DSP1_R15_POWER_ENABLE0_ASRCBANK1_PWR_IC_1_SHIFT 9
#define DSP1_R15_POWER_ENABLE0_CLK_GEN1_PWR_IC_1_MASK  0x400
#define DSP1_R15_POWER_ENABLE0_CLK_GEN1_PWR_IC_1_SHIFT 10
#define DSP1_R15_POWER_ENABLE0_CLK_GEN2_PWR_IC_1_MASK  0x800
#define DSP1_R15_POWER_ENABLE0_CLK_GEN2_PWR_IC_1_SHIFT 11
#define DSP1_R15_POWER_ENABLE0_CLK_GEN3_PWR_IC_1_MASK  0x1000
#define DSP1_R15_POWER_ENABLE0_CLK_GEN3_PWR_IC_1_SHIFT 12

/* POWER_ENABLE1 (IC 1) */
#define DSP1_R16_POWER_ENABLE1_ADC_PWR_IC_1            0x1    /* 1b	[0] */
#define DSP1_R16_POWER_ENABLE1_RX_PWR_IC_1             0x1    /* 1b	[1] */
#define DSP1_R16_POWER_ENABLE1_TX_PWR_IC_1             0x1    /* 1b	[2] */
#define DSP1_R16_POWER_ENABLE1_PDM0_PWR_IC_1           0x1    /* 1b	[3] */
#define DSP1_R16_POWER_ENABLE1_PDM1_PWR_IC_1           0x1    /* 1b	[4] */
#define DSP1_R16_POWER_ENABLE1_ADC_PWR_IC_1_MASK       0x1
#define DSP1_R16_POWER_ENABLE1_ADC_PWR_IC_1_SHIFT      0
#define DSP1_R16_POWER_ENABLE1_RX_PWR_IC_1_MASK        0x2
#define DSP1_R16_POWER_ENABLE1_RX_PWR_IC_1_SHIFT       1
#define DSP1_R16_POWER_ENABLE1_TX_PWR_IC_1_MASK        0x4
#define DSP1_R16_POWER_ENABLE1_TX_PWR_IC_1_SHIFT       2
#define DSP1_R16_POWER_ENABLE1_PDM0_PWR_IC_1_MASK      0x8
#define DSP1_R16_POWER_ENABLE1_PDM0_PWR_IC_1_SHIFT     3
#define DSP1_R16_POWER_ENABLE1_PDM1_PWR_IC_1_MASK      0x10
#define DSP1_R16_POWER_ENABLE1_PDM1_PWR_IC_1_SHIFT     4

/* ASRC_INPUT0 (IC 1) */
#define DSP1_R17_ASRC_INPUT0_ASRC_SOURCE_IC_1          0x0    /* 000b	[2:0] */
#define DSP1_R17_ASRC_INPUT0_ASRC_SIN_CHANNEL_IC_1     0x0    /* 00000b	[7:3] */
#define DSP1_R17_ASRC_INPUT0_ASRC_SOURCE_IC_1_MASK     0x7
#define DSP1_R17_ASRC_INPUT0_ASRC_SOURCE_IC_1_SHIFT    0
#define DSP1_R17_ASRC_INPUT0_ASRC_SIN_CHANNEL_IC_1_MASK 0xF8
#define DSP1_R17_ASRC_INPUT0_ASRC_SIN_CHANNEL_IC_1_SHIFT 3

/* ASRC_INPUT1 (IC 1) */
#define DSP1_R18_ASRC_INPUT1_ASRC_SOURCE_IC_1          0x0    /* 000b	[2:0] */
#define DSP1_R18_ASRC_INPUT1_ASRC_SIN_CHANNEL_IC_1     0x0    /* 00000b	[7:3] */
#define DSP1_R18_ASRC_INPUT1_ASRC_SOURCE_IC_1_MASK     0x7
#define DSP1_R18_ASRC_INPUT1_ASRC_SOURCE_IC_1_SHIFT    0
#define DSP1_R18_ASRC_INPUT1_ASRC_SIN_CHANNEL_IC_1_MASK 0xF8
#define DSP1_R18_ASRC_INPUT1_ASRC_SIN_CHANNEL_IC_1_SHIFT 3

/* ASRC_INPUT2 (IC 1) */
#define DSP1_R19_ASRC_INPUT2_ASRC_SOURCE_IC_1          0x0    /* 000b	[2:0] */
#define DSP1_R19_ASRC_INPUT2_ASRC_SIN_CHANNEL_IC_1     0x0    /* 00000b	[7:3] */
#define DSP1_R19_ASRC_INPUT2_ASRC_SOURCE_IC_1_MASK     0x7
#define DSP1_R19_ASRC_INPUT2_ASRC_SOURCE_IC_1_SHIFT    0
#define DSP1_R19_ASRC_INPUT2_ASRC_SIN_CHANNEL_IC_1_MASK 0xF8
#define DSP1_R19_ASRC_INPUT2_ASRC_SIN_CHANNEL_IC_1_SHIFT 3

/* ASRC_INPUT3 (IC 1) */
#define DSP1_R20_ASRC_INPUT3_ASRC_SOURCE_IC_1          0x0    /* 000b	[2:0] */
#define DSP1_R20_ASRC_INPUT3_ASRC_SIN_CHANNEL_IC_1     0x0    /* 00000b	[7:3] */
#define DSP1_R20_ASRC_INPUT3_ASRC_SOURCE_IC_1_MASK     0x7
#define DSP1_R20_ASRC_INPUT3_ASRC_SOURCE_IC_1_SHIFT    0
#define DSP1_R20_ASRC_INPUT3_ASRC_SIN_CHANNEL_IC_1_MASK 0xF8
#define DSP1_R20_ASRC_INPUT3_ASRC_SIN_CHANNEL_IC_1_SHIFT 3

/* ASRC_INPUT4 (IC 1) */
#define DSP1_R21_ASRC_INPUT4_ASRC_SOURCE_IC_1          0x0    /* 000b	[2:0] */
#define DSP1_R21_ASRC_INPUT4_ASRC_SIN_CHANNEL_IC_1     0x0    /* 00000b	[7:3] */
#define DSP1_R21_ASRC_INPUT4_ASRC_SOURCE_IC_1_MASK     0x7
#define DSP1_R21_ASRC_INPUT4_ASRC_SOURCE_IC_1_SHIFT    0
#define DSP1_R21_ASRC_INPUT4_ASRC_SIN_CHANNEL_IC_1_MASK 0xF8
#define DSP1_R21_ASRC_INPUT4_ASRC_SIN_CHANNEL_IC_1_SHIFT 3

/* ASRC_INPUT5 (IC 1) */
#define DSP1_R22_ASRC_INPUT5_ASRC_SOURCE_IC_1          0x0    /* 000b	[2:0] */
#define DSP1_R22_ASRC_INPUT5_ASRC_SIN_CHANNEL_IC_1     0x0    /* 00000b	[7:3] */
#define DSP1_R22_ASRC_INPUT5_ASRC_SOURCE_IC_1_MASK     0x7
#define DSP1_R22_ASRC_INPUT5_ASRC_SOURCE_IC_1_SHIFT    0
#define DSP1_R22_ASRC_INPUT5_ASRC_SIN_CHANNEL_IC_1_MASK 0xF8
#define DSP1_R22_ASRC_INPUT5_ASRC_SIN_CHANNEL_IC_1_SHIFT 3

/* ASRC_INPUT6 (IC 1) */
#define DSP1_R23_ASRC_INPUT6_ASRC_SOURCE_IC_1          0x0    /* 000b	[2:0] */
#define DSP1_R23_ASRC_INPUT6_ASRC_SIN_CHANNEL_IC_1     0x0    /* 00000b	[7:3] */
#define DSP1_R23_ASRC_INPUT6_ASRC_SOURCE_IC_1_MASK     0x7
#define DSP1_R23_ASRC_INPUT6_ASRC_SOURCE_IC_1_SHIFT    0
#define DSP1_R23_ASRC_INPUT6_ASRC_SIN_CHANNEL_IC_1_MASK 0xF8
#define DSP1_R23_ASRC_INPUT6_ASRC_SIN_CHANNEL_IC_1_SHIFT 3

/* ASRC_INPUT7 (IC 1) */
#define DSP1_R24_ASRC_INPUT7_ASRC_SOURCE_IC_1          0x0    /* 000b	[2:0] */
#define DSP1_R24_ASRC_INPUT7_ASRC_SIN_CHANNEL_IC_1     0x0    /* 00000b	[7:3] */
#define DSP1_R24_ASRC_INPUT7_ASRC_SOURCE_IC_1_MASK     0x7
#define DSP1_R24_ASRC_INPUT7_ASRC_SOURCE_IC_1_SHIFT    0
#define DSP1_R24_ASRC_INPUT7_ASRC_SIN_CHANNEL_IC_1_MASK 0xF8
#define DSP1_R24_ASRC_INPUT7_ASRC_SIN_CHANNEL_IC_1_SHIFT 3

/* ASRC_OUT_RATE0 (IC 1) */
#define DSP1_R25_ASRC_OUT_RATE0_ASRC_RATE_IC_1         0x0    /* 0000b	[3:0] */
#define DSP1_R25_ASRC_OUT_RATE0_ASRC_RATE_IC_1_MASK    0xF
#define DSP1_R25_ASRC_OUT_RATE0_ASRC_RATE_IC_1_SHIFT   0

/* ASRC_OUT_RATE1 (IC 1) */
#define DSP1_R26_ASRC_OUT_RATE1_ASRC_RATE_IC_1         0x0    /* 0000b	[3:0] */
#define DSP1_R26_ASRC_OUT_RATE1_ASRC_RATE_IC_1_MASK    0xF
#define DSP1_R26_ASRC_OUT_RATE1_ASRC_RATE_IC_1_SHIFT   0

/* ASRC_OUT_RATE2 (IC 1) */
#define DSP1_R27_ASRC_OUT_RATE2_ASRC_RATE_IC_1         0x0    /* 0000b	[3:0] */
#define DSP1_R27_ASRC_OUT_RATE2_ASRC_RATE_IC_1_MASK    0xF
#define DSP1_R27_ASRC_OUT_RATE2_ASRC_RATE_IC_1_SHIFT   0

/* ASRC_OUT_RATE3 (IC 1) */
#define DSP1_R28_ASRC_OUT_RATE3_ASRC_RATE_IC_1         0x0    /* 0000b	[3:0] */
#define DSP1_R28_ASRC_OUT_RATE3_ASRC_RATE_IC_1_MASK    0xF
#define DSP1_R28_ASRC_OUT_RATE3_ASRC_RATE_IC_1_SHIFT   0

/* ASRC_OUT_RATE4 (IC 1) */
#define DSP1_R29_ASRC_OUT_RATE4_ASRC_RATE_IC_1         0x0    /* 0000b	[3:0] */
#define DSP1_R29_ASRC_OUT_RATE4_ASRC_RATE_IC_1_MASK    0xF
#define DSP1_R29_ASRC_OUT_RATE4_ASRC_RATE_IC_1_SHIFT   0

/* ASRC_OUT_RATE5 (IC 1) */
#define DSP1_R30_ASRC_OUT_RATE5_ASRC_RATE_IC_1         0x0    /* 0000b	[3:0] */
#define DSP1_R30_ASRC_OUT_RATE5_ASRC_RATE_IC_1_MASK    0xF
#define DSP1_R30_ASRC_OUT_RATE5_ASRC_RATE_IC_1_SHIFT   0

/* ASRC_OUT_RATE6 (IC 1) */
#define DSP1_R31_ASRC_OUT_RATE6_ASRC_RATE_IC_1         0x0    /* 0000b	[3:0] */
#define DSP1_R31_ASRC_OUT_RATE6_ASRC_RATE_IC_1_MASK    0xF
#define DSP1_R31_ASRC_OUT_RATE6_ASRC_RATE_IC_1_SHIFT   0

/* ASRC_OUT_RATE7 (IC 1) */
#define DSP1_R32_ASRC_OUT_RATE7_ASRC_RATE_IC_1         0x0    /* 0000b	[3:0] */
#define DSP1_R32_ASRC_OUT_RATE7_ASRC_RATE_IC_1_MASK    0xF
#define DSP1_R32_ASRC_OUT_RATE7_ASRC_RATE_IC_1_SHIFT   0

/* SOUT_SOURCE0 (IC 1) */
#define DSP1_R33_SOUT_SOURCE0_SOUT_SOURCE_IC_1         0x2    /* 010b	[2:0] */
#define DSP1_R33_SOUT_SOURCE0_SOUT_ASRC_SELECT_IC_1    0x0    /* 000b	[5:3] */
#define DSP1_R33_SOUT_SOURCE0_SOUT_SOURCE_IC_1_MASK    0x7
#define DSP1_R33_SOUT_SOURCE0_SOUT_SOURCE_IC_1_SHIFT   0
#define DSP1_R33_SOUT_SOURCE0_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R33_SOUT_SOURCE0_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE1 (IC 1) */
#define DSP1_R34_SOUT_SOURCE1_SOUT_SOURCE_IC_1         0x2    /* 010b	[2:0] */
#define DSP1_R34_SOUT_SOURCE1_SOUT_ASRC_SELECT_IC_1    0x0    /* 000b	[5:3] */
#define DSP1_R34_SOUT_SOURCE1_SOUT_SOURCE_IC_1_MASK    0x7
#define DSP1_R34_SOUT_SOURCE1_SOUT_SOURCE_IC_1_SHIFT   0
#define DSP1_R34_SOUT_SOURCE1_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R34_SOUT_SOURCE1_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE2 (IC 1) */
#define DSP1_R35_SOUT_SOURCE2_SOUT_SOURCE_IC_1         0x2    /* 010b	[2:0] */
#define DSP1_R35_SOUT_SOURCE2_SOUT_ASRC_SELECT_IC_1    0x0    /* 000b	[5:3] */
#define DSP1_R35_SOUT_SOURCE2_SOUT_SOURCE_IC_1_MASK    0x7
#define DSP1_R35_SOUT_SOURCE2_SOUT_SOURCE_IC_1_SHIFT   0
#define DSP1_R35_SOUT_SOURCE2_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R35_SOUT_SOURCE2_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE3 (IC 1) */
#define DSP1_R36_SOUT_SOURCE3_SOUT_SOURCE_IC_1         0x2    /* 010b	[2:0] */
#define DSP1_R36_SOUT_SOURCE3_SOUT_ASRC_SELECT_IC_1    0x0    /* 000b	[5:3] */
#define DSP1_R36_SOUT_SOURCE3_SOUT_SOURCE_IC_1_MASK    0x7
#define DSP1_R36_SOUT_SOURCE3_SOUT_SOURCE_IC_1_SHIFT   0
#define DSP1_R36_SOUT_SOURCE3_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R36_SOUT_SOURCE3_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE4 (IC 1) */
#define DSP1_R37_SOUT_SOURCE4_SOUT_SOURCE_IC_1         0x2    /* 010b	[2:0] */
#define DSP1_R37_SOUT_SOURCE4_SOUT_ASRC_SELECT_IC_1    0x0    /* 000b	[5:3] */
#define DSP1_R37_SOUT_SOURCE4_SOUT_SOURCE_IC_1_MASK    0x7
#define DSP1_R37_SOUT_SOURCE4_SOUT_SOURCE_IC_1_SHIFT   0
#define DSP1_R37_SOUT_SOURCE4_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R37_SOUT_SOURCE4_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE5 (IC 1) */
#define DSP1_R38_SOUT_SOURCE5_SOUT_SOURCE_IC_1         0x2    /* 010b	[2:0] */
#define DSP1_R38_SOUT_SOURCE5_SOUT_ASRC_SELECT_IC_1    0x0    /* 000b	[5:3] */
#define DSP1_R38_SOUT_SOURCE5_SOUT_SOURCE_IC_1_MASK    0x7
#define DSP1_R38_SOUT_SOURCE5_SOUT_SOURCE_IC_1_SHIFT   0
#define DSP1_R38_SOUT_SOURCE5_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R38_SOUT_SOURCE5_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE6 (IC 1) */
#define DSP1_R39_SOUT_SOURCE6_SOUT_SOURCE_IC_1         0x2    /* 010b	[2:0] */
#define DSP1_R39_SOUT_SOURCE6_SOUT_ASRC_SELECT_IC_1    0x0    /* 000b	[5:3] */
#define DSP1_R39_SOUT_SOURCE6_SOUT_SOURCE_IC_1_MASK    0x7
#define DSP1_R39_SOUT_SOURCE6_SOUT_SOURCE_IC_1_SHIFT   0
#define DSP1_R39_SOUT_SOURCE6_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R39_SOUT_SOURCE6_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE7 (IC 1) */
#define DSP1_R40_SOUT_SOURCE7_SOUT_SOURCE_IC_1         0x2    /* 010b	[2:0] */
#define DSP1_R40_SOUT_SOURCE7_SOUT_ASRC_SELECT_IC_1    0x0    /* 000b	[5:3] */
#define DSP1_R40_SOUT_SOURCE7_SOUT_SOURCE_IC_1_MASK    0x7
#define DSP1_R40_SOUT_SOURCE7_SOUT_SOURCE_IC_1_SHIFT   0
#define DSP1_R40_SOUT_SOURCE7_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R40_SOUT_SOURCE7_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE8 (IC 1) */
#define DSP1_R41_SOUT_SOURCE8_SOUT_SOURCE_IC_1         0x2    /* 010b	[2:0] */
#define DSP1_R41_SOUT_SOURCE8_SOUT_ASRC_SELECT_IC_1    0x0    /* 000b	[5:3] */
#define DSP1_R41_SOUT_SOURCE8_SOUT_SOURCE_IC_1_MASK    0x7
#define DSP1_R41_SOUT_SOURCE8_SOUT_SOURCE_IC_1_SHIFT   0
#define DSP1_R41_SOUT_SOURCE8_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R41_SOUT_SOURCE8_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE9 (IC 1) */
#define DSP1_R42_SOUT_SOURCE9_SOUT_SOURCE_IC_1         0x2    /* 010b	[2:0] */
#define DSP1_R42_SOUT_SOURCE9_SOUT_ASRC_SELECT_IC_1    0x0    /* 000b	[5:3] */
#define DSP1_R42_SOUT_SOURCE9_SOUT_SOURCE_IC_1_MASK    0x7
#define DSP1_R42_SOUT_SOURCE9_SOUT_SOURCE_IC_1_SHIFT   0
#define DSP1_R42_SOUT_SOURCE9_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R42_SOUT_SOURCE9_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE10 (IC 1) */
#define DSP1_R43_SOUT_SOURCE10_SOUT_SOURCE_IC_1        0x2    /* 010b	[2:0] */
#define DSP1_R43_SOUT_SOURCE10_SOUT_ASRC_SELECT_IC_1   0x0    /* 000b	[5:3] */
#define DSP1_R43_SOUT_SOURCE10_SOUT_SOURCE_IC_1_MASK   0x7
#define DSP1_R43_SOUT_SOURCE10_SOUT_SOURCE_IC_1_SHIFT  0
#define DSP1_R43_SOUT_SOURCE10_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R43_SOUT_SOURCE10_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE11 (IC 1) */
#define DSP1_R44_SOUT_SOURCE11_SOUT_SOURCE_IC_1        0x2    /* 010b	[2:0] */
#define DSP1_R44_SOUT_SOURCE11_SOUT_ASRC_SELECT_IC_1   0x0    /* 000b	[5:3] */
#define DSP1_R44_SOUT_SOURCE11_SOUT_SOURCE_IC_1_MASK   0x7
#define DSP1_R44_SOUT_SOURCE11_SOUT_SOURCE_IC_1_SHIFT  0
#define DSP1_R44_SOUT_SOURCE11_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R44_SOUT_SOURCE11_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE12 (IC 1) */
#define DSP1_R45_SOUT_SOURCE12_SOUT_SOURCE_IC_1        0x2    /* 010b	[2:0] */
#define DSP1_R45_SOUT_SOURCE12_SOUT_ASRC_SELECT_IC_1   0x0    /* 000b	[5:3] */
#define DSP1_R45_SOUT_SOURCE12_SOUT_SOURCE_IC_1_MASK   0x7
#define DSP1_R45_SOUT_SOURCE12_SOUT_SOURCE_IC_1_SHIFT  0
#define DSP1_R45_SOUT_SOURCE12_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R45_SOUT_SOURCE12_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE13 (IC 1) */
#define DSP1_R46_SOUT_SOURCE13_SOUT_SOURCE_IC_1        0x2    /* 010b	[2:0] */
#define DSP1_R46_SOUT_SOURCE13_SOUT_ASRC_SELECT_IC_1   0x0    /* 000b	[5:3] */
#define DSP1_R46_SOUT_SOURCE13_SOUT_SOURCE_IC_1_MASK   0x7
#define DSP1_R46_SOUT_SOURCE13_SOUT_SOURCE_IC_1_SHIFT  0
#define DSP1_R46_SOUT_SOURCE13_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R46_SOUT_SOURCE13_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE14 (IC 1) */
#define DSP1_R47_SOUT_SOURCE14_SOUT_SOURCE_IC_1        0x2    /* 010b	[2:0] */
#define DSP1_R47_SOUT_SOURCE14_SOUT_ASRC_SELECT_IC_1   0x0    /* 000b	[5:3] */
#define DSP1_R47_SOUT_SOURCE14_SOUT_SOURCE_IC_1_MASK   0x7
#define DSP1_R47_SOUT_SOURCE14_SOUT_SOURCE_IC_1_SHIFT  0
#define DSP1_R47_SOUT_SOURCE14_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R47_SOUT_SOURCE14_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE15 (IC 1) */
#define DSP1_R48_SOUT_SOURCE15_SOUT_SOURCE_IC_1        0x2    /* 010b	[2:0] */
#define DSP1_R48_SOUT_SOURCE15_SOUT_ASRC_SELECT_IC_1   0x0    /* 000b	[5:3] */
#define DSP1_R48_SOUT_SOURCE15_SOUT_SOURCE_IC_1_MASK   0x7
#define DSP1_R48_SOUT_SOURCE15_SOUT_SOURCE_IC_1_SHIFT  0
#define DSP1_R48_SOUT_SOURCE15_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R48_SOUT_SOURCE15_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE16 (IC 1) */
#define DSP1_R49_SOUT_SOURCE16_SOUT_SOURCE_IC_1        0x2    /* 010b	[2:0] */
#define DSP1_R49_SOUT_SOURCE16_SOUT_ASRC_SELECT_IC_1   0x0    /* 000b	[5:3] */
#define DSP1_R49_SOUT_SOURCE16_SOUT_SOURCE_IC_1_MASK   0x7
#define DSP1_R49_SOUT_SOURCE16_SOUT_SOURCE_IC_1_SHIFT  0
#define DSP1_R49_SOUT_SOURCE16_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R49_SOUT_SOURCE16_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE17 (IC 1) */
#define DSP1_R50_SOUT_SOURCE17_SOUT_SOURCE_IC_1        0x2    /* 010b	[2:0] */
#define DSP1_R50_SOUT_SOURCE17_SOUT_ASRC_SELECT_IC_1   0x0    /* 000b	[5:3] */
#define DSP1_R50_SOUT_SOURCE17_SOUT_SOURCE_IC_1_MASK   0x7
#define DSP1_R50_SOUT_SOURCE17_SOUT_SOURCE_IC_1_SHIFT  0
#define DSP1_R50_SOUT_SOURCE17_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R50_SOUT_SOURCE17_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE18 (IC 1) */
#define DSP1_R51_SOUT_SOURCE18_SOUT_SOURCE_IC_1        0x2    /* 010b	[2:0] */
#define DSP1_R51_SOUT_SOURCE18_SOUT_ASRC_SELECT_IC_1   0x0    /* 000b	[5:3] */
#define DSP1_R51_SOUT_SOURCE18_SOUT_SOURCE_IC_1_MASK   0x7
#define DSP1_R51_SOUT_SOURCE18_SOUT_SOURCE_IC_1_SHIFT  0
#define DSP1_R51_SOUT_SOURCE18_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R51_SOUT_SOURCE18_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE19 (IC 1) */
#define DSP1_R52_SOUT_SOURCE19_SOUT_SOURCE_IC_1        0x2    /* 010b	[2:0] */
#define DSP1_R52_SOUT_SOURCE19_SOUT_ASRC_SELECT_IC_1   0x0    /* 000b	[5:3] */
#define DSP1_R52_SOUT_SOURCE19_SOUT_SOURCE_IC_1_MASK   0x7
#define DSP1_R52_SOUT_SOURCE19_SOUT_SOURCE_IC_1_SHIFT  0
#define DSP1_R52_SOUT_SOURCE19_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R52_SOUT_SOURCE19_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE20 (IC 1) */
#define DSP1_R53_SOUT_SOURCE20_SOUT_SOURCE_IC_1        0x2    /* 010b	[2:0] */
#define DSP1_R53_SOUT_SOURCE20_SOUT_ASRC_SELECT_IC_1   0x0    /* 000b	[5:3] */
#define DSP1_R53_SOUT_SOURCE20_SOUT_SOURCE_IC_1_MASK   0x7
#define DSP1_R53_SOUT_SOURCE20_SOUT_SOURCE_IC_1_SHIFT  0
#define DSP1_R53_SOUT_SOURCE20_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R53_SOUT_SOURCE20_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE21 (IC 1) */
#define DSP1_R54_SOUT_SOURCE21_SOUT_SOURCE_IC_1        0x2    /* 010b	[2:0] */
#define DSP1_R54_SOUT_SOURCE21_SOUT_ASRC_SELECT_IC_1   0x0    /* 000b	[5:3] */
#define DSP1_R54_SOUT_SOURCE21_SOUT_SOURCE_IC_1_MASK   0x7
#define DSP1_R54_SOUT_SOURCE21_SOUT_SOURCE_IC_1_SHIFT  0
#define DSP1_R54_SOUT_SOURCE21_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R54_SOUT_SOURCE21_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE22 (IC 1) */
#define DSP1_R55_SOUT_SOURCE22_SOUT_SOURCE_IC_1        0x2    /* 010b	[2:0] */
#define DSP1_R55_SOUT_SOURCE22_SOUT_ASRC_SELECT_IC_1   0x0    /* 000b	[5:3] */
#define DSP1_R55_SOUT_SOURCE22_SOUT_SOURCE_IC_1_MASK   0x7
#define DSP1_R55_SOUT_SOURCE22_SOUT_SOURCE_IC_1_SHIFT  0
#define DSP1_R55_SOUT_SOURCE22_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R55_SOUT_SOURCE22_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SOUT_SOURCE23 (IC 1) */
#define DSP1_R56_SOUT_SOURCE23_SOUT_SOURCE_IC_1        0x2    /* 010b	[2:0] */
#define DSP1_R56_SOUT_SOURCE23_SOUT_ASRC_SELECT_IC_1   0x0    /* 000b	[5:3] */
#define DSP1_R56_SOUT_SOURCE23_SOUT_SOURCE_IC_1_MASK   0x7
#define DSP1_R56_SOUT_SOURCE23_SOUT_SOURCE_IC_1_SHIFT  0
#define DSP1_R56_SOUT_SOURCE23_SOUT_ASRC_SELECT_IC_1_MASK 0x38
#define DSP1_R56_SOUT_SOURCE23_SOUT_ASRC_SELECT_IC_1_SHIFT 3

/* SPDIFTX_INPUT (IC 1) */
#define DSP1_R57_SPDIFTX_INPUT_SPDIFTX_SOURCE_IC_1     0x0    /* 00b	[1:0] */
#define DSP1_R57_SPDIFTX_INPUT_SPDIFTX_SOURCE_IC_1_MASK 0x3
#define DSP1_R57_SPDIFTX_INPUT_SPDIFTX_SOURCE_IC_1_SHIFT 0

/* SERIAL_BYTE_0_0 (IC 1) */
#define DSP1_R58_SERIAL_BYTE_0_0_TDM_MODE_IC_1         0x3    /* 011b	[2:0] */
#define DSP1_R58_SERIAL_BYTE_0_0_DATA_FMT_IC_1         0x1    /* 01b	[4:3] */
#define DSP1_R58_SERIAL_BYTE_0_0_WORD_LEN_IC_1         0x1    /* 01b	[6:5] */
#define DSP1_R58_SERIAL_BYTE_0_0_BCLK_POL_IC_1         0x1    /* 1b	[7] */
#define DSP1_R58_SERIAL_BYTE_0_0_LRCLK_POL_IC_1        0x1    /* 1b	[8] */
#define DSP1_R58_SERIAL_BYTE_0_0_LRCLK_MODE_IC_1       0x1    /* 1b	[9] */
#define DSP1_R58_SERIAL_BYTE_0_0_BCLK_SRC_IC_1         0x4    /* 100b	[12:10] */
#define DSP1_R58_SERIAL_BYTE_0_0_LRCLK_SRC_IC_1        0x4    /* 100b	[15:13] */
#define DSP1_R58_SERIAL_BYTE_0_0_TDM_MODE_IC_1_MASK    0x7
#define DSP1_R58_SERIAL_BYTE_0_0_TDM_MODE_IC_1_SHIFT   0
#define DSP1_R58_SERIAL_BYTE_0_0_DATA_FMT_IC_1_MASK    0x18
#define DSP1_R58_SERIAL_BYTE_0_0_DATA_FMT_IC_1_SHIFT   3
#define DSP1_R58_SERIAL_BYTE_0_0_WORD_LEN_IC_1_MASK    0x60
#define DSP1_R58_SERIAL_BYTE_0_0_WORD_LEN_IC_1_SHIFT   5
#define DSP1_R58_SERIAL_BYTE_0_0_BCLK_POL_IC_1_MASK    0x80
#define DSP1_R58_SERIAL_BYTE_0_0_BCLK_POL_IC_1_SHIFT   7
#define DSP1_R58_SERIAL_BYTE_0_0_LRCLK_POL_IC_1_MASK   0x100
#define DSP1_R58_SERIAL_BYTE_0_0_LRCLK_POL_IC_1_SHIFT  8
#define DSP1_R58_SERIAL_BYTE_0_0_LRCLK_MODE_IC_1_MASK  0x200
#define DSP1_R58_SERIAL_BYTE_0_0_LRCLK_MODE_IC_1_SHIFT 9
#define DSP1_R58_SERIAL_BYTE_0_0_BCLK_SRC_IC_1_MASK    0x1C00
#define DSP1_R58_SERIAL_BYTE_0_0_BCLK_SRC_IC_1_SHIFT   10
#define DSP1_R58_SERIAL_BYTE_0_0_LRCLK_SRC_IC_1_MASK   0xE000
#define DSP1_R58_SERIAL_BYTE_0_0_LRCLK_SRC_IC_1_SHIFT  13

/* SERIAL_BYTE_0_1 (IC 1) */
#define DSP1_R59_SERIAL_BYTE_0_1_FS_IC_1               0x2    /* 010b	[2:0] */
#define DSP1_R59_SERIAL_BYTE_0_1_CLK_DOMAIN_IC_1       0x0    /* 00b	[4:3] */
#define DSP1_R59_SERIAL_BYTE_0_1_TRISTATE_IC_1         0x0    /* 0b	[5] */
#define DSP1_R59_SERIAL_BYTE_0_1_FS_IC_1_MASK          0x7
#define DSP1_R59_SERIAL_BYTE_0_1_FS_IC_1_SHIFT         0
#define DSP1_R59_SERIAL_BYTE_0_1_CLK_DOMAIN_IC_1_MASK  0x18
#define DSP1_R59_SERIAL_BYTE_0_1_CLK_DOMAIN_IC_1_SHIFT 3
#define DSP1_R59_SERIAL_BYTE_0_1_TRISTATE_IC_1_MASK    0x20
#define DSP1_R59_SERIAL_BYTE_0_1_TRISTATE_IC_1_SHIFT   5

/* SERIAL_BYTE_1_0 (IC 1) */
#define DSP1_R60_SERIAL_BYTE_1_0_TDM_MODE_IC_1         0x3    /* 011b	[2:0] */
#define DSP1_R60_SERIAL_BYTE_1_0_DATA_FMT_IC_1         0x1    /* 01b	[4:3] */
#define DSP1_R60_SERIAL_BYTE_1_0_WORD_LEN_IC_1         0x1    /* 01b	[6:5] */
#define DSP1_R60_SERIAL_BYTE_1_0_BCLK_POL_IC_1         0x1    /* 1b	[7] */
#define DSP1_R60_SERIAL_BYTE_1_0_LRCLK_POL_IC_1        0x1    /* 1b	[8] */
#define DSP1_R60_SERIAL_BYTE_1_0_LRCLK_MODE_IC_1       0x1    /* 1b	[9] */
#define DSP1_R60_SERIAL_BYTE_1_0_BCLK_SRC_IC_1         0x4    /* 100b	[12:10] */
#define DSP1_R60_SERIAL_BYTE_1_0_LRCLK_SRC_IC_1        0x4    /* 100b	[15:13] */
#define DSP1_R60_SERIAL_BYTE_1_0_TDM_MODE_IC_1_MASK    0x7
#define DSP1_R60_SERIAL_BYTE_1_0_TDM_MODE_IC_1_SHIFT   0
#define DSP1_R60_SERIAL_BYTE_1_0_DATA_FMT_IC_1_MASK    0x18
#define DSP1_R60_SERIAL_BYTE_1_0_DATA_FMT_IC_1_SHIFT   3
#define DSP1_R60_SERIAL_BYTE_1_0_WORD_LEN_IC_1_MASK    0x60
#define DSP1_R60_SERIAL_BYTE_1_0_WORD_LEN_IC_1_SHIFT   5
#define DSP1_R60_SERIAL_BYTE_1_0_BCLK_POL_IC_1_MASK    0x80
#define DSP1_R60_SERIAL_BYTE_1_0_BCLK_POL_IC_1_SHIFT   7
#define DSP1_R60_SERIAL_BYTE_1_0_LRCLK_POL_IC_1_MASK   0x100
#define DSP1_R60_SERIAL_BYTE_1_0_LRCLK_POL_IC_1_SHIFT  8
#define DSP1_R60_SERIAL_BYTE_1_0_LRCLK_MODE_IC_1_MASK  0x200
#define DSP1_R60_SERIAL_BYTE_1_0_LRCLK_MODE_IC_1_SHIFT 9
#define DSP1_R60_SERIAL_BYTE_1_0_BCLK_SRC_IC_1_MASK    0x1C00
#define DSP1_R60_SERIAL_BYTE_1_0_BCLK_SRC_IC_1_SHIFT   10
#define DSP1_R60_SERIAL_BYTE_1_0_LRCLK_SRC_IC_1_MASK   0xE000
#define DSP1_R60_SERIAL_BYTE_1_0_LRCLK_SRC_IC_1_SHIFT  13

/* SERIAL_BYTE_1_1 (IC 1) */
#define DSP1_R61_SERIAL_BYTE_1_1_FS_IC_1               0x2    /* 010b	[2:0] */
#define DSP1_R61_SERIAL_BYTE_1_1_CLK_DOMAIN_IC_1       0x0    /* 00b	[4:3] */
#define DSP1_R61_SERIAL_BYTE_1_1_TRISTATE_IC_1         0x0    /* 0b	[5] */
#define DSP1_R61_SERIAL_BYTE_1_1_FS_IC_1_MASK          0x7
#define DSP1_R61_SERIAL_BYTE_1_1_FS_IC_1_SHIFT         0
#define DSP1_R61_SERIAL_BYTE_1_1_CLK_DOMAIN_IC_1_MASK  0x18
#define DSP1_R61_SERIAL_BYTE_1_1_CLK_DOMAIN_IC_1_SHIFT 3
#define DSP1_R61_SERIAL_BYTE_1_1_TRISTATE_IC_1_MASK    0x20
#define DSP1_R61_SERIAL_BYTE_1_1_TRISTATE_IC_1_SHIFT   5

/* SERIAL_BYTE_2_0 (IC 1) */
#define DSP1_R62_SERIAL_BYTE_2_0_TDM_MODE_IC_1         0x2    /* 010b	[2:0] */
#define DSP1_R62_SERIAL_BYTE_2_0_DATA_FMT_IC_1         0x1    /* 01b	[4:3] */
#define DSP1_R62_SERIAL_BYTE_2_0_WORD_LEN_IC_1         0x1    /* 01b	[6:5] */
#define DSP1_R62_SERIAL_BYTE_2_0_BCLK_POL_IC_1         0x1    /* 1b	[7] */
#define DSP1_R62_SERIAL_BYTE_2_0_LRCLK_POL_IC_1        0x1    /* 1b	[8] */
#define DSP1_R62_SERIAL_BYTE_2_0_LRCLK_MODE_IC_1       0x1    /* 1b	[9] */
#define DSP1_R62_SERIAL_BYTE_2_0_BCLK_SRC_IC_1         0x4    /* 100b	[12:10] */
#define DSP1_R62_SERIAL_BYTE_2_0_LRCLK_SRC_IC_1        0x4    /* 100b	[15:13] */
#define DSP1_R62_SERIAL_BYTE_2_0_TDM_MODE_IC_1_MASK    0x7
#define DSP1_R62_SERIAL_BYTE_2_0_TDM_MODE_IC_1_SHIFT   0
#define DSP1_R62_SERIAL_BYTE_2_0_DATA_FMT_IC_1_MASK    0x18
#define DSP1_R62_SERIAL_BYTE_2_0_DATA_FMT_IC_1_SHIFT   3
#define DSP1_R62_SERIAL_BYTE_2_0_WORD_LEN_IC_1_MASK    0x60
#define DSP1_R62_SERIAL_BYTE_2_0_WORD_LEN_IC_1_SHIFT   5
#define DSP1_R62_SERIAL_BYTE_2_0_BCLK_POL_IC_1_MASK    0x80
#define DSP1_R62_SERIAL_BYTE_2_0_BCLK_POL_IC_1_SHIFT   7
#define DSP1_R62_SERIAL_BYTE_2_0_LRCLK_POL_IC_1_MASK   0x100
#define DSP1_R62_SERIAL_BYTE_2_0_LRCLK_POL_IC_1_SHIFT  8
#define DSP1_R62_SERIAL_BYTE_2_0_LRCLK_MODE_IC_1_MASK  0x200
#define DSP1_R62_SERIAL_BYTE_2_0_LRCLK_MODE_IC_1_SHIFT 9
#define DSP1_R62_SERIAL_BYTE_2_0_BCLK_SRC_IC_1_MASK    0x1C00
#define DSP1_R62_SERIAL_BYTE_2_0_BCLK_SRC_IC_1_SHIFT   10
#define DSP1_R62_SERIAL_BYTE_2_0_LRCLK_SRC_IC_1_MASK   0xE000
#define DSP1_R62_SERIAL_BYTE_2_0_LRCLK_SRC_IC_1_SHIFT  13

/* SERIAL_BYTE_2_1 (IC 1) */
#define DSP1_R63_SERIAL_BYTE_2_1_FS_IC_1               0x2    /* 010b	[2:0] */
#define DSP1_R63_SERIAL_BYTE_2_1_CLK_DOMAIN_IC_1       0x0    /* 00b	[4:3] */
#define DSP1_R63_SERIAL_BYTE_2_1_TRISTATE_IC_1         0x0    /* 0b	[5] */
#define DSP1_R63_SERIAL_BYTE_2_1_FS_IC_1_MASK          0x7
#define DSP1_R63_SERIAL_BYTE_2_1_FS_IC_1_SHIFT         0
#define DSP1_R63_SERIAL_BYTE_2_1_CLK_DOMAIN_IC_1_MASK  0x18
#define DSP1_R63_SERIAL_BYTE_2_1_CLK_DOMAIN_IC_1_SHIFT 3
#define DSP1_R63_SERIAL_BYTE_2_1_TRISTATE_IC_1_MASK    0x20
#define DSP1_R63_SERIAL_BYTE_2_1_TRISTATE_IC_1_SHIFT   5

/* SERIAL_BYTE_3_0 (IC 1) */
#define DSP1_R64_SERIAL_BYTE_3_0_TDM_MODE_IC_1         0x2    /* 010b	[2:0] */
#define DSP1_R64_SERIAL_BYTE_3_0_DATA_FMT_IC_1         0x0    /* 00b	[4:3] */
#define DSP1_R64_SERIAL_BYTE_3_0_WORD_LEN_IC_1         0x1    /* 01b	[6:5] */
#define DSP1_R64_SERIAL_BYTE_3_0_BCLK_POL_IC_1         0x0    /* 0b	[7] */
#define DSP1_R64_SERIAL_BYTE_3_0_LRCLK_POL_IC_1        0x1    /* 1b	[8] */
#define DSP1_R64_SERIAL_BYTE_3_0_LRCLK_MODE_IC_1       0x1    /* 1b	[9] */
#define DSP1_R64_SERIAL_BYTE_3_0_BCLK_SRC_IC_1         0x4    /* 100b	[12:10] */
#define DSP1_R64_SERIAL_BYTE_3_0_LRCLK_SRC_IC_1        0x4    /* 100b	[15:13] */
#define DSP1_R64_SERIAL_BYTE_3_0_TDM_MODE_IC_1_MASK    0x7
#define DSP1_R64_SERIAL_BYTE_3_0_TDM_MODE_IC_1_SHIFT   0
#define DSP1_R64_SERIAL_BYTE_3_0_DATA_FMT_IC_1_MASK    0x18
#define DSP1_R64_SERIAL_BYTE_3_0_DATA_FMT_IC_1_SHIFT   3
#define DSP1_R64_SERIAL_BYTE_3_0_WORD_LEN_IC_1_MASK    0x60
#define DSP1_R64_SERIAL_BYTE_3_0_WORD_LEN_IC_1_SHIFT   5
#define DSP1_R64_SERIAL_BYTE_3_0_BCLK_POL_IC_1_MASK    0x80
#define DSP1_R64_SERIAL_BYTE_3_0_BCLK_POL_IC_1_SHIFT   7
#define DSP1_R64_SERIAL_BYTE_3_0_LRCLK_POL_IC_1_MASK   0x100
#define DSP1_R64_SERIAL_BYTE_3_0_LRCLK_POL_IC_1_SHIFT  8
#define DSP1_R64_SERIAL_BYTE_3_0_LRCLK_MODE_IC_1_MASK  0x200
#define DSP1_R64_SERIAL_BYTE_3_0_LRCLK_MODE_IC_1_SHIFT 9
#define DSP1_R64_SERIAL_BYTE_3_0_BCLK_SRC_IC_1_MASK    0x1C00
#define DSP1_R64_SERIAL_BYTE_3_0_BCLK_SRC_IC_1_SHIFT   10
#define DSP1_R64_SERIAL_BYTE_3_0_LRCLK_SRC_IC_1_MASK   0xE000
#define DSP1_R64_SERIAL_BYTE_3_0_LRCLK_SRC_IC_1_SHIFT  13

/* SERIAL_BYTE_3_1 (IC 1) */
#define DSP1_R65_SERIAL_BYTE_3_1_FS_IC_1               0x2    /* 010b	[2:0] */
#define DSP1_R65_SERIAL_BYTE_3_1_CLK_DOMAIN_IC_1       0x0    /* 00b	[4:3] */
#define DSP1_R65_SERIAL_BYTE_3_1_TRISTATE_IC_1         0x0    /* 0b	[5] */
#define DSP1_R65_SERIAL_BYTE_3_1_FS_IC_1_MASK          0x7
#define DSP1_R65_SERIAL_BYTE_3_1_FS_IC_1_SHIFT         0
#define DSP1_R65_SERIAL_BYTE_3_1_CLK_DOMAIN_IC_1_MASK  0x18
#define DSP1_R65_SERIAL_BYTE_3_1_CLK_DOMAIN_IC_1_SHIFT 3
#define DSP1_R65_SERIAL_BYTE_3_1_TRISTATE_IC_1_MASK    0x20
#define DSP1_R65_SERIAL_BYTE_3_1_TRISTATE_IC_1_SHIFT   5

/* SERIAL_BYTE_4_0 (IC 1) */
#define DSP1_R66_SERIAL_BYTE_4_0_TDM_MODE_IC_1         0x3    /* 011b	[2:0] */
#define DSP1_R66_SERIAL_BYTE_4_0_DATA_FMT_IC_1         0x0    /* 00b	[4:3] */
#define DSP1_R66_SERIAL_BYTE_4_0_WORD_LEN_IC_1         0x1    /* 01b	[6:5] */
#define DSP1_R66_SERIAL_BYTE_4_0_BCLK_POL_IC_1         0x0    /* 0b	[7] */
#define DSP1_R66_SERIAL_BYTE_4_0_LRCLK_POL_IC_1        0x1    /* 1b	[8] */
#define DSP1_R66_SERIAL_BYTE_4_0_LRCLK_MODE_IC_1       0x1    /* 1b	[9] */
#define DSP1_R66_SERIAL_BYTE_4_0_BCLK_SRC_IC_1         0x4    /* 100b	[12:10] */
#define DSP1_R66_SERIAL_BYTE_4_0_LRCLK_SRC_IC_1        0x4    /* 100b	[15:13] */
#define DSP1_R66_SERIAL_BYTE_4_0_TDM_MODE_IC_1_MASK    0x7
#define DSP1_R66_SERIAL_BYTE_4_0_TDM_MODE_IC_1_SHIFT   0
#define DSP1_R66_SERIAL_BYTE_4_0_DATA_FMT_IC_1_MASK    0x18
#define DSP1_R66_SERIAL_BYTE_4_0_DATA_FMT_IC_1_SHIFT   3
#define DSP1_R66_SERIAL_BYTE_4_0_WORD_LEN_IC_1_MASK    0x60
#define DSP1_R66_SERIAL_BYTE_4_0_WORD_LEN_IC_1_SHIFT   5
#define DSP1_R66_SERIAL_BYTE_4_0_BCLK_POL_IC_1_MASK    0x80
#define DSP1_R66_SERIAL_BYTE_4_0_BCLK_POL_IC_1_SHIFT   7
#define DSP1_R66_SERIAL_BYTE_4_0_LRCLK_POL_IC_1_MASK   0x100
#define DSP1_R66_SERIAL_BYTE_4_0_LRCLK_POL_IC_1_SHIFT  8
#define DSP1_R66_SERIAL_BYTE_4_0_LRCLK_MODE_IC_1_MASK  0x200
#define DSP1_R66_SERIAL_BYTE_4_0_LRCLK_MODE_IC_1_SHIFT 9
#define DSP1_R66_SERIAL_BYTE_4_0_BCLK_SRC_IC_1_MASK    0x1C00
#define DSP1_R66_SERIAL_BYTE_4_0_BCLK_SRC_IC_1_SHIFT   10
#define DSP1_R66_SERIAL_BYTE_4_0_LRCLK_SRC_IC_1_MASK   0xE000
#define DSP1_R66_SERIAL_BYTE_4_0_LRCLK_SRC_IC_1_SHIFT  13

/* SERIAL_BYTE_4_1 (IC 1) */
#define DSP1_R67_SERIAL_BYTE_4_1_FS_IC_1               0x2    /* 010b	[2:0] */
#define DSP1_R67_SERIAL_BYTE_4_1_CLK_DOMAIN_IC_1       0x0    /* 00b	[4:3] */
#define DSP1_R67_SERIAL_BYTE_4_1_TRISTATE_IC_1         0x0    /* 0b	[5] */
#define DSP1_R67_SERIAL_BYTE_4_1_FS_IC_1_MASK          0x7
#define DSP1_R67_SERIAL_BYTE_4_1_FS_IC_1_SHIFT         0
#define DSP1_R67_SERIAL_BYTE_4_1_CLK_DOMAIN_IC_1_MASK  0x18
#define DSP1_R67_SERIAL_BYTE_4_1_CLK_DOMAIN_IC_1_SHIFT 3
#define DSP1_R67_SERIAL_BYTE_4_1_TRISTATE_IC_1_MASK    0x20
#define DSP1_R67_SERIAL_BYTE_4_1_TRISTATE_IC_1_SHIFT   5

/* SERIAL_BYTE_5_0 (IC 1) */
#define DSP1_R68_SERIAL_BYTE_5_0_TDM_MODE_IC_1         0x3    /* 011b	[2:0] */
#define DSP1_R68_SERIAL_BYTE_5_0_DATA_FMT_IC_1         0x1    /* 01b	[4:3] */
#define DSP1_R68_SERIAL_BYTE_5_0_WORD_LEN_IC_1         0x1    /* 01b	[6:5] */
#define DSP1_R68_SERIAL_BYTE_5_0_BCLK_POL_IC_1         0x1    /* 1b	[7] */
#define DSP1_R68_SERIAL_BYTE_5_0_LRCLK_POL_IC_1        0x1    /* 1b	[8] */
#define DSP1_R68_SERIAL_BYTE_5_0_LRCLK_MODE_IC_1       0x1    /* 1b	[9] */
#define DSP1_R68_SERIAL_BYTE_5_0_BCLK_SRC_IC_1         0x1    /* 001b	[12:10] */
#define DSP1_R68_SERIAL_BYTE_5_0_LRCLK_SRC_IC_1        0x1    /* 001b	[15:13] */
#define DSP1_R68_SERIAL_BYTE_5_0_TDM_MODE_IC_1_MASK    0x7
#define DSP1_R68_SERIAL_BYTE_5_0_TDM_MODE_IC_1_SHIFT   0
#define DSP1_R68_SERIAL_BYTE_5_0_DATA_FMT_IC_1_MASK    0x18
#define DSP1_R68_SERIAL_BYTE_5_0_DATA_FMT_IC_1_SHIFT   3
#define DSP1_R68_SERIAL_BYTE_5_0_WORD_LEN_IC_1_MASK    0x60
#define DSP1_R68_SERIAL_BYTE_5_0_WORD_LEN_IC_1_SHIFT   5
#define DSP1_R68_SERIAL_BYTE_5_0_BCLK_POL_IC_1_MASK    0x80
#define DSP1_R68_SERIAL_BYTE_5_0_BCLK_POL_IC_1_SHIFT   7
#define DSP1_R68_SERIAL_BYTE_5_0_LRCLK_POL_IC_1_MASK   0x100
#define DSP1_R68_SERIAL_BYTE_5_0_LRCLK_POL_IC_1_SHIFT  8
#define DSP1_R68_SERIAL_BYTE_5_0_LRCLK_MODE_IC_1_MASK  0x200
#define DSP1_R68_SERIAL_BYTE_5_0_LRCLK_MODE_IC_1_SHIFT 9
#define DSP1_R68_SERIAL_BYTE_5_0_BCLK_SRC_IC_1_MASK    0x1C00
#define DSP1_R68_SERIAL_BYTE_5_0_BCLK_SRC_IC_1_SHIFT   10
#define DSP1_R68_SERIAL_BYTE_5_0_LRCLK_SRC_IC_1_MASK   0xE000
#define DSP1_R68_SERIAL_BYTE_5_0_LRCLK_SRC_IC_1_SHIFT  13

/* SERIAL_BYTE_5_1 (IC 1) */
#define DSP1_R69_SERIAL_BYTE_5_1_FS_IC_1               0x2    /* 010b	[2:0] */
#define DSP1_R69_SERIAL_BYTE_5_1_CLK_DOMAIN_IC_1       0x0    /* 00b	[4:3] */
#define DSP1_R69_SERIAL_BYTE_5_1_TRISTATE_IC_1         0x0    /* 0b	[5] */
#define DSP1_R69_SERIAL_BYTE_5_1_FS_IC_1_MASK          0x7
#define DSP1_R69_SERIAL_BYTE_5_1_FS_IC_1_SHIFT         0
#define DSP1_R69_SERIAL_BYTE_5_1_CLK_DOMAIN_IC_1_MASK  0x18
#define DSP1_R69_SERIAL_BYTE_5_1_CLK_DOMAIN_IC_1_SHIFT 3
#define DSP1_R69_SERIAL_BYTE_5_1_TRISTATE_IC_1_MASK    0x20
#define DSP1_R69_SERIAL_BYTE_5_1_TRISTATE_IC_1_SHIFT   5

/* SERIAL_BYTE_6_0 (IC 1) */
#define DSP1_R70_SERIAL_BYTE_6_0_TDM_MODE_IC_1         0x2    /* 010b	[2:0] */
#define DSP1_R70_SERIAL_BYTE_6_0_DATA_FMT_IC_1         0x1    /* 01b	[4:3] */
#define DSP1_R70_SERIAL_BYTE_6_0_WORD_LEN_IC_1         0x1    /* 01b	[6:5] */
#define DSP1_R70_SERIAL_BYTE_6_0_BCLK_POL_IC_1         0x1    /* 1b	[7] */
#define DSP1_R70_SERIAL_BYTE_6_0_LRCLK_POL_IC_1        0x1    /* 1b	[8] */
#define DSP1_R70_SERIAL_BYTE_6_0_LRCLK_MODE_IC_1       0x1    /* 1b	[9] */
#define DSP1_R70_SERIAL_BYTE_6_0_BCLK_SRC_IC_1         0x2    /* 010b	[12:10] */
#define DSP1_R70_SERIAL_BYTE_6_0_LRCLK_SRC_IC_1        0x2    /* 010b	[15:13] */
#define DSP1_R70_SERIAL_BYTE_6_0_TDM_MODE_IC_1_MASK    0x7
#define DSP1_R70_SERIAL_BYTE_6_0_TDM_MODE_IC_1_SHIFT   0
#define DSP1_R70_SERIAL_BYTE_6_0_DATA_FMT_IC_1_MASK    0x18
#define DSP1_R70_SERIAL_BYTE_6_0_DATA_FMT_IC_1_SHIFT   3
#define DSP1_R70_SERIAL_BYTE_6_0_WORD_LEN_IC_1_MASK    0x60
#define DSP1_R70_SERIAL_BYTE_6_0_WORD_LEN_IC_1_SHIFT   5
#define DSP1_R70_SERIAL_BYTE_6_0_BCLK_POL_IC_1_MASK    0x80
#define DSP1_R70_SERIAL_BYTE_6_0_BCLK_POL_IC_1_SHIFT   7
#define DSP1_R70_SERIAL_BYTE_6_0_LRCLK_POL_IC_1_MASK   0x100
#define DSP1_R70_SERIAL_BYTE_6_0_LRCLK_POL_IC_1_SHIFT  8
#define DSP1_R70_SERIAL_BYTE_6_0_LRCLK_MODE_IC_1_MASK  0x200
#define DSP1_R70_SERIAL_BYTE_6_0_LRCLK_MODE_IC_1_SHIFT 9
#define DSP1_R70_SERIAL_BYTE_6_0_BCLK_SRC_IC_1_MASK    0x1C00
#define DSP1_R70_SERIAL_BYTE_6_0_BCLK_SRC_IC_1_SHIFT   10
#define DSP1_R70_SERIAL_BYTE_6_0_LRCLK_SRC_IC_1_MASK   0xE000
#define DSP1_R70_SERIAL_BYTE_6_0_LRCLK_SRC_IC_1_SHIFT  13

/* SERIAL_BYTE_6_1 (IC 1) */
#define DSP1_R71_SERIAL_BYTE_6_1_FS_IC_1               0x2    /* 010b	[2:0] */
#define DSP1_R71_SERIAL_BYTE_6_1_CLK_DOMAIN_IC_1       0x0    /* 00b	[4:3] */
#define DSP1_R71_SERIAL_BYTE_6_1_TRISTATE_IC_1         0x0    /* 0b	[5] */
#define DSP1_R71_SERIAL_BYTE_6_1_FS_IC_1_MASK          0x7
#define DSP1_R71_SERIAL_BYTE_6_1_FS_IC_1_SHIFT         0
#define DSP1_R71_SERIAL_BYTE_6_1_CLK_DOMAIN_IC_1_MASK  0x18
#define DSP1_R71_SERIAL_BYTE_6_1_CLK_DOMAIN_IC_1_SHIFT 3
#define DSP1_R71_SERIAL_BYTE_6_1_TRISTATE_IC_1_MASK    0x20
#define DSP1_R71_SERIAL_BYTE_6_1_TRISTATE_IC_1_SHIFT   5

/* SERIAL_BYTE_7_0 (IC 1) */
#define DSP1_R72_SERIAL_BYTE_7_0_TDM_MODE_IC_1         0x2    /* 010b	[2:0] */
#define DSP1_R72_SERIAL_BYTE_7_0_DATA_FMT_IC_1         0x0    /* 00b	[4:3] */
#define DSP1_R72_SERIAL_BYTE_7_0_WORD_LEN_IC_1         0x1    /* 01b	[6:5] */
#define DSP1_R72_SERIAL_BYTE_7_0_BCLK_POL_IC_1         0x0    /* 0b	[7] */
#define DSP1_R72_SERIAL_BYTE_7_0_LRCLK_POL_IC_1        0x1    /* 1b	[8] */
#define DSP1_R72_SERIAL_BYTE_7_0_LRCLK_MODE_IC_1       0x1    /* 1b	[9] */
#define DSP1_R72_SERIAL_BYTE_7_0_BCLK_SRC_IC_1         0x4    /* 100b	[12:10] */
#define DSP1_R72_SERIAL_BYTE_7_0_LRCLK_SRC_IC_1        0x4    /* 100b	[15:13] */
#define DSP1_R72_SERIAL_BYTE_7_0_TDM_MODE_IC_1_MASK    0x7
#define DSP1_R72_SERIAL_BYTE_7_0_TDM_MODE_IC_1_SHIFT   0
#define DSP1_R72_SERIAL_BYTE_7_0_DATA_FMT_IC_1_MASK    0x18
#define DSP1_R72_SERIAL_BYTE_7_0_DATA_FMT_IC_1_SHIFT   3
#define DSP1_R72_SERIAL_BYTE_7_0_WORD_LEN_IC_1_MASK    0x60
#define DSP1_R72_SERIAL_BYTE_7_0_WORD_LEN_IC_1_SHIFT   5
#define DSP1_R72_SERIAL_BYTE_7_0_BCLK_POL_IC_1_MASK    0x80
#define DSP1_R72_SERIAL_BYTE_7_0_BCLK_POL_IC_1_SHIFT   7
#define DSP1_R72_SERIAL_BYTE_7_0_LRCLK_POL_IC_1_MASK   0x100
#define DSP1_R72_SERIAL_BYTE_7_0_LRCLK_POL_IC_1_SHIFT  8
#define DSP1_R72_SERIAL_BYTE_7_0_LRCLK_MODE_IC_1_MASK  0x200
#define DSP1_R72_SERIAL_BYTE_7_0_LRCLK_MODE_IC_1_SHIFT 9
#define DSP1_R72_SERIAL_BYTE_7_0_BCLK_SRC_IC_1_MASK    0x1C00
#define DSP1_R72_SERIAL_BYTE_7_0_BCLK_SRC_IC_1_SHIFT   10
#define DSP1_R72_SERIAL_BYTE_7_0_LRCLK_SRC_IC_1_MASK   0xE000
#define DSP1_R72_SERIAL_BYTE_7_0_LRCLK_SRC_IC_1_SHIFT  13

/* SERIAL_BYTE_7_1 (IC 1) */
#define DSP1_R73_SERIAL_BYTE_7_1_FS_IC_1               0x2    /* 010b	[2:0] */
#define DSP1_R73_SERIAL_BYTE_7_1_CLK_DOMAIN_IC_1       0x0    /* 00b	[4:3] */
#define DSP1_R73_SERIAL_BYTE_7_1_TRISTATE_IC_1         0x0    /* 0b	[5] */
#define DSP1_R73_SERIAL_BYTE_7_1_FS_IC_1_MASK          0x7
#define DSP1_R73_SERIAL_BYTE_7_1_FS_IC_1_SHIFT         0
#define DSP1_R73_SERIAL_BYTE_7_1_CLK_DOMAIN_IC_1_MASK  0x18
#define DSP1_R73_SERIAL_BYTE_7_1_CLK_DOMAIN_IC_1_SHIFT 3
#define DSP1_R73_SERIAL_BYTE_7_1_TRISTATE_IC_1_MASK    0x20
#define DSP1_R73_SERIAL_BYTE_7_1_TRISTATE_IC_1_SHIFT   5

/* FTDM_IN0 (IC 1) */
#define DSP1_R74_FTDM_IN0_BYTE_IN_POS_IC_1             0x0    /* 00b	[1:0] */
#define DSP1_R74_FTDM_IN0_CHANNEL_IN_POS_IC_1          0x0    /* 000b	[4:2] */
#define DSP1_R74_FTDM_IN0_SERIAL_IN_SEL_IC_1           0x0    /* 0b	[5] */
#define DSP1_R74_FTDM_IN0_REVERSE_IN_BYTE_IC_1         0x0    /* 0b	[6] */
#define DSP1_R74_FTDM_IN0_SLOT_ENABLE_IN_IC_1          0x0    /* 0b	[7] */
#define DSP1_R74_FTDM_IN0_BYTE_IN_POS_IC_1_MASK        0x3
#define DSP1_R74_FTDM_IN0_BYTE_IN_POS_IC_1_SHIFT       0
#define DSP1_R74_FTDM_IN0_CHANNEL_IN_POS_IC_1_MASK     0x1C
#define DSP1_R74_FTDM_IN0_CHANNEL_IN_POS_IC_1_SHIFT    2
#define DSP1_R74_FTDM_IN0_SERIAL_IN_SEL_IC_1_MASK      0x20
#define DSP1_R74_FTDM_IN0_SERIAL_IN_SEL_IC_1_SHIFT     5
#define DSP1_R74_FTDM_IN0_REVERSE_IN_BYTE_IC_1_MASK    0x40
#define DSP1_R74_FTDM_IN0_REVERSE_IN_BYTE_IC_1_SHIFT   6
#define DSP1_R74_FTDM_IN0_SLOT_ENABLE_IN_IC_1_MASK     0x80
#define DSP1_R74_FTDM_IN0_SLOT_ENABLE_IN_IC_1_SHIFT    7

/* FTDM_IN1 (IC 1) */
#define DSP1_R75_FTDM_IN1_BYTE_IN_POS_IC_1             0x0    /* 00b	[1:0] */
#define DSP1_R75_FTDM_IN1_CHANNEL_IN_POS_IC_1          0x0    /* 000b	[4:2] */
#define DSP1_R75_FTDM_IN1_SERIAL_IN_SEL_IC_1           0x0    /* 0b	[5] */
#define DSP1_R75_FTDM_IN1_REVERSE_IN_BYTE_IC_1         0x0    /* 0b	[6] */
#define DSP1_R75_FTDM_IN1_SLOT_ENABLE_IN_IC_1          0x0    /* 0b	[7] */
#define DSP1_R75_FTDM_IN1_BYTE_IN_POS_IC_1_MASK        0x3
#define DSP1_R75_FTDM_IN1_BYTE_IN_POS_IC_1_SHIFT       0
#define DSP1_R75_FTDM_IN1_CHANNEL_IN_POS_IC_1_MASK     0x1C
#define DSP1_R75_FTDM_IN1_CHANNEL_IN_POS_IC_1_SHIFT    2
#define DSP1_R75_FTDM_IN1_SERIAL_IN_SEL_IC_1_MASK      0x20
#define DSP1_R75_FTDM_IN1_SERIAL_IN_SEL_IC_1_SHIFT     5
#define DSP1_R75_FTDM_IN1_REVERSE_IN_BYTE_IC_1_MASK    0x40
#define DSP1_R75_FTDM_IN1_REVERSE_IN_BYTE_IC_1_SHIFT   6
#define DSP1_R75_FTDM_IN1_SLOT_ENABLE_IN_IC_1_MASK     0x80
#define DSP1_R75_FTDM_IN1_SLOT_ENABLE_IN_IC_1_SHIFT    7

/* FTDM_IN2 (IC 1) */
#define DSP1_R76_FTDM_IN2_BYTE_IN_POS_IC_1             0x0    /* 00b	[1:0] */
#define DSP1_R76_FTDM_IN2_CHANNEL_IN_POS_IC_1          0x0    /* 000b	[4:2] */
#define DSP1_R76_FTDM_IN2_SERIAL_IN_SEL_IC_1           0x0    /* 0b	[5] */
#define DSP1_R76_FTDM_IN2_REVERSE_IN_BYTE_IC_1         0x0    /* 0b	[6] */
#define DSP1_R76_FTDM_IN2_SLOT_ENABLE_IN_IC_1          0x0    /* 0b	[7] */
#define DSP1_R76_FTDM_IN2_BYTE_IN_POS_IC_1_MASK        0x3
#define DSP1_R76_FTDM_IN2_BYTE_IN_POS_IC_1_SHIFT       0
#define DSP1_R76_FTDM_IN2_CHANNEL_IN_POS_IC_1_MASK     0x1C
#define DSP1_R76_FTDM_IN2_CHANNEL_IN_POS_IC_1_SHIFT    2
#define DSP1_R76_FTDM_IN2_SERIAL_IN_SEL_IC_1_MASK      0x20
#define DSP1_R76_FTDM_IN2_SERIAL_IN_SEL_IC_1_SHIFT     5
#define DSP1_R76_FTDM_IN2_REVERSE_IN_BYTE_IC_1_MASK    0x40
#define DSP1_R76_FTDM_IN2_REVERSE_IN_BYTE_IC_1_SHIFT   6
#define DSP1_R76_FTDM_IN2_SLOT_ENABLE_IN_IC_1_MASK     0x80
#define DSP1_R76_FTDM_IN2_SLOT_ENABLE_IN_IC_1_SHIFT    7

/* FTDM_IN3 (IC 1) */
#define DSP1_R77_FTDM_IN3_BYTE_IN_POS_IC_1             0x0    /* 00b	[1:0] */
#define DSP1_R77_FTDM_IN3_CHANNEL_IN_POS_IC_1          0x0    /* 000b	[4:2] */
#define DSP1_R77_FTDM_IN3_SERIAL_IN_SEL_IC_1           0x0    /* 0b	[5] */
#define DSP1_R77_FTDM_IN3_REVERSE_IN_BYTE_IC_1         0x0    /* 0b	[6] */
#define DSP1_R77_FTDM_IN3_SLOT_ENABLE_IN_IC_1          0x0    /* 0b	[7] */
#define DSP1_R77_FTDM_IN3_BYTE_IN_POS_IC_1_MASK        0x3
#define DSP1_R77_FTDM_IN3_BYTE_IN_POS_IC_1_SHIFT       0
#define DSP1_R77_FTDM_IN3_CHANNEL_IN_POS_IC_1_MASK     0x1C
#define DSP1_R77_FTDM_IN3_CHANNEL_IN_POS_IC_1_SHIFT    2
#define DSP1_R77_FTDM_IN3_SERIAL_IN_SEL_IC_1_MASK      0x20
#define DSP1_R77_FTDM_IN3_SERIAL_IN_SEL_IC_1_SHIFT     5
#define DSP1_R77_FTDM_IN3_REVERSE_IN_BYTE_IC_1_MASK    0x40
#define DSP1_R77_FTDM_IN3_REVERSE_IN_BYTE_IC_1_SHIFT   6
#define DSP1_R77_FTDM_IN3_SLOT_ENABLE_IN_IC_1_MASK     0x80
#define DSP1_R77_FTDM_IN3_SLOT_ENABLE_IN_IC_1_SHIFT    7

/* FTDM_IN4 (IC 1) */
#define DSP1_R78_FTDM_IN4_BYTE_IN_POS_IC_1             0x0    /* 00b	[1:0] */
#define DSP1_R78_FTDM_IN4_CHANNEL_IN_POS_IC_1          0x0    /* 000b	[4:2] */
#define DSP1_R78_FTDM_IN4_SERIAL_IN_SEL_IC_1           0x0    /* 0b	[5] */
#define DSP1_R78_FTDM_IN4_REVERSE_IN_BYTE_IC_1         0x0    /* 0b	[6] */
#define DSP1_R78_FTDM_IN4_SLOT_ENABLE_IN_IC_1          0x0    /* 0b	[7] */
#define DSP1_R78_FTDM_IN4_BYTE_IN_POS_IC_1_MASK        0x3
#define DSP1_R78_FTDM_IN4_BYTE_IN_POS_IC_1_SHIFT       0
#define DSP1_R78_FTDM_IN4_CHANNEL_IN_POS_IC_1_MASK     0x1C
#define DSP1_R78_FTDM_IN4_CHANNEL_IN_POS_IC_1_SHIFT    2
#define DSP1_R78_FTDM_IN4_SERIAL_IN_SEL_IC_1_MASK      0x20
#define DSP1_R78_FTDM_IN4_SERIAL_IN_SEL_IC_1_SHIFT     5
#define DSP1_R78_FTDM_IN4_REVERSE_IN_BYTE_IC_1_MASK    0x40
#define DSP1_R78_FTDM_IN4_REVERSE_IN_BYTE_IC_1_SHIFT   6
#define DSP1_R78_FTDM_IN4_SLOT_ENABLE_IN_IC_1_MASK     0x80
#define DSP1_R78_FTDM_IN4_SLOT_ENABLE_IN_IC_1_SHIFT    7

/* FTDM_IN5 (IC 1) */
#define DSP1_R79_FTDM_IN5_BYTE_IN_POS_IC_1             0x0    /* 00b	[1:0] */
#define DSP1_R79_FTDM_IN5_CHANNEL_IN_POS_IC_1          0x0    /* 000b	[4:2] */
#define DSP1_R79_FTDM_IN5_SERIAL_IN_SEL_IC_1           0x0    /* 0b	[5] */
#define DSP1_R79_FTDM_IN5_REVERSE_IN_BYTE_IC_1         0x0    /* 0b	[6] */
#define DSP1_R79_FTDM_IN5_SLOT_ENABLE_IN_IC_1          0x0    /* 0b	[7] */
#define DSP1_R79_FTDM_IN5_BYTE_IN_POS_IC_1_MASK        0x3
#define DSP1_R79_FTDM_IN5_BYTE_IN_POS_IC_1_SHIFT       0
#define DSP1_R79_FTDM_IN5_CHANNEL_IN_POS_IC_1_MASK     0x1C
#define DSP1_R79_FTDM_IN5_CHANNEL_IN_POS_IC_1_SHIFT    2
#define DSP1_R79_FTDM_IN5_SERIAL_IN_SEL_IC_1_MASK      0x20
#define DSP1_R79_FTDM_IN5_SERIAL_IN_SEL_IC_1_SHIFT     5
#define DSP1_R79_FTDM_IN5_REVERSE_IN_BYTE_IC_1_MASK    0x40
#define DSP1_R79_FTDM_IN5_REVERSE_IN_BYTE_IC_1_SHIFT   6
#define DSP1_R79_FTDM_IN5_SLOT_ENABLE_IN_IC_1_MASK     0x80
#define DSP1_R79_FTDM_IN5_SLOT_ENABLE_IN_IC_1_SHIFT    7

/* FTDM_IN6 (IC 1) */
#define DSP1_R80_FTDM_IN6_BYTE_IN_POS_IC_1             0x0    /* 00b	[1:0] */
#define DSP1_R80_FTDM_IN6_CHANNEL_IN_POS_IC_1          0x0    /* 000b	[4:2] */
#define DSP1_R80_FTDM_IN6_SERIAL_IN_SEL_IC_1           0x0    /* 0b	[5] */
#define DSP1_R80_FTDM_IN6_REVERSE_IN_BYTE_IC_1         0x0    /* 0b	[6] */
#define DSP1_R80_FTDM_IN6_SLOT_ENABLE_IN_IC_1          0x0    /* 0b	[7] */
#define DSP1_R80_FTDM_IN6_BYTE_IN_POS_IC_1_MASK        0x3
#define DSP1_R80_FTDM_IN6_BYTE_IN_POS_IC_1_SHIFT       0
#define DSP1_R80_FTDM_IN6_CHANNEL_IN_POS_IC_1_MASK     0x1C
#define DSP1_R80_FTDM_IN6_CHANNEL_IN_POS_IC_1_SHIFT    2
#define DSP1_R80_FTDM_IN6_SERIAL_IN_SEL_IC_1_MASK      0x20
#define DSP1_R80_FTDM_IN6_SERIAL_IN_SEL_IC_1_SHIFT     5
#define DSP1_R80_FTDM_IN6_REVERSE_IN_BYTE_IC_1_MASK    0x40
#define DSP1_R80_FTDM_IN6_REVERSE_IN_BYTE_IC_1_SHIFT   6
#define DSP1_R80_FTDM_IN6_SLOT_ENABLE_IN_IC_1_MASK     0x80
#define DSP1_R80_FTDM_IN6_SLOT_ENABLE_IN_IC_1_SHIFT    7

/* FTDM_IN7 (IC 1) */
#define DSP1_R81_FTDM_IN7_BYTE_IN_POS_IC_1             0x0    /* 00b	[1:0] */
#define DSP1_R81_FTDM_IN7_CHANNEL_IN_POS_IC_1          0x0    /* 000b	[4:2] */
#define DSP1_R81_FTDM_IN7_SERIAL_IN_SEL_IC_1           0x0    /* 0b	[5] */
#define DSP1_R81_FTDM_IN7_REVERSE_IN_BYTE_IC_1         0x0    /* 0b	[6] */
#define DSP1_R81_FTDM_IN7_SLOT_ENABLE_IN_IC_1          0x0    /* 0b	[7] */
#define DSP1_R81_FTDM_IN7_BYTE_IN_POS_IC_1_MASK        0x3
#define DSP1_R81_FTDM_IN7_BYTE_IN_POS_IC_1_SHIFT       0
#define DSP1_R81_FTDM_IN7_CHANNEL_IN_POS_IC_1_MASK     0x1C
#define DSP1_R81_FTDM_IN7_CHANNEL_IN_POS_IC_1_SHIFT    2
#define DSP1_R81_FTDM_IN7_SERIAL_IN_SEL_IC_1_MASK      0x20
#define DSP1_R81_FTDM_IN7_SERIAL_IN_SEL_IC_1_SHIFT     5
#define DSP1_R81_FTDM_IN7_REVERSE_IN_BYTE_IC_1_MASK    0x40
#define DSP1_R81_FTDM_IN7_REVERSE_IN_BYTE_IC_1_SHIFT   6
#define DSP1_R81_FTDM_IN7_SLOT_ENABLE_IN_IC_1_MASK     0x80
#define DSP1_R81_FTDM_IN7_SLOT_ENABLE_IN_IC_1_SHIFT    7

/* FTDM_IN8 (IC 1) */
#define DSP1_R82_FTDM_IN8_BYTE_IN_POS_IC_1             0x0    /* 00b	[1:0] */
#define DSP1_R82_FTDM_IN8_CHANNEL_IN_POS_IC_1          0x0    /* 000b	[4:2] */
#define DSP1_R82_FTDM_IN8_SERIAL_IN_SEL_IC_1           0x0    /* 0b	[5] */
#define DSP1_R82_FTDM_IN8_REVERSE_IN_BYTE_IC_1         0x0    /* 0b	[6] */
#define DSP1_R82_FTDM_IN8_SLOT_ENABLE_IN_IC_1          0x0    /* 0b	[7] */
#define DSP1_R82_FTDM_IN8_BYTE_IN_POS_IC_1_MASK        0x3
#define DSP1_R82_FTDM_IN8_BYTE_IN_POS_IC_1_SHIFT       0
#define DSP1_R82_FTDM_IN8_CHANNEL_IN_POS_IC_1_MASK     0x1C
#define DSP1_R82_FTDM_IN8_CHANNEL_IN_POS_IC_1_SHIFT    2
#define DSP1_R82_FTDM_IN8_SERIAL_IN_SEL_IC_1_MASK      0x20
#define DSP1_R82_FTDM_IN8_SERIAL_IN_SEL_IC_1_SHIFT     5
#define DSP1_R82_FTDM_IN8_REVERSE_IN_BYTE_IC_1_MASK    0x40
#define DSP1_R82_FTDM_IN8_REVERSE_IN_BYTE_IC_1_SHIFT   6
#define DSP1_R82_FTDM_IN8_SLOT_ENABLE_IN_IC_1_MASK     0x80
#define DSP1_R82_FTDM_IN8_SLOT_ENABLE_IN_IC_1_SHIFT    7

/* FTDM_IN9 (IC 1) */
#define DSP1_R83_FTDM_IN9_BYTE_IN_POS_IC_1             0x0    /* 00b	[1:0] */
#define DSP1_R83_FTDM_IN9_CHANNEL_IN_POS_IC_1          0x0    /* 000b	[4:2] */
#define DSP1_R83_FTDM_IN9_SERIAL_IN_SEL_IC_1           0x0    /* 0b	[5] */
#define DSP1_R83_FTDM_IN9_REVERSE_IN_BYTE_IC_1         0x0    /* 0b	[6] */
#define DSP1_R83_FTDM_IN9_SLOT_ENABLE_IN_IC_1          0x0    /* 0b	[7] */
#define DSP1_R83_FTDM_IN9_BYTE_IN_POS_IC_1_MASK        0x3
#define DSP1_R83_FTDM_IN9_BYTE_IN_POS_IC_1_SHIFT       0
#define DSP1_R83_FTDM_IN9_CHANNEL_IN_POS_IC_1_MASK     0x1C
#define DSP1_R83_FTDM_IN9_CHANNEL_IN_POS_IC_1_SHIFT    2
#define DSP1_R83_FTDM_IN9_SERIAL_IN_SEL_IC_1_MASK      0x20
#define DSP1_R83_FTDM_IN9_SERIAL_IN_SEL_IC_1_SHIFT     5
#define DSP1_R83_FTDM_IN9_REVERSE_IN_BYTE_IC_1_MASK    0x40
#define DSP1_R83_FTDM_IN9_REVERSE_IN_BYTE_IC_1_SHIFT   6
#define DSP1_R83_FTDM_IN9_SLOT_ENABLE_IN_IC_1_MASK     0x80
#define DSP1_R83_FTDM_IN9_SLOT_ENABLE_IN_IC_1_SHIFT    7

/* FTDM_IN10 (IC 1) */
#define DSP1_R84_FTDM_IN10_BYTE_IN_POS_IC_1            0x0    /* 00b	[1:0] */
#define DSP1_R84_FTDM_IN10_CHANNEL_IN_POS_IC_1         0x0    /* 000b	[4:2] */
#define DSP1_R84_FTDM_IN10_SERIAL_IN_SEL_IC_1          0x0    /* 0b	[5] */
#define DSP1_R84_FTDM_IN10_REVERSE_IN_BYTE_IC_1        0x0    /* 0b	[6] */
#define DSP1_R84_FTDM_IN10_SLOT_ENABLE_IN_IC_1         0x0    /* 0b	[7] */
#define DSP1_R84_FTDM_IN10_BYTE_IN_POS_IC_1_MASK       0x3
#define DSP1_R84_FTDM_IN10_BYTE_IN_POS_IC_1_SHIFT      0
#define DSP1_R84_FTDM_IN10_CHANNEL_IN_POS_IC_1_MASK    0x1C
#define DSP1_R84_FTDM_IN10_CHANNEL_IN_POS_IC_1_SHIFT   2
#define DSP1_R84_FTDM_IN10_SERIAL_IN_SEL_IC_1_MASK     0x20
#define DSP1_R84_FTDM_IN10_SERIAL_IN_SEL_IC_1_SHIFT    5
#define DSP1_R84_FTDM_IN10_REVERSE_IN_BYTE_IC_1_MASK   0x40
#define DSP1_R84_FTDM_IN10_REVERSE_IN_BYTE_IC_1_SHIFT  6
#define DSP1_R84_FTDM_IN10_SLOT_ENABLE_IN_IC_1_MASK    0x80
#define DSP1_R84_FTDM_IN10_SLOT_ENABLE_IN_IC_1_SHIFT   7

/* FTDM_IN11 (IC 1) */
#define DSP1_R85_FTDM_IN11_BYTE_IN_POS_IC_1            0x0    /* 00b	[1:0] */
#define DSP1_R85_FTDM_IN11_CHANNEL_IN_POS_IC_1         0x0    /* 000b	[4:2] */
#define DSP1_R85_FTDM_IN11_SERIAL_IN_SEL_IC_1          0x0    /* 0b	[5] */
#define DSP1_R85_FTDM_IN11_REVERSE_IN_BYTE_IC_1        0x0    /* 0b	[6] */
#define DSP1_R85_FTDM_IN11_SLOT_ENABLE_IN_IC_1         0x0    /* 0b	[7] */
#define DSP1_R85_FTDM_IN11_BYTE_IN_POS_IC_1_MASK       0x3
#define DSP1_R85_FTDM_IN11_BYTE_IN_POS_IC_1_SHIFT      0
#define DSP1_R85_FTDM_IN11_CHANNEL_IN_POS_IC_1_MASK    0x1C
#define DSP1_R85_FTDM_IN11_CHANNEL_IN_POS_IC_1_SHIFT   2
#define DSP1_R85_FTDM_IN11_SERIAL_IN_SEL_IC_1_MASK     0x20
#define DSP1_R85_FTDM_IN11_SERIAL_IN_SEL_IC_1_SHIFT    5
#define DSP1_R85_FTDM_IN11_REVERSE_IN_BYTE_IC_1_MASK   0x40
#define DSP1_R85_FTDM_IN11_REVERSE_IN_BYTE_IC_1_SHIFT  6
#define DSP1_R85_FTDM_IN11_SLOT_ENABLE_IN_IC_1_MASK    0x80
#define DSP1_R85_FTDM_IN11_SLOT_ENABLE_IN_IC_1_SHIFT   7

/* FTDM_IN12 (IC 1) */
#define DSP1_R86_FTDM_IN12_BYTE_IN_POS_IC_1            0x0    /* 00b	[1:0] */
#define DSP1_R86_FTDM_IN12_CHANNEL_IN_POS_IC_1         0x0    /* 000b	[4:2] */
#define DSP1_R86_FTDM_IN12_SERIAL_IN_SEL_IC_1          0x0    /* 0b	[5] */
#define DSP1_R86_FTDM_IN12_REVERSE_IN_BYTE_IC_1        0x0    /* 0b	[6] */
#define DSP1_R86_FTDM_IN12_SLOT_ENABLE_IN_IC_1         0x0    /* 0b	[7] */
#define DSP1_R86_FTDM_IN12_BYTE_IN_POS_IC_1_MASK       0x3
#define DSP1_R86_FTDM_IN12_BYTE_IN_POS_IC_1_SHIFT      0
#define DSP1_R86_FTDM_IN12_CHANNEL_IN_POS_IC_1_MASK    0x1C
#define DSP1_R86_FTDM_IN12_CHANNEL_IN_POS_IC_1_SHIFT   2
#define DSP1_R86_FTDM_IN12_SERIAL_IN_SEL_IC_1_MASK     0x20
#define DSP1_R86_FTDM_IN12_SERIAL_IN_SEL_IC_1_SHIFT    5
#define DSP1_R86_FTDM_IN12_REVERSE_IN_BYTE_IC_1_MASK   0x40
#define DSP1_R86_FTDM_IN12_REVERSE_IN_BYTE_IC_1_SHIFT  6
#define DSP1_R86_FTDM_IN12_SLOT_ENABLE_IN_IC_1_MASK    0x80
#define DSP1_R86_FTDM_IN12_SLOT_ENABLE_IN_IC_1_SHIFT   7

/* FTDM_IN13 (IC 1) */
#define DSP1_R87_FTDM_IN13_BYTE_IN_POS_IC_1            0x0    /* 00b	[1:0] */
#define DSP1_R87_FTDM_IN13_CHANNEL_IN_POS_IC_1         0x0    /* 000b	[4:2] */
#define DSP1_R87_FTDM_IN13_SERIAL_IN_SEL_IC_1          0x0    /* 0b	[5] */
#define DSP1_R87_FTDM_IN13_REVERSE_IN_BYTE_IC_1        0x0    /* 0b	[6] */
#define DSP1_R87_FTDM_IN13_SLOT_ENABLE_IN_IC_1         0x0    /* 0b	[7] */
#define DSP1_R87_FTDM_IN13_BYTE_IN_POS_IC_1_MASK       0x3
#define DSP1_R87_FTDM_IN13_BYTE_IN_POS_IC_1_SHIFT      0
#define DSP1_R87_FTDM_IN13_CHANNEL_IN_POS_IC_1_MASK    0x1C
#define DSP1_R87_FTDM_IN13_CHANNEL_IN_POS_IC_1_SHIFT   2
#define DSP1_R87_FTDM_IN13_SERIAL_IN_SEL_IC_1_MASK     0x20
#define DSP1_R87_FTDM_IN13_SERIAL_IN_SEL_IC_1_SHIFT    5
#define DSP1_R87_FTDM_IN13_REVERSE_IN_BYTE_IC_1_MASK   0x40
#define DSP1_R87_FTDM_IN13_REVERSE_IN_BYTE_IC_1_SHIFT  6
#define DSP1_R87_FTDM_IN13_SLOT_ENABLE_IN_IC_1_MASK    0x80
#define DSP1_R87_FTDM_IN13_SLOT_ENABLE_IN_IC_1_SHIFT   7

/* FTDM_IN14 (IC 1) */
#define DSP1_R88_FTDM_IN14_BYTE_IN_POS_IC_1            0x0    /* 00b	[1:0] */
#define DSP1_R88_FTDM_IN14_CHANNEL_IN_POS_IC_1         0x0    /* 000b	[4:2] */
#define DSP1_R88_FTDM_IN14_SERIAL_IN_SEL_IC_1          0x0    /* 0b	[5] */
#define DSP1_R88_FTDM_IN14_REVERSE_IN_BYTE_IC_1        0x0    /* 0b	[6] */
#define DSP1_R88_FTDM_IN14_SLOT_ENABLE_IN_IC_1         0x0    /* 0b	[7] */
#define DSP1_R88_FTDM_IN14_BYTE_IN_POS_IC_1_MASK       0x3
#define DSP1_R88_FTDM_IN14_BYTE_IN_POS_IC_1_SHIFT      0
#define DSP1_R88_FTDM_IN14_CHANNEL_IN_POS_IC_1_MASK    0x1C
#define DSP1_R88_FTDM_IN14_CHANNEL_IN_POS_IC_1_SHIFT   2
#define DSP1_R88_FTDM_IN14_SERIAL_IN_SEL_IC_1_MASK     0x20
#define DSP1_R88_FTDM_IN14_SERIAL_IN_SEL_IC_1_SHIFT    5
#define DSP1_R88_FTDM_IN14_REVERSE_IN_BYTE_IC_1_MASK   0x40
#define DSP1_R88_FTDM_IN14_REVERSE_IN_BYTE_IC_1_SHIFT  6
#define DSP1_R88_FTDM_IN14_SLOT_ENABLE_IN_IC_1_MASK    0x80
#define DSP1_R88_FTDM_IN14_SLOT_ENABLE_IN_IC_1_SHIFT   7

/* FTDM_IN15 (IC 1) */
#define DSP1_R89_FTDM_IN15_BYTE_IN_POS_IC_1            0x0    /* 00b	[1:0] */
#define DSP1_R89_FTDM_IN15_CHANNEL_IN_POS_IC_1         0x0    /* 000b	[4:2] */
#define DSP1_R89_FTDM_IN15_SERIAL_IN_SEL_IC_1          0x0    /* 0b	[5] */
#define DSP1_R89_FTDM_IN15_REVERSE_IN_BYTE_IC_1        0x0    /* 0b	[6] */
#define DSP1_R89_FTDM_IN15_SLOT_ENABLE_IN_IC_1         0x0    /* 0b	[7] */
#define DSP1_R89_FTDM_IN15_BYTE_IN_POS_IC_1_MASK       0x3
#define DSP1_R89_FTDM_IN15_BYTE_IN_POS_IC_1_SHIFT      0
#define DSP1_R89_FTDM_IN15_CHANNEL_IN_POS_IC_1_MASK    0x1C
#define DSP1_R89_FTDM_IN15_CHANNEL_IN_POS_IC_1_SHIFT   2
#define DSP1_R89_FTDM_IN15_SERIAL_IN_SEL_IC_1_MASK     0x20
#define DSP1_R89_FTDM_IN15_SERIAL_IN_SEL_IC_1_SHIFT    5
#define DSP1_R89_FTDM_IN15_REVERSE_IN_BYTE_IC_1_MASK   0x40
#define DSP1_R89_FTDM_IN15_REVERSE_IN_BYTE_IC_1_SHIFT  6
#define DSP1_R89_FTDM_IN15_SLOT_ENABLE_IN_IC_1_MASK    0x80
#define DSP1_R89_FTDM_IN15_SLOT_ENABLE_IN_IC_1_SHIFT   7

/* FTDM_IN16 (IC 1) */
#define DSP1_R90_FTDM_IN16_BYTE_IN_POS_IC_1            0x0    /* 00b	[1:0] */
#define DSP1_R90_FTDM_IN16_CHANNEL_IN_POS_IC_1         0x0    /* 000b	[4:2] */
#define DSP1_R90_FTDM_IN16_SERIAL_IN_SEL_IC_1          0x0    /* 0b	[5] */
#define DSP1_R90_FTDM_IN16_REVERSE_IN_BYTE_IC_1        0x0    /* 0b	[6] */
#define DSP1_R90_FTDM_IN16_SLOT_ENABLE_IN_IC_1         0x0    /* 0b	[7] */
#define DSP1_R90_FTDM_IN16_BYTE_IN_POS_IC_1_MASK       0x3
#define DSP1_R90_FTDM_IN16_BYTE_IN_POS_IC_1_SHIFT      0
#define DSP1_R90_FTDM_IN16_CHANNEL_IN_POS_IC_1_MASK    0x1C
#define DSP1_R90_FTDM_IN16_CHANNEL_IN_POS_IC_1_SHIFT   2
#define DSP1_R90_FTDM_IN16_SERIAL_IN_SEL_IC_1_MASK     0x20
#define DSP1_R90_FTDM_IN16_SERIAL_IN_SEL_IC_1_SHIFT    5
#define DSP1_R90_FTDM_IN16_REVERSE_IN_BYTE_IC_1_MASK   0x40
#define DSP1_R90_FTDM_IN16_REVERSE_IN_BYTE_IC_1_SHIFT  6
#define DSP1_R90_FTDM_IN16_SLOT_ENABLE_IN_IC_1_MASK    0x80
#define DSP1_R90_FTDM_IN16_SLOT_ENABLE_IN_IC_1_SHIFT   7

/* FTDM_IN17 (IC 1) */
#define DSP1_R91_FTDM_IN17_BYTE_IN_POS_IC_1            0x0    /* 00b	[1:0] */
#define DSP1_R91_FTDM_IN17_CHANNEL_IN_POS_IC_1         0x0    /* 000b	[4:2] */
#define DSP1_R91_FTDM_IN17_SERIAL_IN_SEL_IC_1          0x0    /* 0b	[5] */
#define DSP1_R91_FTDM_IN17_REVERSE_IN_BYTE_IC_1        0x0    /* 0b	[6] */
#define DSP1_R91_FTDM_IN17_SLOT_ENABLE_IN_IC_1         0x0    /* 0b	[7] */
#define DSP1_R91_FTDM_IN17_BYTE_IN_POS_IC_1_MASK       0x3
#define DSP1_R91_FTDM_IN17_BYTE_IN_POS_IC_1_SHIFT      0
#define DSP1_R91_FTDM_IN17_CHANNEL_IN_POS_IC_1_MASK    0x1C
#define DSP1_R91_FTDM_IN17_CHANNEL_IN_POS_IC_1_SHIFT   2
#define DSP1_R91_FTDM_IN17_SERIAL_IN_SEL_IC_1_MASK     0x20
#define DSP1_R91_FTDM_IN17_SERIAL_IN_SEL_IC_1_SHIFT    5
#define DSP1_R91_FTDM_IN17_REVERSE_IN_BYTE_IC_1_MASK   0x40
#define DSP1_R91_FTDM_IN17_REVERSE_IN_BYTE_IC_1_SHIFT  6
#define DSP1_R91_FTDM_IN17_SLOT_ENABLE_IN_IC_1_MASK    0x80
#define DSP1_R91_FTDM_IN17_SLOT_ENABLE_IN_IC_1_SHIFT   7

/* FTDM_IN18 (IC 1) */
#define DSP1_R92_FTDM_IN18_BYTE_IN_POS_IC_1            0x0    /* 00b	[1:0] */
#define DSP1_R92_FTDM_IN18_CHANNEL_IN_POS_IC_1         0x0    /* 000b	[4:2] */
#define DSP1_R92_FTDM_IN18_SERIAL_IN_SEL_IC_1          0x0    /* 0b	[5] */
#define DSP1_R92_FTDM_IN18_REVERSE_IN_BYTE_IC_1        0x0    /* 0b	[6] */
#define DSP1_R92_FTDM_IN18_SLOT_ENABLE_IN_IC_1         0x0    /* 0b	[7] */
#define DSP1_R92_FTDM_IN18_BYTE_IN_POS_IC_1_MASK       0x3
#define DSP1_R92_FTDM_IN18_BYTE_IN_POS_IC_1_SHIFT      0
#define DSP1_R92_FTDM_IN18_CHANNEL_IN_POS_IC_1_MASK    0x1C
#define DSP1_R92_FTDM_IN18_CHANNEL_IN_POS_IC_1_SHIFT   2
#define DSP1_R92_FTDM_IN18_SERIAL_IN_SEL_IC_1_MASK     0x20
#define DSP1_R92_FTDM_IN18_SERIAL_IN_SEL_IC_1_SHIFT    5
#define DSP1_R92_FTDM_IN18_REVERSE_IN_BYTE_IC_1_MASK   0x40
#define DSP1_R92_FTDM_IN18_REVERSE_IN_BYTE_IC_1_SHIFT  6
#define DSP1_R92_FTDM_IN18_SLOT_ENABLE_IN_IC_1_MASK    0x80
#define DSP1_R92_FTDM_IN18_SLOT_ENABLE_IN_IC_1_SHIFT   7

/* FTDM_IN19 (IC 1) */
#define DSP1_R93_FTDM_IN19_BYTE_IN_POS_IC_1            0x0    /* 00b	[1:0] */
#define DSP1_R93_FTDM_IN19_CHANNEL_IN_POS_IC_1         0x0    /* 000b	[4:2] */
#define DSP1_R93_FTDM_IN19_SERIAL_IN_SEL_IC_1          0x0    /* 0b	[5] */
#define DSP1_R93_FTDM_IN19_REVERSE_IN_BYTE_IC_1        0x0    /* 0b	[6] */
#define DSP1_R93_FTDM_IN19_SLOT_ENABLE_IN_IC_1         0x0    /* 0b	[7] */
#define DSP1_R93_FTDM_IN19_BYTE_IN_POS_IC_1_MASK       0x3
#define DSP1_R93_FTDM_IN19_BYTE_IN_POS_IC_1_SHIFT      0
#define DSP1_R93_FTDM_IN19_CHANNEL_IN_POS_IC_1_MASK    0x1C
#define DSP1_R93_FTDM_IN19_CHANNEL_IN_POS_IC_1_SHIFT   2
#define DSP1_R93_FTDM_IN19_SERIAL_IN_SEL_IC_1_MASK     0x20
#define DSP1_R93_FTDM_IN19_SERIAL_IN_SEL_IC_1_SHIFT    5
#define DSP1_R93_FTDM_IN19_REVERSE_IN_BYTE_IC_1_MASK   0x40
#define DSP1_R93_FTDM_IN19_REVERSE_IN_BYTE_IC_1_SHIFT  6
#define DSP1_R93_FTDM_IN19_SLOT_ENABLE_IN_IC_1_MASK    0x80
#define DSP1_R93_FTDM_IN19_SLOT_ENABLE_IN_IC_1_SHIFT   7

/* FTDM_IN20 (IC 1) */
#define DSP1_R94_FTDM_IN20_BYTE_IN_POS_IC_1            0x0    /* 00b	[1:0] */
#define DSP1_R94_FTDM_IN20_CHANNEL_IN_POS_IC_1         0x0    /* 000b	[4:2] */
#define DSP1_R94_FTDM_IN20_SERIAL_IN_SEL_IC_1          0x0    /* 0b	[5] */
#define DSP1_R94_FTDM_IN20_REVERSE_IN_BYTE_IC_1        0x0    /* 0b	[6] */
#define DSP1_R94_FTDM_IN20_SLOT_ENABLE_IN_IC_1         0x0    /* 0b	[7] */
#define DSP1_R94_FTDM_IN20_BYTE_IN_POS_IC_1_MASK       0x3
#define DSP1_R94_FTDM_IN20_BYTE_IN_POS_IC_1_SHIFT      0
#define DSP1_R94_FTDM_IN20_CHANNEL_IN_POS_IC_1_MASK    0x1C
#define DSP1_R94_FTDM_IN20_CHANNEL_IN_POS_IC_1_SHIFT   2
#define DSP1_R94_FTDM_IN20_SERIAL_IN_SEL_IC_1_MASK     0x20
#define DSP1_R94_FTDM_IN20_SERIAL_IN_SEL_IC_1_SHIFT    5
#define DSP1_R94_FTDM_IN20_REVERSE_IN_BYTE_IC_1_MASK   0x40
#define DSP1_R94_FTDM_IN20_REVERSE_IN_BYTE_IC_1_SHIFT  6
#define DSP1_R94_FTDM_IN20_SLOT_ENABLE_IN_IC_1_MASK    0x80
#define DSP1_R94_FTDM_IN20_SLOT_ENABLE_IN_IC_1_SHIFT   7

/* FTDM_IN21 (IC 1) */
#define DSP1_R95_FTDM_IN21_BYTE_IN_POS_IC_1            0x0    /* 00b	[1:0] */
#define DSP1_R95_FTDM_IN21_CHANNEL_IN_POS_IC_1         0x0    /* 000b	[4:2] */
#define DSP1_R95_FTDM_IN21_SERIAL_IN_SEL_IC_1          0x0    /* 0b	[5] */
#define DSP1_R95_FTDM_IN21_REVERSE_IN_BYTE_IC_1        0x0    /* 0b	[6] */
#define DSP1_R95_FTDM_IN21_SLOT_ENABLE_IN_IC_1         0x0    /* 0b	[7] */
#define DSP1_R95_FTDM_IN21_BYTE_IN_POS_IC_1_MASK       0x3
#define DSP1_R95_FTDM_IN21_BYTE_IN_POS_IC_1_SHIFT      0
#define DSP1_R95_FTDM_IN21_CHANNEL_IN_POS_IC_1_MASK    0x1C
#define DSP1_R95_FTDM_IN21_CHANNEL_IN_POS_IC_1_SHIFT   2
#define DSP1_R95_FTDM_IN21_SERIAL_IN_SEL_IC_1_MASK     0x20
#define DSP1_R95_FTDM_IN21_SERIAL_IN_SEL_IC_1_SHIFT    5
#define DSP1_R95_FTDM_IN21_REVERSE_IN_BYTE_IC_1_MASK   0x40
#define DSP1_R95_FTDM_IN21_REVERSE_IN_BYTE_IC_1_SHIFT  6
#define DSP1_R95_FTDM_IN21_SLOT_ENABLE_IN_IC_1_MASK    0x80
#define DSP1_R95_FTDM_IN21_SLOT_ENABLE_IN_IC_1_SHIFT   7

/* FTDM_IN22 (IC 1) */
#define DSP1_R96_FTDM_IN22_BYTE_IN_POS_IC_1            0x0    /* 00b	[1:0] */
#define DSP1_R96_FTDM_IN22_CHANNEL_IN_POS_IC_1         0x0    /* 000b	[4:2] */
#define DSP1_R96_FTDM_IN22_SERIAL_IN_SEL_IC_1          0x0    /* 0b	[5] */
#define DSP1_R96_FTDM_IN22_REVERSE_IN_BYTE_IC_1        0x0    /* 0b	[6] */
#define DSP1_R96_FTDM_IN22_SLOT_ENABLE_IN_IC_1         0x0    /* 0b	[7] */
#define DSP1_R96_FTDM_IN22_BYTE_IN_POS_IC_1_MASK       0x3
#define DSP1_R96_FTDM_IN22_BYTE_IN_POS_IC_1_SHIFT      0
#define DSP1_R96_FTDM_IN22_CHANNEL_IN_POS_IC_1_MASK    0x1C
#define DSP1_R96_FTDM_IN22_CHANNEL_IN_POS_IC_1_SHIFT   2
#define DSP1_R96_FTDM_IN22_SERIAL_IN_SEL_IC_1_MASK     0x20
#define DSP1_R96_FTDM_IN22_SERIAL_IN_SEL_IC_1_SHIFT    5
#define DSP1_R96_FTDM_IN22_REVERSE_IN_BYTE_IC_1_MASK   0x40
#define DSP1_R96_FTDM_IN22_REVERSE_IN_BYTE_IC_1_SHIFT  6
#define DSP1_R96_FTDM_IN22_SLOT_ENABLE_IN_IC_1_MASK    0x80
#define DSP1_R96_FTDM_IN22_SLOT_ENABLE_IN_IC_1_SHIFT   7

/* FTDM_IN23 (IC 1) */
#define DSP1_R97_FTDM_IN23_BYTE_IN_POS_IC_1            0x0    /* 00b	[1:0] */
#define DSP1_R97_FTDM_IN23_CHANNEL_IN_POS_IC_1         0x0    /* 000b	[4:2] */
#define DSP1_R97_FTDM_IN23_SERIAL_IN_SEL_IC_1          0x0    /* 0b	[5] */
#define DSP1_R97_FTDM_IN23_REVERSE_IN_BYTE_IC_1        0x0    /* 0b	[6] */
#define DSP1_R97_FTDM_IN23_SLOT_ENABLE_IN_IC_1         0x0    /* 0b	[7] */
#define DSP1_R97_FTDM_IN23_BYTE_IN_POS_IC_1_MASK       0x3
#define DSP1_R97_FTDM_IN23_BYTE_IN_POS_IC_1_SHIFT      0
#define DSP1_R97_FTDM_IN23_CHANNEL_IN_POS_IC_1_MASK    0x1C
#define DSP1_R97_FTDM_IN23_CHANNEL_IN_POS_IC_1_SHIFT   2
#define DSP1_R97_FTDM_IN23_SERIAL_IN_SEL_IC_1_MASK     0x20
#define DSP1_R97_FTDM_IN23_SERIAL_IN_SEL_IC_1_SHIFT    5
#define DSP1_R97_FTDM_IN23_REVERSE_IN_BYTE_IC_1_MASK   0x40
#define DSP1_R97_FTDM_IN23_REVERSE_IN_BYTE_IC_1_SHIFT  6
#define DSP1_R97_FTDM_IN23_SLOT_ENABLE_IN_IC_1_MASK    0x80
#define DSP1_R97_FTDM_IN23_SLOT_ENABLE_IN_IC_1_SHIFT   7

/* FTDM_IN24 (IC 1) */
#define DSP1_R98_FTDM_IN24_BYTE_IN_POS_IC_1            0x0    /* 00b	[1:0] */
#define DSP1_R98_FTDM_IN24_CHANNEL_IN_POS_IC_1         0x0    /* 000b	[4:2] */
#define DSP1_R98_FTDM_IN24_SERIAL_IN_SEL_IC_1          0x0    /* 0b	[5] */
#define DSP1_R98_FTDM_IN24_REVERSE_IN_BYTE_IC_1        0x0    /* 0b	[6] */
#define DSP1_R98_FTDM_IN24_SLOT_ENABLE_IN_IC_1         0x0    /* 0b	[7] */
#define DSP1_R98_FTDM_IN24_BYTE_IN_POS_IC_1_MASK       0x3
#define DSP1_R98_FTDM_IN24_BYTE_IN_POS_IC_1_SHIFT      0
#define DSP1_R98_FTDM_IN24_CHANNEL_IN_POS_IC_1_MASK    0x1C
#define DSP1_R98_FTDM_IN24_CHANNEL_IN_POS_IC_1_SHIFT   2
#define DSP1_R98_FTDM_IN24_SERIAL_IN_SEL_IC_1_MASK     0x20
#define DSP1_R98_FTDM_IN24_SERIAL_IN_SEL_IC_1_SHIFT    5
#define DSP1_R98_FTDM_IN24_REVERSE_IN_BYTE_IC_1_MASK   0x40
#define DSP1_R98_FTDM_IN24_REVERSE_IN_BYTE_IC_1_SHIFT  6
#define DSP1_R98_FTDM_IN24_SLOT_ENABLE_IN_IC_1_MASK    0x80
#define DSP1_R98_FTDM_IN24_SLOT_ENABLE_IN_IC_1_SHIFT   7

/* FTDM_IN25 (IC 1) */
#define DSP1_R99_FTDM_IN25_BYTE_IN_POS_IC_1            0x0    /* 00b	[1:0] */
#define DSP1_R99_FTDM_IN25_CHANNEL_IN_POS_IC_1         0x0    /* 000b	[4:2] */
#define DSP1_R99_FTDM_IN25_SERIAL_IN_SEL_IC_1          0x0    /* 0b	[5] */
#define DSP1_R99_FTDM_IN25_REVERSE_IN_BYTE_IC_1        0x0    /* 0b	[6] */
#define DSP1_R99_FTDM_IN25_SLOT_ENABLE_IN_IC_1         0x0    /* 0b	[7] */
#define DSP1_R99_FTDM_IN25_BYTE_IN_POS_IC_1_MASK       0x3
#define DSP1_R99_FTDM_IN25_BYTE_IN_POS_IC_1_SHIFT      0
#define DSP1_R99_FTDM_IN25_CHANNEL_IN_POS_IC_1_MASK    0x1C
#define DSP1_R99_FTDM_IN25_CHANNEL_IN_POS_IC_1_SHIFT   2
#define DSP1_R99_FTDM_IN25_SERIAL_IN_SEL_IC_1_MASK     0x20
#define DSP1_R99_FTDM_IN25_SERIAL_IN_SEL_IC_1_SHIFT    5
#define DSP1_R99_FTDM_IN25_REVERSE_IN_BYTE_IC_1_MASK   0x40
#define DSP1_R99_FTDM_IN25_REVERSE_IN_BYTE_IC_1_SHIFT  6
#define DSP1_R99_FTDM_IN25_SLOT_ENABLE_IN_IC_1_MASK    0x80
#define DSP1_R99_FTDM_IN25_SLOT_ENABLE_IN_IC_1_SHIFT   7

/* FTDM_IN26 (IC 1) */
#define DSP1_R100_FTDM_IN26_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R100_FTDM_IN26_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R100_FTDM_IN26_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R100_FTDM_IN26_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R100_FTDM_IN26_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R100_FTDM_IN26_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R100_FTDM_IN26_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R100_FTDM_IN26_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R100_FTDM_IN26_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R100_FTDM_IN26_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R100_FTDM_IN26_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R100_FTDM_IN26_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R100_FTDM_IN26_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R100_FTDM_IN26_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R100_FTDM_IN26_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN27 (IC 1) */
#define DSP1_R101_FTDM_IN27_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R101_FTDM_IN27_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R101_FTDM_IN27_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R101_FTDM_IN27_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R101_FTDM_IN27_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R101_FTDM_IN27_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R101_FTDM_IN27_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R101_FTDM_IN27_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R101_FTDM_IN27_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R101_FTDM_IN27_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R101_FTDM_IN27_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R101_FTDM_IN27_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R101_FTDM_IN27_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R101_FTDM_IN27_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R101_FTDM_IN27_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN28 (IC 1) */
#define DSP1_R102_FTDM_IN28_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R102_FTDM_IN28_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R102_FTDM_IN28_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R102_FTDM_IN28_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R102_FTDM_IN28_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R102_FTDM_IN28_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R102_FTDM_IN28_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R102_FTDM_IN28_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R102_FTDM_IN28_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R102_FTDM_IN28_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R102_FTDM_IN28_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R102_FTDM_IN28_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R102_FTDM_IN28_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R102_FTDM_IN28_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R102_FTDM_IN28_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN29 (IC 1) */
#define DSP1_R103_FTDM_IN29_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R103_FTDM_IN29_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R103_FTDM_IN29_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R103_FTDM_IN29_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R103_FTDM_IN29_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R103_FTDM_IN29_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R103_FTDM_IN29_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R103_FTDM_IN29_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R103_FTDM_IN29_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R103_FTDM_IN29_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R103_FTDM_IN29_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R103_FTDM_IN29_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R103_FTDM_IN29_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R103_FTDM_IN29_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R103_FTDM_IN29_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN30 (IC 1) */
#define DSP1_R104_FTDM_IN30_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R104_FTDM_IN30_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R104_FTDM_IN30_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R104_FTDM_IN30_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R104_FTDM_IN30_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R104_FTDM_IN30_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R104_FTDM_IN30_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R104_FTDM_IN30_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R104_FTDM_IN30_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R104_FTDM_IN30_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R104_FTDM_IN30_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R104_FTDM_IN30_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R104_FTDM_IN30_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R104_FTDM_IN30_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R104_FTDM_IN30_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN31 (IC 1) */
#define DSP1_R105_FTDM_IN31_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R105_FTDM_IN31_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R105_FTDM_IN31_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R105_FTDM_IN31_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R105_FTDM_IN31_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R105_FTDM_IN31_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R105_FTDM_IN31_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R105_FTDM_IN31_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R105_FTDM_IN31_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R105_FTDM_IN31_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R105_FTDM_IN31_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R105_FTDM_IN31_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R105_FTDM_IN31_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R105_FTDM_IN31_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R105_FTDM_IN31_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN32 (IC 1) */
#define DSP1_R106_FTDM_IN32_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R106_FTDM_IN32_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R106_FTDM_IN32_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R106_FTDM_IN32_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R106_FTDM_IN32_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R106_FTDM_IN32_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R106_FTDM_IN32_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R106_FTDM_IN32_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R106_FTDM_IN32_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R106_FTDM_IN32_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R106_FTDM_IN32_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R106_FTDM_IN32_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R106_FTDM_IN32_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R106_FTDM_IN32_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R106_FTDM_IN32_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN33 (IC 1) */
#define DSP1_R107_FTDM_IN33_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R107_FTDM_IN33_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R107_FTDM_IN33_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R107_FTDM_IN33_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R107_FTDM_IN33_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R107_FTDM_IN33_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R107_FTDM_IN33_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R107_FTDM_IN33_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R107_FTDM_IN33_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R107_FTDM_IN33_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R107_FTDM_IN33_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R107_FTDM_IN33_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R107_FTDM_IN33_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R107_FTDM_IN33_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R107_FTDM_IN33_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN34 (IC 1) */
#define DSP1_R108_FTDM_IN34_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R108_FTDM_IN34_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R108_FTDM_IN34_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R108_FTDM_IN34_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R108_FTDM_IN34_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R108_FTDM_IN34_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R108_FTDM_IN34_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R108_FTDM_IN34_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R108_FTDM_IN34_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R108_FTDM_IN34_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R108_FTDM_IN34_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R108_FTDM_IN34_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R108_FTDM_IN34_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R108_FTDM_IN34_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R108_FTDM_IN34_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN35 (IC 1) */
#define DSP1_R109_FTDM_IN35_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R109_FTDM_IN35_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R109_FTDM_IN35_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R109_FTDM_IN35_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R109_FTDM_IN35_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R109_FTDM_IN35_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R109_FTDM_IN35_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R109_FTDM_IN35_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R109_FTDM_IN35_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R109_FTDM_IN35_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R109_FTDM_IN35_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R109_FTDM_IN35_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R109_FTDM_IN35_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R109_FTDM_IN35_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R109_FTDM_IN35_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN36 (IC 1) */
#define DSP1_R110_FTDM_IN36_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R110_FTDM_IN36_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R110_FTDM_IN36_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R110_FTDM_IN36_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R110_FTDM_IN36_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R110_FTDM_IN36_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R110_FTDM_IN36_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R110_FTDM_IN36_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R110_FTDM_IN36_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R110_FTDM_IN36_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R110_FTDM_IN36_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R110_FTDM_IN36_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R110_FTDM_IN36_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R110_FTDM_IN36_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R110_FTDM_IN36_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN37 (IC 1) */
#define DSP1_R111_FTDM_IN37_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R111_FTDM_IN37_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R111_FTDM_IN37_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R111_FTDM_IN37_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R111_FTDM_IN37_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R111_FTDM_IN37_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R111_FTDM_IN37_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R111_FTDM_IN37_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R111_FTDM_IN37_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R111_FTDM_IN37_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R111_FTDM_IN37_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R111_FTDM_IN37_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R111_FTDM_IN37_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R111_FTDM_IN37_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R111_FTDM_IN37_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN38 (IC 1) */
#define DSP1_R112_FTDM_IN38_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R112_FTDM_IN38_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R112_FTDM_IN38_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R112_FTDM_IN38_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R112_FTDM_IN38_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R112_FTDM_IN38_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R112_FTDM_IN38_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R112_FTDM_IN38_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R112_FTDM_IN38_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R112_FTDM_IN38_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R112_FTDM_IN38_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R112_FTDM_IN38_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R112_FTDM_IN38_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R112_FTDM_IN38_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R112_FTDM_IN38_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN39 (IC 1) */
#define DSP1_R113_FTDM_IN39_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R113_FTDM_IN39_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R113_FTDM_IN39_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R113_FTDM_IN39_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R113_FTDM_IN39_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R113_FTDM_IN39_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R113_FTDM_IN39_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R113_FTDM_IN39_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R113_FTDM_IN39_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R113_FTDM_IN39_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R113_FTDM_IN39_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R113_FTDM_IN39_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R113_FTDM_IN39_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R113_FTDM_IN39_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R113_FTDM_IN39_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN40 (IC 1) */
#define DSP1_R114_FTDM_IN40_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R114_FTDM_IN40_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R114_FTDM_IN40_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R114_FTDM_IN40_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R114_FTDM_IN40_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R114_FTDM_IN40_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R114_FTDM_IN40_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R114_FTDM_IN40_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R114_FTDM_IN40_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R114_FTDM_IN40_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R114_FTDM_IN40_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R114_FTDM_IN40_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R114_FTDM_IN40_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R114_FTDM_IN40_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R114_FTDM_IN40_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN41 (IC 1) */
#define DSP1_R115_FTDM_IN41_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R115_FTDM_IN41_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R115_FTDM_IN41_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R115_FTDM_IN41_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R115_FTDM_IN41_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R115_FTDM_IN41_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R115_FTDM_IN41_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R115_FTDM_IN41_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R115_FTDM_IN41_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R115_FTDM_IN41_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R115_FTDM_IN41_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R115_FTDM_IN41_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R115_FTDM_IN41_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R115_FTDM_IN41_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R115_FTDM_IN41_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN42 (IC 1) */
#define DSP1_R116_FTDM_IN42_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R116_FTDM_IN42_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R116_FTDM_IN42_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R116_FTDM_IN42_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R116_FTDM_IN42_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R116_FTDM_IN42_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R116_FTDM_IN42_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R116_FTDM_IN42_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R116_FTDM_IN42_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R116_FTDM_IN42_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R116_FTDM_IN42_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R116_FTDM_IN42_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R116_FTDM_IN42_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R116_FTDM_IN42_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R116_FTDM_IN42_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN43 (IC 1) */
#define DSP1_R117_FTDM_IN43_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R117_FTDM_IN43_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R117_FTDM_IN43_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R117_FTDM_IN43_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R117_FTDM_IN43_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R117_FTDM_IN43_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R117_FTDM_IN43_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R117_FTDM_IN43_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R117_FTDM_IN43_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R117_FTDM_IN43_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R117_FTDM_IN43_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R117_FTDM_IN43_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R117_FTDM_IN43_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R117_FTDM_IN43_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R117_FTDM_IN43_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN44 (IC 1) */
#define DSP1_R118_FTDM_IN44_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R118_FTDM_IN44_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R118_FTDM_IN44_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R118_FTDM_IN44_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R118_FTDM_IN44_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R118_FTDM_IN44_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R118_FTDM_IN44_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R118_FTDM_IN44_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R118_FTDM_IN44_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R118_FTDM_IN44_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R118_FTDM_IN44_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R118_FTDM_IN44_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R118_FTDM_IN44_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R118_FTDM_IN44_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R118_FTDM_IN44_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN45 (IC 1) */
#define DSP1_R119_FTDM_IN45_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R119_FTDM_IN45_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R119_FTDM_IN45_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R119_FTDM_IN45_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R119_FTDM_IN45_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R119_FTDM_IN45_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R119_FTDM_IN45_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R119_FTDM_IN45_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R119_FTDM_IN45_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R119_FTDM_IN45_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R119_FTDM_IN45_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R119_FTDM_IN45_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R119_FTDM_IN45_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R119_FTDM_IN45_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R119_FTDM_IN45_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN46 (IC 1) */
#define DSP1_R120_FTDM_IN46_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R120_FTDM_IN46_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R120_FTDM_IN46_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R120_FTDM_IN46_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R120_FTDM_IN46_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R120_FTDM_IN46_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R120_FTDM_IN46_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R120_FTDM_IN46_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R120_FTDM_IN46_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R120_FTDM_IN46_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R120_FTDM_IN46_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R120_FTDM_IN46_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R120_FTDM_IN46_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R120_FTDM_IN46_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R120_FTDM_IN46_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN47 (IC 1) */
#define DSP1_R121_FTDM_IN47_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R121_FTDM_IN47_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R121_FTDM_IN47_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R121_FTDM_IN47_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R121_FTDM_IN47_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R121_FTDM_IN47_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R121_FTDM_IN47_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R121_FTDM_IN47_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R121_FTDM_IN47_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R121_FTDM_IN47_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R121_FTDM_IN47_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R121_FTDM_IN47_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R121_FTDM_IN47_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R121_FTDM_IN47_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R121_FTDM_IN47_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN48 (IC 1) */
#define DSP1_R122_FTDM_IN48_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R122_FTDM_IN48_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R122_FTDM_IN48_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R122_FTDM_IN48_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R122_FTDM_IN48_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R122_FTDM_IN48_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R122_FTDM_IN48_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R122_FTDM_IN48_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R122_FTDM_IN48_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R122_FTDM_IN48_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R122_FTDM_IN48_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R122_FTDM_IN48_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R122_FTDM_IN48_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R122_FTDM_IN48_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R122_FTDM_IN48_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN49 (IC 1) */
#define DSP1_R123_FTDM_IN49_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R123_FTDM_IN49_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R123_FTDM_IN49_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R123_FTDM_IN49_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R123_FTDM_IN49_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R123_FTDM_IN49_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R123_FTDM_IN49_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R123_FTDM_IN49_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R123_FTDM_IN49_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R123_FTDM_IN49_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R123_FTDM_IN49_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R123_FTDM_IN49_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R123_FTDM_IN49_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R123_FTDM_IN49_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R123_FTDM_IN49_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN50 (IC 1) */
#define DSP1_R124_FTDM_IN50_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R124_FTDM_IN50_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R124_FTDM_IN50_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R124_FTDM_IN50_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R124_FTDM_IN50_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R124_FTDM_IN50_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R124_FTDM_IN50_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R124_FTDM_IN50_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R124_FTDM_IN50_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R124_FTDM_IN50_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R124_FTDM_IN50_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R124_FTDM_IN50_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R124_FTDM_IN50_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R124_FTDM_IN50_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R124_FTDM_IN50_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN51 (IC 1) */
#define DSP1_R125_FTDM_IN51_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R125_FTDM_IN51_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R125_FTDM_IN51_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R125_FTDM_IN51_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R125_FTDM_IN51_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R125_FTDM_IN51_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R125_FTDM_IN51_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R125_FTDM_IN51_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R125_FTDM_IN51_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R125_FTDM_IN51_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R125_FTDM_IN51_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R125_FTDM_IN51_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R125_FTDM_IN51_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R125_FTDM_IN51_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R125_FTDM_IN51_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN52 (IC 1) */
#define DSP1_R126_FTDM_IN52_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R126_FTDM_IN52_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R126_FTDM_IN52_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R126_FTDM_IN52_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R126_FTDM_IN52_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R126_FTDM_IN52_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R126_FTDM_IN52_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R126_FTDM_IN52_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R126_FTDM_IN52_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R126_FTDM_IN52_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R126_FTDM_IN52_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R126_FTDM_IN52_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R126_FTDM_IN52_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R126_FTDM_IN52_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R126_FTDM_IN52_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN53 (IC 1) */
#define DSP1_R127_FTDM_IN53_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R127_FTDM_IN53_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R127_FTDM_IN53_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R127_FTDM_IN53_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R127_FTDM_IN53_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R127_FTDM_IN53_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R127_FTDM_IN53_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R127_FTDM_IN53_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R127_FTDM_IN53_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R127_FTDM_IN53_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R127_FTDM_IN53_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R127_FTDM_IN53_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R127_FTDM_IN53_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R127_FTDM_IN53_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R127_FTDM_IN53_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN54 (IC 1) */
#define DSP1_R128_FTDM_IN54_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R128_FTDM_IN54_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R128_FTDM_IN54_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R128_FTDM_IN54_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R128_FTDM_IN54_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R128_FTDM_IN54_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R128_FTDM_IN54_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R128_FTDM_IN54_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R128_FTDM_IN54_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R128_FTDM_IN54_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R128_FTDM_IN54_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R128_FTDM_IN54_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R128_FTDM_IN54_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R128_FTDM_IN54_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R128_FTDM_IN54_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN55 (IC 1) */
#define DSP1_R129_FTDM_IN55_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R129_FTDM_IN55_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R129_FTDM_IN55_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R129_FTDM_IN55_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R129_FTDM_IN55_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R129_FTDM_IN55_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R129_FTDM_IN55_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R129_FTDM_IN55_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R129_FTDM_IN55_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R129_FTDM_IN55_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R129_FTDM_IN55_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R129_FTDM_IN55_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R129_FTDM_IN55_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R129_FTDM_IN55_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R129_FTDM_IN55_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN56 (IC 1) */
#define DSP1_R130_FTDM_IN56_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R130_FTDM_IN56_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R130_FTDM_IN56_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R130_FTDM_IN56_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R130_FTDM_IN56_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R130_FTDM_IN56_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R130_FTDM_IN56_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R130_FTDM_IN56_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R130_FTDM_IN56_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R130_FTDM_IN56_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R130_FTDM_IN56_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R130_FTDM_IN56_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R130_FTDM_IN56_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R130_FTDM_IN56_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R130_FTDM_IN56_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN57 (IC 1) */
#define DSP1_R131_FTDM_IN57_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R131_FTDM_IN57_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R131_FTDM_IN57_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R131_FTDM_IN57_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R131_FTDM_IN57_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R131_FTDM_IN57_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R131_FTDM_IN57_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R131_FTDM_IN57_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R131_FTDM_IN57_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R131_FTDM_IN57_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R131_FTDM_IN57_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R131_FTDM_IN57_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R131_FTDM_IN57_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R131_FTDM_IN57_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R131_FTDM_IN57_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN58 (IC 1) */
#define DSP1_R132_FTDM_IN58_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R132_FTDM_IN58_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R132_FTDM_IN58_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R132_FTDM_IN58_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R132_FTDM_IN58_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R132_FTDM_IN58_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R132_FTDM_IN58_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R132_FTDM_IN58_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R132_FTDM_IN58_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R132_FTDM_IN58_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R132_FTDM_IN58_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R132_FTDM_IN58_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R132_FTDM_IN58_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R132_FTDM_IN58_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R132_FTDM_IN58_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN59 (IC 1) */
#define DSP1_R133_FTDM_IN59_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R133_FTDM_IN59_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R133_FTDM_IN59_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R133_FTDM_IN59_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R133_FTDM_IN59_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R133_FTDM_IN59_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R133_FTDM_IN59_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R133_FTDM_IN59_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R133_FTDM_IN59_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R133_FTDM_IN59_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R133_FTDM_IN59_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R133_FTDM_IN59_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R133_FTDM_IN59_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R133_FTDM_IN59_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R133_FTDM_IN59_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN60 (IC 1) */
#define DSP1_R134_FTDM_IN60_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R134_FTDM_IN60_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R134_FTDM_IN60_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R134_FTDM_IN60_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R134_FTDM_IN60_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R134_FTDM_IN60_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R134_FTDM_IN60_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R134_FTDM_IN60_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R134_FTDM_IN60_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R134_FTDM_IN60_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R134_FTDM_IN60_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R134_FTDM_IN60_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R134_FTDM_IN60_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R134_FTDM_IN60_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R134_FTDM_IN60_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN61 (IC 1) */
#define DSP1_R135_FTDM_IN61_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R135_FTDM_IN61_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R135_FTDM_IN61_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R135_FTDM_IN61_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R135_FTDM_IN61_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R135_FTDM_IN61_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R135_FTDM_IN61_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R135_FTDM_IN61_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R135_FTDM_IN61_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R135_FTDM_IN61_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R135_FTDM_IN61_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R135_FTDM_IN61_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R135_FTDM_IN61_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R135_FTDM_IN61_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R135_FTDM_IN61_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN62 (IC 1) */
#define DSP1_R136_FTDM_IN62_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R136_FTDM_IN62_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R136_FTDM_IN62_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R136_FTDM_IN62_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R136_FTDM_IN62_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R136_FTDM_IN62_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R136_FTDM_IN62_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R136_FTDM_IN62_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R136_FTDM_IN62_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R136_FTDM_IN62_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R136_FTDM_IN62_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R136_FTDM_IN62_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R136_FTDM_IN62_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R136_FTDM_IN62_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R136_FTDM_IN62_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_IN63 (IC 1) */
#define DSP1_R137_FTDM_IN63_BYTE_IN_POS_IC_1           0x0    /* 00b	[1:0] */
#define DSP1_R137_FTDM_IN63_CHANNEL_IN_POS_IC_1        0x0    /* 000b	[4:2] */
#define DSP1_R137_FTDM_IN63_SERIAL_IN_SEL_IC_1         0x0    /* 0b	[5] */
#define DSP1_R137_FTDM_IN63_REVERSE_IN_BYTE_IC_1       0x0    /* 0b	[6] */
#define DSP1_R137_FTDM_IN63_SLOT_ENABLE_IN_IC_1        0x0    /* 0b	[7] */
#define DSP1_R137_FTDM_IN63_BYTE_IN_POS_IC_1_MASK      0x3
#define DSP1_R137_FTDM_IN63_BYTE_IN_POS_IC_1_SHIFT     0
#define DSP1_R137_FTDM_IN63_CHANNEL_IN_POS_IC_1_MASK   0x1C
#define DSP1_R137_FTDM_IN63_CHANNEL_IN_POS_IC_1_SHIFT  2
#define DSP1_R137_FTDM_IN63_SERIAL_IN_SEL_IC_1_MASK    0x20
#define DSP1_R137_FTDM_IN63_SERIAL_IN_SEL_IC_1_SHIFT   5
#define DSP1_R137_FTDM_IN63_REVERSE_IN_BYTE_IC_1_MASK  0x40
#define DSP1_R137_FTDM_IN63_REVERSE_IN_BYTE_IC_1_SHIFT 6
#define DSP1_R137_FTDM_IN63_SLOT_ENABLE_IN_IC_1_MASK   0x80
#define DSP1_R137_FTDM_IN63_SLOT_ENABLE_IN_IC_1_SHIFT  7

/* FTDM_OUT0 (IC 1) */
#define DSP1_R138_FTDM_OUT0_BYTE_OUT_POS_IC_1          0x0    /* 00b	[1:0] */
#define DSP1_R138_FTDM_OUT0_CHANNEL_OUT_POS_IC_1       0x0    /* 000b	[4:2] */
#define DSP1_R138_FTDM_OUT0_SERIAL_OUT_SEL_IC_1        0x0    /* 0b	[5] */
#define DSP1_R138_FTDM_OUT0_REVERSE_OUT_BYTE_IC_1      0x0    /* 0b	[6] */
#define DSP1_R138_FTDM_OUT0_SLOT_ENABLE_OUT_IC_1       0x0    /* 0b	[7] */
#define DSP1_R138_FTDM_OUT0_BYTE_OUT_POS_IC_1_MASK     0x3
#define DSP1_R138_FTDM_OUT0_BYTE_OUT_POS_IC_1_SHIFT    0
#define DSP1_R138_FTDM_OUT0_CHANNEL_OUT_POS_IC_1_MASK  0x1C
#define DSP1_R138_FTDM_OUT0_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R138_FTDM_OUT0_SERIAL_OUT_SEL_IC_1_MASK   0x20
#define DSP1_R138_FTDM_OUT0_SERIAL_OUT_SEL_IC_1_SHIFT  5
#define DSP1_R138_FTDM_OUT0_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R138_FTDM_OUT0_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R138_FTDM_OUT0_SLOT_ENABLE_OUT_IC_1_MASK  0x80
#define DSP1_R138_FTDM_OUT0_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT1 (IC 1) */
#define DSP1_R139_FTDM_OUT1_BYTE_OUT_POS_IC_1          0x0    /* 00b	[1:0] */
#define DSP1_R139_FTDM_OUT1_CHANNEL_OUT_POS_IC_1       0x0    /* 000b	[4:2] */
#define DSP1_R139_FTDM_OUT1_SERIAL_OUT_SEL_IC_1        0x0    /* 0b	[5] */
#define DSP1_R139_FTDM_OUT1_REVERSE_OUT_BYTE_IC_1      0x0    /* 0b	[6] */
#define DSP1_R139_FTDM_OUT1_SLOT_ENABLE_OUT_IC_1       0x0    /* 0b	[7] */
#define DSP1_R139_FTDM_OUT1_BYTE_OUT_POS_IC_1_MASK     0x3
#define DSP1_R139_FTDM_OUT1_BYTE_OUT_POS_IC_1_SHIFT    0
#define DSP1_R139_FTDM_OUT1_CHANNEL_OUT_POS_IC_1_MASK  0x1C
#define DSP1_R139_FTDM_OUT1_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R139_FTDM_OUT1_SERIAL_OUT_SEL_IC_1_MASK   0x20
#define DSP1_R139_FTDM_OUT1_SERIAL_OUT_SEL_IC_1_SHIFT  5
#define DSP1_R139_FTDM_OUT1_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R139_FTDM_OUT1_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R139_FTDM_OUT1_SLOT_ENABLE_OUT_IC_1_MASK  0x80
#define DSP1_R139_FTDM_OUT1_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT2 (IC 1) */
#define DSP1_R140_FTDM_OUT2_BYTE_OUT_POS_IC_1          0x0    /* 00b	[1:0] */
#define DSP1_R140_FTDM_OUT2_CHANNEL_OUT_POS_IC_1       0x0    /* 000b	[4:2] */
#define DSP1_R140_FTDM_OUT2_SERIAL_OUT_SEL_IC_1        0x0    /* 0b	[5] */
#define DSP1_R140_FTDM_OUT2_REVERSE_OUT_BYTE_IC_1      0x0    /* 0b	[6] */
#define DSP1_R140_FTDM_OUT2_SLOT_ENABLE_OUT_IC_1       0x0    /* 0b	[7] */
#define DSP1_R140_FTDM_OUT2_BYTE_OUT_POS_IC_1_MASK     0x3
#define DSP1_R140_FTDM_OUT2_BYTE_OUT_POS_IC_1_SHIFT    0
#define DSP1_R140_FTDM_OUT2_CHANNEL_OUT_POS_IC_1_MASK  0x1C
#define DSP1_R140_FTDM_OUT2_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R140_FTDM_OUT2_SERIAL_OUT_SEL_IC_1_MASK   0x20
#define DSP1_R140_FTDM_OUT2_SERIAL_OUT_SEL_IC_1_SHIFT  5
#define DSP1_R140_FTDM_OUT2_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R140_FTDM_OUT2_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R140_FTDM_OUT2_SLOT_ENABLE_OUT_IC_1_MASK  0x80
#define DSP1_R140_FTDM_OUT2_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT3 (IC 1) */
#define DSP1_R141_FTDM_OUT3_BYTE_OUT_POS_IC_1          0x0    /* 00b	[1:0] */
#define DSP1_R141_FTDM_OUT3_CHANNEL_OUT_POS_IC_1       0x0    /* 000b	[4:2] */
#define DSP1_R141_FTDM_OUT3_SERIAL_OUT_SEL_IC_1        0x0    /* 0b	[5] */
#define DSP1_R141_FTDM_OUT3_REVERSE_OUT_BYTE_IC_1      0x0    /* 0b	[6] */
#define DSP1_R141_FTDM_OUT3_SLOT_ENABLE_OUT_IC_1       0x0    /* 0b	[7] */
#define DSP1_R141_FTDM_OUT3_BYTE_OUT_POS_IC_1_MASK     0x3
#define DSP1_R141_FTDM_OUT3_BYTE_OUT_POS_IC_1_SHIFT    0
#define DSP1_R141_FTDM_OUT3_CHANNEL_OUT_POS_IC_1_MASK  0x1C
#define DSP1_R141_FTDM_OUT3_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R141_FTDM_OUT3_SERIAL_OUT_SEL_IC_1_MASK   0x20
#define DSP1_R141_FTDM_OUT3_SERIAL_OUT_SEL_IC_1_SHIFT  5
#define DSP1_R141_FTDM_OUT3_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R141_FTDM_OUT3_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R141_FTDM_OUT3_SLOT_ENABLE_OUT_IC_1_MASK  0x80
#define DSP1_R141_FTDM_OUT3_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT4 (IC 1) */
#define DSP1_R142_FTDM_OUT4_BYTE_OUT_POS_IC_1          0x0    /* 00b	[1:0] */
#define DSP1_R142_FTDM_OUT4_CHANNEL_OUT_POS_IC_1       0x0    /* 000b	[4:2] */
#define DSP1_R142_FTDM_OUT4_SERIAL_OUT_SEL_IC_1        0x0    /* 0b	[5] */
#define DSP1_R142_FTDM_OUT4_REVERSE_OUT_BYTE_IC_1      0x0    /* 0b	[6] */
#define DSP1_R142_FTDM_OUT4_SLOT_ENABLE_OUT_IC_1       0x0    /* 0b	[7] */
#define DSP1_R142_FTDM_OUT4_BYTE_OUT_POS_IC_1_MASK     0x3
#define DSP1_R142_FTDM_OUT4_BYTE_OUT_POS_IC_1_SHIFT    0
#define DSP1_R142_FTDM_OUT4_CHANNEL_OUT_POS_IC_1_MASK  0x1C
#define DSP1_R142_FTDM_OUT4_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R142_FTDM_OUT4_SERIAL_OUT_SEL_IC_1_MASK   0x20
#define DSP1_R142_FTDM_OUT4_SERIAL_OUT_SEL_IC_1_SHIFT  5
#define DSP1_R142_FTDM_OUT4_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R142_FTDM_OUT4_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R142_FTDM_OUT4_SLOT_ENABLE_OUT_IC_1_MASK  0x80
#define DSP1_R142_FTDM_OUT4_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT5 (IC 1) */
#define DSP1_R143_FTDM_OUT5_BYTE_OUT_POS_IC_1          0x0    /* 00b	[1:0] */
#define DSP1_R143_FTDM_OUT5_CHANNEL_OUT_POS_IC_1       0x0    /* 000b	[4:2] */
#define DSP1_R143_FTDM_OUT5_SERIAL_OUT_SEL_IC_1        0x0    /* 0b	[5] */
#define DSP1_R143_FTDM_OUT5_REVERSE_OUT_BYTE_IC_1      0x0    /* 0b	[6] */
#define DSP1_R143_FTDM_OUT5_SLOT_ENABLE_OUT_IC_1       0x0    /* 0b	[7] */
#define DSP1_R143_FTDM_OUT5_BYTE_OUT_POS_IC_1_MASK     0x3
#define DSP1_R143_FTDM_OUT5_BYTE_OUT_POS_IC_1_SHIFT    0
#define DSP1_R143_FTDM_OUT5_CHANNEL_OUT_POS_IC_1_MASK  0x1C
#define DSP1_R143_FTDM_OUT5_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R143_FTDM_OUT5_SERIAL_OUT_SEL_IC_1_MASK   0x20
#define DSP1_R143_FTDM_OUT5_SERIAL_OUT_SEL_IC_1_SHIFT  5
#define DSP1_R143_FTDM_OUT5_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R143_FTDM_OUT5_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R143_FTDM_OUT5_SLOT_ENABLE_OUT_IC_1_MASK  0x80
#define DSP1_R143_FTDM_OUT5_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT6 (IC 1) */
#define DSP1_R144_FTDM_OUT6_BYTE_OUT_POS_IC_1          0x0    /* 00b	[1:0] */
#define DSP1_R144_FTDM_OUT6_CHANNEL_OUT_POS_IC_1       0x0    /* 000b	[4:2] */
#define DSP1_R144_FTDM_OUT6_SERIAL_OUT_SEL_IC_1        0x0    /* 0b	[5] */
#define DSP1_R144_FTDM_OUT6_REVERSE_OUT_BYTE_IC_1      0x0    /* 0b	[6] */
#define DSP1_R144_FTDM_OUT6_SLOT_ENABLE_OUT_IC_1       0x0    /* 0b	[7] */
#define DSP1_R144_FTDM_OUT6_BYTE_OUT_POS_IC_1_MASK     0x3
#define DSP1_R144_FTDM_OUT6_BYTE_OUT_POS_IC_1_SHIFT    0
#define DSP1_R144_FTDM_OUT6_CHANNEL_OUT_POS_IC_1_MASK  0x1C
#define DSP1_R144_FTDM_OUT6_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R144_FTDM_OUT6_SERIAL_OUT_SEL_IC_1_MASK   0x20
#define DSP1_R144_FTDM_OUT6_SERIAL_OUT_SEL_IC_1_SHIFT  5
#define DSP1_R144_FTDM_OUT6_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R144_FTDM_OUT6_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R144_FTDM_OUT6_SLOT_ENABLE_OUT_IC_1_MASK  0x80
#define DSP1_R144_FTDM_OUT6_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT7 (IC 1) */
#define DSP1_R145_FTDM_OUT7_BYTE_OUT_POS_IC_1          0x0    /* 00b	[1:0] */
#define DSP1_R145_FTDM_OUT7_CHANNEL_OUT_POS_IC_1       0x0    /* 000b	[4:2] */
#define DSP1_R145_FTDM_OUT7_SERIAL_OUT_SEL_IC_1        0x0    /* 0b	[5] */
#define DSP1_R145_FTDM_OUT7_REVERSE_OUT_BYTE_IC_1      0x0    /* 0b	[6] */
#define DSP1_R145_FTDM_OUT7_SLOT_ENABLE_OUT_IC_1       0x0    /* 0b	[7] */
#define DSP1_R145_FTDM_OUT7_BYTE_OUT_POS_IC_1_MASK     0x3
#define DSP1_R145_FTDM_OUT7_BYTE_OUT_POS_IC_1_SHIFT    0
#define DSP1_R145_FTDM_OUT7_CHANNEL_OUT_POS_IC_1_MASK  0x1C
#define DSP1_R145_FTDM_OUT7_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R145_FTDM_OUT7_SERIAL_OUT_SEL_IC_1_MASK   0x20
#define DSP1_R145_FTDM_OUT7_SERIAL_OUT_SEL_IC_1_SHIFT  5
#define DSP1_R145_FTDM_OUT7_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R145_FTDM_OUT7_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R145_FTDM_OUT7_SLOT_ENABLE_OUT_IC_1_MASK  0x80
#define DSP1_R145_FTDM_OUT7_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT8 (IC 1) */
#define DSP1_R146_FTDM_OUT8_BYTE_OUT_POS_IC_1          0x0    /* 00b	[1:0] */
#define DSP1_R146_FTDM_OUT8_CHANNEL_OUT_POS_IC_1       0x0    /* 000b	[4:2] */
#define DSP1_R146_FTDM_OUT8_SERIAL_OUT_SEL_IC_1        0x0    /* 0b	[5] */
#define DSP1_R146_FTDM_OUT8_REVERSE_OUT_BYTE_IC_1      0x0    /* 0b	[6] */
#define DSP1_R146_FTDM_OUT8_SLOT_ENABLE_OUT_IC_1       0x0    /* 0b	[7] */
#define DSP1_R146_FTDM_OUT8_BYTE_OUT_POS_IC_1_MASK     0x3
#define DSP1_R146_FTDM_OUT8_BYTE_OUT_POS_IC_1_SHIFT    0
#define DSP1_R146_FTDM_OUT8_CHANNEL_OUT_POS_IC_1_MASK  0x1C
#define DSP1_R146_FTDM_OUT8_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R146_FTDM_OUT8_SERIAL_OUT_SEL_IC_1_MASK   0x20
#define DSP1_R146_FTDM_OUT8_SERIAL_OUT_SEL_IC_1_SHIFT  5
#define DSP1_R146_FTDM_OUT8_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R146_FTDM_OUT8_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R146_FTDM_OUT8_SLOT_ENABLE_OUT_IC_1_MASK  0x80
#define DSP1_R146_FTDM_OUT8_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT9 (IC 1) */
#define DSP1_R147_FTDM_OUT9_BYTE_OUT_POS_IC_1          0x0    /* 00b	[1:0] */
#define DSP1_R147_FTDM_OUT9_CHANNEL_OUT_POS_IC_1       0x0    /* 000b	[4:2] */
#define DSP1_R147_FTDM_OUT9_SERIAL_OUT_SEL_IC_1        0x0    /* 0b	[5] */
#define DSP1_R147_FTDM_OUT9_REVERSE_OUT_BYTE_IC_1      0x0    /* 0b	[6] */
#define DSP1_R147_FTDM_OUT9_SLOT_ENABLE_OUT_IC_1       0x0    /* 0b	[7] */
#define DSP1_R147_FTDM_OUT9_BYTE_OUT_POS_IC_1_MASK     0x3
#define DSP1_R147_FTDM_OUT9_BYTE_OUT_POS_IC_1_SHIFT    0
#define DSP1_R147_FTDM_OUT9_CHANNEL_OUT_POS_IC_1_MASK  0x1C
#define DSP1_R147_FTDM_OUT9_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R147_FTDM_OUT9_SERIAL_OUT_SEL_IC_1_MASK   0x20
#define DSP1_R147_FTDM_OUT9_SERIAL_OUT_SEL_IC_1_SHIFT  5
#define DSP1_R147_FTDM_OUT9_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R147_FTDM_OUT9_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R147_FTDM_OUT9_SLOT_ENABLE_OUT_IC_1_MASK  0x80
#define DSP1_R147_FTDM_OUT9_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT10 (IC 1) */
#define DSP1_R148_FTDM_OUT10_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R148_FTDM_OUT10_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R148_FTDM_OUT10_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R148_FTDM_OUT10_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R148_FTDM_OUT10_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R148_FTDM_OUT10_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R148_FTDM_OUT10_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R148_FTDM_OUT10_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R148_FTDM_OUT10_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R148_FTDM_OUT10_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R148_FTDM_OUT10_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R148_FTDM_OUT10_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R148_FTDM_OUT10_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R148_FTDM_OUT10_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R148_FTDM_OUT10_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT11 (IC 1) */
#define DSP1_R149_FTDM_OUT11_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R149_FTDM_OUT11_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R149_FTDM_OUT11_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R149_FTDM_OUT11_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R149_FTDM_OUT11_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R149_FTDM_OUT11_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R149_FTDM_OUT11_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R149_FTDM_OUT11_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R149_FTDM_OUT11_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R149_FTDM_OUT11_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R149_FTDM_OUT11_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R149_FTDM_OUT11_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R149_FTDM_OUT11_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R149_FTDM_OUT11_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R149_FTDM_OUT11_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT12 (IC 1) */
#define DSP1_R150_FTDM_OUT12_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R150_FTDM_OUT12_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R150_FTDM_OUT12_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R150_FTDM_OUT12_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R150_FTDM_OUT12_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R150_FTDM_OUT12_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R150_FTDM_OUT12_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R150_FTDM_OUT12_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R150_FTDM_OUT12_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R150_FTDM_OUT12_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R150_FTDM_OUT12_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R150_FTDM_OUT12_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R150_FTDM_OUT12_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R150_FTDM_OUT12_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R150_FTDM_OUT12_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT13 (IC 1) */
#define DSP1_R151_FTDM_OUT13_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R151_FTDM_OUT13_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R151_FTDM_OUT13_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R151_FTDM_OUT13_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R151_FTDM_OUT13_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R151_FTDM_OUT13_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R151_FTDM_OUT13_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R151_FTDM_OUT13_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R151_FTDM_OUT13_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R151_FTDM_OUT13_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R151_FTDM_OUT13_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R151_FTDM_OUT13_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R151_FTDM_OUT13_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R151_FTDM_OUT13_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R151_FTDM_OUT13_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT14 (IC 1) */
#define DSP1_R152_FTDM_OUT14_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R152_FTDM_OUT14_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R152_FTDM_OUT14_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R152_FTDM_OUT14_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R152_FTDM_OUT14_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R152_FTDM_OUT14_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R152_FTDM_OUT14_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R152_FTDM_OUT14_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R152_FTDM_OUT14_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R152_FTDM_OUT14_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R152_FTDM_OUT14_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R152_FTDM_OUT14_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R152_FTDM_OUT14_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R152_FTDM_OUT14_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R152_FTDM_OUT14_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT15 (IC 1) */
#define DSP1_R153_FTDM_OUT15_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R153_FTDM_OUT15_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R153_FTDM_OUT15_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R153_FTDM_OUT15_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R153_FTDM_OUT15_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R153_FTDM_OUT15_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R153_FTDM_OUT15_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R153_FTDM_OUT15_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R153_FTDM_OUT15_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R153_FTDM_OUT15_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R153_FTDM_OUT15_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R153_FTDM_OUT15_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R153_FTDM_OUT15_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R153_FTDM_OUT15_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R153_FTDM_OUT15_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT16 (IC 1) */
#define DSP1_R154_FTDM_OUT16_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R154_FTDM_OUT16_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R154_FTDM_OUT16_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R154_FTDM_OUT16_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R154_FTDM_OUT16_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R154_FTDM_OUT16_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R154_FTDM_OUT16_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R154_FTDM_OUT16_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R154_FTDM_OUT16_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R154_FTDM_OUT16_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R154_FTDM_OUT16_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R154_FTDM_OUT16_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R154_FTDM_OUT16_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R154_FTDM_OUT16_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R154_FTDM_OUT16_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT17 (IC 1) */
#define DSP1_R155_FTDM_OUT17_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R155_FTDM_OUT17_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R155_FTDM_OUT17_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R155_FTDM_OUT17_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R155_FTDM_OUT17_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R155_FTDM_OUT17_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R155_FTDM_OUT17_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R155_FTDM_OUT17_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R155_FTDM_OUT17_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R155_FTDM_OUT17_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R155_FTDM_OUT17_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R155_FTDM_OUT17_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R155_FTDM_OUT17_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R155_FTDM_OUT17_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R155_FTDM_OUT17_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT18 (IC 1) */
#define DSP1_R156_FTDM_OUT18_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R156_FTDM_OUT18_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R156_FTDM_OUT18_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R156_FTDM_OUT18_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R156_FTDM_OUT18_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R156_FTDM_OUT18_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R156_FTDM_OUT18_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R156_FTDM_OUT18_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R156_FTDM_OUT18_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R156_FTDM_OUT18_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R156_FTDM_OUT18_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R156_FTDM_OUT18_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R156_FTDM_OUT18_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R156_FTDM_OUT18_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R156_FTDM_OUT18_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT19 (IC 1) */
#define DSP1_R157_FTDM_OUT19_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R157_FTDM_OUT19_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R157_FTDM_OUT19_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R157_FTDM_OUT19_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R157_FTDM_OUT19_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R157_FTDM_OUT19_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R157_FTDM_OUT19_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R157_FTDM_OUT19_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R157_FTDM_OUT19_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R157_FTDM_OUT19_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R157_FTDM_OUT19_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R157_FTDM_OUT19_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R157_FTDM_OUT19_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R157_FTDM_OUT19_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R157_FTDM_OUT19_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT20 (IC 1) */
#define DSP1_R158_FTDM_OUT20_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R158_FTDM_OUT20_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R158_FTDM_OUT20_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R158_FTDM_OUT20_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R158_FTDM_OUT20_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R158_FTDM_OUT20_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R158_FTDM_OUT20_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R158_FTDM_OUT20_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R158_FTDM_OUT20_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R158_FTDM_OUT20_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R158_FTDM_OUT20_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R158_FTDM_OUT20_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R158_FTDM_OUT20_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R158_FTDM_OUT20_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R158_FTDM_OUT20_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT21 (IC 1) */
#define DSP1_R159_FTDM_OUT21_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R159_FTDM_OUT21_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R159_FTDM_OUT21_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R159_FTDM_OUT21_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R159_FTDM_OUT21_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R159_FTDM_OUT21_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R159_FTDM_OUT21_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R159_FTDM_OUT21_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R159_FTDM_OUT21_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R159_FTDM_OUT21_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R159_FTDM_OUT21_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R159_FTDM_OUT21_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R159_FTDM_OUT21_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R159_FTDM_OUT21_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R159_FTDM_OUT21_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT22 (IC 1) */
#define DSP1_R160_FTDM_OUT22_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R160_FTDM_OUT22_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R160_FTDM_OUT22_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R160_FTDM_OUT22_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R160_FTDM_OUT22_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R160_FTDM_OUT22_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R160_FTDM_OUT22_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R160_FTDM_OUT22_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R160_FTDM_OUT22_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R160_FTDM_OUT22_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R160_FTDM_OUT22_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R160_FTDM_OUT22_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R160_FTDM_OUT22_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R160_FTDM_OUT22_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R160_FTDM_OUT22_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT23 (IC 1) */
#define DSP1_R161_FTDM_OUT23_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R161_FTDM_OUT23_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R161_FTDM_OUT23_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R161_FTDM_OUT23_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R161_FTDM_OUT23_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R161_FTDM_OUT23_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R161_FTDM_OUT23_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R161_FTDM_OUT23_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R161_FTDM_OUT23_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R161_FTDM_OUT23_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R161_FTDM_OUT23_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R161_FTDM_OUT23_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R161_FTDM_OUT23_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R161_FTDM_OUT23_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R161_FTDM_OUT23_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT24 (IC 1) */
#define DSP1_R162_FTDM_OUT24_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R162_FTDM_OUT24_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R162_FTDM_OUT24_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R162_FTDM_OUT24_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R162_FTDM_OUT24_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R162_FTDM_OUT24_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R162_FTDM_OUT24_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R162_FTDM_OUT24_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R162_FTDM_OUT24_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R162_FTDM_OUT24_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R162_FTDM_OUT24_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R162_FTDM_OUT24_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R162_FTDM_OUT24_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R162_FTDM_OUT24_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R162_FTDM_OUT24_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT25 (IC 1) */
#define DSP1_R163_FTDM_OUT25_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R163_FTDM_OUT25_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R163_FTDM_OUT25_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R163_FTDM_OUT25_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R163_FTDM_OUT25_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R163_FTDM_OUT25_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R163_FTDM_OUT25_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R163_FTDM_OUT25_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R163_FTDM_OUT25_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R163_FTDM_OUT25_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R163_FTDM_OUT25_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R163_FTDM_OUT25_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R163_FTDM_OUT25_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R163_FTDM_OUT25_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R163_FTDM_OUT25_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT26 (IC 1) */
#define DSP1_R164_FTDM_OUT26_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R164_FTDM_OUT26_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R164_FTDM_OUT26_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R164_FTDM_OUT26_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R164_FTDM_OUT26_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R164_FTDM_OUT26_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R164_FTDM_OUT26_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R164_FTDM_OUT26_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R164_FTDM_OUT26_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R164_FTDM_OUT26_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R164_FTDM_OUT26_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R164_FTDM_OUT26_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R164_FTDM_OUT26_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R164_FTDM_OUT26_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R164_FTDM_OUT26_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT27 (IC 1) */
#define DSP1_R165_FTDM_OUT27_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R165_FTDM_OUT27_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R165_FTDM_OUT27_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R165_FTDM_OUT27_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R165_FTDM_OUT27_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R165_FTDM_OUT27_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R165_FTDM_OUT27_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R165_FTDM_OUT27_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R165_FTDM_OUT27_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R165_FTDM_OUT27_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R165_FTDM_OUT27_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R165_FTDM_OUT27_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R165_FTDM_OUT27_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R165_FTDM_OUT27_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R165_FTDM_OUT27_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT28 (IC 1) */
#define DSP1_R166_FTDM_OUT28_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R166_FTDM_OUT28_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R166_FTDM_OUT28_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R166_FTDM_OUT28_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R166_FTDM_OUT28_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R166_FTDM_OUT28_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R166_FTDM_OUT28_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R166_FTDM_OUT28_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R166_FTDM_OUT28_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R166_FTDM_OUT28_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R166_FTDM_OUT28_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R166_FTDM_OUT28_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R166_FTDM_OUT28_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R166_FTDM_OUT28_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R166_FTDM_OUT28_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT29 (IC 1) */
#define DSP1_R167_FTDM_OUT29_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R167_FTDM_OUT29_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R167_FTDM_OUT29_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R167_FTDM_OUT29_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R167_FTDM_OUT29_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R167_FTDM_OUT29_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R167_FTDM_OUT29_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R167_FTDM_OUT29_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R167_FTDM_OUT29_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R167_FTDM_OUT29_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R167_FTDM_OUT29_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R167_FTDM_OUT29_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R167_FTDM_OUT29_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R167_FTDM_OUT29_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R167_FTDM_OUT29_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT30 (IC 1) */
#define DSP1_R168_FTDM_OUT30_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R168_FTDM_OUT30_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R168_FTDM_OUT30_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R168_FTDM_OUT30_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R168_FTDM_OUT30_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R168_FTDM_OUT30_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R168_FTDM_OUT30_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R168_FTDM_OUT30_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R168_FTDM_OUT30_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R168_FTDM_OUT30_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R168_FTDM_OUT30_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R168_FTDM_OUT30_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R168_FTDM_OUT30_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R168_FTDM_OUT30_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R168_FTDM_OUT30_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT31 (IC 1) */
#define DSP1_R169_FTDM_OUT31_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R169_FTDM_OUT31_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R169_FTDM_OUT31_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R169_FTDM_OUT31_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R169_FTDM_OUT31_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R169_FTDM_OUT31_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R169_FTDM_OUT31_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R169_FTDM_OUT31_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R169_FTDM_OUT31_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R169_FTDM_OUT31_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R169_FTDM_OUT31_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R169_FTDM_OUT31_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R169_FTDM_OUT31_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R169_FTDM_OUT31_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R169_FTDM_OUT31_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT32 (IC 1) */
#define DSP1_R170_FTDM_OUT32_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R170_FTDM_OUT32_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R170_FTDM_OUT32_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R170_FTDM_OUT32_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R170_FTDM_OUT32_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R170_FTDM_OUT32_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R170_FTDM_OUT32_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R170_FTDM_OUT32_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R170_FTDM_OUT32_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R170_FTDM_OUT32_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R170_FTDM_OUT32_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R170_FTDM_OUT32_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R170_FTDM_OUT32_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R170_FTDM_OUT32_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R170_FTDM_OUT32_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT33 (IC 1) */
#define DSP1_R171_FTDM_OUT33_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R171_FTDM_OUT33_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R171_FTDM_OUT33_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R171_FTDM_OUT33_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R171_FTDM_OUT33_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R171_FTDM_OUT33_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R171_FTDM_OUT33_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R171_FTDM_OUT33_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R171_FTDM_OUT33_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R171_FTDM_OUT33_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R171_FTDM_OUT33_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R171_FTDM_OUT33_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R171_FTDM_OUT33_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R171_FTDM_OUT33_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R171_FTDM_OUT33_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT34 (IC 1) */
#define DSP1_R172_FTDM_OUT34_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R172_FTDM_OUT34_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R172_FTDM_OUT34_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R172_FTDM_OUT34_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R172_FTDM_OUT34_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R172_FTDM_OUT34_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R172_FTDM_OUT34_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R172_FTDM_OUT34_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R172_FTDM_OUT34_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R172_FTDM_OUT34_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R172_FTDM_OUT34_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R172_FTDM_OUT34_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R172_FTDM_OUT34_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R172_FTDM_OUT34_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R172_FTDM_OUT34_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT35 (IC 1) */
#define DSP1_R173_FTDM_OUT35_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R173_FTDM_OUT35_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R173_FTDM_OUT35_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R173_FTDM_OUT35_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R173_FTDM_OUT35_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R173_FTDM_OUT35_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R173_FTDM_OUT35_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R173_FTDM_OUT35_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R173_FTDM_OUT35_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R173_FTDM_OUT35_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R173_FTDM_OUT35_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R173_FTDM_OUT35_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R173_FTDM_OUT35_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R173_FTDM_OUT35_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R173_FTDM_OUT35_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT36 (IC 1) */
#define DSP1_R174_FTDM_OUT36_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R174_FTDM_OUT36_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R174_FTDM_OUT36_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R174_FTDM_OUT36_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R174_FTDM_OUT36_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R174_FTDM_OUT36_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R174_FTDM_OUT36_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R174_FTDM_OUT36_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R174_FTDM_OUT36_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R174_FTDM_OUT36_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R174_FTDM_OUT36_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R174_FTDM_OUT36_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R174_FTDM_OUT36_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R174_FTDM_OUT36_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R174_FTDM_OUT36_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT37 (IC 1) */
#define DSP1_R175_FTDM_OUT37_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R175_FTDM_OUT37_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R175_FTDM_OUT37_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R175_FTDM_OUT37_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R175_FTDM_OUT37_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R175_FTDM_OUT37_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R175_FTDM_OUT37_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R175_FTDM_OUT37_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R175_FTDM_OUT37_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R175_FTDM_OUT37_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R175_FTDM_OUT37_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R175_FTDM_OUT37_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R175_FTDM_OUT37_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R175_FTDM_OUT37_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R175_FTDM_OUT37_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT38 (IC 1) */
#define DSP1_R176_FTDM_OUT38_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R176_FTDM_OUT38_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R176_FTDM_OUT38_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R176_FTDM_OUT38_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R176_FTDM_OUT38_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R176_FTDM_OUT38_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R176_FTDM_OUT38_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R176_FTDM_OUT38_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R176_FTDM_OUT38_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R176_FTDM_OUT38_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R176_FTDM_OUT38_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R176_FTDM_OUT38_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R176_FTDM_OUT38_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R176_FTDM_OUT38_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R176_FTDM_OUT38_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT39 (IC 1) */
#define DSP1_R177_FTDM_OUT39_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R177_FTDM_OUT39_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R177_FTDM_OUT39_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R177_FTDM_OUT39_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R177_FTDM_OUT39_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R177_FTDM_OUT39_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R177_FTDM_OUT39_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R177_FTDM_OUT39_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R177_FTDM_OUT39_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R177_FTDM_OUT39_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R177_FTDM_OUT39_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R177_FTDM_OUT39_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R177_FTDM_OUT39_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R177_FTDM_OUT39_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R177_FTDM_OUT39_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT40 (IC 1) */
#define DSP1_R178_FTDM_OUT40_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R178_FTDM_OUT40_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R178_FTDM_OUT40_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R178_FTDM_OUT40_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R178_FTDM_OUT40_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R178_FTDM_OUT40_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R178_FTDM_OUT40_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R178_FTDM_OUT40_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R178_FTDM_OUT40_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R178_FTDM_OUT40_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R178_FTDM_OUT40_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R178_FTDM_OUT40_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R178_FTDM_OUT40_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R178_FTDM_OUT40_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R178_FTDM_OUT40_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT41 (IC 1) */
#define DSP1_R179_FTDM_OUT41_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R179_FTDM_OUT41_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R179_FTDM_OUT41_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R179_FTDM_OUT41_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R179_FTDM_OUT41_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R179_FTDM_OUT41_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R179_FTDM_OUT41_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R179_FTDM_OUT41_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R179_FTDM_OUT41_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R179_FTDM_OUT41_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R179_FTDM_OUT41_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R179_FTDM_OUT41_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R179_FTDM_OUT41_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R179_FTDM_OUT41_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R179_FTDM_OUT41_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT42 (IC 1) */
#define DSP1_R180_FTDM_OUT42_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R180_FTDM_OUT42_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R180_FTDM_OUT42_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R180_FTDM_OUT42_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R180_FTDM_OUT42_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R180_FTDM_OUT42_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R180_FTDM_OUT42_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R180_FTDM_OUT42_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R180_FTDM_OUT42_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R180_FTDM_OUT42_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R180_FTDM_OUT42_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R180_FTDM_OUT42_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R180_FTDM_OUT42_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R180_FTDM_OUT42_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R180_FTDM_OUT42_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT43 (IC 1) */
#define DSP1_R181_FTDM_OUT43_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R181_FTDM_OUT43_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R181_FTDM_OUT43_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R181_FTDM_OUT43_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R181_FTDM_OUT43_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R181_FTDM_OUT43_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R181_FTDM_OUT43_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R181_FTDM_OUT43_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R181_FTDM_OUT43_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R181_FTDM_OUT43_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R181_FTDM_OUT43_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R181_FTDM_OUT43_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R181_FTDM_OUT43_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R181_FTDM_OUT43_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R181_FTDM_OUT43_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT44 (IC 1) */
#define DSP1_R182_FTDM_OUT44_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R182_FTDM_OUT44_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R182_FTDM_OUT44_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R182_FTDM_OUT44_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R182_FTDM_OUT44_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R182_FTDM_OUT44_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R182_FTDM_OUT44_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R182_FTDM_OUT44_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R182_FTDM_OUT44_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R182_FTDM_OUT44_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R182_FTDM_OUT44_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R182_FTDM_OUT44_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R182_FTDM_OUT44_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R182_FTDM_OUT44_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R182_FTDM_OUT44_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT45 (IC 1) */
#define DSP1_R183_FTDM_OUT45_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R183_FTDM_OUT45_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R183_FTDM_OUT45_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R183_FTDM_OUT45_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R183_FTDM_OUT45_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R183_FTDM_OUT45_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R183_FTDM_OUT45_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R183_FTDM_OUT45_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R183_FTDM_OUT45_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R183_FTDM_OUT45_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R183_FTDM_OUT45_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R183_FTDM_OUT45_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R183_FTDM_OUT45_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R183_FTDM_OUT45_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R183_FTDM_OUT45_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT46 (IC 1) */
#define DSP1_R184_FTDM_OUT46_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R184_FTDM_OUT46_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R184_FTDM_OUT46_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R184_FTDM_OUT46_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R184_FTDM_OUT46_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R184_FTDM_OUT46_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R184_FTDM_OUT46_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R184_FTDM_OUT46_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R184_FTDM_OUT46_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R184_FTDM_OUT46_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R184_FTDM_OUT46_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R184_FTDM_OUT46_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R184_FTDM_OUT46_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R184_FTDM_OUT46_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R184_FTDM_OUT46_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT47 (IC 1) */
#define DSP1_R185_FTDM_OUT47_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R185_FTDM_OUT47_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R185_FTDM_OUT47_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R185_FTDM_OUT47_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R185_FTDM_OUT47_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R185_FTDM_OUT47_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R185_FTDM_OUT47_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R185_FTDM_OUT47_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R185_FTDM_OUT47_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R185_FTDM_OUT47_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R185_FTDM_OUT47_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R185_FTDM_OUT47_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R185_FTDM_OUT47_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R185_FTDM_OUT47_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R185_FTDM_OUT47_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT48 (IC 1) */
#define DSP1_R186_FTDM_OUT48_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R186_FTDM_OUT48_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R186_FTDM_OUT48_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R186_FTDM_OUT48_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R186_FTDM_OUT48_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R186_FTDM_OUT48_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R186_FTDM_OUT48_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R186_FTDM_OUT48_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R186_FTDM_OUT48_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R186_FTDM_OUT48_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R186_FTDM_OUT48_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R186_FTDM_OUT48_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R186_FTDM_OUT48_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R186_FTDM_OUT48_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R186_FTDM_OUT48_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT49 (IC 1) */
#define DSP1_R187_FTDM_OUT49_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R187_FTDM_OUT49_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R187_FTDM_OUT49_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R187_FTDM_OUT49_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R187_FTDM_OUT49_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R187_FTDM_OUT49_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R187_FTDM_OUT49_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R187_FTDM_OUT49_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R187_FTDM_OUT49_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R187_FTDM_OUT49_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R187_FTDM_OUT49_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R187_FTDM_OUT49_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R187_FTDM_OUT49_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R187_FTDM_OUT49_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R187_FTDM_OUT49_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT50 (IC 1) */
#define DSP1_R188_FTDM_OUT50_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R188_FTDM_OUT50_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R188_FTDM_OUT50_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R188_FTDM_OUT50_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R188_FTDM_OUT50_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R188_FTDM_OUT50_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R188_FTDM_OUT50_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R188_FTDM_OUT50_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R188_FTDM_OUT50_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R188_FTDM_OUT50_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R188_FTDM_OUT50_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R188_FTDM_OUT50_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R188_FTDM_OUT50_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R188_FTDM_OUT50_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R188_FTDM_OUT50_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT51 (IC 1) */
#define DSP1_R189_FTDM_OUT51_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R189_FTDM_OUT51_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R189_FTDM_OUT51_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R189_FTDM_OUT51_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R189_FTDM_OUT51_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R189_FTDM_OUT51_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R189_FTDM_OUT51_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R189_FTDM_OUT51_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R189_FTDM_OUT51_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R189_FTDM_OUT51_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R189_FTDM_OUT51_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R189_FTDM_OUT51_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R189_FTDM_OUT51_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R189_FTDM_OUT51_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R189_FTDM_OUT51_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT52 (IC 1) */
#define DSP1_R190_FTDM_OUT52_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R190_FTDM_OUT52_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R190_FTDM_OUT52_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R190_FTDM_OUT52_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R190_FTDM_OUT52_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R190_FTDM_OUT52_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R190_FTDM_OUT52_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R190_FTDM_OUT52_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R190_FTDM_OUT52_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R190_FTDM_OUT52_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R190_FTDM_OUT52_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R190_FTDM_OUT52_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R190_FTDM_OUT52_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R190_FTDM_OUT52_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R190_FTDM_OUT52_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT53 (IC 1) */
#define DSP1_R191_FTDM_OUT53_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R191_FTDM_OUT53_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R191_FTDM_OUT53_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R191_FTDM_OUT53_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R191_FTDM_OUT53_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R191_FTDM_OUT53_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R191_FTDM_OUT53_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R191_FTDM_OUT53_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R191_FTDM_OUT53_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R191_FTDM_OUT53_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R191_FTDM_OUT53_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R191_FTDM_OUT53_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R191_FTDM_OUT53_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R191_FTDM_OUT53_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R191_FTDM_OUT53_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT54 (IC 1) */
#define DSP1_R192_FTDM_OUT54_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R192_FTDM_OUT54_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R192_FTDM_OUT54_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R192_FTDM_OUT54_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R192_FTDM_OUT54_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R192_FTDM_OUT54_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R192_FTDM_OUT54_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R192_FTDM_OUT54_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R192_FTDM_OUT54_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R192_FTDM_OUT54_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R192_FTDM_OUT54_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R192_FTDM_OUT54_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R192_FTDM_OUT54_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R192_FTDM_OUT54_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R192_FTDM_OUT54_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT55 (IC 1) */
#define DSP1_R193_FTDM_OUT55_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R193_FTDM_OUT55_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R193_FTDM_OUT55_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R193_FTDM_OUT55_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R193_FTDM_OUT55_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R193_FTDM_OUT55_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R193_FTDM_OUT55_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R193_FTDM_OUT55_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R193_FTDM_OUT55_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R193_FTDM_OUT55_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R193_FTDM_OUT55_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R193_FTDM_OUT55_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R193_FTDM_OUT55_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R193_FTDM_OUT55_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R193_FTDM_OUT55_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT56 (IC 1) */
#define DSP1_R194_FTDM_OUT56_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R194_FTDM_OUT56_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R194_FTDM_OUT56_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R194_FTDM_OUT56_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R194_FTDM_OUT56_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R194_FTDM_OUT56_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R194_FTDM_OUT56_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R194_FTDM_OUT56_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R194_FTDM_OUT56_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R194_FTDM_OUT56_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R194_FTDM_OUT56_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R194_FTDM_OUT56_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R194_FTDM_OUT56_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R194_FTDM_OUT56_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R194_FTDM_OUT56_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT57 (IC 1) */
#define DSP1_R195_FTDM_OUT57_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R195_FTDM_OUT57_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R195_FTDM_OUT57_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R195_FTDM_OUT57_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R195_FTDM_OUT57_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R195_FTDM_OUT57_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R195_FTDM_OUT57_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R195_FTDM_OUT57_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R195_FTDM_OUT57_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R195_FTDM_OUT57_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R195_FTDM_OUT57_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R195_FTDM_OUT57_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R195_FTDM_OUT57_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R195_FTDM_OUT57_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R195_FTDM_OUT57_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT58 (IC 1) */
#define DSP1_R196_FTDM_OUT58_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R196_FTDM_OUT58_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R196_FTDM_OUT58_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R196_FTDM_OUT58_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R196_FTDM_OUT58_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R196_FTDM_OUT58_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R196_FTDM_OUT58_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R196_FTDM_OUT58_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R196_FTDM_OUT58_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R196_FTDM_OUT58_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R196_FTDM_OUT58_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R196_FTDM_OUT58_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R196_FTDM_OUT58_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R196_FTDM_OUT58_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R196_FTDM_OUT58_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT59 (IC 1) */
#define DSP1_R197_FTDM_OUT59_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R197_FTDM_OUT59_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R197_FTDM_OUT59_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R197_FTDM_OUT59_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R197_FTDM_OUT59_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R197_FTDM_OUT59_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R197_FTDM_OUT59_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R197_FTDM_OUT59_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R197_FTDM_OUT59_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R197_FTDM_OUT59_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R197_FTDM_OUT59_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R197_FTDM_OUT59_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R197_FTDM_OUT59_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R197_FTDM_OUT59_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R197_FTDM_OUT59_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT60 (IC 1) */
#define DSP1_R198_FTDM_OUT60_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R198_FTDM_OUT60_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R198_FTDM_OUT60_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R198_FTDM_OUT60_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R198_FTDM_OUT60_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R198_FTDM_OUT60_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R198_FTDM_OUT60_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R198_FTDM_OUT60_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R198_FTDM_OUT60_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R198_FTDM_OUT60_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R198_FTDM_OUT60_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R198_FTDM_OUT60_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R198_FTDM_OUT60_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R198_FTDM_OUT60_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R198_FTDM_OUT60_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT61 (IC 1) */
#define DSP1_R199_FTDM_OUT61_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R199_FTDM_OUT61_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R199_FTDM_OUT61_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R199_FTDM_OUT61_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R199_FTDM_OUT61_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R199_FTDM_OUT61_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R199_FTDM_OUT61_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R199_FTDM_OUT61_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R199_FTDM_OUT61_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R199_FTDM_OUT61_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R199_FTDM_OUT61_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R199_FTDM_OUT61_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R199_FTDM_OUT61_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R199_FTDM_OUT61_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R199_FTDM_OUT61_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT62 (IC 1) */
#define DSP1_R200_FTDM_OUT62_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R200_FTDM_OUT62_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R200_FTDM_OUT62_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R200_FTDM_OUT62_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R200_FTDM_OUT62_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R200_FTDM_OUT62_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R200_FTDM_OUT62_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R200_FTDM_OUT62_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R200_FTDM_OUT62_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R200_FTDM_OUT62_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R200_FTDM_OUT62_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R200_FTDM_OUT62_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R200_FTDM_OUT62_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R200_FTDM_OUT62_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R200_FTDM_OUT62_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* FTDM_OUT63 (IC 1) */
#define DSP1_R201_FTDM_OUT63_BYTE_OUT_POS_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R201_FTDM_OUT63_CHANNEL_OUT_POS_IC_1      0x0    /* 000b	[4:2] */
#define DSP1_R201_FTDM_OUT63_SERIAL_OUT_SEL_IC_1       0x0    /* 0b	[5] */
#define DSP1_R201_FTDM_OUT63_REVERSE_OUT_BYTE_IC_1     0x0    /* 0b	[6] */
#define DSP1_R201_FTDM_OUT63_SLOT_ENABLE_OUT_IC_1      0x0    /* 0b	[7] */
#define DSP1_R201_FTDM_OUT63_BYTE_OUT_POS_IC_1_MASK    0x3
#define DSP1_R201_FTDM_OUT63_BYTE_OUT_POS_IC_1_SHIFT   0
#define DSP1_R201_FTDM_OUT63_CHANNEL_OUT_POS_IC_1_MASK 0x1C
#define DSP1_R201_FTDM_OUT63_CHANNEL_OUT_POS_IC_1_SHIFT 2
#define DSP1_R201_FTDM_OUT63_SERIAL_OUT_SEL_IC_1_MASK  0x20
#define DSP1_R201_FTDM_OUT63_SERIAL_OUT_SEL_IC_1_SHIFT 5
#define DSP1_R201_FTDM_OUT63_REVERSE_OUT_BYTE_IC_1_MASK 0x40
#define DSP1_R201_FTDM_OUT63_REVERSE_OUT_BYTE_IC_1_SHIFT 6
#define DSP1_R201_FTDM_OUT63_SLOT_ENABLE_OUT_IC_1_MASK 0x80
#define DSP1_R201_FTDM_OUT63_SLOT_ENABLE_OUT_IC_1_SHIFT 7

/* HIBERNATE (IC 1) */
#define DSP1_R202_HIBERNATE_HIBERNATE_IC_1             0x0    /* 0b	[0] */
#define DSP1_R202_HIBERNATE_HIBERNATE_IC_1_MASK        0x1
#define DSP1_R202_HIBERNATE_HIBERNATE_IC_1_SHIFT       0

/* START_PULSE (IC 1) */
#define DSP1_R203_START_PULSE_START_PULSE_IC_1         0x2    /* 00010b	[4:0] */
#define DSP1_R203_START_PULSE_START_PULSE_IC_1_MASK    0x1F
#define DSP1_R203_START_PULSE_START_PULSE_IC_1_SHIFT   0

/* START_CORE (IC 1) */
#define DSP1_R204_START_CORE_START_CORE_IC_1           0x1    /* 1b	[0] */
#define DSP1_R204_START_CORE_START_CORE_IC_1_MASK      0x1
#define DSP1_R204_START_CORE_START_CORE_IC_1_SHIFT     0

/* KILL_CORE (IC 1) */
#define DSP1_R205_KILL_CORE_KILL_CORE_IC_1             0x0    /* 0b	[0] */
#define DSP1_R205_KILL_CORE_KILL_CORE_IC_1_MASK        0x1
#define DSP1_R205_KILL_CORE_KILL_CORE_IC_1_SHIFT       0

/* START_ADDRESS (IC 1) */
#define DSP1_R206_START_ADDRESS_START_ADDRESS_IC_1     0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R206_START_ADDRESS_START_ADDRESS_IC_1_MASK 0xFFFF
#define DSP1_R206_START_ADDRESS_START_ADDRESS_IC_1_SHIFT 0

/* CORE_STATUS (IC 1) */
#define DSP1_R207_CORE_STATUS_CORE_STATUS_IC_1         0x0    /* 000b	[2:0] */
#define DSP1_R207_CORE_STATUS_CORE_STATUS_IC_1_MASK    0x7
#define DSP1_R207_CORE_STATUS_CORE_STATUS_IC_1_SHIFT   0

/* PANIC_CLEAR (IC 1) */
#define DSP1_R208_PANIC_CLEAR_PANIC_CLEAR_IC_1         0x0    /* 0b	[0] */
#define DSP1_R208_PANIC_CLEAR_PANIC_CLEAR_IC_1_MASK    0x1
#define DSP1_R208_PANIC_CLEAR_PANIC_CLEAR_IC_1_SHIFT   0

/* PANIC_PARITY_MASK (IC 1) */
#define DSP1_R209_PANIC_PARITY_MASK_ASRC0_MASK_IC_1    0x1    /* 1b	[0] */
#define DSP1_R209_PANIC_PARITY_MASK_ASRC1_MASK_IC_1    0x1    /* 1b	[1] */
#define DSP1_R209_PANIC_PARITY_MASK_PM0_MASK_IC_1      0x0    /* 0b	[2] */
#define DSP1_R209_PANIC_PARITY_MASK_PM1_MASK_IC_1      0x0    /* 0b	[3] */
#define DSP1_R209_PANIC_PARITY_MASK_DM0_BANK0_MASK_IC_1 0x0   /* 0b	[4] */
#define DSP1_R209_PANIC_PARITY_MASK_DM0_BANK1_MASK_IC_1 0x0   /* 0b	[5] */
#define DSP1_R209_PANIC_PARITY_MASK_DM0_BANK2_MASK_IC_1 0x0   /* 0b	[6] */
#define DSP1_R209_PANIC_PARITY_MASK_DM0_BANK3_MASK_IC_1 0x0   /* 0b	[7] */
#define DSP1_R209_PANIC_PARITY_MASK_DM1_BANK0_MASK_IC_1 0x0   /* 0b	[8] */
#define DSP1_R209_PANIC_PARITY_MASK_DM1_BANK1_MASK_IC_1 0x0   /* 0b	[9] */
#define DSP1_R209_PANIC_PARITY_MASK_DM1_BANK2_MASK_IC_1 0x0   /* 0b	[10] */
#define DSP1_R209_PANIC_PARITY_MASK_DM1_BANK3_MASK_IC_1 0x0   /* 0b	[11] */
#define DSP1_R209_PANIC_PARITY_MASK_ASRC0_MASK_IC_1_MASK 0x1
#define DSP1_R209_PANIC_PARITY_MASK_ASRC0_MASK_IC_1_SHIFT 0
#define DSP1_R209_PANIC_PARITY_MASK_ASRC1_MASK_IC_1_MASK 0x2
#define DSP1_R209_PANIC_PARITY_MASK_ASRC1_MASK_IC_1_SHIFT 1
#define DSP1_R209_PANIC_PARITY_MASK_PM0_MASK_IC_1_MASK 0x4
#define DSP1_R209_PANIC_PARITY_MASK_PM0_MASK_IC_1_SHIFT 2
#define DSP1_R209_PANIC_PARITY_MASK_PM1_MASK_IC_1_MASK 0x8
#define DSP1_R209_PANIC_PARITY_MASK_PM1_MASK_IC_1_SHIFT 3
#define DSP1_R209_PANIC_PARITY_MASK_DM0_BANK0_MASK_IC_1_MASK 0x10
#define DSP1_R209_PANIC_PARITY_MASK_DM0_BANK0_MASK_IC_1_SHIFT 4
#define DSP1_R209_PANIC_PARITY_MASK_DM0_BANK1_MASK_IC_1_MASK 0x20
#define DSP1_R209_PANIC_PARITY_MASK_DM0_BANK1_MASK_IC_1_SHIFT 5
#define DSP1_R209_PANIC_PARITY_MASK_DM0_BANK2_MASK_IC_1_MASK 0x40
#define DSP1_R209_PANIC_PARITY_MASK_DM0_BANK2_MASK_IC_1_SHIFT 6
#define DSP1_R209_PANIC_PARITY_MASK_DM0_BANK3_MASK_IC_1_MASK 0x80
#define DSP1_R209_PANIC_PARITY_MASK_DM0_BANK3_MASK_IC_1_SHIFT 7
#define DSP1_R209_PANIC_PARITY_MASK_DM1_BANK0_MASK_IC_1_MASK 0x100
#define DSP1_R209_PANIC_PARITY_MASK_DM1_BANK0_MASK_IC_1_SHIFT 8
#define DSP1_R209_PANIC_PARITY_MASK_DM1_BANK1_MASK_IC_1_MASK 0x200
#define DSP1_R209_PANIC_PARITY_MASK_DM1_BANK1_MASK_IC_1_SHIFT 9
#define DSP1_R209_PANIC_PARITY_MASK_DM1_BANK2_MASK_IC_1_MASK 0x400
#define DSP1_R209_PANIC_PARITY_MASK_DM1_BANK2_MASK_IC_1_SHIFT 10
#define DSP1_R209_PANIC_PARITY_MASK_DM1_BANK3_MASK_IC_1_MASK 0x800
#define DSP1_R209_PANIC_PARITY_MASK_DM1_BANK3_MASK_IC_1_SHIFT 11

/* PANIC_SOFTWARE_MASK (IC 1) */
#define DSP1_R210_PANIC_SOFTWARE_MASK_PANIC_SOFTWARE_IC_1 0x0 /* 0b	[0] */
#define DSP1_R210_PANIC_SOFTWARE_MASK_PANIC_SOFTWARE_IC_1_MASK 0x1
#define DSP1_R210_PANIC_SOFTWARE_MASK_PANIC_SOFTWARE_IC_1_SHIFT 0

/* PANIC_WD_MASK (IC 1) */
#define DSP1_R211_PANIC_WD_MASK_PANIC_WD_IC_1          0x0    /* 0b	[0] */
#define DSP1_R211_PANIC_WD_MASK_PANIC_WD_IC_1_MASK     0x1
#define DSP1_R211_PANIC_WD_MASK_PANIC_WD_IC_1_SHIFT    0

/* PANIC_STACK_MASK (IC 1) */
#define DSP1_R212_PANIC_STACK_MASK_PANIC_STACK_IC_1    0x0    /* 0b	[0] */
#define DSP1_R212_PANIC_STACK_MASK_PANIC_STACK_IC_1_MASK 0x1
#define DSP1_R212_PANIC_STACK_MASK_PANIC_STACK_IC_1_SHIFT 0

/* PANIC_LOOP_MASK (IC 1) */
#define DSP1_R213_PANIC_LOOP_MASK_PANIC_LOOP_IC_1      0x0    /* 0b	[0] */
#define DSP1_R213_PANIC_LOOP_MASK_PANIC_LOOP_IC_1_MASK 0x1
#define DSP1_R213_PANIC_LOOP_MASK_PANIC_LOOP_IC_1_SHIFT 0

/* PANIC_FLAG (IC 1) */
#define DSP1_R214_PANIC_FLAG_PANIC_FLAG_IC_1           0x0    /* 0b	[0] */
#define DSP1_R214_PANIC_FLAG_PANIC_FLAG_IC_1_MASK      0x1
#define DSP1_R214_PANIC_FLAG_PANIC_FLAG_IC_1_SHIFT     0

/* PANIC_CODE (IC 1) */
#define DSP1_R215_PANIC_CODE_ERR_ASRC0_IC_1            0x0    /* 0b	[0] */
#define DSP1_R215_PANIC_CODE_ERR_ASRC1_IC_1            0x0    /* 0b	[1] */
#define DSP1_R215_PANIC_CODE_ERR_PM0_IC_1              0x0    /* 0b	[2] */
#define DSP1_R215_PANIC_CODE_ERR_PM1_IC_1              0x0    /* 0b	[3] */
#define DSP1_R215_PANIC_CODE_ERR_DM0B0_IC_1            0x0    /* 0b	[4] */
#define DSP1_R215_PANIC_CODE_ERR_DM0B1_IC_1            0x0    /* 0b	[5] */
#define DSP1_R215_PANIC_CODE_ERR_DM0B2_IC_1            0x0    /* 0b	[6] */
#define DSP1_R215_PANIC_CODE_ERR_DM0B3_IC_1            0x0    /* 0b	[7] */
#define DSP1_R215_PANIC_CODE_ERR_DM1B0_IC_1            0x0    /* 0b	[8] */
#define DSP1_R215_PANIC_CODE_ERR_DM1B1_IC_1            0x0    /* 0b	[9] */
#define DSP1_R215_PANIC_CODE_ERR_DM1B2_IC_1            0x0    /* 0b	[10] */
#define DSP1_R215_PANIC_CODE_ERR_DM1B3_IC_1            0x0    /* 0b	[11] */
#define DSP1_R215_PANIC_CODE_ERR_WATCHDOG_IC_1         0x0    /* 0b	[12] */
#define DSP1_R215_PANIC_CODE_ERR_STACK_IC_1            0x0    /* 0b	[13] */
#define DSP1_R215_PANIC_CODE_ERR_LOOP_IC_1             0x0    /* 0b	[14] */
#define DSP1_R215_PANIC_CODE_ERR_SOFT_IC_1             0x0    /* 0b	[15] */
#define DSP1_R215_PANIC_CODE_ERR_ASRC0_IC_1_MASK       0x1
#define DSP1_R215_PANIC_CODE_ERR_ASRC0_IC_1_SHIFT      0
#define DSP1_R215_PANIC_CODE_ERR_ASRC1_IC_1_MASK       0x2
#define DSP1_R215_PANIC_CODE_ERR_ASRC1_IC_1_SHIFT      1
#define DSP1_R215_PANIC_CODE_ERR_PM0_IC_1_MASK         0x4
#define DSP1_R215_PANIC_CODE_ERR_PM0_IC_1_SHIFT        2
#define DSP1_R215_PANIC_CODE_ERR_PM1_IC_1_MASK         0x8
#define DSP1_R215_PANIC_CODE_ERR_PM1_IC_1_SHIFT        3
#define DSP1_R215_PANIC_CODE_ERR_DM0B0_IC_1_MASK       0x10
#define DSP1_R215_PANIC_CODE_ERR_DM0B0_IC_1_SHIFT      4
#define DSP1_R215_PANIC_CODE_ERR_DM0B1_IC_1_MASK       0x20
#define DSP1_R215_PANIC_CODE_ERR_DM0B1_IC_1_SHIFT      5
#define DSP1_R215_PANIC_CODE_ERR_DM0B2_IC_1_MASK       0x40
#define DSP1_R215_PANIC_CODE_ERR_DM0B2_IC_1_SHIFT      6
#define DSP1_R215_PANIC_CODE_ERR_DM0B3_IC_1_MASK       0x80
#define DSP1_R215_PANIC_CODE_ERR_DM0B3_IC_1_SHIFT      7
#define DSP1_R215_PANIC_CODE_ERR_DM1B0_IC_1_MASK       0x100
#define DSP1_R215_PANIC_CODE_ERR_DM1B0_IC_1_SHIFT      8
#define DSP1_R215_PANIC_CODE_ERR_DM1B1_IC_1_MASK       0x200
#define DSP1_R215_PANIC_CODE_ERR_DM1B1_IC_1_SHIFT      9
#define DSP1_R215_PANIC_CODE_ERR_DM1B2_IC_1_MASK       0x400
#define DSP1_R215_PANIC_CODE_ERR_DM1B2_IC_1_SHIFT      10
#define DSP1_R215_PANIC_CODE_ERR_DM1B3_IC_1_MASK       0x800
#define DSP1_R215_PANIC_CODE_ERR_DM1B3_IC_1_SHIFT      11
#define DSP1_R215_PANIC_CODE_ERR_WATCHDOG_IC_1_MASK    0x1000
#define DSP1_R215_PANIC_CODE_ERR_WATCHDOG_IC_1_SHIFT   12
#define DSP1_R215_PANIC_CODE_ERR_STACK_IC_1_MASK       0x2000
#define DSP1_R215_PANIC_CODE_ERR_STACK_IC_1_SHIFT      13
#define DSP1_R215_PANIC_CODE_ERR_LOOP_IC_1_MASK        0x4000
#define DSP1_R215_PANIC_CODE_ERR_LOOP_IC_1_SHIFT       14
#define DSP1_R215_PANIC_CODE_ERR_SOFT_IC_1_MASK        0x8000
#define DSP1_R215_PANIC_CODE_ERR_SOFT_IC_1_SHIFT       15

/* EXECUTE_COUNT (IC 1) */
#define DSP1_R216_EXECUTE_COUNT_EXECUTE_COUNT_IC_1     0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R216_EXECUTE_COUNT_EXECUTE_COUNT_IC_1_MASK 0xFFFF
#define DSP1_R216_EXECUTE_COUNT_EXECUTE_COUNT_IC_1_SHIFT 0

/* WATCHDOG_MAXCOUNT (IC 1) */
#define DSP1_R217_WATCHDOG_MAXCOUNT_WD_MAXCOUNT_IC_1   0x000  /* 0000000000000b	[12:0] */
#define DSP1_R217_WATCHDOG_MAXCOUNT_WD_MAXCOUNT_IC_1_MASK 0x1FFF
#define DSP1_R217_WATCHDOG_MAXCOUNT_WD_MAXCOUNT_IC_1_SHIFT 0

/* WATCHDOG_PRESCALE (IC 1) */
#define DSP1_R218_WATCHDOG_PRESCALE_WD_PRESCALE_IC_1   0x0    /* 0000b	[3:0] */
#define DSP1_R218_WATCHDOG_PRESCALE_WD_PRESCALE_IC_1_MASK 0xF
#define DSP1_R218_WATCHDOG_PRESCALE_WD_PRESCALE_IC_1_SHIFT 0

/* BLOCKINT_EN (IC 1) */
#define DSP1_R219_BLOCKINT_EN_BLOCKINT_EN_IC_1         0x0    /* 0b	[0] */
#define DSP1_R219_BLOCKINT_EN_BLOCKINT_EN_IC_1_MASK    0x1
#define DSP1_R219_BLOCKINT_EN_BLOCKINT_EN_IC_1_SHIFT   0

/* BLOCKINT_VALUE (IC 1) */
#define DSP1_R220_BLOCKINT_VALUE_BLOCKINT_VALUE_IC_1   0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R220_BLOCKINT_VALUE_BLOCKINT_VALUE_IC_1_MASK 0xFFFF
#define DSP1_R220_BLOCKINT_VALUE_BLOCKINT_VALUE_IC_1_SHIFT 0

/* PROG_CNTR0 (IC 1) */
#define DSP1_R221_PROG_CNTR0_PROG_CNTR_MSB_IC_1        0x00   /* 00000000b	[7:0] */
#define DSP1_R221_PROG_CNTR0_PROG_CNTR_MSB_IC_1_MASK   0xFF
#define DSP1_R221_PROG_CNTR0_PROG_CNTR_MSB_IC_1_SHIFT  0

/* PROG_CNTR1 (IC 1) */
#define DSP1_R222_PROG_CNTR1_PROG_CNTR_LSB_IC_1        0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R222_PROG_CNTR1_PROG_CNTR_LSB_IC_1_MASK   0xFFFF
#define DSP1_R222_PROG_CNTR1_PROG_CNTR_LSB_IC_1_SHIFT  0

/* PROG_CNTR_CLEAR (IC 1) */
#define DSP1_R223_PROG_CNTR_CLEAR_PROG_CNTR_CLEAR_IC_1 0x0    /* 0b	[0] */
#define DSP1_R223_PROG_CNTR_CLEAR_PROG_CNTR_CLEAR_IC_1_MASK 0x1
#define DSP1_R223_PROG_CNTR_CLEAR_PROG_CNTR_CLEAR_IC_1_SHIFT 0

/* PROG_CNTR_LENGTH0 (IC 1) */
#define DSP1_R224_PROG_CNTR_LENGTH0_PROG_LENGTH_MSB_IC_1 0x00 /* 00000000b	[7:0] */
#define DSP1_R224_PROG_CNTR_LENGTH0_PROG_LENGTH_MSB_IC_1_MASK 0xFF
#define DSP1_R224_PROG_CNTR_LENGTH0_PROG_LENGTH_MSB_IC_1_SHIFT 0

/* PROG_CNTR_LENGTH1 (IC 1) */
#define DSP1_R225_PROG_CNTR_LENGTH1_PROG_LENGTH_LSB_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R225_PROG_CNTR_LENGTH1_PROG_LENGTH_LSB_IC_1_MASK 0xFFFF
#define DSP1_R225_PROG_CNTR_LENGTH1_PROG_LENGTH_LSB_IC_1_SHIFT 0

/* PROG_CNTR_MAXLENGTH0 (IC 1) */
#define DSP1_R226_PROG_CNTR_MAXLENGTH0_PROG_MAXLENGTH_MSB_IC_1 0x00 /* 00000000b	[7:0] */
#define DSP1_R226_PROG_CNTR_MAXLENGTH0_PROG_MAXLENGTH_MSB_IC_1_MASK 0xFF
#define DSP1_R226_PROG_CNTR_MAXLENGTH0_PROG_MAXLENGTH_MSB_IC_1_SHIFT 0

/* PROG_CNTR_MAXLENGTH1 (IC 1) */
#define DSP1_R227_PROG_CNTR_MAXLENGTH1_PROG_MAXLENGTH_LSB_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R227_PROG_CNTR_MAXLENGTH1_PROG_MAXLENGTH_LSB_IC_1_MASK 0xFFFF
#define DSP1_R227_PROG_CNTR_MAXLENGTH1_PROG_MAXLENGTH_LSB_IC_1_SHIFT 0

/* PANIC_PARITY_MASK1 (IC 1) */
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK0_SUBBANK0_MASK_IC_1 0x0 /* 0b	[0] */
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK0_SUBBANK1_MASK_IC_1 0x0 /* 0b	[1] */
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK0_SUBBANK2_MASK_IC_1 0x0 /* 0b	[2] */
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK0_SUBBANK3_MASK_IC_1 0x0 /* 0b	[3] */
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK0_SUBBANK4_MASK_IC_1 0x0 /* 0b	[4] */
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK1_SUBBANK0_MASK_IC_1 0x0 /* 0b	[8] */
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK1_SUBBANK1_MASK_IC_1 0x0 /* 0b	[9] */
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK1_SUBBANK2_MASK_IC_1 0x0 /* 0b	[10] */
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK1_SUBBANK3_MASK_IC_1 0x0 /* 0b	[11] */
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK1_SUBBANK4_MASK_IC_1 0x0 /* 0b	[12] */
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK0_SUBBANK0_MASK_IC_1_MASK 0x1
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK0_SUBBANK0_MASK_IC_1_SHIFT 0
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK0_SUBBANK1_MASK_IC_1_MASK 0x2
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK0_SUBBANK1_MASK_IC_1_SHIFT 1
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK0_SUBBANK2_MASK_IC_1_MASK 0x4
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK0_SUBBANK2_MASK_IC_1_SHIFT 2
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK0_SUBBANK3_MASK_IC_1_MASK 0x8
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK0_SUBBANK3_MASK_IC_1_SHIFT 3
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK0_SUBBANK4_MASK_IC_1_MASK 0x10
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK0_SUBBANK4_MASK_IC_1_SHIFT 4
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK1_SUBBANK0_MASK_IC_1_MASK 0x100
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK1_SUBBANK0_MASK_IC_1_SHIFT 8
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK1_SUBBANK1_MASK_IC_1_MASK 0x200
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK1_SUBBANK1_MASK_IC_1_SHIFT 9
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK1_SUBBANK2_MASK_IC_1_MASK 0x400
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK1_SUBBANK2_MASK_IC_1_SHIFT 10
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK1_SUBBANK3_MASK_IC_1_MASK 0x800
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK1_SUBBANK3_MASK_IC_1_SHIFT 11
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK1_SUBBANK4_MASK_IC_1_MASK 0x1000
#define DSP1_R228_PANIC_PARITY_MASK1_DM0_BANK1_SUBBANK4_MASK_IC_1_SHIFT 12

/* PANIC_PARITY_MASK2 (IC 1) */
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK2_SUBBANK0_MASK_IC_1 0x0 /* 0b	[0] */
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK2_SUBBANK1_MASK_IC_1 0x0 /* 0b	[1] */
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK2_SUBBANK2_MASK_IC_1 0x0 /* 0b	[2] */
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK2_SUBBANK3_MASK_IC_1 0x0 /* 0b	[3] */
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK2_SUBBANK4_MASK_IC_1 0x0 /* 0b	[4] */
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK3_SUBBANK0_MASK_IC_1 0x0 /* 0b	[8] */
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK3_SUBBANK1_MASK_IC_1 0x0 /* 0b	[9] */
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK3_SUBBANK2_MASK_IC_1 0x0 /* 0b	[10] */
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK3_SUBBANK3_MASK_IC_1 0x0 /* 0b	[11] */
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK3_SUBBANK4_MASK_IC_1 0x0 /* 0b	[12] */
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK2_SUBBANK0_MASK_IC_1_MASK 0x1
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK2_SUBBANK0_MASK_IC_1_SHIFT 0
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK2_SUBBANK1_MASK_IC_1_MASK 0x2
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK2_SUBBANK1_MASK_IC_1_SHIFT 1
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK2_SUBBANK2_MASK_IC_1_MASK 0x4
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK2_SUBBANK2_MASK_IC_1_SHIFT 2
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK2_SUBBANK3_MASK_IC_1_MASK 0x8
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK2_SUBBANK3_MASK_IC_1_SHIFT 3
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK2_SUBBANK4_MASK_IC_1_MASK 0x10
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK2_SUBBANK4_MASK_IC_1_SHIFT 4
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK3_SUBBANK0_MASK_IC_1_MASK 0x100
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK3_SUBBANK0_MASK_IC_1_SHIFT 8
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK3_SUBBANK1_MASK_IC_1_MASK 0x200
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK3_SUBBANK1_MASK_IC_1_SHIFT 9
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK3_SUBBANK2_MASK_IC_1_MASK 0x400
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK3_SUBBANK2_MASK_IC_1_SHIFT 10
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK3_SUBBANK3_MASK_IC_1_MASK 0x800
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK3_SUBBANK3_MASK_IC_1_SHIFT 11
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK3_SUBBANK4_MASK_IC_1_MASK 0x1000
#define DSP1_R229_PANIC_PARITY_MASK2_DM0_BANK3_SUBBANK4_MASK_IC_1_SHIFT 12

/* PANIC_PARITY_MASK3 (IC 1) */
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK0_SUBBANK0_MASK_IC_1 0x0 /* 0b	[0] */
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK0_SUBBANK1_MASK_IC_1 0x0 /* 0b	[1] */
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK0_SUBBANK2_MASK_IC_1 0x0 /* 0b	[2] */
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK0_SUBBANK3_MASK_IC_1 0x0 /* 0b	[3] */
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK0_SUBBANK4_MASK_IC_1 0x0 /* 0b	[4] */
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK1_SUBBANK0_MASK_IC_1 0x0 /* 0b	[8] */
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK1_SUBBANK1_MASK_IC_1 0x0 /* 0b	[9] */
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK1_SUBBANK2_MASK_IC_1 0x0 /* 0b	[10] */
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK1_SUBBANK3_MASK_IC_1 0x0 /* 0b	[11] */
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK1_SUBBANK4_MASK_IC_1 0x0 /* 0b	[12] */
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK0_SUBBANK0_MASK_IC_1_MASK 0x1
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK0_SUBBANK0_MASK_IC_1_SHIFT 0
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK0_SUBBANK1_MASK_IC_1_MASK 0x2
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK0_SUBBANK1_MASK_IC_1_SHIFT 1
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK0_SUBBANK2_MASK_IC_1_MASK 0x4
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK0_SUBBANK2_MASK_IC_1_SHIFT 2
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK0_SUBBANK3_MASK_IC_1_MASK 0x8
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK0_SUBBANK3_MASK_IC_1_SHIFT 3
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK0_SUBBANK4_MASK_IC_1_MASK 0x10
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK0_SUBBANK4_MASK_IC_1_SHIFT 4
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK1_SUBBANK0_MASK_IC_1_MASK 0x100
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK1_SUBBANK0_MASK_IC_1_SHIFT 8
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK1_SUBBANK1_MASK_IC_1_MASK 0x200
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK1_SUBBANK1_MASK_IC_1_SHIFT 9
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK1_SUBBANK2_MASK_IC_1_MASK 0x400
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK1_SUBBANK2_MASK_IC_1_SHIFT 10
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK1_SUBBANK3_MASK_IC_1_MASK 0x800
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK1_SUBBANK3_MASK_IC_1_SHIFT 11
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK1_SUBBANK4_MASK_IC_1_MASK 0x1000
#define DSP1_R230_PANIC_PARITY_MASK3_DM1_BANK1_SUBBANK4_MASK_IC_1_SHIFT 12

/* PANIC_PARITY_MASK4 (IC 1) */
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK2_SUBBANK0_MASK_IC_1 0x0 /* 0b	[0] */
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK2_SUBBANK1_MASK_IC_1 0x0 /* 0b	[1] */
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK2_SUBBANK2_MASK_IC_1 0x0 /* 0b	[2] */
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK2_SUBBANK3_MASK_IC_1 0x0 /* 0b	[3] */
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK2_SUBBANK4_MASK_IC_1 0x0 /* 0b	[4] */
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK3_SUBBANK0_MASK_IC_1 0x0 /* 0b	[8] */
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK3_SUBBANK1_MASK_IC_1 0x0 /* 0b	[9] */
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK3_SUBBANK2_MASK_IC_1 0x0 /* 0b	[10] */
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK3_SUBBANK3_MASK_IC_1 0x0 /* 0b	[11] */
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK3_SUBBANK4_MASK_IC_1 0x0 /* 0b	[12] */
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK2_SUBBANK0_MASK_IC_1_MASK 0x1
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK2_SUBBANK0_MASK_IC_1_SHIFT 0
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK2_SUBBANK1_MASK_IC_1_MASK 0x2
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK2_SUBBANK1_MASK_IC_1_SHIFT 1
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK2_SUBBANK2_MASK_IC_1_MASK 0x4
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK2_SUBBANK2_MASK_IC_1_SHIFT 2
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK2_SUBBANK3_MASK_IC_1_MASK 0x8
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK2_SUBBANK3_MASK_IC_1_SHIFT 3
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK2_SUBBANK4_MASK_IC_1_MASK 0x10
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK2_SUBBANK4_MASK_IC_1_SHIFT 4
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK3_SUBBANK0_MASK_IC_1_MASK 0x100
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK3_SUBBANK0_MASK_IC_1_SHIFT 8
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK3_SUBBANK1_MASK_IC_1_MASK 0x200
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK3_SUBBANK1_MASK_IC_1_SHIFT 9
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK3_SUBBANK2_MASK_IC_1_MASK 0x400
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK3_SUBBANK2_MASK_IC_1_SHIFT 10
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK3_SUBBANK3_MASK_IC_1_MASK 0x800
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK3_SUBBANK3_MASK_IC_1_SHIFT 11
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK3_SUBBANK4_MASK_IC_1_MASK 0x1000
#define DSP1_R231_PANIC_PARITY_MASK4_DM1_BANK3_SUBBANK4_MASK_IC_1_SHIFT 12

/* PANIC_PARITY_MASK5 (IC 1) */
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK0_SUBBANK0_MASK_IC_1 0x0 /* 0b	[0] */
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK0_SUBBANK1_MASK_IC_1 0x0 /* 0b	[1] */
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK0_SUBBANK2_MASK_IC_1 0x0 /* 0b	[2] */
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK0_SUBBANK3_MASK_IC_1 0x0 /* 0b	[3] */
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK0_SUBBANK4_MASK_IC_1 0x0 /* 0b	[4] */
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK0_SUBBANK5_MASK_IC_1 0x0 /* 0b	[5] */
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK1_SUBBANK0_MASK_IC_1 0x0 /* 0b	[8] */
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK1_SUBBANK1_MASK_IC_1 0x0 /* 0b	[9] */
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK1_SUBBANK2_MASK_IC_1 0x0 /* 0b	[10] */
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK1_SUBBANK3_MASK_IC_1 0x0 /* 0b	[11] */
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK1_SUBBANK4_MASK_IC_1 0x0 /* 0b	[12] */
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK1_SUBBANK5_MASK_IC_1 0x0 /* 0b	[13] */
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK0_SUBBANK0_MASK_IC_1_MASK 0x1
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK0_SUBBANK0_MASK_IC_1_SHIFT 0
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK0_SUBBANK1_MASK_IC_1_MASK 0x2
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK0_SUBBANK1_MASK_IC_1_SHIFT 1
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK0_SUBBANK2_MASK_IC_1_MASK 0x4
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK0_SUBBANK2_MASK_IC_1_SHIFT 2
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK0_SUBBANK3_MASK_IC_1_MASK 0x8
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK0_SUBBANK3_MASK_IC_1_SHIFT 3
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK0_SUBBANK4_MASK_IC_1_MASK 0x10
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK0_SUBBANK4_MASK_IC_1_SHIFT 4
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK0_SUBBANK5_MASK_IC_1_MASK 0x20
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK0_SUBBANK5_MASK_IC_1_SHIFT 5
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK1_SUBBANK0_MASK_IC_1_MASK 0x100
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK1_SUBBANK0_MASK_IC_1_SHIFT 8
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK1_SUBBANK1_MASK_IC_1_MASK 0x200
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK1_SUBBANK1_MASK_IC_1_SHIFT 9
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK1_SUBBANK2_MASK_IC_1_MASK 0x400
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK1_SUBBANK2_MASK_IC_1_SHIFT 10
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK1_SUBBANK3_MASK_IC_1_MASK 0x800
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK1_SUBBANK3_MASK_IC_1_SHIFT 11
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK1_SUBBANK4_MASK_IC_1_MASK 0x1000
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK1_SUBBANK4_MASK_IC_1_SHIFT 12
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK1_SUBBANK5_MASK_IC_1_MASK 0x2000
#define DSP1_R232_PANIC_PARITY_MASK5_PM_BANK1_SUBBANK5_MASK_IC_1_SHIFT 13

/* PANIC_CODE1 (IC 1) */
#define DSP1_R233_PANIC_CODE1_ERR_DM0B0SB0_IC_1        0x0    /* 0b	[0] */
#define DSP1_R233_PANIC_CODE1_ERR_DM0B0SB1_IC_1        0x0    /* 0b	[1] */
#define DSP1_R233_PANIC_CODE1_ERR_DM0B0SB2_IC_1        0x0    /* 0b	[2] */
#define DSP1_R233_PANIC_CODE1_ERR_DM0B0SB3_IC_1        0x0    /* 0b	[3] */
#define DSP1_R233_PANIC_CODE1_ERR_DM0B0SB4_IC_1        0x0    /* 0b	[4] */
#define DSP1_R233_PANIC_CODE1_ERR_DM0B1SB0_IC_1        0x0    /* 0b	[8] */
#define DSP1_R233_PANIC_CODE1_ERR_DM0B1SB1_IC_1        0x0    /* 0b	[9] */
#define DSP1_R233_PANIC_CODE1_ERR_DM0B1SB2_IC_1        0x0    /* 0b	[10] */
#define DSP1_R233_PANIC_CODE1_ERR_DM0B1SB3_IC_1        0x0    /* 0b	[11] */
#define DSP1_R233_PANIC_CODE1_ERR_DM0B1SB4_IC_1        0x0    /* 0b	[12] */
#define DSP1_R233_PANIC_CODE1_ERR_DM0B0SB0_IC_1_MASK   0x1
#define DSP1_R233_PANIC_CODE1_ERR_DM0B0SB0_IC_1_SHIFT  0
#define DSP1_R233_PANIC_CODE1_ERR_DM0B0SB1_IC_1_MASK   0x2
#define DSP1_R233_PANIC_CODE1_ERR_DM0B0SB1_IC_1_SHIFT  1
#define DSP1_R233_PANIC_CODE1_ERR_DM0B0SB2_IC_1_MASK   0x4
#define DSP1_R233_PANIC_CODE1_ERR_DM0B0SB2_IC_1_SHIFT  2
#define DSP1_R233_PANIC_CODE1_ERR_DM0B0SB3_IC_1_MASK   0x8
#define DSP1_R233_PANIC_CODE1_ERR_DM0B0SB3_IC_1_SHIFT  3
#define DSP1_R233_PANIC_CODE1_ERR_DM0B0SB4_IC_1_MASK   0x10
#define DSP1_R233_PANIC_CODE1_ERR_DM0B0SB4_IC_1_SHIFT  4
#define DSP1_R233_PANIC_CODE1_ERR_DM0B1SB0_IC_1_MASK   0x100
#define DSP1_R233_PANIC_CODE1_ERR_DM0B1SB0_IC_1_SHIFT  8
#define DSP1_R233_PANIC_CODE1_ERR_DM0B1SB1_IC_1_MASK   0x200
#define DSP1_R233_PANIC_CODE1_ERR_DM0B1SB1_IC_1_SHIFT  9
#define DSP1_R233_PANIC_CODE1_ERR_DM0B1SB2_IC_1_MASK   0x400
#define DSP1_R233_PANIC_CODE1_ERR_DM0B1SB2_IC_1_SHIFT  10
#define DSP1_R233_PANIC_CODE1_ERR_DM0B1SB3_IC_1_MASK   0x800
#define DSP1_R233_PANIC_CODE1_ERR_DM0B1SB3_IC_1_SHIFT  11
#define DSP1_R233_PANIC_CODE1_ERR_DM0B1SB4_IC_1_MASK   0x1000
#define DSP1_R233_PANIC_CODE1_ERR_DM0B1SB4_IC_1_SHIFT  12

/* PANIC_CODE2 (IC 1) */
#define DSP1_R234_PANIC_CODE2_ERR_DM0B2SB0_IC_1        0x0    /* 0b	[0] */
#define DSP1_R234_PANIC_CODE2_ERR_DM0B2SB1_IC_1        0x0    /* 0b	[1] */
#define DSP1_R234_PANIC_CODE2_ERR_DM0B2SB2_IC_1        0x0    /* 0b	[2] */
#define DSP1_R234_PANIC_CODE2_ERR_DM0B2SB3_IC_1        0x0    /* 0b	[3] */
#define DSP1_R234_PANIC_CODE2_ERR_DM0B2SB4_IC_1        0x0    /* 0b	[4] */
#define DSP1_R234_PANIC_CODE2_ERR_DM0B3SB0_IC_1        0x0    /* 0b	[8] */
#define DSP1_R234_PANIC_CODE2_ERR_DM0B3SB1_IC_1        0x0    /* 0b	[9] */
#define DSP1_R234_PANIC_CODE2_ERR_DM0B3SB2_IC_1        0x0    /* 0b	[10] */
#define DSP1_R234_PANIC_CODE2_ERR_DM0B3SB3_IC_1        0x0    /* 0b	[11] */
#define DSP1_R234_PANIC_CODE2_ERR_DM0B3SB4_IC_1        0x0    /* 0b	[12] */
#define DSP1_R234_PANIC_CODE2_ERR_DM0B2SB0_IC_1_MASK   0x1
#define DSP1_R234_PANIC_CODE2_ERR_DM0B2SB0_IC_1_SHIFT  0
#define DSP1_R234_PANIC_CODE2_ERR_DM0B2SB1_IC_1_MASK   0x2
#define DSP1_R234_PANIC_CODE2_ERR_DM0B2SB1_IC_1_SHIFT  1
#define DSP1_R234_PANIC_CODE2_ERR_DM0B2SB2_IC_1_MASK   0x4
#define DSP1_R234_PANIC_CODE2_ERR_DM0B2SB2_IC_1_SHIFT  2
#define DSP1_R234_PANIC_CODE2_ERR_DM0B2SB3_IC_1_MASK   0x8
#define DSP1_R234_PANIC_CODE2_ERR_DM0B2SB3_IC_1_SHIFT  3
#define DSP1_R234_PANIC_CODE2_ERR_DM0B2SB4_IC_1_MASK   0x10
#define DSP1_R234_PANIC_CODE2_ERR_DM0B2SB4_IC_1_SHIFT  4
#define DSP1_R234_PANIC_CODE2_ERR_DM0B3SB0_IC_1_MASK   0x100
#define DSP1_R234_PANIC_CODE2_ERR_DM0B3SB0_IC_1_SHIFT  8
#define DSP1_R234_PANIC_CODE2_ERR_DM0B3SB1_IC_1_MASK   0x200
#define DSP1_R234_PANIC_CODE2_ERR_DM0B3SB1_IC_1_SHIFT  9
#define DSP1_R234_PANIC_CODE2_ERR_DM0B3SB2_IC_1_MASK   0x400
#define DSP1_R234_PANIC_CODE2_ERR_DM0B3SB2_IC_1_SHIFT  10
#define DSP1_R234_PANIC_CODE2_ERR_DM0B3SB3_IC_1_MASK   0x800
#define DSP1_R234_PANIC_CODE2_ERR_DM0B3SB3_IC_1_SHIFT  11
#define DSP1_R234_PANIC_CODE2_ERR_DM0B3SB4_IC_1_MASK   0x1000
#define DSP1_R234_PANIC_CODE2_ERR_DM0B3SB4_IC_1_SHIFT  12

/* PANIC_CODE3 (IC 1) */
#define DSP1_R235_PANIC_CODE3_ERR_DM1B0SB0_IC_1        0x0    /* 0b	[0] */
#define DSP1_R235_PANIC_CODE3_ERR_DM1B0SB1_IC_1        0x0    /* 0b	[1] */
#define DSP1_R235_PANIC_CODE3_ERR_DM1B0SB2_IC_1        0x0    /* 0b	[2] */
#define DSP1_R235_PANIC_CODE3_ERR_DM1B0SB3_IC_1        0x0    /* 0b	[3] */
#define DSP1_R235_PANIC_CODE3_ERR_DM1B0SB4_IC_1        0x0    /* 0b	[4] */
#define DSP1_R235_PANIC_CODE3_ERR_DM1B1SB0_IC_1        0x0    /* 0b	[8] */
#define DSP1_R235_PANIC_CODE3_ERR_DM1B1SB1_IC_1        0x0    /* 0b	[9] */
#define DSP1_R235_PANIC_CODE3_ERR_DM1B1SB2_IC_1        0x0    /* 0b	[10] */
#define DSP1_R235_PANIC_CODE3_ERR_DM1B1SB3_IC_1        0x0    /* 0b	[11] */
#define DSP1_R235_PANIC_CODE3_ERR_DM1B1SB4_IC_1        0x0    /* 0b	[12] */
#define DSP1_R235_PANIC_CODE3_ERR_DM1B0SB0_IC_1_MASK   0x1
#define DSP1_R235_PANIC_CODE3_ERR_DM1B0SB0_IC_1_SHIFT  0
#define DSP1_R235_PANIC_CODE3_ERR_DM1B0SB1_IC_1_MASK   0x2
#define DSP1_R235_PANIC_CODE3_ERR_DM1B0SB1_IC_1_SHIFT  1
#define DSP1_R235_PANIC_CODE3_ERR_DM1B0SB2_IC_1_MASK   0x4
#define DSP1_R235_PANIC_CODE3_ERR_DM1B0SB2_IC_1_SHIFT  2
#define DSP1_R235_PANIC_CODE3_ERR_DM1B0SB3_IC_1_MASK   0x8
#define DSP1_R235_PANIC_CODE3_ERR_DM1B0SB3_IC_1_SHIFT  3
#define DSP1_R235_PANIC_CODE3_ERR_DM1B0SB4_IC_1_MASK   0x10
#define DSP1_R235_PANIC_CODE3_ERR_DM1B0SB4_IC_1_SHIFT  4
#define DSP1_R235_PANIC_CODE3_ERR_DM1B1SB0_IC_1_MASK   0x100
#define DSP1_R235_PANIC_CODE3_ERR_DM1B1SB0_IC_1_SHIFT  8
#define DSP1_R235_PANIC_CODE3_ERR_DM1B1SB1_IC_1_MASK   0x200
#define DSP1_R235_PANIC_CODE3_ERR_DM1B1SB1_IC_1_SHIFT  9
#define DSP1_R235_PANIC_CODE3_ERR_DM1B1SB2_IC_1_MASK   0x400
#define DSP1_R235_PANIC_CODE3_ERR_DM1B1SB2_IC_1_SHIFT  10
#define DSP1_R235_PANIC_CODE3_ERR_DM1B1SB3_IC_1_MASK   0x800
#define DSP1_R235_PANIC_CODE3_ERR_DM1B1SB3_IC_1_SHIFT  11
#define DSP1_R235_PANIC_CODE3_ERR_DM1B1SB4_IC_1_MASK   0x1000
#define DSP1_R235_PANIC_CODE3_ERR_DM1B1SB4_IC_1_SHIFT  12

/* PANIC_CODE4 (IC 1) */
#define DSP1_R236_PANIC_CODE4_ERR_DM1B2SB0_IC_1        0x0    /* 0b	[0] */
#define DSP1_R236_PANIC_CODE4_ERR_DM1B2SB1_IC_1        0x0    /* 0b	[1] */
#define DSP1_R236_PANIC_CODE4_ERR_DM1B2SB2_IC_1        0x0    /* 0b	[2] */
#define DSP1_R236_PANIC_CODE4_ERR_DM1B2SB3_IC_1        0x0    /* 0b	[3] */
#define DSP1_R236_PANIC_CODE4_ERR_DM1B2SB4_IC_1        0x0    /* 0b	[4] */
#define DSP1_R236_PANIC_CODE4_ERR_DM1B3SB0_IC_1        0x0    /* 0b	[8] */
#define DSP1_R236_PANIC_CODE4_ERR_DM1B3SB1_IC_1        0x0    /* 0b	[9] */
#define DSP1_R236_PANIC_CODE4_ERR_DM1B3SB2_IC_1        0x0    /* 0b	[10] */
#define DSP1_R236_PANIC_CODE4_ERR_DM1B3SB3_IC_1        0x0    /* 0b	[11] */
#define DSP1_R236_PANIC_CODE4_ERR_DM1B3SB4_IC_1        0x0    /* 0b	[12] */
#define DSP1_R236_PANIC_CODE4_ERR_DM1B2SB0_IC_1_MASK   0x1
#define DSP1_R236_PANIC_CODE4_ERR_DM1B2SB0_IC_1_SHIFT  0
#define DSP1_R236_PANIC_CODE4_ERR_DM1B2SB1_IC_1_MASK   0x2
#define DSP1_R236_PANIC_CODE4_ERR_DM1B2SB1_IC_1_SHIFT  1
#define DSP1_R236_PANIC_CODE4_ERR_DM1B2SB2_IC_1_MASK   0x4
#define DSP1_R236_PANIC_CODE4_ERR_DM1B2SB2_IC_1_SHIFT  2
#define DSP1_R236_PANIC_CODE4_ERR_DM1B2SB3_IC_1_MASK   0x8
#define DSP1_R236_PANIC_CODE4_ERR_DM1B2SB3_IC_1_SHIFT  3
#define DSP1_R236_PANIC_CODE4_ERR_DM1B2SB4_IC_1_MASK   0x10
#define DSP1_R236_PANIC_CODE4_ERR_DM1B2SB4_IC_1_SHIFT  4
#define DSP1_R236_PANIC_CODE4_ERR_DM1B3SB0_IC_1_MASK   0x100
#define DSP1_R236_PANIC_CODE4_ERR_DM1B3SB0_IC_1_SHIFT  8
#define DSP1_R236_PANIC_CODE4_ERR_DM1B3SB1_IC_1_MASK   0x200
#define DSP1_R236_PANIC_CODE4_ERR_DM1B3SB1_IC_1_SHIFT  9
#define DSP1_R236_PANIC_CODE4_ERR_DM1B3SB2_IC_1_MASK   0x400
#define DSP1_R236_PANIC_CODE4_ERR_DM1B3SB2_IC_1_SHIFT  10
#define DSP1_R236_PANIC_CODE4_ERR_DM1B3SB3_IC_1_MASK   0x800
#define DSP1_R236_PANIC_CODE4_ERR_DM1B3SB3_IC_1_SHIFT  11
#define DSP1_R236_PANIC_CODE4_ERR_DM1B3SB4_IC_1_MASK   0x1000
#define DSP1_R236_PANIC_CODE4_ERR_DM1B3SB4_IC_1_SHIFT  12

/* PANIC_CODE5 (IC 1) */
#define DSP1_R237_PANIC_CODE5_ERR_PM_B0SB0_IC_1        0x0    /* 0b	[0] */
#define DSP1_R237_PANIC_CODE5_ERR_PM_B0SB1_IC_1        0x0    /* 0b	[1] */
#define DSP1_R237_PANIC_CODE5_ERR_PM_B0SB2_IC_1        0x0    /* 0b	[2] */
#define DSP1_R237_PANIC_CODE5_ERR_PM_B0SB3_IC_1        0x0    /* 0b	[3] */
#define DSP1_R237_PANIC_CODE5_ERR_PM_B0SB4_IC_1        0x0    /* 0b	[4] */
#define DSP1_R237_PANIC_CODE5_ERR_PM_B0SB5_IC_1        0x0    /* 0b	[5] */
#define DSP1_R237_PANIC_CODE5_ERR_PM_B1SB0_IC_1        0x0    /* 0b	[8] */
#define DSP1_R237_PANIC_CODE5_ERR_PM_B1SB1_IC_1        0x0    /* 0b	[9] */
#define DSP1_R237_PANIC_CODE5_ERR_PM_B1SB2_IC_1        0x0    /* 0b	[10] */
#define DSP1_R237_PANIC_CODE5_ERR_PM_B1SB3_IC_1        0x0    /* 0b	[11] */
#define DSP1_R237_PANIC_CODE5_ERR_PM_B1SB4_IC_1        0x0    /* 0b	[12] */
#define DSP1_R237_PANIC_CODE5_ERR_PM_B1SB5_IC_1        0x0    /* 0b	[13] */
#define DSP1_R237_PANIC_CODE5_ERR_PM_B0SB0_IC_1_MASK   0x1
#define DSP1_R237_PANIC_CODE5_ERR_PM_B0SB0_IC_1_SHIFT  0
#define DSP1_R237_PANIC_CODE5_ERR_PM_B0SB1_IC_1_MASK   0x2
#define DSP1_R237_PANIC_CODE5_ERR_PM_B0SB1_IC_1_SHIFT  1
#define DSP1_R237_PANIC_CODE5_ERR_PM_B0SB2_IC_1_MASK   0x4
#define DSP1_R237_PANIC_CODE5_ERR_PM_B0SB2_IC_1_SHIFT  2
#define DSP1_R237_PANIC_CODE5_ERR_PM_B0SB3_IC_1_MASK   0x8
#define DSP1_R237_PANIC_CODE5_ERR_PM_B0SB3_IC_1_SHIFT  3
#define DSP1_R237_PANIC_CODE5_ERR_PM_B0SB4_IC_1_MASK   0x10
#define DSP1_R237_PANIC_CODE5_ERR_PM_B0SB4_IC_1_SHIFT  4
#define DSP1_R237_PANIC_CODE5_ERR_PM_B0SB5_IC_1_MASK   0x20
#define DSP1_R237_PANIC_CODE5_ERR_PM_B0SB5_IC_1_SHIFT  5
#define DSP1_R237_PANIC_CODE5_ERR_PM_B1SB0_IC_1_MASK   0x100
#define DSP1_R237_PANIC_CODE5_ERR_PM_B1SB0_IC_1_SHIFT  8
#define DSP1_R237_PANIC_CODE5_ERR_PM_B1SB1_IC_1_MASK   0x200
#define DSP1_R237_PANIC_CODE5_ERR_PM_B1SB1_IC_1_SHIFT  9
#define DSP1_R237_PANIC_CODE5_ERR_PM_B1SB2_IC_1_MASK   0x400
#define DSP1_R237_PANIC_CODE5_ERR_PM_B1SB2_IC_1_SHIFT  10
#define DSP1_R237_PANIC_CODE5_ERR_PM_B1SB3_IC_1_MASK   0x800
#define DSP1_R237_PANIC_CODE5_ERR_PM_B1SB3_IC_1_SHIFT  11
#define DSP1_R237_PANIC_CODE5_ERR_PM_B1SB4_IC_1_MASK   0x1000
#define DSP1_R237_PANIC_CODE5_ERR_PM_B1SB4_IC_1_SHIFT  12
#define DSP1_R237_PANIC_CODE5_ERR_PM_B1SB5_IC_1_MASK   0x2000
#define DSP1_R237_PANIC_CODE5_ERR_PM_B1SB5_IC_1_SHIFT  13

/* MP0_MODE (IC 1) */
#define DSP1_R238_MP0_MODE_MP_ENABLE_IC_1              0x0    /* 0b	[0] */
#define DSP1_R238_MP0_MODE_MP_MODE_IC_1                0x0    /* 000b	[3:1] */
#define DSP1_R238_MP0_MODE_DEBOUNCE_VALUE_IC_1         0x0    /* 0000b	[7:4] */
#define DSP1_R238_MP0_MODE_SS_SELECT_IC_1              0x0    /* 000b	[10:8] */
#define DSP1_R238_MP0_MODE_MP_ENABLE_IC_1_MASK         0x1
#define DSP1_R238_MP0_MODE_MP_ENABLE_IC_1_SHIFT        0
#define DSP1_R238_MP0_MODE_MP_MODE_IC_1_MASK           0xE
#define DSP1_R238_MP0_MODE_MP_MODE_IC_1_SHIFT          1
#define DSP1_R238_MP0_MODE_DEBOUNCE_VALUE_IC_1_MASK    0xF0
#define DSP1_R238_MP0_MODE_DEBOUNCE_VALUE_IC_1_SHIFT   4
#define DSP1_R238_MP0_MODE_SS_SELECT_IC_1_MASK         0x700
#define DSP1_R238_MP0_MODE_SS_SELECT_IC_1_SHIFT        8

/* MP1_MODE (IC 1) */
#define DSP1_R239_MP1_MODE_MP_ENABLE_IC_1              0x0    /* 0b	[0] */
#define DSP1_R239_MP1_MODE_MP_MODE_IC_1                0x0    /* 000b	[3:1] */
#define DSP1_R239_MP1_MODE_DEBOUNCE_VALUE_IC_1         0x0    /* 0000b	[7:4] */
#define DSP1_R239_MP1_MODE_SS_SELECT_IC_1              0x0    /* 000b	[10:8] */
#define DSP1_R239_MP1_MODE_MP_ENABLE_IC_1_MASK         0x1
#define DSP1_R239_MP1_MODE_MP_ENABLE_IC_1_SHIFT        0
#define DSP1_R239_MP1_MODE_MP_MODE_IC_1_MASK           0xE
#define DSP1_R239_MP1_MODE_MP_MODE_IC_1_SHIFT          1
#define DSP1_R239_MP1_MODE_DEBOUNCE_VALUE_IC_1_MASK    0xF0
#define DSP1_R239_MP1_MODE_DEBOUNCE_VALUE_IC_1_SHIFT   4
#define DSP1_R239_MP1_MODE_SS_SELECT_IC_1_MASK         0x700
#define DSP1_R239_MP1_MODE_SS_SELECT_IC_1_SHIFT        8

/* MP2_MODE (IC 1) */
#define DSP1_R240_MP2_MODE_MP_ENABLE_IC_1              0x0    /* 0b	[0] */
#define DSP1_R240_MP2_MODE_MP_MODE_IC_1                0x0    /* 000b	[3:1] */
#define DSP1_R240_MP2_MODE_DEBOUNCE_VALUE_IC_1         0x0    /* 0000b	[7:4] */
#define DSP1_R240_MP2_MODE_SS_SELECT_IC_1              0x0    /* 000b	[10:8] */
#define DSP1_R240_MP2_MODE_MP_ENABLE_IC_1_MASK         0x1
#define DSP1_R240_MP2_MODE_MP_ENABLE_IC_1_SHIFT        0
#define DSP1_R240_MP2_MODE_MP_MODE_IC_1_MASK           0xE
#define DSP1_R240_MP2_MODE_MP_MODE_IC_1_SHIFT          1
#define DSP1_R240_MP2_MODE_DEBOUNCE_VALUE_IC_1_MASK    0xF0
#define DSP1_R240_MP2_MODE_DEBOUNCE_VALUE_IC_1_SHIFT   4
#define DSP1_R240_MP2_MODE_SS_SELECT_IC_1_MASK         0x700
#define DSP1_R240_MP2_MODE_SS_SELECT_IC_1_SHIFT        8

/* MP3_MODE (IC 1) */
#define DSP1_R241_MP3_MODE_MP_ENABLE_IC_1              0x0    /* 0b	[0] */
#define DSP1_R241_MP3_MODE_MP_MODE_IC_1                0x0    /* 000b	[3:1] */
#define DSP1_R241_MP3_MODE_DEBOUNCE_VALUE_IC_1         0x0    /* 0000b	[7:4] */
#define DSP1_R241_MP3_MODE_SS_SELECT_IC_1              0x0    /* 000b	[10:8] */
#define DSP1_R241_MP3_MODE_MP_ENABLE_IC_1_MASK         0x1
#define DSP1_R241_MP3_MODE_MP_ENABLE_IC_1_SHIFT        0
#define DSP1_R241_MP3_MODE_MP_MODE_IC_1_MASK           0xE
#define DSP1_R241_MP3_MODE_MP_MODE_IC_1_SHIFT          1
#define DSP1_R241_MP3_MODE_DEBOUNCE_VALUE_IC_1_MASK    0xF0
#define DSP1_R241_MP3_MODE_DEBOUNCE_VALUE_IC_1_SHIFT   4
#define DSP1_R241_MP3_MODE_SS_SELECT_IC_1_MASK         0x700
#define DSP1_R241_MP3_MODE_SS_SELECT_IC_1_SHIFT        8

/* MP4_MODE (IC 1) */
#define DSP1_R242_MP4_MODE_MP_ENABLE_IC_1              0x0    /* 0b	[0] */
#define DSP1_R242_MP4_MODE_MP_MODE_IC_1                0x0    /* 000b	[3:1] */
#define DSP1_R242_MP4_MODE_DEBOUNCE_VALUE_IC_1         0x0    /* 0000b	[7:4] */
#define DSP1_R242_MP4_MODE_SS_SELECT_IC_1              0x0    /* 000b	[10:8] */
#define DSP1_R242_MP4_MODE_MP_ENABLE_IC_1_MASK         0x1
#define DSP1_R242_MP4_MODE_MP_ENABLE_IC_1_SHIFT        0
#define DSP1_R242_MP4_MODE_MP_MODE_IC_1_MASK           0xE
#define DSP1_R242_MP4_MODE_MP_MODE_IC_1_SHIFT          1
#define DSP1_R242_MP4_MODE_DEBOUNCE_VALUE_IC_1_MASK    0xF0
#define DSP1_R242_MP4_MODE_DEBOUNCE_VALUE_IC_1_SHIFT   4
#define DSP1_R242_MP4_MODE_SS_SELECT_IC_1_MASK         0x700
#define DSP1_R242_MP4_MODE_SS_SELECT_IC_1_SHIFT        8

/* MP5_MODE (IC 1) */
#define DSP1_R243_MP5_MODE_MP_ENABLE_IC_1              0x0    /* 0b	[0] */
#define DSP1_R243_MP5_MODE_MP_MODE_IC_1                0x0    /* 000b	[3:1] */
#define DSP1_R243_MP5_MODE_DEBOUNCE_VALUE_IC_1         0x0    /* 0000b	[7:4] */
#define DSP1_R243_MP5_MODE_SS_SELECT_IC_1              0x0    /* 000b	[10:8] */
#define DSP1_R243_MP5_MODE_MP_ENABLE_IC_1_MASK         0x1
#define DSP1_R243_MP5_MODE_MP_ENABLE_IC_1_SHIFT        0
#define DSP1_R243_MP5_MODE_MP_MODE_IC_1_MASK           0xE
#define DSP1_R243_MP5_MODE_MP_MODE_IC_1_SHIFT          1
#define DSP1_R243_MP5_MODE_DEBOUNCE_VALUE_IC_1_MASK    0xF0
#define DSP1_R243_MP5_MODE_DEBOUNCE_VALUE_IC_1_SHIFT   4
#define DSP1_R243_MP5_MODE_SS_SELECT_IC_1_MASK         0x700
#define DSP1_R243_MP5_MODE_SS_SELECT_IC_1_SHIFT        8

/* MP6_MODE (IC 1) */
#define DSP1_R244_MP6_MODE_MP_ENABLE_IC_1              0x0    /* 0b	[0] */
#define DSP1_R244_MP6_MODE_MP_MODE_IC_1                0x0    /* 000b	[3:1] */
#define DSP1_R244_MP6_MODE_DEBOUNCE_VALUE_IC_1         0x0    /* 0000b	[7:4] */
#define DSP1_R244_MP6_MODE_SS_SELECT_IC_1              0x0    /* 000b	[10:8] */
#define DSP1_R244_MP6_MODE_MP_ENABLE_IC_1_MASK         0x1
#define DSP1_R244_MP6_MODE_MP_ENABLE_IC_1_SHIFT        0
#define DSP1_R244_MP6_MODE_MP_MODE_IC_1_MASK           0xE
#define DSP1_R244_MP6_MODE_MP_MODE_IC_1_SHIFT          1
#define DSP1_R244_MP6_MODE_DEBOUNCE_VALUE_IC_1_MASK    0xF0
#define DSP1_R244_MP6_MODE_DEBOUNCE_VALUE_IC_1_SHIFT   4
#define DSP1_R244_MP6_MODE_SS_SELECT_IC_1_MASK         0x700
#define DSP1_R244_MP6_MODE_SS_SELECT_IC_1_SHIFT        8

/* MP7_MODE (IC 1) */
#define DSP1_R245_MP7_MODE_MP_ENABLE_IC_1              0x0    /* 0b	[0] */
#define DSP1_R245_MP7_MODE_MP_MODE_IC_1                0x0    /* 000b	[3:1] */
#define DSP1_R245_MP7_MODE_DEBOUNCE_VALUE_IC_1         0x0    /* 0000b	[7:4] */
#define DSP1_R245_MP7_MODE_SS_SELECT_IC_1              0x0    /* 000b	[10:8] */
#define DSP1_R245_MP7_MODE_MP_ENABLE_IC_1_MASK         0x1
#define DSP1_R245_MP7_MODE_MP_ENABLE_IC_1_SHIFT        0
#define DSP1_R245_MP7_MODE_MP_MODE_IC_1_MASK           0xE
#define DSP1_R245_MP7_MODE_MP_MODE_IC_1_SHIFT          1
#define DSP1_R245_MP7_MODE_DEBOUNCE_VALUE_IC_1_MASK    0xF0
#define DSP1_R245_MP7_MODE_DEBOUNCE_VALUE_IC_1_SHIFT   4
#define DSP1_R245_MP7_MODE_SS_SELECT_IC_1_MASK         0x700
#define DSP1_R245_MP7_MODE_SS_SELECT_IC_1_SHIFT        8

/* MP8_MODE (IC 1) */
#define DSP1_R246_MP8_MODE_MP_ENABLE_IC_1              0x0    /* 0b	[0] */
#define DSP1_R246_MP8_MODE_MP_MODE_IC_1                0x0    /* 000b	[3:1] */
#define DSP1_R246_MP8_MODE_DEBOUNCE_VALUE_IC_1         0x0    /* 0000b	[7:4] */
#define DSP1_R246_MP8_MODE_SS_SELECT_IC_1              0x0    /* 000b	[10:8] */
#define DSP1_R246_MP8_MODE_MP_ENABLE_IC_1_MASK         0x1
#define DSP1_R246_MP8_MODE_MP_ENABLE_IC_1_SHIFT        0
#define DSP1_R246_MP8_MODE_MP_MODE_IC_1_MASK           0xE
#define DSP1_R246_MP8_MODE_MP_MODE_IC_1_SHIFT          1
#define DSP1_R246_MP8_MODE_DEBOUNCE_VALUE_IC_1_MASK    0xF0
#define DSP1_R246_MP8_MODE_DEBOUNCE_VALUE_IC_1_SHIFT   4
#define DSP1_R246_MP8_MODE_SS_SELECT_IC_1_MASK         0x700
#define DSP1_R246_MP8_MODE_SS_SELECT_IC_1_SHIFT        8

/* MP9_MODE (IC 1) */
#define DSP1_R247_MP9_MODE_MP_ENABLE_IC_1              0x0    /* 0b	[0] */
#define DSP1_R247_MP9_MODE_MP_MODE_IC_1                0x0    /* 000b	[3:1] */
#define DSP1_R247_MP9_MODE_DEBOUNCE_VALUE_IC_1         0x0    /* 0000b	[7:4] */
#define DSP1_R247_MP9_MODE_SS_SELECT_IC_1              0x0    /* 000b	[10:8] */
#define DSP1_R247_MP9_MODE_MP_ENABLE_IC_1_MASK         0x1
#define DSP1_R247_MP9_MODE_MP_ENABLE_IC_1_SHIFT        0
#define DSP1_R247_MP9_MODE_MP_MODE_IC_1_MASK           0xE
#define DSP1_R247_MP9_MODE_MP_MODE_IC_1_SHIFT          1
#define DSP1_R247_MP9_MODE_DEBOUNCE_VALUE_IC_1_MASK    0xF0
#define DSP1_R247_MP9_MODE_DEBOUNCE_VALUE_IC_1_SHIFT   4
#define DSP1_R247_MP9_MODE_SS_SELECT_IC_1_MASK         0x700
#define DSP1_R247_MP9_MODE_SS_SELECT_IC_1_SHIFT        8

/* MP10_MODE (IC 1) */
#define DSP1_R248_MP10_MODE_MP_ENABLE_IC_1             0x0    /* 0b	[0] */
#define DSP1_R248_MP10_MODE_MP_MODE_IC_1               0x0    /* 000b	[3:1] */
#define DSP1_R248_MP10_MODE_DEBOUNCE_VALUE_IC_1        0x0    /* 0000b	[7:4] */
#define DSP1_R248_MP10_MODE_SS_SELECT_IC_1             0x0    /* 000b	[10:8] */
#define DSP1_R248_MP10_MODE_MP_ENABLE_IC_1_MASK        0x1
#define DSP1_R248_MP10_MODE_MP_ENABLE_IC_1_SHIFT       0
#define DSP1_R248_MP10_MODE_MP_MODE_IC_1_MASK          0xE
#define DSP1_R248_MP10_MODE_MP_MODE_IC_1_SHIFT         1
#define DSP1_R248_MP10_MODE_DEBOUNCE_VALUE_IC_1_MASK   0xF0
#define DSP1_R248_MP10_MODE_DEBOUNCE_VALUE_IC_1_SHIFT  4
#define DSP1_R248_MP10_MODE_SS_SELECT_IC_1_MASK        0x700
#define DSP1_R248_MP10_MODE_SS_SELECT_IC_1_SHIFT       8

/* MP11_MODE (IC 1) */
#define DSP1_R249_MP11_MODE_MP_ENABLE_IC_1             0x0    /* 0b	[0] */
#define DSP1_R249_MP11_MODE_MP_MODE_IC_1               0x0    /* 000b	[3:1] */
#define DSP1_R249_MP11_MODE_DEBOUNCE_VALUE_IC_1        0x0    /* 0000b	[7:4] */
#define DSP1_R249_MP11_MODE_SS_SELECT_IC_1             0x0    /* 000b	[10:8] */
#define DSP1_R249_MP11_MODE_MP_ENABLE_IC_1_MASK        0x1
#define DSP1_R249_MP11_MODE_MP_ENABLE_IC_1_SHIFT       0
#define DSP1_R249_MP11_MODE_MP_MODE_IC_1_MASK          0xE
#define DSP1_R249_MP11_MODE_MP_MODE_IC_1_SHIFT         1
#define DSP1_R249_MP11_MODE_DEBOUNCE_VALUE_IC_1_MASK   0xF0
#define DSP1_R249_MP11_MODE_DEBOUNCE_VALUE_IC_1_SHIFT  4
#define DSP1_R249_MP11_MODE_SS_SELECT_IC_1_MASK        0x700
#define DSP1_R249_MP11_MODE_SS_SELECT_IC_1_SHIFT       8

/* MP12_MODE (IC 1) */
#define DSP1_R250_MP12_MODE_MP_ENABLE_IC_1             0x0    /* 0b	[0] */
#define DSP1_R250_MP12_MODE_MP_MODE_IC_1               0x0    /* 000b	[3:1] */
#define DSP1_R250_MP12_MODE_DEBOUNCE_VALUE_IC_1        0x0    /* 0000b	[7:4] */
#define DSP1_R250_MP12_MODE_SS_SELECT_IC_1             0x0    /* 000b	[10:8] */
#define DSP1_R250_MP12_MODE_MP_ENABLE_IC_1_MASK        0x1
#define DSP1_R250_MP12_MODE_MP_ENABLE_IC_1_SHIFT       0
#define DSP1_R250_MP12_MODE_MP_MODE_IC_1_MASK          0xE
#define DSP1_R250_MP12_MODE_MP_MODE_IC_1_SHIFT         1
#define DSP1_R250_MP12_MODE_DEBOUNCE_VALUE_IC_1_MASK   0xF0
#define DSP1_R250_MP12_MODE_DEBOUNCE_VALUE_IC_1_SHIFT  4
#define DSP1_R250_MP12_MODE_SS_SELECT_IC_1_MASK        0x700
#define DSP1_R250_MP12_MODE_SS_SELECT_IC_1_SHIFT       8

/* MP13_MODE (IC 1) */
#define DSP1_R251_MP13_MODE_MP_ENABLE_IC_1             0x0    /* 0b	[0] */
#define DSP1_R251_MP13_MODE_MP_MODE_IC_1               0x0    /* 000b	[3:1] */
#define DSP1_R251_MP13_MODE_DEBOUNCE_VALUE_IC_1        0x0    /* 0000b	[7:4] */
#define DSP1_R251_MP13_MODE_SS_SELECT_IC_1             0x0    /* 000b	[10:8] */
#define DSP1_R251_MP13_MODE_MP_ENABLE_IC_1_MASK        0x1
#define DSP1_R251_MP13_MODE_MP_ENABLE_IC_1_SHIFT       0
#define DSP1_R251_MP13_MODE_MP_MODE_IC_1_MASK          0xE
#define DSP1_R251_MP13_MODE_MP_MODE_IC_1_SHIFT         1
#define DSP1_R251_MP13_MODE_DEBOUNCE_VALUE_IC_1_MASK   0xF0
#define DSP1_R251_MP13_MODE_DEBOUNCE_VALUE_IC_1_SHIFT  4
#define DSP1_R251_MP13_MODE_SS_SELECT_IC_1_MASK        0x700
#define DSP1_R251_MP13_MODE_SS_SELECT_IC_1_SHIFT       8

/* MP0_WRITE (IC 1) */
#define DSP1_R252_MP0_WRITE_MP_REG_WRITE_IC_1          0x0    /* 0b	[0] */
#define DSP1_R252_MP0_WRITE_MP_REG_WRITE_IC_1_MASK     0x1
#define DSP1_R252_MP0_WRITE_MP_REG_WRITE_IC_1_SHIFT    0

/* MP1_WRITE (IC 1) */
#define DSP1_R253_MP1_WRITE_MP_REG_WRITE_IC_1          0x0    /* 0b	[0] */
#define DSP1_R253_MP1_WRITE_MP_REG_WRITE_IC_1_MASK     0x1
#define DSP1_R253_MP1_WRITE_MP_REG_WRITE_IC_1_SHIFT    0

/* MP2_WRITE (IC 1) */
#define DSP1_R254_MP2_WRITE_MP_REG_WRITE_IC_1          0x0    /* 0b	[0] */
#define DSP1_R254_MP2_WRITE_MP_REG_WRITE_IC_1_MASK     0x1
#define DSP1_R254_MP2_WRITE_MP_REG_WRITE_IC_1_SHIFT    0

/* MP3_WRITE (IC 1) */
#define DSP1_R255_MP3_WRITE_MP_REG_WRITE_IC_1          0x0    /* 0b	[0] */
#define DSP1_R255_MP3_WRITE_MP_REG_WRITE_IC_1_MASK     0x1
#define DSP1_R255_MP3_WRITE_MP_REG_WRITE_IC_1_SHIFT    0

/* MP4_WRITE (IC 1) */
#define DSP1_R256_MP4_WRITE_MP_REG_WRITE_IC_1          0x0    /* 0b	[0] */
#define DSP1_R256_MP4_WRITE_MP_REG_WRITE_IC_1_MASK     0x1
#define DSP1_R256_MP4_WRITE_MP_REG_WRITE_IC_1_SHIFT    0

/* MP5_WRITE (IC 1) */
#define DSP1_R257_MP5_WRITE_MP_REG_WRITE_IC_1          0x0    /* 0b	[0] */
#define DSP1_R257_MP5_WRITE_MP_REG_WRITE_IC_1_MASK     0x1
#define DSP1_R257_MP5_WRITE_MP_REG_WRITE_IC_1_SHIFT    0

/* MP6_WRITE (IC 1) */
#define DSP1_R258_MP6_WRITE_MP_REG_WRITE_IC_1          0x0    /* 0b	[0] */
#define DSP1_R258_MP6_WRITE_MP_REG_WRITE_IC_1_MASK     0x1
#define DSP1_R258_MP6_WRITE_MP_REG_WRITE_IC_1_SHIFT    0

/* MP7_WRITE (IC 1) */
#define DSP1_R259_MP7_WRITE_MP_REG_WRITE_IC_1          0x0    /* 0b	[0] */
#define DSP1_R259_MP7_WRITE_MP_REG_WRITE_IC_1_MASK     0x1
#define DSP1_R259_MP7_WRITE_MP_REG_WRITE_IC_1_SHIFT    0

/* MP8_WRITE (IC 1) */
#define DSP1_R260_MP8_WRITE_MP_REG_WRITE_IC_1          0x0    /* 0b	[0] */
#define DSP1_R260_MP8_WRITE_MP_REG_WRITE_IC_1_MASK     0x1
#define DSP1_R260_MP8_WRITE_MP_REG_WRITE_IC_1_SHIFT    0

/* MP9_WRITE (IC 1) */
#define DSP1_R261_MP9_WRITE_MP_REG_WRITE_IC_1          0x0    /* 0b	[0] */
#define DSP1_R261_MP9_WRITE_MP_REG_WRITE_IC_1_MASK     0x1
#define DSP1_R261_MP9_WRITE_MP_REG_WRITE_IC_1_SHIFT    0

/* MP10_WRITE (IC 1) */
#define DSP1_R262_MP10_WRITE_MP_REG_WRITE_IC_1         0x0    /* 0b	[0] */
#define DSP1_R262_MP10_WRITE_MP_REG_WRITE_IC_1_MASK    0x1
#define DSP1_R262_MP10_WRITE_MP_REG_WRITE_IC_1_SHIFT   0

/* MP11_WRITE (IC 1) */
#define DSP1_R263_MP11_WRITE_MP_REG_WRITE_IC_1         0x0    /* 0b	[0] */
#define DSP1_R263_MP11_WRITE_MP_REG_WRITE_IC_1_MASK    0x1
#define DSP1_R263_MP11_WRITE_MP_REG_WRITE_IC_1_SHIFT   0

/* MP12_WRITE (IC 1) */
#define DSP1_R264_MP12_WRITE_MP_REG_WRITE_IC_1         0x0    /* 0b	[0] */
#define DSP1_R264_MP12_WRITE_MP_REG_WRITE_IC_1_MASK    0x1
#define DSP1_R264_MP12_WRITE_MP_REG_WRITE_IC_1_SHIFT   0

/* MP13_WRITE (IC 1) */
#define DSP1_R265_MP13_WRITE_MP_REG_WRITE_IC_1         0x0    /* 0b	[0] */
#define DSP1_R265_MP13_WRITE_MP_REG_WRITE_IC_1_MASK    0x1
#define DSP1_R265_MP13_WRITE_MP_REG_WRITE_IC_1_SHIFT   0

/* MP0_READ (IC 1) */
#define DSP1_R266_MP0_READ_MP_REG_READ_IC_1            0x0    /* 0b	[0] */
#define DSP1_R266_MP0_READ_MP_REG_READ_IC_1_MASK       0x1
#define DSP1_R266_MP0_READ_MP_REG_READ_IC_1_SHIFT      0

/* MP1_READ (IC 1) */
#define DSP1_R267_MP1_READ_MP_REG_READ_IC_1            0x0    /* 0b	[0] */
#define DSP1_R267_MP1_READ_MP_REG_READ_IC_1_MASK       0x1
#define DSP1_R267_MP1_READ_MP_REG_READ_IC_1_SHIFT      0

/* MP2_READ (IC 1) */
#define DSP1_R268_MP2_READ_MP_REG_READ_IC_1            0x0    /* 0b	[0] */
#define DSP1_R268_MP2_READ_MP_REG_READ_IC_1_MASK       0x1
#define DSP1_R268_MP2_READ_MP_REG_READ_IC_1_SHIFT      0

/* MP3_READ (IC 1) */
#define DSP1_R269_MP3_READ_MP_REG_READ_IC_1            0x0    /* 0b	[0] */
#define DSP1_R269_MP3_READ_MP_REG_READ_IC_1_MASK       0x1
#define DSP1_R269_MP3_READ_MP_REG_READ_IC_1_SHIFT      0

/* MP4_READ (IC 1) */
#define DSP1_R270_MP4_READ_MP_REG_READ_IC_1            0x0    /* 0b	[0] */
#define DSP1_R270_MP4_READ_MP_REG_READ_IC_1_MASK       0x1
#define DSP1_R270_MP4_READ_MP_REG_READ_IC_1_SHIFT      0

/* MP5_READ (IC 1) */
#define DSP1_R271_MP5_READ_MP_REG_READ_IC_1            0x0    /* 0b	[0] */
#define DSP1_R271_MP5_READ_MP_REG_READ_IC_1_MASK       0x1
#define DSP1_R271_MP5_READ_MP_REG_READ_IC_1_SHIFT      0

/* MP6_READ (IC 1) */
#define DSP1_R272_MP6_READ_MP_REG_READ_IC_1            0x0    /* 0b	[0] */
#define DSP1_R272_MP6_READ_MP_REG_READ_IC_1_MASK       0x1
#define DSP1_R272_MP6_READ_MP_REG_READ_IC_1_SHIFT      0

/* MP7_READ (IC 1) */
#define DSP1_R273_MP7_READ_MP_REG_READ_IC_1            0x0    /* 0b	[0] */
#define DSP1_R273_MP7_READ_MP_REG_READ_IC_1_MASK       0x1
#define DSP1_R273_MP7_READ_MP_REG_READ_IC_1_SHIFT      0

/* MP8_READ (IC 1) */
#define DSP1_R274_MP8_READ_MP_REG_READ_IC_1            0x0    /* 0b	[0] */
#define DSP1_R274_MP8_READ_MP_REG_READ_IC_1_MASK       0x1
#define DSP1_R274_MP8_READ_MP_REG_READ_IC_1_SHIFT      0

/* MP9_READ (IC 1) */
#define DSP1_R275_MP9_READ_MP_REG_READ_IC_1            0x0    /* 0b	[0] */
#define DSP1_R275_MP9_READ_MP_REG_READ_IC_1_MASK       0x1
#define DSP1_R275_MP9_READ_MP_REG_READ_IC_1_SHIFT      0

/* MP10_READ (IC 1) */
#define DSP1_R276_MP10_READ_MP_REG_READ_IC_1           0x0    /* 0b	[0] */
#define DSP1_R276_MP10_READ_MP_REG_READ_IC_1_MASK      0x1
#define DSP1_R276_MP10_READ_MP_REG_READ_IC_1_SHIFT     0

/* MP11_READ (IC 1) */
#define DSP1_R277_MP11_READ_MP_REG_READ_IC_1           0x0    /* 0b	[0] */
#define DSP1_R277_MP11_READ_MP_REG_READ_IC_1_MASK      0x1
#define DSP1_R277_MP11_READ_MP_REG_READ_IC_1_SHIFT     0

/* MP12_READ (IC 1) */
#define DSP1_R278_MP12_READ_MP_REG_READ_IC_1           0x0    /* 0b	[0] */
#define DSP1_R278_MP12_READ_MP_REG_READ_IC_1_MASK      0x1
#define DSP1_R278_MP12_READ_MP_REG_READ_IC_1_SHIFT     0

/* MP13_READ (IC 1) */
#define DSP1_R279_MP13_READ_MP_REG_READ_IC_1           0x0    /* 0b	[0] */
#define DSP1_R279_MP13_READ_MP_REG_READ_IC_1_MASK      0x1
#define DSP1_R279_MP13_READ_MP_REG_READ_IC_1_SHIFT     0

/* DMIC_CTRL0 (IC 1) */
#define DSP1_R280_DMIC_CTRL0_DMIC_EN_IC_1              0x0    /* 0b	[0] */
#define DSP1_R280_DMIC_CTRL0_DMSW_IC_1                 0x0    /* 0b	[1] */
#define DSP1_R280_DMIC_CTRL0_DMPOL_IC_1                0x0    /* 0b	[2] */
#define DSP1_R280_DMIC_CTRL0_HPF_IC_1                  0x0    /* 0b	[3] */
#define DSP1_R280_DMIC_CTRL0_DMIC_CLK_IC_1             0x0    /* 000b	[6:4] */
#define DSP1_R280_DMIC_CTRL0_MIC_DATA_SRC_IC_1         0x0    /* 0000b	[11:8] */
#define DSP1_R280_DMIC_CTRL0_CUTOFF_IC_1               0x4    /* 100b	[14:12] */
#define DSP1_R280_DMIC_CTRL0_DMIC_EN_IC_1_MASK         0x1
#define DSP1_R280_DMIC_CTRL0_DMIC_EN_IC_1_SHIFT        0
#define DSP1_R280_DMIC_CTRL0_DMSW_IC_1_MASK            0x2
#define DSP1_R280_DMIC_CTRL0_DMSW_IC_1_SHIFT           1
#define DSP1_R280_DMIC_CTRL0_DMPOL_IC_1_MASK           0x4
#define DSP1_R280_DMIC_CTRL0_DMPOL_IC_1_SHIFT          2
#define DSP1_R280_DMIC_CTRL0_HPF_IC_1_MASK             0x8
#define DSP1_R280_DMIC_CTRL0_HPF_IC_1_SHIFT            3
#define DSP1_R280_DMIC_CTRL0_DMIC_CLK_IC_1_MASK        0x70
#define DSP1_R280_DMIC_CTRL0_DMIC_CLK_IC_1_SHIFT       4
#define DSP1_R280_DMIC_CTRL0_MIC_DATA_SRC_IC_1_MASK    0xF00
#define DSP1_R280_DMIC_CTRL0_MIC_DATA_SRC_IC_1_SHIFT   8
#define DSP1_R280_DMIC_CTRL0_CUTOFF_IC_1_MASK          0x7000
#define DSP1_R280_DMIC_CTRL0_CUTOFF_IC_1_SHIFT         12

/* DMIC_CTRL1 (IC 1) */
#define DSP1_R281_DMIC_CTRL1_DMIC_EN_IC_1              0x0    /* 0b	[0] */
#define DSP1_R281_DMIC_CTRL1_DMSW_IC_1                 0x0    /* 0b	[1] */
#define DSP1_R281_DMIC_CTRL1_DMPOL_IC_1                0x0    /* 0b	[2] */
#define DSP1_R281_DMIC_CTRL1_HPF_IC_1                  0x0    /* 0b	[3] */
#define DSP1_R281_DMIC_CTRL1_DMIC_CLK_IC_1             0x0    /* 000b	[6:4] */
#define DSP1_R281_DMIC_CTRL1_MIC_DATA_SRC_IC_1         0x0    /* 0000b	[11:8] */
#define DSP1_R281_DMIC_CTRL1_CUTOFF_IC_1               0x4    /* 100b	[14:12] */
#define DSP1_R281_DMIC_CTRL1_DMIC_EN_IC_1_MASK         0x1
#define DSP1_R281_DMIC_CTRL1_DMIC_EN_IC_1_SHIFT        0
#define DSP1_R281_DMIC_CTRL1_DMSW_IC_1_MASK            0x2
#define DSP1_R281_DMIC_CTRL1_DMSW_IC_1_SHIFT           1
#define DSP1_R281_DMIC_CTRL1_DMPOL_IC_1_MASK           0x4
#define DSP1_R281_DMIC_CTRL1_DMPOL_IC_1_SHIFT          2
#define DSP1_R281_DMIC_CTRL1_HPF_IC_1_MASK             0x8
#define DSP1_R281_DMIC_CTRL1_HPF_IC_1_SHIFT            3
#define DSP1_R281_DMIC_CTRL1_DMIC_CLK_IC_1_MASK        0x70
#define DSP1_R281_DMIC_CTRL1_DMIC_CLK_IC_1_SHIFT       4
#define DSP1_R281_DMIC_CTRL1_MIC_DATA_SRC_IC_1_MASK    0xF00
#define DSP1_R281_DMIC_CTRL1_MIC_DATA_SRC_IC_1_SHIFT   8
#define DSP1_R281_DMIC_CTRL1_CUTOFF_IC_1_MASK          0x7000
#define DSP1_R281_DMIC_CTRL1_CUTOFF_IC_1_SHIFT         12

/* ASRC_LOCK (IC 1) */
#define DSP1_R282_ASRC_LOCK_ASRC0L_IC_1                0x0    /* 0b	[0] */
#define DSP1_R282_ASRC_LOCK_ASRC1L_IC_1                0x0    /* 0b	[1] */
#define DSP1_R282_ASRC_LOCK_ASRC2L_IC_1                0x0    /* 0b	[2] */
#define DSP1_R282_ASRC_LOCK_ASRC3L_IC_1                0x0    /* 0b	[3] */
#define DSP1_R282_ASRC_LOCK_ASRC4L_IC_1                0x0    /* 0b	[4] */
#define DSP1_R282_ASRC_LOCK_ASRC5L_IC_1                0x0    /* 0b	[5] */
#define DSP1_R282_ASRC_LOCK_ASRC6L_IC_1                0x0    /* 0b	[6] */
#define DSP1_R282_ASRC_LOCK_ASRC7L_IC_1                0x0    /* 0b	[7] */
#define DSP1_R282_ASRC_LOCK_ASRC0L_IC_1_MASK           0x1
#define DSP1_R282_ASRC_LOCK_ASRC0L_IC_1_SHIFT          0
#define DSP1_R282_ASRC_LOCK_ASRC1L_IC_1_MASK           0x2
#define DSP1_R282_ASRC_LOCK_ASRC1L_IC_1_SHIFT          1
#define DSP1_R282_ASRC_LOCK_ASRC2L_IC_1_MASK           0x4
#define DSP1_R282_ASRC_LOCK_ASRC2L_IC_1_SHIFT          2
#define DSP1_R282_ASRC_LOCK_ASRC3L_IC_1_MASK           0x8
#define DSP1_R282_ASRC_LOCK_ASRC3L_IC_1_SHIFT          3
#define DSP1_R282_ASRC_LOCK_ASRC4L_IC_1_MASK           0x10
#define DSP1_R282_ASRC_LOCK_ASRC4L_IC_1_SHIFT          4
#define DSP1_R282_ASRC_LOCK_ASRC5L_IC_1_MASK           0x20
#define DSP1_R282_ASRC_LOCK_ASRC5L_IC_1_SHIFT          5
#define DSP1_R282_ASRC_LOCK_ASRC6L_IC_1_MASK           0x40
#define DSP1_R282_ASRC_LOCK_ASRC6L_IC_1_SHIFT          6
#define DSP1_R282_ASRC_LOCK_ASRC7L_IC_1_MASK           0x80
#define DSP1_R282_ASRC_LOCK_ASRC7L_IC_1_SHIFT          7

/* ASRC_MUTE (IC 1) */
#define DSP1_R283_ASRC_MUTE_ASRC0M_IC_1                0x0    /* 0b	[0] */
#define DSP1_R283_ASRC_MUTE_ASRC1M_IC_1                0x0    /* 0b	[1] */
#define DSP1_R283_ASRC_MUTE_ASRC2M_IC_1                0x0    /* 0b	[2] */
#define DSP1_R283_ASRC_MUTE_ASRC3M_IC_1                0x0    /* 0b	[3] */
#define DSP1_R283_ASRC_MUTE_ASRC4M_IC_1                0x0    /* 0b	[4] */
#define DSP1_R283_ASRC_MUTE_ASRC5M_IC_1                0x0    /* 0b	[5] */
#define DSP1_R283_ASRC_MUTE_ASRC6M_IC_1                0x0    /* 0b	[6] */
#define DSP1_R283_ASRC_MUTE_ASRC7M_IC_1                0x0    /* 0b	[7] */
#define DSP1_R283_ASRC_MUTE_ASRC_RAMP0_IC_1            0x0    /* 0b	[8] */
#define DSP1_R283_ASRC_MUTE_ASRC_RAMP1_IC_1            0x0    /* 0b	[9] */
#define DSP1_R283_ASRC_MUTE_LOCKMUTE_IC_1              0x0    /* 0b	[10] */
#define DSP1_R283_ASRC_MUTE_ASRC0M_IC_1_MASK           0x1
#define DSP1_R283_ASRC_MUTE_ASRC0M_IC_1_SHIFT          0
#define DSP1_R283_ASRC_MUTE_ASRC1M_IC_1_MASK           0x2
#define DSP1_R283_ASRC_MUTE_ASRC1M_IC_1_SHIFT          1
#define DSP1_R283_ASRC_MUTE_ASRC2M_IC_1_MASK           0x4
#define DSP1_R283_ASRC_MUTE_ASRC2M_IC_1_SHIFT          2
#define DSP1_R283_ASRC_MUTE_ASRC3M_IC_1_MASK           0x8
#define DSP1_R283_ASRC_MUTE_ASRC3M_IC_1_SHIFT          3
#define DSP1_R283_ASRC_MUTE_ASRC4M_IC_1_MASK           0x10
#define DSP1_R283_ASRC_MUTE_ASRC4M_IC_1_SHIFT          4
#define DSP1_R283_ASRC_MUTE_ASRC5M_IC_1_MASK           0x20
#define DSP1_R283_ASRC_MUTE_ASRC5M_IC_1_SHIFT          5
#define DSP1_R283_ASRC_MUTE_ASRC6M_IC_1_MASK           0x40
#define DSP1_R283_ASRC_MUTE_ASRC6M_IC_1_SHIFT          6
#define DSP1_R283_ASRC_MUTE_ASRC7M_IC_1_MASK           0x80
#define DSP1_R283_ASRC_MUTE_ASRC7M_IC_1_SHIFT          7
#define DSP1_R283_ASRC_MUTE_ASRC_RAMP0_IC_1_MASK       0x100
#define DSP1_R283_ASRC_MUTE_ASRC_RAMP0_IC_1_SHIFT      8
#define DSP1_R283_ASRC_MUTE_ASRC_RAMP1_IC_1_MASK       0x200
#define DSP1_R283_ASRC_MUTE_ASRC_RAMP1_IC_1_SHIFT      9
#define DSP1_R283_ASRC_MUTE_LOCKMUTE_IC_1_MASK         0x400
#define DSP1_R283_ASRC_MUTE_LOCKMUTE_IC_1_SHIFT        10

/* ASRC0_RATIO (IC 1) */
#define DSP1_R284_ASRC0_RATIO_ASRC_RATIO_IC_1          0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R284_ASRC0_RATIO_ASRC_RATIO_IC_1_MASK     0xFFFF
#define DSP1_R284_ASRC0_RATIO_ASRC_RATIO_IC_1_SHIFT    0

/* ASRC1_RATIO (IC 1) */
#define DSP1_R285_ASRC1_RATIO_ASRC_RATIO_IC_1          0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R285_ASRC1_RATIO_ASRC_RATIO_IC_1_MASK     0xFFFF
#define DSP1_R285_ASRC1_RATIO_ASRC_RATIO_IC_1_SHIFT    0

/* ASRC2_RATIO (IC 1) */
#define DSP1_R286_ASRC2_RATIO_ASRC_RATIO_IC_1          0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R286_ASRC2_RATIO_ASRC_RATIO_IC_1_MASK     0xFFFF
#define DSP1_R286_ASRC2_RATIO_ASRC_RATIO_IC_1_SHIFT    0

/* ASRC3_RATIO (IC 1) */
#define DSP1_R287_ASRC3_RATIO_ASRC_RATIO_IC_1          0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R287_ASRC3_RATIO_ASRC_RATIO_IC_1_MASK     0xFFFF
#define DSP1_R287_ASRC3_RATIO_ASRC_RATIO_IC_1_SHIFT    0

/* ASRC4_RATIO (IC 1) */
#define DSP1_R288_ASRC4_RATIO_ASRC_RATIO_IC_1          0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R288_ASRC4_RATIO_ASRC_RATIO_IC_1_MASK     0xFFFF
#define DSP1_R288_ASRC4_RATIO_ASRC_RATIO_IC_1_SHIFT    0

/* ASRC5_RATIO (IC 1) */
#define DSP1_R289_ASRC5_RATIO_ASRC_RATIO_IC_1          0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R289_ASRC5_RATIO_ASRC_RATIO_IC_1_MASK     0xFFFF
#define DSP1_R289_ASRC5_RATIO_ASRC_RATIO_IC_1_SHIFT    0

/* ASRC6_RATIO (IC 1) */
#define DSP1_R290_ASRC6_RATIO_ASRC_RATIO_IC_1          0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R290_ASRC6_RATIO_ASRC_RATIO_IC_1_MASK     0xFFFF
#define DSP1_R290_ASRC6_RATIO_ASRC_RATIO_IC_1_SHIFT    0

/* ASRC7_RATIO (IC 1) */
#define DSP1_R291_ASRC7_RATIO_ASRC_RATIO_IC_1          0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R291_ASRC7_RATIO_ASRC_RATIO_IC_1_MASK     0xFFFF
#define DSP1_R291_ASRC7_RATIO_ASRC_RATIO_IC_1_SHIFT    0

/* ASRC_RAMPMAX_OVR (IC 1) */
#define DSP1_R292_ASRC_RAMPMAX_OVR_OVR_RAMPMAX_VALUE_IC_1 0x7FF /* 11111111111b	[10:0] */
#define DSP1_R292_ASRC_RAMPMAX_OVR_OVERRIDE_IC_1       0x0    /* 0b	[11] */
#define DSP1_R292_ASRC_RAMPMAX_OVR_OVR_RAMPMAX_VALUE_IC_1_MASK 0x7FF
#define DSP1_R292_ASRC_RAMPMAX_OVR_OVR_RAMPMAX_VALUE_IC_1_SHIFT 0
#define DSP1_R292_ASRC_RAMPMAX_OVR_OVERRIDE_IC_1_MASK  0x800
#define DSP1_R292_ASRC_RAMPMAX_OVR_OVERRIDE_IC_1_SHIFT 11

/* ASRC0_RAMPMAX (IC 1) */
#define DSP1_R293_ASRC0_RAMPMAX_RAMPMAX_VALUE_IC_1     0x7FF  /* 11111111111b	[10:0] */
#define DSP1_R293_ASRC0_RAMPMAX_RAMPMAX_VALUE_IC_1_MASK 0x7FF
#define DSP1_R293_ASRC0_RAMPMAX_RAMPMAX_VALUE_IC_1_SHIFT 0

/* ASRC1_RAMPMAX (IC 1) */
#define DSP1_R294_ASRC1_RAMPMAX_RAMPMAX_VALUE_IC_1     0x7FF  /* 11111111111b	[10:0] */
#define DSP1_R294_ASRC1_RAMPMAX_RAMPMAX_VALUE_IC_1_MASK 0x7FF
#define DSP1_R294_ASRC1_RAMPMAX_RAMPMAX_VALUE_IC_1_SHIFT 0

/* ASRC2_RAMPMAX (IC 1) */
#define DSP1_R295_ASRC2_RAMPMAX_RAMPMAX_VALUE_IC_1     0x7FF  /* 11111111111b	[10:0] */
#define DSP1_R295_ASRC2_RAMPMAX_RAMPMAX_VALUE_IC_1_MASK 0x7FF
#define DSP1_R295_ASRC2_RAMPMAX_RAMPMAX_VALUE_IC_1_SHIFT 0

/* ASRC3_RAMPMAX (IC 1) */
#define DSP1_R296_ASRC3_RAMPMAX_RAMPMAX_VALUE_IC_1     0x7FF  /* 11111111111b	[10:0] */
#define DSP1_R296_ASRC3_RAMPMAX_RAMPMAX_VALUE_IC_1_MASK 0x7FF
#define DSP1_R296_ASRC3_RAMPMAX_RAMPMAX_VALUE_IC_1_SHIFT 0

/* ASRC4_RAMPMAX (IC 1) */
#define DSP1_R297_ASRC4_RAMPMAX_RAMPMAX_VALUE_IC_1     0x7FF  /* 11111111111b	[10:0] */
#define DSP1_R297_ASRC4_RAMPMAX_RAMPMAX_VALUE_IC_1_MASK 0x7FF
#define DSP1_R297_ASRC4_RAMPMAX_RAMPMAX_VALUE_IC_1_SHIFT 0

/* ASRC5_RAMPMAX (IC 1) */
#define DSP1_R298_ASRC5_RAMPMAX_RAMPMAX_VALUE_IC_1     0x7FF  /* 11111111111b	[10:0] */
#define DSP1_R298_ASRC5_RAMPMAX_RAMPMAX_VALUE_IC_1_MASK 0x7FF
#define DSP1_R298_ASRC5_RAMPMAX_RAMPMAX_VALUE_IC_1_SHIFT 0

/* ASRC6_RAMPMAX (IC 1) */
#define DSP1_R299_ASRC6_RAMPMAX_RAMPMAX_VALUE_IC_1     0x7FF  /* 11111111111b	[10:0] */
#define DSP1_R299_ASRC6_RAMPMAX_RAMPMAX_VALUE_IC_1_MASK 0x7FF
#define DSP1_R299_ASRC6_RAMPMAX_RAMPMAX_VALUE_IC_1_SHIFT 0

/* ASRC7_RAMPMAX (IC 1) */
#define DSP1_R300_ASRC7_RAMPMAX_RAMPMAX_VALUE_IC_1     0x7FF  /* 11111111111b	[10:0] */
#define DSP1_R300_ASRC7_RAMPMAX_RAMPMAX_VALUE_IC_1_MASK 0x7FF
#define DSP1_R300_ASRC7_RAMPMAX_RAMPMAX_VALUE_IC_1_SHIFT 0

/* ADC_READ0 (IC 1) */
#define DSP1_R301_ADC_READ0_ADC_VALUE_IC_1             0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R301_ADC_READ0_ADC_VALUE_IC_1_MASK        0xFFFF
#define DSP1_R301_ADC_READ0_ADC_VALUE_IC_1_SHIFT       0

/* ADC_READ1 (IC 1) */
#define DSP1_R302_ADC_READ1_ADC_VALUE_IC_1             0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R302_ADC_READ1_ADC_VALUE_IC_1_MASK        0xFFFF
#define DSP1_R302_ADC_READ1_ADC_VALUE_IC_1_SHIFT       0

/* ADC_READ2 (IC 1) */
#define DSP1_R303_ADC_READ2_ADC_VALUE_IC_1             0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R303_ADC_READ2_ADC_VALUE_IC_1_MASK        0xFFFF
#define DSP1_R303_ADC_READ2_ADC_VALUE_IC_1_SHIFT       0

/* ADC_READ3 (IC 1) */
#define DSP1_R304_ADC_READ3_ADC_VALUE_IC_1             0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R304_ADC_READ3_ADC_VALUE_IC_1_MASK        0xFFFF
#define DSP1_R304_ADC_READ3_ADC_VALUE_IC_1_SHIFT       0

/* ADC_READ4 (IC 1) */
#define DSP1_R305_ADC_READ4_ADC_VALUE_IC_1             0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R305_ADC_READ4_ADC_VALUE_IC_1_MASK        0xFFFF
#define DSP1_R305_ADC_READ4_ADC_VALUE_IC_1_SHIFT       0

/* ADC_READ5 (IC 1) */
#define DSP1_R306_ADC_READ5_ADC_VALUE_IC_1             0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R306_ADC_READ5_ADC_VALUE_IC_1_MASK        0xFFFF
#define DSP1_R306_ADC_READ5_ADC_VALUE_IC_1_SHIFT       0

/* MP0_MODE1 (IC 1) */
#define DSP1_R307_MP0_MODE1_MP_ENABLE1_IC_1            0x0    /* 0b	[0] */
#define DSP1_R307_MP0_MODE1_MP_MODE1_IC_1              0x0    /* 000b	[3:1] */
#define DSP1_R307_MP0_MODE1_DEBOUNCE_VALUE1_IC_1       0x0    /* 0000b	[7:4] */
#define DSP1_R307_MP0_MODE1_SS_SELECT1_IC_1            0x0    /* 000b	[10:8] */
#define DSP1_R307_MP0_MODE1_MP_ENABLE1_IC_1_MASK       0x1
#define DSP1_R307_MP0_MODE1_MP_ENABLE1_IC_1_SHIFT      0
#define DSP1_R307_MP0_MODE1_MP_MODE1_IC_1_MASK         0xE
#define DSP1_R307_MP0_MODE1_MP_MODE1_IC_1_SHIFT        1
#define DSP1_R307_MP0_MODE1_DEBOUNCE_VALUE1_IC_1_MASK  0xF0
#define DSP1_R307_MP0_MODE1_DEBOUNCE_VALUE1_IC_1_SHIFT 4
#define DSP1_R307_MP0_MODE1_SS_SELECT1_IC_1_MASK       0x700
#define DSP1_R307_MP0_MODE1_SS_SELECT1_IC_1_SHIFT      8

/* MP1_MODE1 (IC 1) */
#define DSP1_R308_MP1_MODE1_MP_ENABLE1_IC_1            0x0    /* 0b	[0] */
#define DSP1_R308_MP1_MODE1_MP_MODE1_IC_1              0x0    /* 000b	[3:1] */
#define DSP1_R308_MP1_MODE1_DEBOUNCE_VALUE1_IC_1       0x0    /* 0000b	[7:4] */
#define DSP1_R308_MP1_MODE1_SS_SELECT1_IC_1            0x0    /* 000b	[10:8] */
#define DSP1_R308_MP1_MODE1_MP_ENABLE1_IC_1_MASK       0x1
#define DSP1_R308_MP1_MODE1_MP_ENABLE1_IC_1_SHIFT      0
#define DSP1_R308_MP1_MODE1_MP_MODE1_IC_1_MASK         0xE
#define DSP1_R308_MP1_MODE1_MP_MODE1_IC_1_SHIFT        1
#define DSP1_R308_MP1_MODE1_DEBOUNCE_VALUE1_IC_1_MASK  0xF0
#define DSP1_R308_MP1_MODE1_DEBOUNCE_VALUE1_IC_1_SHIFT 4
#define DSP1_R308_MP1_MODE1_SS_SELECT1_IC_1_MASK       0x700
#define DSP1_R308_MP1_MODE1_SS_SELECT1_IC_1_SHIFT      8

/* MP2_MODE1 (IC 1) */
#define DSP1_R309_MP2_MODE1_MP_ENABLE1_IC_1            0x0    /* 0b	[0] */
#define DSP1_R309_MP2_MODE1_MP_MODE1_IC_1              0x0    /* 000b	[3:1] */
#define DSP1_R309_MP2_MODE1_DEBOUNCE_VALUE1_IC_1       0x0    /* 0000b	[7:4] */
#define DSP1_R309_MP2_MODE1_SS_SELECT1_IC_1            0x0    /* 000b	[10:8] */
#define DSP1_R309_MP2_MODE1_MP_ENABLE1_IC_1_MASK       0x1
#define DSP1_R309_MP2_MODE1_MP_ENABLE1_IC_1_SHIFT      0
#define DSP1_R309_MP2_MODE1_MP_MODE1_IC_1_MASK         0xE
#define DSP1_R309_MP2_MODE1_MP_MODE1_IC_1_SHIFT        1
#define DSP1_R309_MP2_MODE1_DEBOUNCE_VALUE1_IC_1_MASK  0xF0
#define DSP1_R309_MP2_MODE1_DEBOUNCE_VALUE1_IC_1_SHIFT 4
#define DSP1_R309_MP2_MODE1_SS_SELECT1_IC_1_MASK       0x700
#define DSP1_R309_MP2_MODE1_SS_SELECT1_IC_1_SHIFT      8

/* MP3_MODE1 (IC 1) */
#define DSP1_R310_MP3_MODE1_MP_ENABLE1_IC_1            0x0    /* 0b	[0] */
#define DSP1_R310_MP3_MODE1_MP_MODE1_IC_1              0x0    /* 000b	[3:1] */
#define DSP1_R310_MP3_MODE1_DEBOUNCE_VALUE1_IC_1       0x0    /* 0000b	[7:4] */
#define DSP1_R310_MP3_MODE1_SS_SELECT1_IC_1            0x0    /* 000b	[10:8] */
#define DSP1_R310_MP3_MODE1_MP_ENABLE1_IC_1_MASK       0x1
#define DSP1_R310_MP3_MODE1_MP_ENABLE1_IC_1_SHIFT      0
#define DSP1_R310_MP3_MODE1_MP_MODE1_IC_1_MASK         0xE
#define DSP1_R310_MP3_MODE1_MP_MODE1_IC_1_SHIFT        1
#define DSP1_R310_MP3_MODE1_DEBOUNCE_VALUE1_IC_1_MASK  0xF0
#define DSP1_R310_MP3_MODE1_DEBOUNCE_VALUE1_IC_1_SHIFT 4
#define DSP1_R310_MP3_MODE1_SS_SELECT1_IC_1_MASK       0x700
#define DSP1_R310_MP3_MODE1_SS_SELECT1_IC_1_SHIFT      8

/* MP4_MODE1 (IC 1) */
#define DSP1_R311_MP4_MODE1_MP_ENABLE1_IC_1            0x0    /* 0b	[0] */
#define DSP1_R311_MP4_MODE1_MP_MODE1_IC_1              0x0    /* 000b	[3:1] */
#define DSP1_R311_MP4_MODE1_DEBOUNCE_VALUE1_IC_1       0x0    /* 0000b	[7:4] */
#define DSP1_R311_MP4_MODE1_SS_SELECT1_IC_1            0x0    /* 000b	[10:8] */
#define DSP1_R311_MP4_MODE1_MP_ENABLE1_IC_1_MASK       0x1
#define DSP1_R311_MP4_MODE1_MP_ENABLE1_IC_1_SHIFT      0
#define DSP1_R311_MP4_MODE1_MP_MODE1_IC_1_MASK         0xE
#define DSP1_R311_MP4_MODE1_MP_MODE1_IC_1_SHIFT        1
#define DSP1_R311_MP4_MODE1_DEBOUNCE_VALUE1_IC_1_MASK  0xF0
#define DSP1_R311_MP4_MODE1_DEBOUNCE_VALUE1_IC_1_SHIFT 4
#define DSP1_R311_MP4_MODE1_SS_SELECT1_IC_1_MASK       0x700
#define DSP1_R311_MP4_MODE1_SS_SELECT1_IC_1_SHIFT      8

/* MP5_MODE1 (IC 1) */
#define DSP1_R312_MP5_MODE1_MP_ENABLE1_IC_1            0x0    /* 0b	[0] */
#define DSP1_R312_MP5_MODE1_MP_MODE1_IC_1              0x0    /* 000b	[3:1] */
#define DSP1_R312_MP5_MODE1_DEBOUNCE_VALUE1_IC_1       0x0    /* 0000b	[7:4] */
#define DSP1_R312_MP5_MODE1_SS_SELECT1_IC_1            0x0    /* 000b	[10:8] */
#define DSP1_R312_MP5_MODE1_MP_ENABLE1_IC_1_MASK       0x1
#define DSP1_R312_MP5_MODE1_MP_ENABLE1_IC_1_SHIFT      0
#define DSP1_R312_MP5_MODE1_MP_MODE1_IC_1_MASK         0xE
#define DSP1_R312_MP5_MODE1_MP_MODE1_IC_1_SHIFT        1
#define DSP1_R312_MP5_MODE1_DEBOUNCE_VALUE1_IC_1_MASK  0xF0
#define DSP1_R312_MP5_MODE1_DEBOUNCE_VALUE1_IC_1_SHIFT 4
#define DSP1_R312_MP5_MODE1_SS_SELECT1_IC_1_MASK       0x700
#define DSP1_R312_MP5_MODE1_SS_SELECT1_IC_1_SHIFT      8

/* MP6_MODE1 (IC 1) */
#define DSP1_R313_MP6_MODE1_MP_ENABLE1_IC_1            0x0    /* 0b	[0] */
#define DSP1_R313_MP6_MODE1_MP_MODE1_IC_1              0x0    /* 000b	[3:1] */
#define DSP1_R313_MP6_MODE1_DEBOUNCE_VALUE1_IC_1       0x0    /* 0000b	[7:4] */
#define DSP1_R313_MP6_MODE1_SS_SELECT1_IC_1            0x0    /* 000b	[10:8] */
#define DSP1_R313_MP6_MODE1_MP_ENABLE1_IC_1_MASK       0x1
#define DSP1_R313_MP6_MODE1_MP_ENABLE1_IC_1_SHIFT      0
#define DSP1_R313_MP6_MODE1_MP_MODE1_IC_1_MASK         0xE
#define DSP1_R313_MP6_MODE1_MP_MODE1_IC_1_SHIFT        1
#define DSP1_R313_MP6_MODE1_DEBOUNCE_VALUE1_IC_1_MASK  0xF0
#define DSP1_R313_MP6_MODE1_DEBOUNCE_VALUE1_IC_1_SHIFT 4
#define DSP1_R313_MP6_MODE1_SS_SELECT1_IC_1_MASK       0x700
#define DSP1_R313_MP6_MODE1_SS_SELECT1_IC_1_SHIFT      8

/* MP7_MODE1 (IC 1) */
#define DSP1_R314_MP7_MODE1_MP_ENABLE1_IC_1            0x0    /* 0b	[0] */
#define DSP1_R314_MP7_MODE1_MP_MODE1_IC_1              0x0    /* 000b	[3:1] */
#define DSP1_R314_MP7_MODE1_DEBOUNCE_VALUE1_IC_1       0x0    /* 0000b	[7:4] */
#define DSP1_R314_MP7_MODE1_SS_SELECT1_IC_1            0x0    /* 000b	[10:8] */
#define DSP1_R314_MP7_MODE1_MP_ENABLE1_IC_1_MASK       0x1
#define DSP1_R314_MP7_MODE1_MP_ENABLE1_IC_1_SHIFT      0
#define DSP1_R314_MP7_MODE1_MP_MODE1_IC_1_MASK         0xE
#define DSP1_R314_MP7_MODE1_MP_MODE1_IC_1_SHIFT        1
#define DSP1_R314_MP7_MODE1_DEBOUNCE_VALUE1_IC_1_MASK  0xF0
#define DSP1_R314_MP7_MODE1_DEBOUNCE_VALUE1_IC_1_SHIFT 4
#define DSP1_R314_MP7_MODE1_SS_SELECT1_IC_1_MASK       0x700
#define DSP1_R314_MP7_MODE1_SS_SELECT1_IC_1_SHIFT      8

/* MP8_MODE1 (IC 1) */
#define DSP1_R315_MP8_MODE1_MP_ENABLE1_IC_1            0x0    /* 0b	[0] */
#define DSP1_R315_MP8_MODE1_MP_MODE1_IC_1              0x0    /* 000b	[3:1] */
#define DSP1_R315_MP8_MODE1_DEBOUNCE_VALUE1_IC_1       0x0    /* 0000b	[7:4] */
#define DSP1_R315_MP8_MODE1_SS_SELECT1_IC_1            0x0    /* 000b	[10:8] */
#define DSP1_R315_MP8_MODE1_MP_ENABLE1_IC_1_MASK       0x1
#define DSP1_R315_MP8_MODE1_MP_ENABLE1_IC_1_SHIFT      0
#define DSP1_R315_MP8_MODE1_MP_MODE1_IC_1_MASK         0xE
#define DSP1_R315_MP8_MODE1_MP_MODE1_IC_1_SHIFT        1
#define DSP1_R315_MP8_MODE1_DEBOUNCE_VALUE1_IC_1_MASK  0xF0
#define DSP1_R315_MP8_MODE1_DEBOUNCE_VALUE1_IC_1_SHIFT 4
#define DSP1_R315_MP8_MODE1_SS_SELECT1_IC_1_MASK       0x700
#define DSP1_R315_MP8_MODE1_SS_SELECT1_IC_1_SHIFT      8

/* MP9_MODE1 (IC 1) */
#define DSP1_R316_MP9_MODE1_MP_ENABLE1_IC_1            0x0    /* 0b	[0] */
#define DSP1_R316_MP9_MODE1_MP_MODE1_IC_1              0x0    /* 000b	[3:1] */
#define DSP1_R316_MP9_MODE1_DEBOUNCE_VALUE1_IC_1       0x0    /* 0000b	[7:4] */
#define DSP1_R316_MP9_MODE1_SS_SELECT1_IC_1            0x0    /* 000b	[10:8] */
#define DSP1_R316_MP9_MODE1_MP_ENABLE1_IC_1_MASK       0x1
#define DSP1_R316_MP9_MODE1_MP_ENABLE1_IC_1_SHIFT      0
#define DSP1_R316_MP9_MODE1_MP_MODE1_IC_1_MASK         0xE
#define DSP1_R316_MP9_MODE1_MP_MODE1_IC_1_SHIFT        1
#define DSP1_R316_MP9_MODE1_DEBOUNCE_VALUE1_IC_1_MASK  0xF0
#define DSP1_R316_MP9_MODE1_DEBOUNCE_VALUE1_IC_1_SHIFT 4
#define DSP1_R316_MP9_MODE1_SS_SELECT1_IC_1_MASK       0x700
#define DSP1_R316_MP9_MODE1_SS_SELECT1_IC_1_SHIFT      8

/* MP10_MODE1 (IC 1) */
#define DSP1_R317_MP10_MODE1_MP_ENABLE1_IC_1           0x0    /* 0b	[0] */
#define DSP1_R317_MP10_MODE1_MP_MODE1_IC_1             0x0    /* 000b	[3:1] */
#define DSP1_R317_MP10_MODE1_DEBOUNCE_VALUE1_IC_1      0x0    /* 0000b	[7:4] */
#define DSP1_R317_MP10_MODE1_SS_SELECT1_IC_1           0x0    /* 000b	[10:8] */
#define DSP1_R317_MP10_MODE1_MP_ENABLE1_IC_1_MASK      0x1
#define DSP1_R317_MP10_MODE1_MP_ENABLE1_IC_1_SHIFT     0
#define DSP1_R317_MP10_MODE1_MP_MODE1_IC_1_MASK        0xE
#define DSP1_R317_MP10_MODE1_MP_MODE1_IC_1_SHIFT       1
#define DSP1_R317_MP10_MODE1_DEBOUNCE_VALUE1_IC_1_MASK 0xF0
#define DSP1_R317_MP10_MODE1_DEBOUNCE_VALUE1_IC_1_SHIFT 4
#define DSP1_R317_MP10_MODE1_SS_SELECT1_IC_1_MASK      0x700
#define DSP1_R317_MP10_MODE1_SS_SELECT1_IC_1_SHIFT     8

/* MP11_MODE1 (IC 1) */
#define DSP1_R318_MP11_MODE1_MP_ENABLE1_IC_1           0x0    /* 0b	[0] */
#define DSP1_R318_MP11_MODE1_MP_MODE1_IC_1             0x0    /* 000b	[3:1] */
#define DSP1_R318_MP11_MODE1_DEBOUNCE_VALUE1_IC_1      0x0    /* 0000b	[7:4] */
#define DSP1_R318_MP11_MODE1_SS_SELECT1_IC_1           0x0    /* 000b	[10:8] */
#define DSP1_R318_MP11_MODE1_MP_ENABLE1_IC_1_MASK      0x1
#define DSP1_R318_MP11_MODE1_MP_ENABLE1_IC_1_SHIFT     0
#define DSP1_R318_MP11_MODE1_MP_MODE1_IC_1_MASK        0xE
#define DSP1_R318_MP11_MODE1_MP_MODE1_IC_1_SHIFT       1
#define DSP1_R318_MP11_MODE1_DEBOUNCE_VALUE1_IC_1_MASK 0xF0
#define DSP1_R318_MP11_MODE1_DEBOUNCE_VALUE1_IC_1_SHIFT 4
#define DSP1_R318_MP11_MODE1_SS_SELECT1_IC_1_MASK      0x700
#define DSP1_R318_MP11_MODE1_SS_SELECT1_IC_1_SHIFT     8

/* MP0_WRITE1 (IC 1) */
#define DSP1_R319_MP0_WRITE1_MP_REG_WRITE1_IC_1        0x0    /* 0b	[0] */
#define DSP1_R319_MP0_WRITE1_MP_REG_WRITE1_IC_1_MASK   0x1
#define DSP1_R319_MP0_WRITE1_MP_REG_WRITE1_IC_1_SHIFT  0

/* MP1_WRITE1 (IC 1) */
#define DSP1_R320_MP1_WRITE1_MP_REG_WRITE1_IC_1        0x0    /* 0b	[0] */
#define DSP1_R320_MP1_WRITE1_MP_REG_WRITE1_IC_1_MASK   0x1
#define DSP1_R320_MP1_WRITE1_MP_REG_WRITE1_IC_1_SHIFT  0

/* MP2_WRITE1 (IC 1) */
#define DSP1_R321_MP2_WRITE1_MP_REG_WRITE1_IC_1        0x0    /* 0b	[0] */
#define DSP1_R321_MP2_WRITE1_MP_REG_WRITE1_IC_1_MASK   0x1
#define DSP1_R321_MP2_WRITE1_MP_REG_WRITE1_IC_1_SHIFT  0

/* MP3_WRITE1 (IC 1) */
#define DSP1_R322_MP3_WRITE1_MP_REG_WRITE1_IC_1        0x0    /* 0b	[0] */
#define DSP1_R322_MP3_WRITE1_MP_REG_WRITE1_IC_1_MASK   0x1
#define DSP1_R322_MP3_WRITE1_MP_REG_WRITE1_IC_1_SHIFT  0

/* MP4_WRITE1 (IC 1) */
#define DSP1_R323_MP4_WRITE1_MP_REG_WRITE1_IC_1        0x0    /* 0b	[0] */
#define DSP1_R323_MP4_WRITE1_MP_REG_WRITE1_IC_1_MASK   0x1
#define DSP1_R323_MP4_WRITE1_MP_REG_WRITE1_IC_1_SHIFT  0

/* MP5_WRITE1 (IC 1) */
#define DSP1_R324_MP5_WRITE1_MP_REG_WRITE1_IC_1        0x0    /* 0b	[0] */
#define DSP1_R324_MP5_WRITE1_MP_REG_WRITE1_IC_1_MASK   0x1
#define DSP1_R324_MP5_WRITE1_MP_REG_WRITE1_IC_1_SHIFT  0

/* MP6_WRITE1 (IC 1) */
#define DSP1_R325_MP6_WRITE1_MP_REG_WRITE1_IC_1        0x0    /* 0b	[0] */
#define DSP1_R325_MP6_WRITE1_MP_REG_WRITE1_IC_1_MASK   0x1
#define DSP1_R325_MP6_WRITE1_MP_REG_WRITE1_IC_1_SHIFT  0

/* MP7_WRITE1 (IC 1) */
#define DSP1_R326_MP7_WRITE1_MP_REG_WRITE1_IC_1        0x0    /* 0b	[0] */
#define DSP1_R326_MP7_WRITE1_MP_REG_WRITE1_IC_1_MASK   0x1
#define DSP1_R326_MP7_WRITE1_MP_REG_WRITE1_IC_1_SHIFT  0

/* MP8_WRITE1 (IC 1) */
#define DSP1_R327_MP8_WRITE1_MP_REG_WRITE1_IC_1        0x0    /* 0b	[0] */
#define DSP1_R327_MP8_WRITE1_MP_REG_WRITE1_IC_1_MASK   0x1
#define DSP1_R327_MP8_WRITE1_MP_REG_WRITE1_IC_1_SHIFT  0

/* MP9_WRITE1 (IC 1) */
#define DSP1_R328_MP9_WRITE1_MP_REG_WRITE1_IC_1        0x0    /* 0b	[0] */
#define DSP1_R328_MP9_WRITE1_MP_REG_WRITE1_IC_1_MASK   0x1
#define DSP1_R328_MP9_WRITE1_MP_REG_WRITE1_IC_1_SHIFT  0

/* MP10_WRITE1 (IC 1) */
#define DSP1_R329_MP10_WRITE1_MP_REG_WRITE1_IC_1       0x0    /* 0b	[0] */
#define DSP1_R329_MP10_WRITE1_MP_REG_WRITE1_IC_1_MASK  0x1
#define DSP1_R329_MP10_WRITE1_MP_REG_WRITE1_IC_1_SHIFT 0

/* MP11_WRITE1 (IC 1) */
#define DSP1_R330_MP11_WRITE1_MP_REG_WRITE1_IC_1       0x0    /* 0b	[0] */
#define DSP1_R330_MP11_WRITE1_MP_REG_WRITE1_IC_1_MASK  0x1
#define DSP1_R330_MP11_WRITE1_MP_REG_WRITE1_IC_1_SHIFT 0

/* MP0_READ1 (IC 1) */
#define DSP1_R331_MP0_READ1_MP_REG_READ1_IC_1          0x0    /* 0b	[0] */
#define DSP1_R331_MP0_READ1_MP_REG_READ1_IC_1_MASK     0x1
#define DSP1_R331_MP0_READ1_MP_REG_READ1_IC_1_SHIFT    0

/* MP1_READ1 (IC 1) */
#define DSP1_R332_MP1_READ1_MP_REG_READ1_IC_1          0x0    /* 0b	[0] */
#define DSP1_R332_MP1_READ1_MP_REG_READ1_IC_1_MASK     0x1
#define DSP1_R332_MP1_READ1_MP_REG_READ1_IC_1_SHIFT    0

/* MP2_READ1 (IC 1) */
#define DSP1_R333_MP2_READ1_MP_REG_READ1_IC_1          0x0    /* 0b	[0] */
#define DSP1_R333_MP2_READ1_MP_REG_READ1_IC_1_MASK     0x1
#define DSP1_R333_MP2_READ1_MP_REG_READ1_IC_1_SHIFT    0

/* MP3_READ1 (IC 1) */
#define DSP1_R334_MP3_READ1_MP_REG_READ1_IC_1          0x0    /* 0b	[0] */
#define DSP1_R334_MP3_READ1_MP_REG_READ1_IC_1_MASK     0x1
#define DSP1_R334_MP3_READ1_MP_REG_READ1_IC_1_SHIFT    0

/* MP4_READ1 (IC 1) */
#define DSP1_R335_MP4_READ1_MP_REG_READ1_IC_1          0x0    /* 0b	[0] */
#define DSP1_R335_MP4_READ1_MP_REG_READ1_IC_1_MASK     0x1
#define DSP1_R335_MP4_READ1_MP_REG_READ1_IC_1_SHIFT    0

/* MP5_READ1 (IC 1) */
#define DSP1_R336_MP5_READ1_MP_REG_READ1_IC_1          0x0    /* 0b	[0] */
#define DSP1_R336_MP5_READ1_MP_REG_READ1_IC_1_MASK     0x1
#define DSP1_R336_MP5_READ1_MP_REG_READ1_IC_1_SHIFT    0

/* MP6_READ1 (IC 1) */
#define DSP1_R337_MP6_READ1_MP_REG_READ1_IC_1          0x0    /* 0b	[0] */
#define DSP1_R337_MP6_READ1_MP_REG_READ1_IC_1_MASK     0x1
#define DSP1_R337_MP6_READ1_MP_REG_READ1_IC_1_SHIFT    0

/* MP7_READ1 (IC 1) */
#define DSP1_R338_MP7_READ1_MP_REG_READ1_IC_1          0x0    /* 0b	[0] */
#define DSP1_R338_MP7_READ1_MP_REG_READ1_IC_1_MASK     0x1
#define DSP1_R338_MP7_READ1_MP_REG_READ1_IC_1_SHIFT    0

/* MP8_READ1 (IC 1) */
#define DSP1_R339_MP8_READ1_MP_REG_READ1_IC_1          0x0    /* 0b	[0] */
#define DSP1_R339_MP8_READ1_MP_REG_READ1_IC_1_MASK     0x1
#define DSP1_R339_MP8_READ1_MP_REG_READ1_IC_1_SHIFT    0

/* MP9_READ1 (IC 1) */
#define DSP1_R340_MP9_READ1_MP_REG_READ1_IC_1          0x0    /* 0b	[0] */
#define DSP1_R340_MP9_READ1_MP_REG_READ1_IC_1_MASK     0x1
#define DSP1_R340_MP9_READ1_MP_REG_READ1_IC_1_SHIFT    0

/* MP10_READ1 (IC 1) */
#define DSP1_R341_MP10_READ1_MP_REG_READ1_IC_1         0x0    /* 0b	[0] */
#define DSP1_R341_MP10_READ1_MP_REG_READ1_IC_1_MASK    0x1
#define DSP1_R341_MP10_READ1_MP_REG_READ1_IC_1_SHIFT   0

/* MP11_READ1 (IC 1) */
#define DSP1_R342_MP11_READ1_MP_REG_READ1_IC_1         0x0    /* 0b	[0] */
#define DSP1_R342_MP11_READ1_MP_REG_READ1_IC_1_MASK    0x1
#define DSP1_R342_MP11_READ1_MP_REG_READ1_IC_1_SHIFT   0

/* SPDIF_LOCK_DET (IC 1) */
#define DSP1_R343_SPDIF_LOCK_DET_LOCK_IC_1             0x0    /* 0b	[0] */
#define DSP1_R343_SPDIF_LOCK_DET_LOCK_IC_1_MASK        0x1
#define DSP1_R343_SPDIF_LOCK_DET_LOCK_IC_1_SHIFT       0

/* SPDIF_RX_CTRL (IC 1) */
#define DSP1_R344_SPDIF_RX_CTRL_RX_LENGTHCTRL_IC_1     0x0    /* 00b	[1:0] */
#define DSP1_R344_SPDIF_RX_CTRL_FSOUTSTRENGTH_IC_1     0x0    /* 0b	[2] */
#define DSP1_R344_SPDIF_RX_CTRL_FASTLOCK_IC_1          0x0    /* 0b	[3] */
#define DSP1_R344_SPDIF_RX_CTRL_RX_LENGTHCTRL_IC_1_MASK 0x3
#define DSP1_R344_SPDIF_RX_CTRL_RX_LENGTHCTRL_IC_1_SHIFT 0
#define DSP1_R344_SPDIF_RX_CTRL_FSOUTSTRENGTH_IC_1_MASK 0x4
#define DSP1_R344_SPDIF_RX_CTRL_FSOUTSTRENGTH_IC_1_SHIFT 2
#define DSP1_R344_SPDIF_RX_CTRL_FASTLOCK_IC_1_MASK     0x8
#define DSP1_R344_SPDIF_RX_CTRL_FASTLOCK_IC_1_SHIFT    3

/* SPDIF_RX_DECODE (IC 1) */
#define DSP1_R345_SPDIF_RX_DECODE_AUDIO_TYPE_IC_1      0x0    /* 0b	[0] */
#define DSP1_R345_SPDIF_RX_DECODE_COMPR_TYPE_IC_1      0x0    /* 0b	[1] */
#define DSP1_R345_SPDIF_RX_DECODE_RX_WORDLENGTH_L_IC_1 0x2    /* 0010b	[5:2] */
#define DSP1_R345_SPDIF_RX_DECODE_RX_WORDLENGTH_R_IC_1 0x2    /* 0010b	[9:6] */
#define DSP1_R345_SPDIF_RX_DECODE_AUDIO_TYPE_IC_1_MASK 0x1
#define DSP1_R345_SPDIF_RX_DECODE_AUDIO_TYPE_IC_1_SHIFT 0
#define DSP1_R345_SPDIF_RX_DECODE_COMPR_TYPE_IC_1_MASK 0x2
#define DSP1_R345_SPDIF_RX_DECODE_COMPR_TYPE_IC_1_SHIFT 1
#define DSP1_R345_SPDIF_RX_DECODE_RX_WORDLENGTH_L_IC_1_MASK 0x3C
#define DSP1_R345_SPDIF_RX_DECODE_RX_WORDLENGTH_L_IC_1_SHIFT 2
#define DSP1_R345_SPDIF_RX_DECODE_RX_WORDLENGTH_R_IC_1_MASK 0x3C0
#define DSP1_R345_SPDIF_RX_DECODE_RX_WORDLENGTH_R_IC_1_SHIFT 6

/* SPDIF_RX_COMPRMODE (IC 1) */
#define DSP1_R346_SPDIF_RX_COMPRMODE_COMPR_MODE_IC_1   0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R346_SPDIF_RX_COMPRMODE_COMPR_MODE_IC_1_MASK 0xFFFF
#define DSP1_R346_SPDIF_RX_COMPRMODE_COMPR_MODE_IC_1_SHIFT 0

/* SPDIF_RESTART (IC 1) */
#define DSP1_R347_SPDIF_RESTART_RESTART_AUDIO_IC_1     0x0    /* 0b	[0] */
#define DSP1_R347_SPDIF_RESTART_RESTART_AUDIO_IC_1_MASK 0x1
#define DSP1_R347_SPDIF_RESTART_RESTART_AUDIO_IC_1_SHIFT 0

/* SPDIF_LOSS_OF_LOCK (IC 1) */
#define DSP1_R348_SPDIF_LOSS_OF_LOCK_LOSS_OF_LOCK_IC_1 0x0    /* 0b	[0] */
#define DSP1_R348_SPDIF_LOSS_OF_LOCK_LOSS_OF_LOCK_IC_1_MASK 0x1
#define DSP1_R348_SPDIF_LOSS_OF_LOCK_LOSS_OF_LOCK_IC_1_SHIFT 0

/* SPDIF_RX_MCLKSPEED (IC 1) */
#define DSP1_R349_SPDIF_RX_MCLKSPEED_RX_MCLKSPEED_IC_1 0x1    /* 1b	[0] */
#define DSP1_R349_SPDIF_RX_MCLKSPEED_RX_MCLKSPEED_IC_1_MASK 0x1
#define DSP1_R349_SPDIF_RX_MCLKSPEED_RX_MCLKSPEED_IC_1_SHIFT 0

/* SPDIF_TX_MCLKSPEED (IC 1) */
#define DSP1_R350_SPDIF_TX_MCLKSPEED_TX_MCLKSPEED_IC_1 0x1    /* 1b	[0] */
#define DSP1_R350_SPDIF_TX_MCLKSPEED_TX_MCLKSPEED_IC_1_MASK 0x1
#define DSP1_R350_SPDIF_TX_MCLKSPEED_TX_MCLKSPEED_IC_1_SHIFT 0

/* SPDIF_AUX_EN (IC 1) */
#define DSP1_R351_SPDIF_AUX_EN_TDMOUT_IC_1             0x0    /* 0000b	[3:0] */
#define DSP1_R351_SPDIF_AUX_EN_TDMOUT_CLK_IC_1         0x0    /* 0b	[4] */
#define DSP1_R351_SPDIF_AUX_EN_TDMOUT_IC_1_MASK        0xF
#define DSP1_R351_SPDIF_AUX_EN_TDMOUT_IC_1_SHIFT       0
#define DSP1_R351_SPDIF_AUX_EN_TDMOUT_CLK_IC_1_MASK    0x10
#define DSP1_R351_SPDIF_AUX_EN_TDMOUT_CLK_IC_1_SHIFT   4

/* SPDIF_RX_AUXBIT_READY (IC 1) */
#define DSP1_R352_SPDIF_RX_AUXBIT_READY_AUXBITS_READY_IC_1 0x0 /* 0b	[0] */
#define DSP1_R352_SPDIF_RX_AUXBIT_READY_AUXBITS_READY_IC_1_MASK 0x1
#define DSP1_R352_SPDIF_RX_AUXBIT_READY_AUXBITS_READY_IC_1_SHIFT 0

/* SPDIF_RX_CS_LEFT_0 (IC 1) */
#define DSP1_R353_SPDIF_RX_CS_LEFT_0_SPDIF_RX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R353_SPDIF_RX_CS_LEFT_0_SPDIF_RX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R353_SPDIF_RX_CS_LEFT_0_SPDIF_RX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_CS_LEFT_1 (IC 1) */
#define DSP1_R354_SPDIF_RX_CS_LEFT_1_SPDIF_RX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R354_SPDIF_RX_CS_LEFT_1_SPDIF_RX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R354_SPDIF_RX_CS_LEFT_1_SPDIF_RX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_CS_LEFT_2 (IC 1) */
#define DSP1_R355_SPDIF_RX_CS_LEFT_2_SPDIF_RX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R355_SPDIF_RX_CS_LEFT_2_SPDIF_RX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R355_SPDIF_RX_CS_LEFT_2_SPDIF_RX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_CS_LEFT_3 (IC 1) */
#define DSP1_R356_SPDIF_RX_CS_LEFT_3_SPDIF_RX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R356_SPDIF_RX_CS_LEFT_3_SPDIF_RX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R356_SPDIF_RX_CS_LEFT_3_SPDIF_RX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_CS_LEFT_4 (IC 1) */
#define DSP1_R357_SPDIF_RX_CS_LEFT_4_SPDIF_RX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R357_SPDIF_RX_CS_LEFT_4_SPDIF_RX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R357_SPDIF_RX_CS_LEFT_4_SPDIF_RX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_CS_LEFT_5 (IC 1) */
#define DSP1_R358_SPDIF_RX_CS_LEFT_5_SPDIF_RX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R358_SPDIF_RX_CS_LEFT_5_SPDIF_RX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R358_SPDIF_RX_CS_LEFT_5_SPDIF_RX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_CS_LEFT_6 (IC 1) */
#define DSP1_R359_SPDIF_RX_CS_LEFT_6_SPDIF_RX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R359_SPDIF_RX_CS_LEFT_6_SPDIF_RX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R359_SPDIF_RX_CS_LEFT_6_SPDIF_RX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_CS_LEFT_7 (IC 1) */
#define DSP1_R360_SPDIF_RX_CS_LEFT_7_SPDIF_RX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R360_SPDIF_RX_CS_LEFT_7_SPDIF_RX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R360_SPDIF_RX_CS_LEFT_7_SPDIF_RX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_CS_LEFT_8 (IC 1) */
#define DSP1_R361_SPDIF_RX_CS_LEFT_8_SPDIF_RX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R361_SPDIF_RX_CS_LEFT_8_SPDIF_RX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R361_SPDIF_RX_CS_LEFT_8_SPDIF_RX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_CS_LEFT_9 (IC 1) */
#define DSP1_R362_SPDIF_RX_CS_LEFT_9_SPDIF_RX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R362_SPDIF_RX_CS_LEFT_9_SPDIF_RX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R362_SPDIF_RX_CS_LEFT_9_SPDIF_RX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_CS_LEFT_10 (IC 1) */
#define DSP1_R363_SPDIF_RX_CS_LEFT_10_SPDIF_RX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R363_SPDIF_RX_CS_LEFT_10_SPDIF_RX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R363_SPDIF_RX_CS_LEFT_10_SPDIF_RX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_CS_LEFT_11 (IC 1) */
#define DSP1_R364_SPDIF_RX_CS_LEFT_11_SPDIF_RX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R364_SPDIF_RX_CS_LEFT_11_SPDIF_RX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R364_SPDIF_RX_CS_LEFT_11_SPDIF_RX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_CS_RIGHT_0 (IC 1) */
#define DSP1_R365_SPDIF_RX_CS_RIGHT_0_SPDIF_RX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R365_SPDIF_RX_CS_RIGHT_0_SPDIF_RX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R365_SPDIF_RX_CS_RIGHT_0_SPDIF_RX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_CS_RIGHT_1 (IC 1) */
#define DSP1_R366_SPDIF_RX_CS_RIGHT_1_SPDIF_RX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R366_SPDIF_RX_CS_RIGHT_1_SPDIF_RX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R366_SPDIF_RX_CS_RIGHT_1_SPDIF_RX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_CS_RIGHT_2 (IC 1) */
#define DSP1_R367_SPDIF_RX_CS_RIGHT_2_SPDIF_RX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R367_SPDIF_RX_CS_RIGHT_2_SPDIF_RX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R367_SPDIF_RX_CS_RIGHT_2_SPDIF_RX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_CS_RIGHT_3 (IC 1) */
#define DSP1_R368_SPDIF_RX_CS_RIGHT_3_SPDIF_RX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R368_SPDIF_RX_CS_RIGHT_3_SPDIF_RX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R368_SPDIF_RX_CS_RIGHT_3_SPDIF_RX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_CS_RIGHT_4 (IC 1) */
#define DSP1_R369_SPDIF_RX_CS_RIGHT_4_SPDIF_RX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R369_SPDIF_RX_CS_RIGHT_4_SPDIF_RX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R369_SPDIF_RX_CS_RIGHT_4_SPDIF_RX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_CS_RIGHT_5 (IC 1) */
#define DSP1_R370_SPDIF_RX_CS_RIGHT_5_SPDIF_RX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R370_SPDIF_RX_CS_RIGHT_5_SPDIF_RX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R370_SPDIF_RX_CS_RIGHT_5_SPDIF_RX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_CS_RIGHT_6 (IC 1) */
#define DSP1_R371_SPDIF_RX_CS_RIGHT_6_SPDIF_RX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R371_SPDIF_RX_CS_RIGHT_6_SPDIF_RX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R371_SPDIF_RX_CS_RIGHT_6_SPDIF_RX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_CS_RIGHT_7 (IC 1) */
#define DSP1_R372_SPDIF_RX_CS_RIGHT_7_SPDIF_RX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R372_SPDIF_RX_CS_RIGHT_7_SPDIF_RX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R372_SPDIF_RX_CS_RIGHT_7_SPDIF_RX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_CS_RIGHT_8 (IC 1) */
#define DSP1_R373_SPDIF_RX_CS_RIGHT_8_SPDIF_RX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R373_SPDIF_RX_CS_RIGHT_8_SPDIF_RX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R373_SPDIF_RX_CS_RIGHT_8_SPDIF_RX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_CS_RIGHT_9 (IC 1) */
#define DSP1_R374_SPDIF_RX_CS_RIGHT_9_SPDIF_RX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R374_SPDIF_RX_CS_RIGHT_9_SPDIF_RX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R374_SPDIF_RX_CS_RIGHT_9_SPDIF_RX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_CS_RIGHT_10 (IC 1) */
#define DSP1_R375_SPDIF_RX_CS_RIGHT_10_SPDIF_RX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R375_SPDIF_RX_CS_RIGHT_10_SPDIF_RX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R375_SPDIF_RX_CS_RIGHT_10_SPDIF_RX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_CS_RIGHT_11 (IC 1) */
#define DSP1_R376_SPDIF_RX_CS_RIGHT_11_SPDIF_RX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R376_SPDIF_RX_CS_RIGHT_11_SPDIF_RX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R376_SPDIF_RX_CS_RIGHT_11_SPDIF_RX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_UD_LEFT_0 (IC 1) */
#define DSP1_R377_SPDIF_RX_UD_LEFT_0_SPDIF_RX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R377_SPDIF_RX_UD_LEFT_0_SPDIF_RX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R377_SPDIF_RX_UD_LEFT_0_SPDIF_RX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_UD_LEFT_1 (IC 1) */
#define DSP1_R378_SPDIF_RX_UD_LEFT_1_SPDIF_RX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R378_SPDIF_RX_UD_LEFT_1_SPDIF_RX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R378_SPDIF_RX_UD_LEFT_1_SPDIF_RX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_UD_LEFT_2 (IC 1) */
#define DSP1_R379_SPDIF_RX_UD_LEFT_2_SPDIF_RX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R379_SPDIF_RX_UD_LEFT_2_SPDIF_RX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R379_SPDIF_RX_UD_LEFT_2_SPDIF_RX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_UD_LEFT_3 (IC 1) */
#define DSP1_R380_SPDIF_RX_UD_LEFT_3_SPDIF_RX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R380_SPDIF_RX_UD_LEFT_3_SPDIF_RX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R380_SPDIF_RX_UD_LEFT_3_SPDIF_RX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_UD_LEFT_4 (IC 1) */
#define DSP1_R381_SPDIF_RX_UD_LEFT_4_SPDIF_RX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R381_SPDIF_RX_UD_LEFT_4_SPDIF_RX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R381_SPDIF_RX_UD_LEFT_4_SPDIF_RX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_UD_LEFT_5 (IC 1) */
#define DSP1_R382_SPDIF_RX_UD_LEFT_5_SPDIF_RX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R382_SPDIF_RX_UD_LEFT_5_SPDIF_RX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R382_SPDIF_RX_UD_LEFT_5_SPDIF_RX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_UD_LEFT_6 (IC 1) */
#define DSP1_R383_SPDIF_RX_UD_LEFT_6_SPDIF_RX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R383_SPDIF_RX_UD_LEFT_6_SPDIF_RX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R383_SPDIF_RX_UD_LEFT_6_SPDIF_RX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_UD_LEFT_7 (IC 1) */
#define DSP1_R384_SPDIF_RX_UD_LEFT_7_SPDIF_RX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R384_SPDIF_RX_UD_LEFT_7_SPDIF_RX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R384_SPDIF_RX_UD_LEFT_7_SPDIF_RX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_UD_LEFT_8 (IC 1) */
#define DSP1_R385_SPDIF_RX_UD_LEFT_8_SPDIF_RX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R385_SPDIF_RX_UD_LEFT_8_SPDIF_RX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R385_SPDIF_RX_UD_LEFT_8_SPDIF_RX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_UD_LEFT_9 (IC 1) */
#define DSP1_R386_SPDIF_RX_UD_LEFT_9_SPDIF_RX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R386_SPDIF_RX_UD_LEFT_9_SPDIF_RX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R386_SPDIF_RX_UD_LEFT_9_SPDIF_RX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_UD_LEFT_10 (IC 1) */
#define DSP1_R387_SPDIF_RX_UD_LEFT_10_SPDIF_RX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R387_SPDIF_RX_UD_LEFT_10_SPDIF_RX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R387_SPDIF_RX_UD_LEFT_10_SPDIF_RX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_UD_LEFT_11 (IC 1) */
#define DSP1_R388_SPDIF_RX_UD_LEFT_11_SPDIF_RX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R388_SPDIF_RX_UD_LEFT_11_SPDIF_RX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R388_SPDIF_RX_UD_LEFT_11_SPDIF_RX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_UD_RIGHT_0 (IC 1) */
#define DSP1_R389_SPDIF_RX_UD_RIGHT_0_SPDIF_RX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R389_SPDIF_RX_UD_RIGHT_0_SPDIF_RX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R389_SPDIF_RX_UD_RIGHT_0_SPDIF_RX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_UD_RIGHT_1 (IC 1) */
#define DSP1_R390_SPDIF_RX_UD_RIGHT_1_SPDIF_RX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R390_SPDIF_RX_UD_RIGHT_1_SPDIF_RX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R390_SPDIF_RX_UD_RIGHT_1_SPDIF_RX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_UD_RIGHT_2 (IC 1) */
#define DSP1_R391_SPDIF_RX_UD_RIGHT_2_SPDIF_RX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R391_SPDIF_RX_UD_RIGHT_2_SPDIF_RX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R391_SPDIF_RX_UD_RIGHT_2_SPDIF_RX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_UD_RIGHT_3 (IC 1) */
#define DSP1_R392_SPDIF_RX_UD_RIGHT_3_SPDIF_RX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R392_SPDIF_RX_UD_RIGHT_3_SPDIF_RX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R392_SPDIF_RX_UD_RIGHT_3_SPDIF_RX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_UD_RIGHT_4 (IC 1) */
#define DSP1_R393_SPDIF_RX_UD_RIGHT_4_SPDIF_RX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R393_SPDIF_RX_UD_RIGHT_4_SPDIF_RX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R393_SPDIF_RX_UD_RIGHT_4_SPDIF_RX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_UD_RIGHT_5 (IC 1) */
#define DSP1_R394_SPDIF_RX_UD_RIGHT_5_SPDIF_RX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R394_SPDIF_RX_UD_RIGHT_5_SPDIF_RX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R394_SPDIF_RX_UD_RIGHT_5_SPDIF_RX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_UD_RIGHT_6 (IC 1) */
#define DSP1_R395_SPDIF_RX_UD_RIGHT_6_SPDIF_RX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R395_SPDIF_RX_UD_RIGHT_6_SPDIF_RX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R395_SPDIF_RX_UD_RIGHT_6_SPDIF_RX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_UD_RIGHT_7 (IC 1) */
#define DSP1_R396_SPDIF_RX_UD_RIGHT_7_SPDIF_RX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R396_SPDIF_RX_UD_RIGHT_7_SPDIF_RX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R396_SPDIF_RX_UD_RIGHT_7_SPDIF_RX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_UD_RIGHT_8 (IC 1) */
#define DSP1_R397_SPDIF_RX_UD_RIGHT_8_SPDIF_RX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R397_SPDIF_RX_UD_RIGHT_8_SPDIF_RX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R397_SPDIF_RX_UD_RIGHT_8_SPDIF_RX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_UD_RIGHT_9 (IC 1) */
#define DSP1_R398_SPDIF_RX_UD_RIGHT_9_SPDIF_RX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R398_SPDIF_RX_UD_RIGHT_9_SPDIF_RX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R398_SPDIF_RX_UD_RIGHT_9_SPDIF_RX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_UD_RIGHT_10 (IC 1) */
#define DSP1_R399_SPDIF_RX_UD_RIGHT_10_SPDIF_RX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R399_SPDIF_RX_UD_RIGHT_10_SPDIF_RX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R399_SPDIF_RX_UD_RIGHT_10_SPDIF_RX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_UD_RIGHT_11 (IC 1) */
#define DSP1_R400_SPDIF_RX_UD_RIGHT_11_SPDIF_RX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R400_SPDIF_RX_UD_RIGHT_11_SPDIF_RX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R400_SPDIF_RX_UD_RIGHT_11_SPDIF_RX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_VB_LEFT_0 (IC 1) */
#define DSP1_R401_SPDIF_RX_VB_LEFT_0_SPDIF_RX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R401_SPDIF_RX_VB_LEFT_0_SPDIF_RX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R401_SPDIF_RX_VB_LEFT_0_SPDIF_RX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_VB_LEFT_1 (IC 1) */
#define DSP1_R402_SPDIF_RX_VB_LEFT_1_SPDIF_RX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R402_SPDIF_RX_VB_LEFT_1_SPDIF_RX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R402_SPDIF_RX_VB_LEFT_1_SPDIF_RX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_VB_LEFT_2 (IC 1) */
#define DSP1_R403_SPDIF_RX_VB_LEFT_2_SPDIF_RX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R403_SPDIF_RX_VB_LEFT_2_SPDIF_RX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R403_SPDIF_RX_VB_LEFT_2_SPDIF_RX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_VB_LEFT_3 (IC 1) */
#define DSP1_R404_SPDIF_RX_VB_LEFT_3_SPDIF_RX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R404_SPDIF_RX_VB_LEFT_3_SPDIF_RX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R404_SPDIF_RX_VB_LEFT_3_SPDIF_RX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_VB_LEFT_4 (IC 1) */
#define DSP1_R405_SPDIF_RX_VB_LEFT_4_SPDIF_RX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R405_SPDIF_RX_VB_LEFT_4_SPDIF_RX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R405_SPDIF_RX_VB_LEFT_4_SPDIF_RX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_VB_LEFT_5 (IC 1) */
#define DSP1_R406_SPDIF_RX_VB_LEFT_5_SPDIF_RX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R406_SPDIF_RX_VB_LEFT_5_SPDIF_RX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R406_SPDIF_RX_VB_LEFT_5_SPDIF_RX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_VB_LEFT_6 (IC 1) */
#define DSP1_R407_SPDIF_RX_VB_LEFT_6_SPDIF_RX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R407_SPDIF_RX_VB_LEFT_6_SPDIF_RX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R407_SPDIF_RX_VB_LEFT_6_SPDIF_RX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_VB_LEFT_7 (IC 1) */
#define DSP1_R408_SPDIF_RX_VB_LEFT_7_SPDIF_RX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R408_SPDIF_RX_VB_LEFT_7_SPDIF_RX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R408_SPDIF_RX_VB_LEFT_7_SPDIF_RX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_VB_LEFT_8 (IC 1) */
#define DSP1_R409_SPDIF_RX_VB_LEFT_8_SPDIF_RX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R409_SPDIF_RX_VB_LEFT_8_SPDIF_RX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R409_SPDIF_RX_VB_LEFT_8_SPDIF_RX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_VB_LEFT_9 (IC 1) */
#define DSP1_R410_SPDIF_RX_VB_LEFT_9_SPDIF_RX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R410_SPDIF_RX_VB_LEFT_9_SPDIF_RX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R410_SPDIF_RX_VB_LEFT_9_SPDIF_RX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_VB_LEFT_10 (IC 1) */
#define DSP1_R411_SPDIF_RX_VB_LEFT_10_SPDIF_RX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R411_SPDIF_RX_VB_LEFT_10_SPDIF_RX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R411_SPDIF_RX_VB_LEFT_10_SPDIF_RX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_VB_LEFT_11 (IC 1) */
#define DSP1_R412_SPDIF_RX_VB_LEFT_11_SPDIF_RX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R412_SPDIF_RX_VB_LEFT_11_SPDIF_RX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R412_SPDIF_RX_VB_LEFT_11_SPDIF_RX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_VB_RIGHT_0 (IC 1) */
#define DSP1_R413_SPDIF_RX_VB_RIGHT_0_SPDIF_RX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R413_SPDIF_RX_VB_RIGHT_0_SPDIF_RX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R413_SPDIF_RX_VB_RIGHT_0_SPDIF_RX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_VB_RIGHT_1 (IC 1) */
#define DSP1_R414_SPDIF_RX_VB_RIGHT_1_SPDIF_RX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R414_SPDIF_RX_VB_RIGHT_1_SPDIF_RX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R414_SPDIF_RX_VB_RIGHT_1_SPDIF_RX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_VB_RIGHT_2 (IC 1) */
#define DSP1_R415_SPDIF_RX_VB_RIGHT_2_SPDIF_RX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R415_SPDIF_RX_VB_RIGHT_2_SPDIF_RX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R415_SPDIF_RX_VB_RIGHT_2_SPDIF_RX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_VB_RIGHT_3 (IC 1) */
#define DSP1_R416_SPDIF_RX_VB_RIGHT_3_SPDIF_RX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R416_SPDIF_RX_VB_RIGHT_3_SPDIF_RX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R416_SPDIF_RX_VB_RIGHT_3_SPDIF_RX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_VB_RIGHT_4 (IC 1) */
#define DSP1_R417_SPDIF_RX_VB_RIGHT_4_SPDIF_RX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R417_SPDIF_RX_VB_RIGHT_4_SPDIF_RX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R417_SPDIF_RX_VB_RIGHT_4_SPDIF_RX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_VB_RIGHT_5 (IC 1) */
#define DSP1_R418_SPDIF_RX_VB_RIGHT_5_SPDIF_RX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R418_SPDIF_RX_VB_RIGHT_5_SPDIF_RX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R418_SPDIF_RX_VB_RIGHT_5_SPDIF_RX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_VB_RIGHT_6 (IC 1) */
#define DSP1_R419_SPDIF_RX_VB_RIGHT_6_SPDIF_RX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R419_SPDIF_RX_VB_RIGHT_6_SPDIF_RX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R419_SPDIF_RX_VB_RIGHT_6_SPDIF_RX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_VB_RIGHT_7 (IC 1) */
#define DSP1_R420_SPDIF_RX_VB_RIGHT_7_SPDIF_RX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R420_SPDIF_RX_VB_RIGHT_7_SPDIF_RX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R420_SPDIF_RX_VB_RIGHT_7_SPDIF_RX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_VB_RIGHT_8 (IC 1) */
#define DSP1_R421_SPDIF_RX_VB_RIGHT_8_SPDIF_RX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R421_SPDIF_RX_VB_RIGHT_8_SPDIF_RX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R421_SPDIF_RX_VB_RIGHT_8_SPDIF_RX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_VB_RIGHT_9 (IC 1) */
#define DSP1_R422_SPDIF_RX_VB_RIGHT_9_SPDIF_RX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R422_SPDIF_RX_VB_RIGHT_9_SPDIF_RX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R422_SPDIF_RX_VB_RIGHT_9_SPDIF_RX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_VB_RIGHT_10 (IC 1) */
#define DSP1_R423_SPDIF_RX_VB_RIGHT_10_SPDIF_RX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R423_SPDIF_RX_VB_RIGHT_10_SPDIF_RX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R423_SPDIF_RX_VB_RIGHT_10_SPDIF_RX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_VB_RIGHT_11 (IC 1) */
#define DSP1_R424_SPDIF_RX_VB_RIGHT_11_SPDIF_RX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R424_SPDIF_RX_VB_RIGHT_11_SPDIF_RX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R424_SPDIF_RX_VB_RIGHT_11_SPDIF_RX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_PB_LEFT_0 (IC 1) */
#define DSP1_R425_SPDIF_RX_PB_LEFT_0_SPDIF_RX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R425_SPDIF_RX_PB_LEFT_0_SPDIF_RX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R425_SPDIF_RX_PB_LEFT_0_SPDIF_RX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_PB_LEFT_1 (IC 1) */
#define DSP1_R426_SPDIF_RX_PB_LEFT_1_SPDIF_RX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R426_SPDIF_RX_PB_LEFT_1_SPDIF_RX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R426_SPDIF_RX_PB_LEFT_1_SPDIF_RX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_PB_LEFT_2 (IC 1) */
#define DSP1_R427_SPDIF_RX_PB_LEFT_2_SPDIF_RX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R427_SPDIF_RX_PB_LEFT_2_SPDIF_RX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R427_SPDIF_RX_PB_LEFT_2_SPDIF_RX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_PB_LEFT_3 (IC 1) */
#define DSP1_R428_SPDIF_RX_PB_LEFT_3_SPDIF_RX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R428_SPDIF_RX_PB_LEFT_3_SPDIF_RX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R428_SPDIF_RX_PB_LEFT_3_SPDIF_RX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_PB_LEFT_4 (IC 1) */
#define DSP1_R429_SPDIF_RX_PB_LEFT_4_SPDIF_RX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R429_SPDIF_RX_PB_LEFT_4_SPDIF_RX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R429_SPDIF_RX_PB_LEFT_4_SPDIF_RX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_PB_LEFT_5 (IC 1) */
#define DSP1_R430_SPDIF_RX_PB_LEFT_5_SPDIF_RX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R430_SPDIF_RX_PB_LEFT_5_SPDIF_RX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R430_SPDIF_RX_PB_LEFT_5_SPDIF_RX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_PB_LEFT_6 (IC 1) */
#define DSP1_R431_SPDIF_RX_PB_LEFT_6_SPDIF_RX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R431_SPDIF_RX_PB_LEFT_6_SPDIF_RX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R431_SPDIF_RX_PB_LEFT_6_SPDIF_RX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_PB_LEFT_7 (IC 1) */
#define DSP1_R432_SPDIF_RX_PB_LEFT_7_SPDIF_RX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R432_SPDIF_RX_PB_LEFT_7_SPDIF_RX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R432_SPDIF_RX_PB_LEFT_7_SPDIF_RX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_PB_LEFT_8 (IC 1) */
#define DSP1_R433_SPDIF_RX_PB_LEFT_8_SPDIF_RX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R433_SPDIF_RX_PB_LEFT_8_SPDIF_RX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R433_SPDIF_RX_PB_LEFT_8_SPDIF_RX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_PB_LEFT_9 (IC 1) */
#define DSP1_R434_SPDIF_RX_PB_LEFT_9_SPDIF_RX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R434_SPDIF_RX_PB_LEFT_9_SPDIF_RX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R434_SPDIF_RX_PB_LEFT_9_SPDIF_RX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_PB_LEFT_10 (IC 1) */
#define DSP1_R435_SPDIF_RX_PB_LEFT_10_SPDIF_RX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R435_SPDIF_RX_PB_LEFT_10_SPDIF_RX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R435_SPDIF_RX_PB_LEFT_10_SPDIF_RX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_PB_LEFT_11 (IC 1) */
#define DSP1_R436_SPDIF_RX_PB_LEFT_11_SPDIF_RX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R436_SPDIF_RX_PB_LEFT_11_SPDIF_RX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R436_SPDIF_RX_PB_LEFT_11_SPDIF_RX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_RX_PB_RIGHT_0 (IC 1) */
#define DSP1_R437_SPDIF_RX_PB_RIGHT_0_SPDIF_RX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R437_SPDIF_RX_PB_RIGHT_0_SPDIF_RX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R437_SPDIF_RX_PB_RIGHT_0_SPDIF_RX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_PB_RIGHT_1 (IC 1) */
#define DSP1_R438_SPDIF_RX_PB_RIGHT_1_SPDIF_RX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R438_SPDIF_RX_PB_RIGHT_1_SPDIF_RX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R438_SPDIF_RX_PB_RIGHT_1_SPDIF_RX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_PB_RIGHT_2 (IC 1) */
#define DSP1_R439_SPDIF_RX_PB_RIGHT_2_SPDIF_RX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R439_SPDIF_RX_PB_RIGHT_2_SPDIF_RX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R439_SPDIF_RX_PB_RIGHT_2_SPDIF_RX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_PB_RIGHT_3 (IC 1) */
#define DSP1_R440_SPDIF_RX_PB_RIGHT_3_SPDIF_RX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R440_SPDIF_RX_PB_RIGHT_3_SPDIF_RX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R440_SPDIF_RX_PB_RIGHT_3_SPDIF_RX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_PB_RIGHT_4 (IC 1) */
#define DSP1_R441_SPDIF_RX_PB_RIGHT_4_SPDIF_RX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R441_SPDIF_RX_PB_RIGHT_4_SPDIF_RX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R441_SPDIF_RX_PB_RIGHT_4_SPDIF_RX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_PB_RIGHT_5 (IC 1) */
#define DSP1_R442_SPDIF_RX_PB_RIGHT_5_SPDIF_RX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R442_SPDIF_RX_PB_RIGHT_5_SPDIF_RX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R442_SPDIF_RX_PB_RIGHT_5_SPDIF_RX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_PB_RIGHT_6 (IC 1) */
#define DSP1_R443_SPDIF_RX_PB_RIGHT_6_SPDIF_RX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R443_SPDIF_RX_PB_RIGHT_6_SPDIF_RX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R443_SPDIF_RX_PB_RIGHT_6_SPDIF_RX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_PB_RIGHT_7 (IC 1) */
#define DSP1_R444_SPDIF_RX_PB_RIGHT_7_SPDIF_RX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R444_SPDIF_RX_PB_RIGHT_7_SPDIF_RX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R444_SPDIF_RX_PB_RIGHT_7_SPDIF_RX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_PB_RIGHT_8 (IC 1) */
#define DSP1_R445_SPDIF_RX_PB_RIGHT_8_SPDIF_RX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R445_SPDIF_RX_PB_RIGHT_8_SPDIF_RX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R445_SPDIF_RX_PB_RIGHT_8_SPDIF_RX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_PB_RIGHT_9 (IC 1) */
#define DSP1_R446_SPDIF_RX_PB_RIGHT_9_SPDIF_RX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R446_SPDIF_RX_PB_RIGHT_9_SPDIF_RX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R446_SPDIF_RX_PB_RIGHT_9_SPDIF_RX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_PB_RIGHT_10 (IC 1) */
#define DSP1_R447_SPDIF_RX_PB_RIGHT_10_SPDIF_RX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R447_SPDIF_RX_PB_RIGHT_10_SPDIF_RX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R447_SPDIF_RX_PB_RIGHT_10_SPDIF_RX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_RX_PB_RIGHT_11 (IC 1) */
#define DSP1_R448_SPDIF_RX_PB_RIGHT_11_SPDIF_RX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R448_SPDIF_RX_PB_RIGHT_11_SPDIF_RX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R448_SPDIF_RX_PB_RIGHT_11_SPDIF_RX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_EN (IC 1) */
#define DSP1_R449_SPDIF_TX_EN_TXEN_IC_1                0x0    /* 0b	[0] */
#define DSP1_R449_SPDIF_TX_EN_TXEN_IC_1_MASK           0x1
#define DSP1_R449_SPDIF_TX_EN_TXEN_IC_1_SHIFT          0

/* SPDIF_TX_CTRL (IC 1) */
#define DSP1_R450_SPDIF_TX_CTRL_TX_LENGTHCTRL_IC_1     0x0    /* 00b	[1:0] */
#define DSP1_R450_SPDIF_TX_CTRL_TX_LENGTHCTRL_IC_1_MASK 0x3
#define DSP1_R450_SPDIF_TX_CTRL_TX_LENGTHCTRL_IC_1_SHIFT 0

/* SPDIF_TX_AUXBIT_SOURCE (IC 1) */
#define DSP1_R451_SPDIF_TX_AUXBIT_SOURCE_TX_AUXBITS_SOURCE_IC_1 0x0 /* 0b	[0] */
#define DSP1_R451_SPDIF_TX_AUXBIT_SOURCE_TX_AUXBITS_SOURCE_IC_1_MASK 0x1
#define DSP1_R451_SPDIF_TX_AUXBIT_SOURCE_TX_AUXBITS_SOURCE_IC_1_SHIFT 0

/* SPDIF_TX_CS_LEFT_0 (IC 1) */
#define DSP1_R452_SPDIF_TX_CS_LEFT_0_SPDIF_TX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R452_SPDIF_TX_CS_LEFT_0_SPDIF_TX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R452_SPDIF_TX_CS_LEFT_0_SPDIF_TX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_CS_LEFT_1 (IC 1) */
#define DSP1_R453_SPDIF_TX_CS_LEFT_1_SPDIF_TX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R453_SPDIF_TX_CS_LEFT_1_SPDIF_TX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R453_SPDIF_TX_CS_LEFT_1_SPDIF_TX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_CS_LEFT_2 (IC 1) */
#define DSP1_R454_SPDIF_TX_CS_LEFT_2_SPDIF_TX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R454_SPDIF_TX_CS_LEFT_2_SPDIF_TX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R454_SPDIF_TX_CS_LEFT_2_SPDIF_TX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_CS_LEFT_3 (IC 1) */
#define DSP1_R455_SPDIF_TX_CS_LEFT_3_SPDIF_TX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R455_SPDIF_TX_CS_LEFT_3_SPDIF_TX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R455_SPDIF_TX_CS_LEFT_3_SPDIF_TX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_CS_LEFT_4 (IC 1) */
#define DSP1_R456_SPDIF_TX_CS_LEFT_4_SPDIF_TX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R456_SPDIF_TX_CS_LEFT_4_SPDIF_TX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R456_SPDIF_TX_CS_LEFT_4_SPDIF_TX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_CS_LEFT_5 (IC 1) */
#define DSP1_R457_SPDIF_TX_CS_LEFT_5_SPDIF_TX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R457_SPDIF_TX_CS_LEFT_5_SPDIF_TX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R457_SPDIF_TX_CS_LEFT_5_SPDIF_TX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_CS_LEFT_6 (IC 1) */
#define DSP1_R458_SPDIF_TX_CS_LEFT_6_SPDIF_TX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R458_SPDIF_TX_CS_LEFT_6_SPDIF_TX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R458_SPDIF_TX_CS_LEFT_6_SPDIF_TX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_CS_LEFT_7 (IC 1) */
#define DSP1_R459_SPDIF_TX_CS_LEFT_7_SPDIF_TX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R459_SPDIF_TX_CS_LEFT_7_SPDIF_TX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R459_SPDIF_TX_CS_LEFT_7_SPDIF_TX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_CS_LEFT_8 (IC 1) */
#define DSP1_R460_SPDIF_TX_CS_LEFT_8_SPDIF_TX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R460_SPDIF_TX_CS_LEFT_8_SPDIF_TX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R460_SPDIF_TX_CS_LEFT_8_SPDIF_TX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_CS_LEFT_9 (IC 1) */
#define DSP1_R461_SPDIF_TX_CS_LEFT_9_SPDIF_TX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R461_SPDIF_TX_CS_LEFT_9_SPDIF_TX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R461_SPDIF_TX_CS_LEFT_9_SPDIF_TX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_CS_LEFT_10 (IC 1) */
#define DSP1_R462_SPDIF_TX_CS_LEFT_10_SPDIF_TX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R462_SPDIF_TX_CS_LEFT_10_SPDIF_TX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R462_SPDIF_TX_CS_LEFT_10_SPDIF_TX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_CS_LEFT_11 (IC 1) */
#define DSP1_R463_SPDIF_TX_CS_LEFT_11_SPDIF_TX_CS_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R463_SPDIF_TX_CS_LEFT_11_SPDIF_TX_CS_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R463_SPDIF_TX_CS_LEFT_11_SPDIF_TX_CS_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_CS_RIGHT_0 (IC 1) */
#define DSP1_R464_SPDIF_TX_CS_RIGHT_0_SPDIF_TX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R464_SPDIF_TX_CS_RIGHT_0_SPDIF_TX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R464_SPDIF_TX_CS_RIGHT_0_SPDIF_TX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_CS_RIGHT_1 (IC 1) */
#define DSP1_R465_SPDIF_TX_CS_RIGHT_1_SPDIF_TX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R465_SPDIF_TX_CS_RIGHT_1_SPDIF_TX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R465_SPDIF_TX_CS_RIGHT_1_SPDIF_TX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_CS_RIGHT_2 (IC 1) */
#define DSP1_R466_SPDIF_TX_CS_RIGHT_2_SPDIF_TX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R466_SPDIF_TX_CS_RIGHT_2_SPDIF_TX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R466_SPDIF_TX_CS_RIGHT_2_SPDIF_TX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_CS_RIGHT_3 (IC 1) */
#define DSP1_R467_SPDIF_TX_CS_RIGHT_3_SPDIF_TX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R467_SPDIF_TX_CS_RIGHT_3_SPDIF_TX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R467_SPDIF_TX_CS_RIGHT_3_SPDIF_TX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_CS_RIGHT_4 (IC 1) */
#define DSP1_R468_SPDIF_TX_CS_RIGHT_4_SPDIF_TX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R468_SPDIF_TX_CS_RIGHT_4_SPDIF_TX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R468_SPDIF_TX_CS_RIGHT_4_SPDIF_TX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_CS_RIGHT_5 (IC 1) */
#define DSP1_R469_SPDIF_TX_CS_RIGHT_5_SPDIF_TX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R469_SPDIF_TX_CS_RIGHT_5_SPDIF_TX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R469_SPDIF_TX_CS_RIGHT_5_SPDIF_TX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_CS_RIGHT_6 (IC 1) */
#define DSP1_R470_SPDIF_TX_CS_RIGHT_6_SPDIF_TX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R470_SPDIF_TX_CS_RIGHT_6_SPDIF_TX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R470_SPDIF_TX_CS_RIGHT_6_SPDIF_TX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_CS_RIGHT_7 (IC 1) */
#define DSP1_R471_SPDIF_TX_CS_RIGHT_7_SPDIF_TX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R471_SPDIF_TX_CS_RIGHT_7_SPDIF_TX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R471_SPDIF_TX_CS_RIGHT_7_SPDIF_TX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_CS_RIGHT_8 (IC 1) */
#define DSP1_R472_SPDIF_TX_CS_RIGHT_8_SPDIF_TX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R472_SPDIF_TX_CS_RIGHT_8_SPDIF_TX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R472_SPDIF_TX_CS_RIGHT_8_SPDIF_TX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_CS_RIGHT_9 (IC 1) */
#define DSP1_R473_SPDIF_TX_CS_RIGHT_9_SPDIF_TX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R473_SPDIF_TX_CS_RIGHT_9_SPDIF_TX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R473_SPDIF_TX_CS_RIGHT_9_SPDIF_TX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_CS_RIGHT_10 (IC 1) */
#define DSP1_R474_SPDIF_TX_CS_RIGHT_10_SPDIF_TX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R474_SPDIF_TX_CS_RIGHT_10_SPDIF_TX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R474_SPDIF_TX_CS_RIGHT_10_SPDIF_TX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_CS_RIGHT_11 (IC 1) */
#define DSP1_R475_SPDIF_TX_CS_RIGHT_11_SPDIF_TX_CS_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R475_SPDIF_TX_CS_RIGHT_11_SPDIF_TX_CS_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R475_SPDIF_TX_CS_RIGHT_11_SPDIF_TX_CS_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_UD_LEFT_0 (IC 1) */
#define DSP1_R476_SPDIF_TX_UD_LEFT_0_SPDIF_TX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R476_SPDIF_TX_UD_LEFT_0_SPDIF_TX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R476_SPDIF_TX_UD_LEFT_0_SPDIF_TX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_UD_LEFT_1 (IC 1) */
#define DSP1_R477_SPDIF_TX_UD_LEFT_1_SPDIF_TX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R477_SPDIF_TX_UD_LEFT_1_SPDIF_TX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R477_SPDIF_TX_UD_LEFT_1_SPDIF_TX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_UD_LEFT_2 (IC 1) */
#define DSP1_R478_SPDIF_TX_UD_LEFT_2_SPDIF_TX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R478_SPDIF_TX_UD_LEFT_2_SPDIF_TX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R478_SPDIF_TX_UD_LEFT_2_SPDIF_TX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_UD_LEFT_3 (IC 1) */
#define DSP1_R479_SPDIF_TX_UD_LEFT_3_SPDIF_TX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R479_SPDIF_TX_UD_LEFT_3_SPDIF_TX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R479_SPDIF_TX_UD_LEFT_3_SPDIF_TX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_UD_LEFT_4 (IC 1) */
#define DSP1_R480_SPDIF_TX_UD_LEFT_4_SPDIF_TX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R480_SPDIF_TX_UD_LEFT_4_SPDIF_TX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R480_SPDIF_TX_UD_LEFT_4_SPDIF_TX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_UD_LEFT_5 (IC 1) */
#define DSP1_R481_SPDIF_TX_UD_LEFT_5_SPDIF_TX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R481_SPDIF_TX_UD_LEFT_5_SPDIF_TX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R481_SPDIF_TX_UD_LEFT_5_SPDIF_TX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_UD_LEFT_6 (IC 1) */
#define DSP1_R482_SPDIF_TX_UD_LEFT_6_SPDIF_TX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R482_SPDIF_TX_UD_LEFT_6_SPDIF_TX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R482_SPDIF_TX_UD_LEFT_6_SPDIF_TX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_UD_LEFT_7 (IC 1) */
#define DSP1_R483_SPDIF_TX_UD_LEFT_7_SPDIF_TX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R483_SPDIF_TX_UD_LEFT_7_SPDIF_TX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R483_SPDIF_TX_UD_LEFT_7_SPDIF_TX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_UD_LEFT_8 (IC 1) */
#define DSP1_R484_SPDIF_TX_UD_LEFT_8_SPDIF_TX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R484_SPDIF_TX_UD_LEFT_8_SPDIF_TX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R484_SPDIF_TX_UD_LEFT_8_SPDIF_TX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_UD_LEFT_9 (IC 1) */
#define DSP1_R485_SPDIF_TX_UD_LEFT_9_SPDIF_TX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R485_SPDIF_TX_UD_LEFT_9_SPDIF_TX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R485_SPDIF_TX_UD_LEFT_9_SPDIF_TX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_UD_LEFT_10 (IC 1) */
#define DSP1_R486_SPDIF_TX_UD_LEFT_10_SPDIF_TX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R486_SPDIF_TX_UD_LEFT_10_SPDIF_TX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R486_SPDIF_TX_UD_LEFT_10_SPDIF_TX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_UD_LEFT_11 (IC 1) */
#define DSP1_R487_SPDIF_TX_UD_LEFT_11_SPDIF_TX_UD_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R487_SPDIF_TX_UD_LEFT_11_SPDIF_TX_UD_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R487_SPDIF_TX_UD_LEFT_11_SPDIF_TX_UD_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_UD_RIGHT_0 (IC 1) */
#define DSP1_R488_SPDIF_TX_UD_RIGHT_0_SPDIF_TX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R488_SPDIF_TX_UD_RIGHT_0_SPDIF_TX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R488_SPDIF_TX_UD_RIGHT_0_SPDIF_TX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_UD_RIGHT_1 (IC 1) */
#define DSP1_R489_SPDIF_TX_UD_RIGHT_1_SPDIF_TX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R489_SPDIF_TX_UD_RIGHT_1_SPDIF_TX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R489_SPDIF_TX_UD_RIGHT_1_SPDIF_TX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_UD_RIGHT_2 (IC 1) */
#define DSP1_R490_SPDIF_TX_UD_RIGHT_2_SPDIF_TX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R490_SPDIF_TX_UD_RIGHT_2_SPDIF_TX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R490_SPDIF_TX_UD_RIGHT_2_SPDIF_TX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_UD_RIGHT_3 (IC 1) */
#define DSP1_R491_SPDIF_TX_UD_RIGHT_3_SPDIF_TX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R491_SPDIF_TX_UD_RIGHT_3_SPDIF_TX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R491_SPDIF_TX_UD_RIGHT_3_SPDIF_TX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_UD_RIGHT_4 (IC 1) */
#define DSP1_R492_SPDIF_TX_UD_RIGHT_4_SPDIF_TX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R492_SPDIF_TX_UD_RIGHT_4_SPDIF_TX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R492_SPDIF_TX_UD_RIGHT_4_SPDIF_TX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_UD_RIGHT_5 (IC 1) */
#define DSP1_R493_SPDIF_TX_UD_RIGHT_5_SPDIF_TX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R493_SPDIF_TX_UD_RIGHT_5_SPDIF_TX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R493_SPDIF_TX_UD_RIGHT_5_SPDIF_TX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_UD_RIGHT_6 (IC 1) */
#define DSP1_R494_SPDIF_TX_UD_RIGHT_6_SPDIF_TX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R494_SPDIF_TX_UD_RIGHT_6_SPDIF_TX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R494_SPDIF_TX_UD_RIGHT_6_SPDIF_TX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_UD_RIGHT_7 (IC 1) */
#define DSP1_R495_SPDIF_TX_UD_RIGHT_7_SPDIF_TX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R495_SPDIF_TX_UD_RIGHT_7_SPDIF_TX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R495_SPDIF_TX_UD_RIGHT_7_SPDIF_TX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_UD_RIGHT_8 (IC 1) */
#define DSP1_R496_SPDIF_TX_UD_RIGHT_8_SPDIF_TX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R496_SPDIF_TX_UD_RIGHT_8_SPDIF_TX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R496_SPDIF_TX_UD_RIGHT_8_SPDIF_TX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_UD_RIGHT_9 (IC 1) */
#define DSP1_R497_SPDIF_TX_UD_RIGHT_9_SPDIF_TX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R497_SPDIF_TX_UD_RIGHT_9_SPDIF_TX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R497_SPDIF_TX_UD_RIGHT_9_SPDIF_TX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_UD_RIGHT_10 (IC 1) */
#define DSP1_R498_SPDIF_TX_UD_RIGHT_10_SPDIF_TX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R498_SPDIF_TX_UD_RIGHT_10_SPDIF_TX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R498_SPDIF_TX_UD_RIGHT_10_SPDIF_TX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_UD_RIGHT_11 (IC 1) */
#define DSP1_R499_SPDIF_TX_UD_RIGHT_11_SPDIF_TX_UD_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R499_SPDIF_TX_UD_RIGHT_11_SPDIF_TX_UD_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R499_SPDIF_TX_UD_RIGHT_11_SPDIF_TX_UD_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_VB_LEFT_0 (IC 1) */
#define DSP1_R500_SPDIF_TX_VB_LEFT_0_SPDIF_TX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R500_SPDIF_TX_VB_LEFT_0_SPDIF_TX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R500_SPDIF_TX_VB_LEFT_0_SPDIF_TX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_VB_LEFT_1 (IC 1) */
#define DSP1_R501_SPDIF_TX_VB_LEFT_1_SPDIF_TX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R501_SPDIF_TX_VB_LEFT_1_SPDIF_TX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R501_SPDIF_TX_VB_LEFT_1_SPDIF_TX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_VB_LEFT_2 (IC 1) */
#define DSP1_R502_SPDIF_TX_VB_LEFT_2_SPDIF_TX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R502_SPDIF_TX_VB_LEFT_2_SPDIF_TX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R502_SPDIF_TX_VB_LEFT_2_SPDIF_TX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_VB_LEFT_3 (IC 1) */
#define DSP1_R503_SPDIF_TX_VB_LEFT_3_SPDIF_TX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R503_SPDIF_TX_VB_LEFT_3_SPDIF_TX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R503_SPDIF_TX_VB_LEFT_3_SPDIF_TX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_VB_LEFT_4 (IC 1) */
#define DSP1_R504_SPDIF_TX_VB_LEFT_4_SPDIF_TX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R504_SPDIF_TX_VB_LEFT_4_SPDIF_TX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R504_SPDIF_TX_VB_LEFT_4_SPDIF_TX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_VB_LEFT_5 (IC 1) */
#define DSP1_R505_SPDIF_TX_VB_LEFT_5_SPDIF_TX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R505_SPDIF_TX_VB_LEFT_5_SPDIF_TX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R505_SPDIF_TX_VB_LEFT_5_SPDIF_TX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_VB_LEFT_6 (IC 1) */
#define DSP1_R506_SPDIF_TX_VB_LEFT_6_SPDIF_TX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R506_SPDIF_TX_VB_LEFT_6_SPDIF_TX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R506_SPDIF_TX_VB_LEFT_6_SPDIF_TX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_VB_LEFT_7 (IC 1) */
#define DSP1_R507_SPDIF_TX_VB_LEFT_7_SPDIF_TX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R507_SPDIF_TX_VB_LEFT_7_SPDIF_TX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R507_SPDIF_TX_VB_LEFT_7_SPDIF_TX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_VB_LEFT_8 (IC 1) */
#define DSP1_R508_SPDIF_TX_VB_LEFT_8_SPDIF_TX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R508_SPDIF_TX_VB_LEFT_8_SPDIF_TX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R508_SPDIF_TX_VB_LEFT_8_SPDIF_TX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_VB_LEFT_9 (IC 1) */
#define DSP1_R509_SPDIF_TX_VB_LEFT_9_SPDIF_TX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R509_SPDIF_TX_VB_LEFT_9_SPDIF_TX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R509_SPDIF_TX_VB_LEFT_9_SPDIF_TX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_VB_LEFT_10 (IC 1) */
#define DSP1_R510_SPDIF_TX_VB_LEFT_10_SPDIF_TX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R510_SPDIF_TX_VB_LEFT_10_SPDIF_TX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R510_SPDIF_TX_VB_LEFT_10_SPDIF_TX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_VB_LEFT_11 (IC 1) */
#define DSP1_R511_SPDIF_TX_VB_LEFT_11_SPDIF_TX_VB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R511_SPDIF_TX_VB_LEFT_11_SPDIF_TX_VB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R511_SPDIF_TX_VB_LEFT_11_SPDIF_TX_VB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_VB_RIGHT_0 (IC 1) */
#define DSP1_R512_SPDIF_TX_VB_RIGHT_0_SPDIF_TX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R512_SPDIF_TX_VB_RIGHT_0_SPDIF_TX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R512_SPDIF_TX_VB_RIGHT_0_SPDIF_TX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_VB_RIGHT_1 (IC 1) */
#define DSP1_R513_SPDIF_TX_VB_RIGHT_1_SPDIF_TX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R513_SPDIF_TX_VB_RIGHT_1_SPDIF_TX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R513_SPDIF_TX_VB_RIGHT_1_SPDIF_TX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_VB_RIGHT_2 (IC 1) */
#define DSP1_R514_SPDIF_TX_VB_RIGHT_2_SPDIF_TX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R514_SPDIF_TX_VB_RIGHT_2_SPDIF_TX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R514_SPDIF_TX_VB_RIGHT_2_SPDIF_TX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_VB_RIGHT_3 (IC 1) */
#define DSP1_R515_SPDIF_TX_VB_RIGHT_3_SPDIF_TX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R515_SPDIF_TX_VB_RIGHT_3_SPDIF_TX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R515_SPDIF_TX_VB_RIGHT_3_SPDIF_TX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_VB_RIGHT_4 (IC 1) */
#define DSP1_R516_SPDIF_TX_VB_RIGHT_4_SPDIF_TX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R516_SPDIF_TX_VB_RIGHT_4_SPDIF_TX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R516_SPDIF_TX_VB_RIGHT_4_SPDIF_TX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_VB_RIGHT_5 (IC 1) */
#define DSP1_R517_SPDIF_TX_VB_RIGHT_5_SPDIF_TX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R517_SPDIF_TX_VB_RIGHT_5_SPDIF_TX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R517_SPDIF_TX_VB_RIGHT_5_SPDIF_TX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_VB_RIGHT_6 (IC 1) */
#define DSP1_R518_SPDIF_TX_VB_RIGHT_6_SPDIF_TX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R518_SPDIF_TX_VB_RIGHT_6_SPDIF_TX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R518_SPDIF_TX_VB_RIGHT_6_SPDIF_TX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_VB_RIGHT_7 (IC 1) */
#define DSP1_R519_SPDIF_TX_VB_RIGHT_7_SPDIF_TX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R519_SPDIF_TX_VB_RIGHT_7_SPDIF_TX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R519_SPDIF_TX_VB_RIGHT_7_SPDIF_TX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_VB_RIGHT_8 (IC 1) */
#define DSP1_R520_SPDIF_TX_VB_RIGHT_8_SPDIF_TX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R520_SPDIF_TX_VB_RIGHT_8_SPDIF_TX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R520_SPDIF_TX_VB_RIGHT_8_SPDIF_TX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_VB_RIGHT_9 (IC 1) */
#define DSP1_R521_SPDIF_TX_VB_RIGHT_9_SPDIF_TX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R521_SPDIF_TX_VB_RIGHT_9_SPDIF_TX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R521_SPDIF_TX_VB_RIGHT_9_SPDIF_TX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_VB_RIGHT_10 (IC 1) */
#define DSP1_R522_SPDIF_TX_VB_RIGHT_10_SPDIF_TX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R522_SPDIF_TX_VB_RIGHT_10_SPDIF_TX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R522_SPDIF_TX_VB_RIGHT_10_SPDIF_TX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_VB_RIGHT_11 (IC 1) */
#define DSP1_R523_SPDIF_TX_VB_RIGHT_11_SPDIF_TX_VB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R523_SPDIF_TX_VB_RIGHT_11_SPDIF_TX_VB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R523_SPDIF_TX_VB_RIGHT_11_SPDIF_TX_VB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_PB_LEFT_0 (IC 1) */
#define DSP1_R524_SPDIF_TX_PB_LEFT_0_SPDIF_TX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R524_SPDIF_TX_PB_LEFT_0_SPDIF_TX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R524_SPDIF_TX_PB_LEFT_0_SPDIF_TX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_PB_LEFT_1 (IC 1) */
#define DSP1_R525_SPDIF_TX_PB_LEFT_1_SPDIF_TX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R525_SPDIF_TX_PB_LEFT_1_SPDIF_TX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R525_SPDIF_TX_PB_LEFT_1_SPDIF_TX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_PB_LEFT_2 (IC 1) */
#define DSP1_R526_SPDIF_TX_PB_LEFT_2_SPDIF_TX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R526_SPDIF_TX_PB_LEFT_2_SPDIF_TX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R526_SPDIF_TX_PB_LEFT_2_SPDIF_TX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_PB_LEFT_3 (IC 1) */
#define DSP1_R527_SPDIF_TX_PB_LEFT_3_SPDIF_TX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R527_SPDIF_TX_PB_LEFT_3_SPDIF_TX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R527_SPDIF_TX_PB_LEFT_3_SPDIF_TX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_PB_LEFT_4 (IC 1) */
#define DSP1_R528_SPDIF_TX_PB_LEFT_4_SPDIF_TX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R528_SPDIF_TX_PB_LEFT_4_SPDIF_TX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R528_SPDIF_TX_PB_LEFT_4_SPDIF_TX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_PB_LEFT_5 (IC 1) */
#define DSP1_R529_SPDIF_TX_PB_LEFT_5_SPDIF_TX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R529_SPDIF_TX_PB_LEFT_5_SPDIF_TX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R529_SPDIF_TX_PB_LEFT_5_SPDIF_TX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_PB_LEFT_6 (IC 1) */
#define DSP1_R530_SPDIF_TX_PB_LEFT_6_SPDIF_TX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R530_SPDIF_TX_PB_LEFT_6_SPDIF_TX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R530_SPDIF_TX_PB_LEFT_6_SPDIF_TX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_PB_LEFT_7 (IC 1) */
#define DSP1_R531_SPDIF_TX_PB_LEFT_7_SPDIF_TX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R531_SPDIF_TX_PB_LEFT_7_SPDIF_TX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R531_SPDIF_TX_PB_LEFT_7_SPDIF_TX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_PB_LEFT_8 (IC 1) */
#define DSP1_R532_SPDIF_TX_PB_LEFT_8_SPDIF_TX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R532_SPDIF_TX_PB_LEFT_8_SPDIF_TX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R532_SPDIF_TX_PB_LEFT_8_SPDIF_TX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_PB_LEFT_9 (IC 1) */
#define DSP1_R533_SPDIF_TX_PB_LEFT_9_SPDIF_TX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R533_SPDIF_TX_PB_LEFT_9_SPDIF_TX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R533_SPDIF_TX_PB_LEFT_9_SPDIF_TX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_PB_LEFT_10 (IC 1) */
#define DSP1_R534_SPDIF_TX_PB_LEFT_10_SPDIF_TX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R534_SPDIF_TX_PB_LEFT_10_SPDIF_TX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R534_SPDIF_TX_PB_LEFT_10_SPDIF_TX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_PB_LEFT_11 (IC 1) */
#define DSP1_R535_SPDIF_TX_PB_LEFT_11_SPDIF_TX_PB_LEFT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R535_SPDIF_TX_PB_LEFT_11_SPDIF_TX_PB_LEFT_IC_1_MASK 0xFFFF
#define DSP1_R535_SPDIF_TX_PB_LEFT_11_SPDIF_TX_PB_LEFT_IC_1_SHIFT 0

/* SPDIF_TX_PB_RIGHT_0 (IC 1) */
#define DSP1_R536_SPDIF_TX_PB_RIGHT_0_SPDIF_TX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R536_SPDIF_TX_PB_RIGHT_0_SPDIF_TX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R536_SPDIF_TX_PB_RIGHT_0_SPDIF_TX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_PB_RIGHT_1 (IC 1) */
#define DSP1_R537_SPDIF_TX_PB_RIGHT_1_SPDIF_TX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R537_SPDIF_TX_PB_RIGHT_1_SPDIF_TX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R537_SPDIF_TX_PB_RIGHT_1_SPDIF_TX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_PB_RIGHT_2 (IC 1) */
#define DSP1_R538_SPDIF_TX_PB_RIGHT_2_SPDIF_TX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R538_SPDIF_TX_PB_RIGHT_2_SPDIF_TX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R538_SPDIF_TX_PB_RIGHT_2_SPDIF_TX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_PB_RIGHT_3 (IC 1) */
#define DSP1_R539_SPDIF_TX_PB_RIGHT_3_SPDIF_TX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R539_SPDIF_TX_PB_RIGHT_3_SPDIF_TX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R539_SPDIF_TX_PB_RIGHT_3_SPDIF_TX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_PB_RIGHT_4 (IC 1) */
#define DSP1_R540_SPDIF_TX_PB_RIGHT_4_SPDIF_TX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R540_SPDIF_TX_PB_RIGHT_4_SPDIF_TX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R540_SPDIF_TX_PB_RIGHT_4_SPDIF_TX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_PB_RIGHT_5 (IC 1) */
#define DSP1_R541_SPDIF_TX_PB_RIGHT_5_SPDIF_TX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R541_SPDIF_TX_PB_RIGHT_5_SPDIF_TX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R541_SPDIF_TX_PB_RIGHT_5_SPDIF_TX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_PB_RIGHT_6 (IC 1) */
#define DSP1_R542_SPDIF_TX_PB_RIGHT_6_SPDIF_TX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R542_SPDIF_TX_PB_RIGHT_6_SPDIF_TX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R542_SPDIF_TX_PB_RIGHT_6_SPDIF_TX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_PB_RIGHT_7 (IC 1) */
#define DSP1_R543_SPDIF_TX_PB_RIGHT_7_SPDIF_TX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R543_SPDIF_TX_PB_RIGHT_7_SPDIF_TX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R543_SPDIF_TX_PB_RIGHT_7_SPDIF_TX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_PB_RIGHT_8 (IC 1) */
#define DSP1_R544_SPDIF_TX_PB_RIGHT_8_SPDIF_TX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R544_SPDIF_TX_PB_RIGHT_8_SPDIF_TX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R544_SPDIF_TX_PB_RIGHT_8_SPDIF_TX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_PB_RIGHT_9 (IC 1) */
#define DSP1_R545_SPDIF_TX_PB_RIGHT_9_SPDIF_TX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R545_SPDIF_TX_PB_RIGHT_9_SPDIF_TX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R545_SPDIF_TX_PB_RIGHT_9_SPDIF_TX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_PB_RIGHT_10 (IC 1) */
#define DSP1_R546_SPDIF_TX_PB_RIGHT_10_SPDIF_TX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R546_SPDIF_TX_PB_RIGHT_10_SPDIF_TX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R546_SPDIF_TX_PB_RIGHT_10_SPDIF_TX_PB_RIGHT_IC_1_SHIFT 0

/* SPDIF_TX_PB_RIGHT_11 (IC 1) */
#define DSP1_R547_SPDIF_TX_PB_RIGHT_11_SPDIF_TX_PB_RIGHT_IC_1 0x0000 /* 0000000000000000b	[15:0] */
#define DSP1_R547_SPDIF_TX_PB_RIGHT_11_SPDIF_TX_PB_RIGHT_IC_1_MASK 0xFFFF
#define DSP1_R547_SPDIF_TX_PB_RIGHT_11_SPDIF_TX_PB_RIGHT_IC_1_SHIFT 0

/* BCLK_IN0_PIN (IC 1) */
#define DSP1_R548_BCLK_IN0_PIN_BCLK_IN_DRIVE_IC_1      0x0    /* 00b	[1:0] */
#define DSP1_R548_BCLK_IN0_PIN_BCLK_IN_SLEW_IC_1       0x2    /* 10b	[3:2] */
#define DSP1_R548_BCLK_IN0_PIN_BCLK_IN_PULL_IC_1       0x1    /* 1b	[4] */
#define DSP1_R548_BCLK_IN0_PIN_BCLK_IN_DRIVE_IC_1_MASK 0x3
#define DSP1_R548_BCLK_IN0_PIN_BCLK_IN_DRIVE_IC_1_SHIFT 0
#define DSP1_R548_BCLK_IN0_PIN_BCLK_IN_SLEW_IC_1_MASK  0xC
#define DSP1_R548_BCLK_IN0_PIN_BCLK_IN_SLEW_IC_1_SHIFT 2
#define DSP1_R548_BCLK_IN0_PIN_BCLK_IN_PULL_IC_1_MASK  0x10
#define DSP1_R548_BCLK_IN0_PIN_BCLK_IN_PULL_IC_1_SHIFT 4

/* BCLK_IN1_PIN (IC 1) */
#define DSP1_R549_BCLK_IN1_PIN_BCLK_IN_DRIVE_IC_1      0x0    /* 00b	[1:0] */
#define DSP1_R549_BCLK_IN1_PIN_BCLK_IN_SLEW_IC_1       0x2    /* 10b	[3:2] */
#define DSP1_R549_BCLK_IN1_PIN_BCLK_IN_PULL_IC_1       0x1    /* 1b	[4] */
#define DSP1_R549_BCLK_IN1_PIN_BCLK_IN_DRIVE_IC_1_MASK 0x3
#define DSP1_R549_BCLK_IN1_PIN_BCLK_IN_DRIVE_IC_1_SHIFT 0
#define DSP1_R549_BCLK_IN1_PIN_BCLK_IN_SLEW_IC_1_MASK  0xC
#define DSP1_R549_BCLK_IN1_PIN_BCLK_IN_SLEW_IC_1_SHIFT 2
#define DSP1_R549_BCLK_IN1_PIN_BCLK_IN_PULL_IC_1_MASK  0x10
#define DSP1_R549_BCLK_IN1_PIN_BCLK_IN_PULL_IC_1_SHIFT 4

/* BCLK_IN2_PIN (IC 1) */
#define DSP1_R550_BCLK_IN2_PIN_BCLK_IN_DRIVE_IC_1      0x0    /* 00b	[1:0] */
#define DSP1_R550_BCLK_IN2_PIN_BCLK_IN_SLEW_IC_1       0x2    /* 10b	[3:2] */
#define DSP1_R550_BCLK_IN2_PIN_BCLK_IN_PULL_IC_1       0x1    /* 1b	[4] */
#define DSP1_R550_BCLK_IN2_PIN_BCLK_IN_DRIVE_IC_1_MASK 0x3
#define DSP1_R550_BCLK_IN2_PIN_BCLK_IN_DRIVE_IC_1_SHIFT 0
#define DSP1_R550_BCLK_IN2_PIN_BCLK_IN_SLEW_IC_1_MASK  0xC
#define DSP1_R550_BCLK_IN2_PIN_BCLK_IN_SLEW_IC_1_SHIFT 2
#define DSP1_R550_BCLK_IN2_PIN_BCLK_IN_PULL_IC_1_MASK  0x10
#define DSP1_R550_BCLK_IN2_PIN_BCLK_IN_PULL_IC_1_SHIFT 4

/* BCLK_IN3_PIN (IC 1) */
#define DSP1_R551_BCLK_IN3_PIN_BCLK_IN_DRIVE_IC_1      0x0    /* 00b	[1:0] */
#define DSP1_R551_BCLK_IN3_PIN_BCLK_IN_SLEW_IC_1       0x2    /* 10b	[3:2] */
#define DSP1_R551_BCLK_IN3_PIN_BCLK_IN_PULL_IC_1       0x1    /* 1b	[4] */
#define DSP1_R551_BCLK_IN3_PIN_BCLK_IN_DRIVE_IC_1_MASK 0x3
#define DSP1_R551_BCLK_IN3_PIN_BCLK_IN_DRIVE_IC_1_SHIFT 0
#define DSP1_R551_BCLK_IN3_PIN_BCLK_IN_SLEW_IC_1_MASK  0xC
#define DSP1_R551_BCLK_IN3_PIN_BCLK_IN_SLEW_IC_1_SHIFT 2
#define DSP1_R551_BCLK_IN3_PIN_BCLK_IN_PULL_IC_1_MASK  0x10
#define DSP1_R551_BCLK_IN3_PIN_BCLK_IN_PULL_IC_1_SHIFT 4

/* BCLK_OUT0_PIN (IC 1) */
#define DSP1_R552_BCLK_OUT0_PIN_BCLK_OUT_DRIVE_IC_1    0x0    /* 00b	[1:0] */
#define DSP1_R552_BCLK_OUT0_PIN_BCLK_OUT_SLEW_IC_1     0x2    /* 10b	[3:2] */
#define DSP1_R552_BCLK_OUT0_PIN_BCLK_OUT_PULL_IC_1     0x1    /* 1b	[4] */
#define DSP1_R552_BCLK_OUT0_PIN_BCLK_OUT_DRIVE_IC_1_MASK 0x3
#define DSP1_R552_BCLK_OUT0_PIN_BCLK_OUT_DRIVE_IC_1_SHIFT 0
#define DSP1_R552_BCLK_OUT0_PIN_BCLK_OUT_SLEW_IC_1_MASK 0xC
#define DSP1_R552_BCLK_OUT0_PIN_BCLK_OUT_SLEW_IC_1_SHIFT 2
#define DSP1_R552_BCLK_OUT0_PIN_BCLK_OUT_PULL_IC_1_MASK 0x10
#define DSP1_R552_BCLK_OUT0_PIN_BCLK_OUT_PULL_IC_1_SHIFT 4

/* BCLK_OUT1_PIN (IC 1) */
#define DSP1_R553_BCLK_OUT1_PIN_BCLK_OUT_DRIVE_IC_1    0x0    /* 00b	[1:0] */
#define DSP1_R553_BCLK_OUT1_PIN_BCLK_OUT_SLEW_IC_1     0x2    /* 10b	[3:2] */
#define DSP1_R553_BCLK_OUT1_PIN_BCLK_OUT_PULL_IC_1     0x1    /* 1b	[4] */
#define DSP1_R553_BCLK_OUT1_PIN_BCLK_OUT_DRIVE_IC_1_MASK 0x3
#define DSP1_R553_BCLK_OUT1_PIN_BCLK_OUT_DRIVE_IC_1_SHIFT 0
#define DSP1_R553_BCLK_OUT1_PIN_BCLK_OUT_SLEW_IC_1_MASK 0xC
#define DSP1_R553_BCLK_OUT1_PIN_BCLK_OUT_SLEW_IC_1_SHIFT 2
#define DSP1_R553_BCLK_OUT1_PIN_BCLK_OUT_PULL_IC_1_MASK 0x10
#define DSP1_R553_BCLK_OUT1_PIN_BCLK_OUT_PULL_IC_1_SHIFT 4

/* BCLK_OUT2_PIN (IC 1) */
#define DSP1_R554_BCLK_OUT2_PIN_BCLK_OUT_DRIVE_IC_1    0x0    /* 00b	[1:0] */
#define DSP1_R554_BCLK_OUT2_PIN_BCLK_OUT_SLEW_IC_1     0x2    /* 10b	[3:2] */
#define DSP1_R554_BCLK_OUT2_PIN_BCLK_OUT_PULL_IC_1     0x1    /* 1b	[4] */
#define DSP1_R554_BCLK_OUT2_PIN_BCLK_OUT_DRIVE_IC_1_MASK 0x3
#define DSP1_R554_BCLK_OUT2_PIN_BCLK_OUT_DRIVE_IC_1_SHIFT 0
#define DSP1_R554_BCLK_OUT2_PIN_BCLK_OUT_SLEW_IC_1_MASK 0xC
#define DSP1_R554_BCLK_OUT2_PIN_BCLK_OUT_SLEW_IC_1_SHIFT 2
#define DSP1_R554_BCLK_OUT2_PIN_BCLK_OUT_PULL_IC_1_MASK 0x10
#define DSP1_R554_BCLK_OUT2_PIN_BCLK_OUT_PULL_IC_1_SHIFT 4

/* BCLK_OUT3_PIN (IC 1) */
#define DSP1_R555_BCLK_OUT3_PIN_BCLK_OUT_DRIVE_IC_1    0x0    /* 00b	[1:0] */
#define DSP1_R555_BCLK_OUT3_PIN_BCLK_OUT_SLEW_IC_1     0x2    /* 10b	[3:2] */
#define DSP1_R555_BCLK_OUT3_PIN_BCLK_OUT_PULL_IC_1     0x1    /* 1b	[4] */
#define DSP1_R555_BCLK_OUT3_PIN_BCLK_OUT_DRIVE_IC_1_MASK 0x3
#define DSP1_R555_BCLK_OUT3_PIN_BCLK_OUT_DRIVE_IC_1_SHIFT 0
#define DSP1_R555_BCLK_OUT3_PIN_BCLK_OUT_SLEW_IC_1_MASK 0xC
#define DSP1_R555_BCLK_OUT3_PIN_BCLK_OUT_SLEW_IC_1_SHIFT 2
#define DSP1_R555_BCLK_OUT3_PIN_BCLK_OUT_PULL_IC_1_MASK 0x10
#define DSP1_R555_BCLK_OUT3_PIN_BCLK_OUT_PULL_IC_1_SHIFT 4

/* LRCLK_IN0_PIN (IC 1) */
#define DSP1_R556_LRCLK_IN0_PIN_LRCLK_IN_DRIVE_IC_1    0x0    /* 00b	[1:0] */
#define DSP1_R556_LRCLK_IN0_PIN_LRCLK_IN_SLEW_IC_1     0x2    /* 10b	[3:2] */
#define DSP1_R556_LRCLK_IN0_PIN_LRCLK_IN_PULL_IC_1     0x1    /* 1b	[4] */
#define DSP1_R556_LRCLK_IN0_PIN_LRCLK_IN_DRIVE_IC_1_MASK 0x3
#define DSP1_R556_LRCLK_IN0_PIN_LRCLK_IN_DRIVE_IC_1_SHIFT 0
#define DSP1_R556_LRCLK_IN0_PIN_LRCLK_IN_SLEW_IC_1_MASK 0xC
#define DSP1_R556_LRCLK_IN0_PIN_LRCLK_IN_SLEW_IC_1_SHIFT 2
#define DSP1_R556_LRCLK_IN0_PIN_LRCLK_IN_PULL_IC_1_MASK 0x10
#define DSP1_R556_LRCLK_IN0_PIN_LRCLK_IN_PULL_IC_1_SHIFT 4

/* LRCLK_IN1_PIN (IC 1) */
#define DSP1_R557_LRCLK_IN1_PIN_LRCLK_IN_DRIVE_IC_1    0x0    /* 00b	[1:0] */
#define DSP1_R557_LRCLK_IN1_PIN_LRCLK_IN_SLEW_IC_1     0x2    /* 10b	[3:2] */
#define DSP1_R557_LRCLK_IN1_PIN_LRCLK_IN_PULL_IC_1     0x1    /* 1b	[4] */
#define DSP1_R557_LRCLK_IN1_PIN_LRCLK_IN_DRIVE_IC_1_MASK 0x3
#define DSP1_R557_LRCLK_IN1_PIN_LRCLK_IN_DRIVE_IC_1_SHIFT 0
#define DSP1_R557_LRCLK_IN1_PIN_LRCLK_IN_SLEW_IC_1_MASK 0xC
#define DSP1_R557_LRCLK_IN1_PIN_LRCLK_IN_SLEW_IC_1_SHIFT 2
#define DSP1_R557_LRCLK_IN1_PIN_LRCLK_IN_PULL_IC_1_MASK 0x10
#define DSP1_R557_LRCLK_IN1_PIN_LRCLK_IN_PULL_IC_1_SHIFT 4

/* LRCLK_IN2_PIN (IC 1) */
#define DSP1_R558_LRCLK_IN2_PIN_LRCLK_IN_DRIVE_IC_1    0x0    /* 00b	[1:0] */
#define DSP1_R558_LRCLK_IN2_PIN_LRCLK_IN_SLEW_IC_1     0x2    /* 10b	[3:2] */
#define DSP1_R558_LRCLK_IN2_PIN_LRCLK_IN_PULL_IC_1     0x1    /* 1b	[4] */
#define DSP1_R558_LRCLK_IN2_PIN_LRCLK_IN_DRIVE_IC_1_MASK 0x3
#define DSP1_R558_LRCLK_IN2_PIN_LRCLK_IN_DRIVE_IC_1_SHIFT 0
#define DSP1_R558_LRCLK_IN2_PIN_LRCLK_IN_SLEW_IC_1_MASK 0xC
#define DSP1_R558_LRCLK_IN2_PIN_LRCLK_IN_SLEW_IC_1_SHIFT 2
#define DSP1_R558_LRCLK_IN2_PIN_LRCLK_IN_PULL_IC_1_MASK 0x10
#define DSP1_R558_LRCLK_IN2_PIN_LRCLK_IN_PULL_IC_1_SHIFT 4

/* LRCLK_IN3_PIN (IC 1) */
#define DSP1_R559_LRCLK_IN3_PIN_LRCLK_IN_DRIVE_IC_1    0x0    /* 00b	[1:0] */
#define DSP1_R559_LRCLK_IN3_PIN_LRCLK_IN_SLEW_IC_1     0x2    /* 10b	[3:2] */
#define DSP1_R559_LRCLK_IN3_PIN_LRCLK_IN_PULL_IC_1     0x1    /* 1b	[4] */
#define DSP1_R559_LRCLK_IN3_PIN_LRCLK_IN_DRIVE_IC_1_MASK 0x3
#define DSP1_R559_LRCLK_IN3_PIN_LRCLK_IN_DRIVE_IC_1_SHIFT 0
#define DSP1_R559_LRCLK_IN3_PIN_LRCLK_IN_SLEW_IC_1_MASK 0xC
#define DSP1_R559_LRCLK_IN3_PIN_LRCLK_IN_SLEW_IC_1_SHIFT 2
#define DSP1_R559_LRCLK_IN3_PIN_LRCLK_IN_PULL_IC_1_MASK 0x10
#define DSP1_R559_LRCLK_IN3_PIN_LRCLK_IN_PULL_IC_1_SHIFT 4

/* LRCLK_OUT0_PIN (IC 1) */
#define DSP1_R560_LRCLK_OUT0_PIN_LRCLK_OUT_DRIVE_IC_1  0x0    /* 00b	[1:0] */
#define DSP1_R560_LRCLK_OUT0_PIN_LRCLK_OUT_SLEW_IC_1   0x2    /* 10b	[3:2] */
#define DSP1_R560_LRCLK_OUT0_PIN_LRCLK_OUT_PULL_IC_1   0x1    /* 1b	[4] */
#define DSP1_R560_LRCLK_OUT0_PIN_LRCLK_OUT_DRIVE_IC_1_MASK 0x3
#define DSP1_R560_LRCLK_OUT0_PIN_LRCLK_OUT_DRIVE_IC_1_SHIFT 0
#define DSP1_R560_LRCLK_OUT0_PIN_LRCLK_OUT_SLEW_IC_1_MASK 0xC
#define DSP1_R560_LRCLK_OUT0_PIN_LRCLK_OUT_SLEW_IC_1_SHIFT 2
#define DSP1_R560_LRCLK_OUT0_PIN_LRCLK_OUT_PULL_IC_1_MASK 0x10
#define DSP1_R560_LRCLK_OUT0_PIN_LRCLK_OUT_PULL_IC_1_SHIFT 4

/* LRCLK_OUT1_PIN (IC 1) */
#define DSP1_R561_LRCLK_OUT1_PIN_LRCLK_OUT_DRIVE_IC_1  0x0    /* 00b	[1:0] */
#define DSP1_R561_LRCLK_OUT1_PIN_LRCLK_OUT_SLEW_IC_1   0x2    /* 10b	[3:2] */
#define DSP1_R561_LRCLK_OUT1_PIN_LRCLK_OUT_PULL_IC_1   0x1    /* 1b	[4] */
#define DSP1_R561_LRCLK_OUT1_PIN_LRCLK_OUT_DRIVE_IC_1_MASK 0x3
#define DSP1_R561_LRCLK_OUT1_PIN_LRCLK_OUT_DRIVE_IC_1_SHIFT 0
#define DSP1_R561_LRCLK_OUT1_PIN_LRCLK_OUT_SLEW_IC_1_MASK 0xC
#define DSP1_R561_LRCLK_OUT1_PIN_LRCLK_OUT_SLEW_IC_1_SHIFT 2
#define DSP1_R561_LRCLK_OUT1_PIN_LRCLK_OUT_PULL_IC_1_MASK 0x10
#define DSP1_R561_LRCLK_OUT1_PIN_LRCLK_OUT_PULL_IC_1_SHIFT 4

/* LRCLK_OUT2_PIN (IC 1) */
#define DSP1_R562_LRCLK_OUT2_PIN_LRCLK_OUT_DRIVE_IC_1  0x0    /* 00b	[1:0] */
#define DSP1_R562_LRCLK_OUT2_PIN_LRCLK_OUT_SLEW_IC_1   0x2    /* 10b	[3:2] */
#define DSP1_R562_LRCLK_OUT2_PIN_LRCLK_OUT_PULL_IC_1   0x1    /* 1b	[4] */
#define DSP1_R562_LRCLK_OUT2_PIN_LRCLK_OUT_DRIVE_IC_1_MASK 0x3
#define DSP1_R562_LRCLK_OUT2_PIN_LRCLK_OUT_DRIVE_IC_1_SHIFT 0
#define DSP1_R562_LRCLK_OUT2_PIN_LRCLK_OUT_SLEW_IC_1_MASK 0xC
#define DSP1_R562_LRCLK_OUT2_PIN_LRCLK_OUT_SLEW_IC_1_SHIFT 2
#define DSP1_R562_LRCLK_OUT2_PIN_LRCLK_OUT_PULL_IC_1_MASK 0x10
#define DSP1_R562_LRCLK_OUT2_PIN_LRCLK_OUT_PULL_IC_1_SHIFT 4

/* LRCLK_OUT3_PIN (IC 1) */
#define DSP1_R563_LRCLK_OUT3_PIN_LRCLK_OUT_DRIVE_IC_1  0x0    /* 00b	[1:0] */
#define DSP1_R563_LRCLK_OUT3_PIN_LRCLK_OUT_SLEW_IC_1   0x2    /* 10b	[3:2] */
#define DSP1_R563_LRCLK_OUT3_PIN_LRCLK_OUT_PULL_IC_1   0x1    /* 1b	[4] */
#define DSP1_R563_LRCLK_OUT3_PIN_LRCLK_OUT_DRIVE_IC_1_MASK 0x3
#define DSP1_R563_LRCLK_OUT3_PIN_LRCLK_OUT_DRIVE_IC_1_SHIFT 0
#define DSP1_R563_LRCLK_OUT3_PIN_LRCLK_OUT_SLEW_IC_1_MASK 0xC
#define DSP1_R563_LRCLK_OUT3_PIN_LRCLK_OUT_SLEW_IC_1_SHIFT 2
#define DSP1_R563_LRCLK_OUT3_PIN_LRCLK_OUT_PULL_IC_1_MASK 0x10
#define DSP1_R563_LRCLK_OUT3_PIN_LRCLK_OUT_PULL_IC_1_SHIFT 4

/* SDATA_IN0_PIN (IC 1) */
#define DSP1_R564_SDATA_IN0_PIN_SDATA_IN_DRIVE_IC_1    0x0    /* 00b	[1:0] */
#define DSP1_R564_SDATA_IN0_PIN_SDATA_IN_SLEW_IC_1     0x2    /* 10b	[3:2] */
#define DSP1_R564_SDATA_IN0_PIN_SDATA_IN_PULL_IC_1     0x1    /* 1b	[4] */
#define DSP1_R564_SDATA_IN0_PIN_SDATA_IN_DRIVE_IC_1_MASK 0x3
#define DSP1_R564_SDATA_IN0_PIN_SDATA_IN_DRIVE_IC_1_SHIFT 0
#define DSP1_R564_SDATA_IN0_PIN_SDATA_IN_SLEW_IC_1_MASK 0xC
#define DSP1_R564_SDATA_IN0_PIN_SDATA_IN_SLEW_IC_1_SHIFT 2
#define DSP1_R564_SDATA_IN0_PIN_SDATA_IN_PULL_IC_1_MASK 0x10
#define DSP1_R564_SDATA_IN0_PIN_SDATA_IN_PULL_IC_1_SHIFT 4

/* SDATA_IN1_PIN (IC 1) */
#define DSP1_R565_SDATA_IN1_PIN_SDATA_IN_DRIVE_IC_1    0x0    /* 00b	[1:0] */
#define DSP1_R565_SDATA_IN1_PIN_SDATA_IN_SLEW_IC_1     0x2    /* 10b	[3:2] */
#define DSP1_R565_SDATA_IN1_PIN_SDATA_IN_PULL_IC_1     0x1    /* 1b	[4] */
#define DSP1_R565_SDATA_IN1_PIN_SDATA_IN_DRIVE_IC_1_MASK 0x3
#define DSP1_R565_SDATA_IN1_PIN_SDATA_IN_DRIVE_IC_1_SHIFT 0
#define DSP1_R565_SDATA_IN1_PIN_SDATA_IN_SLEW_IC_1_MASK 0xC
#define DSP1_R565_SDATA_IN1_PIN_SDATA_IN_SLEW_IC_1_SHIFT 2
#define DSP1_R565_SDATA_IN1_PIN_SDATA_IN_PULL_IC_1_MASK 0x10
#define DSP1_R565_SDATA_IN1_PIN_SDATA_IN_PULL_IC_1_SHIFT 4

/* SDATA_IN2_PIN (IC 1) */
#define DSP1_R566_SDATA_IN2_PIN_SDATA_IN_DRIVE_IC_1    0x0    /* 00b	[1:0] */
#define DSP1_R566_SDATA_IN2_PIN_SDATA_IN_SLEW_IC_1     0x2    /* 10b	[3:2] */
#define DSP1_R566_SDATA_IN2_PIN_SDATA_IN_PULL_IC_1     0x1    /* 1b	[4] */
#define DSP1_R566_SDATA_IN2_PIN_SDATA_IN_DRIVE_IC_1_MASK 0x3
#define DSP1_R566_SDATA_IN2_PIN_SDATA_IN_DRIVE_IC_1_SHIFT 0
#define DSP1_R566_SDATA_IN2_PIN_SDATA_IN_SLEW_IC_1_MASK 0xC
#define DSP1_R566_SDATA_IN2_PIN_SDATA_IN_SLEW_IC_1_SHIFT 2
#define DSP1_R566_SDATA_IN2_PIN_SDATA_IN_PULL_IC_1_MASK 0x10
#define DSP1_R566_SDATA_IN2_PIN_SDATA_IN_PULL_IC_1_SHIFT 4

/* SDATA_IN3_PIN (IC 1) */
#define DSP1_R567_SDATA_IN3_PIN_SDATA_IN_DRIVE_IC_1    0x0    /* 00b	[1:0] */
#define DSP1_R567_SDATA_IN3_PIN_SDATA_IN_SLEW_IC_1     0x2    /* 10b	[3:2] */
#define DSP1_R567_SDATA_IN3_PIN_SDATA_IN_PULL_IC_1     0x1    /* 1b	[4] */
#define DSP1_R567_SDATA_IN3_PIN_SDATA_IN_DRIVE_IC_1_MASK 0x3
#define DSP1_R567_SDATA_IN3_PIN_SDATA_IN_DRIVE_IC_1_SHIFT 0
#define DSP1_R567_SDATA_IN3_PIN_SDATA_IN_SLEW_IC_1_MASK 0xC
#define DSP1_R567_SDATA_IN3_PIN_SDATA_IN_SLEW_IC_1_SHIFT 2
#define DSP1_R567_SDATA_IN3_PIN_SDATA_IN_PULL_IC_1_MASK 0x10
#define DSP1_R567_SDATA_IN3_PIN_SDATA_IN_PULL_IC_1_SHIFT 4

/* SDATA_OUT0_PIN (IC 1) */
#define DSP1_R568_SDATA_OUT0_PIN_SDATA_OUT_DRIVE_IC_1  0x0    /* 00b	[1:0] */
#define DSP1_R568_SDATA_OUT0_PIN_SDATA_OUT_SLEW_IC_1   0x2    /* 10b	[3:2] */
#define DSP1_R568_SDATA_OUT0_PIN_SDATA_OUT_PULL_IC_1   0x0    /* 0b	[4] */
#define DSP1_R568_SDATA_OUT0_PIN_SDATA_OUT_DRIVE_IC_1_MASK 0x3
#define DSP1_R568_SDATA_OUT0_PIN_SDATA_OUT_DRIVE_IC_1_SHIFT 0
#define DSP1_R568_SDATA_OUT0_PIN_SDATA_OUT_SLEW_IC_1_MASK 0xC
#define DSP1_R568_SDATA_OUT0_PIN_SDATA_OUT_SLEW_IC_1_SHIFT 2
#define DSP1_R568_SDATA_OUT0_PIN_SDATA_OUT_PULL_IC_1_MASK 0x10
#define DSP1_R568_SDATA_OUT0_PIN_SDATA_OUT_PULL_IC_1_SHIFT 4

/* SDATA_OUT1_PIN (IC 1) */
#define DSP1_R569_SDATA_OUT1_PIN_SDATA_OUT_DRIVE_IC_1  0x0    /* 00b	[1:0] */
#define DSP1_R569_SDATA_OUT1_PIN_SDATA_OUT_SLEW_IC_1   0x2    /* 10b	[3:2] */
#define DSP1_R569_SDATA_OUT1_PIN_SDATA_OUT_PULL_IC_1   0x0    /* 0b	[4] */
#define DSP1_R569_SDATA_OUT1_PIN_SDATA_OUT_DRIVE_IC_1_MASK 0x3
#define DSP1_R569_SDATA_OUT1_PIN_SDATA_OUT_DRIVE_IC_1_SHIFT 0
#define DSP1_R569_SDATA_OUT1_PIN_SDATA_OUT_SLEW_IC_1_MASK 0xC
#define DSP1_R569_SDATA_OUT1_PIN_SDATA_OUT_SLEW_IC_1_SHIFT 2
#define DSP1_R569_SDATA_OUT1_PIN_SDATA_OUT_PULL_IC_1_MASK 0x10
#define DSP1_R569_SDATA_OUT1_PIN_SDATA_OUT_PULL_IC_1_SHIFT 4

/* SDATA_OUT2_PIN (IC 1) */
#define DSP1_R570_SDATA_OUT2_PIN_SDATA_OUT_DRIVE_IC_1  0x0    /* 00b	[1:0] */
#define DSP1_R570_SDATA_OUT2_PIN_SDATA_OUT_SLEW_IC_1   0x2    /* 10b	[3:2] */
#define DSP1_R570_SDATA_OUT2_PIN_SDATA_OUT_PULL_IC_1   0x0    /* 0b	[4] */
#define DSP1_R570_SDATA_OUT2_PIN_SDATA_OUT_DRIVE_IC_1_MASK 0x3
#define DSP1_R570_SDATA_OUT2_PIN_SDATA_OUT_DRIVE_IC_1_SHIFT 0
#define DSP1_R570_SDATA_OUT2_PIN_SDATA_OUT_SLEW_IC_1_MASK 0xC
#define DSP1_R570_SDATA_OUT2_PIN_SDATA_OUT_SLEW_IC_1_SHIFT 2
#define DSP1_R570_SDATA_OUT2_PIN_SDATA_OUT_PULL_IC_1_MASK 0x10
#define DSP1_R570_SDATA_OUT2_PIN_SDATA_OUT_PULL_IC_1_SHIFT 4

/* SDATA_OUT3_PIN (IC 1) */
#define DSP1_R571_SDATA_OUT3_PIN_SDATA_OUT_DRIVE_IC_1  0x0    /* 00b	[1:0] */
#define DSP1_R571_SDATA_OUT3_PIN_SDATA_OUT_SLEW_IC_1   0x2    /* 10b	[3:2] */
#define DSP1_R571_SDATA_OUT3_PIN_SDATA_OUT_PULL_IC_1   0x0    /* 0b	[4] */
#define DSP1_R571_SDATA_OUT3_PIN_SDATA_OUT_DRIVE_IC_1_MASK 0x3
#define DSP1_R571_SDATA_OUT3_PIN_SDATA_OUT_DRIVE_IC_1_SHIFT 0
#define DSP1_R571_SDATA_OUT3_PIN_SDATA_OUT_SLEW_IC_1_MASK 0xC
#define DSP1_R571_SDATA_OUT3_PIN_SDATA_OUT_SLEW_IC_1_SHIFT 2
#define DSP1_R571_SDATA_OUT3_PIN_SDATA_OUT_PULL_IC_1_MASK 0x10
#define DSP1_R571_SDATA_OUT3_PIN_SDATA_OUT_PULL_IC_1_SHIFT 4

/* SPDIF_TX_PIN (IC 1) */
#define DSP1_R572_SPDIF_TX_PIN_SPDIF_TX_DRIVE_IC_1     0x0    /* 00b	[1:0] */
#define DSP1_R572_SPDIF_TX_PIN_SPDIF_TX_SLEW_IC_1      0x2    /* 10b	[3:2] */
#define DSP1_R572_SPDIF_TX_PIN_SPDIF_TX_PULL_IC_1      0x0    /* 0b	[4] */
#define DSP1_R572_SPDIF_TX_PIN_SPDIF_TX_DRIVE_IC_1_MASK 0x3
#define DSP1_R572_SPDIF_TX_PIN_SPDIF_TX_DRIVE_IC_1_SHIFT 0
#define DSP1_R572_SPDIF_TX_PIN_SPDIF_TX_SLEW_IC_1_MASK 0xC
#define DSP1_R572_SPDIF_TX_PIN_SPDIF_TX_SLEW_IC_1_SHIFT 2
#define DSP1_R572_SPDIF_TX_PIN_SPDIF_TX_PULL_IC_1_MASK 0x10
#define DSP1_R572_SPDIF_TX_PIN_SPDIF_TX_PULL_IC_1_SHIFT 4

/* SCLK_SCL_PIN (IC 1) */
#define DSP1_R573_SCLK_SCL_PIN_SCLK_SCL_DRIVE_IC_1     0x0    /* 00b	[1:0] */
#define DSP1_R573_SCLK_SCL_PIN_SCLK_SCL_SLEW_IC_1      0x2    /* 10b	[3:2] */
#define DSP1_R573_SCLK_SCL_PIN_SCLK_SCL_PULL_IC_1      0x0    /* 0b	[4] */
#define DSP1_R573_SCLK_SCL_PIN_SCLK_SCL_DRIVE_IC_1_MASK 0x3
#define DSP1_R573_SCLK_SCL_PIN_SCLK_SCL_DRIVE_IC_1_SHIFT 0
#define DSP1_R573_SCLK_SCL_PIN_SCLK_SCL_SLEW_IC_1_MASK 0xC
#define DSP1_R573_SCLK_SCL_PIN_SCLK_SCL_SLEW_IC_1_SHIFT 2
#define DSP1_R573_SCLK_SCL_PIN_SCLK_SCL_PULL_IC_1_MASK 0x10
#define DSP1_R573_SCLK_SCL_PIN_SCLK_SCL_PULL_IC_1_SHIFT 4

/* MISO_SDA_PIN (IC 1) */
#define DSP1_R574_MISO_SDA_PIN_MISO_SDA_DRIVE_IC_1     0x0    /* 00b	[1:0] */
#define DSP1_R574_MISO_SDA_PIN_MISO_SDA_SLEW_IC_1      0x2    /* 10b	[3:2] */
#define DSP1_R574_MISO_SDA_PIN_MISO_SDA_PULL_IC_1      0x0    /* 0b	[4] */
#define DSP1_R574_MISO_SDA_PIN_MISO_SDA_DRIVE_IC_1_MASK 0x3
#define DSP1_R574_MISO_SDA_PIN_MISO_SDA_DRIVE_IC_1_SHIFT 0
#define DSP1_R574_MISO_SDA_PIN_MISO_SDA_SLEW_IC_1_MASK 0xC
#define DSP1_R574_MISO_SDA_PIN_MISO_SDA_SLEW_IC_1_SHIFT 2
#define DSP1_R574_MISO_SDA_PIN_MISO_SDA_PULL_IC_1_MASK 0x10
#define DSP1_R574_MISO_SDA_PIN_MISO_SDA_PULL_IC_1_SHIFT 4

/* SS_PIN (IC 1) */
#define DSP1_R575_SS_PIN_SS_DRIVE_IC_1                 0x0    /* 00b	[1:0] */
#define DSP1_R575_SS_PIN_SS_SLEW_IC_1                  0x2    /* 10b	[3:2] */
#define DSP1_R575_SS_PIN_SS_PULL_IC_1                  0x1    /* 1b	[4] */
#define DSP1_R575_SS_PIN_SS_DRIVE_IC_1_MASK            0x3
#define DSP1_R575_SS_PIN_SS_DRIVE_IC_1_SHIFT           0
#define DSP1_R575_SS_PIN_SS_SLEW_IC_1_MASK             0xC
#define DSP1_R575_SS_PIN_SS_SLEW_IC_1_SHIFT            2
#define DSP1_R575_SS_PIN_SS_PULL_IC_1_MASK             0x10
#define DSP1_R575_SS_PIN_SS_PULL_IC_1_SHIFT            4

/* MOSI_ADDR1_PIN (IC 1) */
#define DSP1_R576_MOSI_ADDR1_PIN_MOSI_ADDR1_DRIVE_IC_1 0x0    /* 00b	[1:0] */
#define DSP1_R576_MOSI_ADDR1_PIN_MOSI_ADDR1_SLEW_IC_1  0x2    /* 10b	[3:2] */
#define DSP1_R576_MOSI_ADDR1_PIN_MOSI_ADDR1_PULL_IC_1  0x1    /* 1b	[4] */
#define DSP1_R576_MOSI_ADDR1_PIN_MOSI_ADDR1_DRIVE_IC_1_MASK 0x3
#define DSP1_R576_MOSI_ADDR1_PIN_MOSI_ADDR1_DRIVE_IC_1_SHIFT 0
#define DSP1_R576_MOSI_ADDR1_PIN_MOSI_ADDR1_SLEW_IC_1_MASK 0xC
#define DSP1_R576_MOSI_ADDR1_PIN_MOSI_ADDR1_SLEW_IC_1_SHIFT 2
#define DSP1_R576_MOSI_ADDR1_PIN_MOSI_ADDR1_PULL_IC_1_MASK 0x10
#define DSP1_R576_MOSI_ADDR1_PIN_MOSI_ADDR1_PULL_IC_1_SHIFT 4

/* SCLK_SCL_M_PIN (IC 1) */
#define DSP1_R577_SCLK_SCL_M_PIN_SCLK_SCL_M_DRIVE_IC_1 0x0    /* 00b	[1:0] */
#define DSP1_R577_SCLK_SCL_M_PIN_SCLK_SCL_M_SLEW_IC_1  0x2    /* 10b	[3:2] */
#define DSP1_R577_SCLK_SCL_M_PIN_SCLK_SCL_M_PULL_IC_1  0x0    /* 0b	[4] */
#define DSP1_R577_SCLK_SCL_M_PIN_SCLK_SCL_M_DRIVE_IC_1_MASK 0x3
#define DSP1_R577_SCLK_SCL_M_PIN_SCLK_SCL_M_DRIVE_IC_1_SHIFT 0
#define DSP1_R577_SCLK_SCL_M_PIN_SCLK_SCL_M_SLEW_IC_1_MASK 0xC
#define DSP1_R577_SCLK_SCL_M_PIN_SCLK_SCL_M_SLEW_IC_1_SHIFT 2
#define DSP1_R577_SCLK_SCL_M_PIN_SCLK_SCL_M_PULL_IC_1_MASK 0x10
#define DSP1_R577_SCLK_SCL_M_PIN_SCLK_SCL_M_PULL_IC_1_SHIFT 4

/* MISO_SDA_M_PIN (IC 1) */
#define DSP1_R578_MISO_SDA_M_PIN_MISO_SDA_M_DRIVE_IC_1 0x0    /* 00b	[1:0] */
#define DSP1_R578_MISO_SDA_M_PIN_MISO_SDA_M_SLEW_IC_1  0x2    /* 10b	[3:2] */
#define DSP1_R578_MISO_SDA_M_PIN_MISO_SDA_M_PULL_IC_1  0x0    /* 0b	[4] */
#define DSP1_R578_MISO_SDA_M_PIN_MISO_SDA_M_DRIVE_IC_1_MASK 0x3
#define DSP1_R578_MISO_SDA_M_PIN_MISO_SDA_M_DRIVE_IC_1_SHIFT 0
#define DSP1_R578_MISO_SDA_M_PIN_MISO_SDA_M_SLEW_IC_1_MASK 0xC
#define DSP1_R578_MISO_SDA_M_PIN_MISO_SDA_M_SLEW_IC_1_SHIFT 2
#define DSP1_R578_MISO_SDA_M_PIN_MISO_SDA_M_PULL_IC_1_MASK 0x10
#define DSP1_R578_MISO_SDA_M_PIN_MISO_SDA_M_PULL_IC_1_SHIFT 4

/* SS_M_PIN (IC 1) */
#define DSP1_R579_SS_M_PIN_SS_M_DRIVE_IC_1             0x0    /* 00b	[1:0] */
#define DSP1_R579_SS_M_PIN_SS_M_SLEW_IC_1              0x2    /* 10b	[3:2] */
#define DSP1_R579_SS_M_PIN_SS_M_PULL_IC_1              0x1    /* 1b	[4] */
#define DSP1_R579_SS_M_PIN_SS_M_DRIVE_IC_1_MASK        0x3
#define DSP1_R579_SS_M_PIN_SS_M_DRIVE_IC_1_SHIFT       0
#define DSP1_R579_SS_M_PIN_SS_M_SLEW_IC_1_MASK         0xC
#define DSP1_R579_SS_M_PIN_SS_M_SLEW_IC_1_SHIFT        2
#define DSP1_R579_SS_M_PIN_SS_M_PULL_IC_1_MASK         0x10
#define DSP1_R579_SS_M_PIN_SS_M_PULL_IC_1_SHIFT        4

/* MOSI_M_PIN (IC 1) */
#define DSP1_R580_MOSI_M_PIN_MOSI_M_DRIVE_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R580_MOSI_M_PIN_MOSI_M_SLEW_IC_1          0x2    /* 10b	[3:2] */
#define DSP1_R580_MOSI_M_PIN_MOSI_M_PULL_IC_1          0x1    /* 1b	[4] */
#define DSP1_R580_MOSI_M_PIN_MOSI_M_DRIVE_IC_1_MASK    0x3
#define DSP1_R580_MOSI_M_PIN_MOSI_M_DRIVE_IC_1_SHIFT   0
#define DSP1_R580_MOSI_M_PIN_MOSI_M_SLEW_IC_1_MASK     0xC
#define DSP1_R580_MOSI_M_PIN_MOSI_M_SLEW_IC_1_SHIFT    2
#define DSP1_R580_MOSI_M_PIN_MOSI_M_PULL_IC_1_MASK     0x10
#define DSP1_R580_MOSI_M_PIN_MOSI_M_PULL_IC_1_SHIFT    4

/* MP6_PIN (IC 1) */
#define DSP1_R581_MP6_PIN_MP6_DRIVE_IC_1               0x0    /* 00b	[1:0] */
#define DSP1_R581_MP6_PIN_MP6_SLEW_IC_1                0x2    /* 10b	[3:2] */
#define DSP1_R581_MP6_PIN_MP6_PULL_IC_1                0x1    /* 1b	[4] */
#define DSP1_R581_MP6_PIN_MP6_DRIVE_IC_1_MASK          0x3
#define DSP1_R581_MP6_PIN_MP6_DRIVE_IC_1_SHIFT         0
#define DSP1_R581_MP6_PIN_MP6_SLEW_IC_1_MASK           0xC
#define DSP1_R581_MP6_PIN_MP6_SLEW_IC_1_SHIFT          2
#define DSP1_R581_MP6_PIN_MP6_PULL_IC_1_MASK           0x10
#define DSP1_R581_MP6_PIN_MP6_PULL_IC_1_SHIFT          4

/* MP7_PIN (IC 1) */
#define DSP1_R582_MP7_PIN_MP7_DRIVE_IC_1               0x0    /* 00b	[1:0] */
#define DSP1_R582_MP7_PIN_MP7_SLEW_IC_1                0x2    /* 10b	[3:2] */
#define DSP1_R582_MP7_PIN_MP7_PULL_IC_1                0x1    /* 1b	[4] */
#define DSP1_R582_MP7_PIN_MP7_DRIVE_IC_1_MASK          0x3
#define DSP1_R582_MP7_PIN_MP7_DRIVE_IC_1_SHIFT         0
#define DSP1_R582_MP7_PIN_MP7_SLEW_IC_1_MASK           0xC
#define DSP1_R582_MP7_PIN_MP7_SLEW_IC_1_SHIFT          2
#define DSP1_R582_MP7_PIN_MP7_PULL_IC_1_MASK           0x10
#define DSP1_R582_MP7_PIN_MP7_PULL_IC_1_SHIFT          4

/* CLKOUT_PIN (IC 1) */
#define DSP1_R583_CLKOUT_PIN_CLKOUT_DRIVE_IC_1         0x0    /* 00b	[1:0] */
#define DSP1_R583_CLKOUT_PIN_CLKOUT_SLEW_IC_1          0x2    /* 10b	[3:2] */
#define DSP1_R583_CLKOUT_PIN_CLKOUT_PULL_IC_1          0x0    /* 0b	[4] */
#define DSP1_R583_CLKOUT_PIN_CLKOUT_DRIVE_IC_1_MASK    0x3
#define DSP1_R583_CLKOUT_PIN_CLKOUT_DRIVE_IC_1_SHIFT   0
#define DSP1_R583_CLKOUT_PIN_CLKOUT_SLEW_IC_1_MASK     0xC
#define DSP1_R583_CLKOUT_PIN_CLKOUT_SLEW_IC_1_SHIFT    2
#define DSP1_R583_CLKOUT_PIN_CLKOUT_PULL_IC_1_MASK     0x10
#define DSP1_R583_CLKOUT_PIN_CLKOUT_PULL_IC_1_SHIFT    4

/* SOFT_RESET (IC 1) */
#define DSP1_R584_SOFTWARE_RESET_IC_1                  0x1    /* 1b	[0] */
#define DSP1_R584_SOFTWARE_RESET_IC_1_MASK             0x1
#define DSP1_R584_SOFTWARE_RESET_IC_1_SHIFT            0

/* SECOND_PAGE_ENABLE (IC 1) */
#define DSP1_R585_SECOND_PAGE_ENABLE_IC_1              0x0    /* 0b	[0] */
#define DSP1_R585_SECOND_PAGE_ENABLE_IC_1_MASK         0x1
#define DSP1_R585_SECOND_PAGE_ENABLE_IC_1_SHIFT        0

#endif
