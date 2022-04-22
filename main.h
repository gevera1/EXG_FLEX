/*
 * main.h
 *
 *  Created on: Mar 3, 2022
 *      Author: adria
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <msp430f5529.h>
#include <limits.h>

/* Packet DEFINEs */
#define EXG_H_IND   0U
#define EXG_L_IND   1U
#define ACL_IND     2U

/* Electrode DEFINEs */
#define EXGn        2U

/* Accelerometer DEFINEs */
#define ACLn        6U
#define ACL_SHIFT   128U

#define ACL_START   (0x3B)
#define ACL_STOP    (0x40)

/* Timer DEFINEs */
#define TMR_PERIOD  33U    // 1 ms

// MPU6050 I2C ADDRESS
#define MPU_6050_ADDR   (0x68)
#define WHO_AM_I        (0x75)

#define SIG_PTH_RST     (0x68)

#define ACCEL_STT       (0x3B)
#define ACCEL_STP       (0x40)

// Power Management 1 Reg.
#define PWR_MGMT_1  (0x6B)
#define CLKSEL_0    (0x00)
#define CLKSEL_1    (0x01)
#define CLKSEL_2    (0x02)
#define CLKSEL_3    (0x03)
#define CLKSEL_4    (0x04)
#define CLKSEL_5    (0x05)
#define CLKSEL_6    (0x06)
#define CLKSEL_7    (0x07)
#define TEMP_DIS    (0x08)
#define CYCLE       (0x20)
#define SLEEP       (0x40)
#define DEVICE_RST  (0x80)

// Power Management 2 Reg.
#define PWR_MGMT_2  (0x6C)
#define LP_0        (0x00)
#define LP_1        (0x40)
#define LP_2        (0x80)
#define LP_3        (0xC0)
#define STBY_G      (0x07)
#define STBY_A      (0x38)

#endif /* MAIN_H_ */
