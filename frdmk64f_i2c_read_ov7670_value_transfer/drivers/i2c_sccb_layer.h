/***************************************************************************//**
  @file     i2c_sccb_layer.h
  @brief    I2C SCCB Layer Header File
  @author   Grupo 4 (Based on i2c_read_accel_value_transfer in K64f SDK by NXP)
 ******************************************************************************/

#ifndef _I2C_SCCB_H_
#define _I2C_SCCB_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "MK64F12.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define BOARD_SCCB_I2C_BASEADDR   I2C0
#define BOARD_SCCB_I2C_CLOCK_FREQ CLOCK_GetFreq(I2C0_CLK_SRC)

#define SCCB_I2C_CLK_SRC     I2C0_CLK_SRC
#define SCCB_I2C_CLK_FREQ    CLOCK_GetFreq(I2C0_CLK_SRC)
#define I2C_BAUDRATE          100000
#define I2C_RELEASE_SDA_PORT  PORTE
#define I2C_RELEASE_SCL_PORT  PORTE
#define I2C_RELEASE_SDA_GPIO  GPIOE
#define I2C_RELEASE_SDA_PIN   25U
#define I2C_RELEASE_SCL_GPIO  GPIOE
#define I2C_RELEASE_SCL_PIN   24U
#define I2C_RELEASE_BUS_COUNT 100U

#define CAM_PID_REG           0x0AU    // ID address for com testing
#define CAM_PID_H             0x76U    // ID value for com testing

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/


/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief fun description
 */
void BOARD_I2C_ReleaseBus(void);
void I2C_InitModule(void);
bool I2C_ReadSCCBWhoAmI(void);
bool I2C_WriteSCCBReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value);
bool I2C_ReadSCCBRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);


/*******************************************************************************
 ******************************************************************************/

#endif // _I2C_SCCB_H_
