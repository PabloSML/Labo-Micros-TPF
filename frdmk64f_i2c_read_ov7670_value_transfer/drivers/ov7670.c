/***************************************************************************//**
  @file     ov7670.c
  @brief    OV7670 Camera Driver Source File
  @author   Grupo 4
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "i2c_sccb_layer.h"
#include "ov7670.h"
#include "fsl_debug_console.h"
/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define OV7670_SLAVE_ADDRESS        0x21U
#define OV7670_COM3_ADDRESS         0x0CU
#define OV7670_COM7_ADDRESS         0x12U
#define OV7670_COM10_ADDRESS        0x15U

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/



/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

static bool masked_modify_register(uint8_t register, uint8_t mask, uint8_t databyte);

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/



/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

bool ov7670_reset_regs(void)
{
    // uint8_t write_reg = OV7670_COM7_ADDRESS;
    const uint8_t databyte = 0x80U;
    return I2C_WriteSCCBReg(BOARD_SCCB_I2C_BASEADDR, OV7670_SLAVE_ADDRESS, OV7670_COM7_ADDRESS, databyte);
}

bool ov7670_enable_downsampling(void)
{
    return masked_modify_register(OV7670_COM3_ADDRESS, 0x40U, 0x40U);
}

uint8_t ov7670_read_com10(void)
{
    uint8_t readBuff;
    I2C_ReadSCCBRegs(BOARD_SCCB_I2C_BASEADDR, OV7670_SLAVE_ADDRESS, OV7670_COM10_ADDRESS, &readBuff, 1);
    return readBuff;
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

static bool masked_modify_register(uint8_t reg, uint8_t mask, uint8_t databyte)
{
    bool success;
    uint8_t readBuff;
    uint8_t my_databyte;

    success = I2C_ReadSCCBRegs(BOARD_SCCB_I2C_BASEADDR, OV7670_SLAVE_ADDRESS, reg, &readBuff, 1);
    if(success)
    {
        readBuff = readBuff & ~mask;        // Clear target bits
        PRINTF("\r\nMMR readBuff value is: %x", readBuff);
        my_databyte = readBuff | databyte;     // Add new bits
        PRINTF("\r\nMMR my_dv value is: %x", my_databyte);
        success =  I2C_WriteSCCBReg(BOARD_SCCB_I2C_BASEADDR, OV7670_SLAVE_ADDRESS, reg, my_databyte);

        if(success)
            return true;
        else
            return false;
    }
    else
        return false;
}

/******************************************************************************/
