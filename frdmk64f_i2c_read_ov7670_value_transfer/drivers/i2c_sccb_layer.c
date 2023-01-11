/***************************************************************************//**
  @file     i2c_sccb_layer.c
  @brief    I2C SCCB Layer Source File
  @author   Grupo 4 (Based on i2c_read_accel_value_transfer in K64f SDK by NXP)
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
/*  SDK Included Files */
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"
#include "i2c_sccb_layer.h"

#include "fsl_gpio.h"
#include "fsl_port.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/


/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

static void i2c_release_bus_delay(void);
static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData);

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE   CHECK STATIC!
 ******************************************************************************/

static const uint8_t cam_address = 0x21U;

static i2c_master_handle_t g_m_handle;

static volatile bool completionFlag = false;
static volatile bool nakFlag        = false;

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void BOARD_I2C_ReleaseBus(void)
{
    uint8_t i = 0;
    gpio_pin_config_t pin_config;
    port_pin_config_t i2c_pin_config = {0};

    /* Config pin mux as gpio */
    i2c_pin_config.pullSelect = kPORT_PullUp;
    i2c_pin_config.mux        = kPORT_MuxAsGpio;

    pin_config.pinDirection = kGPIO_DigitalOutput;
    pin_config.outputLogic  = 1U;
    CLOCK_EnableClock(kCLOCK_PortE);
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &i2c_pin_config);
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SDA_PIN, &i2c_pin_config);

    GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
    GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

    /* Drive SDA low first to simulate a start */
    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    /* Send 9 pulses on SCL and keep SDA high */
    for (i = 0; i < 9; i++)
    {
        GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
        i2c_release_bus_delay();

        GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
        i2c_release_bus_delay();

        GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    /* Send stop */
    GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
    i2c_release_bus_delay();
}

void I2C_InitModule(void)
{
    static bool yaInit = false;

    if(!yaInit)
    {
        yaInit = true;
        i2c_master_config_t masterConfig;
        uint32_t sourceClock = 0;

        /*
        * masterConfig.baudRate_Bps = 100000U;
        * masterConfig.enableStopHold = false;
        * masterConfig.glitchFilterWidth = 0U;
        * masterConfig.enableMaster = true;
        */
        I2C_MasterGetDefaultConfig(&masterConfig);
        masterConfig.baudRate_Bps = I2C_BAUDRATE;
        sourceClock               = SCCB_I2C_CLK_FREQ;
        I2C_MasterInit(BOARD_SCCB_I2C_BASEADDR, &masterConfig, sourceClock);
        I2C_MasterTransferCreateHandle(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, i2c_master_callback, NULL);
    }
}


bool I2C_ReadSCCBWhoAmI(void)
{
    /*
    How to read the device who_am_I value ?
    Start + Device_address_Write , who_am_I_register;
    Repeart_Start + Device_address_Read , who_am_I_value.
    */

    uint8_t who_am_i_reg          = CAM_PID_REG;

    uint8_t who_am_i_value        = 0x00;
    uint8_t accel_addr_array_size = 0x00;
    bool find_device              = false;
    uint8_t i                     = 0;

    // I2C_InitModule();

    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress   = cam_address;

    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = &who_am_i_reg;
    masterXfer.dataSize       = 1;
    //masterXfer.flags          = kI2C_TransferNoStopFlag;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    I2C_MasterTransferNonBlocking(BOARD_SCCB_I2C_BASEADDR, &g_m_handle, &masterXfer);

    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag     = false;
        find_device        = true;
    }

    if (find_device == true)
    {
        masterXfer.direction      = kI2C_Read;
        masterXfer.subaddress     = 0;
        masterXfer.subaddressSize = 0;
        masterXfer.data           = &who_am_i_value;
        masterXfer.dataSize       = 1;
        // masterXfer.flags          = kI2C_TransferRepeatedStartFlag;
        masterXfer.flags          = kI2C_TransferDefaultFlag;

        I2C_MasterTransferNonBlocking(BOARD_SCCB_I2C_BASEADDR, &g_m_handle, &masterXfer);

        /*  wait for transfer completed. */
        while ((!nakFlag) && (!completionFlag))
        {
        }

        nakFlag = false;

        if (completionFlag == true)
        {
            completionFlag = false;

            if (who_am_i_value == CAM_PID_H)
            {
                PRINTF("Found cam , the device address is 0x%x . \r\n", masterXfer.slaveAddress);
                return true;
            }

            else
            {
                PRINTF("Found a device, the WhoAmI value is 0x%x\r\n", who_am_i_value);
                PRINTF("The device address is 0x%x. \r\n", masterXfer.slaveAddress);
                return false;
            }
        }
        else
        {
            PRINTF("Not a successful i2c communication \r\n");
            return false;
        }
    }
    else
    {
        PRINTF("\r\n Did not find camera device ! \r\n");
        return false;
    }
}

bool I2C_WriteSCCBReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress   = device_addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data           = &value;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    I2C_MasterTransferNonBlocking(BOARD_SCCB_I2C_BASEADDR, &g_m_handle, &masterXfer);

    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}

bool I2C_ReadSCCBRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress   = device_addr;
    masterXfer.direction      = kI2C_Write;
    masterXfer.subaddress     = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data           = &reg_addr;
    masterXfer.dataSize       = 1;
    masterXfer.flags          = kI2C_TransferDefaultFlag;

    I2C_MasterTransferNonBlocking(BOARD_SCCB_I2C_BASEADDR, &g_m_handle, &masterXfer);

    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;

        masterXfer.direction      = kI2C_Read;
        masterXfer.subaddress     = 0;
        masterXfer.subaddressSize = 0;
        masterXfer.data           = rxBuff;
        masterXfer.dataSize       = rxSize;
        masterXfer.flags          = kI2C_TransferDefaultFlag;

        I2C_MasterTransferNonBlocking(BOARD_SCCB_I2C_BASEADDR, &g_m_handle, &masterXfer);

        /*  wait for transfer completed. */
        while ((!nakFlag) && (!completionFlag))
        {
        }

        nakFlag = false;

        if (completionFlag == true)
        {
            completionFlag = false;
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}

// bool I2C_ReadSCCBRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
// {
//     i2c_master_transfer_t masterXfer;
//     memset(&masterXfer, 0, sizeof(masterXfer));
//     masterXfer.slaveAddress   = device_addr;
//     masterXfer.direction      = kI2C_Read;
//     masterXfer.subaddress     = reg_addr;
//     masterXfer.subaddressSize = 1;
//     masterXfer.data           = rxBuff;
//     masterXfer.dataSize       = rxSize;
//     masterXfer.flags          = kI2C_TransferDefaultFlag;

//     /*  direction=write : start+device_write;cmdbuff;xBuff; */
//     /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

//     I2C_MasterTransferNonBlocking(BOARD_SCCB_I2C_BASEADDR, &g_m_handle, &masterXfer);

//     /*  wait for transfer completed. */
//     while ((!nakFlag) && (!completionFlag))
//     {
//     }

//     nakFlag = false;

//     if (completionFlag == true)
//     {
//         completionFlag = false;
//         return true;
//     }
//     else
//     {
//         return false;
//     }
// }

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

static void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
    {
        __NOP();
    }
}

static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        completionFlag = true;
    }
    /* Signal transfer success when received success status. */
    if ((status == kStatus_I2C_Nak) || (status == kStatus_I2C_Addr_Nak))
    {
        nakFlag = true;
    }
}
