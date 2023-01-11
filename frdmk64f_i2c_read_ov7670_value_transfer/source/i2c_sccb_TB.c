/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017,2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*  SDK Included Files */
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "myboard.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"
#include "hardware.h"

#include "ftm.h"
#include "i2c_sccb_layer.h"
#include "ov7670.h"
#include "DMA.h"
#include "magnetic_reader_drv.h"

#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_common.h"

#include "jpec.h"

#define BOARD_DBUS_GPIO         GPIOC
#define BOARD_DBUS_PORT         PORTC
#define BOARD_VSYNC_GPIO        GPIOA
#define BOARD_VSYNC_PORT        PORTA
#define BOARD_VSYNC_GPIO_PIN    PIN2NUM(VSYNC_PIN)
#define BOARD_VSYNC_IRQ         PORTA_IRQn
#define BOARD_VSYNC_IRQ_HANDLER PORTA_IRQHandler
#define BOARD_HREF_GPIO         GPIOD
#define BOARD_HREF_PORT         PORTD
#define BOARD_HREF_GPIO_PIN     PIN2NUM(HREF_PIN)
#define BOARD_HREF_IRQ          PORTD_IRQn
#define BOARD_HREF_IRQ_HANDLER  PORTD_IRQHandler
#define BOARD_PCLK_GPIO         GPIOB
#define BOARD_SW_GPIO           BOARD_SW2_GPIO
#define BOARD_SW_PORT           BOARD_SW2_PORT
#define BOARD_SW_GPIO_PIN       BOARD_SW2_GPIO_PIN
#define BOARD_SW_IRQ            BOARD_SW2_IRQ
#define BOARD_SW_IRQ_HANDLER    BOARD_SW2_IRQ_HANDLER
#define BOARD_SW_NAME           BOARD_SW2_NAME
#define PCR_PE_ENABLE	  	    1

#define FRAME_WIDTH         320
#define FRAME_HEIGHT        240

#define ENCODING_BUFFER_SIZE    36330       // Testing pre-allocated buffer due to small heap size.

#define BITCLR(x,pos) ((x) &= ((uint32_t)(~(((uint32_t)1) << (pos)))))
#define BITSET(x,pos) ((x) |= ((uint32_t)((((uint32_t)1) << (pos)))))


/* Whether the SW button is pressed */
volatile bool g_ButtonPress = false;
volatile bool g_frame_requested = false;
volatile bool g_new_frame_started = false;
volatile bool g_pic_ready = false;

static uint16_t buffer[FRAME_WIDTH+80];
static uint8_t frame[FRAME_HEIGHT][FRAME_WIDTH];

static uint8_t enc_buffer[ENCODING_BUFFER_SIZE];

//static uint8_t test_buffer_1[3][FRAME_WIDTH];
//static uint8_t test_buffer_2[3][FRAME_WIDTH];
static uint16_t v_irq_counter;
static uint16_t h_irq_counter;

/*!
 * @brief Interrupt service fuction of switch.
 */
__ISR__ BOARD_SW_IRQ_HANDLER(void)
{
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BOARD_SW_GPIO, 1U << BOARD_SW_GPIO_PIN);
    if(!g_frame_requested)
    {
        NVIC_DisableIRQ(BOARD_SW_IRQ);          //  Deshabilito IRQ por pedido de frame   
        g_frame_requested = true;
        GPIO_PortClearInterruptFlags(BOARD_VSYNC_GPIO, 1U << BOARD_VSYNC_GPIO_PIN);
        NVIC_ClearPendingIRQ(BOARD_VSYNC_IRQ);  //  Limpio pedidos previos al valido
        NVIC_EnableIRQ(BOARD_VSYNC_IRQ);        //  Habilito IRQ por nuevo frame
    }
    SDK_ISR_EXIT_BARRIER;
}

/*!
 * @brief Interrupt service fuction of vsync.
 */
__ISR__ BOARD_VSYNC_IRQ_HANDLER(void)
{
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BOARD_VSYNC_GPIO, 1U << BOARD_VSYNC_GPIO_PIN);

    if(g_frame_requested)
    {
        NVIC_DisableIRQ(BOARD_VSYNC_IRQ); //  Deshabilito IRQ por nuevo frame
        g_frame_requested = false;
        g_new_frame_started = true;
        v_irq_counter++;
        DMA_Enable_Requests();
        GPIO_PortClearInterruptFlags(BOARD_HREF_GPIO, 1U << BOARD_HREF_GPIO_PIN);
        NVIC_ClearPendingIRQ(BOARD_HREF_IRQ);
        NVIC_EnableIRQ(BOARD_HREF_IRQ);
    }
    SDK_ISR_EXIT_BARRIER;
}

/*!
 * @brief Interrupt service fuction of href.
 */
__ISR__ BOARD_HREF_IRQ_HANDLER(void)
{
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BOARD_HREF_GPIO, 1U << BOARD_HREF_GPIO_PIN);

    if(g_new_frame_started)
    {
        DMA_Reset_DADDR();
        for(uint16_t i = 0; i < FRAME_WIDTH; i++)
        {
            frame[h_irq_counter][i] = (uint8_t)((buffer[i]&0x003F) + ((buffer[i]&0x0180)>>1));
        }

        if(++h_irq_counter == FRAME_HEIGHT)                 // Termino el frame
        {
            DMA_Disable_Requests();
            NVIC_DisableIRQ(BOARD_HREF_IRQ);                //  Deshabilito IRQ por nuevo frame
            DMA_Reset_TCD();

            PRINTF("\r\nNew pic taken, sample value: %x\r\n", frame[119][159]);

            g_pic_ready = true;
            g_new_frame_started = false;
            h_irq_counter = 0;
            NVIC_ClearPendingIRQ(BOARD_SW_IRQ);
            NVIC_EnableIRQ(BOARD_SW_IRQ);
        }
    }

    SDK_ISR_EXIT_BARRIER;
}

int main(void)
{
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    hw_Init();
    hw_DisableInterrupts();

    FTM_Init();

    magneticReaderInit();

    //while(1);

    BOARD_I2C_ReleaseBus();
    BOARD_I2C_ConfigurePins();
    I2C_InitModule();

    DMA_init(buffer);

    BOARD_InitDebugConsole();

    hw_EnableInterrupts();


/*******************************************************************************
 *******************************************************************************
                                    SCCB CONFIG
*******************************************************************************
******************************************************************************/

    PRINTF("\r\nI2C example -- Read OV7670 Value\r\n");

    bool isThereCam = false;

    isThereCam = I2C_ReadSCCBWhoAmI();

    if (true == isThereCam)
    {
        uint8_t cam_address = 0x21U;
        uint8_t write_reg;
        uint8_t databyte;
        uint8_t read_reg;
        uint8_t readBuff;

        // Reset SCCB Registers
        //write_reg = 0x12U;
        //databyte = 0x80U;
        //I2C_WriteSCCBReg(BOARD_SCCB_I2C_BASEADDR, cam_address, write_reg, databyte);
        // ov7670_reset_regs();
        //PRINTF("\r\nSCCB Registers have been reset");

        uint64_t looper = 0;

        // PRINTF("\r\nEl loop tarda...");
        // while(looper++ < 20000000){}
        // PRINTF("\r\n... esto.");

        read_reg = 0x11U;
        I2C_ReadSCCBRegs(BOARD_SCCB_I2C_BASEADDR, cam_address, read_reg, &readBuff, 1);
        PRINTF("\r\nCurrent CLKRC value is: %x", readBuff);  // Default = 0x80

        write_reg = 0x11U;
        databyte = readBuff | 0x07U;
        I2C_WriteSCCBReg(BOARD_SCCB_I2C_BASEADDR, cam_address, write_reg, databyte);
        PRINTF("\r\nPre scalar has been enabled");

        read_reg = 0x11U;
        I2C_ReadSCCBRegs(BOARD_SCCB_I2C_BASEADDR, cam_address, read_reg, &readBuff, 1);
        PRINTF("\r\nNew CLKRC value is: %x", readBuff);  // 0x87 if success

        // Read current COM10 (Default = 0x00)
        read_reg = 0x15U;
        I2C_ReadSCCBRegs(BOARD_SCCB_I2C_BASEADDR, cam_address, read_reg, &readBuff, 1);
        PRINTF("\r\nCurrent COM10 value is: %x", readBuff);

        // PCLK during data only
        write_reg = 0x15U;
        databyte = readBuff | 0x20U;
        I2C_WriteSCCBReg(BOARD_SCCB_I2C_BASEADDR, cam_address, write_reg, databyte);
        PRINTF("\r\nPCLK during data only has been enabled");

        // Read new COM10 (0x20 if success)
        read_reg = 0x15U;
        I2C_ReadSCCBRegs(BOARD_SCCB_I2C_BASEADDR, cam_address, read_reg, &readBuff, 1);
        PRINTF("\r\nNew COM3 value is: %x", readBuff);

        // Read current Contrast (Default = 0x40)
        read_reg = 0x56U;
        I2C_ReadSCCBRegs(BOARD_SCCB_I2C_BASEADDR, cam_address, read_reg, &readBuff, 1);
        PRINTF("\r\nCurrent Contrast value is: %x", readBuff);

        write_reg = 0x56U;
        databyte = 0x44U;
        I2C_WriteSCCBReg(BOARD_SCCB_I2C_BASEADDR, cam_address, write_reg, databyte);
        PRINTF("\r\nContrast Modified");

        // Test Image
        //write_reg = 0x70U;
        //databyte = 0x3AU | 0x80U;
        //I2C_WriteSCCBReg(BOARD_SCCB_I2C_BASEADDR, cam_address, write_reg, databyte);
        // write_reg = 0x71U;
        // databyte = 0x35U;
        // I2C_WriteSCCBReg(BOARD_SCCB_I2C_BASEADDR, cam_address, write_reg, databyte);
        // PRINTF("\r\nTest image has been enabled");

        looper = 0;
        while(looper++ < 20000000){}

    }
    PRINTF("\r\nEnd of I2C example .\r\n");

/*******************************************************************************
 *******************************************************************************
                        SW2 CONFIG (HTTP Frame Req Placeholder)
*******************************************************************************
******************************************************************************/

    hw_DisableInterrupts();

    /* Define the init structure for input pin */
    gpio_pin_config_t in_config = {
        kGPIO_DigitalInput,
        0,
    };

    //  Config SW2 IRQ con flanco descendente
    BOARD_SW_PORT->PCR[BOARD_SW_GPIO_PIN]=0x00;
	BOARD_SW_PORT->PCR[BOARD_SW_GPIO_PIN]|=PORT_PCR_MUX(PORT_mGPIO); 		       //Set MUX to GPIO;
	BOARD_SW_PORT->PCR[BOARD_SW_GPIO_PIN]|=PORT_PCR_PE(1);          		       //Pull UP/Down  Enable;
	BOARD_SW_PORT->PCR[BOARD_SW_GPIO_PIN]|=PORT_PCR_PS(1);          		       //Pull UP
	BOARD_SW_PORT->PCR[BOARD_SW_GPIO_PIN]|=PORT_PCR_IRQC(PORT_eInterruptFalling);
    NVIC_EnableIRQ(BOARD_SW_IRQ);

    //  Config VSYNC IRQ con flanco ascendente
    NVIC_DisableIRQ(BOARD_VSYNC_IRQ);
    BOARD_VSYNC_PORT->PCR[BOARD_VSYNC_GPIO_PIN]=0x00;
	BOARD_VSYNC_PORT->PCR[BOARD_VSYNC_GPIO_PIN]|=PORT_PCR_MUX(PORT_mGPIO); 		        //Set MUX to GPIO;
    BOARD_VSYNC_PORT->PCR[BOARD_VSYNC_GPIO_PIN]|=PORT_PCR_PE(1);                        //Pull UP/Down  Enable;
    BOARD_VSYNC_PORT->PCR[BOARD_VSYNC_GPIO_PIN]|=PORT_PCR_PS(0);                        //Pull Down
	BOARD_VSYNC_PORT->PCR[BOARD_VSYNC_GPIO_PIN]|=PORT_PCR_IRQC(PORT_eInterruptRising);

    //  Config HREF IRQ con flanco descendente
    NVIC_DisableIRQ(BOARD_HREF_IRQ);
    BOARD_HREF_PORT->PCR[BOARD_HREF_GPIO_PIN]=0x00;
	BOARD_HREF_PORT->PCR[BOARD_HREF_GPIO_PIN]|=PORT_PCR_MUX(PORT_mGPIO); 		        //Set MUX to GPIO;
    BOARD_HREF_PORT->PCR[BOARD_HREF_GPIO_PIN]|=PORT_PCR_PE(1);                          //Pull UP/Down  Enable;
    BOARD_HREF_PORT->PCR[BOARD_HREF_GPIO_PIN]|=PORT_PCR_PS(0);                          //Pull Down
	BOARD_HREF_PORT->PCR[BOARD_HREF_GPIO_PIN]|=PORT_PCR_IRQC(PORT_eInterruptFalling);

    //  Init Pines como Input

    //  Init SW2 Pin
    GPIO_PinInit(BOARD_SW_GPIO, BOARD_SW_GPIO_PIN, &in_config);
    //  Init Sync Pins
    GPIO_PinInit(BOARD_VSYNC_GPIO, PIN2NUM(VSYNC_PIN), &in_config);
    GPIO_PinInit(BOARD_HREF_GPIO, PIN2NUM(HREF_PIN), &in_config);
    GPIO_PinInit(BOARD_PCLK_GPIO, PIN2NUM(PCLK_PIN), &in_config);
    //  Init Data bus pins
    BOARD_DBUS_PORT->PCR[PIN2NUM(D0_PIN)]=0x00;
	BOARD_DBUS_PORT->PCR[PIN2NUM(D0_PIN)]|=PORT_PCR_MUX(PORT_mGPIO); 		       //Set MUX to GPIO;
    GPIO_PinInit(BOARD_DBUS_GPIO, PIN2NUM(D0_PIN), &in_config);
    BOARD_DBUS_PORT->PCR[PIN2NUM(D1_PIN)]=0x00;
	BOARD_DBUS_PORT->PCR[PIN2NUM(D1_PIN)]|=PORT_PCR_MUX(PORT_mGPIO); 		       //Set MUX to GPIO;
    GPIO_PinInit(BOARD_DBUS_GPIO, PIN2NUM(D1_PIN), &in_config);
    BOARD_DBUS_PORT->PCR[PIN2NUM(D2_PIN)]=0x00;
	BOARD_DBUS_PORT->PCR[PIN2NUM(D2_PIN)]|=PORT_PCR_MUX(PORT_mGPIO); 		       //Set MUX to GPIO;
    GPIO_PinInit(BOARD_DBUS_GPIO, PIN2NUM(D2_PIN), &in_config);
    BOARD_DBUS_PORT->PCR[PIN2NUM(D3_PIN)]=0x00;
	BOARD_DBUS_PORT->PCR[PIN2NUM(D3_PIN)]|=PORT_PCR_MUX(PORT_mGPIO); 		       //Set MUX to GPIO;
    GPIO_PinInit(BOARD_DBUS_GPIO, PIN2NUM(D3_PIN), &in_config);
    BOARD_DBUS_PORT->PCR[PIN2NUM(D4_PIN)]=0x00;
	BOARD_DBUS_PORT->PCR[PIN2NUM(D4_PIN)]|=PORT_PCR_MUX(PORT_mGPIO); 		       //Set MUX to GPIO;
    GPIO_PinInit(BOARD_DBUS_GPIO, PIN2NUM(D4_PIN), &in_config);
    BOARD_DBUS_PORT->PCR[PIN2NUM(D5_PIN)]=0x00;
	BOARD_DBUS_PORT->PCR[PIN2NUM(D5_PIN)]|=PORT_PCR_MUX(PORT_mGPIO); 		       //Set MUX to GPIO;
    GPIO_PinInit(BOARD_DBUS_GPIO, PIN2NUM(D5_PIN), &in_config);
    BOARD_DBUS_PORT->PCR[PIN2NUM(D6_PIN)]=0x00;
	BOARD_DBUS_PORT->PCR[PIN2NUM(D6_PIN)]|=PORT_PCR_MUX(PORT_mGPIO); 		       //Set MUX to GPIO;
    GPIO_PinInit(BOARD_DBUS_GPIO, PIN2NUM(D6_PIN), &in_config);
    BOARD_DBUS_PORT->PCR[PIN2NUM(D7_PIN)]=0x00;
	BOARD_DBUS_PORT->PCR[PIN2NUM(D7_PIN)]|=PORT_PCR_MUX(PORT_mGPIO); 		       //Set MUX to GPIO;
    GPIO_PinInit(BOARD_DBUS_GPIO, PIN2NUM(D7_PIN), &in_config);
    
    hw_EnableInterrupts();
    NVIC_DisableIRQ(BOARD_VSYNC_IRQ);
    NVIC_DisableIRQ(BOARD_HREF_IRQ);

    while (1)
    {
        if(g_pic_ready)
        {
            // JPG Conversion
            int w = FRAME_WIDTH;
            int h = FRAME_HEIGHT;
            uint8_t *img = (uint8_t*)frame;
            jpec_enc_t *e = jpec_enc_new(img, w, h, enc_buffer);
            int len;
            const uint8_t *jpeg = jpec_enc_run(e, &len);
            PRINTF("Done: result (%d bytes)\n", len);

            // Pasar jpeg al server

            if(len > 9000)
            {
                len = len;
            }

            jpec_enc_del(e);
            g_pic_ready = false;
        }

        else if(!g_frame_requested && magreader_hasEvent())
        {
            if(magreader_getEvent() == MAGREADER_cardUpload)
            {
                g_frame_requested = true;
                GPIO_PortClearInterruptFlags(BOARD_VSYNC_GPIO, 1U << BOARD_VSYNC_GPIO_PIN);
                NVIC_ClearPendingIRQ(BOARD_VSYNC_IRQ);  //  Limpio pedidos previos al valido
                NVIC_EnableIRQ(BOARD_VSYNC_IRQ);        //  Habilito IRQ por nuevo frame
            }
        }

    }
}
