/***************************************************************************//**
  @file     DMA.c
  @brief    Source File DMA
  @author   Grupo 4
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "DMA.h"
#include "MK64F12.h"
#include "hardware.h"
#include "myboard.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

/* Auxiliary variable used to modify the source buffer on each iteration. */
static uint8_t MinorTransferDone = 0;

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/


/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/


/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static uint16_t* destinationBufferSave;
static uint16_t dumpBuffer;
static uint32_t p_irq_counter;

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void DMA_init(uint16_t* destinationBuffer)
{
	destinationBufferSave = destinationBuffer;

	/* Enable the clock for the PORT B and the PORT E. */
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	// /* Configure the MUX option. */

	PORTB->PCR[21] |= PORT_PCR_MUX(PORT_mGPIO);

	/* Turn all the LEDs off. (Active low)*/
	PTB->PSOR |= (1 << 21);
	// PTE->PSOR |= (1 << PIN_LED_GREEN);

	/* Select LEDs as outputs. */
	PTB->PDDR |= (1 << 21);

	/**************************************************************************/
	/***** Configure the FRDM-K64F PTB9 (PCLK) as the DMA request source. *****/

	PORTB->PCR[9] = 0x00;
	PORTB->PCR[9] |= PORT_PCR_MUX(PORT_mGPIO); 		       //Set MUX to GPIO;
	PORTB->PCR[9] |= PORT_PCR_IRQC(PORT_eDMARising);

	/* Enable the clock for the eDMA and the DMAMUX. */
	SIM->SCGC7 |= SIM_SCGC7_DMA_MASK;
	SIM->SCGC6 |= SIM_SCGC6_DMAMUX_MASK;

	/* Enable the eDMA channel 0 and set the PORT B as the DMA request source. */
	DMAMUX->CHCFG[0] |= DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(50);   // PORT B 

	NVIC_DisableIRQ(DMA0_IRQn);

	// /* Clear all the pending events. */
	// NVIC_ClearPendingIRQ(DMA0_IRQn);
	// /* Enable the DMA interrupts. */
	// NVIC_EnableIRQ(DMA0_IRQn);


	/// ============= INIT TCD0 ===================//
	/* Set memory address for source and destination. */
	DMA0->TCD[0].SADDR = (uint32_t)(&(GPIOC->PDIR));
	DMA0->TCD[0].DADDR = (uint32_t)(&dumpBuffer);

		/* Set an offset for source and destination address. */
	DMA0->TCD[0].SOFF =0x00; // Source address offset of 2 bytes per transaction.
	DMA0->TCD[0].DOFF =0x00; // Destination address offset of 2 byte per transaction.

	/* Set source and destination data transfer size is 2 bytes. */
	DMA0->TCD[0].ATTR = DMA_ATTR_SSIZE(1) | DMA_ATTR_DSIZE(1);

	/*Number of bytes to be transfered in each service request of the channel.*/
	DMA0->TCD[0].NBYTES_MLNO= 0x02;

	/* Current major iteration count (5 iteration of 1 byte each one). */
	DMA0->TCD[0].CITER_ELINKNO = DMA_CITER_ELINKNO_CITER(0x01);
	DMA0->TCD[0].BITER_ELINKNO = DMA_BITER_ELINKNO_BITER(0x01);



	/* Address for the next TCD to be loaded in the scatter/gather mode. */
	//	tcd[TCD0].SLAST = 0;			// Source address adjustment not used.
	//	tcd[TCD0].DLASTSGA = (uint32_t)&tcd[TCD1];	// The tcd[TCD1] is the next TCD to be loaded.


	DMA0->TCD[0].SLAST = 0x00;
	DMA0->TCD[0].DLAST_SGA = (uint32_t)&(DMA0->TCD[1]);

	/* Setup control and status register. */

	DMA0->TCD[0].CSR = DMA_CSR_ESG_MASK;	//Enable scatter and gather.


	/// ============= INIT TCD1 ===================//
		/* Set memory address for source and destination. */
	DMA0->TCD[1].SADDR = (uint32_t)(&(GPIOC->PDIR));
	DMA0->TCD[1].DADDR = (uint32_t)(destinationBuffer);

		/* Set an offset for source and destination address. */
	DMA0->TCD[1].SOFF =0x02; // Source address offset of 2 bytes per transaction.
	DMA0->TCD[1].DOFF =0x02; // Destination address offset of 2 byte per transaction.

	/* Set source and destination data transfer size is 2 bytes. */
	DMA0->TCD[1].ATTR = DMA_ATTR_SSIZE(1) | DMA_ATTR_DSIZE(1);

	/*Number of bytes to be transfered in each service request of the channel.*/
	DMA0->TCD[1].NBYTES_MLNO= 0x02;

	/* Current major iteration count (5 iteration of 1 byte each one). */
	DMA0->TCD[1].CITER_ELINKNO = DMA_CITER_ELINKNO_CITER(0x01);
	DMA0->TCD[1].BITER_ELINKNO = DMA_BITER_ELINKNO_BITER(0x01);



	/* Address for the next TCD to be loaded in the scatter/gather mode. */
	//	tcd[TCD0].SLAST = 0;			// Source address adjustment not used.
	//	tcd[TCD0].DLASTSGA = (uint32_t)&tcd[TCD1];	// The tcd[TCD1] is the next TCD to be loaded.


	DMA0->TCD[1].SLAST = 0x00;
	DMA0->TCD[1].DLAST_SGA = (uint32_t)&(DMA0->TCD[2]);

	/* Setup control and status register. */

	DMA0->TCD[1].CSR = DMA_CSR_INTMAJOR_MASK | DMA_CSR_ESG_MASK;	//Enable Major Interrupt, enable scatter and gather.
	
	/// ============= INIT TCD2 ===================//
	/* Set memory address for source and destination. */

	DMA0->TCD[2].SADDR= (uint32_t)(&(GPIOC->PDIR));
	DMA0->TCD[2].DADDR = (uint32_t)(&dumpBuffer);

		/* Set an offset for source and destination address. */
	DMA0->TCD[2].SOFF =0x02; // Source address offset of 2 bytes per transaction.
	DMA0->TCD[2].DOFF =0x00; // Destination address offset of 0 bytes per transaction.

	/* Set source and destination data transfer size is 2 bytes. */
	DMA0->TCD[2].ATTR = DMA_ATTR_SSIZE(1) | DMA_ATTR_DSIZE(1);

	/*Number of bytes to be transfered in each service request of the channel.*/
	DMA0->TCD[2].NBYTES_MLNO= 0x02;

	/* Current major iteration count (5 iteration of 1 byte each one). */
	DMA0->TCD[2].CITER_ELINKNO = DMA_CITER_ELINKNO_CITER(0x03);
	DMA0->TCD[2].BITER_ELINKNO = DMA_BITER_ELINKNO_BITER(0x03);


	/* Address for the next TCD to be loaded in the scatter/gather mode. */
	//	tcd[TCD0].SLAST = 0;			// Source address adjustment not used.
	//	tcd[TCD0].DLASTSGA = (uint32_t)&tcd[TCD1];	// The tcd[TCD1] is the next TCD to be loaded.


	DMA0->TCD[2].SLAST = 0x00;
	DMA0->TCD[2].DLAST_SGA = (uint32_t)&(DMA0->TCD[1]);

	/* Setup control and status register. */

	DMA0->TCD[2].CSR = DMA_CSR_ESG_MASK;

	return ;
}

/* The blue LED is toggled when a TCD is completed. */
void DMA0_IRQHandler()
{
	/* Clear the interrupt flag. */
	DMA0->CINT |= 0;

	/* Toggle the blue LED. */
	PTB->PCOR |= (1 << 21);

	DMA0->TCD[1].DADDR += 2;	//	Increment buffer pointer

	p_irq_counter++;

	/* Change the source buffer contents. */
	MinorTransferDone = 1;
}

/* The red LED is toggled when an error occurs. */
void DMA_Error_IRQHandler()
{
	/* Clear the error interrupt flag.*/
	DMA0->CERR |= 0;
}

void DMA_Enable_Requests()
{
	/* Clear all the pending events. */
	NVIC_ClearPendingIRQ(DMA0_IRQn);
	/* Enable the DMA interrupts. */
	NVIC_EnableIRQ(DMA0_IRQn);
	/* Enable request signal for channel 0. */
	DMA0->ERQ = DMA_ERQ_ERQ0_MASK;
}

void DMA_Disable_Requests()
{
	NVIC_DisableIRQ(DMA0_IRQn);
	/* Disable request signal for channel 0. */
	DMA0->ERQ = 0x0U;
}

void DMA_Reset_DADDR()
{
	DMA0->TCD[1].DADDR = (uint32_t)(destinationBufferSave);
}

void DMA_Reset_TCD()
{
	/// ============= INIT TCD0 ===================//
	/* Set memory address for source and destination. */
	DMA0->TCD[0].SADDR = (uint32_t)(&(GPIOC->PDIR));
	DMA0->TCD[0].DADDR = (uint32_t)(&dumpBuffer);

	/* Set an offset for source and destination address. */
	DMA0->TCD[0].SOFF =0x00; // Source address offset of 2 bytes per transaction.
	DMA0->TCD[0].DOFF =0x00; // Destination address offset of 2 byte per transaction.

	/* Set source and destination data transfer size is 2 bytes. */
	DMA0->TCD[0].ATTR = DMA_ATTR_SSIZE(1) | DMA_ATTR_DSIZE(1);

	/*Number of bytes to be transfered in each service request of the channel.*/
	DMA0->TCD[0].NBYTES_MLNO= 0x02;

	/* Current major iteration count (5 iteration of 1 byte each one). */
	DMA0->TCD[0].CITER_ELINKNO = DMA_CITER_ELINKNO_CITER(0x01);
	DMA0->TCD[0].BITER_ELINKNO = DMA_BITER_ELINKNO_BITER(0x01);

	DMA0->TCD[0].SLAST = 0x00;
	DMA0->TCD[0].DLAST_SGA = (uint32_t)&(DMA0->TCD[1]);

	/* Setup control and status register. */
	DMA0->TCD[0].CSR = 0;
	DMA0->TCD[0].CSR = DMA_CSR_ESG_MASK;	//Enable scatter and gather.
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/
