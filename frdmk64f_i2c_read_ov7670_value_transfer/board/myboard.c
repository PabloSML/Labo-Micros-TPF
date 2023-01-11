/***************************************************************************//**
  @file     board.c
  @brief    Board controller.
  @author   Grupo 4
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "myboard.h"
#include "MK64F12.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/


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



/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

/*********** LED init & services ****************/
bool boardInit(void)
{
  static bool yaInit = false;
    
  if (!yaInit) // init peripheral
  {
    //Enable clocking for port B,A,E
    // SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
    // SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    // SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    // SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    // PORT_Type * portpointer[] = PORT_BASE_PTRS;
    // portpointer[PB]->ISFR |= 0xFFFFU;
    
    // NVIC_EnableIRQ(PORTB_IRQn);

  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

  PORTA->ISFR = PORT_ISFR_ISF_MASK;
  PORTB->ISFR = PORT_ISFR_ISF_MASK;
  PORTC->ISFR = PORT_ISFR_ISF_MASK;
  PORTD->ISFR = PORT_ISFR_ISF_MASK;
  PORTE->ISFR = PORT_ISFR_ISF_MASK;

	NVIC_EnableIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTB_IRQn);
	NVIC_EnableIRQ(PORTC_IRQn);
	NVIC_EnableIRQ(PORTD_IRQn);
	NVIC_EnableIRQ(PORTE_IRQn);

    yaInit = true;
  }

  return yaInit;
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

/******************************************************************************/
