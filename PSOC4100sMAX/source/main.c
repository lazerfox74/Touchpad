/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PSoC 4 - SPI Master
 *             for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2020-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "SpiMaster.h"

#include "cycfg_capsense.h"


/*******************************************************************************
 * Macros
 ******************************************************************************/
/* Number of elements in the transmit buffer */
/* There are three elements - one for head, one for command and one for tail */
#define NUMBER_OF_ELEMENTS  (6UL)
#define SIZE_OF_ELEMENT     (1UL)
#define SIZE_OF_PACKET      (NUMBER_OF_ELEMENTS * SIZE_OF_ELEMENT)
#define DATA_LENGTH (3UL)

#define NOTE_X 0UL
#define NOTE_Y 1UL
#define NOTE_Z 2UL

#define STATE_OFF 0UL
#define STATE_ATT 1UL
#define STATE_HOLD 2UL
#define STATE_REL 3UL

uint8_t numActive;

bool sendSpi = false;
bool led_toggle = false;

#define sendSPI_INTR_PRIORITY      (1u)
#define TIMER_PERIOD_MSEC   100U

//spi priority is 2UL
#define CAPSENSE_MSC0_INTR_PRIORITY      (3u)
#define CAPSENSE_MSC1_INTR_PRIORITY      (3u)
#define CY_ASSERT_FAILED                 (0u)
#define MSC_CAPSENSE_WIDGET_INACTIVE     (0u)


/* CAPSENSE Touchad info */
cy_stc_capsense_touch_t *touchpad_touch_info;
uint16_t slider_pos = 0;
uint8_t touchpad_touch_status = 0;
uint16_t prev = -1;
#define numVoices 2


//note struct

#define noteThresh 25

typedef struct
{
	uint8_t x,y,z;
	uint8_t state;
	bool isHeld;
	uint8_t age,prevAge;
}notes;

notes myNotes[numVoices];



/*******************************************************************************
 * Function Name: main
 *******************************************************************************
 * Summary:
 * Entry function for the application. This function sets up the SPI Master.
 * SPI master sends commands to the slave to turn LED ON or OFF, every
 * second using high level APIs.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  int
 *
 ******************************************************************************/
static void initialize_capsense(void);
static void capsense_msc0_isr(void);
static void capsense_msc1_isr(void);

void processNoteData(cy_stc_capsense_position_t* note,uint8_t index);


void timer_interrupt_handler(void)
{

    //enable spi transfer
    sendSpi = true;

    /* Clear the terminal count interrupt */
    Cy_TCPWM_ClearInterrupt(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, CY_TCPWM_INT_ON_TC );
}



int main(void)
{
    cy_rslt_t result;
    cy_en_scb_spi_status_t status;
//    uint32_t cmd_send = CYBSP_LED_STATE_OFF;

    /* Buffer to hold command packet to be sent to the slave by the master */
    uint8_t txBuffer[NUMBER_OF_ELEMENTS];

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    //initialise notstruct
    for(int i =0;i < numVoices;i++)
    {
		myNotes[i].x=0;
		myNotes[i].y=0;
		myNotes[i].z=0;
		myNotes[i].state = STATE_OFF;
		myNotes[i].isHeld = false;
		myNotes[i].age = 255;
		myNotes[i].prevAge = 255;

    }




    /* Initialize the interrupt vector table with the timer interrupt handler
     * address and assign priority.
     */
    cy_stc_sysint_t intrCfg =
    {
        /*.intrSrc =*/ CYBSP_TIMER_IRQ,    /* Interrupt source is Timer interrupt */
        /*.intrPriority =*/ 1UL            /* Interrupt priority is 3 */
    };

    result = Cy_SysInt_Init(&intrCfg, timer_interrupt_handler);
    if(result != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable Interrupt */
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Initialize the SPI Master */
    result = initMaster();
    /* Initialization failed. Stop program execution */
    if(result != INIT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();


    /* Initialize the TCPWM component in timer/counter mode. The return value of the
     * function indicates whether the arguments are valid or not.
     */
    result = Cy_TCPWM_Counter_Init(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, &CYBSP_TIMER_config);
    if(result != CY_TCPWM_SUCCESS)
    {
        CY_ASSERT(0);
    }

    Cy_TCPWM_Counter_Enable(CYBSP_TIMER_HW, CYBSP_TIMER_NUM);


    /* Check if the desired interrupt is enabled prior to triggering */
    if (0UL != (CY_TCPWM_INT_ON_TC & Cy_TCPWM_GetInterruptMask(CYBSP_TIMER_HW, CYBSP_TIMER_NUM)))
    {
       Cy_TCPWM_SetInterrupt(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, CY_TCPWM_INT_ON_TC);
    }
    /* Set the timer period in milliseconds. To count N cycles, period should be
     * set to N-1.
     */
    Cy_TCPWM_Counter_SetPeriod(CYBSP_TIMER_HW, CYBSP_TIMER_NUM, TIMER_PERIOD_MSEC-1 );

    /* Trigger a software start on the counter instance. This is required when
     * no other hardware input signal is connected to the component to act as
     * a trigger source.
     */
    Cy_TCPWM_TriggerStart(CYBSP_TIMER_HW, CYBSP_TIMER_MASK);

    /* Initialize MSC CapSense */
    initialize_capsense();

    //start first scan
    Cy_CapSense_ScanAllSlots(&cy_capsense_context);






    for(;;)
    {

//    	if(led_toggle ==true)
//    	{
//    		  Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, CYBSP_LED_STATE_ON);
//    	}
    	//capsense stuff
    if (CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
    {
    	/* Process all widgets */
    	Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

        touchpad_touch_info = Cy_CapSense_GetTouchInfo(CY_CAPSENSE_TOUCHPAD0_WDGT_ID, &cy_capsense_context);
        touchpad_touch_status = touchpad_touch_info->numPosition;
//        if(touchpad_touch_status> 2){touchpad_touch_status = 2;}

    	/* Turns LED ON/OFF based on Touchpad status */
    	//led_control();
    	if (MSC_CAPSENSE_WIDGET_INACTIVE != Cy_CapSense_IsWidgetActive(CY_CAPSENSE_TOUCHPAD0_WDGT_ID, &cy_capsense_context))
    	{
    		Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, CYBSP_LED_STATE_ON);

    	}
    	else
        {
    		Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, CYBSP_LED_STATE_OFF);
    		//set all z index to zero
//            txBuffer[2] = 0;
//            txBuffer[5] = 0;
//            txBuffer[8] = 0;
        }

        for(uint8_t i = 0; i < numVoices;i++)
        {
        	cy_stc_capsense_position_t* noteToProcess = touchpad_touch_info->ptrPosition;
        	processNoteData(noteToProcess + i,i);

    //				cy_stc_capsense_touch_t* notetoChoose;
    //
    //				notetoChoose = touchpad_touch_info->ptrPosition;
    //    		txBuffer[NOTE_X + index] = touchpad_touch_info->ptrPosition[i].y;
    //    		txBuffer[NOTE_Y + index] = touchpad_touch_info->ptrPosition[i].x;
    //    		txBuffer[NOTE_Z + index] = touchpad_touch_info->ptrPosition[i].z
        }



//        for(int i = 0;i < numVoices;i++)
//        {
//        	if(myNotes[i].age == myNotes[i].prevAge)
//        	{
//        		myNotes[i].z = 0;
//        	}
//        	else
//        	{
//        		myNotes[i].prevAge = myNotes[i].age;
//        	}
//        }


    	Cy_CapSense_ScanAllSlots(&cy_capsense_context);
    }

    //clear note if inactive










            /* Establishes synchronized communication with the CapSense Tuner tool */
//            Cy_CapSense_RunTuner(&cy_capsense_context);
    		if(sendSpi==true)
    		{
    			for(int i =0; i < numVoices; i++)
    			{
    				uint8_t index = i * DATA_LENGTH;
        			txBuffer[index + NOTE_X] = myNotes[i].y;
        			txBuffer[index + NOTE_Y] = myNotes[i].x;
        			txBuffer[index + NOTE_Z] = myNotes[i].z;
//    				        			txBuffer[index + NOTE_X] = index + NOTE_X;
//    				        			txBuffer[index + NOTE_Y] = index + NOTE_Y;
//    				        			txBuffer[index + NOTE_Z] = index + NOTE_Z;


    			}

    			status = sendPacket(txBuffer, SIZE_OF_PACKET);

    			if(status != CY_SCB_SPI_SUCCESS)
    			{
    				CY_ASSERT(0);
    			}

				sendSpi = false;
    		}



            /* Start the next scan */






            /* Toggles GPIO for refresh rate measurement. Probe at P10.4. */
//            Cy_GPIO_Inv(CYBSP_SENSE_SCAN_RATE_PORT, CYBSP_SENSE_SCAN_RATE_NUM);
    }





//    	//spi stuff
//


//
//        /* Send the packet to the slave */
//        status = sendPacket(txBuffer, SIZE_OF_PACKET);
//        if(status != CY_SCB_SPI_SUCCESS)
//        {
//            CY_ASSERT(0);
//        }



        /* Delay between commands */
//        Cy_SysLib_Delay(1000);

}


/*******************************************************************************
* Function Name: initialize_capsense
********************************************************************************
* Summary:
*  This function initializes the CapSense Blocks and configures the CapSense
*  interrupt.
*
*******************************************************************************/
static void initialize_capsense(void)
{
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;

    /* CapSense interrupt configuration MSC 0 */
    const cy_stc_sysint_t capsense_msc0_interrupt_config =
    {
        .intrSrc = CY_MSC0_IRQ,
        .intrPriority = CAPSENSE_MSC0_INTR_PRIORITY,
    };

    /* CapSense interrupt configuration MSC 1 */
    const cy_stc_sysint_t capsense_msc1_interrupt_config =
    {
        .intrSrc = CY_MSC1_IRQ,
        .intrPriority = CAPSENSE_MSC1_INTR_PRIORITY,
    };

    /* Capture the MSC HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);

    if (status != CY_CAPSENSE_STATUS_SUCCESS)
    {
        /* CapSense initialization failed, the middleware may not operate
         * as expected, and repeating of initialization is required.*/
        CY_ASSERT(CY_ASSERT_FAILED);


    }

    if (CY_CAPSENSE_STATUS_SUCCESS == status)
    {
        /* Initialize CapSense interrupt for MSC 0 */
        Cy_SysInt_Init(&capsense_msc0_interrupt_config, capsense_msc0_isr);
        NVIC_ClearPendingIRQ(capsense_msc0_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_msc0_interrupt_config.intrSrc);

        /* Initialize CapSense interrupt for MSC 1 */
        Cy_SysInt_Init(&capsense_msc1_interrupt_config, capsense_msc1_isr);
        NVIC_ClearPendingIRQ(capsense_msc1_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_msc1_interrupt_config.intrSrc);

        /* Initialize the CapSense firmware modules. */
        status = Cy_CapSense_Enable(&cy_capsense_context);
    }

    if (status != CY_CAPSENSE_STATUS_SUCCESS)
    {
        /* This status could fail before tuning the sensors correctly.
         * Ensure that this function passes after the CapSense sensors are tuned
         * as per procedure give in the Readme.md file */
    }
}


/*******************************************************************************
* Function Name: capsense_msc0_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CapSense MSC0 block.
*
*******************************************************************************/
static void capsense_msc0_isr(void)
{
    Cy_CapSense_InterruptHandler(CY_MSC0_HW, &cy_capsense_context);
}


/*******************************************************************************
* Function Name: capsense_msc1_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CapSense MSC1 block.
*
*******************************************************************************/
static void capsense_msc1_isr(void)
{
    Cy_CapSense_InterruptHandler(CY_MSC1_HW, &cy_capsense_context);
}


//void toggle_led_on_systick_handler(void)
//{
//    /* toggle led */
//	sendSpi = true;
//}

void processNoteData(cy_stc_capsense_position_t* note,uint8_t index)
{
	uint16_t noteId = note->id;

	uint8_t idDebounce = (noteId >> 8) & 0xFF;  // Upper 8 bits: 0xAB
	uint8_t touchID = noteId & 0xFF;  // Lower 8 bits: 0xCD.

	notes* activeNote = &myNotes[index];


	uint16_t noteZ = (note->z);  // Extracts the 8 least significant bits.

//	uint8_t age = (noteZ >> 8)&0xFF;
	uint8_t z_value = (noteZ & 0xFF);  // Extracts the 8 least significant bits.
//	if(age == 255)
//	{
//		note->z = 0;
//	}
//	if(z_value < noteThresh)
//	{
//		activeNote->z = 0;
//	}

	if(touchID == 128)
	{
		activeNote->z = 0;
	}
	else
	{
		activeNote->z = z_value;
		activeNote->x = note->x;
		activeNote->y = note->y;
//		activeNote->age = age;
	}



	//check for off
//	if(activeNote->isHeld == true && note->z <= noteThresh)
//	{
//		activeNote->isHeld = false;
//		activeNote->z = 0;
//	}
//	else if(activeNote->isHeld == false && note->z >=noteThresh)
//	{
//		activeNote->isHeld = true;
//		activeNote->z = note->z;
//		activeNote->x = note->x;
//		activeNote->y = note->y;
//	}
//	else if(activeNote->isHeld == true && note->z >= noteThresh)
//	{
//		activeNote->z = note->z;
//		activeNote->x = note->x;
//		activeNote->y = note->y;
//	}
//	else
//	{
//		activeNote->z = ;
//	}
}









/* [] END OF FILE */
