/******************************************************************************
* File Name: main.c
*
* Related Document: See Readme.md
*
* Description: This  project reads the temperature and pressure from the
* 			   Infineon DPS368 Barometric Pressure Sensor
*
*******************************************************************************
* (c) 2020, Arrow Electronics. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Arrow hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Infineon's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Arrow.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "app_config.h"
#include "dps368.h"

/***************************************
*            Constants
****************************************/
#define SPI_FREQ_HZ                (1000000UL)
#define CMD_TO_CMD_DELAY_MS        (1000UL)
#define STATUS_REG_READ_DELAY_MS   (1L)


cyhal_spi_t mSPI;

/*******************************************************************************
* Function Name: main_handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void main_handle_error(void)
{
	printf("main.c Error\r\n");
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU.
*   1. SPI Master sends command packet to the slave
*   2. Slave reads the packet and executes the instructions
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    uint32_t pressure;
    uint32_t temperature;
    uint8_t  sensor_ready;
    uint8_t  pressure_ready;
    uint8_t  temperature_ready;
    uint8_t	 wait_count;
 	float temp_flt ;
 	float prs_flt;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    if (result != CY_RSLT_SUCCESS)
    {
        main_handle_error();
    }

    result = cy_retarget_io_init( CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, 
                                  CY_RETARGET_IO_BAUDRATE);
    
    if (result != CY_RSLT_SUCCESS)
    {
        main_handle_error();
    }

    result = cyhal_gpio_init( CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
                              CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);


    if (result != CY_RSLT_SUCCESS)
    {
        main_handle_error();
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("**********************************\r\n");
    printf("PSoC 6 MCU: SPI Master            \r\n");
    printf("DPS368 Barometric Pressure Sensor \r\n");
    printf("**********************************\r\n\n");
    
    /* Init LED */
    uint32_t cmd_send = CYBSP_LED_STATE_OFF;
    cyhal_gpio_write( CYBSP_USER_LED, cmd_send);

    /* Configure SPI Master */
    printf("Configuring SPI master...\r\n");
    result = cyhal_spi_init( &mSPI, mSPI_MOSI, mSPI_MISO, mSPI_SCLK, 
                             mSPI_SS, NULL, 8, CYHAL_SPI_MODE_11_MSB, false);

    if (result != CY_RSLT_SUCCESS)
    {
        main_handle_error();
    }

    result = cyhal_spi_set_frequency( &mSPI, SPI_FREQ_HZ);
    if (result != CY_RSLT_SUCCESS)
    {
        main_handle_error();
    }
    /* Enable interrupts */
    __enable_irq();

    DPS368_Init();

    /* Check if sensor is ready and initialized */
    sensor_ready = SENSOR_RDY & Get_DPS368_Sensor_Status();

    while (!sensor_ready)
    {
    	sensor_ready = SENSOR_RDY & Get_DPS368_Sensor_Status();
    }
    printf("DPS368 Ready\r\n");

    for (;;)
    {
       /* Toggle the current LED state */
        cmd_send = (cmd_send == CYBSP_LED_STATE_OFF) ? CYBSP_LED_STATE_ON : CYBSP_LED_STATE_OFF;
        cyhal_gpio_write( CYBSP_USER_LED, cmd_send);



        /* Get temperature */

         Set_DPS368_Sensor_Operating_Mode(START_TEMPERATURE_MEASUREMENT);
         wait_count = 0;
         temperature_ready = TMP_RDY & Get_DPS368_Sensor_Status();
         while (!temperature_ready)
         {
        	 cyhal_system_delay_ms(STATUS_REG_READ_DELAY_MS);
        	 temperature_ready = (TMP_RDY & Get_DPS368_Sensor_Status());
         	 wait_count++;
        	 if (wait_count > 100)
        	 {
        		 printf("Temp Ready Timeout \r\n");
        		 Set_DPS368_Sensor_Operating_Mode(START_TEMPERATURE_MEASUREMENT);
        		 wait_count = 0;
        	 }
        	 cyhal_system_delay_ms(STATUS_REG_READ_DELAY_MS);
         }
         temperature = Get_DPS368_Temperature();

         Set_DPS368_Sensor_Operating_Mode(STANDBY);
         cyhal_system_delay_ms(STATUS_REG_READ_DELAY_MS);
         temp_flt = Calculate_Temperature(temperature);


         /* Get pressure */
        Set_DPS368_Sensor_Operating_Mode(START_PRESSURE_MEASUREMENT);
        pressure_ready = PRS_RDY & Get_DPS368_Sensor_Status();
        wait_count = 0;
        while (!pressure_ready)
        {
        	cyhal_system_delay_ms(STATUS_REG_READ_DELAY_MS);
        	pressure_ready = (PRS_RDY & Get_DPS368_Sensor_Status());
        	wait_count++;
        	if (wait_count > 100)
        	{
        		Set_DPS368_Sensor_Operating_Mode(STANDBY);
        		cyhal_system_delay_ms(STATUS_REG_READ_DELAY_MS);
        		Set_DPS368_Sensor_Operating_Mode(START_PRESSURE_MEASUREMENT);
        		wait_count = 0;
        	}
        }
        pressure = Get_DPS368_Pressure();


    	prs_flt = Calculate_Pressure(pressure);
     	printf("Pressure %4.1f Temperature %4.1f  \r\n", prs_flt, (temp_flt));

        Set_DPS368_Sensor_Operating_Mode(STANDBY);


//         printf("Pressure %d Temperaure %d  \r\n", (unsigned int) pressure, (unsigned int) temperature);

      /* delay between loops */
        cyhal_system_delay_ms(CMD_TO_CMD_DELAY_MS);

    }
}
