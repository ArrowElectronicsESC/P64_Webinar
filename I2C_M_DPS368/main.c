/******************************************************************************
* File Name: main.c
*
* Related Document: See Readme.md
*
* Description: This example project demonstrates the basic operation of the
* I2C resource as Master using HAL APIs. The I2C master sends the
* command packets to the I2C slave to control an user LED.
*
*******************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
#include "resource_map.h"
#include "dps368.h"

/***************************************
*            Constants
****************************************/
#define CMD_TO_CMD_DELAY        (1000UL)
#define STATUS_REG_READ_DELAY_MS   (1L)

/* I2C slave address to communicate with */
#define I2C_SLAVE_ADDR          (0x76UL)

/* I2C bus frequency */
#define I2C_FREQ                (400000UL)

/***************************************
*          Global Variables
****************************************/
cyhal_i2c_t mI2C;
cyhal_i2c_cfg_t mI2C_cfg;

/*******************************************************************************
* Function Name: handle_error
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
void handle_error(void)
{
	printf("Error in main.c ");
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU.
*   1. I2C Master sends command packet to the slave
*   2. I2C Master reads the response packet to generate the next command
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
    uint8_t	 first_prs;
 	float temp_flt ;
 	float prs_flt;
 	float last_prs_flt;

    /* Set up the device based on configurator selections */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }
    result = cy_retarget_io_init( CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, 
                                  CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************************\r\n");
    printf("PSoC6 MCU with DPS368 \r\n");
    printf("Barometric Pressure Sensor\r\n");
    printf("****************************\r\n\n");

    /* Init LED */
    result = cyhal_gpio_init( CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
                              CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    cyhal_gpio_write( CYBSP_USER_LED, 0);

    last_prs_flt = 0;
    first_prs = 0;

    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }

    /* Configure I2C Master */

#if PRINT_DEBUG
    printf(">> Configuring I2C master..... ");
#endif

    mI2C_cfg.is_slave = false;
    mI2C_cfg.address = 0;
    mI2C_cfg.frequencyhal_hz = I2C_FREQ;
    result = cyhal_i2c_init( &mI2C, mI2C_SDA, mI2C_SCL, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }
    result = cyhal_i2c_configure( &mI2C, &mI2C_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }
#if PRINT_DEBUG
    printf("Done\r\n");
#endif

    /* Enable interrupts */
    __enable_irq();
 
    printf("Init DPS368 \r\n");
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

     	/* This is a delta pressure calc. Print the 2nd pressure read and after */
     	if (first_prs)
     	{
     		printf("Pressure %4.1f \r\n", prs_flt);
     		printf("Delta P %4.1f \t%4.1f F \r\n", prs_flt- last_prs_flt, ((temp_flt *9/5) +32));
     	}
     	if (prs_flt- last_prs_flt > 80)
     	{
     		cyhal_gpio_write( CYBSP_USER_LED, CYBSP_LED_STATE_ON);
     		printf("Fall Event \r\n");
     	}
     	else
     	{
     		cyhal_gpio_write( CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
     	}
     	last_prs_flt = prs_flt;
     	first_prs = 1;

     	Set_DPS368_Sensor_Operating_Mode(STANDBY);

        cyhal_system_delay_ms(500);

    }
}
