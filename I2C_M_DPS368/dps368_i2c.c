/***************************************************************************
* Filename dps368_i2c.c
*
*  Created on: 8/5/2020
*      Author: Mike Roberts
*
* Description: This project has function calls for the Infineon DPS368
* 			   Barometric Pressure Sensor
*
*******************************************************************************
* (c) 2020, Arrow Electronics. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Arrow Electronics or one of its
* subsidiaries ("Arrow") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Arrow hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
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
 */

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "resource_map.h"
#include "dps368.h"

/* Global Variables */
cyhal_i2c_t mI2C;
cy_rslt_t result;

/* structure to store the compensation coefficients */
struct	coef	DPS368_Coef;

/* Scaling Factors from the DPS368 data sheet Section 4.9.3 page 16 */
int32_t scaling_factor[8] = {524288, 1572864, 3670016, 7844320, 253952, 516096, 1040384, 2088960};

float Traw_sc;		// for the scale raw temperature data used in calculating pressure

/*******************************************************************************
* Function Name: DPS-368_handle_error
********************************************************************************
* Summary:
* Defined error handling function
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void DPS368_handle_error(void)
{
	printf("DPS368 Error\r\n");
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

/*******************************************************************************
* Function Name: DPS368_Init
********************************************************************************
* Summary:
* Initialize the DPS368
*  - Check the Product ID = 0x10
*  - Get the Coefficient Source
*  - Get the Coefficients and convert from bit pack to extended
*  - Set the Pressure Parameters
*  - Set the Temperature Parameters
*  - Go into Standby / Idle Mode
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void DPS368_Init()
{
	uint8_t	product_ID;
	uint8_t	tmp_coef_source;
	uint8_t	coef_ready;
	uint8_t config_data;

    /* Get DPS368 Product Revision and ID -  should be 0x10 */
    product_ID = Get_DPS368_ID();

#if PRINT_DEBUG
    printf("Product ID = %x \r\n", product_ID);
#endif
    if(product_ID != 0x10)
    {
    	printf("Wrong Product ID\r\n");
    	DPS368_handle_error();
    }

    /* Get DPS368 Temperature Coefficient Source */
    tmp_coef_source = Get_DPS368_Coef_Source();

#if PRINT_DEBUG
    if (!tmp_coef_source){
    	printf("Temp Sensor Internal \r\n"); 	}
    else{
    	printf("Temp Sensor = MEMS Sensor \r\n"); 	}

    printf("Read Coefficients \r\n");
#endif

    /* Get the Coeffients from registers 0x10 to 0x21 */
    coef_ready = (COEF_RDY & Get_DPS368_Sensor_Status());
    while (!coef_ready)
    {
    	coef_ready = (COEF_RDY & Get_DPS368_Sensor_Status());
    }
    Get_DPS368_Coefficients();

    /* Set to standby for further configuration */
    Set_DPS368_Sensor_Operating_Mode(STANDBY);

    /*  Set Pressure --  Measurement Rate = Bit6-4, Oversampling Rate = Bit3-0 */
    config_data = (PRS_PM << 4)|(PRS_RATE);
    Set_DPS368_Pressure_Config(config_data);

#if PRINT_DEBUG

    printf("PRS Config Datq = %x \r\n", config_data);

#endif

    /*  Set Temp Measurement  - Temp Coef Source = Bit7, Measurement Rate = Bit6-4, Oversampling Rate = Bit3-0 */
    config_data =  (TMP_EXT_MASK & tmp_coef_source) | (TEMP_PM << 4) | TEMP_RATE  ;

    Set_DPS368_Temperature_Config(config_data);

#if PRINT_DEBUG

    printf("TMP Config Datq = %x \r\n", config_data);

#endif

    /* No Interrupts and no FIFO */
    config_data = 0x00;

    /* Check if Oversampling rate > 8, then the shift bit needs to be set */
    if(PRS_RATE > OVERSAMPLE_RATE_8)
     {
     	config_data = config_data | P_SHIFT;
     }
    if(TEMP_RATE > OVERSAMPLE_RATE_8)
     {
     	config_data = config_data | T_SHIFT;
     }
    Set_DPS368_Intr_FIFO_Config(config_data);

#if PRINT_DEBUG

    printf("CFG Data = %x \r\n", config_data);

#endif

    Set_DPS368_Sensor_Operating_Mode(STANDBY);
}

/*******************************************************************************
* Function Name: Get_DPS368_ID
********************************************************************************
* Summary:
*  Get the Product Revision and ID
*
* Parameters:
*  void
*
* Return:
*  product_ID
*
*******************************************************************************/
uint8_t Get_DPS368_ID()
{

	uint8_t 	send_buffer;
	uint8_t 	rcv_buffer;

	send_buffer = ID;

  	result = cyhal_i2c_master_write( &mI2C, I2C_SLAVE_ADDR, &send_buffer, SEND_ONE_BYTE, 0, SEND_RESTART);
   	result = cyhal_i2c_master_read( &mI2C, I2C_SLAVE_ADDR,  &rcv_buffer, RCV_ONE_BYTE , 0, SEND_STOP);

    if (CY_RSLT_SUCCESS != result)
    {
       DPS368_handle_error();
    }

 	return (rcv_buffer);
 }

/*******************************************************************************
* Function Name: Get_DPS368_Pressure
********************************************************************************
* Summary:
*  Get the pressure
*
* Parameters:
*  void
*
* Return:
*  raw pressure
*
*******************************************************************************/
uint32_t Get_DPS368_Pressure()
{
	uint32_t	psr_b2, psr_b1, psr_b0;
	uint8_t 	send_data;
	uint8_t 	rcv_data;

    /* Read MSB */
	send_data = PSR_B2;
	result = cyhal_i2c_master_write( &mI2C, I2C_SLAVE_ADDR, &send_data, SEND_ONE_BYTE, 0, SEND_RESTART);
	result = cyhal_i2c_master_read( &mI2C, I2C_SLAVE_ADDR,  &rcv_data, RCV_ONE_BYTE , 0, SEND_STOP);
	psr_b2 = rcv_data;
    if (CY_RSLT_SUCCESS != result)
    {
       DPS368_handle_error();
    }

    /* Read LSB */
    send_data = PSR_B1;
	result = cyhal_i2c_master_write( &mI2C, I2C_SLAVE_ADDR, &send_data, SEND_ONE_BYTE, 0, SEND_RESTART);
	result = cyhal_i2c_master_read( &mI2C, I2C_SLAVE_ADDR,  &rcv_data, RCV_ONE_BYTE , 0, SEND_STOP);
	psr_b1 = rcv_data;

    if (CY_RSLT_SUCCESS != result)
    {
       DPS368_handle_error();
    }

    /* Read XLSB */
    send_data = PSR_B0;
	result = cyhal_i2c_master_write( &mI2C, I2C_SLAVE_ADDR, &send_data, SEND_ONE_BYTE, 0, SEND_RESTART);
	result = cyhal_i2c_master_read( &mI2C, I2C_SLAVE_ADDR,  &rcv_data, RCV_ONE_BYTE , 0, SEND_STOP);
	psr_b0 = rcv_data;

    if (CY_RSLT_SUCCESS != result)
    {
       DPS368_handle_error();
    }

 	return ((uint32_t)(psr_b2 <<16 | psr_b1 << 8 | psr_b0));
}

/*******************************************************************************
* Function Name: Get_DPS368_temperature
********************************************************************************
* Summary:
* User Get the pressure
*
* Parameters:
*  void
*
* Return:
*  raw temperature
*
*******************************************************************************/
uint32_t Get_DPS368_Temperature()
{
	uint32_t	tmp_b2, tmp_b1, tmp_b0;
	uint8_t		send_data;
	uint8_t		rcv_data;

    /* Read MSB */
	send_data = TMP_B2;
	result = cyhal_i2c_master_write( &mI2C, I2C_SLAVE_ADDR, &send_data, SEND_ONE_BYTE, 0, false);
	result = cyhal_i2c_master_read( &mI2C, I2C_SLAVE_ADDR,  &rcv_data, RCV_ONE_BYTE , 0, true);

	tmp_b2 = rcv_data;

	if (CY_RSLT_SUCCESS != result)
    {
		DPS368_handle_error();
    }

    /* Read LSB */
    send_data = TMP_B1;
	result = cyhal_i2c_master_write( &mI2C, I2C_SLAVE_ADDR, &send_data, SEND_PACKET_SIZE, 0, false);
	result = cyhal_i2c_master_read( &mI2C, I2C_SLAVE_ADDR,  &rcv_data, RCV_PACKET_SIZE , 0, true);
	tmp_b1 = rcv_data;
    if (CY_RSLT_SUCCESS != result)
    {
       DPS368_handle_error();
    }

    /* Read XLSB */
    send_data = TMP_B0;
	result = cyhal_i2c_master_write( &mI2C, I2C_SLAVE_ADDR, &send_data, SEND_PACKET_SIZE, 0, false);
	result = cyhal_i2c_master_read( &mI2C, I2C_SLAVE_ADDR,  &rcv_data, RCV_PACKET_SIZE , 0, true);
	tmp_b0 = rcv_data;
	if (CY_RSLT_SUCCESS != result)
    {
       DPS368_handle_error();
    }

    return ((uint32_t)(tmp_b2 <<16 | tmp_b1 << 8 | tmp_b0));
}

/*******************************************************************************
* Function Name: Calculate Pressure
********************************************************************************
* Summary:
*  Calculate Pressure based on Raw 24 bit Temperature and stored Coefficients.
*  Temperature needs to be calculated first and Traw_sc
*
* Parameters:
*  Raw Pressure - 24 bits
*  Global Variable Traw_sc- Raw Temperature divided by scaling factor
*
* Return:
*  calculated pressure
*
*******************************************************************************/
float Calculate_Pressure(int32_t raw_pressure)
{
 	float prs = raw_pressure;
 	float Praw_sc;

 	//scale pressure according to scaling table and oversampling
    Praw_sc = prs/scaling_factor[PRS_RATE];

    /*Calculate compensated pressure - from data sheet Section 4.9.1 page 15
      Pcomp(Pa) = c00 + Praw_sc*(c10 + Praw_sc *(c20+ Praw_sc *c30)) + Traw_sc *c01 + Traw_sc *Praw_sc *(c11+Praw_sc*c21)
      Traw_sc = Traw/kT
      Praw_sc = Praw/kP
    */
    prs = DPS368_Coef.c00 + Praw_sc*(DPS368_Coef.c10 + Praw_sc *(DPS368_Coef.c20+ Praw_sc *DPS368_Coef.c30)) + Traw_sc *DPS368_Coef.c01 +
 		  Traw_sc *Praw_sc *(DPS368_Coef.c11+Praw_sc*DPS368_Coef.c21);

//    printf("Praw sc = %4.2f Raw Pressure =  %4.2f, scaling factor = %d \r\n", Praw_sc, raw_pressure, scaling_factor[PRS_RATE]);

 	return(prs);
}

/*******************************************************************************
* Function Name: Calculate Temperature
********************************************************************************
 Summary:
*  Calculate temperature in degrees C based on Raw 24 bit Temperature
*  and stored Coefficients.
*
* Parameters:
*  Raw Temperature - 24 bits
*
* Return:
*  calculated temperature
*
*******************************************************************************/
float Calculate_Temperature(int32_t raw_temperature)
{
 	/* scale temperature according to scaling table and oversampling -
 	 * from the data sheet Section 4.9.2 page 15
 	 * Traw_sc = Traw/kT
 	 * Tcomp (°C) = c0*0.5 + c1*Traw_sc
 	 */
   	Traw_sc = (float)raw_temperature/scaling_factor[TEMP_RATE];

 	//Calculate compensated temperature and return it
  	return((float)DPS368_Coef.c0/2  + (float)DPS368_Coef.c1 * Traw_sc);
}
/*******************************************************************************
* Function Name: Get_DPS368_Coef_Source
********************************************************************************
* Summary:
*  Get the Coefficient Source from the Coefficient Source register (0x28)
*
* Parameters:
*  None
*
* Return:
*  Temperature Coefficient Bit 7 Source 0 = Internal. 1 = from MEMS Pressure sensor
*
*******************************************************************************/
uint8_t Get_DPS368_Coef_Source()
{
	uint8_t 	send_data;
	uint8_t 	rcv_data;

	send_data = COEF_SRC;

  	result = cyhal_i2c_master_write( &mI2C, I2C_SLAVE_ADDR, &send_data, SEND_ONE_BYTE, 0, SEND_RESTART);
   	result = cyhal_i2c_master_read( &mI2C, I2C_SLAVE_ADDR,  &rcv_data, RCV_ONE_BYTE , 0, SEND_STOP);

    if (CY_RSLT_SUCCESS != result)
    {
       DPS368_handle_error();
    }

	return (rcv_data);
}

/*******************************************************************************
* Function Name: Get_DPS368_Coefficients
********************************************************************************
* Summary:
*  Get the DPS368 Sensor Coefficients and convert from bit compressed  to actual
*  (0x10 to 0x21) - 18 eight bit values total
*  Create C0, C1, C00, C10, C01, C11, C20, C21 and C30 from these values
*
* Parameters:
*  void
*
* Return:
* void
*
*******************************************************************************/
void Get_DPS368_Coefficients()
{
	uint8_t 	send_data;
	uint8_t 	rcv_data;
	uint8_t		i;

	send_data = C0;

    for(i=0; i < COEF_MAX; i++)
    {
      	result = cyhal_i2c_master_write( &mI2C, I2C_SLAVE_ADDR, &send_data, SEND_ONE_BYTE, 0, SEND_RESTART);
       	result = cyhal_i2c_master_read( &mI2C, I2C_SLAVE_ADDR,  &rcv_data, RCV_ONE_BYTE , 0, SEND_STOP);

        if (CY_RSLT_SUCCESS != result)
        {
           DPS368_handle_error();
        }
    	coef_buffer[i] = (uint8_t) rcv_data;

    	send_data++;
    }

    /* Convert raw registers into actual coefficients */
     DPS368_Coef.c0  = ((uint32_t)coef_buffer[0] << 4) | (((uint32_t)coef_buffer[1] >> 4) & 0x0F);
     getTwosComplement(&DPS368_Coef.c0, 12);
     DPS368_Coef.c1  = (((uint32_t)coef_buffer[1] & 0x0F) << 8) | (uint32_t)coef_buffer[2];
     getTwosComplement(&DPS368_Coef.c1, 12);
     DPS368_Coef.c00 =  ((uint32_t)coef_buffer[3] <<12)| ((uint32_t)coef_buffer[4] << 4) | (((uint32_t)coef_buffer[5] >> 4) & 0x0F);
     getTwosComplement(&DPS368_Coef.c00, 20);
     DPS368_Coef.c10 = (((uint32_t)coef_buffer[5] & 0x0F) << 16) | ((uint32_t)coef_buffer[6] << 8) | (uint32_t)coef_buffer[7];
     getTwosComplement(&DPS368_Coef.c10, 20);
     DPS368_Coef.c01 = ((uint32_t)coef_buffer[8] << 8) | (uint32_t)coef_buffer[9];
     getTwosComplement(&DPS368_Coef.c01, 16);
     DPS368_Coef.c11 = ((uint32_t)coef_buffer[10] << 8) | (uint32_t)coef_buffer[11];
     getTwosComplement(&DPS368_Coef.c11, 16);
     DPS368_Coef.c20 = ((uint32_t)coef_buffer[12] << 8) | (uint32_t)coef_buffer[13];
     getTwosComplement(&DPS368_Coef.c20, 16);
     DPS368_Coef.c21 = ((uint32_t)coef_buffer[14] << 8) | (uint32_t)coef_buffer[15];
     getTwosComplement(&DPS368_Coef.c21, 16);
     DPS368_Coef.c30 = ((uint32_t)coef_buffer[16] << 8) | (uint32_t)coef_buffer[17];
     getTwosComplement(&DPS368_Coef.c30, 16);

#if PRINT_DEBUG

    for(i=0; i < COEF_MAX; i++)
    {

      	printf(" Raw Coef %x = %x", (unsigned int) i, (unsigned int) coef_buffer[i]);
      	printf("\r\n");
    }
    printf("Coef C0 = %x %d \r\n", (int32_t)DPS368_Coef.c0, (int32_t)DPS368_Coef.c0);
    printf("Coef C1 = %x %d \r\n", (int32_t)DPS368_Coef.c1, (int32_t)DPS368_Coef.c1);
    printf("Coef C00 = %x %d \r\n", (int32_t)DPS368_Coef.c00, (int32_t)DPS368_Coef.c00);
    printf("Coef C10 = %x %d \r\n", (int32_t)DPS368_Coef.c10, (int32_t)DPS368_Coef.c10);
    printf("Coef C01 = %x %d \r\n", (int32_t)DPS368_Coef.c01, (int32_t)DPS368_Coef.c01);
    printf("Coef C11 = %x %d \r\n", (int32_t)DPS368_Coef.c11, (int32_t)DPS368_Coef.c11);
    printf("Coef C20 = %x %d \r\n", (int32_t)DPS368_Coef.c20, (int32_t)DPS368_Coef.c20);
    printf("Coef C21 = %x %d \r\n", (int32_t)DPS368_Coef.c21,(int32_t)DPS368_Coef.c21);
    printf("Coef C30 = %x %d \r\n", (int32_t)DPS368_Coef.c30, (int32_t)DPS368_Coef.c30);

#endif

}

void getTwosComplement(int32_t *raw, uint8_t length)
{
	if (*raw & ((uint32_t)1 << (length - 1)))
	{
		*raw -= (uint32_t)1 << length;
	}
}

/*******************************************************************************
* Function Name: Get_DPS368_Sensor_Status
********************************************************************************
* Summary:
*  Get the DPS368 Sensor Status (0x08)
*
* Parameters:
*  void
*
* Return:
*  status
*  Bit 7 = COEF_RDY		- Coefficients will be read to the Coefficents Registers after startup
*  Bit 6 = SENSOR_RDY 	- Sensor initialization complete
*  Bit 5 = TMP_RDY		- New temperature measurement is ready.
*  Bit 4 = PRS_RDY		- New pressure measurement is ready
*
*******************************************************************************/
uint8_t Get_DPS368_Sensor_Status()
{
	uint8_t 	send_data;
	uint8_t 	rcv_data;

	send_data = MEAS_CFG;

 	result = cyhal_i2c_master_write( &mI2C, I2C_SLAVE_ADDR, &send_data, SEND_ONE_BYTE, 0, SEND_RESTART);
   	result = cyhal_i2c_master_read( &mI2C, I2C_SLAVE_ADDR,  &rcv_data, RCV_ONE_BYTE , 0, SEND_STOP);


    if (CY_RSLT_SUCCESS != result)
    {
       DPS368_handle_error();
    }


	return (rcv_data);
}

/*******************************************************************************
* Function Name: Set_DPS368_Pressure_Config
********************************************************************************
* Summary:
* User
*
* Parameters:
*  void
*
* Return:
*  product_ID
*
*******************************************************************************/
void Set_DPS368_Pressure_Config(uint8_t config)
{
		uint8_t 	send_buffer[WRITE_CMD_PACKET_SIZE];

		send_buffer[0] = PRS_CFG;
		send_buffer[1] = config;

		result = cyhal_i2c_master_write( &mI2C, I2C_SLAVE_ADDR, send_buffer, WRITE_CMD_PACKET_SIZE, 0, SEND_STOP);

    if (CY_RSLT_SUCCESS != result)
    {
       DPS368_handle_error();
    }
}

/*******************************************************************************
* Function Name: Set_DPS368_Temperature_Config
********************************************************************************
* Summary:
* User
*
* Parameters:
*  void
*
* Return:
*  product_ID
*
*******************************************************************************/
void Set_DPS368_Temperature_Config(uint8_t config)
{
	uint8_t 	send_buffer[WRITE_CMD_PACKET_SIZE];

	send_buffer[0] = TMP_CFG;
	send_buffer[1] = config;
	result = cyhal_i2c_master_write( &mI2C, I2C_SLAVE_ADDR, send_buffer, WRITE_CMD_PACKET_SIZE, 0, SEND_STOP);

    if (CY_RSLT_SUCCESS != result)
    {
       DPS368_handle_error();
    }
}

/*******************************************************************************
* Function Name: Set_DPS368_Temperature_Config
********************************************************************************
* Summary:
* User
*
* Parameters:
*  void
*
* Return:
*  product_ID
*
*******************************************************************************/
void Set_DPS368_Intr_FIFO_Config(uint8_t config)
{
	uint8_t 	send_buffer[WRITE_CMD_PACKET_SIZE];

	send_buffer[0] = CFG_REG;
	send_buffer[1] = config;
	result = cyhal_i2c_master_write( &mI2C, I2C_SLAVE_ADDR, send_buffer, WRITE_CMD_PACKET_SIZE, 0, SEND_STOP);
    if (CY_RSLT_SUCCESS != result)
    {
       DPS368_handle_error();
    }
}

/*******************************************************************************
* Function Name: Set_DPS368_Sensor_Operating_Mode
********************************************************************************
* Summary: Mode Value  into the MEAS_CFG register to set the DPS368
*
* Parameters:
*  uint8_t mode
*
* 	Set measurement mode and type in Bits2:0:
*	Standby Mode
*	000 - Idle / Stop background measurement
*
*	Command Mode
*	001 - Pressure measurement
*	010 - Temperature measurement
*
*	Background Mode
*	101 - Continuous pressure measurement
*	110 - Continuous temperature measurement
*	111 - Continuous pressure and temperature measurement
*
* Return:
*  void
*
*******************************************************************************/
void Set_DPS368_Sensor_Operating_Mode(uint8_t mode)
{
	uint8_t 	send_buffer[WRITE_CMD_PACKET_SIZE];

	send_buffer[0] = MEAS_CFG;
	send_buffer[1] = mode;

 	result = cyhal_i2c_master_write( &mI2C, I2C_SLAVE_ADDR, send_buffer, WRITE_CMD_PACKET_SIZE, 0, SEND_STOP);
    if (CY_RSLT_SUCCESS != result)
    {
       DPS368_handle_error();
    }
 }

