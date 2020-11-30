/******************************************************************************
* File Name: dps368.h
*
* Description: This file defines the SPI pin map for all the supported kits.
*
*******************************************************************************
* (c) 2020, Arrow Electronics
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
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
* significant property damage, injury or death (“High Risk Product”). By
* including Cypress’s product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#ifndef DPS368_H_
#define DPS368_H_

/* Enable / Disable printing of debug info from DPS368 */
#define PRINT_DEBUG 1


/* I2C define */
#define I2C_SLAVE_ADDR	(0x76UL)
#define SEND_RESTART	false
#define SEND_STOP		true

/* DPS368 Command Read / Write -  Bit 7 = 1 for Read Command and 0 for Write Command */
#define DPS368_READ_CMD		0x80
#define DPS368_WRITE		0x7F

/* Buffer and packet size */

#define SEND_ONE_BYTE	    	(1UL)
#define WRITE_CMD_PACKET_SIZE  	(2UL)
#define RCV_ONE_BYTE	        (1UL)
#define SEND_PACKET_SIZE        (1UL)
#define RCV_PACKET_SIZE	        (1UL)

#define COEF_MAX			18

/* DPS368 Registers  */
#define PSR_B2		0x00	// Pressure [23:16]
#define PSR_B1		0x01	// Pressure [15:08]
#define PSR_B0		0x02	// Pressure [07:00]
#define TMP_B2		0x03	// Temperature [23:16]
#define TMP_B1		0x04	// Temperature [15:08]
#define TMP_B0		0x05	// Temperature [07:00]
#define PRS_CFG		0x06	// Pressure configuration PM_RATE[2{0] bit6 bit5  PM_PRC[3:0] Bit3:Bit0 (resolution)
#define TMP_CFG		0x07	// Temperature configuration TMP_RATE[2{0] bit6 bit5  TMP_PRC[3:0] Bit3:Bit0 (resolution)
#define MEAS_CFG 	0x08	// Sensor Operation Mode and Status - COEF RDY, SENSOR_RDY, TMP_RDY, PRS_RDY and MEAS_CTRL
#define CFG_REG		0x09	// Interrupt and FIFO Configuration
#define	INT_STS		0x0A	// Interrupt Status
#define FIFO_STS	0x0B	// FIFO Status
#define RESET		0x0C	// Soft Reset
#define ID			0x0D	// Product and Revision ID - Value - 0x10
#define C0			0x10	// Start of Coeffiecent Registers is C0
#define C1			0x11	// Start of Coeffiecent Registers is C0
#define COEF_SRC	0x28	// 0 = Internal temperature of ASIC, 1 = External temperature sesnor (of the pressure MEMS element)

/* Register Bit Masks */
#define	PM_RATE			0x70	// Pressure Measurement Rate - # of measurements per second
#define	PM_PRC			0x0F	// Pressure Resolution - Oversampling rate from single to 128 times
#define	TMP_RATE		0x70	// Temperature Measurement Rate - # of measurements per second
#define	TMP_PRC			0x0F	// Temperature Resolution - Oversampling rate from single to 128 times
#define TMP_EXT_MASK	0x80	// 0 = if using internal temperature sensor, 1 = if using external sensor
#define COEF_RDY		0x80	// COEF Ready Bit in MEAS_CFG
#define SENSOR_RDY		0x40	// Sensor Ready Bit in MEAS_CFG - initialization complete
#define	TMP_RDY			0x20	// Temperature Measurement Ready Bit in MEAS_CFG
#define	PRS_RDY			0x10	// Pressure Measurement Ready Bit in MEAS_CFG
#define P_SHIFT			0x04	// Pressure shift if oversampling > 8
#define T_SHIFT			0x08	// Temperature shift if oversampling > 8

/* Sensor Mode configuration Constants */
#define STANDBY							0x00	// Standby Mode
#define START_PRESSURE_MEASUREMENT		0x01	// Start Pressure Measurement
#define START_TEMPERATURE_MEASUREMENT	0x02	// Start Temperature Measurement

/* Measurement Rates for Temperature and Pressure */
#define MEASUREMENT_RATE_1		0	// 1 measurement/second
#define MEASUREMENT_RATE_2		1	// 2 measurements/second
#define MEASUREMENT_RATE_4		2	// 4 measurements/second
#define MEASUREMENT_RATE_8		3	// 8 measurements/second
#define MEASUREMENT_RATE_16		4	// 16 measurements/second
#define MEASUREMENT_RATE_32		5	// 32 measurements/second
#define MEASUREMENT_RATE_64		6	// 64 measurements/second
#define MEASUREMENT_RATE_128		7	// 128 measurements/second


/* Oversampling Rates for Temperature and Pressure */
#define OVERSAMPLE_RATE_1	0	// Single
#define OVERSAMPLE_RATE_2	1	// 2 Times
#define OVERSAMPLE_RATE_4	2	// 4 Times
#define OVERSAMPLE_RATE_8	3	// 8 Times
#define OVERSAMPLE_RATE_16	4	// 16 Times
#define OVERSAMPLE_RATE_32 	5	// 32 Times
#define OVERSAMPLE_RATE_64	6	// 64 Times
#define OVERSAMPLE_RATE_128	7	// 128 Times

#define PRS_PM		MEASUREMENT_RATE_4
#define PRS_RATE	OVERSAMPLE_RATE_64
#define TEMP_PM		MEASUREMENT_RATE_4
#define TEMP_RATE	OVERSAMPLE_RATE_1


/* Declare raw coefficients buffer and actual coefficient structure used for temperature and pressure calculations  */
uint8_t coef_buffer[18];

struct coef	{
		int32_t c0;				// c0
		int32_t c1;				// c1
		int32_t c00;			// c00
		int32_t c10;			// c10
		int32_t c01;			// c01
		int32_t c11;			// c11
		int32_t c20;			// c20
		int32_t c21;			// c21
		int32_t c30;			// c30
};



/* Function Prototypes */

void 	 DPS368_Init();
uint8_t  Get_DPS368_ID();
uint32_t Get_DPS368_Pressure();
uint32_t Get_DPS368_Temperature();
float 	 Calculate_Pressure(int32_t raw_pressure);
float 	 Calculate_Temperature(int32_t raw_temperature);
uint8_t  Get_DPS368_Coef_Source();
uint8_t  Get_DPS368_Sensor_Status();
void 	 Get_DPS368_Coefficients();
void 	 getTwosComplement(int32_t *raw, uint8_t length);
void 	 Set_DPS368_Pressure_Config(uint8_t config);
void 	 Set_DPS368_Temperature_Config(uint8_t config);
void 	 Set_DPS368_Intr_FIFO_Config(uint8_t config);
void 	 Set_DPS368_Sensor_Operating_Mode(uint8_t mode);

#endif /* DPS368_H_ */
