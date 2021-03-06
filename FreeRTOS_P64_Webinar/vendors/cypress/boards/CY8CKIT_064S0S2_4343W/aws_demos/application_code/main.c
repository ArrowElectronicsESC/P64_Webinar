/*
 * Amazon FreeRTOS V1.4.7
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

/* FIX ME: If you are using Ethernet network connections and the FreeRTOS+TCP stack,
 * uncomment the define:
 * #define CY_USE_FREERTOS_TCP 1
 */

/* DPS368 Includes */
#include "dps368.h"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "resource_map.h"

/* Switch Interrupt defines and functions */

#define GPIO_INTERRUPT_PRIORITY (7u)

static void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);

volatile bool gpio_intr_flag = false;

/* DPS368 Variables and Constants */
/***************************************
*            Constants
****************************************/
// #define CMD_TO_CMD_DELAY        (1000UL)
#define STATUS_REG_READ_DELAY_MS   (1L)
#define false 0

/***************************************
*          Global Variables
****************************************/
cyhal_i2c_t mI2C;
cyhal_i2c_cfg_t mI2C_cfg;

float prs_flt;
int	  publish_fall_event;
int	  publish_pressure;
int	  fall_event;

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#ifdef CY_BOOT_USE_EXTERNAL_FLASH
#include "flash_qspi.h"
#include "cy_smif_psoc6.h"
#endif

#ifdef CY_USE_LWIP
#include "lwip/tcpip.h"
#endif

/* Demo includes */
#include "aws_demo.h"

/* AWS library includes. */
#include "iot_system_init.h"
#include "iot_logging_task.h"
#include "iot_wifi.h"
#include "aws_clientcredential.h"
#include "aws_application_version.h"
#include "aws_dev_mode_key_provisioning.h"

/* BSP & Abstraction inclues */
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "sysflash.h"

#include "iot_network_manager_private.h"

#if BLE_ENABLED
    #include "bt_hal_manager_adapter_ble.h"
    #include "bt_hal_manager.h"
    #include "bt_hal_gatt_server.h"

    #include "iot_ble.h"
    #include "iot_ble_config.h"
    #include "iot_ble_wifi_provisioning.h"
    #include "iot_ble_numericComparison.h"

    #include "cyhal_uart.h"

    #define INPUT_MSG_ALLOC_SIZE             (50u)
    #define DELAY_BETWEEN_GETC_IN_TICKS      (1500u)
#endif

#ifdef CY_TFM_PSA_SUPPORTED
#include "tfm_multi_core_api.h"
#include "tfm_ns_interface.h"
#include "tfm_ns_mailbox.h"
#endif

/* Logging Task Defines. */
#define mainLOGGING_MESSAGE_QUEUE_LENGTH    ( 15 )
#define mainLOGGING_TASK_STACK_SIZE         ( configMINIMAL_STACK_SIZE * 8 )

/* The task delay for allowing the lower priority logging task to print out Wi-Fi
 * failure status before blocking indefinitely. */
#define mainLOGGING_WIFI_STATUS_DELAY       pdMS_TO_TICKS( 1000 )

/* Unit test defines. */
#define mainTEST_RUNNER_TASK_STACK_SIZE     ( configMINIMAL_STACK_SIZE * 16 )

/* The name of the devices for xApplicationDNSQueryHook. */
#define mainDEVICE_NICK_NAME				"cypress_demo" /* FIX ME.*/


/* Static arrays for FreeRTOS-Plus-TCP stack initialization for Ethernet network
 * connections are declared below. If you are using an Ethernet connection on your MCU
 * device it is recommended to use the FreeRTOS+TCP stack. The default values are
 * defined in FreeRTOSConfig.h. */

#ifdef CY_USE_FREERTOS_TCP
/* Default MAC address configuration.  The application creates a virtual network
 * connection that uses this MAC address by accessing the raw Ethernet data
 * to and from a real network connection on the host PC.  See the
 * configNETWORK_INTERFACE_TO_USE definition for information on how to configure
 * the real network connection to use. */
const uint8_t ucMACAddress[ 6 ] =
{
    configMAC_ADDR0,
    configMAC_ADDR1,
    configMAC_ADDR2,
    configMAC_ADDR3,
    configMAC_ADDR4,
    configMAC_ADDR5
};

/* The default IP and MAC address used by the application.  The address configuration
 * defined here will be used if ipconfigUSE_DHCP is 0, or if ipconfigUSE_DHCP is
 * 1 but a DHCP server could not be contacted.  See the online documentation for
 * more information. */
static const uint8_t ucIPAddress[ 4 ] =
{
    configIP_ADDR0,
    configIP_ADDR1,
    configIP_ADDR2,
    configIP_ADDR3
};
static const uint8_t ucNetMask[ 4 ] =
{
    configNET_MASK0,
    configNET_MASK1,
    configNET_MASK2,
    configNET_MASK3
};
static const uint8_t ucGatewayAddress[ 4 ] =
{
    configGATEWAY_ADDR0,
    configGATEWAY_ADDR1,
    configGATEWAY_ADDR2,
    configGATEWAY_ADDR3
};
static const uint8_t ucDNSServerAddress[ 4 ] =
{
    configDNS_SERVER_ADDR0,
    configDNS_SERVER_ADDR1,
    configDNS_SERVER_ADDR2,
    configDNS_SERVER_ADDR3
};
#endif /* CY_USE_FREERTOS_TCP */

/**
 * @brief Application task startup hook for applications using Wi-Fi. If you are not
 * using Wi-Fi, then start network dependent applications in the vApplicationIPNetorkEventHook
 * function. If you are not using Wi-Fi, this hook can be disabled by setting
 * configUSE_DAEMON_TASK_STARTUP_HOOK to 0.
 */
void vApplicationDaemonTaskStartupHook( void );

/**
 * @brief Connects to Wi-Fi.
 */
static void prvWifiConnect( void );

/**
 * @brief Initializes the board.
 */
static void prvMiscInitialization( void );

#ifdef CY_TFM_PSA_SUPPORTED
static struct ns_mailbox_queue_t ns_mailbox_queue;

static void tfm_ns_multi_core_boot(void)
{
    int32_t ret;

    printf("Non-secure code running on non-secure core.\r\n");

    if (tfm_ns_wait_for_s_cpu_ready()) {
        printf("Error sync'ing with secure core.\r\n");

        /* Avoid undefined behavior after multi-core sync-up failed */
        for (;;) {
        }
    }

    ret = tfm_ns_mailbox_init(&ns_mailbox_queue);
    if (ret != MAILBOX_SUCCESS) {
        printf("Non-secure mailbox initialization failed.\r\n");

        /* Avoid undefined behavior after NS mailbox initialization failed */
        for (;;) {
        }
    }
}
#endif

/*-----------------------------------------------------------*/

/*******************************************************************************/
void handle_error(void)
{
	printf("Error in main.c ");
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

/*******************************************************************************
* Function Name: gpio_interrupt_handler
********************************************************************************
* Summary:
*   GPIO interrupt handler.
*
* Parameters:
*  void *handler_arg (unused)
*  cyhal_gpio_irq_event_t (unused)
*
*******************************************************************************/
static void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_irq_event_t event)
{
    gpio_intr_flag = 1;
}


/*******************************************************************************
* Function Name: Read Pressure Task
********************************************************************************
* Summary:
*   Init I2C
*   Init DPS368
*   Read Temperature
*   Read Pressure
*   Calucate Pressure
*   Check for Fall Eveent
*   Publish data
*
* Parameters:
*
*
*******************************************************************************/
void Read_Pressure( void * pvParameters )
{
	    cy_rslt_t result;
	    uint32_t pressure;
	    uint32_t temperature;
	    uint8_t  sensor_ready;
	    uint8_t  pressure_ready;
	    uint8_t  temperature_ready;
	    uint8_t	 wait_count;
	    uint8_t	 first_prs;
	    uint8_t	 publish_count;
	    float 	 temp_flt ;
	 	float 	 last_prs_flt;


	    /* The parameter value is expected to be 1 as 1 is passed in the
	    pvParameters value in the call to xTaskCreate() below. */
	    configASSERT( ( ( uint32_t ) pvParameters ) == 1 );

	    /* Initialize the Green LED - It will toggle everytime an MQTT message is sent */
	    cyhal_gpio_init(CYBSP_LED_RGB_GREEN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);
	    cyhal_gpio_init(CYBSP_LED_RGB_RED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);

	    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
	    printf("\x1b[2J\x1b[;H");

	    printf("****************************\r\n");
	    printf("PSoC6 MCU with DPS368 		 \r\n");
	    printf("Barometric Pressure Sensor   \r\n");
	    printf("****************************\r\n\n");

	    last_prs_flt = 0;
	    first_prs = 0;

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

	    fall_event = 0;
	    publish_count = 0;
	    cyhal_gpio_write( CYBSP_LED_RGB_GREEN, CYBSP_LED_STATE_OFF);
	    cyhal_gpio_write( CYBSP_LED_RGB_RED, CYBSP_LED_STATE_OFF);

	    printf("Init DPS368 \r\n");
	    DPS368_Init();

	    /* Check if sensor is ready and initialized */
	    sensor_ready = SENSOR_RDY & Get_DPS368_Sensor_Status();

	    while (!sensor_ready)
	    {
	    	sensor_ready = SENSOR_RDY & Get_DPS368_Sensor_Status();
	    }
	    printf("DPS368 Ready\r\n");

    for( ;; )
    {
   		cyhal_gpio_write( CYBSP_LED_RGB_RED, CYBSP_LED_STATE_OFF);
    	if(gpio_intr_flag)
    	{
    		gpio_intr_flag = 0;
    		printf("User Button 1 Depressed \r\n");
    		cyhal_gpio_write( CYBSP_LED_RGB_GREEN, CYBSP_LED_STATE_OFF);
    		fall_event = 0;
    	}
        /* Start by reading the Temperature - this is required for an accurate Pressure calculation. */
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


        /* Get pressure by starting a pressure measurement and tesing the status bit for ready.
         * Timeout after not ready for 100ms */

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

        /* Once the status is ready, read the raw pressure and calculate the real pressure in Pa */
        pressure = Get_DPS368_Pressure();

       	prs_flt = Calculate_Pressure(pressure);

       	/* This is a delta pressure calc. Print after 2nd pressure read */
       	if (first_prs)
       	{
       		printf("Pressure %4.1f Pa \tDelta P %4.1f Pa \tTemp =  %4.1f F \r\n", prs_flt, prs_flt- last_prs_flt, ((temp_flt *9/5) +32));

       		/* If there is a change of more that +3 Pa, that indicates a fall event */
       		if (prs_flt- last_prs_flt > 3)
       		{
       			/* Set Fall event flag and publish fall event flag, turn on LED and print message to debug port*/
       			fall_event = 1;
       			publish_fall_event = 1;
       			publish_count = 0;
       			cyhal_gpio_write( CYBSP_LED_RGB_GREEN, CYBSP_LED_STATE_ON);
       			printf("Fall Event \r\n");
       		}

       		last_prs_flt = prs_flt;		/* this pressure reading becomes the last reading for testing on the next reading */
       	}

       	/* The 1st pressure read, so set the flag */
       	else
       	{
       		first_prs = 1;
       	}

       	/* Publish the latest pressure read after 10 times through this task or 5 seconds */

       	if (!publish_pressure)
       	{
       		publish_count++;
       		if (publish_count > 10)
       		{
       			publish_pressure = 1;
       			publish_count = 0;
       			cyhal_gpio_write( CYBSP_LED_RGB_RED, CYBSP_LED_STATE_ON);
       		}
       	}
       	/* put the sensor in standby mode */
       	Set_DPS368_Sensor_Operating_Mode(STANDBY);

       	/* release to scheduler for 500ms */
     	vTaskDelay(500);
    }
}

/**
 * @brief Application runtime entry point.
 */
int main( void )
{
    cy_rslt_t result;
    /* Perform any hardware initialization that does not require the RTOS to be * running.  */
    prvMiscInitialization();

    /* Initialize the user button */
     result = cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT,
                     CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);

     /* if the result is an error, go to the error handler and stop */
	 if (result != CY_RSLT_SUCCESS)
	 {
	     handle_error();
	 }

     /* Configure GPIO interrupt for the User Button*/
     cyhal_gpio_register_callback(CYBSP_USER_BTN,
                                  gpio_interrupt_handler, NULL);
     cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL,
                                  GPIO_INTERRUPT_PRIORITY, true);


#ifdef CY_TFM_PSA_SUPPORTED
    tfm_ns_multi_core_boot();
#endif

    /* Create tasks that are not dependent on the Wi-Fi being initialized. */
    xLoggingTaskInitialize( mainLOGGING_TASK_STACK_SIZE,
                            tskIDLE_PRIORITY,
                            mainLOGGING_MESSAGE_QUEUE_LENGTH );

#ifdef CY_TFM_PSA_SUPPORTED
    /* Initialize TFM interface */
    tfm_ns_interface_init();
#endif

#ifdef CY_USE_FREERTOS_TCP
    FreeRTOS_IPInit( ucIPAddress,
                     ucNetMask,
                     ucGatewayAddress,
                     ucDNSServerAddress,
                     ucMACAddress );
#endif /* CY_USE_FREERTOS_TCP */


 	/* Create the task, storing the handle. */
 	xTaskCreate(Read_Pressure,       /* Function that implements the task. */
 	                    "Pressure",          /* Text name for the task. */
 	                    512,      /* Stack size in words, not bytes. */
 	                    ( void * ) 1,    /* Parameter passed into the task. */
 						democonfigDEMO_PRIORITY+1,/* Priority at which the task is created. */
 	                    NULL );      /* Used to pass out the created task's handle. */

    /* Start the scheduler.  Initialization that requires the OS to be running,
     * including the Wi-Fi initialization, is performed in the RTOS daemon task
     * startup hook. */
    vTaskStartScheduler();

    return 0;
}
/*-----------------------------------------------------------*/

static void prvMiscInitialization( void )
{
    cy_rslt_t result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        printf(  "BSP initialization failed \r\n" );
    }
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        printf( "Retarget IO initialization failed \r\n" );
    }

    #if BLE_ENABLED
        NumericComparisonInit();
    #endif

#ifdef CY_BOOT_USE_EXTERNAL_FLASH
    __enable_irq();

#ifdef PDL_CODE
    if (qspi_init_sfdp(1) < 0)
    {
        printf("QSPI Init failed\r\n");
        while (1);
    }
#else   /* PDL_CODE */
    if (psoc6_qspi_init() != 0)
    {
       printf("psoc6_qspi_init() FAILED!!\r\n");
    }
#endif /* PDL_CODE */
#endif /* CY_BOOT_USE_EXTERNAL_FLASH */
}
/*-----------------------------------------------------------*/
void vApplicationDaemonTaskStartupHook( void )
{
    /* FIX ME: Perform any hardware initialization, that require the RTOS to be
     * running, here. */


    /* FIX ME: If your MCU is using Wi-Fi, delete surrounding compiler directives to
     * enable the unit tests and after MQTT, Bufferpool, and Secure Sockets libraries
     * have been imported into the project. If you are not using Wi-Fi, see the
     * vApplicationIPNetworkEventHook function. */
    if( SYSTEM_Init() == pdPASS )
    {
#ifdef CY_USE_LWIP
        /* Initialize lwIP stack. This needs the RTOS to be up since this function will spawn 
         * the tcp_ip thread.
         */
        tcpip_init(NULL, NULL);
#endif
        /* Connect to the Wi-Fi before running the tests. */
        prvWifiConnect();

#if ( pkcs11configVENDOR_DEVICE_CERTIFICATE_SUPPORTED == 0 )
        /* Provision the device with AWS certificate and private key. */
        vDevModeKeyProvisioning();
#endif

        /* Start the demo tasks. */
        DEMO_RUNNER_RunDemos();
    }
}
/*-----------------------------------------------------------*/

void prvWifiConnect( void )
{
    WIFINetworkParams_t xNetworkParams;
    WIFIReturnCode_t xWifiStatus;
    uint8_t ucTempIp[4] = { 0 };

    xWifiStatus = WIFI_On();

    if( xWifiStatus == eWiFiSuccess )
    {

        configPRINTF( ( "Wi-Fi module initialized. Connecting to AP...\r\n" ) );
    }
    else
    {
        configPRINTF( ( "Wi-Fi module failed to initialize.\r\n" ) );

        /* Delay to allow the lower priority logging task to print the above status.
            * The while loop below will block the above printing. */
        vTaskDelay( mainLOGGING_WIFI_STATUS_DELAY );

        while( 1 )
        {
        }
    }

    /* Setup parameters. */
    xNetworkParams.pcSSID = clientcredentialWIFI_SSID;
    xNetworkParams.ucSSIDLength = sizeof( clientcredentialWIFI_SSID ) - 1;
    xNetworkParams.pcPassword = clientcredentialWIFI_PASSWORD;
    xNetworkParams.ucPasswordLength = sizeof( clientcredentialWIFI_PASSWORD ) - 1;
    xNetworkParams.xSecurity = clientcredentialWIFI_SECURITY;
    xNetworkParams.cChannel = 0;

    xWifiStatus = WIFI_ConnectAP( &( xNetworkParams ) );

    if( xWifiStatus == eWiFiSuccess )
    {
        configPRINTF( ( "Wi-Fi Connected to AP. Creating tasks which use network...\r\n" ) );

        xWifiStatus = WIFI_GetIP( ucTempIp );
        if ( eWiFiSuccess == xWifiStatus )
        {
            configPRINTF( ( "IP Address acquired %d.%d.%d.%d\r\n",
                            ucTempIp[ 0 ], ucTempIp[ 1 ], ucTempIp[ 2 ], ucTempIp[ 3 ] ) );
        }
    }
    else
    {
        /* Connection failed, configure SoftAP. */
        configPRINTF( ( "Wi-Fi failed to connect to AP %s.\r\n", xNetworkParams.pcSSID ) );

        xNetworkParams.pcSSID = wificonfigACCESS_POINT_SSID_PREFIX;
        xNetworkParams.pcPassword = wificonfigACCESS_POINT_PASSKEY;
        xNetworkParams.xSecurity = wificonfigACCESS_POINT_SECURITY;
        xNetworkParams.cChannel = wificonfigACCESS_POINT_CHANNEL;

        configPRINTF( ( "Connect to SoftAP %s using password %s. \r\n",
                        xNetworkParams.pcSSID, xNetworkParams.pcPassword ) );

        while( WIFI_ConfigureAP( &xNetworkParams ) != eWiFiSuccess )
        {
            configPRINTF( ( "Connect to SoftAP %s using password %s and configure Wi-Fi. \r\n",
                            xNetworkParams.pcSSID, xNetworkParams.pcPassword ) );
        }

        configPRINTF( ( "Wi-Fi configuration successful. \r\n" ) );
    }
}

/*-----------------------------------------------------------*/

/**
 * @brief User defined Idle task function.
 *
 * @note Do not make any blocking operations in this function.
 */
void vApplicationIdleHook( void )
{
    /* FIX ME. If necessary, update to application idle periodic actions. */

    static TickType_t xLastPrint = 0;
    TickType_t xTimeNow;
    const TickType_t xPrintFrequency = pdMS_TO_TICKS( 5000 );

    xTimeNow = xTaskGetTickCount();

    if( ( xTimeNow - xLastPrint ) > xPrintFrequency )
    {
        configPRINT( "." );
        xLastPrint = xTimeNow;
    }
}

void vApplicationTickHook()
{
}

/*-----------------------------------------------------------*/

/**
* @brief User defined application hook to process names returned by the DNS server.
*/
#if ( ipconfigUSE_LLMNR != 0 ) || ( ipconfigUSE_NBNS != 0 )
    BaseType_t xApplicationDNSQueryHook( const char * pcName )
    {
        /* FIX ME. If necessary, update to applicable DNS name lookup actions. */

        BaseType_t xReturn;

        /* Determine if a name lookup is for this node.  Two names are given
         * to this node: that returned by pcApplicationHostnameHook() and that set
         * by mainDEVICE_NICK_NAME. */
        if( strcmp( pcName, pcApplicationHostnameHook() ) == 0 )
        {
            xReturn = pdPASS;
        }
        else if( strcmp( pcName, mainDEVICE_NICK_NAME ) == 0 )
        {
            xReturn = pdPASS;
        }
        else
        {
            xReturn = pdFAIL;
        }

        return xReturn;
    }

#endif /* if ( ipconfigUSE_LLMNR != 0 ) || ( ipconfigUSE_NBNS != 0 ) */
/*-----------------------------------------------------------*/

/**
 * @brief User defined assertion call. This function is plugged into configASSERT.
 * See FreeRTOSConfig.h to define configASSERT to something different.
 */
void vAssertCalled(const char * pcFile,
	uint32_t ulLine)
{
    /* FIX ME. If necessary, update to applicable assertion routine actions. */

	const uint32_t ulLongSleep = 1000UL;
	volatile uint32_t ulBlockVariable = 0UL;
	volatile char * pcFileName = (volatile char *)pcFile;
	volatile uint32_t ulLineNumber = ulLine;

	(void)pcFileName;
	(void)ulLineNumber;

	printf("vAssertCalled %s, %ld\n", pcFile, (long)ulLine);
	fflush(stdout);

	/* Setting ulBlockVariable to a non-zero value in the debugger will allow
	* this function to be exited. */
	taskDISABLE_INTERRUPTS();
	{
		while (ulBlockVariable == 0UL)
		{
			vTaskDelay( pdMS_TO_TICKS( ulLongSleep ) );
		}
	}
	taskENABLE_INTERRUPTS();
}
/*-----------------------------------------------------------*/

/**
 * @brief User defined application hook need by the FreeRTOS-Plus-TCP library.
 */
#if ( ipconfigUSE_LLMNR != 0 ) || ( ipconfigUSE_NBNS != 0 ) || ( ipconfigDHCP_REGISTER_HOSTNAME == 1 )
    const char * pcApplicationHostnameHook(void)
    {
        /* FIX ME: If necessary, update to applicable registration name. */

        /* This function will be called during the DHCP: the machine will be registered
         * with an IP address plus this name. */
        return clientcredentialIOT_THING_NAME;
    }

#endif

#if BLE_ENABLED
    /**
     * @brief "Function to receive user input from a UART terminal. This function reads until a line feed or 
     * carriage return character is received and returns a null terminated string through a pointer to INPUTMessage_t.
     * 
     * @note The line feed and carriage return characters are removed from the returned string.
     * 
     * @param pxINPUTmessage Message structure using which the user input and the message size are returned.
     * @param xAuthTimeout Time in ticks to be waited for the user input.
     * @returns pdTrue if the user input was successfully captured, else pdFalse.
     */
    BaseType_t getUserMessage( INPUTMessage_t * pxINPUTmessage, TickType_t xAuthTimeout )
    {
        BaseType_t xReturnMessage = pdFALSE;
        TickType_t xTimeOnEntering;
        uint8_t *ptr;
        uint32_t numBytes = 0;
        uint8_t msgLength = 0;

        /* Dynamically allocate memory to store user input. */
        pxINPUTmessage->pcData = ( uint8_t * ) pvPortMalloc( sizeof( uint8_t ) * INPUT_MSG_ALLOC_SIZE ); 

        /* ptr points to the memory location where the next character is to be stored. */
        ptr = pxINPUTmessage->pcData;   

        /* Store the current tick value to implement a timeout. */
        xTimeOnEntering = xTaskGetTickCount();

        do
        {
            /* Check for data in the UART buffer with zero timeout. */
            numBytes = cyhal_uart_readable(&cy_retarget_io_uart_obj);
            if (numBytes > 0)
            {
                /* Get a single character from UART buffer. */
                cyhal_uart_getc(&cy_retarget_io_uart_obj, ptr, 0);

                /* Stop checking for more characters when line feed or carriage return is received. */
                if((*ptr == '\n') || (*ptr == '\r'))
                {
                    *ptr = '\0';
                    xReturnMessage = pdTRUE;
                    break;
                }

                ptr++;
                msgLength++;

                /* Check if the allocated buffer for user input storage is full. */
                if (msgLength >= INPUT_MSG_ALLOC_SIZE) 
                {
                    break;
                }
            }

            /* Yield to other tasks while waiting for user data. */
            vTaskDelay( DELAY_BETWEEN_GETC_IN_TICKS );

        } while ((xTaskGetTickCount() - xTimeOnEntering) < xAuthTimeout); /* Wait for user data until timeout period is elapsed. */

        if (xReturnMessage == pdTRUE)
        {
            pxINPUTmessage->xDataSize = msgLength;
        }
        else if (msgLength >= INPUT_MSG_ALLOC_SIZE)
        {
            configPRINTF( ( "User input exceeds buffer size !!\n" ) );
        }
        else
        {
            configPRINTF( ( "Timeout period elapsed !!\n" ) );
        }       

        return xReturnMessage;
    }

#endif /* if BLE_ENABLED */  
