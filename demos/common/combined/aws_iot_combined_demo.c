/*
Amazon FreeRTOS Combined Demo V1.0.0
Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 http://aws.amazon.com/freertos
 http://www.FreeRTOS.org
*/


/**
 * @file aws_iot_combined_demo.c
 * @brief A demo that shows OTA, Pub/Sub, 
 *
 * This example initializes the OTA agent to enable OTA updates via the
 * MQTT broker. It simply connects to the MQTT broker with the users
 * credentials and spins in an indefinite loop to allow MQTT messages to be
 * forwarded to the OTA agent for possible processing. The OTA agent does all
 * of the real work; checking to see if the message topic is one destined for
 * the OTA agent. If not, it is simply ignored.
 */
/* MQTT include. */

#include <stdbool.h>
#include <string.h>

/* Build using a config header, if provided. */
#include "aws_iot_demo.h"

/*
 * FreeRTOS header includes
 */
#include "FreeRTOS.h"
#include "semphr.h"

/* MQTT library header includes */
#include "aws_iot_mqtt.h"

/* POSIX and Platform layer includes. */
#include "platform/aws_iot_clock.h"
#include "FreeRTOS_POSIX/time.h"

/* Network connection includes */
#include "aws_iot_network.h"
#include "aws_iot_demo_network.h"

#include "aws_clientcredential.h"
#include "aws_demo_config.h"

#include "aws_iot_network_manager.h"

/* Amazon FreeRTOS OTA agent includes. */
#include "aws_ota_agent.h"
#include "driver/gpio.h"
#include "aws_application_version.h"


/**
 * @brief The topic that the MQTT client both subscribes and publishes to.
 */
#define echoTOPIC_NAME      "freertos/demos/echo"

#define echoDATA            "HelloWorld %d"

/**
 * @brief Dimension of the character array buffers used to hold data (strings in
 * this case) that is published to and received from the MQTT broker (in the cloud).
 */
#define echoMAX_DATA_LENGTH            ( sizeof( echoDATA ) + 4 )

/**
 * @brief The string appended to messages that are echoed back to the MQTT broker.
 *
 * It is also used to detect if a received message has already been acknowledged.
 */
#define echoACK_STRING                     " ACK"

#define echoACK_LENGTH                     ( 4 )

#define echoMAX_ACK_MSG_LENGTH             ( echoMAX_DATA_LENGTH + echoACK_LENGTH )

#define echoQOS                            ( 0 )

#define echoKEEPALIVE_SECONDS              ( 120 )

#define echoNUM_MESSAGES                   ( 120 )

#define echoMSG_INTERVAL_SECONDS           ( 1 )

#define echoMQTT_TIMEOUT_MS                ( 5000 )

#define echoCONN_RETRY_INTERVAL_SECONDS    ( 5 )

#define echoCONN_RETRY_LIMIT               ( 100 )

#define echoTASK_STACK_SIZE                ( configMINIMAL_STACK_SIZE * 7 +  echoMAX_DATA_LENGTH )

#define producerQUEUE_LENGTH               (10)

#define ONE_SECOND_DELAY_IN_TICKS       pdMS_TO_TICKS( 1000UL )
#define MS_200_DELAY_IN_TICKS           pdMS_TO_TICKS( 200UL )

#define PUB_TIMEOUT_MS                  ( 5000 )

#define _PUBLISH_RETRY_MS                         ( 1000 )
/**
 * @brief Format string of the PUBLISH messages in this demo.
 */
#define _PUBLISH_PAYLOAD_FORMAT                   "Hello world %d!"
/**
 * @brief Size of the buffer that holds the PUBLISH messages in this demo.
 */
#define _PUBLISH_PAYLOAD_BUFFER_LENGTH            ( sizeof( _PUBLISH_PAYLOAD_FORMAT ) + 2 )

/**
 * @brief The maximum number of times each PUBLISH in this demo will be retried.
 */
#define _PUBLISH_RETRY_LIMIT                      ( 10 )

/**
 * @brief IO pin for push button.
 */
#define GPIO_INPUT_IO_0                 ( 4 )
/**
 * @brief Value that is the start of a count down counter for maximum MQTT pub 
 * timeout or send error.
 */
#define  DISCONNECT_COUNT_DOWN          ( 10 )

/*-----------------------------------------------------------*/
/**
 * @brief Callback invoked when a message is recieved from the cloud on the demo topic.
 *
 * If not already echoed, the function echoes back the same message by appending an ACK
 * to the original message. If it's an already echoed message, function discards the message.
 *
 * @param pvUserParam[in] User param for the callback
 * @param pxPublishParam[in] Publish param which contains the topic, the payload and other details.
 */
static void prvEchoMessage( void* pvUserParam, AwsIotMqttCallbackParam_t* pxPublishParam );

/**
 * @brief Opens a new MQTT connection with the IOT broker endpoint.
 *
 * Function first creates a network connection which can either be a Bluetooth Low Energy connection with
 * a compatible device or a secure socket connection over WIFI. It then performs MQTT connect with the broker
 * endpoint, and subscribes to the demo topic configured.
 *
 * @return true if all the steps succeeded.
 */
bool prbOpenMqttConnection( void );
/**
 * @brief Closes an MQTT connection with the broker endpoint.
 *
 * Function first closes the MQTT connection with the broker by optionally (set in the parameter) sending a
 * DISCONNECT message to the broker. It also tears down the physical connectivity betweend the device and the
 * broker.
 *
 * @param bSendDisconnect[in] Should send a DISCONNECT message to the broker.
 */
void prvCloseMqttConnection( bool bSendDisconnect );
/**
 * @brief Subscribes or unsubscribes to the demo topic.
 * @param bSubscribe[in] Set to true if its a subscribe or false for unsubscribe.
 *
 * @return true if operation was successful.
 */
AwsIotMqttError_t prxSubscribeorUnsubscribeToTopic( bool bSubscribe );
/**
 * @brief Publishes a message using QoS0 or QoS1 to the specified topic.
 *
 * @param pcMesg Pointer to the message
 * @param xLength Length of the message
 * @return AWS_IOT_MQTT_SUCCESS if the publish was successful.
 */
static AwsIotMqttError_t prxPublishMessage(uint32_t receiveValue );

/**
 * @brief Task used to publish all the messages to the topic.
 * @param pvParameters [in] Task parameter
 */
static void vMqttPubTask( void * pvParameters  );

/**
 * @brief Task used to produce messages into a queue.
 * @param pvParameters [in] Task parameter
 */
static void vProducerTask(void * pvParameters ); 

/**
 * @brief Callback invoked when a DISCONNECT event was received for the connected network.
 *
 * @param xConnection The handle for the connection to the network.
 */
static void prvOnNetworkDisconnect( AwsIotDemoNetworkConnection_t xConnection );

/* Declaration of snprintf. The header stdio.h is not included because it
 * includes conflicting symbols on some platforms. */
extern int snprintf( char * pcS,
                     size_t xN,
                     const char * pcFormat,
                     ... );
/**
 * @brief Underlying network interface used for the MQTT connection.
 */
static AwsIotMqttNetIf_t xNetworkInterface = AWS_IOT_MQTT_NETIF_INITIALIZER;
/**
 * @brief Underlying network Connection used for the MQTT connection.
 */
static AwsIotDemoNetworkConnection_t xNetworkConnection = NULL;

/**
 * @brief Handle to the MQTT connection.
 */
static AwsIotMqttConnection_t xMqttConnection = AWS_IOT_MQTT_CONNECTION_INITIALIZER;

/**
 * @brief Global variable used to indiacte network is connected. Set by
 * primary application protocol (e.g. MQTT)making connection or breaking the connection.
 * Also set in primary app protocol callback if network goes away (e.g. wifi disconnects).
 */
static bool xNetworkConnected = false;
/**
 * @brief Global variable used to indiacte if the networking socket is connected.
 * Set by primary applicatino protocol (e.g. MQTT). Used by secondary application
 * protocols (e.g. OTA) to initiate or shutdown application protocol.
 */
static bool xSocketConnected = false;
/**
 * @brief Global variable used to indiacte that an appliction protcol is
 * using the socket. If secondary application is actively using the socket
 * this flag is true. 
 */
static SemaphoreHandle_t xOtaProtocolUsingSocket;

/**
 * @brief Queue of message sent to MQTT Pub task. Meant to simulate UART input.
 */
static QueueHandle_t xTelemetryQueue = NULL;

const char * publishTopic = "telemetry/example";
#define jsonFORMAT "{ \"key\": %d }"
static char pPublishPayload[_PUBLISH_PAYLOAD_BUFFER_LENGTH] = {0};

static const char *pcStateStr[eOTA_NumAgentStates] =
{
     "Not Ready",
     "Ready",
     "Active",
     "Shutting down"
};

/* The OTA agent has completed the update job or determined that we're in
 * self test mode. If it was accepted, we want to activate the new image.
 * This typically means we should reset the device to run the new firmware.
 * If now is not a good time to reset the device, it may be activated later
 * by your user code. If the update was rejected, just return without doing
 * anything and we'll wait for another job. If it reported that we should
 * start test mode, normally we would perform some kind of system checks to
 * make sure our new firmware does the basic things we think it should do
 * but we'll just go ahead and set the image as accepted for demo purposes.
 * The accept function varies depending on your platform. Refer to the OTA
 * PAL implementation for your platform in aws_ota_pal.c to see what it
 * does for you.
 */
static void App_OTACompleteCallback( OTA_JobEvent_t eEvent )
{
    OTA_Err_t xErr = kOTA_Err_Uninitialized;
	
    if ( eEvent == eOTA_JobEvent_Activate )
    {
        AwsIotLogInfo ( "App_OTACompleteCallback: Received eOTA_JobEvent_Activate callback from OTA Agent.\r\n" ) ;
        OTA_ActivateNewImage();
    }
    else if (eEvent == eOTA_JobEvent_Fail)
    {
        AwsIotLogInfo(  "App_OTACompleteCallback: Received eOTA_JobEvent_Fail callback from OTA Agent.\r\n" ) ;
        /* Nothing special to do. The OTA agent handles it. */
    }
    else if (eEvent == eOTA_JobEvent_StartTest)
    {
        /* This demo just accepts the image since it was a good OTA update and networking
         * and services are all working (or we wouldn't have made it this far). If this
         * were some custom device that wants to test other things before calling it OK,
         * this would be the place to kick off those tests before calling OTA_SetImageState()
         * with the final result of either accepted or rejected. */
        AwsIotLogInfo(  "App_OTACompleteCallback: Received eOTA_JobEvent_StartTest callback from OTA Agent.\r\n"  );
	xErr = OTA_SetImageState (eOTA_ImageState_Accepted);
        if( xErr != kOTA_Err_None )
        {
            OTA_LOG_L1( " App_OTACompleteCallback: Error! Failed to set image state as accepted.\r\n" );    
        }
    }
}

void vOtaTask( void * pvParameters )
{
    OTA_State_t otaState = eOTA_AgentState_Unknown;
    OTA_State_t otaLastState = eOTA_AgentState_Unknown;
    bool otaInited = false;

    /* Remove compiler warnings about unused parameters. */
    ( void ) pvParameters;

    AwsIotLogInfo( "vOtaTask: start task\n" );

    while(1)
    {
        otaState = OTA_GetAgentState();

        if (xSocketConnected)
        {
            if (!otaInited)
            {
                AwsIotLogInfo( "vOtaTask: Socket just connected, initing OTA\n" );
                xSemaphoreTake(xOtaProtocolUsingSocket, portMAX_DELAY);
                AwsIotLogInfo( "vOtaTask: initing OTA, after take semaphore\n" );
                if (OTA_AgentInit(xMqttConnection, ( const uint8_t * ) ( clientcredentialIOT_THING_NAME ), App_OTACompleteCallback, ( TickType_t ) ~0 ) == eOTA_AgentState_Ready)
                {
                    AwsIotLogInfo( "vOtaTask: initing OTA, after init\n" );
                    otaInited = true;
                }
                else
                {
                    xSemaphoreGive(xOtaProtocolUsingSocket);
                }
            }
            else 
            {
                if (otaState ==  eOTA_AgentState_Active)
                {
                    AwsIotLogInfo(  "vOtaTask: State: %s  Received: %u   Queued: %u   Processed: %u   Dropped: %u\r\n",
                        pcStateStr[otaState], OTA_GetPacketsReceived(), OTA_GetPacketsQueued(), 
                        OTA_GetPacketsProcessed(), OTA_GetPacketsDropped() ) ;
                }
                else if (otaState ==  eOTA_AgentState_NotReady || otaState ==  eOTA_AgentState_Ready || 
                         otaState ==  eOTA_AgentState_ShuttingDown)
                {
                    AwsIotLogInfo("vOtaTask: State: %s \n", pcStateStr[otaState]);
                }
                else if (otaState ==  eOTA_AgentState_NotReady || otaState ==  eOTA_AgentState_Ready || 
                         otaState ==  eOTA_AgentState_ShuttingDown)
                {
                    AwsIotLogInfo("vOtaTask: State: %s \n", pcStateStr[otaState]);
                }
                else if (otaState ==  eOTA_AgentState_Unknown)
                {
                    /* Note: value of eOTA_AgentState_Unknown is -1. Don't use as index intopcStateStr[] */
                    AwsIotLogInfo("vOtaTask: State: Unknown\n");
                }

                if ((otaLastState == eOTA_AgentState_Ready || 
                     otaLastState == eOTA_AgentState_Active)
                        &&
                    (otaState == eOTA_AgentState_ShuttingDown || 
                     otaState == eOTA_AgentState_NotReady  || 
                     otaState == eOTA_AgentState_Unknown))
                {
                    AwsIotLogInfo("vOtaTask: Socket is connected. Unexpected state change\n");
                }
            }
        }
        else 
        {
            if (otaInited)
            {
                /* take Semaphore */
                AwsIotLogInfo( "vOtaTask: Socket disconnected, shutdown OTA\n" );
                OTA_AgentShutdown(  (TickType_t ) ~0 );
                AwsIotLogInfo( "vOtaTask: OTA is shutdown\n" );
                otaInited = false;
                xSemaphoreGive(xOtaProtocolUsingSocket);
                AwsIotLogInfo( "vOtaTask: Gave up semaphore\n" );
            }
            else
            {
                AwsIotLogInfo( "vOtaTask: waiting for socket to connect\n" );
                vTaskDelay( 5 * ONE_SECOND_DELAY_IN_TICKS  );
            }

        } 
        vTaskDelay( ONE_SECOND_DELAY_IN_TICKS  );
        otaLastState = otaState;
    }
}


bool buttonWasPushed(void)
{
    int numSecsToWait = 5;
    bool pushed = false;
    bool released  = false;

    while (numSecsToWait--)
    {
        if (gpio_get_level(GPIO_NUM_0) == 0)
        {
            /*   AwsIotLogInfo("buttonWasPushed: button pushed\n"); */
            pushed = true;
            break;
        }
        vTaskDelay( ONE_SECOND_DELAY_IN_TICKS );
    }
    if (pushed)
    {
        numSecsToWait = 20;
        while (numSecsToWait--)
        {
            if (gpio_get_level(GPIO_NUM_0) == 1)
            {
                /* AwsIotLogInfo("buttonWasPushed: button released\n"); */
                released = true;
                break;
            }
            vTaskDelay( ONE_SECOND_DELAY_IN_TICKS );
        }
    }
    return released;
}

void vBLEButtonTask(void * pvParameters )
{
    // Enable GPIO 0 so that we can read the state
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //bit mask of the pins, use GPIO0 here
    io_conf.pin_bit_mask = (1<<GPIO_INPUT_IO_0);
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    while( true )
    {
        if (buttonWasPushed())
        {
            if (AwsIotNetworkManager_GetEnabledNetworks() & AWSIOT_NETWORK_TYPE_BLE) 
            {
                AwsIotLogInfo( "vBLEButtonTask: BLE Button OFF\n");
                AwsIotNetworkManager_DisableNetwork(AWSIOT_NETWORK_TYPE_BLE);
            }
            else 
            {
                AwsIotLogInfo( "vBLEButtonTask: BLE Button ON\n");
                AwsIotNetworkManager_EnableNetwork(AWSIOT_NETWORK_TYPE_BLE);
            }
        }
    }
}

void vProducerTask(void * pvParameters )
{
    uint16_t counter = 0;
    /* Create the queue used to pass data to publish task. */
    while( true )
    {
        if( xQueueSend( xTelemetryQueue, &counter, pdMS_TO_TICKS(1000) ) != pdPASS )
        {
            AwsIotLogError(( "vProducerTask: Failed to send data to TelemetryQueue\n" ) );
        }
        else 
        {
            ++counter;
        }   
        /*AwsIotLogInfo( "vProducerTask: produced %d\n", (int) counter);*/
        vTaskDelay( ONE_SECOND_DELAY_IN_TICKS );
    }
}

AwsIotMqttError_t prxPublishMessage(uint32_t receiveValue)
{
    /* AwsIotMqttReference_t xOperationLock = AWS_IOT_MQTT_REFERENCE_INITIALIZER; */
    AwsIotMqttPublishInfo_t publishInfo = AWS_IOT_MQTT_PUBLISH_INFO_INITIALIZER;
    AwsIotMqttError_t xStatus;

    /* Set the common members of the publish info. */
    publishInfo.QoS = 1;
    publishInfo.pPayload = &pPublishPayload[0];
    publishInfo.retryMs = _PUBLISH_RETRY_MS;
    publishInfo.retryLimit = _PUBLISH_RETRY_LIMIT;

    /* Choose a topic name. */
    publishInfo.pTopicName = publishTopic;
    publishInfo.topicNameLength = strlen(publishTopic);

    /* Generate the payload for the PUBLISH. */
    if (snprintf(pPublishPayload,
                        _PUBLISH_PAYLOAD_BUFFER_LENGTH,
                        jsonFORMAT,
                        (int) (receiveValue)) < 0)

    {
        AwsIotLogError("Failed to generate MQTT PUBLISH payload");
        xStatus = AWS_IOT_MQTT_BAD_PARAMETER;
    }
    else
    {
        /* PUBLISH a message with timeout */
        xStatus = AwsIotMqtt_TimedPublish(xMqttConnection,
            &publishInfo,
            0,
            PUB_TIMEOUT_MS);
        AwsIotLogInfo("prxPublishMessag: pub string = %s", publishInfo.pPayload);
    }
    return xStatus;
}

void prvEchoMessage( void* pvUserParam, AwsIotMqttCallbackParam_t* pxPublishParam )
{
    size_t xAckPos, xPayloadLen = pxPublishParam->message.info.payloadLength;
    const char *pcPayload = ( const char *) pxPublishParam->message.info.pPayload;
    char cAck[ echoMAX_ACK_MSG_LENGTH ] = { 0 };
    uint32_t fake_value = 42;
    AwsIotMqttError_t xStatus;

    /* User params not used */
    ( void ) pvUserParam;

    xAckPos = xPayloadLen - echoACK_LENGTH;
    if( strncmp( ( pcPayload + xAckPos ), echoACK_STRING, echoACK_LENGTH ) != 0 )
    {
        AwsIotLogInfo( "Received: %.*s\n", xPayloadLen, pcPayload);

        if( xPayloadLen < echoMAX_DATA_LENGTH )
        {
            memcpy(cAck, pcPayload,  xPayloadLen );
            strcat( cAck, echoACK_STRING );
            xStatus = prxPublishMessage(fake_value);
            if( xStatus != AWS_IOT_MQTT_SUCCESS )
            {
                AwsIotLogInfo(" Failed to send: %s, error = %s\n", cAck, AwsIotMqtt_strerror( xStatus ));
            }
            else
            {
                AwsIotLogInfo( "Echo Sent: %s\n", cAck );
            }
        }
    }
}

AwsIotMqttError_t prxSubscribeorUnsubscribeToTopic( bool bSubscribe )
{
    AwsIotMqttReference_t xOperationLock = AWS_IOT_MQTT_REFERENCE_INITIALIZER;
    AwsIotMqttSubscription_t xSubscription = AWS_IOT_MQTT_SUBSCRIPTION_INITIALIZER;
    AwsIotMqttError_t xMqttStatus;

    xSubscription.QoS = echoQOS;
    xSubscription.callback.function = prvEchoMessage;
    xSubscription.callback.param1 = NULL;
    xSubscription.pTopicFilter = echoTOPIC_NAME;
    xSubscription.topicFilterLength = strlen( echoTOPIC_NAME );

    if( bSubscribe )
    {
        xMqttStatus = AwsIotMqtt_Subscribe(
                xMqttConnection,
                &xSubscription,
                1,
                AWS_IOT_MQTT_FLAG_WAITABLE,
                NULL,
                &xOperationLock );
    }
    else
    {
        xMqttStatus = AwsIotMqtt_Unsubscribe(
                xMqttConnection,
                &xSubscription,
                1,
                AWS_IOT_MQTT_FLAG_WAITABLE,
                NULL,
                &xOperationLock );
    }

    if( xMqttStatus == AWS_IOT_MQTT_STATUS_PENDING )
    {
        xMqttStatus = AwsIotMqtt_Wait( xOperationLock, echoMQTT_TIMEOUT_MS );
    }

    return xMqttStatus;
}

bool prbOpenMqttConnection( void )
{
    AwsIotMqttConnectInfo_t xConnectInfo = AWS_IOT_MQTT_CONNECT_INFO_INITIALIZER;
    bool xStatus = false;

    AwsIotDemo_CreateNetworkConnection(
            &xNetworkInterface,
            &xMqttConnection,
            prvOnNetworkDisconnect,
            &xNetworkConnection,
            echoCONN_RETRY_INTERVAL_SECONDS,
            echoCONN_RETRY_LIMIT );

    if( xNetworkConnection != NULL )
    {
       /*
        * If the network type is BLE, MQTT library connects to the IoT broker using
        * a proxy device as intermediary. So set .awsIotMqttMode to false. Disable keep alive
        * by setting keep alive seconds to zero.
        */
        if( AwsIotDemo_GetNetworkType( xNetworkConnection ) == AWSIOT_NETWORK_TYPE_BLE )
        {
            xConnectInfo.awsIotMqttMode = false;
            xConnectInfo.keepAliveSeconds = 0;
        }
        else
        {
            xConnectInfo.awsIotMqttMode = true;
            xConnectInfo.keepAliveSeconds = echoKEEPALIVE_SECONDS;
        }

        xConnectInfo.cleanSession = true;
        xConnectInfo.clientIdentifierLength = strlen( clientcredentialIOT_THING_NAME );
        xConnectInfo.pClientIdentifier = clientcredentialIOT_THING_NAME;

        /* Connect to the IoT broker endpoint */
        if( AwsIotMqtt_Connect( &xMqttConnection,
                &xNetworkInterface,
                &xConnectInfo,
                NULL,
                echoMQTT_TIMEOUT_MS ) == AWS_IOT_MQTT_SUCCESS )
        {
            xStatus = true;
        }

        if( xStatus == true )
        {
            /* MQTT Connection succeeded, subscribe to the topic */
            if( prxSubscribeorUnsubscribeToTopic( true ) != AWS_IOT_MQTT_SUCCESS )
            {
                xStatus = false;
            }
        }

        if( xStatus == false )
        {
        	/* Close the MQTT connection to perform any cleanup */
        	prvCloseMqttConnection( true );
        }

    }

    return xStatus;

}

void prvCloseMqttConnection( bool bSendDisconnect )
{
    /* Close the MQTT connection either by sending a DISCONNECT operation or not */
    if( xMqttConnection != AWS_IOT_MQTT_CONNECTION_INITIALIZER )
    {
        AwsIotMqtt_Disconnect( xMqttConnection, !(bSendDisconnect) );
        xMqttConnection = AWS_IOT_MQTT_CONNECTION_INITIALIZER;
    }
    /* Delete the network connection */
    if( xNetworkConnection != NULL )
    {
    	AwsIotDemo_DeleteNetworkConnection( xNetworkConnection );
        xNetworkConnection = NULL;
    }
}

static void prvOnNetworkDisconnect( AwsIotDemoNetworkConnection_t xConnection )
{
    xNetworkConnected = false;
}

void vMqttPubTask( void * pvParameters  )
{
    uint16_t receiveValue = 0;
    uint32_t disconnectCountDown = DISCONNECT_COUNT_DOWN;
    AwsIotMqttError_t xError = AWS_IOT_MQTT_SUCCESS ;

    xNetworkConnected = prbOpenMqttConnection();

    while (1)
    {
        if( xNetworkConnected && disconnectCountDown)
        {
            xSocketConnected = true;
            /* Block to wait for data from publisher. */
            if (xQueueReceive(xTelemetryQueue, &receiveValue, portMAX_DELAY) == pdPASS)
            {
                xError = prxPublishMessage((uint32_t) receiveValue);
                if( xError == AWS_IOT_MQTT_SUCCESS)
                {
                    AwsIotLogInfo( "vMqttPubTask:Pub Thread Sent SUCCESS");
                    disconnectCountDown = DISCONNECT_COUNT_DOWN;
                }
                else if( xError == AWS_IOT_MQTT_SEND_ERROR)
                {
                    AwsIotLogInfo( "vMqttPubTask: publish SEND ERROR\n");
                    disconnectCountDown--;
                    vTaskDelay( ONE_SECOND_DELAY_IN_TICKS );
                }
                else if( xError == AWS_IOT_MQTT_TIMEOUT)
                {
                    AwsIotLogInfo( "vMqttPubTask: publish TIMEOUT error\n");
                    vTaskDelay( ONE_SECOND_DELAY_IN_TICKS );
                    disconnectCountDown--;
                }
                else if( ( xError == AWS_IOT_MQTT_NO_MEMORY ) || ( xError == AWS_IOT_MQTT_BAD_PARAMETER ) )
                {
                    AwsIotLogInfo( "vMqttPubTask:Pub Task Failed to publish Message, error = %s", AwsIotMqtt_strerror( xError ) );
                    break;
                }

            }
        }
        else 
        {
            xSocketConnected = false;
            AwsIotLogInfo( "vMqttPubTask; Socket connection dropped, reconnect\n" );
            xSemaphoreTake(xOtaProtocolUsingSocket, portMAX_DELAY);
            AwsIotLogInfo( "vMqttPubTask; Socket connection dropped, have semaphore\n" );
            prvCloseMqttConnection( false );
            AwsIotLogInfo( "vMqttPubTask; Socket connection dropped, completed mqtt close\n" );
            xNetworkConnected = prbOpenMqttConnection();
            AwsIotLogInfo( "vMqttPubTask; Socket connection dropped, completed open\n" );
            if (xNetworkConnected)
            {
                xSocketConnected = true;
                AwsIotLogInfo( "vMqttPubTask; Socket connection now back\n" );
            }
            xSemaphoreGive(xOtaProtocolUsingSocket);
            disconnectCountDown = DISCONNECT_COUNT_DOWN;
        }
    }
}

void vStartCombinedDemo( void )
{
    bool goodStart = false;
    AwsIotLogInfo( "vStartCombinedDemo; ****** starting ****** \n");

	configPRINTF ( ("Combined demo version %u.%u.%u\r\n",
        xAppFirmwareVersion.u.x.ucMajor,
        xAppFirmwareVersion.u.x.ucMinor,
        xAppFirmwareVersion.u.x.usBuild ) );
            xTelemetryQueue = xQueueCreate( producerQUEUE_LENGTH, sizeof( uint16_t ) );
    if( xTelemetryQueue != NULL )
    {
        xOtaProtocolUsingSocket = xSemaphoreCreateBinary();

        if( xOtaProtocolUsingSocket != NULL )
        {
            xSemaphoreGive(xOtaProtocolUsingSocket);
            xTaskCreate( vMqttPubTask,
                "MqttPub",
                democonfigMQTT_PUB_TASK_STACK_SIZE,
                NULL,
                democonfigMQTT_PUB_TASK_PRIORITY,
                NULL);

            AwsIotLogInfo( "vStartCombinedDemo: started vMqttPubTask " );

            xTaskCreate(vProducerTask,
                "Producer",
                democonfigPRODUCER_TASK_STACK_SIZE,
                NULL,
                democonfigPRODUCER_TASK_PRIORITY,
                NULL);

            AwsIotLogInfo( "vStartCombinedDemo: started vProducerTask" );

            xTaskCreate(vBLEButtonTask,
                "BLE_Button",
                democonfigBLE_BUTTON_TASK_STACK_SIZE,
                NULL,
                democonfigBLE_BUTTON_TASK_PRIORITY,
                NULL);

            AwsIotLogInfo( "vStartCombinedDemo: started vBLEButtonTask" );

            xTaskCreate( vOtaTask,
                "OTA",
                democonfigCOMBINED_OTA_TASK_STACK_SIZE,
                NULL,
                democonfigCOMBINED_OTA_TASK_PRIORITY,
                NULL);
            AwsIotLogInfo( "vStartCombinedDemo: started vOtaTask" );
            goodStart = true;
        }
    }

    if (goodStart)
    {
        AwsIotLogInfo( "vStartCombinedDemo; ****** exiting - good start ****** \n");
    }
    else
    {
        AwsIotLogInfo( "vStartCombinedDemo; ****** exiting - bad start ****** \n");
    }

}
