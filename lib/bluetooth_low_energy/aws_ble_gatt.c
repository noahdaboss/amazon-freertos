/*
 * Amazon FreeRTOS
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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

/**
 * @file aws_BLE.c
 * @brief BLE GATT API.
 */

#include "string.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include "bt_hal_manager_adapter_ble.h"
#include "bt_hal_manager.h"
#include "bt_hal_gatt_server.h"
#include "aws_ble.h"
#include "aws_ble_internals.h"

static void vServerRegisteredCb( BTStatus_t xStatus,
                                 uint8_t ucServerIf,
                                 BTUuid_t * pxAppUuid );
static void vServerUnregisteredCb( BTStatus_t xStatus,
                                   uint8_t ucServerIf );
static void vConnectionCb( uint16_t usConnId,
                           uint8_t ucServerIf,
                           bool bConnected,
                           BTBdaddr_t * pxBda );
static void vSeviceAddedCb( BTStatus_t xStatus,
                            uint8_t ucServerIf,
                            BTGattSrvcId_t * pxSrvcId,
                            uint16_t usSrvcHandle );
static void vCharAddedCb( BTStatus_t xStatus,
                          uint8_t ucServerIf,
                          BTUuid_t * pxUuid,
                          uint16_t usSrvcHandle,
                          uint16_t usHandle );
static void vCharDescrAddedCb( BTStatus_t xStatus,
                               uint8_t ucServerIf,
                               BTUuid_t * pxUuid,
                               uint16_t usSrvcHandle,
                               uint16_t usHandle );
static void vServiceStartedCb( BTStatus_t xStatus,
                               uint8_t ucServerIf,
                               uint16_t usSrvcHandle );
static void vServiceStoppedCb( BTStatus_t xStatus,
                               uint8_t ucServerIf,
                               uint16_t usSrvcHandle );
static void vServiceDeletedCb( BTStatus_t xStatus,
                               uint8_t ucServerIf,
                               uint16_t usSrvcHandle );
static void vIncludedServiceAdded( BTStatus_t xStatus,
                                   uint8_t ucServerIf,
                                   uint16_t usSrvcHandle,
                                   uint16_t usInclSrvcHandle );
static void vRequestReadCb( uint16_t usConnId,
                            uint32_t ulTransId,
                            BTBdaddr_t * pxBda,
                            uint16_t usAttrHandle,
                            uint16_t usOffset );
static void vRequestWriteCb( uint16_t usConnId,
                             uint32_t ulTransId,
                             BTBdaddr_t * pxBda,
                             uint16_t usAttrHandle,
                             uint16_t usOffset,
                             size_t xLength,
                             bool bNeedRsp,
                             bool bIsPrep,
                             uint8_t * pucValue );
static void vExecWriteCb( uint16_t usConnId,
                          uint32_t ulTransId,
                          BTBdaddr_t * pxBda,
                          bool bExecWrite );
static void vMtuChangedCb( uint16_t usConnId,
                           uint16_t usMtu );
static void vResponseConfirmationCb( BTStatus_t xStatus,
                                     uint16_t usHandle );
static void vIndicationSentCb( uint16_t usConnId,
                               BTStatus_t xStatus );

BTGattServerCallbacks_t xBTGattServerCb =
{
    .pxRegisterServerCb       = vServerRegisteredCb,
    .pxUnregisterServerCb     = vServerUnregisteredCb,
    .pxConnectionCb           = vConnectionCb,
    .pxServiceAddedCb         = vSeviceAddedCb,
    .pxIncludedServiceAddedCb = vIncludedServiceAdded,
    .pxCharacteristicAddedCb  = vCharAddedCb,
    .pxSetValCallbackCb       = NULL,
    .pxDescriptorAddedCb      = vCharDescrAddedCb,
    .pxServiceStartedCb       = vServiceStartedCb,
    .pxServiceStoppedCb       = vServiceStoppedCb,
    .pxServiceDeletedCb       = vServiceDeletedCb,
    .pxRequestReadCb          = vRequestReadCb,
    .pxRequestWriteCb         = vRequestWriteCb,
    .pxRequestExecWriteCb     = vExecWriteCb,
    .pxResponseConfirmationCb = vResponseConfirmationCb,
    .pxIndicationSentCb       = vIndicationSentCb,
    .pxCongestionCb           = NULL,
    .pxMtuChangedCb           = vMtuChangedCb
};

/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/

void prvServiceClean( BLEServiceListElement_t * pxServiceElem )
{
    size_t xCharId;

    if( pxServiceElem != NULL )
    {
        listREMOVE( &pxServiceElem->xServiceList );
        vPortFree( pxServiceElem );
    }
}

BLEServiceListElement_t * prvGetServiceListElemFromHandle( uint16_t usSrvcHandle )
{
    Link_t * pxTmpElem;
    BLEServiceListElement_t * pxServiceElem = NULL;

   if( xSemaphoreTake( ( SemaphoreHandle_t ) &xBTInterface.xThreadSafetyMutex, portMAX_DELAY ) == pdPASS )
    {
        /* Remove service from service list */
        listFOR_EACH( pxTmpElem, &xBTInterface.xServiceListHead )
        {
            pxServiceElem = listCONTAINER( pxTmpElem, BLEServiceListElement_t, xServiceList );

            if( pxServiceElem->pxService->pusHandlesBuffer[0] == usSrvcHandle )
            {
                    break;
            }
        }
        xSemaphoreGive( ( SemaphoreHandle_t ) &xBTInterface.xThreadSafetyMutex );
    }
    return pxServiceElem;
}

/*-----------------------------------------------------------*/

BaseType_t prvGetAttributeAndCbFromHandle( uint16_t usAttrHandle,
                                           BLEAttribute_t ** ppxAttribute,
                                           BLEAttributeEventCallback_t * pxEventsCallbacks )
{
    Link_t * pxTmpElem;
    BLEServiceListElement_t * pxServiceElem;
    BaseType_t bFoundService = pdFAIL;
    size_t xAttributeIndex;

    /* The service that was just added is the last in the list */
    pxServiceElem = prvGetServiceListElemFromHandle(usAttrHandle);

    if( pxServiceElem != NULL)
    {
        for(xAttributeIndex = 0; xAttributeIndex  < pxServiceElem->pxService->xNumberOfAttributes; xAttributeIndex++)
        {
            if(pxServiceElem->pxService->pusHandlesBuffer[xAttributeIndex] == usAttrHandle)
            {
                *ppxAttribute = &pxServiceElem->pxService->pxBLEAttributes[xAttributeIndex];
                *pxEventsCallbacks = pxServiceElem->pxEventsCallbacks[xAttributeIndex];
                bFoundService = pdPASS;
                break; 
            }
        }
     }

    return bFoundService;
}

/*-----------------------------------------------------------*/

void vServerRegisteredCb( BTStatus_t xStatus,
                          uint8_t ucServerIf,
                          BTUuid_t * pxAppUuid )
{
    xBTInterface.ucServerIf = ucServerIf;
    xBTInterface.xCbStatus = xStatus;
    ( void ) xEventGroupSetBits( ( EventGroupHandle_t ) &xBTInterface.xWaitOperationComplete, 1 << eBLEHALEventServerRegisteredCb );
}

/*-----------------------------------------------------------*/

void vServerUnregisteredCb( BTStatus_t xStatus,
                            uint8_t ucServerIf )
{
    xBTInterface.ucServerIf = ucServerIf;
    xBTInterface.xCbStatus = xStatus;
    ( void ) xEventGroupSetBits( ( EventGroupHandle_t ) &xBTInterface.xWaitOperationComplete, 1 << eBLEHALEventServerRegisteredCb );
}

/*-----------------------------------------------------------*/

void vConnectionCb( uint16_t usConnId,
                    uint8_t ucServerIf,
                    bool bConnected,
                    BTBdaddr_t * pxBda )
{
    BTStatus_t xStatus = eBTStatusSuccess;
    BLEConnectionInfoListElement_t * pxConnInfoListElem;
    Link_t * pxEventListIndex;
    BLESubscrEventListElement_t * pxEventIndex;

    if( xSemaphoreTake( ( SemaphoreHandle_t ) &xBTInterface.xThreadSafetyMutex, portMAX_DELAY ) == pdPASS )
    {
        if( bConnected == true )
        {
            /* Add a new connection to the list */
            pxConnInfoListElem = pvPortMalloc( sizeof( BLEConnectionInfoListElement_t ) );

            if( pxConnInfoListElem != NULL )
            {
                /* Initialize the new connection element */
                listINIT_HEAD( &pxConnInfoListElem->xClientCharDescrListHead );
                memcpy( &pxConnInfoListElem->pxRemoteBdAddr, pxBda, sizeof( BTBdaddr_t ) );
                pxConnInfoListElem->usConnId = usConnId;

                listADD( &xBTInterface.xConnectionListHead, &pxConnInfoListElem->xConnectionList );
            }
            else
            {
                xStatus = eBTStatusNoMem;
            }
        }
        else
        {
            xStatus = BLE_GetConnectionInfo( usConnId, &pxConnInfoListElem );

            if( xStatus == eBTStatusSuccess )
            {
                /* Remove connection from the list safely */
                listREMOVE( &pxConnInfoListElem->xConnectionList );
                vPortFree( pxConnInfoListElem );
            }
        }

        /* Get the event associated to the callback */
        listFOR_EACH( pxEventListIndex, &xBTInterface.xSubscrEventListHead[ eBLEConnection ] )
        {
            pxEventIndex = listCONTAINER( pxEventListIndex, BLESubscrEventListElement_t, xEventList );
            pxEventIndex->xSubscribedEventCb.pxConnectionCb( xStatus, usConnId, bConnected, pxBda );
        }
        ( void ) xSemaphoreGive( ( SemaphoreHandle_t ) &xBTInterface.xThreadSafetyMutex );
    }
}

/*-----------------------------------------------------------*/

BLEServiceListElement_t * prvGetLastAddedServiceElem( void )
{
    BLEServiceListElement_t * pxServiceElem = NULL;

    /* The service that was just added is the first in the list */
    if( !listIS_EMPTY( &xBTInterface.xServiceListHead ) )
    {
        pxServiceElem = listCONTAINER( xBTInterface.xServiceListHead.pxNext, BLEServiceListElement_t, xServiceList );
    }

    return pxServiceElem;
}

/*-----------------------------------------------------------*/
void prvAttributeAdded(uint16_t usHandle, BTStatus_t xStatus, BLEHALEventsInternals_t xEvent )
{
    BLEServiceListElement_t * pxServiceElem = prvGetLastAddedServiceElem();
    uint16_t usIndex;

    /* Now that service is found, add the handle */
    if( pxServiceElem != NULL )
    {
        for(usIndex = 0; usIndex < pxServiceElem->pxService->xNumberOfAttributes; usIndex++)
        {
          if(pxServiceElem->pxService->pusHandlesBuffer[usIndex] == 0)
           {
              pxServiceElem->pxService->pusHandlesBuffer[usIndex] = usHandle; 
              break;
           }
        } 
        pxServiceElem->usEndHandle = usHandle;
    }

    xBTInterface.xCbStatus = xStatus;
    ( void ) xEventGroupSetBits( ( EventGroupHandle_t ) &xBTInterface.xWaitOperationComplete, 1 << xEvent );
}

void vSeviceAddedCb( BTStatus_t xStatus,
                     uint8_t ucServerIf,
                     BTGattSrvcId_t * pxSrvcId,
                     uint16_t usSrvcHandle )
{
  prvAttributeAdded(usSrvcHandle,  xStatus, eBLEHALEventServiceAddedCb);
}

/*-----------------------------------------------------------*/

void vCharAddedCb( BTStatus_t xStatus,
                   uint8_t ucServerIf,
                   BTUuid_t * pxUuid,
                   uint16_t usSrvcHandle,
                   uint16_t usHandle )
{
    prvAttributeAdded(usSrvcHandle,  xStatus, eBLEHALEventCharAddedCb);
}

/*-----------------------------------------------------------*/

void vCharDescrAddedCb( BTStatus_t xStatus,
                        uint8_t ucServerIf,
                        BTUuid_t * pxUuid,
                        uint16_t usSrvcHandle,
                        uint16_t usHandle )
{
    prvAttributeAdded(usSrvcHandle,  xStatus, eBLEHALEventCharDescrAddedCb);
}


void vIncludedServiceAdded( BTStatus_t xStatus,
                            uint8_t ucServerIf,
                            uint16_t usSrvcHandle,
                            uint16_t usInclSrvcHandle )
{
    prvAttributeAdded(usSrvcHandle,  xStatus, eBLEHALEventIncludedServiceAdded);
}

/*-----------------------------------------------------------*/

void vServiceStartedCb( BTStatus_t xStatus,
                        uint8_t ucServerIf,
                        uint16_t usSrvcHandle )
{
    xBTInterface.xCbStatus = xStatus;
    ( void ) xEventGroupSetBits( ( EventGroupHandle_t ) &xBTInterface.xWaitOperationComplete, 1 << eBLEHALEventSeviceStartedCb );
}

/*-----------------------------------------------------------*/

void vServiceStoppedCb( BTStatus_t xStatus,
                        uint8_t ucServerIf,
                        uint16_t usSrvcHandle )
{
     xBTInterface.xCbStatus = xStatus;
    ( void ) xEventGroupSetBits( ( EventGroupHandle_t ) &xBTInterface.xWaitOperationComplete, 1 << eBLEHALEventSeviceStoppedCb );
}

/*-----------------------------------------------------------*/

void vServiceDeletedCb( BTStatus_t xStatus,
                        uint8_t ucServerIf,
                        uint16_t usSrvcHandle )
{
    BLEServiceListElement_t * pxServiceElem;

    /* The service has been stopped so it can be deleted safely */
    pxServiceElem = prvGetServiceListElemFromHandle( usSrvcHandle );

    if( pxServiceElem != NULL )
    {
    	prvServiceClean( pxServiceElem );
    }
    else
    {
    	xStatus = eBTStatusFail;
    }

    xBTInterface.xCbStatus = xStatus;
    ( void ) xEventGroupSetBits( ( EventGroupHandle_t ) &xBTInterface.xWaitOperationComplete, 1 << eBLEHALEventServiceDeletedCb );
}

/*-----------------------------------------------------------*/

void vRequestReadCb( uint16_t usConnId,
                     uint32_t ulTransId,
                     BTBdaddr_t * pxBda,
                     uint16_t usAttrHandle,
                     uint16_t usOffset )
{
    BLEAttribute_t * pxAttribute;
    BLEAttributeEventCallback_t xEventsCallbacks;
    BLEReadEventParams_t xReadParam;
    BLEAttributeEvent_t xEventParam;

    if( prvGetAttributeAndCbFromHandle( usAttrHandle, &pxAttribute, &xEventsCallbacks ) == pdPASS )
    {
        xReadParam.pxAttribute = &xAttribute;
        xReadParam.pxRemoteBdAddr = pxBda;
        xReadParam.ulTransId = ulTransId;
        xReadParam.usConnId = usConnId;
        xReadParam.usOffset = usOffset;

        xEventParam.xEventType = eBLERead;
        xEventParam.pxParamRead = &xReadParam;

        prvTriggerAttriButeCallback( &xAttribute, &xEventParam );
    }
}

/*-----------------------------------------------------------*/

void vRequestWriteCb( uint16_t usConnId,
                      uint32_t ulTransId,
                      BTBdaddr_t * pxBda,
                      uint16_t usAttrHandle,
                      uint16_t usOffset,
                      size_t xLength,
                      bool bNeedRsp,
                      bool bIsPrep,
                      uint8_t * pucValue )
{
    BLEAttribute_t * pxAttribute;
    BLEWriteEventParams_t xWriteParam;
    BLEAttributeEvent_t xEventParam;
    BLEAttributeEventCallback_t xEventsCallbacks;

    if( prvGetAttributeAndCbFromHandle( usAttrHandle, &pxAttribute, &xEventsCallbacks ) == pdPASS )
    {
        if( bIsPrep == true )
        {
            xBTInterface.usHandlePendingPrepareWrite = usAttrHandle;
        }

        if( bNeedRsp == true )
        {
            xEventParam.xEventType = eBLEWrite;
        }
        else
        {
            xEventParam.xEventType = eBLEWriteNoResponse;
        }

        xWriteParam.pxAttribute = &xAttribute;
        xWriteParam.bIsPrep = bIsPrep;
        xWriteParam.pucValue = pucValue;
        xWriteParam.pxRemoteBdAddr = pxBda;
        xWriteParam.ulTransId = ulTransId;
        xWriteParam.usConnId = usConnId;
        xWriteParam.usOffset = usOffset;
        xWriteParam.xLength = xLength;

        xEventParam.pxParamWrite = &xWriteParam;

        prvTriggerAttriButeCallback( &xAttribute, &xEventParam );
    }
}

/*-----------------------------------------------------------*/

void vExecWriteCb( uint16_t usConnId,
                   uint32_t ulTransId,
                   BTBdaddr_t * pxBda,
                   bool bExecWrite )
{
    BLEAttribute_t * pxAttribute;
    BLEExecWriteEventParams_t xExecWriteParam;
    BLEAttributeEvent_t xEventParam;
    BLEAttributeEventCallback_t xEventsCallbacks;

    if( prvGetAttributeAndCbFromHandle( xBTInterface.usHandlePendingPrepareWrite, &pxAttribute, &xEventsCallbacks ) == pdPASS )
    {
        xExecWriteParam.pxAttribute = &xAttribute;
        xExecWriteParam.pxRemoteBdAddr = pxBda;
        xExecWriteParam.ulTransId = ulTransId;
        xExecWriteParam.usConnId = usConnId;

        xEventParam.xEventType = eBLEExecWrite;
        xEventParam.pxParamExecWrite = &xExecWriteParam;

        prvTriggerAttriButeCallback( &xAttribute, &xEventParam );
    }
}

/*-----------------------------------------------------------*/

void vMtuChangedCb( uint16_t usConnId,
                    uint16_t usMtu )
{
    Link_t * pxEventListIndex;
    BLESubscrEventListElement_t * pxEventIndex;

    if( xSemaphoreTake( ( SemaphoreHandle_t ) &xBTInterface.xThreadSafetyMutex, portMAX_DELAY ) == pdPASS )
    {
        /* Get the event associated to the callback */
        listFOR_EACH( pxEventListIndex, &xBTInterface.xSubscrEventListHead[ eBLEMtuChanged ] )
        {
            pxEventIndex = listCONTAINER( pxEventListIndex, BLESubscrEventListElement_t, xEventList );
            pxEventIndex->xSubscribedEventCb.pxMtuChangedCb( usConnId, usMtu );
        }
        xSemaphoreGive( ( SemaphoreHandle_t ) &xBTInterface.xThreadSafetyMutex );
    }
}

/*-----------------------------------------------------------*/

static void vResponseConfirmationCb( BTStatus_t xStatus,
                                     uint16_t usHandle )
{
    BLEAttribute_t * pxAttribute;
    BLERespConfirmEventParams_t xRespConfirmParam;
    BLEAttributeEvent_t xEventParam;
    BLEAttributeEventCallback_t xEventsCallbacks;

    if( prvGetAttributeAndCbFromHandle( usHandle, &pxAttribute,  &xEventsCallbacks ) == pdPASS )
    {
        xRespConfirmParam.pxAttribute = &xAttribute;
        xRespConfirmParam.xStatus = xStatus;

        xEventParam.xEventType = eBLEResponseConfirmation;
        xEventParam.pxParamRespConfim = &xRespConfirmParam;

        prvTriggerAttriButeCallback( &xAttribute, &xEventParam );
    }
}

/*-----------------------------------------------------------*/

static void vIndicationSentCb( uint16_t usConnId,
                               BTStatus_t xStatus )
{
    BLEAttribute_t xAttribute;
    BLEIndicationSentEventParams_t xIndicationSentParam;
    BLEAttributeEvent_t xEventParam;
    BLEAttributeEventCallback_t pxEventsCallbacks;

    if( prvGetAttributeAndCbFromHandle( xBTInterface.usHandlePendingIndicationResponse, &xAttribute ) == pdPASS )
    {
        xIndicationSentParam.pxAttribute = &xAttribute;
        xIndicationSentParam.usConnId = usConnId;
        xIndicationSentParam.xStatus = xStatus;

        xEventParam.pxParamIndicationSent = &xIndicationSentParam;
        xEventParam.xEventType = eBLEIndicationConfirmReceived;

        prvTriggerAttriButeCallback( &xAttribute, &xEventParam );
    }
}

/*-----------------------------------------------------------*/
BTStatus_t prvAddServiceToList(BLEService_t * pxService, BLEAttributeEventCallback_t pxEventsCallbacks[])
{
  BTStatus_t xStatus = eBTStatusSuccess;
  BLEServiceListElement_t * pxServiceElem;

  /* Create a space in the list for the service. */
  pxNewElem = pvPortMalloc( sizeof( BLEServiceListElement_t ) );
  memset( pxNewElem, 0, sizeof( BLEServiceListElement_t ) );

  if( pxNewElem != NULL )
  {
    pxNewElem->pxEventsCallbacks = pxEventsCallbacks;
    pxNewElem->pxService = pxService;

    if( pxNewElem->pxAttributesPtr != NULL )
    {
        if( xSemaphoreTake( ( SemaphoreHandle_t ) &xBTInterface.xThreadSafetyMutex, portMAX_DELAY ) == pdPASS )
        {
          listADD( &xBTInterface.xServiceListHead, &pxNewElem->xServiceList );
          xSemaphoreGive( ( SemaphoreHandle_t ) &xBTInterface.xThreadSafetyMutex );
        }
    }
    else
    {
        xStatus = eBTStatusNoMem;
    }
  }
  else
  {
    xStatus = eBTStatusFail;
  }

  return eBTStatusSuccess;
}

BTStatus_t prvCreateAttributes(BLEService_t * pxService)
{
  uint16_t usAttributes = 0;
  BLEAttribute_t * pxCurrentAtrribute;
  BTStatus_t xStatus = eBTStatusParamInvalid;

   for(usAttributes = 0; usAttributes < pxService->xNumberOfAttributes; usAttributes++)
    {
        pxCurrentAtrribute = &pxService->pxBLEAttributes[usAttributes];
        switch(pxCurrentAtrribute->xAttributeType)
        {
            case eBTDbIncludedService:
            {

              break;
            }
            case eBTDbCharacteristicDecl:
            {

              break;
            }
            case eBTDbCharacteristic:
            {
              xBTInterface.pxGattServerInterface->pxAddCharacteristic( xBTInterface.ucServerIf,
                                                                       pxService->xAttributeData.xHandle,
                                                                       &pxCharacteristic->xAttributeData.xUuid,
                                                                       pxCharacteristic->xProperties,
                                                                       pxCharacteristic->xPermissions );
              break;
            }
            default:
        }


        if(xStatus != eBTStatusSuccess)
        {
          break;
        }
    }
    return xStatus;
}

BTStatus_t BLE_CreateService( BLEService_t * pxService, BLEAttributeEventCallback_t pxEventsCallbacks[] )
{
  BTStatus_t xStatus = eBTStatusParamInvalid;

  if(pxService == NULL)
  {
    memset(pxService->pusHandlesBuffer, 0, pxService->xNumberOfAttributes);
    /* Create Service. Start handle is 0 so the number of handle is usEndHandle + 1.
     * The real handle is assigned in the callback of that call.*/
    xStatus = xBTInterface.pxGattServerInterface->pxAddService( xBTInterface.ucServerIf,
                                                      &pxService->pxBLEAttributes[0]->xSrvcId,
                                                      pxService->xNumberOfAttributes );

    xEventGroupWaitBits( ( EventGroupHandle_t ) &xBTInterface.xWaitOperationComplete,
                                             1 << eBLEHALEventSeviceAddedCb,
                                             pdTRUE,
                                             pdTRUE,
                                             portMAX_DELAY );
  }

  /* Create all attributes. */
  if(xStatus == eBTStatusSuccess)
  {
     xStatus = prvCreateAttributes(pxService);
  }

  /* After all attributes have been create successfully, the service is added to the list. */
  if(xStatus == eBTStatusSuccess)
  {
     xStatus = prvAddServiceToList(pxService, pxEventsCallbacks);
  }else /* Otherwise the service is destroyed. */
  {

  }


    return xStatus;
}

/*-----------------------------------------------------------*/

BTStatus_t BLE_AddService( BLEService_t * pxService )
{
    if (pxService != NULL)
    {














    BTStatus_t xStatus = eBTStatusSuccess;
    BTGattSrvcId_t pxSrvcId;
    BLEServiceListElement_t * pxServiceElem;
    size_t xIndex;
    size_t xAttributeCounter = 0;

    if( pxService != NULL )
    {
        if( xSemaphoreTake( ( SemaphoreHandle_t ) &xBTInterface.xThreadSafetyMutex, portMAX_DELAY ) == pdPASS )
        {
            pxSrvcId.xId.ucInstId = pxService->ucInstId;
            pxSrvcId.xServiceType = pxService->xServiceType;
            pxSrvcId.xId.xUuid.ucType = pxService->xAttributeData.xUuid.ucType;

            memcpy( pxSrvcId.xId.xUuid.uu.uu128, pxService->xAttributeData.xUuid.uu.uu128, bt128BIT_UUID_LEN );
            pxServiceElem = prvGetLastAddedServiceElem();

            if(pxServiceElem != NULL)
            {
				if( pxService->xServiceType == eBTServiceTypePrimary )
				{
					pxServiceElem->pxAttributesPtr[ xAttributeCounter ].xAttributeType = eBTDbPrimaryService;
				}
				else
				{
					pxServiceElem->pxAttributesPtr[ xAttributeCounter ].xAttributeType = eBTDbSecondaryService;
				}

				pxServiceElem->pxAttributesPtr[ xAttributeCounter++ ].pxService = pxService;

				/* Create Service. Start handle is 0 so the number of handle is usEndHandle + 1.
				 * The real handle is assigned in the callback of that call.*/
				xBTInterface.pxGattServerInterface->pxAddService( xBTInterface.ucServerIf,
																  &pxSrvcId,
																  pxServiceElem->usEndHandle + 1 );
				xEventGroupWaitBits( ( EventGroupHandle_t ) &xBTInterface.xWaitOperationComplete,
									 1 << eBLEHALEventSeviceAddedCb,
									 pdTRUE,
									 pdTRUE,
									 portMAX_DELAY );

				xStatus = xBTInterface.xCbStatus;
            }else
            {
                xStatus = eBTStatusFail;
            }

            /* Create Characteristics and associated descriptors. */
            if( xStatus == eBTStatusSuccess )
            {
                for( xIndex = 0; xIndex < pxService->xNbCharacteristics; xIndex++ )
                {
                    xStatus = prvCreateCharacteristicAndDescriptors( pxService, &pxService->pxCharacteristics[ xIndex ], pxServiceElem->pxAttributesPtr, &xAttributeCounter );

                    if( xStatus != eBTStatusSuccess )
                    {
                        break;
                    }
                }
            }

            /* Create Included Services. */
            if( xStatus == eBTStatusSuccess )
            {
                for( xIndex = 0; xIndex < pxService->xNbIncludedServices; xIndex++ )
                {
                    pxServiceElem->pxAttributesPtr[ xAttributeCounter ].xAttributeType = eBTDbIncludedService;
                    pxServiceElem->pxAttributesPtr[ xAttributeCounter++ ].pxIncludedService = &pxService->pxIncludedServices[ xIndex ];

                    pxService->pxIncludedServices[ xIndex ].xAttributeData.xHandle = 0;
                    xBTInterface.pxGattServerInterface->pxAddIncludedService( xBTInterface.ucServerIf,
                                                                              pxService->xAttributeData.xHandle,
                                                                              pxService->pxIncludedServices->pxPtrToService->xAttributeData.xHandle );
                    xEventGroupWaitBits( ( EventGroupHandle_t ) &xBTInterface.xWaitOperationComplete,
                                         1 << eBLEHALEventIncludedServiceAdded,
										 pdTRUE,
										 pdTRUE,
                                         portMAX_DELAY );

                    if( ( xBTInterface.xCbStatus != eBTStatusSuccess ) ||
                        ( pxService->pxIncludedServices[ xIndex ].xAttributeData.xHandle == 0 ) )
                    {
                        xStatus = eBTStatusFail;
                        break;
                    }
                }
            }

            xSemaphoreGive( ( SemaphoreHandle_t ) &xBTInterface.xThreadSafetyMutex );
        }
        else
        {
            xStatus = eBTStatusFail;
        }
    }
    else
    {
        xStatus = eBTStatusParamInvalid;
    }

    return xStatus;
}

/*-----------------------------------------------------------*/

BTStatus_t BLE_StartService( BLEService_t * pxService)
{
    BTStatus_t xStatus = eBTStatusSuccess;

    if( pxService != NULL )
    {
        xBTInterface.pxPendingServiceEvent = ( void * ) pxCallback;
        xStatus = xBTInterface.pxGattServerInterface->pxStartService( xBTInterface.ucServerIf,
                                                                      pxService->xAttributeData.xHandle,
                                                                      BTTransportLe );
    }
    else
    {
        xStatus = eBTStatusFail;
    }

    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t BLE_StopService( BLEService_t * pxService,
                            BLEServiceStoppedCallback_t pxCallback )
{
    BTStatus_t xStatus = eBTStatusSuccess;

    if( pxService != NULL )
    {

    }
    else
    {
        xStatus = eBTStatusParamInvalid;
    }

    return xStatus;
}

/*-----------------------------------------------------------*/

BTStatus_t BLE_DeleteService( BLEService_t * pxService,
                              BLEServiceDeletedCallback_t pxCallback )
{
    BTStatus_t xStatus = eBTStatusSuccess;

    if( pxService != NULL )
    {
        xBTInterface.pxPendingServiceEvent = ( void * ) pxCallback;
        xStatus = xBTInterface.pxGattServerInterface->pxStopService( xBTInterface.ucServerIf,
                                                                     pxService->xAttributeData.xHandle );
        /* To DO remove service from the list */
        xBTInterface.pxPendingServiceEvent = ( void * ) pxCallback;
        xStatus = xBTInterface.pxGattServerInterface->pxDeleteService( xBTInterface.ucServerIf,
                                                                       pxService->xAttributeData.xHandle );
    }
    else
    {
        xStatus = eBTStatusParamInvalid;
    }

    return xStatus;
}

/*-----------------------------------------------------------*/

BTStatus_t BLE_SendIndication( BLEEventResponse_t * pxResp,
                               uint16_t usConnId,
                               bool bConfirm )
{
    BTStatus_t xStatus = eBTStatusSuccess;

    if( pxResp != NULL )
    {
        xStatus = xBTInterface.pxGattServerInterface->pxSendIndication( xBTInterface.ucServerIf,
                                                                        pxResp->pxAttrData->xHandle,
                                                                        usConnId,
                                                                        pxResp->pxAttrData->xSize,
                                                                        pxResp->pxAttrData->pucData,
                                                                        bConfirm );
    }
    else
    {
        xStatus = eBTStatusParamInvalid;
    }

    return xStatus;
}

/*-----------------------------------------------------------*/

BTStatus_t BLE_SendResponse( BLEEventResponse_t * pxResp,
                             uint16_t usConnId,
                             uint32_t ulTransId )
{
    BTStatus_t xStatus = eBTStatusSuccess;
    BTGattResponse_t xResponse;

    if( pxResp != NULL )
    {
        xResponse.xAttrValue.pucValue = pxResp->pxAttrData->pucData;
        xResponse.xAttrValue.usOffset = pxResp->xAttrDataOffset;
        xResponse.xAttrValue.usHandle = pxResp->pxAttrData->xHandle;
        xResponse.xAttrValue.xLen = pxResp->pxAttrData->xSize;
        xResponse.xAttrValue.xRspErrorStatus = pxResp->xRspErrorStatus;
        xBTInterface.usHandlePendingIndicationResponse = xResponse.xAttrValue.usHandle;

        xStatus = xBTInterface.pxGattServerInterface->pxSendResponse( usConnId,
                                                                      ulTransId,
                                                                      pxResp->xEventStatus,
                                                                      &xResponse );
    }
    else
    {
        xStatus = eBTStatusParamInvalid;
    }

    return xStatus;
}

/*-----------------------------------------------------------*/

BTStatus_t BLE_GetConnectionInfoList( Link_t ** ppxConnectionInfoList )
{
    *ppxConnectionInfoList = &xBTInterface.xConnectionListHead;

    return eBTStatusSuccess;
}

/*-----------------------------------------------------------*/

BTStatus_t BLE_GetConnectionInfo( uint16_t usConnId,
                                  BLEConnectionInfoListElement_t ** ppvConnectionInfo )
{
    BTStatus_t xStatus = eBTStatusFail;
    BLEConnectionInfoListElement_t * pxConnInfoListElem;
    Link_t * pxConnListIndex;

    listFOR_EACH( pxConnListIndex, &xBTInterface.xConnectionListHead )
    {
        pxConnInfoListElem = listCONTAINER( pxConnListIndex, BLEConnectionInfoListElement_t, xConnectionList );

        if( usConnId == pxConnInfoListElem->usConnId )
        {
            xStatus = eBTStatusSuccess;
            *ppvConnectionInfo = pxConnInfoListElem;
            break;
        }
    }

    return xStatus;
}
