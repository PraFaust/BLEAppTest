/*! *********************************************************************************
* \addtogroup Thermometer
* @{
********************************************************************************** */
/*!
* Copyright (c) 2014, Freescale Semiconductor, Inc.
* All rights reserved.
* \file app.c
* This file is the source file for the Thermometer application
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* o Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
*
* o Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* o Neither the name of Freescale Semiconductor, Inc. nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
/* Framework / Drivers */
#include "RNG_interface.h"
#include "Keyboard.h"
#include "Led.h"
#include "TimersManager.h"
#include "FunctionLib.h"
#include "fsl_os_abstraction.h"
#include "MemManager.h"
#include "panic.h"

/* BLE Host Stack */
#include "gatt_interface.h"
#include "gatt_server_interface.h"
#include "gatt_client_interface.h"
#include "gatt_database.h"
#include "gap_interface.h"
#include "gatt_db_app_interface.h"
#include "gatt_db_handles.h"

/* Profile / Services */
#include "battery_interface.h"
#include "device_info_interface.h"
#include "current_time_interface.h"
#include "health_thermometer_interface.h"
#include "rfs.h"

#include "board.h"
#include "ApplMain.h"
#include "app.h"

/************************************************************************************
*************************************************************************************
* Private constants & macros
*************************************************************************************
************************************************************************************/
/*! Thermometer Intermediate Temperature Update Rate in ms */
#define mIntTempUpdateRate_c 1000

#define mBatteryLevelReportInterval_c   (10)        /* battery level report interval in seconds  */
#define mLedUpdateInterval (1)                   // in ms
/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/
typedef enum
{
#if gBondingSupported_d
    fastWhiteListAdvState_c,
#endif
    fastAdvState_c,
    slowAdvState_c
}advType_t;

typedef struct advState_tag{
    bool_t      advOn;
    advType_t   advType;
}advState_t;

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/

/* Host Stack Data*/
static bleDeviceAddress_t   maBleDeviceAddress;
static deviceId_t           mPeerDeviceId = gInvalidDeviceId_c;

/* Adv State */
static advState_t  mAdvState;

#if gBondingSupported_d
static bleDeviceAddress_t   maPeerDeviceAddress;
static uint8_t mcBondedDevices = 0;
static bleAddressType_t     mPeerDeviceAddressType;
#endif


/* Service Data*/
static htsUserData_t    mUserData;

static htsConfig_t htsServiceConfig = {service_health_therm,
                                       mIntTempUpdateRate_c,
                                       gHts_UnitInCelsius_c,
                                       &mUserData};
static basConfig_t basServiceConfig = {service_battery, 0};
static LED_FORMAT ledData;
static rfsConfig_t rfsServiceConfig = {service_ready_for_sky, 0, &ledData};

static uint16_t cpHandles[] = { value_measure_int};

/* Application specific data*/
static tmrTimerID_t mAdvTimerId;
static tmrTimerID_t mMeasurementTimerId;
static tmrTimerID_t mBatteryMeasurementTimerId;
static tmrTimerID_t mLedUpdateTimerId;

/* Counts number of interm temp between temp measurements */
static uint8_t mIntermediateTempCounter = 0;

// Button status
static bool_t buttonPressed;

/************************************************************************************
*************************************************************************************
* Private functions prototypes
*************************************************************************************
************************************************************************************/

/* Gatt and Att callbacks */
static void BleApp_AdvertisingCallback (gapAdvertisingEvent_t* pAdvertisingEvent);
static void BleApp_ConnectionCallback (deviceId_t peerDeviceId, gapConnectionEvent_t* pConnectionEvent);
static void BleApp_GattServerCallback (deviceId_t deviceId, gattServerEvent_t* pServerEvent);
static void BleApp_Config();

/* Timer Callbacks */
static void AdvertisingTimerCallback (void *);
static void TimerMeasurementCallback (void *);
static void BatteryMeasurementTimerCallback (void *);
static void LedUpdateTimerCallback(void *);

static void BleApp_Advertise(void);

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief    Initializes application specific functionality before the BLE stack init.
*
********************************************************************************** */
void BleApp_Init(void)
{
    /* Initialize application support for drivers */
    BOARD_InitAdc();
}

/*! *********************************************************************************
* \brief    Starts the BLE application.
*
********************************************************************************** */
void BleApp_Start(void)
{
#if gBondingSupported_d
    if (mcBondedDevices > 0)
    {
        mAdvState.advType = fastWhiteListAdvState_c;
    }
    else
    {
#endif
        mAdvState.advType = fastAdvState_c;
#if gBondingSupported_d
    }
#endif

    BleApp_Advertise();
}

static inline void ToogleLedState(bool_t* ledState)
{
  if (*ledState) {
    *ledState = false;
  } else {
    *ledState = true;
  }
}

static inline void UpdateLedState(bool_t ledState)
{
  if (ledState) {
    Led4On();
  } else {
    Led4Off();
  }
}

/*! *********************************************************************************
* \brief        Handles keyboard events.
*
* \param[in]    events    Key event structure.
********************************************************************************** */
void BleApp_HandleKeys(key_event_t events)
{
  switch (events)
  {
  case gKBD_EventPressPB1_c:
    {
      BleApp_Start();
      break;
    }   
  case gKBD_EventLongPB1_c:
    {
      if (mPeerDeviceId != gInvalidDeviceId_c)
        Gap_Disconnect(mPeerDeviceId);
      break;
    }  
  case gKBD_EventPressPB2_c:
    {
      buttonPressed = 1;
      Rfs_RecordButtonValue(service_ready_for_sky, buttonPressed);
      ToogleLedState(&rfsServiceConfig.ledData->ledState);
      Rfs_RecordLedValue(service_ready_for_sky, rfsServiceConfig.ledData);
      break;
    }
  case gKBD_EventReleasePB2_c:
    {
      buttonPressed = 0;
      Rfs_RecordButtonValue(service_ready_for_sky, buttonPressed);
      ToogleLedState(&rfsServiceConfig.ledData->ledState);
      Rfs_RecordLedValue(service_ready_for_sky, rfsServiceConfig.ledData);
      break;
    }
  default:
    break;
  }
}

/*! *********************************************************************************
* \brief        Handles BLE generic callback.
*
* \param[in]    pGenericEvent    Pointer to gapGenericEvent_t.
********************************************************************************** */
void BleApp_GenericCallback (gapGenericEvent_t* pGenericEvent)
{
    switch (pGenericEvent->eventType)
    {
        case gInitializationComplete_c:    
        {
            BleApp_Config();
        }
        break;    
            
        case gPublicAddressRead_c:
        {
            /* Use address read from the controller */
            FLib_MemCpyReverseOrder(maBleDeviceAddress, pGenericEvent->eventData.aAddress, sizeof(bleDeviceAddress_t));
        }
        break;            
            
        case gAdvertisingDataSetupComplete_c:
        {            
        }
        break;  
        
        case gAdvertisingParametersSetupComplete_c:
        {
            App_StartAdvertising(BleApp_AdvertisingCallback, BleApp_ConnectionCallback);
        }
        break;         

        case gInternalError_c:
        {
            panic(0,0,0,0);
        }
        break;

        default: 
            break;
    }
}

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief        Configures BLE Stack after initialization. Usually used for
*               configuring advertising, scanning, white list, services, et al.
*
********************************************************************************** */
static void BleApp_Config()
{  
    /* Read public address from controller */
    Gap_ReadPublicDeviceAddress();

    /* Register for callbacks*/
    App_RegisterGattServerCallback(BleApp_GattServerCallback);
    GattServer_RegisterHandlesForWriteNotifications(NumberOfElements(cpHandles), cpHandles);    
    
    /* Register security requirements */
#if gUseServiceSecurity_d
    Gap_RegisterDeviceSecurityRequirements(&deviceSecurityRequirements);
#endif

    /* Set local passkey */
#if gBondingSupported_d
    Gap_SetLocalPasskey(gPasskeyValue_c);
#endif

    /* Setup Advertising and scanning data */
    Gap_SetAdvertisingData(&gAppAdvertisingData, &gAppScanRspData);

    /* Populate White List if bonding is supported */
#if gBondingSupported_d
    bleDeviceAddress_t aBondedDevAdds[gcGapMaximumBondedDevices_d];
    bleResult_t result = Gap_GetBondedStaticAddresses(aBondedDevAdds, gcGapMaximumBondedDevices_d, &mcBondedDevices);

    if (gBleSuccess_c == result && mcBondedDevices > 0)
    {
        for (uint8_t i = 0; i < mcBondedDevices; i++)
        {
            Gap_AddDeviceToWhiteList(gBleAddrTypePublic_c, aBondedDevAdds[i]);
        }
    }
#endif

    mAdvState.advOn = FALSE;

    /* Start services */
    mUserData.measIntervalIndPending = FALSE;
#if gHts_EnableStoredMeasurements_d    
    mUserData.cMeasurements = 0;
    mUserData.pStoredMeasurements = MEM_BufferAlloc(10);
#endif

    Hts_Start(&htsServiceConfig);
    
    basServiceConfig.batteryLevel = BOARD_GetBatteryLevel();
    Bas_Start(&basServiceConfig);
    Rfs_Start(&rfsServiceConfig);
    
    /* Allocate application timers */
    mAdvTimerId = TMR_AllocateTimer();
    mMeasurementTimerId = TMR_AllocateTimer();
    mBatteryMeasurementTimerId = TMR_AllocateTimer();
}

/*! *********************************************************************************
* \brief        Configures GAP Advertise parameters. Advertise will satrt after
*               the parameters are set.
*
********************************************************************************** */
static void BleApp_Advertise(void)
{
    uint32_t timeout;

    switch (mAdvState.advType)
    {
#if gBondingSupported_d
        case fastWhiteListAdvState_c:
        {
            gAdvParams.minInterval = gFastConnMinAdvInterval_c;
            gAdvParams.maxInterval = gFastConnMaxAdvInterval_c;
            gAdvParams.filterPolicy = (gapAdvertisingFilterPolicyFlags_t)(gProcessConnWhiteListFlag_c | gProcessScanWhiteListFlag_c);
            timeout = gFastConnWhiteListAdvTime_c;
        }
        break;
#endif
        case fastAdvState_c:
        {
            gAdvParams.minInterval = gFastConnMinAdvInterval_c;
            gAdvParams.maxInterval = gFastConnMaxAdvInterval_c;
            gAdvParams.filterPolicy = gProcessAll_c;
            timeout = gFastConnAdvTime_c - gFastConnWhiteListAdvTime_c;
        }
        break;

        case slowAdvState_c:
        {
            gAdvParams.minInterval = gReducedPowerMinAdvInterval_c;
            gAdvParams.maxInterval = gReducedPowerMinAdvInterval_c;
            gAdvParams.filterPolicy = gProcessAll_c;
            timeout = gReducedPowerAdvTime_c;
        }
        break;
    }

    /* Set advertising parameters*/
    Gap_SetAdvertisingParameters(&gAdvParams);

    /* Start advertising timer */
    TMR_StartLowPowerTimer(mAdvTimerId,gTmrLowPowerSecondTimer_c,
               TmrSeconds(timeout), AdvertisingTimerCallback, NULL);
}

/*! *********************************************************************************
* \brief        Handles BLE Advertising callback from host stack.
*
* \param[in]    pAdvertisingEvent    Pointer to gapAdvertisingEvent_t.
********************************************************************************** */
static void BleApp_AdvertisingCallback (gapAdvertisingEvent_t* pAdvertisingEvent)
{
    switch (pAdvertisingEvent->eventType)
    {
        case gAdvertisingStateChanged_c:
        {
            mAdvState.advOn = !mAdvState.advOn;
            LED_StopFlashingAllLeds();
            Led1Flashing();

            if(!mAdvState.advOn)
            {
                Led2Flashing();
                Led3Flashing();
                Led4Flashing();
            }
        }
        break;

        case gAdvertisingCommandFailed_c:
        {
            panic(0,0,0,0);
        }
        break;

        default:
            break;
    }
}

/*! *********************************************************************************
* \brief        Handles BLE Connection callback from host stack.
*
* \param[in]    peerDeviceId        Peer device ID.
* \param[in]    pConnectionEvent    Pointer to gapConnectionEvent_t.
********************************************************************************** */
static void BleApp_ConnectionCallback (deviceId_t peerDeviceId, gapConnectionEvent_t* pConnectionEvent)
{
    switch (pConnectionEvent->eventType)
    {
        case gConnEvtConnected_c:
        {
            mPeerDeviceId = peerDeviceId;
 
            /* Advertising stops when connected */
            mAdvState.advOn = FALSE;
            
#if gBondingSupported_d    
            /* Copy peer device address information */
            mPeerDeviceAddressType = pConnectionEvent->eventData.connectedEvent.peerAddressType;
            FLib_MemCpy(maPeerDeviceAddress, pConnectionEvent->eventData.connectedEvent.peerAddress, sizeof(bleDeviceAddress_t));
#endif             
#if gUseServiceSecurity_d            
            {
                bool_t isBonded = FALSE ;
                
                if (gBleSuccess_c == Gap_CheckIfBonded(peerDeviceId, &isBonded) &&
                    FALSE == isBonded) 
                {
                      Gap_SendSlaveSecurityRequest(peerDeviceId, TRUE, gSecurityMode_1_Level_3_c);
                }
            }
#endif            
            /* Subscribe client*/
            Bas_Subscribe(peerDeviceId);        
            Hts_Subscribe(peerDeviceId);
            Rfs_Subscribe(peerDeviceId);

            /* UI */
            LED_StopFlashingAllLeds();
            Led1On();
            
            /* Stop Advertising Timer*/
            mAdvState.advOn = FALSE;
            TMR_StopTimer(mAdvTimerId);
            
            /* Start measurements */
            TMR_StartLowPowerTimer(mMeasurementTimerId, gTmrLowPowerIntervalMillisTimer_c,
                       TmrSeconds(1), TimerMeasurementCallback, NULL);

            /* The Server shall indicate the new Measurement Interval value the next time there is a connection*/
            if (mUserData.measIntervalIndPending)
            {
                Hts_SendMeasurementIntervalIndication(value_measure_int);
                mUserData.measIntervalIndPending = FALSE;
            }
			
            /* Start battery measurements */
            TMR_StartLowPowerTimer(mBatteryMeasurementTimerId, gTmrLowPowerIntervalMillisTimer_c,
                       TmrSeconds(mBatteryLevelReportInterval_c), BatteryMeasurementTimerCallback, NULL);
            // Start led update timer
            TMR_StartLowPowerTimer(mLedUpdateTimerId, gTmrLowPowerIntervalMillisTimer_c,
                       TmrSeconds(mLedUpdateInterval), LedUpdateTimerCallback, NULL);
        }
        break;
        
        case gConnEvtDisconnected_c:
        {
            /* Unsubscribe client */
            Bas_Unsubscribe();
            Hts_Unsubscribe();
            Rfs_Unsubscribe();
            mPeerDeviceId = gInvalidDeviceId_c;

            if (pConnectionEvent->eventData.disconnectedEvent.reason == gHciConnectionTimeout_c)
            {
                /* Link loss detected*/
                BleApp_Start();
            }
            else
            {
              /* Connection was terminated by peer or application */
                BleApp_Start();
            }
        }
        break;

#if gBondingSupported_d
        case gConnEvtKeysReceived_c:
        {
            /* Copy peer device address information when IRK is used */
            if (pConnectionEvent->eventData.keysReceivedEvent.pKeys->aIrk != NULL)
            {
                mPeerDeviceAddressType = pConnectionEvent->eventData.keysReceivedEvent.pKeys->addressType;
                FLib_MemCpy(maPeerDeviceAddress, pConnectionEvent->eventData.keysReceivedEvent.pKeys->aAddress, sizeof(bleDeviceAddress_t));
            }
        }
        break;

        case gConnEvtPairingComplete_c:
        {
            if (pConnectionEvent->eventData.pairingCompleteEvent.pairingSuccessful &&
                pConnectionEvent->eventData.pairingCompleteEvent.pairingCompleteData.withBonding)
            {
                /* If a bond is created, write device address in controller’s White List */
                Gap_AddDeviceToWhiteList(mPeerDeviceAddressType, maPeerDeviceAddress);
            }
        }
        break;

        case gConnEvtPairingRequest_c:
        {
            gPairingParameters.centralKeys = pConnectionEvent->eventData.pairingEvent.centralKeys;
            Gap_AcceptPairingRequest(peerDeviceId, &gPairingParameters);
        }    
        break;
        
        case gConnEvtKeyExchangeRequest_c:
        {
            gapSmpKeys_t sentSmpKeys = gSmpKeys;
            
            if (!(pConnectionEvent->eventData.keyExchangeRequestEvent.requestedKeys & gLtk_c))
            {
                sentSmpKeys.aLtk = NULL;
                /* When the LTK is NULL EDIV and Rand are not sent and will be ignored. */
            }
            
            if (!(pConnectionEvent->eventData.keyExchangeRequestEvent.requestedKeys & gIrk_c))
            {
                sentSmpKeys.aIrk = NULL;
                /* When the IRK is NULL the Address and Address Type are not sent and will be ignored. */
            }
            
            if (!(pConnectionEvent->eventData.keyExchangeRequestEvent.requestedKeys & gCsrk_c))
            {
                sentSmpKeys.aCsrk = NULL;
            }
            
            Gap_SendSmpKeys(peerDeviceId, &sentSmpKeys);
            break;
        }
            
        case gConnEvtLongTermKeyRequest_c:
        {
            if (pConnectionEvent->eventData.longTermKeyRequestEvent.ediv == gSmpKeys.ediv &&
                pConnectionEvent->eventData.longTermKeyRequestEvent.randSize == gSmpKeys.cRandSize)
            {
                /* EDIV and RAND both matched */
                Gap_ProvideLongTermKey(peerDeviceId, gSmpKeys.aLtk, gSmpKeys.cLtkSize);
            }
            else
            /* EDIV or RAND size did not match */
            {
                Gap_DenyLongTermKey(peerDeviceId);
            }
        }
        break;
#endif
        
    default:
        break;
    }
}

/*! *********************************************************************************
* \brief        Handles GATT server callback from host stack.
*
* \param[in]    deviceId        Peer device ID.
* \param[in]    pServerEvent    Pointer to gattServerEvent_t.
********************************************************************************** */
static void BleApp_GattServerCallback (deviceId_t deviceId, gattServerEvent_t* pServerEvent)
{
    switch (pServerEvent->eventType)
    {
        case gEvtAttributeWritten_c:
        {
            if( pServerEvent->eventData.attributeWrittenEvent.handle == value_measure_int)
            {
                attErrorCode_t result;
                uint16_t    newValue = Utils_ExtractTwoByteValue(pServerEvent->eventData.attributeWrittenEvent.aValue);
                
                result = Hts_MeasurementIntervalWriting(&htsServiceConfig, value_measure_int, newValue);
                
                if (result == gAttErrCodeNoError_c)
                {
                    TMR_StopTimer(mMeasurementTimerId);
                
                    if (newValue)
                    {
                        /* Restart timer. Divide by 6 to compensate for interm temp */
                        TMR_StartLowPowerTimer(mMeasurementTimerId, gTmrLowPowerIntervalMillisTimer_c,
                                            newValue * 1000 / 6, TimerMeasurementCallback, NULL);
                    }
                }
                
                /* Send response OTA */
                GattServer_SendAttributeWrittenStatus(deviceId,
                                                      value_measure_int,
                                                      result);
            }
        }
        break;
        
        case gEvtCharacteristicCccdWritten_c:
        {

        }                   
        break;


    default:
        break;
    }
}


/*! *********************************************************************************
* \brief        Handles advertising timer callback.
*
* \param[in]    pParam        Calback parameters.
********************************************************************************** */
static void AdvertisingTimerCallback(void * pParam)
{
    /* Stop and restart advertising with new parameters */
    Gap_StopAdvertising();
    switch (mAdvState.advType)
    {
#if gBondingSupported_d
        case fastWhiteListAdvState_c:
        {
            mAdvState.advType = fastAdvState_c;
        }
        break;
#endif
        case fastAdvState_c:
        {
            mAdvState.advType = slowAdvState_c;
        }
        break;

        default:
        break;
    }
    BleApp_Advertise();
}

/*! *********************************************************************************
* \brief        Handles measurement timer callback.
*
* \param[in]    pParam        Calback parameters.
********************************************************************************** */
static void TimerMeasurementCallback(void * pParam)
{
    uint32_t random = 0;
    htsMeasurement_t tempMeas;
    ctsDateTime_t time = {2014, 12, 15, 0, 0, 0};

    RNG_GetRandomNo(&random);
    
    tempMeas.tempTypePresent = TRUE;
    tempMeas.tempType = gHts_Armpit_c;
      
    tempMeas.timeStampPresent = TRUE;
    
    tempMeas.unit = gHts_UnitInCelsius_c;
    FLib_MemCpy(&tempMeas.timeStamp, &time, sizeof(ctsDateTime_t));
    tempMeas.temperature = 35 + (random & 0x07);
        
    if (mIntermediateTempCounter < 4)
    {    
        Hts_RecordIntermediateTemperature(service_health_therm, &tempMeas);
    }
    else
    {
        Hts_RecordTemperatureMeasurement(service_health_therm, &tempMeas);
    }
    
    mIntermediateTempCounter += 1;
    mIntermediateTempCounter = mIntermediateTempCounter % 5;
    
}

/*! *********************************************************************************
* \brief        Handles battery measurement timer callback.
*
* \param[in]    pParam        Calback parameters.
********************************************************************************** */
static void BatteryMeasurementTimerCallback(void * pParam)
{
    basServiceConfig.batteryLevel = BOARD_GetBatteryLevel();
    Bas_RecordBatteryMeasurement(basServiceConfig.serviceHandle, basServiceConfig.batteryLevel);
}

/*! *********************************************************************************
* @}
********************************************************************************** */
static void LedUpdateTimerCallback(void * pParam)
{
  Rfs_LedDataWriting(service_ready_for_sky, rfsServiceConfig.ledData);
  UpdateLedState(rfsServiceConfig.ledData->ledState);
}