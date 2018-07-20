#include "rfs.h"
#include <stdbool.h>

// LED_FORMAT ledFormat;

// Function prototype
static void Rfs_SendButtonStatusNotification(uint16_t handle);
static void Rfs_SendLedStatusNotification(uint16_t handle);

/* RFS Service - Subscribed Client*/
static deviceId_t mRfs_SubscribedClientId;

//Characteristic declarations
const bleUuid_t buttonCharacteristicUuid128 = {
  .uuid128 = {0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08, 0x68, 0xb5, 0xe8, 0x11, 0x53, 0x8b, 0x61, 0xbd, 0xf9, 0xce}
};

const bleUuid_t ledCharacteristicUuid128 = {
  .uuid128 = {0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08, 0x68, 0xb5, 0xe8, 0x11, 0x53, 0x8b, 0x62, 0xbd, 0xf9, 0xce}
};

bleResult_t Rfs_Start (rfsConfig_t *pServiceConfig)
{
  bleResult_t result;
  
  pServiceConfig->ledData->ledState = false;
  pServiceConfig->ledData->reserved[0] = 0x12;
  pServiceConfig->ledData->reserved[1] = 0x34;
  pServiceConfig->ledData->ledPos = 0x04;
  
  mRfs_SubscribedClientId = gInvalidDeviceId_c;
  
  result = Rfs_RecordButtonValue (pServiceConfig->serviceHandle,
                                  pServiceConfig->buttonState);
  if (result != gBleSuccess_c)
    return result;
  
  result = Rfs_RecordLedValue (pServiceConfig->serviceHandle,
                               pServiceConfig->ledData);
  return result;
}


bleResult_t Rfs_Stop (rfsConfig_t *pServiceConfig)
{
  mRfs_SubscribedClientId = gInvalidDeviceId_c;
  return gBleSuccess_c;
}

bleResult_t Rfs_Subscribe(deviceId_t clientDeviceId)
{
  mRfs_SubscribedClientId = clientDeviceId;
  return gBleSuccess_c;
}

bleResult_t Rfs_Unsubscribe()
{
  mRfs_SubscribedClientId = gInvalidDeviceId_c;
  return gBleSuccess_c;
}

bleResult_t Rfs_RecordButtonValue (uint16_t serviceHandle, uint8_t buttonState)
{
  uint16_t handle;
  bleResult_t result;
                           
  // Get handle of Button characteristic 
  result = GattDb_FindCharValueHandleInService(serviceHandle, gBleUuidType128_c, (bleUuid_t*)&buttonCharacteristicUuid128, &handle);
    if (result != gBleSuccess_c)
      return result;

  // Update characteristic value
  result = GattDb_WriteAttribute(handle, sizeof(uint8_t), (uint8_t*)&buttonState);
    if (result != gBleSuccess_c)
      return result;
                           
  Rfs_SendButtonStatusNotification(handle);
                           
  return gBleSuccess_c;
}

bleResult_t Rfs_RecordLedValue (uint16_t serviceHandle, LED_FORMAT* ledFormat)
{
  uint16_t handle;
  bleResult_t result;
                           
  // Get handle of Led characteristic 
  result = GattDb_FindCharValueHandleInService(serviceHandle, gBleUuidType128_c, (bleUuid_t*)&ledCharacteristicUuid128, &handle);
    if (result != gBleSuccess_c)
      return result;

  // Update characteristic value
  result = GattDb_WriteAttribute(handle, sizeof(ledFormat), (void*)ledFormat);
    if (result != gBleSuccess_c)
      return result;
                           
  Rfs_SendButtonStatusNotification(handle);
                           
  return gBleSuccess_c;
}

static void Rfs_SendButtonStatusNotification(uint16_t handle)
{
  uint16_t  handleCccd;
  bool_t isNotificationActive;
  
  // Get handle of CCCD
  if (GattDb_FindCccdHandleForCharValueHandle(handle, &handleCccd) != gBleSuccess_c)
    return;
  
  if (gBleSuccess_c == Gap_CheckNotificationStatus
      (mRfs_SubscribedClientId, handleCccd, &isNotificationActive) &&
        TRUE == isNotificationActive)
  {
    GattServer_SendNotification(mRfs_SubscribedClientId, handle);
  }                          
}

static void Rfs_SendLedStatusNotification(uint16_t handle)
{
  uint16_t  handleCccd;
  bool_t isNotificationActive;
  
  // Get handle of CCCD
  if (GattDb_FindCccdHandleForCharValueHandle(handle, &handleCccd) != gBleSuccess_c)
    return;
  
  if (gBleSuccess_c == Gap_CheckNotificationStatus
      (mRfs_SubscribedClientId, handleCccd, &isNotificationActive) &&
        TRUE == isNotificationActive)
  {
    GattServer_SendNotification(mRfs_SubscribedClientId, handle);
  }                              
}