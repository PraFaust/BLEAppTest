#ifndef _RFS_H_
#define _RFS_H_

#include "EmbeddedTypes.h"
#include "FunctionLib.h"
#include "ble_general.h"
#include "gatt_db_app_interface.h"
#include "gatt_server_interface.h"
#include "gap_interface.h"

typedef struct _packed
{
  bool_t ledState;
  uint8_t reserved[2];
  uint8_t ledPos;
} LED_FORMAT;

// RFs Service - Configuration
typedef struct rfsConfig_tag
{
 uint16_t serviceHandle;
 uint8_t buttonState;
 LED_FORMAT* ledData;
} rfsConfig_t;


// RFS service start/stop
bleResult_t Rfs_Start (rfsConfig_t *pServiceConfig);
bleResult_t Rfs_Stop (rfsConfig_t *pServiceConfig);
// Subscribe/unsubscribe a GATT client
bleResult_t Rfs_Subscribe(deviceId_t clientDeviceId);
bleResult_t Rfs_Unsubscribe();

// RFs service record value
bleResult_t Rfs_RecordButtonValue (uint16_t serviceHandle, uint8_t buttonState);
bleResult_t Rfs_RecordLedValue (uint16_t serviceHandle, LED_FORMAT* ledData);


#endif /* _RFS_H_ */