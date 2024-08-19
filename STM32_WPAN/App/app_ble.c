/**
 ******************************************************************************
 * @file    app_ble.c
 * @author  MCD Application Team
 * @brief   BLE Application
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/
#include "app_common.h"

#include "dbg_trace.h"

#include "ble.h"
#include "tl.h"
#include "app_ble.h"

#include "hids_app.h"
#include "hrs_app.h"
#include "DispTools.h"

#include "stm32_seq.h"
#include "shci.h"
#include "stm32_lpm.h"
#include "otp.h"

#include "dis_app.h"

#include "nfc04a1_nfctag.h"
#include "lib_NDEF_Bluetooth.h"
#include "lib_wrapper.h"
#include "nfc04a1.h"

/* OOB_SECURE is set to enable Secure Connection using NFC */
#define OOB_SECURE 1
/* Set USE_RANDOM_ADDR to use a random BT address */
/* #define USE_RANDOM_ADDR 1 */

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
  HID_IDLE,
  HID_FAST_ADV,
  HID_LP_ADV,
  HID_LP_CONNECTING,
  HID_CONNECTED_SERVER,
  HID_CONNECTED_CLIENT
} HID_ConnStatus_t;


/**
 * security parameters structure
 */
typedef struct _tSecurityParams
{
  /**
   * IO capability of the device
   */
  uint8_t ioCapability;

  /**
   * Authentication requirement of the device
   * Man In the Middle protection required?
   */
  uint8_t mitm_mode;

  /**
   * bonding mode of the device
   */
  uint8_t bonding_mode;

  /**
   * Flag to tell whether OOB data has
   * to be used during the pairing process
   */
  uint8_t OOB_Data_Present;

  /**
   * OOB data to be used in the pairing process if
   * OOB_Data_Present is set to TRUE
   */
  uint8_t OOB_Data[16];

  /**
   * this variable indicates whether to use a fixed pin
   * during the pairing process or a passkey has to be
   * requested to the application during the pairing process
   * 0 implies use fixed pin and 1 implies request for passkey
   */
  uint8_t Use_Fixed_Pin;

  /**
   * minimum encryption key size requirement
   */
  uint8_t encryptionKeySizeMin;

  /**
   * maximum encryption key size requirement
   */
  uint8_t encryptionKeySizeMax;

  /**
   * fixed pin to be used in the pairing process if
   * Use_Fixed_Pin is set to 1
   */
  uint32_t Fixed_Pin;

  /**
   * this flag indicates whether the host has to initiate
   * the security, wait for pairing or does not have any security
   * requirements.\n
   * 0x00 : no security required
   * 0x01 : host should initiate security by sending the slave security
   *        request command
   * 0x02 : host need not send the clave security request but it
   * has to wait for paiirng to complete before doing any other
   * processing
   */
  uint8_t initiateSecurity;
}tSecurityParams;

/**
 * global context
 * contains the variables common to all
 * services
 */
typedef struct _tBLEProfileGlobalContext
{

  /**
   * security requirements of the host
   */
  tSecurityParams bleSecurityParam;

  /**
   * gap service handle
   */
  uint16_t gapServiceHandle;

  /**
   * device name characteristic handle
   */
  uint16_t devNameCharHandle;

  /**
   * appearance characteristic handle
   */
  uint16_t appearanceCharHandle;

  /**
   * connection handle of the current active connection
   * When not in connection, the handle is set to 0xFFFF
   */
  uint16_t connectionHandle[CFG_MAX_CONNECTION];

  /**
   * length of the UUID list to be used while advertising
   */
  uint8_t advtServUUIDlen;

  /**
   * the UUID list to be used while advertising
   */
  uint8_t advtServUUID[100];

}BleGlobalContext_t;

typedef struct
{
  BleGlobalContext_t BleApplicationContext_legacy;
  HID_ConnStatus_t Device_Connection_Status[CFG_MAX_CONNECTION];
  uint8_t Connection_mgr_timer_Id;
} BleApplicationContext_t;

/* Private defines -----------------------------------------------------------*/
#define APPBLE_GAP_DEVICE_NAME_LENGTH 7
#define FAST_ADV_TIMEOUT            (30*1000*1000/CFG_TS_TICK_VAL) /**< 30s */

#define BD_ADDR_SIZE_LOCAL    6

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_CmdPacket_t BleCmdBuffer;

static const uint8_t M_bd_addr[BD_ADDR_SIZE_LOCAL] =
    {
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000000000FF)),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00000000FF00) >> 8),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x000000FF0000) >> 16),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000FF000000) >> 24),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00FF00000000) >> 32),
        (uint8_t)((CFG_ADV_BD_ADDRESS & 0xFF0000000000) >> 40)
    };

static uint8_t bd_addr_udn[BD_ADDR_SIZE_LOCAL];

/**
*   Identity root key used to derive LTK and CSRK
*/
static const uint8_t BLE_CFG_IR_VALUE[16] = CFG_BLE_IRK;

/**
* Encryption root key used to derive LTK and CSRK
*/
static const uint8_t BLE_CFG_ER_VALUE[16] = CFG_BLE_ERK;



PLACE_IN_SECTION("BLE_APP_CONTEXT") static BleApplicationContext_t BleApplicationContext;
PLACE_IN_SECTION("BLE_APP_CONTEXT") static uint16_t AdvIntervalMin, AdvIntervalMax;



uint8_t Test_Status =0x00;
static const char *name = "ST25OOB";
static const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'S','T','2','5','O','O','B'};

/* Global variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void BLE_UserEvtRx( void * pPayload );
static void BLE_StatusNot( HCI_TL_CmdStatus_t status );
static void Ble_Tl_Init( void );
static void Ble_Hci_Gap_Gatt_Init(void);
static const uint8_t* BleGetBdAddress( void );
static void Add_Advertisment_Service_UUID(uint16_t servUUID);
static void Adv_Request(HID_ConnStatus_t New_Status);
static void ConnMgr( void );
static void AdvUpdate ( void );
static void AdvFastUpdate ( void );
static void runOOB(void);


/** Callback called when the NFC tag has been read */
void BSP_GPO_Callback(void)
{
    /* ask to start the fast advertising */
    UTIL_SEQ_SetTask( 1<<CFG_TASK_ADV_UPDATE_ID,CFG_SCH_PRIO_0);
    /* Stop the LP advertising */
    HW_TS_Stop(BleApplicationContext.Connection_mgr_timer_Id);

}

/* Functions Definition ------------------------------------------------------*/
void APP_BLE_Init( void )
{
  uint8_t index;
  SHCI_C2_Ble_Init_Cmd_Packet_t ble_init_cmd_packet =
  {
    {{0,0,0}},                              /**< Header unused */
    {0,                                  /** pBleBufferAddress not used */
    0,                                  /** BleBufferSize not used */
    CFG_BLE_NUM_GATT_ATTRIBUTES,
    CFG_BLE_NUM_GATT_SERVICES,
    CFG_BLE_ATT_VALUE_ARRAY_SIZE,
    CFG_BLE_NUM_LINK,
    CFG_BLE_DATA_LENGTH_EXTENSION,
    CFG_BLE_PREPARE_WRITE_LIST_SIZE,
    CFG_BLE_MBLOCK_COUNT,
    CFG_BLE_MAX_ATT_MTU,
    CFG_BLE_SLAVE_SCA,
    CFG_BLE_MASTER_SCA,
    CFG_BLE_LSE_SOURCE,
    CFG_BLE_MAX_CONN_EVENT_LENGTH,
    CFG_BLE_HSE_STARTUP_TIME,
    CFG_BLE_VITERBI_MODE,
    CFG_BLE_LL_ONLY,
    0},
  };

  /**
   * Initialize Ble Transport Layer
   */
  Ble_Tl_Init( );

  /**
   * Do not allow standby in the application
   */
  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_DISABLE);

  /**
   * Register the hci transport layer to handle BLE User Asynchronous Events
   */
  UTIL_SEQ_RegTask( 1<<CFG_TASK_HCI_ASYNCH_EVT_ID, UTIL_SEQ_RFU, hci_user_evt_proc);
  /**
   * Starts the BLE Stack on CPU2
   */
  SHCI_C2_BLE_Init( &ble_init_cmd_packet );

  /**
   * Initialization of HCI & GATT & GAP layer
   */
  Ble_Hci_Gap_Gatt_Init();

  /**
   * Initialization of the BLE Services
   */
  SVCCTL_Init();

  /**
   * From here, all initialization are BLE application specific
   */
  UTIL_SEQ_RegTask( 1<< CFG_TASK_CONN_MGR_ID, UTIL_SEQ_RFU, AdvUpdate );
  UTIL_SEQ_RegTask( 1<< CFG_TASK_ADV_UPDATE_ID, UTIL_SEQ_RFU, AdvFastUpdate );

  /**
   * Initialization of the BLE App Context
   */
  for(index = 0; index < CFG_MAX_CONNECTION; index++)
  {
    BleApplicationContext.Device_Connection_Status[index] = HID_IDLE;
    BleApplicationContext.BleApplicationContext_legacy.connectionHandle[index] = 0xFFFF;
  }

  ConsoleMenu_Init( );

  /**
   * Initialize Human Interface Device Service
   */
  HIDSAPP_Init();

#ifndef DISABLE_HR
  HRSAPP_Init();
#endif

  DISAPP_Init();

  /**
   * Create timer to handle the connection state machine
   */
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.Connection_mgr_timer_Id), hw_ts_SingleShot, ConnMgr);

  /**
   * Make device discoverable
   */
  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[0] = AD_TYPE_16_BIT_SERV_UUID;
  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen = 1;

#ifndef DISABLE_HR
  Add_Advertisment_Service_UUID(HEART_RATE_SERVICE_UUID);
#endif
  Add_Advertisment_Service_UUID(HUMAN_INTERFACE_DEVICE_SERVICE_UUID);
  Add_Advertisment_Service_UUID(DEVICE_INFORMATION_SERVICE_UUID);

  /* Initialize intervals for reconnexion without intervals update */
  AdvIntervalMin = CFG_FAST_CONN_ADV_INTERVAL_MIN;
  AdvIntervalMax = CFG_FAST_CONN_ADV_INTERVAL_MAX;

  Adv_Request(HID_FAST_ADV);
  return;
}



SVCCTL_UserEvtFlowStatus_t SVCCTL_App_Notification(void *pckt)
{
  hci_event_pckt *event_pckt;
  evt_le_meta_event *meta_evt;
  uint8_t index;
  evt_blue_aci *blue_evt;

  event_pckt = (hci_event_pckt*)((hci_uart_pckt *)pckt)->data;

  switch(event_pckt->evt)
  {
    case HCI_ENCRYPTION_CHANGE_EVT_CODE:

    break;
    case EVT_DISCONN_COMPLETE:
    {
      hci_disconnection_complete_event_rp0 * disconnection_complete_event;

      disconnection_complete_event = (hci_disconnection_complete_event_rp0 *)event_pckt->data;

      APP_DBG_MSG("EVT_DISCONN_COMPLETE for connection handle 0x%x\n",
                  disconnection_complete_event->Connection_Handle);
      /* Find index of the handle deconnected */
      index = 0;
      while((index < CFG_MAX_CONNECTION) &&
          (BleApplicationContext.BleApplicationContext_legacy.connectionHandle[index] != disconnection_complete_event->Connection_Handle))
      {
        index++;
      }

      if(index < CFG_MAX_CONNECTION)
      {
        APP_DBG_MSG("Index of the handle deconnected: %d\n", index);
        BleApplicationContext.Device_Connection_Status[index] = HID_IDLE;
        BleApplicationContext.BleApplicationContext_legacy.connectionHandle[index] =
            0xFFFF;
      }
      else
      {
        APP_DBG_MSG("No index found for the handle discconnected !\n");
      }

      /* restart advertising */
#ifndef DISABLE_HR
      HRS_App_Notification_evt_t stopHRNotif = {.HRS_Evt_Opcode = HRS_NOTIFICATION_DISABLED};
      HRS_Notification(&stopHRNotif);
#endif
      HIDS_App_Notification_evt_t stopHIDNotif = {.HIDS_Evt_Opcode = HIDS_REPORT_NOTIFICATION_DISABLED};
      HIDS_Notification(&stopHIDNotif);
#ifdef USE_RANDOM_ADDR
    runOOB();
#endif
    /* stop LP ADV */
    HW_TS_Stop(BleApplicationContext.Connection_mgr_timer_Id);

    }
    break; /* EVT_DISCONN_COMPLETE */

    case EVT_LE_META_EVENT:
    {
      meta_evt = (evt_le_meta_event*)event_pckt->data;

      switch(meta_evt->subevent)
      {
        case 0x08:
          /* the Key pair has changed, update OOB data */
          runOOB();
        break;
        case EVT_LE_CONN_COMPLETE:
        {
          hci_le_connection_complete_event_rp0 * connection_complete_event;

          /**
           * The connection is done, there is no need anymore to schedule the LP ADV
           */
          connection_complete_event = (hci_le_connection_complete_event_rp0 *)meta_evt->data;
          HW_TS_Stop(BleApplicationContext.Connection_mgr_timer_Id);


          APP_DBG_MSG("EVT_LE_CONN_COMPLETE for connection handle 0x%x\n",
                      connection_complete_event->Connection_Handle);

          /* Find index of a connection not in HID_IDLE, HID_CONNECTED_SERVER or HID_CONNECTED_CLIENT state */
          index = 0;
          while((index < CFG_MAX_CONNECTION) &&
              ((BleApplicationContext.Device_Connection_Status[index] == HID_IDLE) ||
                  (BleApplicationContext.Device_Connection_Status[index] == HID_CONNECTED_SERVER) ||
                  (BleApplicationContext.Device_Connection_Status[index] == HID_CONNECTED_CLIENT)))
          {
            index++;
          }

          if(index < CFG_MAX_CONNECTION)
          {
            APP_DBG_MSG("First index in state %d: %d\n",
                        BleApplicationContext.Device_Connection_Status[index],
                        index);
            if(BleApplicationContext.Device_Connection_Status[index] == HID_LP_CONNECTING)
              /* Connection as client */
              BleApplicationContext.Device_Connection_Status[index] = HID_CONNECTED_CLIENT;
            else
              /* Connection as server */
              BleApplicationContext.Device_Connection_Status[index] = HID_CONNECTED_SERVER;
            BleApplicationContext.BleApplicationContext_legacy.connectionHandle[index] = connection_complete_event->Connection_Handle;

          }
          else
          {
            APP_DBG_MSG("No stored connection in state different than HID_IDLE, HID_CONNECTED_CLIENT and HID_CONNECTED_SERVER!\n");
          }
        }
        break; /* HCI_EVT_LE_CONN_COMPLETE */

        default:
          break;
      }
    }
    break; /* HCI_EVT_LE_META_EVENT */
        case EVT_VENDOR:
          blue_evt = (evt_blue_aci*) event_pckt->data;
          /* USER CODE BEGIN EVT_VENDOR */
      break;

    default:
      break;
  }
  return (SVCCTL_UserEvtFlowEnable);
}


/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
static void Ble_Tl_Init( void )
{
  HCI_TL_HciInitConf_t Hci_Tl_Init_Conf;

  Hci_Tl_Init_Conf.p_cmdbuffer = (uint8_t*)&BleCmdBuffer;
  Hci_Tl_Init_Conf.StatusNotCallBack = BLE_StatusNot;
  hci_init(BLE_UserEvtRx, (void*) &Hci_Tl_Init_Conf);

  return;
}

/** This function enables the OOB and update the NFC tag content accordingly*/
static void runOOB(void)
{
  uint8_t at = 0;
  uint8_t add[6] = {0,0,0,0,0,0};
  uint8_t len = 0;
  uint8_t rand[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  uint8_t hash[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  ST25DV_PASSWD default_password = {.MsbPasswd=0, .LsbPasswd=0};
  ST25DV_RF_PROT_ZONE rf_write_protect = {.PasswdCtrl = ST25DV_PROT_PASSWD1 , .RWprotection =ST25DV_WRITE_PROT};

  NFC04A1_NFCTAG_SetRFDisable_Dyn(0);
  NFC04A1_NFCTAG_PresentI2CPassword(0,default_password);
  NFC04A1_NFCTAG_WriteRFZxSS(0, ST25DV_PROT_ZONE1,  rf_write_protect );

  /* this dummy call is required to enable OOB */
#ifdef OOB_SECURE
   int status = aci_gap_set_oob_data(0,0,add, 0, 0, rand);
#endif

   Ndef_Bluetooth_OOB_t NdefBle = {

#ifdef OOB_SECURE
                                    .LeOptionalMask = (NDEF_BLUETOOTH_OPTION(BLUETOOTH_EIR_LE_SECURE_CONNECTIONS_CONFIRMATION_VALUE) |
                                                       NDEF_BLUETOOTH_OPTION(BLUETOOTH_EIR_LE_SECURE_CONNECTIONS_RANDOM_VALUE)),
#endif
                                    .OptionalMask = (NDEF_BLUETOOTH_OPTION(BLUETOOTH_EIR_COMPLETE_LOCAL_NAME) |
                                                     NDEF_BLUETOOTH_OPTION(BLUETOOTH_EIR_SECURITY_MANAGER_TK_VALUE)),
                                    .Type = NDEF_BLUETOOTH_BLE,
                                    .Role = NDEF_BLE_ROLE_PERIPH_ONLY,
#ifndef USE_RANDOM_ADDR
                                    .DeviceAddressType = NDEF_BLE_PUBLIC_ADDRESS_TYPE
#else
                                    .DeviceAddressType = NDEF_BLE_RANDOM_ADDRESS_TYPE
#endif
                                  };
    strcpy(NdefBle.LocalName,name);


#ifndef OOB_SECURE
 memcpy(NdefBle.DeviceAddress,bd_addr_udn,sizeof(NdefBle.DeviceAddress));
  for(int i = 5; i >= 0; i --)
    NdefBle.DeviceAddress[i] = bd_addr_udn[5-i];
#else
   status = aci_gap_get_oob_data(1, &at,add,&len, rand);
    status = aci_gap_get_oob_data(2, &at,add,&len, hash);

  for(int i = 5; i >= 0; i --)
    NdefBle.DeviceAddress[i] = add[5-i];
  for(int i = 15; i >= 0; i --)
  {
    NdefBle.SimplePairingRandomizer[i] = rand[15-i];
    NdefBle.SimplePairingHash[i] = hash[15-i];
  }
#endif

   NDEF_ClearNDEF();
   NDEF_AppendBluetoothOOB(&NdefBle, NULL);

  NFC04A1_NFCTAG_ResetRFDisable_Dyn(0);
  BSP_LED_On(LED_BLUE);

}


/**
 * Define PHY
 */
#define ALL_PHYS_PREFERENCE                             0x00
#define RX_2M_PREFERRED                                 0x02
#define TX_2M_PREFERRED                                 0x02
#define TX_1M                                           0x01
#define TX_2M                                           0x02
#define RX_1M                                           0x01
#define RX_2M                                           0x02


static void Ble_Hci_Gap_Gatt_Init(void){

  uint8_t role;
  uint8_t index;
  uint16_t gap_service_handle, gap_dev_name_char_handle, gap_appearance_char_handle;
  const uint8_t *bd_addr;
  uint32_t srd_bd_addr[2];
  uint16_t appearance[1] = { CFG_GAP_APPEARANCE }; /* Generic Heart Rate Sensor */
  int status = 0;

  /**
   * Initialize HCI layer
   */
  /*HCI Reset to synchronise BLE Stack*/
  hci_reset();

  /**
   * Write the BD Address
   */

  bd_addr = BleGetBdAddress();
  aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                            CONFIG_DATA_PUBADDR_LEN,
                            (uint8_t*) bd_addr);

  /**
   * Static random Address
   * The two upper bits shall be set to 1
   * The lowest 32bits is read from the UDN to differentiate between devices
   * The RNG may be used to provide a random number on each power on
   */
  srd_bd_addr[1] =  0x0000ED6E;
  srd_bd_addr[0] =  LL_FLASH_GetUDN( );
  aci_hal_write_config_data( CONFIG_DATA_RANDOM_ADDRESS_OFFSET, CONFIG_DATA_RANDOM_ADDRESS_LEN, (uint8_t*)srd_bd_addr );

  /**
   * Write Identity root key used to derive LTK and CSRK
   */
    aci_hal_write_config_data( CONFIG_DATA_IR_OFFSET, CONFIG_DATA_IR_LEN, (uint8_t*)BLE_CFG_IR_VALUE );

   /**
   * Write Encryption root key used to derive LTK and CSRK
   */
    aci_hal_write_config_data( CONFIG_DATA_ER_OFFSET, CONFIG_DATA_ER_LEN, (uint8_t*)BLE_CFG_ER_VALUE );

  /**
   * Set TX Power to 0dBm.
   */
  aci_hal_set_tx_power_level(1, CFG_TX_POWER);

  /**
   * Initialize GATT interface
   */
  aci_gatt_init();

  /**
   * Initialize GAP interface
   */
  role = 0;

#if (BLE_CFG_PERIPHERAL == 1)
  role |= GAP_PERIPHERAL_ROLE;
#endif

#if (BLE_CFG_CENTRAL == 1)
  role |= GAP_CENTRAL_ROLE;
#endif

  if (role > 0)
  {

    aci_gap_init(role, 0,
                 APPBLE_GAP_DEVICE_NAME_LENGTH,
                 &gap_service_handle, &gap_dev_name_char_handle, &gap_appearance_char_handle);

    if (aci_gatt_update_char_value(gap_service_handle, gap_dev_name_char_handle, 0, strlen(name), (uint8_t *) name))
    {
      BLE_DBG_SVCCTL_MSG("Device Name aci_gatt_update_char_value failed.\n");
    }
  }

  if(aci_gatt_update_char_value(gap_service_handle,
                                gap_appearance_char_handle,
                                0,
                                2,
                                (uint8_t *)&appearance))
  {
    BLE_DBG_SVCCTL_MSG("Appearance aci_gatt_update_char_value failed.\n");
  }

  uint8_t preferred[] = {0x20,0x00,0x30,0x00,0x00,0x00,0xF4,0x01};
  uint16_t preferred_conn_handle = 0x000A;
    status = aci_gatt_update_char_value(gap_service_handle,
                                      preferred_conn_handle,
                                      0,
                                      8,//sizeof(preferred),
                                      preferred);


  /**
   * Initialize IO capability
   */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability = CFG_IO_CAPABILITY;
  aci_gap_set_io_capability(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability);

  /**
   * Initialize authentication
   */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode = CFG_MITM_PROTECTION_NOT_REQUIRED;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.OOB_Data_Present = 0;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin = 8;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax = 16;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin = 1;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin = 111111;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode = 1;
  for (index = 0; index < 16; index++)
  {
    BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.OOB_Data[index] = (uint8_t) index;
  }

// for native pairing -> Fixed_Pin = 111111
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin = 0;


  status = 0xFF;

  status = aci_gap_set_authentication_requirement(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode,
#ifdef OOB_SECURE
                                         0x2,
#else
                                         0,
#endif
                                         0,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin,
#ifndef USE_RANDOM_ADDR
                                         0
#else
                                         1
#endif
  );


#ifdef OOB_SECURE
  uint8_t ALL_EVENTS[8]={0x9F,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
  hci_le_set_event_mask(ALL_EVENTS);
  status = hci_le_read_local_p256_public_key();
#else
  runOOB();
#endif
  /**
   * Initialize whitelist
   */
   if (BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode)
   {
     aci_gap_configure_whitelist();
   }

}

static void AdvUpdate ( void )
{
  Adv_Request(HID_LP_ADV);

  return;
}

static void AdvFastUpdate ( void )
{
  Adv_Request(HID_FAST_ADV);

  return;
}

static void ConnMgr( void )
{
  /**
   * The code shall be executed in the background as an aci command may be sent
   * The background is the only place where the application can make sure a new aci command
   * is not sent if there is a pending one
   */
  UTIL_SEQ_SetTask( 1<<CFG_TASK_CONN_MGR_ID,CFG_SCH_PRIO_0);

  return;
}

static void Add_Advertisment_Service_UUID(uint16_t servUUID)
{
  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen] = (uint8_t)(servUUID & 0xFF);
  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen++;
  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen] = (uint8_t)(servUUID >> 8) & 0xFF;
  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen++;

  return;
}

static void Adv_Request(HID_ConnStatus_t New_Status)
{
  uint8_t index;
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint16_t Min_Inter, Max_Inter;

  index = 0;
  if(New_Status == HID_FAST_ADV)
  {
    Min_Inter = AdvIntervalMin;
    Max_Inter = AdvIntervalMax;

    /* Find first index of connection in ADVERTISE or IDLE mode */
    while((index < CFG_MAX_CONNECTION) &&
        (BleApplicationContext.Device_Connection_Status[index] != HID_IDLE))
    {
      index++;
    }
  }
  else
  {
    Min_Inter = CFG_LP_CONN_ADV_INTERVAL_MIN;
    Max_Inter = CFG_LP_CONN_ADV_INTERVAL_MAX;

    /* Find first index of connection in Fast ADVERTISE mode */
    while((index < CFG_MAX_CONNECTION) &&
        (BleApplicationContext.Device_Connection_Status[index] != HID_FAST_ADV))
    {
      index++;
    }
  }

  if(index < CFG_MAX_CONNECTION)
  {
    /**
     * Stop the timer, it will be restarted for a new shot
     * It does not hurt if the timer was not running
     */
    HW_TS_Stop(BleApplicationContext.Connection_mgr_timer_Id);

    APP_DBG_MSG("First index in %d state: %d\n",
                BleApplicationContext.Device_Connection_Status[index],
                index);

    if((New_Status == HID_LP_ADV) &&
        ((BleApplicationContext.Device_Connection_Status[index] == HID_FAST_ADV) ||
            (BleApplicationContext.Device_Connection_Status[index] == HID_LP_ADV)))
    {
      /* Connection in ADVERTISE mode have to stop the current advertising */
      ret = aci_gap_set_non_discoverable();
      if( ret == BLE_STATUS_SUCCESS )
      {
        APP_DBG_MSG("Successfully Stopped Advertising at index: %d\n", index);
      }
      else
      {
        APP_DBG_MSG("Stop Advertising Failed at index: %d, result: %d \n", index, ret);
      }
    }

    BleApplicationContext.Device_Connection_Status[index] = New_Status;
    /* Start Fast or Low Power Advertising */
    ret = aci_gap_set_discoverable(ADV_IND,
                                   Min_Inter,
                                   Max_Inter,
#ifndef USE_RANDOM_ADDR
                                   PUBLIC_ADDR,
#else
                                   RANDOM_ADDR,
#endif
                                   NO_WHITE_LIST_USE, /* use white list */
                                   sizeof(local_name),
                                   (uint8_t*)&local_name,
                                   BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen,
                                   BleApplicationContext.BleApplicationContext_legacy.advtServUUID,
                                   80,
                                   100);
    if( ret == BLE_STATUS_SUCCESS )
    {
      if(New_Status == HID_FAST_ADV)
      {
        APP_DBG_MSG("Successfully Start Fast Advertising at index: %d\n", index);
      }
      else
      {
        APP_DBG_MSG("Successfully Start Low Power Advertising at index: %d\n", index);
      }
    }
    else
    {
      if(New_Status == HID_FAST_ADV)
      {
        APP_DBG_MSG("Start Fast Advertising Failed at index: %d, result: %d \n", index, ret);
      }
      else
      {
        APP_DBG_MSG("Start Low Power Advertising Failed at index: %d, result: %d \n", index, ret);
      }
    }
    if(New_Status == HID_FAST_ADV)
    {
      /* Fast advertising during FAST_ADV_TIMEOUT */
      HW_TS_Start(BleApplicationContext.Connection_mgr_timer_Id, FAST_ADV_TIMEOUT);
    }
  }
  else
    APP_DBG_MSG("No index in HID_IDLE state !\n");

  return;
}

void APP_BLE_Key_Button1_Action(void)
{
  UTIL_SEQ_SetTask( 1<<CFG_TASK_HID_UPDATE_REQ_ID, CFG_SCH_PRIO_0);
}

void APP_BLE_Key_Button2_Action(void)
{
}

void APP_BLE_Key_Button3_Action(void)
{
}

const uint8_t* BleGetBdAddress( void )
{
  uint8_t *otp_addr;
  const uint8_t *bd_addr;
  uint32_t udn;
  uint32_t company_id;
  uint32_t device_id;

  udn = LL_FLASH_GetUDN();

  if(udn != 0xFFFFFFFF)
  {
    company_id = LL_FLASH_GetSTCompanyID();
    device_id = LL_FLASH_GetDeviceID();

    bd_addr_udn[0] = (uint8_t)(udn & 0x000000FF);
    bd_addr_udn[1] = (uint8_t)( (udn & 0x0000FF00) >> 8 );
    bd_addr_udn[2] = (uint8_t)( (udn & 0x00FF0000) >> 16 );
    bd_addr_udn[3] = (uint8_t)device_id;
    bd_addr_udn[4] = (uint8_t)(company_id & 0x000000FF);;
    bd_addr_udn[5] = (uint8_t)( (company_id & 0x0000FF00) >> 8 );

    bd_addr = (const uint8_t *)bd_addr_udn;
  }
  else
  {
    otp_addr = OTP_Read(0);
    if(otp_addr)
    {
      bd_addr = ((OTP_ID0_t*)otp_addr)->bd_address;
    }
    else
    {
      bd_addr = M_bd_addr;
    }

  }

  return bd_addr;
}

/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/
void hci_notify_asynch_evt(void* pdata)
{
  UTIL_SEQ_SetTask( 1<<CFG_TASK_HCI_ASYNCH_EVT_ID, CFG_SCH_PRIO_0);
  return;
}

void hci_cmd_resp_release(uint32_t flag)
{
  UTIL_SEQ_SetEvt( 1<< CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID );
  return;
}

void hci_cmd_resp_wait(uint32_t timeout)
{
  UTIL_SEQ_WaitEvt( 1<< CFG_IDLEEVT_HCI_CMD_EVT_RSP_ID );
  return;
}

static void BLE_UserEvtRx( void * pPayload )
{
  SVCCTL_UserEvtFlowStatus_t svctl_return_status;
  tHCI_UserEvtRxParam *pParam;

  pParam = (tHCI_UserEvtRxParam *)pPayload;

  svctl_return_status = SVCCTL_UserEvtRx((void *)&(pParam->pckt->evtserial));
  if (svctl_return_status != SVCCTL_UserEvtFlowDisable)
  {
    pParam->status = HCI_TL_UserEventFlow_Enable;
  }
  else
  {
    pParam->status = HCI_TL_UserEventFlow_Disable;
  }
}

static void BLE_StatusNot( HCI_TL_CmdStatus_t status )
{
  uint32_t task_id_list;
  switch(status)
  {
    case HCI_TL_CmdBusy:
      /**
       * All tasks that may send an aci/hci commands shall be listed here
       * This is to prevent a new command is sent while one is already pending
       */
      task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
      UTIL_SEQ_PauseTask( task_id_list );

      break;

    case HCI_TL_CmdAvailable:
      /**
       * All tasks that may send an aci/hci commands shall be listed here
       * This is to prevent a new command is sent while one is already pending
       */
      task_id_list = (1 << CFG_LAST_TASK_ID_WITH_HCICMD) - 1;
      UTIL_SEQ_ResumeTask( task_id_list );

      break;

    default:
      break;
  }
  return;
}

void SVCCTL_ResumeUserEventFlow( void )
{
  hci_resume_flow();
  return;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
