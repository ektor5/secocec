//
// Seco Srl
//
// Stm32 microntroller define
//

#ifndef __STM32_MICROCONTROLLER_H__
#define __STM32_MICROCONTROLLER_H__

//
// Microntroller Address
//
#define MICRO_ADDRESS 0x40


//
// SMBUS Registers
//
#define  VERSION                               0x00
#define  ENABLE_REGISTER_1                     0x01
#define  ENABLE_REGISTER_2                     0x02
#define  STATUS_REGISTER_1                     0x03
#define  STATUS_REGISTER_2                     0x04
#define  INTERNAL_FAN_FREQUENCY                0x05
#define  INTERNAL_FAN_DC                       0x06
#define  INTERNAL_FAN_RPM                      0x07
#define  EXTERNAL_FAN_FREQUENCY                0x08
#define  EXTERNAL_FAN_DC                       0x09
#define  EXTERNAL_FAN_RPM                      0x0A
#define  LAST_STATE                            0x0B
#define  AFTER_G3                              0x0C
#define  RESERVED_0D                           0x0D
#define  RESERVED_0E                           0x0E
#define  RESERVED_0F                           0x0F
#define  WATCHDOG_CONFIGURATION                0x10
#define  WATCHDOG_DELAY                        0x11
#define  WATCHDOG_TIMEOUT                      0x12
#define  WATCHDOG_REFRESH                      0x13
#define  WATCHDOG_DELAY_COUNTER                0x14
#define  WATCHDOG_TIMEOUT_COUNTER              0x15
#define  MUTE_AMPLIFIER                        0x16
#define  RESERVED_17                           0x17
#define  RESERVED_18                           0x18
#define  RESERVED_19                           0x19
#define  JUMP_TO_BOOTLOADER                    0x1A
#define  REGION_SELECTION                      0x1B
#define  REGION_SIZE                           0x1C
#define  FLASH_UNLOCK                          0x1D
#define  FLASH_LOCK                            0x1E
#define  WORD_PROGRAM                          0x1F
#define  WORD_READ                             0x20
#define  ERASE_REGION                          0x21
#define  REGION_OFFSET                         0x22
#define  RESERVED_023                          0x23
#define  RESERVED_024                          0x24
#define  RESET_CONFIGURATION                   0x25
#define  CURRENT_BOOTLOADER                    0x26
#define  RESERVED_027                          0x27
#define  CEC_STATUS                            0x28
#define  CEC_Device_LA            0x29   //bit[7:0] logical Address
#define  CEC_READ_OPERATION_ID                 0x2A   //bit[7:0] operationID 
#define  CEC_READ_DATA_LENGTH                  0x2B
#define  CEC_READ_DATA_00                      0x2C
#define  CEC_READ_DATA_02                      0x2D
#define  CEC_READ_DATA_04                      0x2E
#define  CEC_READ_DATA_06                      0x2F
#define  CEC_READ_DATA_08                      0x30
#define  CEC_READ_BYTE0                        0x31
#define  CEC_WRITE_OPERATION_ID                0x32   //bit[7:0] operationID
#define  CEC_WRITE_DATA_LENGTH                 0x33
#define  CEC_WRITE_DATA_00                     0x34
#define  CEC_WRITE_DATA_02                     0x35
#define  CEC_WRITE_DATA_04                     0x36
#define  CEC_WRITE_DATA_06                     0x37
#define  CEC_WRITE_DATA_08                     0x38
#define  CEC_WRITE_BYTE0                       0x39

//
// DFU mode register
//
#define  DFU_BOOTLOADER_REVISION          0x00
#define  DFU_RESET_INTERNAL_COUNTER       0x01
#define  DFU_FLASH_UNLOCK                 0x02
#define  DFU_FLASH_LOCK                   0x03
#define  DFU_ERASE_USER_SPACE_BLOCKS      0x04
#define  DFU_WORD_PROGRAM                 0x05
#define  DFU_JUMP_TO_USER_CODE            0x06
#define  DFU_WORD_READ                    0x07

//
// Enabling register
//
#define ENABLE_REGISTER_1_WATCHDOG              0x01
#define ENABLE_REGISTER_1_RESET_MODE_MASK       0x000E
#define ENABLE_REGISTER_1_SYS_RESET_1S          0x0002
#define ENABLE_REGISTER_1_POWER_BUTTON_1S       0x0004
#define ENABLE_REGISTER_1_POWER_BUTTON_4S       0x0008
#define ENABLE_REGISTER_1_RSVD00                0x0010
#define ENABLE_REGISTER_1_uCWAKE_TIMER          0x0020 // se abilitato, asserisce uC_WAKE a tempo
#define ENABLE_REGISTER_1_LID_TRANSITION_HIGH   0x0040 // se abilitato, asserisce uC_WAKE dove LID gestito
#define ENABLE_REGISTER_1_LID_TRANSITION_LOW    0x0080 // se abilitato, asserisce uC_WAKE dove LID gestito
#define ENABLE_REGISTER_1_LID_TRANSITION_MASK   0x00c0 // all ENABLE_REGISTER_1_LID_TRANSITION_xx ORed together
#define ENABLE_REGISTER_1_RSVD01                0x0100
#define ENABLE_REGISTER_1_SLEEP_BUTTON          0x0200 // se abilitato, asserisce uC_WAKE dove SLEEP gestito
#define ENABLE_REGISTER_1_RSVD02                0x0400
#define ENABLE_REGISTER_1_RSVD03                0x0800
#define ENABLE_REGISTER_1_CEC                   0x1000 // 0=Disable CEC 1=Enable CEC
#define ENABLE_REGISTER_1_IRDA_RC5              0x2000
#define ENABLE_REGISTER_1_IRDA_PASSTHROUGH      0x4000

#define ENABLE_REGISTER_2_PCIE_WAKE_0           0x0001
#define ENABLE_REGISTER_2_PCIE_WAKE_2           0x0002
#define ENABLE_REGISTER_2_PCIE_WAKE_3           0x0004
#define ENABLE_REGISTER_2_PCIE_WAKE_1           0x0008
#define ENABLE_REGISTER_2_INTERNAL_FAN_4PIN     0x0010
#define ENABLE_REGISTER_2_EXTERNAL_FAN_4PIN     0x0020

//
// Status register
//
#define STATUS_REGISTER_1_WATCHDOG              ENABLE_REGISTER_1_WATCHDOG
#define STATUS_REGISTER_1_RESET_MODE_MASK       ENABLE_REGISTER_1_RESET_MODE_MASK
#define STATUS_REGISTER_1_POWER_FAIL            ENABLE_REGISTER_1_RSVD00
#define STATUS_REGISTER_1_uCWAKE_TIMER          ENABLE_REGISTER_1_uCWAKE_TIMER
#define STATUS_REGISTER_1_LID_TRANSITION_HIGH   ENABLE_REGISTER_1_LID_TRANSITION_HIGH
#define STATUS_REGISTER_1_LID_TRANSITION_LOW    ENABLE_REGISTER_1_LID_TRANSITION_LOW
#define STATUS_REGISTER_1_LID_STATE             ENABLE_REGISTER_1_RSVD01                // Stato corrente del segnale
#define STATUS_REGISTER_1_SLEEP_BUTTON          ENABLE_REGISTER_1_SLEEP_BUTTON
#define STATUS_REGISTER_1_WDT_INPUT             ENABLE_REGISTER_1_RSVD02
#define STATUS_REGISTER_1_WAKE_BUTTON           ENABLE_REGISTER_1_RSVD03

#define STATUS_REGISTER_1_CEC			ENABLE_REGISTER_1_CEC
#define STATUS_REGISTER_1_IRDA_RC5              ENABLE_REGISTER_1_IRDA_RC5
#define STATUS_REGISTER_1_IRDA_PASSTHROUGH      ENABLE_REGISTER_1_IRDA_PASSTHROUGH

//
// Status data
//

#define CEC_STATUS_MSG_RECEIVED_MASK		BIT(0)
#define CEC_STATUS_RX_ERROR_MASK		BIT(1)
#define CEC_STATUS_MSG_SENT_MASK		BIT(2)
#define CEC_STATUS_TX_ERROR_MASK		BIT(3)

#define CEC_STATUS_TX_NACK_ERROR		BIT(4)
#define CEC_STATUS_TX_LINE_ERROR		BIT(5)

//
// After G3 values
//
#define AFTER_G3_ON           0
#define AFTER_G3_OFF          1
#define AFTER_G3_LAST_STATE   2

//
// Watchdog
//
#define WATCHDOG_CONFIGURATION_MODE_MASK        0x03  // Mask Watchdog field
#define WATCHDOG_CONFIGURATION_MODE_NOACTION    0x00  // Watchdog field
#define WATCHDOG_CONFIGURATION_MODE_PWB_1S      0x01  // Watchdog field
#define WATCHDOG_CONFIGURATION_MODE_PWB_4S      0x02  // Watchdog field
#define WATCHDOG_CONFIGURATION_MODE_RESET       0x03  // Watchdog field
#define WATCHDOG_CONFIGURATION_MODE_SIGNALING   0x04  // Flag

//
// Firmware Recognition Signature
//
#define FIRMWARE_RECOGNITION_SIGNATURE  'R','i','C','o','N','o','S','c','I','m','E','n','T','o'

//
// Default value for unknown firmware
//
#define FIRMWARE_UNKNOWN                0xFFFF

//
// Firmware max dimension
//
#define FIRMWARE_SIZE                   0x2800  // Not include NVS e AON/AOFF data, just the firmware code
#define FIRMWARE_OFFSET                 0x0000
#define DEVICE_FIRMWARE_UPDATER_OFFSET  0x3000
#define DEVICE_FIRMWARE_UPDATER_SIZE    0x1000
#define MICRO_BINARY_SIZE               0x4000

//
// switch firmware signature
//
//
// Firma per poter switch firmware
//
#define MICRO_SWITCH_MODE_SIGNATURE     0x5EC0

#endif // __STM32_MICROCONTROLLER_H__

