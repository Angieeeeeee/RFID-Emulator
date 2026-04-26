// RC522 Functions
// Angelina Abuhilal Marwan Abulibdeh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

#ifndef RFID_H_
#define RFID_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

// WIRIGN


// RC522_1 (SSI1)
#define RC1TX   PORTD,3     // SSI1Tx  (MOSI)
#define RC1RX   PORTD,2     // SSI1Rx  (MISO)
#define RC1FSS  PORTD,1     // Chip-select //active low
#define RC1CLK  PORTD,0     // SSI1Clk

// RC522_2 (SSI2)
#define RC2TX   PORTB,7     // SSI2Tx  (MOSI)
#define RC2RX   PORTB,6     // SSI2Rx  (MISO)
#define RC2FSS  PORTB,5     // Chip-select //active low
#define RC2CLK  PORTB,4     // SSI2Clk

#define MAX_RFID_ENTRIES  8     // distinct tags the table can hold
#define MAX_RFID_BLOCKS   16    // max MIFARE blocks stored per tag
#define BLOCK_SIZE        16    // bytes in one MIFARE Classic data block

// One stored block: its address on the physical card + raw 16-byte content
typedef struct {
    uint8_t addr;               // block address on card (0–63 for 1K)
    uint8_t data[BLOCK_SIZE];
} RFIDBlock;

// One entry in the emulator table
typedef struct {
    char      name[30];                     // user-supplied label
    uint32_t  id;                           // UID packed as 32 bits (bytes 0–3)
    uint8_t   uid[10];                      // full UID (4, 7, or 10 bytes)
    uint8_t   uidLength;                    // actual UID byte count
    uint8_t   blockCount;                   // how many entries in blocks[] are valid
    uint8_t   hasData;                      // 1 = slot occupied, 0 = free
    RFIDBlock blocks[MAX_RFID_BLOCKS];      // stored block data (no heap needed)
} RFIDdata;

extern RFIDdata rfidTable[MAX_RFID_ENTRIES];
extern uint8_t  rfidCount;

//-----------------------------------------------------------------------------
// RC522 Register Map: Created by Eelco Rouw - Mainly based on code from Grant Gibson (www.grantgibson.co.uk) and Dr.Leong ( WWW.B2CQSHOP.COM ) Released into the public domain
//-----------------------------------------------------------------------------
// Page 0 — Command & Status
#define CommandReg      0x01
#define ComIEnReg       0x02
#define DivIEnReg       0x03
#define ComIrqReg       0x04
#define DivIrqReg       0x05
#define ErrorReg        0x06
#define Status1Reg      0x07
#define Status2Reg      0x08
#define FIFODataReg     0x09
#define FIFOLevelReg    0x0A
#define WaterLevelReg   0x0B
#define ControlReg      0x0C
#define BitFramingReg   0x0D
#define CollReg         0x0E
// Page 1 — Command Configuration
#define ModeReg         0x11
#define TxModeReg       0x12
#define RxModeReg       0x13
#define TxControlReg    0x14
#define TxASKReg        0x15
// Page 2 — Configuration
#define CRCResultRegH   0x21
#define CRCResultRegL   0x22
#define TModeReg        0x2A
#define TPrescalerReg   0x2B
#define TReloadRegH     0x2C
#define TReloadRegL     0x2D

//-----------------------------------------------------------------------------
// RC522 PCD (Proximity Coupling Device) Commands
//-----------------------------------------------------------------------------
#define PCD_Idle         0x00
#define PCD_CalcCRC      0x03
#define PCD_Transmit     0x04
#define PCD_Receive      0x08
#define PCD_Transceive   0x0C   // transmit + receive
#define PCD_MFAuthent    0x0E   // MIFARE Classic authentication
#define PCD_SoftReset    0x0F

//-----------------------------------------------------------------------------
// PICC (Card) Commands — MIFARE Classic
//-----------------------------------------------------------------------------
#define PICC_REQA           0x26    // Request Type A
#define PICC_WUPA           0x52    // Wake-up Type A
#define PICC_SEL_CL1        0x93    // Anti-collision / Select, Cascade Level 1
#define PICC_SEL_CL2        0x95    // Anti-collision / Select, Cascade Level 2
#define PICC_HLTA           0x50    // Halt (Type A)
#define PICC_MF_AUTH_KEY_A  0x60    // Authenticate with Key A
#define PICC_MF_AUTH_KEY_B  0x61    // Authenticate with Key B
#define PICC_MF_READ        0x30    // Read block
#define PICC_MF_WRITE       0xA0    // Write block

#define RC522_1   1     // reader
#define RC522_2   2     // writer

#define STATUS_OK 0
#define STATUS_ERR 1
#define STATUS_NOTAGERR 2

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initRC(); // USE IN MAIN
void initSpi1(void);
void setSpi1BaudRate(uint32_t baudRate, uint32_t fcyc);
void initSpi2(void);
void setSpi2BaudRate(uint32_t baudRate, uint32_t fcyc);

static uint8_t spi1Transfer(uint8_t txByte);
static uint8_t spi2Transfer(uint8_t txByte);
void rcWriteReg(uint8_t module, uint8_t reg, uint8_t val);
uint8_t rcReadReg(uint8_t module, uint8_t reg);

void rcSetBitMask(uint8_t module, uint8_t reg, uint8_t mask);
void rcClearBitMask(uint8_t module, uint8_t reg, uint8_t mask);
void rc522Init(uint8_t module);
bool rc522SpiSelfTest(uint8_t module);
uint8_t rc522GetVersion(uint8_t module);
void rc522CalcCRC(uint8_t module, uint8_t *pIn, uint8_t len, uint8_t *pOut);
uint8_t rc522ToCard(uint8_t module, uint8_t cmd, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint16_t *backLen);
uint8_t rc522Request(uint8_t module, uint8_t reqMode, uint8_t *tagType);
uint8_t rc522Anticoll(uint8_t module, uint8_t *serNum);
uint8_t rc522SelectTag(uint8_t module, uint8_t *serNum);
uint8_t rc522Auth(uint8_t module, uint8_t authMode, uint8_t blockAddr, uint8_t *sectorKey, uint8_t *serNum);
uint8_t rc522ReadBlock(uint8_t module, uint8_t blockAddr, uint8_t *recvData);
uint8_t rc522WriteBlock(uint8_t module, uint8_t blockAddr, uint8_t *writeData);
void rc522HaltA(uint8_t module);
void rc522StopCrypto(uint8_t module);

int8_t readRFID(char *name); //USE IN MAIN
uint8_t writeRFID(uint32_t selectedId); //USE IN MAIN

#endif
