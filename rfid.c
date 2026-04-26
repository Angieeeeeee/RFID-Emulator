// RC522 functions
// Angelina Marwan

// inspired by
// MFRC522: Arduino RFID Library for MFRC522 (SPI) https://github.com/miguelbalboa/rfid
// spi1.c by Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "rfid.h"
#include "gpio.h"
#include "clock.h"
#include "wait.h"

RFIDdata rfidTable[MAX_RFID_ENTRIES];
uint8_t  rfidCount = 0;

//-----------------------------------------------------------------------------
// SPI INITIALIZATIONS
//-----------------------------------------------------------------------------

void initRC(){
    initSystemClockTo40Mhz();

    enablePort(PORTD);
    enablePort(PORTB);
    enablePort(PORTF);
    _delay_cycles(3);

    // init SPI
    initSpi1();
    initSpi2();
    setSpi1BaudRate(1000000, 40000000);
    setSpi2BaudRate(1000000, 40000000);

    rc522Init(RC522_1);
    rc522Init(RC522_2);

    // empty table
    int i;
    for (i = 0; i < MAX_RFID_ENTRIES; i++) {
        rfidTable[i].hasData = 0;
    }
}

void initSpi1(void)
{
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R1;
    _delay_cycles(3);

    // MOSI
    selectPinPushPullOutput(RC1TX);
    setPinAuxFunction(RC1TX, GPIO_PCTL_PD3_SSI1TX);

    // SCLK
    selectPinPushPullOutput(RC1CLK);
    setPinAuxFunction(RC1CLK, GPIO_PCTL_PD0_SSI1CLK);

    // CS
    selectPinPushPullOutput(RC1FSS);
    setPinValue(RC1FSS, 1);

    // MISO
    selectPinDigitalInput(RC1RX);
    setPinAuxFunction(RC1RX, GPIO_PCTL_PD2_SSI1RX);

    // Configure the SSI1 as a SPI master, mode 3, 8bit operation
    SSI1_CR1_R &= ~SSI_CR1_SSE;            // disable while configuring
    SSI1_CR1_R  = 0;                        // master mode
    SSI1_CC_R   = 0;                        // system clock source
    SSI1_CR0_R  = SSI_CR0_FRF_MOTO         // Motorola SPI frame format
                | SSI_CR0_DSS_8;            // 8-bit data (SPO=0, SPH=0 default)
}

void setSpi1BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes2 = (fcyc * 2) / baudRate;
    SSI1_CR1_R  &= ~SSI_CR1_SSE;
    SSI1_CPSR_R  = (divisorTimes2 + 1) >> 1;   // round to nearest integer
    SSI1_CR1_R  |= SSI_CR1_SSE;                // enable SSI1
}

void initSpi2(void)
{
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;
    _delay_cycles(3);

    // MOSI
    selectPinPushPullOutput(RC2TX);
    setPinAuxFunction(RC2TX, GPIO_PCTL_PB7_SSI2TX);

    // SCLK
    selectPinPushPullOutput(RC2CLK);
    setPinAuxFunction(RC2CLK, GPIO_PCTL_PB4_SSI2CLK);

    // CS
    selectPinPushPullOutput(RC2FSS);
    setPinValue(RC2FSS, 1);

    // MISO
    selectPinDigitalInput(RC2RX);
    setPinAuxFunction(RC2RX, GPIO_PCTL_PB6_SSI2RX);

    // Configure SSI2 as master, Motorola frame, Mode 0, 8-bit
    SSI2_CR1_R &= ~SSI_CR1_SSE;
    SSI2_CR1_R  = 0;
    SSI2_CC_R   = 0;
    SSI2_CR0_R  = SSI_CR0_FRF_MOTO | SSI_CR0_DSS_8;
}

void setSpi2BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes2 = (fcyc * 2) / baudRate;
    SSI2_CR1_R  &= ~SSI_CR1_SSE;
    SSI2_CPSR_R  = (divisorTimes2 + 1) >> 1;
    SSI2_CR1_R  |= SSI_CR1_SSE;
}

//-----------------------------------------------------------------------------
// SENDING RECIEVING SPI DATA
//-----------------------------------------------------------------------------
/*
 * Send one byte and simultaneously receive one byte
 * The three-step wait sequence:
 *    TX FIFO not full   so   safe to write
 *    peripheral active  so   byte has been clocked in/out
 *    RX FIFO not empty  so   received byte is ready to read
 */
static uint8_t spi1Transfer(uint8_t txByte)
{
    while (!(SSI1_SR_R & SSI_SR_TNF));      // TX FIFO not full
    SSI1_DR_R = txByte;                     // sending the data
    while (SSI1_SR_R  & SSI_SR_BSY);        // transfer complete
    while (!(SSI1_SR_R & SSI_SR_RNE));      // RX byte ready
    return (uint8_t)(SSI1_DR_R & 0xFF);     // returns the data register
}

static uint8_t spi2Transfer(uint8_t txByte)
{
    while (!(SSI2_SR_R & SSI_SR_TNF));
    SSI2_DR_R = txByte;
    while (SSI2_SR_R  & SSI_SR_BSY);
    while (!(SSI2_SR_R & SSI_SR_RNE));
    return (uint8_t)(SSI2_DR_R & 0xFF);
}

void rcWriteReg(uint8_t module, uint8_t reg, uint8_t val)
{
    uint8_t addr = (reg << 1) & 0x7E;   // when bit 7 is 0, write. bit 0 is 0

    if (module == RC522_1) {
        while (SSI1_SR_R & SSI_SR_RNE) {};
        setPinValue(RC1FSS, 0);          // CS assert
        spi1Transfer(addr);              // address phase  (RX byte discarded)
        spi1Transfer(val);               // data phase     (RX byte discarded)
        setPinValue(RC1FSS, 1);          // CS deassert
    }
    else { // if we wanna use rc2
        while (SSI2_SR_R & SSI_SR_RNE) {};
        setPinValue(RC2FSS, 0);
        spi2Transfer(addr);
        spi2Transfer(val);
        setPinValue(RC2FSS, 1);
    }
}

uint8_t rcReadReg(uint8_t module, uint8_t reg)
{
    uint8_t addr = ((reg << 1) & 0x7E) | 0x80;  // bit 7 is 1 read
    uint8_t val;

    if (module == RC522_1) {
        while (SSI1_SR_R & SSI_SR_RNE) {};
        setPinValue(RC1FSS, 0);
        spi1Transfer(addr);              // starts driving MISO
        val = spi1Transfer(0x00);        // function returns SSI1_DR_R
        setPinValue(RC1FSS, 1);
    } else {
        while (SSI2_SR_R & SSI_SR_RNE) {};
        setPinValue(RC2FSS, 0);
        spi2Transfer(addr);
        val = spi2Transfer(0x00);
        setPinValue(RC2FSS, 1);
    }
    return val;
}

//-----------------------------------------------------------------------------
// RC522 TRANSMISSION
//-----------------------------------------------------------------------------

// add bits read from this reg onto a mask
void rcSetBitMask(uint8_t module, uint8_t reg, uint8_t mask)
{
    rcWriteReg(module, reg, rcReadReg(module, reg) | mask);
}

// Clear bits read from the reg
void rcClearBitMask(uint8_t module, uint8_t reg, uint8_t mask)
{
    rcWriteReg(module, reg, rcReadReg(module, reg) & ~mask);
}

// module initialization cause guess what this is inception apparently and there are registers inside registers
void rc522Init(uint8_t module)
{
    rcWriteReg(module, CommandReg, PCD_SoftReset);
    waitMicrosecond(50000);     // RC522 needs at least 37 ms after reset

    // no timeout , keep waiting till it recieves a key
    rcWriteReg(module, TModeReg,      0x00);
    rcWriteReg(module, TPrescalerReg, 0x00);
    rcWriteReg(module, TReloadRegH,   0x00);
    rcWriteReg(module, TReloadRegL,   0x00);

    rcWriteReg(module, TxASKReg, 0x40);         // 100 percent ASK modulation
    rcWriteReg(module, ModeReg,  0x3D);         // CRC preset = 0x6363

    // antenna ON
    if (!(rcReadReg(module, TxControlReg) & 0x03)) {
        rcSetBitMask(module, TxControlReg, 0x03);
    }
}

//
void rc522CalcCRC(uint8_t module, uint8_t *pIn, uint8_t len, uint8_t *pOut)
{
    uint8_t i, n;

    rcClearBitMask(module, DivIrqReg,    0x04); // clear Irq
    rcSetBitMask(module, FIFOLevelReg, 0x80); // flush FIFO buffer
    rcWriteReg(module, CommandReg,   PCD_Idle);

    for (i = 0; i < len; i++) {
        rcWriteReg(module, FIFODataReg, pIn[i]);
    }
    rcWriteReg(module, CommandReg, PCD_CalcCRC);

    // Poll for Irq
    i = 255;
    do {
        n = rcReadReg(module, DivIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x04));

    pOut[0] = rcReadReg(module, CRCResultRegL);
    pOut[1] = rcReadReg(module, CRCResultRegH);
}

// return: status
uint8_t rc522ToCard(uint8_t module, uint8_t cmd, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint16_t *backLen){
    uint8_t  status   = STATUS_ERR;
    uint8_t  irqEn    = 0x00;
    uint8_t  waitIRq  = 0x00;
    uint8_t  lastBits, n;
    uint16_t i;

    // Each command enables a different set of interrupt sources
    switch (cmd) {
        case PCD_MFAuthent:
            irqEn   = 0x12;     // ErrIEn | IdleIEn
            waitIRq = 0x10;     // IdleIRq
            break;
        case PCD_Transceive:
            irqEn   = 0x77;     // Tx|Rx|Idle|LoAlert|Err|Timer interrupt enables
            waitIRq = 0x30;     // RxIRq | IdleIRq
            break;
        default:
            break;
    }

    rcWriteReg(module, ComIEnReg,     irqEn | 0x80); // enable IRQs (inverted signal)
    rcClearBitMask(module, ComIrqReg, 0x80);          // clear all pending IRQ flags
    rcSetBitMask(module,  FIFOLevelReg, 0x80);        // flush FIFO
    rcWriteReg(module,    CommandReg, PCD_Idle);       // cancel any running command

    // Load payload into FIFO
    for (i = 0; i < sendLen; i++) {
        rcWriteReg(module, FIFODataReg, sendData[i]);
    }

    // Fire the command
    rcWriteReg(module, CommandReg, cmd);
    if (cmd == PCD_Transceive) {
        rcSetBitMask(module, BitFramingReg, 0x80);    // StartSend = 1
    }

    // Poll ComIrqReg until the expected IRQ fires or the hardware timer expires.
    // At 1 MHz SPI each register read takes ~20 µs → 3000 iterations ≈ 60 ms.
    i = 3000;
    do {
        n = rcReadReg(module, ComIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));

    rcClearBitMask(module, BitFramingReg, 0x80);      // StartSend = 0

    if (i == 0) { return STATUS_ERR; }                // software timeout

    // Check hardware error register (BufferOvfl | ColErr | CRCErr | ProtocolErr)
    if (rcReadReg(module, ErrorReg) & 0x1B) { return STATUS_ERR; }

    // A timer expiry with TimerIEn set means no card responded
    status = (n & irqEn & 0x01) ? STATUS_NOTAGERR : STATUS_OK;

    // get response bytes from FIFO (Transceive only)
    if (cmd == PCD_Transceive) {
        n        = rcReadReg(module, FIFOLevelReg);   // number of bytes in FIFO
        lastBits = rcReadReg(module, ControlReg) & 0x07; // valid bits in last byte

        // Convert to bit count
        *backLen = lastBits ? (uint16_t)(n - 1) * 8u + lastBits
                            : (uint16_t)n * 8u;

        if (n == 0) n = 1;
        if (n > 18) n = 18;                           // cap: max MIFARE read = 18 bytes

        for (i = 0; i < n; i++) {
            backData[i] = rcReadReg(module, FIFODataReg);
        }
    }

    return status;
}

uint8_t rc522Request(uint8_t module, uint8_t reqMode, uint8_t *tagType)
{
    uint16_t backBits;
    uint8_t  status;

    rcWriteReg(module, BitFramingReg, 0x07);   // TxLastBits = 7 → 7-bit short frame

    tagType[0] = reqMode;
    status = rc522ToCard(module, PCD_Transceive, tagType, 1, tagType, &backBits);

    if ((status != STATUS_OK) || (backBits != 16)) {
        status = STATUS_ERR;
    }
    return status;
}

uint8_t rc522Anticoll(uint8_t module, uint8_t *serNum)
{
    uint8_t  i, status;
    uint8_t  serCheck = 0;
    uint16_t unLen;
    uint8_t  sendBuf[2] = { PICC_SEL_CL1, 0x20 };  // NVB = 0x20 (no UID bits known)

    rcWriteReg(module, BitFramingReg, 0x00);   // full-byte framing

    status = rc522ToCard(module, PCD_Transceive, sendBuf, 2, serNum, &unLen);

    if (status == STATUS_OK) {
        // Verify Block Check Character: UID[0]^UID[1]^UID[2]^UID[3] == BCC
        for (i = 0; i < 4; i++) { serCheck ^= serNum[i]; }
        if (serCheck != serNum[4]) { status = STATUS_ERR; }
    }
    return status;
}

uint8_t rc522SelectTag(uint8_t module, uint8_t *serNum)
{
    uint8_t  i, status;
    uint16_t recvBits;
    uint8_t  buf[9];

    buf[0] = PICC_SEL_CL1;
    buf[1] = 0x70;                          // NVB = 0x70 → all 5 UID bytes follow
    for (i = 0; i < 5; i++) { buf[i + 2] = serNum[i]; }
    rc522CalcCRC(module, buf, 7, &buf[7]);  // append CRC over [SEL NVB UID BCC]

    status = rc522ToCard(module, PCD_Transceive, buf, 9, buf, &recvBits);

    // Card replies with 3 bytes (SAK + CRC), recvBits = 24; we only need SAK
    return ((status == STATUS_OK) && (recvBits == 0x18)) ? buf[0] : 0;
}

/*
 * rc522Auth() — MIFARE Classic sector authentication.
 *
 * authMode : PICC_MF_AUTH_KEY_A (0x60) or PICC_MF_AUTH_KEY_B (0x61)
 * blockAddr: any block address within the target sector
 * sectorKey: 6-byte key
 * serNum   : first 4 bytes of the card UID
 *
 * Must be called once per sector before any block in that sector can be
 * read or written.  Call rc522StopCrypto() when done with the sector.
 */
uint8_t rc522Auth(uint8_t module, uint8_t authMode, uint8_t blockAddr,
                  uint8_t *sectorKey, uint8_t *serNum)
{
    uint8_t  i, status;
    uint16_t recvBits;
    uint8_t  buf[12];

    buf[0] = authMode;
    buf[1] = blockAddr;
    for (i = 0; i < 6; i++) { buf[i + 2] = sectorKey[i]; }  // 6-byte key
    for (i = 0; i < 4; i++) { buf[i + 8] = serNum[i]; }      // UID[0..3]

    status = rc522ToCard(module, PCD_MFAuthent, buf, 12, buf, &recvBits);

    // Authentication success is confirmed by the Crypto1On bit in Status2Reg
    if ((status != STATUS_OK) || !(rcReadReg(module, Status2Reg) & 0x08)) {
        status = STATUS_ERR;
    }
    return status;
}

/*
 * rc522ReadBlock() — read one 16-byte MIFARE Classic block.
 *
 * recvData must point to an 18-byte buffer (16 data + 2 CRC).
 * The sector containing blockAddr must already be authenticated.
 */
uint8_t rc522ReadBlock(uint8_t module, uint8_t blockAddr, uint8_t *recvData)
{
    uint16_t unLen;
    uint8_t  status;

    recvData[0] = PICC_MF_READ;
    recvData[1] = blockAddr;
    rc522CalcCRC(module, recvData, 2, &recvData[2]);    // append 2-byte CRC

    status = rc522ToCard(module, PCD_Transceive, recvData, 4, recvData, &unLen);

    // Card sends 16 data bytes + 2 CRC bytes = 18 bytes = 144 bits (0x90)
    if ((status != STATUS_OK) || (unLen != 0x90)) {
        status = STATUS_ERR;
    }
    return status;
}

uint8_t rc522WriteBlock(uint8_t module, uint8_t blockAddr, uint8_t *writeData)
{
    uint8_t  i, status;
    uint16_t recvBits;
    uint8_t  buf[18];

    // --- Phase 1: command ---
    buf[0] = PICC_MF_WRITE;
    buf[1] = blockAddr;
    rc522CalcCRC(module, buf, 2, &buf[2]);

    status = rc522ToCard(module, PCD_Transceive, buf, 4, buf, &recvBits);
    // Expect a 4-bit ACK = 0x0A
    if ((status != STATUS_OK) || (recvBits != 4) || ((buf[0] & 0x0F) != 0x0A)) {
        return STATUS_ERR;
    }

    // --- Phase 2: data ---
    for (i = 0; i < 16; i++) { buf[i] = writeData[i]; }
    rc522CalcCRC(module, buf, 16, &buf[16]);

    status = rc522ToCard(module, PCD_Transceive, buf, 18, buf, &recvBits);
    if ((status != STATUS_OK) || (recvBits != 4) || ((buf[0] & 0x0F) != 0x0A)) {
        status = STATUS_ERR;
    }
    return status;
}

/*
 * rc522HaltA() — send HLTA to put the card into the HALT state.
 * The card does not reply to HLTA; we ignore the return status.
 */
void rc522HaltA(uint8_t module)
{
    uint16_t unLen;
    uint8_t  buf[4];

    buf[0] = PICC_HLTA;
    buf[1] = 0x00;
    rc522CalcCRC(module, buf, 2, &buf[2]);
    rc522ToCard(module, PCD_Transceive, buf, 4, buf, &unLen);
    // No status check — card is not required to reply
}

void rc522StopCrypto(uint8_t module)
{
    rcClearBitMask(module, Status2Reg, 0x08);   // MFCrypto1On = 0
}

//-----------------------------------------------------------------------------
// ALL IN ONE FUNCTIONS FOR MENU
//-----------------------------------------------------------------------------

/*
 * readRFID() — detect a MIFARE Classic card on RC522_1, authenticate each
 * sector with the default Key A, read every data block (skipping sector
 * trailers), and store the result in rfidTable.
 *
 * Parameters:
 *   name  — null-terminated label for this tag (truncated to 29 chars)
 *
 * Returns:
 *   >= 0   table index of the stored entry (new or updated)
 *   -1     no tag detected
 *   -2     there is no space on table
 *
 * If the same UID is already in the table the existing entry is refreshed.
 * Reads blocks 0–63 (full MIFARE Classic 1K range) up to MAX_RFID_BLOCKS
 * data blocks.  Block 0 of sector 0 (manufacturer block) is included for
 * storage completeness but will be skipped during writeRFID.
 */
int8_t readRFID(char *name)
{
    uint8_t  status;
    uint8_t  atqa[2];           // Answer To reQuest type A (2 bytes)
    uint8_t  serNum[5];         // UID bytes [0..3] + BCC
    uint8_t  rawBlock[18];      // 16 data bytes + 2 CRC bytes
    uint8_t  i, j;
    int8_t   entryIdx = -1;
    uint8_t  lastSector = 0xFF;

    // detect card
    status = rc522Request(RC522_1, PICC_REQA, atqa);
    if (status != STATUS_OK) { return -1; }

    // resolve UID via anti-collision
    status = rc522Anticoll(RC522_1, serNum);
    if (status != STATUS_OK) { return -1; }

    // pack 4-byte UID into uint32_t id
    uint32_t newId = ((uint32_t)serNum[0] << 24)
                   | ((uint32_t)serNum[1] << 16)
                   | ((uint32_t)serNum[2] <<  8)
                   |  (uint32_t)serNum[3];

    rc522SelectTag(RC522_1, serNum);

    // find place in table
    for (i = 0; i < MAX_RFID_ENTRIES; i++) {
        if (rfidTable[i].hasData && rfidTable[i].id == newId) { // already exists in table
            entryIdx = (int8_t)i;
            break;
        }
    }
    if (entryIdx == -1) {
        for (i = 0; i < MAX_RFID_ENTRIES; i++) { // empty spot
            if (!rfidTable[i].hasData) {
                entryIdx = (int8_t)i;
                break;
            }
        }
    }
    if (entryIdx == -1) {
        rc522HaltA(RC522_1);
        return -2;              // table full
    }

    // populate table
    rfidTable[entryIdx].id        = newId;
    rfidTable[entryIdx].uidLength = 5;              // 4 UID + 1 BCC
    for (i = 0; i < 5; i++) { rfidTable[entryIdx].uid[i] = serNum[i]; }

    // Copy name safely
    for (i = 0; i < 29 && name[i] != '\0'; i++) {
        rfidTable[entryIdx].name[i] = name[i];
    }
    rfidTable[entryIdx].name[i] = '\0';

    // MIFARE Classic 1K card (used in key fobs)
    // 16 sectors with 4 blocks, 16 bytes each
    uint8_t blockCount = 0;

    for (j = 0; j < 64 && blockCount < MAX_RFID_BLOCKS; j++) {
        uint8_t sector        = j / 4;
        uint8_t blockInSector = j % 4;

        // Re-authenticate whenever we enter a new sector
        if (sector != lastSector) {
            status = rc522Auth(RC522_1, PICC_MF_AUTH_KEY_A,
                               j, (uint8_t *)defaultKeyA, serNum);
            if (status != STATUS_OK) {
                // Auth failed => skip the rest of this sector
                j += (3u - blockInSector);  // advance to last block of sector
                continue;
            }
            lastSector = sector;
        }

        // Block 3 of every sector is the sector trailer (keys + access bits)
        if (blockInSector == 3) { continue; }

        // Read 16-byte block (rawBlock[0..15] = data, [16..17] = CRC)
        status = rc522ReadBlock(RC522_1, j, rawBlock);
        if (status == STATUS_OK) {
            rfidTable[entryIdx].blocks[blockCount].addr = j;
            for (i = 0; i < BLOCK_SIZE; i++) {
                rfidTable[entryIdx].blocks[blockCount].data[i] = rawBlock[i];
            }
            blockCount++;
        }
    }

    rfidTable[entryIdx].blockCount = blockCount;
    rfidTable[entryIdx].hasData    = 1;
    if (rfidCount < MAX_RFID_ENTRIES) { rfidCount++; }

    rc522StopCrypto(RC522_1);
    rc522HaltA(RC522_1);

    return entryIdx;
}

/*
 * writeRFID() — find the rfidTable entry matching selectedId and write all
 * stored blocks onto a blank/writable tag presented to RC522_2.
 *
 * Parameters:
 *   selectedId  — the uint32_t id of the entry to write (as packed by readRFID)
 *
 * Returns:
 *   STATUS_OK   all blocks written successfully
 *   STATUS_ERR  id not in table, no tag on RC522_2, auth failed, or write error
 * Block 0 of sector 0 is the manufacturer block
 */
uint8_t writeRFID(uint32_t selectedId)
{
    uint8_t i;
    int8_t  entryIdx = -1;

    // locate the key in table
    for (i = 0; i < MAX_RFID_ENTRIES; i++) {
        if (rfidTable[i].hasData && rfidTable[i].id == selectedId) {
            entryIdx = (int8_t)i;
            break;
        }
    }
    if (entryIdx == -1) { return STATUS_ERR; }  // id not found

    uint8_t  status;
    uint8_t  atqa[2];
    uint8_t  serNum[5];

    // detect target tag
    status = rc522Request(RC522_2, PICC_REQA, atqa);
    if (status != STATUS_OK) { return STATUS_ERR; }

    status = rc522Anticoll(RC522_2, serNum);
    if (status != STATUS_OK) { return STATUS_ERR; }

    // SELECT target tag
    rc522SelectTag(RC522_2, serNum);

    // iterate over stored blocks and write each one
    uint8_t lastSector = 0xFF;

    for (i = 0; i < rfidTable[entryIdx].blockCount; i++) {
        uint8_t blockAddr = rfidTable[entryIdx].blocks[i].addr;
        uint8_t sector    = blockAddr / 4;

        // Block 0 (manufacturer block) is permanently locked on real cards
        if (blockAddr == 0) { continue; }

        // Authenticate at the start of each new sector
        if (sector != lastSector) {
            status = rc522Auth(RC522_2, PICC_MF_AUTH_KEY_A,
                               blockAddr, (uint8_t *)defaultKeyA, serNum);
            if (status != STATUS_OK) {
                rc522StopCrypto(RC522_2);
                rc522HaltA(RC522_2);
                return STATUS_ERR;
            }
            lastSector = sector;
        }

        // Write the 16-byte block to the target card
        status = rc522WriteBlock(RC522_2, blockAddr,
                                 rfidTable[entryIdx].blocks[i].data);
        if (status != STATUS_OK) {
            rc522StopCrypto(RC522_2);
            rc522HaltA(RC522_2);
            return STATUS_ERR;
        }
    }

    rc522StopCrypto(RC522_2);
    rc522HaltA(RC522_2);
    return STATUS_OK;
}
