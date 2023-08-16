// GIF Player with four buttons
// Author Juha-Pekka Varjonen
// Copyright (c) 2023 Geeky Gadgets Online
// 
// This program is intended for RP2040 especially with Waveshare LCD display.
// It shows four gif's selectable by user with four buttons. Gif's are saved
// to local FAT filesystem via USB.
//
// Gif's are named after which button activates it. Format is Xbtn.gif where
// X is a button number. Gif's must be same size than display. In this case
// 240x240 pixels.
//
// Program uses DMA and full frame buffer to update display.
//
// Links to source codes that was helping during development process
// https://learn.adafruit.com/mini-gif-players/coding-the-mini-gif-players
// https://github.com/MarkTillotson/PicoSPI
// https://github.com/stienman/Raspberry-Pi-Pico-PIO-LCD-Driver
//
// ---------------------------------------------------------------------------
// Parts of this program is lisenced under following licenses
//
// Copyright 2022 Limor Fried for Adafruit Industries
// License MIT
//
//  (c) 2023 Dmitry Grinberg  https://dmitry.gr
//  Redistribution and use in source and binary forms, with or without modification,
//    are permitted provided that the following conditions are met:
//
//	Redistributions of source code must retain the above copyright notice, this list
//		of conditions and the following disclaimer.
//	Redistributions in binary form must reproduce the above copyright notice, this
//		list of conditions and the following disclaimer in the documentation and/or
//		other materials provided with the distribution.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//	EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
//	IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
//	INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
//	NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
//	PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
//	WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//	POSSIBILITY OF SUCH DAMAGE.

#include <AnimatedGIF.h>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>
#include "SPI.h"
#include "Adafruit_GC9A01A.h"
#include "hardware/dma.h"
#include "PicoSPI.h"

#include "sleep.h"
#include "rosc.h"
#include "hardware/structs/scb.h"

#define TFT_CS     9
#define TFT_DC     8
#define TFT_RST   12
#define TFT_MISO  12 // not in use
#define TFT_MOSI  11
#define TFT_SCLK  10

#define DISPLAY_WIDTH 240
#define DISPLAY_HEIGHT 240

#define GIFDIRNAME "/"

#define MADCTL_MY 0x80  ///< Bottom to top
#define MADCTL_MX 0x40  ///< Right to left
#define MADCTL_MV 0x20  ///< Reverse Mode
#define MADCTL_ML 0x10  ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00 ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08 ///< Blue-Green-Red pixel order
#define MADCTL_MH 0x04  ///< LCD refresh right to left

#define SIDE_SET_HAS_ENABLE_BIT 1
#define SIDE_SET_NUM_BITS       2
#define DEFINE_PIO_INSTRS
#include "pioAsm.h"

static uint32_t mFramebufBytes = DISPLAY_WIDTH * DISPLAY_HEIGHT * 2;
static uint16_t fBuffer[DISPLAY_WIDTH * DISPLAY_HEIGHT];
static void* mFb = (void*)&fBuffer;

int lastPressedBtn = 0;
unsigned long sleepTimer = 0;

// registers to be saved before sleep and restored after sleep
uint _scb_orig;           // clock registers
uint _en0_orig;           // ""
uint _en1_orig;           // ""

#if defined(ARDUINO_ARCH_ESP32)
  // ESP32 use same flash device that store code.
  // Therefore there is no need to specify the SPI and SS
  Adafruit_FlashTransport_ESP32 flashTransport;

#elif defined(ARDUINO_ARCH_RP2040)
  // RP2040 use same flash device that store code for file system. Therefore we
  // only need to specify start address and size (no need SPI or SS)
  // By default (start=0, size=0), values that match file system setting in
  // 'Tools->Flash Size' menu selection will be used.
  // Adafruit_FlashTransport_RP2040 flashTransport;

  // To be compatible with CircuitPython partition scheme (start_address = 1 MB,
  // size = total flash - 1 MB) use const value (CPY_START_ADDR, CPY_SIZE) or
  // subclass Adafruit_FlashTransport_RP2040_CPY. Un-comment either of the
  // following line:
    Adafruit_FlashTransport_RP2040
      flashTransport(Adafruit_FlashTransport_RP2040::CPY_START_ADDR,
                     Adafruit_FlashTransport_RP2040::CPY_SIZE);
  //Adafruit_FlashTransport_RP2040_CPY flashTransport;

#else
  // On-board external flash (QSPI or SPI) macros should already
  // defined in your board variant if supported
  // - EXTERNAL_FLASH_USE_QSPI
  // - EXTERNAL_FLASH_USE_CS/EXTERNAL_FLASH_USE_SPI
  #if defined(EXTERNAL_FLASH_USE_QSPI)
    Adafruit_FlashTransport_QSPI flashTransport;

  #elif defined(EXTERNAL_FLASH_USE_SPI)
    Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS, EXTERNAL_FLASH_USE_SPI);

  #else
    #error No QSPI/SPI flash are defined on your board variant.h !
  #endif
#endif

Adafruit_SPIFlash flash(&flashTransport);

// file system object from SdFat
FatFileSystem fatfs;

AnimatedGIF gif;
File32 f, root;

// TFT init commands
static const uint8_t PROGMEM initcmd[] = {
  GC9A01A_INREGEN2, 0,
  0xEB, 1, 0x14, // ?
  GC9A01A_INREGEN1, 0,
  GC9A01A_INREGEN2, 0,
  0xEB, 1, 0x14, // ?
  0x84, 1, 0x40, // ?
  0x85, 1, 0xFF, // ?
  0x86, 1, 0xFF, // ?
  0x87, 1, 0xFF, // ?
  0x88, 1, 0x0A, // ?
  0x89, 1, 0x21, // ?
  0x8A, 1, 0x00, // ?
  0x8B, 1, 0x80, // ?
  0x8C, 1, 0x01, // ?
  0x8D, 1, 0x01, // ?
  0x8E, 1, 0xFF, // ?
  0x8F, 1, 0xFF, // ?
  0xB6, 2, 0x00, 0x00, // ?
  GC9A01A_MADCTL, 1, MADCTL_MX | MADCTL_BGR,
  GC9A01A_COLMOD, 1, 0x05,
  0x90, 4, 0x08, 0x08, 0x08, 0x08, // ?
  0xBD, 1, 0x06, // ?
  0xBC, 1, 0x00, // ?
  0xFF, 3, 0x60, 0x01, 0x04, // ?
  GC9A01A1_POWER2, 1, 0x13,
  GC9A01A1_POWER3, 1, 0x13,
  GC9A01A1_POWER4, 1, 0x22,
  0xBE, 1, 0x11, // ?
  0xE1, 2, 0x10, 0x0E, // ?
  0xDF, 3, 0x21, 0x0c, 0x02, // ?
  GC9A01A_GAMMA1, 6, 0x45, 0x09, 0x08, 0x08, 0x26, 0x2A,
  GC9A01A_GAMMA2, 6, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F,
  GC9A01A_GAMMA3, 6, 0x45, 0x09, 0x08, 0x08, 0x26, 0x2A,
  GC9A01A_GAMMA4, 6, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F,
  0xED, 2, 0x1B, 0x0B, // ?
  0xAE, 1, 0x77, // ?
  0xCD, 1, 0x63, // ?
  // Unsure what this line (from manufacturer's boilerplate code) is
  // meant to do, but users reported issues, seems to work OK without:
  //0x70, 9, 0x07, 0x07, 0x04, 0x0E, 0x0F, 0x09, 0x07, 0x08, 0x03, // ?
  GC9A01A_FRAMERATE, 1, 0x34,
  0x62, 12, 0x18, 0x0D, 0x71, 0xED, 0x70, 0x70, // ?
            0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70,
  0x63, 12, 0x18, 0x11, 0x71, 0xF1, 0x70, 0x70, // ?
            0x18, 0x13, 0x71, 0xF3, 0x70, 0x70,
  0x64, 7, 0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07, // ?
  0x66, 10, 0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00, // ?
  0x67, 10, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98, // ?
  0x74, 7, 0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00, // ?
  0x98, 2, 0x3e, 0x07, // ?
  GC9A01A_TEON, 0,
  GC9A01A_INVON, 0,
  GC9A01A_SLPOUT, 0x80, // Exit sleep
  GC9A01A_DISPON, 0x80, // Display on
  0x00                  // End of list
};

// Blocking version of pixel write without DMA
void writePixels2Blocking(uint16_t *colors, uint32_t len) {
  // switch to 16-bit writes
  hw_write_masked(&spi_get_hw(spi1)->cr0, 15 << SPI_SSPCR0_DSS_LSB, SPI_SSPCR0_DSS_BITS);
  spi_write16_blocking(spi1, colors, len);
  // switch back to 8-bit
  hw_write_masked(&spi_get_hw(spi1)->cr0, 7 << SPI_SSPCR0_DSS_LSB, SPI_SSPCR0_DSS_BITS);
}

void spiWrite(uint8_t b) {
  spi_write_blocking(spi1, &b, 1);
}

void SPI_WRITE16(uint16_t w) {
  w = __builtin_bswap16(w);
  spi_write_blocking(spi1, (uint8_t *)&w, 2);
}

void writeCommand(uint8_t cmd) {
  digitalWrite(TFT_DC, LOW);
  spiWrite(cmd);
  digitalWrite(TFT_DC, HIGH);
}

void sendCommand(uint8_t commandByte, const uint8_t *dataBytes = NULL, uint8_t numDataBytes = 0) {
  digitalWrite(TFT_CS, LOW);

  digitalWrite(TFT_DC, LOW); // Command mode
  spiWrite(commandByte);     // Send the command byte
  digitalWrite(TFT_DC, HIGH); 
  for (int i = 0; i < numDataBytes; i++) {
    spiWrite(*dataBytes);    // Send the data bytes
    dataBytes++;
  }

  digitalWrite(TFT_CS, HIGH);
}

void displayInit() {
  uint8_t cmd, x, numArgs;
  const uint8_t *addr = initcmd;
  while ((cmd = pgm_read_byte(addr++)) > 0) {
    x = pgm_read_byte(addr++);
    numArgs = x & 0x7F;
    sendCommand(cmd, addr, numArgs);
    addr += numArgs;
    if (x & 0x80)
      delay(150);
  }
}

void setAddrWindow(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h) {
  uint16_t x2 = (x1 + w - 1), y2 = (y1 + h - 1);
  writeCommand(GC9A01A_CASET); // Column address set
  SPI_WRITE16(x1);
  SPI_WRITE16(x2);
  writeCommand(GC9A01A_RASET); // Row address set
  SPI_WRITE16(y1);
  SPI_WRITE16(y2);
  writeCommand(GC9A01A_RAMWR); // Write to RAM
}

void write2Buffer(int x, int y, int count, uint16_t* data) {
  memcpy(&fBuffer[(DISPLAY_HEIGHT * y) + x], data, count * 2);
}

void * GIFOpenFile(const char *fname, int32_t *pSize)
{
  f = fatfs.open(fname);
  if (f)
  {
    *pSize = f.size();
    return (void *)&f;
  }
  return NULL;
} /* GIFOpenFile() */

void GIFCloseFile(void *pHandle)
{
  File32 *f = static_cast<File32 *>(pHandle);
  if (f != NULL)
     f->close();
} /* GIFCloseFile() */

int32_t GIFReadFile(GIFFILE *pFile, uint8_t *pBuf, int32_t iLen)
{
    int32_t iBytesRead;
    iBytesRead = iLen;
    File32 *f = static_cast<File32 *>(pFile->fHandle);
    // Note: If you read a file all the way to the last byte, seek() stops working
    if ((pFile->iSize - pFile->iPos) < iLen)
       iBytesRead = pFile->iSize - pFile->iPos - 1; // <-- ugly work-around
    if (iBytesRead <= 0)
       return 0;
    iBytesRead = (int32_t)f->read(pBuf, iBytesRead);
    pFile->iPos = f->position();
    return iBytesRead;
} /* GIFReadFile() */

int32_t GIFSeekFile(GIFFILE *pFile, int32_t iPosition)
{ 
  int i = micros();
  File32 *f = static_cast<File32 *>(pFile->fHandle);
  f->seek(iPosition);
  pFile->iPos = (int32_t)f->position();
  i = micros() - i;
//  Serial.printf("Seek time = %d us\n", i);
  return pFile->iPos;
} /* GIFSeekFile() */

// Draw a line of image directly on the LCD
void GIFDraw(GIFDRAW *pDraw) {
    uint8_t *s;
    uint16_t *d, *usPalette, usTemp[320];
    int x, y, iWidth;

    iWidth = pDraw->iWidth;

    if (iWidth + pDraw->iX > DISPLAY_WIDTH)
       iWidth = DISPLAY_WIDTH - pDraw->iX;
    usPalette = pDraw->pPalette;
    y = pDraw->iY + pDraw->y; // current line
    if (y >= DISPLAY_HEIGHT || pDraw->iX >= DISPLAY_WIDTH || iWidth < 1)
       return; 
    s = pDraw->pPixels;
    if (pDraw->ucDisposalMethod == 2) // restore to background color
    {
      for (x=0; x<iWidth; x++)
      {
        if (s[x] == pDraw->ucTransparent)
           s[x] = pDraw->ucBackground;
      }
      pDraw->ucHasTransparency = 0;
    }

    // Apply the new pixels to the main image
    if (pDraw->ucHasTransparency) // if transparency used
    {
      uint8_t *pEnd, c, ucTransparent = pDraw->ucTransparent;
      int x, iCount;
      pEnd = s + iWidth;
      x = 0;
      iCount = 0; // count non-transparent pixels
      while(x < iWidth)
      {
        c = ucTransparent-1;
        d = usTemp;
        while (c != ucTransparent && s < pEnd)
        {
          c = *s++;
          if (c == ucTransparent) // done, stop
          {
            s--; // back up to treat it like transparent
          }
          else // opaque
          {
             *d++ = usPalette[c];
             iCount++;
          }
        } // while looking for opaque pixels
        if (iCount) { // any opaque pixels?
          
          write2Buffer(pDraw->iX+x, y, iCount, (uint16_t*)&usTemp);
          
          x += iCount;
          iCount = 0;
        }
        // no, look for a run of transparent pixels
        c = ucTransparent;
        while (c == ucTransparent && s < pEnd)
        {
          c = *s++;
          if (c == ucTransparent)
             iCount++;
          else
             s--; 
        }
        if (iCount)
        {
          x += iCount; // skip these
          iCount = 0;
        }
      }
    }
    else
    {
      s = pDraw->pPixels;
      // Translate the 8-bit pixels through the RGB565 palette (already byte reversed)
      for (x=0; x<iWidth; x++)
        usTemp[x] = usPalette[*s++];
      
      write2Buffer(pDraw->iX, y, iWidth, (uint16_t*)&usTemp);

    }
} /* GIFDraw() */

static void dispPrvPioProgram16bpp(void) {

	uint_fast8_t pc = 0, lblMore, lblPullNgo, lblMoreBits, sm0StartPC, sm0EndPC, sm2StartPC, sm2EndPC;

	//SM0 program: SPI the data out. input 16 bit words
	//OSR shifts left, ISR shifts left, sideset used for clock, data output to MOSI using OUT, autopull
	
	sm0StartPC = pc;
	pio0_hw->instr_mem[pc++] = I_SET(0, 4, SET_DST_X, 0x09);
	pio0_hw->instr_mem[pc++] = I_MOV(0, 0, MOV_DST_ISR, MOV_OP_COPY, MOV_SRC_X);
	pio0_hw->instr_mem[pc++] = I_IN(0, 0, IN_SRC_ZEROES, 10);
	pio0_hw->instr_mem[pc++] = I_MOV(0, 0, MOV_DST_X, MOV_OP_COPY, MOV_SRC_ISR);
	lblPullNgo = pc;
	pio0_hw->instr_mem[pc++] = I_PULL(0, 0, 0, 1);
	pio0_hw->instr_mem[pc++] = I_SET(0, 0, SET_DST_Y, 15);
	lblMoreBits = pc;
	pio0_hw->instr_mem[pc++] = I_OUT(0, 4, OUT_DST_PINS, 1);
	pio0_hw->instr_mem[pc++] = I_JMP(0, 6, JMP_Y_POSTDEC, lblMoreBits);	//1 cy delay after here no matter if we jumped
	pio0_hw->instr_mem[pc++] = I_JMP(0, 4, JMP_X_POSTDEC, lblPullNgo);

	sm0EndPC = pc - 1;	//that was the last instr
	
	//configure sm0
	pio0_hw->sm[0].clkdiv = (1 << PIO_SM0_CLKDIV_INT_LSB);	//full speed
	pio0_hw->sm[0].execctrl = (pio0_hw->sm[1].execctrl &~ (PIO_SM0_EXECCTRL_WRAP_TOP_BITS | PIO_SM0_EXECCTRL_WRAP_BOTTOM_BITS | PIO_SM2_EXECCTRL_SIDE_EN_BITS)) | (sm0EndPC << PIO_SM0_EXECCTRL_WRAP_TOP_LSB) | (sm0StartPC << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB) | (SIDE_SET_HAS_ENABLE_BIT ? PIO_SM2_EXECCTRL_SIDE_EN_BITS : 0);
	pio0_hw->sm[0].shiftctrl = (pio0_hw->sm[1].shiftctrl &~ (PIO_SM1_SHIFTCTRL_PULL_THRESH_BITS | PIO_SM1_SHIFTCTRL_PUSH_THRESH_BITS | PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_BITS | PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_BITS | PIO_SM0_SHIFTCTRL_AUTOPULL_BITS | PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS));
	pio0_hw->sm[0].pinctrl = (SIDE_SET_BITS_USED << PIO_SM1_PINCTRL_SIDESET_COUNT_LSB) | (1 << PIO_SM1_PINCTRL_OUT_COUNT_LSB) | (TFT_MISO << PIO_SM1_PINCTRL_IN_BASE_LSB) | (TFT_CS << PIO_SM1_PINCTRL_SIDESET_BASE_LSB) | (TFT_MOSI << PIO_SM1_PINCTRL_OUT_BASE_LSB);
	
	//start sm0 & sm2
	pio0_hw->sm[0].instr = I_JMP(0, 0, JMP_ALWAYS, sm0StartPC);
	pio0_hw->ctrl |= (5 << PIO_CTRL_SM_ENABLE_LSB);

	//prime irq 4
	pio0_hw->irq_force = 1 << 4;
	
	//set up dma to send data to SM0
	dma_hw->ch[0].write_addr = (uintptr_t)&pio0_hw->txf[0];
	dma_hw->ch[0].transfer_count = mFramebufBytes / sizeof(uint16_t);
	dma_hw->ch[0].al1_ctrl = (0 << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (1 << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_HALFWORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_INCR_READ_BITS | DMA_CH0_CTRL_TRIG_EN_BITS;
	
	dma_hw->ch[1].read_addr = (uintptr_t)&mFb;
	dma_hw->ch[1].write_addr = (uintptr_t)&dma_hw->ch[0].al3_read_addr_trig;
	dma_hw->ch[1].transfer_count = 1;
	dma_hw->ch[1].ctrl_trig = (0x3f << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) | (1 << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) | (DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) | DMA_CH0_CTRL_TRIG_EN_BITS;
}

static void dispPrvPinsSetup(bool forPio) {		//uses SM0. only safe while SM0 is stopped

	const uint8_t mPinsForDir[] = {TFT_SCLK, TFT_MOSI, TFT_CS}; //first in others out
	uint_fast8_t j;
	
	for (j = 0; j < sizeof(mPinsForDir) / sizeof(*mPinsForDir); j++) {
		
		uint32_t pin = mPinsForDir[j];
		
		//seems that only PIO can set directions when pins are in PIO mode, so do that, one at a time
	
		pio0_hw->sm[0].pinctrl = (pio0_hw->sm[0].pinctrl &~ (PIO_SM1_PINCTRL_SET_BASE_BITS | PIO_SM1_PINCTRL_SET_COUNT_BITS)) | (pin << PIO_SM1_PINCTRL_SET_BASE_LSB) | (1 << PIO_SM1_PINCTRL_SET_COUNT_LSB);
		pio0_hw->sm[0].instr = I_SET(0, 0, SET_DST_PINDIRS, 1);
		pio0_hw->sm[0].instr = I_SET(0, 0, SET_DST_PINS, j >= 2);
		
		iobank0_hw->io[pin].ctrl = (iobank0_hw->io[pin].ctrl &~ IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS) | ((forPio ? IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_PIO0_0 : IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_SIO_0) << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB);
	}
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Adafruit SPIFlash Animated GIF Example");

  // Initialize flash library and check its chip ID.
  if (!flash.begin()) {
    Serial.println("Error, failed to initialize flash chip!");
    while(1);
  }
  Serial.print("Flash chip JEDEC ID: 0x"); Serial.println(flash.getJEDECID(), HEX);

  // First call begin to mount the filesystem.  Check that it returns true
  // to make sure the filesystem was mounted.
  if (!fatfs.begin(&flash)) {
    Serial.println("Failed to mount filesystem!");
    Serial.println("Was CircuitPython loaded on the board first to create the filesystem?");
    while(1);
  }
  Serial.println("Mounted filesystem!");

  if (!root.open(GIFDIRNAME)) {
    Serial.println("Open dir failed");
  }
  while (f.openNext(&root, O_RDONLY)) {
    f.printFileSize(&Serial);
    Serial.write(' ');
    f.printModifyDateTime(&Serial);
    Serial.write(' ');
    f.printName(&Serial);
    if (f.isDir()) {
      // Indicate a directory.
      Serial.write('/');
    }
    Serial.println();
    f.close();
  }
  root.close();
  
  // ext buttons init
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  pinMode(25, OUTPUT);
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_RST, OUTPUT);
  digitalWrite(25, HIGH); // Backlight on

  // TFT reset
  digitalWrite(TFT_RST, HIGH);
  delay(100);
  digitalWrite(TFT_RST, LOW);
  delay(100);
  digitalWrite(TFT_RST, HIGH);
  delay(200);

  digitalWrite(TFT_CS, HIGH); // Deselect
  pinMode(TFT_DC, OUTPUT);
  digitalWrite(TFT_DC, HIGH); // Data mode

  PicoSPI1.configure(TFT_SCLK, TFT_MOSI, 8, TFT_CS, 24000*1000, 0, false);

  displayInit();

  // fill screen with solid color
  digitalWrite(TFT_CS, LOW);
  setAddrWindow(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
  uint16_t initColor = GC9A01A_BLUE;
  for (int i = 0; i < DISPLAY_WIDTH * DISPLAY_HEIGHT; i++)
    writePixels2Blocking((uint16_t*)&initColor, 1);
  digitalWrite(TFT_CS, HIGH);
  delay(3000);

  //stop SMs
	pio0_hw->ctrl &=~ (7 << PIO_CTRL_SM_ENABLE_LSB);
	//reset SMs
	pio0_hw->ctrl = (7 << PIO_CTRL_SM_RESTART_LSB);
  dispPrvPinsSetup(true);
  dispPrvPioProgram16bpp();

  gif.begin(LITTLE_ENDIAN_PIXELS);
}

void loop() {
  char thefilename[80];
  
  if (!root.open(GIFDIRNAME)) {
    Serial.println("Open GIF directory failed");
    while (1);
  }
  while (f.openNext(&root, O_RDONLY)) {
    f.printFileSize(&Serial);
    Serial.write(' ');
    f.printModifyDateTime(&Serial);
    Serial.write(' ');
    f.printName(&Serial);
    if (f.isDir()) {
      // Indicate a directory.
      Serial.write('/');
    }
    Serial.println();
    f.getName(thefilename, sizeof(thefilename)-1);
    f.close();
    
    char fName[10];
    sprintf(fName, "%dbtn.gif", lastPressedBtn);

    if (strstr(thefilename, fName)) {
      // found a gif
      if (gif.open(thefilename, GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)) {
        
        GIFINFO gi;
        Serial.printf("Successfully opened GIF %s; Canvas size = %d x %d\n",  thefilename, gif.getCanvasWidth(), gif.getCanvasHeight());
        
        if (gif.getInfo(&gi)) {
          Serial.printf("frame count: %d\n", gi.iFrameCount);
          Serial.printf("duration: %d ms\n", gi.iDuration);
          Serial.printf("max delay: %d ms\n", gi.iMaxDelay);
          Serial.printf("min delay: %d ms\n", gi.iMinDelay);
        }
        
        while (1) {
          while (gif.playFrame(true, NULL));
          gif.reset();
          int old = lastPressedBtn;
          if (digitalRead(0) == 0) lastPressedBtn = 0;
          if (digitalRead(1) == 0) lastPressedBtn = 1;
          if (digitalRead(2) == 0) lastPressedBtn = 2;
          if (digitalRead(3) == 0) lastPressedBtn = 3;
          if (old != lastPressedBtn) {
            sleepTimer = millis();
            break;
          }
          // two minutes and goes to deep sleep mode
          if ((millis() - sleepTimer) > (1000 * 60 * 2)) {
            // --- to sleep
            digitalWrite(25, LOW); // Backlight off
            // save clock registers
            _scb_orig = scb_hw->scr;
            _en0_orig = clocks_hw->sleep_en0;
            _en1_orig = clocks_hw->sleep_en1;
            sleep_run_from_xosc();
            //sleep_goto_dormant_until_pin(0, false, false);
            sleep_goto_dormant_until_edge_high(0);
            // --- to awake
            rosc_write(&rosc_hw->ctrl, ROSC_CTRL_ENABLE_BITS);
            // restore clock registers
            scb_hw->scr = _scb_orig;
            clocks_hw->sleep_en0 = _en0_orig;
            clocks_hw->sleep_en1 = _en1_orig;
            clocks_init();
            sleepTimer = millis();
            digitalWrite(25, HIGH); // Backlight on
          }
        }
        gif.close();
      } else {
        Serial.printf("Error opening file %s = %d\n", thefilename, gif.getLastError());
      }
    }
  }
  root.close();
}
