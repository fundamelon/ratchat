#include <Arduino.h>
#ifndef _I2S_H
  #define _I2S_H

#define BITS_PER_SAMPLE 16

  typedef enum {
    I2S_PHILIPS_MODE,
    I2S_RIGHT_JUSTIFIED_MODE,
    I2S_LEFT_JUSTIFIED_MODE,
    I2S_PCM_SHORT_MODE,
    I2S_PCM_LONG_MODE
  } i2s_mode_t;

  class I2SClass {
    public:
      I2SClass(SPI_TypeDef *instance);

      /*

      I2SClass(SPI_TypeDef *instance, uint8_t sd, uint8_t ws, uint8_t ck);

      I2SClass(SPI_TypeDef *instance, uint8_t sd, uint8_t ws, uint8_t ck, uint8_t mck);

      */

      uint8_t begin(i2s_mode_t mode, uint32_t sampleRate, uint8_t bitsPerSample);

      #if BITS_PER_SAMPLE == 16
      void setBuffer(int16_t *buffer, int bufferSize);
      int16_t *doubleBuffer16;
      #else
      void setBuffer(int32_t *buffer, int bufferSize);
      int32_t *doubleBuffer32;
      #endif

      int getBufferSize();

      uint32_t getDelay();

      uint32_t availableForWrite();

      int read();

      void write(int16_t data);

      void write(int16_t *data, size_t size);

      /*
      void stm32SetSD(uint8_t sd);
      void stm32SetWS(uint8_t ws);
      void stm32SetCK(uint8_t ck);
      void stm32SetMCK(uint8_t mck);
      */

      I2S_HandleTypeDef handle;

      bool useMck = false;

      GPIO_TypeDef *sdPort = NULL;
      uint32_t sdPin = 0;
      GPIO_TypeDef *wsPort = NULL;
      uint32_t wsPin = 0;
      GPIO_TypeDef *ckPort = NULL;
      uint32_t ckPin = 0;
      GPIO_TypeDef *mckPort = NULL;
      uint32_t mckPin = 0;

      DMA_HandleTypeDef dmaHandle;

      uint32_t bufferSize;
      
      uint16_t dmaSendSize = 64;

      

      volatile uint32_t head = 0, tail = 0;
      volatile bool dmaDone = true;

      volatile bool dataReady = false;
      
  };
#endif
