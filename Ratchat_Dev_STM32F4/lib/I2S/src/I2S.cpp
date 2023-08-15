#include "I2S.h"
#include <Arduino.h>

I2SClass::I2SClass(SPI_TypeDef *instance) {
    handle.Instance = instance;
}

#pragma message ("Proper library")

#if BITS_PER_SAMPLE == 16
void I2SClass::setBuffer(int16_t *buffer, int bufferSize) {
    this->doubleBuffer16 = buffer;
    this->bufferSize = bufferSize;
}
#else
void I2SClass::setBuffer(int32_t *buffer, int bufferSize) {
    this->doubleBuffer32 = buffer;
    this->bufferSize = bufferSize;
}
#endif

int I2SClass::getBufferSize() {
    return bufferSize;
}

uint8_t I2SClass::begin(i2s_mode_t mode, uint32_t sampleRate, uint8_t bitsPerSample) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2S2 GPIO Configuration
    PB12     ------> I2S2_WS
    PB13     ------> I2S2_CK
    PB15     ------> I2S2_SD
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    //static int16_t static_buffer[2048];
    #if BITS_PER_SAMPLE == 16
    static int16_t static_buffer[2048]; 
    #else
    static int32_t static_buffer[2048]; 
    #endif
    setBuffer(static_buffer, 2048);

    //__HAL_I2S_DISABLE(&handle);

    #ifdef SPI1
    if (handle.Instance == SPI1) __HAL_RCC_SPI1_CLK_ENABLE();
    #endif
    #ifdef SPI2
    if (handle.Instance == SPI2) __HAL_RCC_SPI2_CLK_ENABLE();
    #endif
    #ifdef SPI3
    if (handle.Instance == SPI3) __HAL_RCC_SPI3_CLK_ENABLE();
    #endif
    #ifdef SPI4
    if (handle.Instance == SPI4) __HAL_RCC_SPI4_CLK_ENABLE();
    #endif
    #ifdef SPI5
    if (handle.Instance == SPI5) __HAL_RCC_SPI5_CLK_ENABLE();
    #endif
    #ifdef SPI6
    if (handle.Instance == SPI6) __HAL_RCC_SPI6_CLK_ENABLE();
    #endif

    handle.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;

    if (mode == I2S_PHILIPS_MODE) {
        handle.Init.Standard = I2S_STANDARD_PHILIPS;
    } else if (mode == I2S_LEFT_JUSTIFIED_MODE) {
        handle.Init.Standard = I2S_STANDARD_LSB;
    } else if (mode == I2S_RIGHT_JUSTIFIED_MODE) {
        handle.Init.Standard = I2S_STANDARD_MSB;
    } else if (mode == I2S_PCM_SHORT_MODE){
        handle.Init.Standard = I2S_STANDARD_PCM_SHORT;
    } else if (mode == I2S_PCM_LONG_MODE){
        handle.Init.Standard = I2S_STANDARD_PCM_LONG;
    } else {
        return false;
    }

    handle.Init.Mode = I2S_MODE_MASTER_RX;
    handle.Init.AudioFreq = I2S_AUDIOFREQ_44K;
    handle.Init.CPOL = I2S_CPOL_LOW;
    handle.Init.ClockSource = I2S_CLOCK_PLL;

    #if defined(STM32F3) || defined(STM32F4)
    handle.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
    #endif

    //handle.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
    
    handle.Init.AudioFreq = sampleRate;

    #if BITS_PER_SAMPLE == 16
    handle.Init.DataFormat = I2S_DATAFORMAT_16B;
    #else
    handle.Init.DataFormat = I2S_DATAFORMAT_32B;
    #endif
    
    //DMA INIT SECTION
    __HAL_RCC_DMA1_CLK_ENABLE();

    if(handle.Instance == SPI1){
        dmaHandle.Instance = DMA2_Stream3;
        dmaHandle.Init.Channel = DMA_CHANNEL_0;
    }else if(handle.Instance == SPI2){
        dmaHandle.Instance = DMA1_Stream3;
        dmaHandle.Init.Channel = DMA_CHANNEL_0;
    }else{
        dmaHandle.Instance = DMA1_Stream2;
        dmaHandle.Init.Channel = DMA_CHANNEL_0;
    }
    
    dmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    dmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    dmaHandle.Init.MemInc = DMA_MINC_ENABLE;
    dmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    dmaHandle.Init.MemDataAlignment = DMA_PDATAALIGN_HALFWORD;
    dmaHandle.Init.Mode = DMA_CIRCULAR;
    dmaHandle.Init.Priority = DMA_PRIORITY_HIGH;

    HAL_DMA_DeInit(&dmaHandle);
    HAL_DMA_Init(&dmaHandle);

    __HAL_LINKDMA(&handle,hdmarx,dmaHandle);

    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

    return HAL_I2S_Init(&handle) == HAL_OK;

    
}

I2SClass *i2sDma;

/*
extern "C" {
    void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
        i2sDma->tail += i2sDma->dmaSendSize;

        if ((uint32_t)(i2sDma->head - i2sDma->tail) > i2sDma->dmaSendSize) {
            HAL_I2S_Transmit_DMA(hi2s, (uint16_t*)(i2sDma->doubleBuffer + (i2sDma->tail % i2sDma->bufferSize)), i2sDma->dmaSendSize);
        } else {
            i2sDma->dmaDone = true;
        }

    }

}
*/
#if BITS_PER_SAMPLE == 16
int I2SClass::read() {
   return HAL_I2S_Receive_DMA(&handle, (uint16_t*)(doubleBuffer16), dmaSendSize);
}
void I2SClass::write(int16_t data) {
    while((head + 1) % bufferSize == tail % bufferSize);

    doubleBuffer16[head % bufferSize] = data;
    head++;

    if (dmaDone && (uint32_t)(head - tail) > dmaSendSize) {
        i2sDma = this;
        dmaDone = false;
        HAL_I2S_Transmit_DMA(&handle, (uint16_t*)(doubleBuffer16 + (tail % bufferSize)), dmaSendSize);
    }

}
#else
int I2SClass::read() {
   return HAL_I2S_Receive_DMA(&handle, (uint16_t*)(doubleBuffer32), dmaSendSize);
}
void I2SClass::write(int16_t data) {
    while((head + 1) % bufferSize == tail % bufferSize);

    doubleBuffer32[head % bufferSize] = data;
    head++;

    if (dmaDone && (uint32_t)(head - tail) > dmaSendSize) {
        i2sDma = this;
        dmaDone = false;
        HAL_I2S_Transmit_DMA(&handle, (uint16_t*)(doubleBuffer32 + (tail % bufferSize)), dmaSendSize);
    }

}
#endif


void I2SClass::write(int16_t *data, size_t samples) {
    for(size_t i=0; i<samples; i++) {
        write(data[i]);
    }
}

uint32_t I2SClass::availableForWrite() {
    return bufferSize - (head - tail);
}

uint32_t I2SClass::getDelay() {
    return dmaSendSize;
}