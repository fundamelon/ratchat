#include <Arduino.h>
#include <I2S.h>
#include <TFT_eSPI.h>      

#include <arm_math.h>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

extern "C" void DMA1_Stream3_IRQHandler();

#define SAMPLINGFREQUENCY 150000 //program multiplies this by 32 to get PDM clock frequency
#define NUMBEROFSAMPLES   1024 //64*8

#define BLOCK_SIZE NUMBEROFSAMPLES*BITS_PER_SAMPLE/BITS_PER_SUM
#define N_DATA_CIC_DEC (BLOCK_SIZE/DEC_CIC_FACTOR)
#define DEC_OUT_FACTOR 8 //prev was 10
#define FIR_DELAY 4
#define DEC_CIC_FACTOR 2
//#define BITS_PER_SAMPLE 32
#define CIC_FACTOR 4
#define BITS_PER_SUM 16



/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 150000 Hz

* 0 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -51.579354914507284 dB

* 12000 Hz - 18000 Hz
  gain = 5.011872336272722
  desired ripple = 0.2 dB
  actual ripple = 0.038961815685832235 dB

* 22000 Hz - 28000 Hz
  gain = 2.51188643150958
  desired ripple = 0.2 dB
  actual ripple = 0.0389618156858349 dB

* 32000 Hz - 38000 Hz
  gain = 3.1622776601683795
  desired ripple = 0.2 dB
  actual ripple = 0.03896181568583401 dB

* 42000 Hz - 48000 Hz
  gain = 12.589254117941675
  desired ripple = 0.2 dB
  actual ripple = 0.03896181568583401 dB

* 52000 Hz - 58000 Hz
  gain = 12.589254117941675
  desired ripple = 0.2 dB
  actual ripple = 0.03896181568583401 dB

* 62000 Hz - 68000 Hz
  gain = 11.220184543019634
  desired ripple = 0.2 dB
  actual ripple = 0.03896181568583401 dB

* 72000 Hz - 75000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -51.579354914507284 dB

*/

#define FILTER_TAP_NUM 128

static float32_t filter_taps[FILTER_TAP_NUM] = {
  0.011126878728119793,
  -0.012118105217009201,
  0.0024143782711823665,
  -0.0006711858331227271,
  -0.018490883236594924,
  0.022608965616740664,
  -0.01362283215867087,
  0.005092313655147591,
  0.0063248651330870915,
  0.013974015187137059,
  -0.006213658869134259,
  -0.0023887933199158353,
  0.005164323638645784,
  -0.005430894966819873,
  -0.012129385537041591,
  0.017302760380019835,
  -0.011446355984016688,
  -0.010021188500122267,
  -0.003103795787516363,
  -0.0466425693004489,
  0.07323432801680202,
  -0.05673355905733877,
  0.027369232471599624,
  0.026346630040040026,
  0.049889980886214005,
  -0.02366803905462074,
  -0.005004067391751751,
  0.021022254695251187,
  -0.030727467007164094,
  -0.043847857700005155,
  0.008178906108405323,
  0.022304044976115486,
  -0.06013983475122368,
  -0.0003892658598933706,
  -0.0909685619219162,
  0.18130934062201262,
  -0.1563504623353898,
  0.08115625329071809,
  0.07967836911279019,
  0.13282188653975885,
  -0.06746903882200199,
  -0.007935485604890248,
  0.06643969787302456,
  -0.11462369270728075,
  -0.12710302087399292,
  -0.05254095980046631,
  0.16406507872457687,
  -0.23795191039471755,
  0.03770946715304994,
  -0.16971839156339896,
  0.4586287693189518,
  -0.42813614312608966,
  0.23177986025233913,
  0.2687372222187886,
  0.4094153757336084,
  -0.23623099330787953,
  -0.009839924100199387,
  0.2983491530007684,
  -0.6369501353392565,
  -0.6689802430613156,
  -0.6301496177016839,
  1.916098837340664,
  -3.7835018434693666,
  3.144015238271058,
  3.144015238271058,
  -3.7835018434693666,
  1.916098837340664,
  -0.6301496177016839,
  -0.6689802430613156,
  -0.6369501353392565,
  0.2983491530007684,
  -0.009839924100199387,
  -0.23623099330787953,
  0.4094153757336084,
  0.2687372222187886,
  0.23177986025233913,
  -0.42813614312608966,
  0.4586287693189518,
  -0.16971839156339896,
  0.03770946715304994,
  -0.23795191039471755,
  0.16406507872457687,
  -0.05254095980046631,
  -0.12710302087399292,
  -0.11462369270728075,
  0.06643969787302456,
  -0.007935485604890248,
  -0.06746903882200199,
  0.13282188653975885,
  0.07967836911279019,
  0.08115625329071809,
  -0.1563504623353898,
  0.18130934062201262,
  -0.0909685619219162,
  -0.0003892658598933706,
  -0.06013983475122368,
  0.022304044976115486,
  0.008178906108405323,
  -0.043847857700005155,
  -0.030727467007164094,
  0.021022254695251187,
  -0.005004067391751751,
  -0.02366803905462074,
  0.049889980886214005,
  0.026346630040040026,
  0.027369232471599624,
  -0.05673355905733877,
  0.07323432801680202,
  -0.0466425693004489,
  -0.003103795787516363,
  -0.010021188500122267,
  -0.011446355984016688,
  0.017302760380019835,
  -0.012129385537041591,
  -0.005430894966819873,
  0.005164323638645784,
  -0.0023887933199158353,
  -0.006213658869134259,
  0.013974015187137059,
  0.0063248651330870915,
  0.005092313655147591,
  -0.01362283215867087,
  0.022608965616740664,
  -0.018490883236594924,
  -0.0006711858331227271,
  0.0024143782711823665,
  -0.012118105217009201,
  0.011126878728119793
};

float32_t firStateArr[FILTER_TAP_NUM + BLOCK_SIZE - 1] = {0};


I2SClass I2S(SPI2);
TFT_eSPI tft = TFT_eSPI();         // Invoke custom library
arm_rfft_fast_instance_f32 fft_handler;
arm_fir_decimate_instance_f32 fir_handler;

#if BITS_PER_SAMPLE == 16
  int16_t Buffer[NUMBEROFSAMPLES];
  int8_t dblBuffer[NUMBEROFSAMPLES*2];
#elif BITS_PER_SAMPLE == 32
  int32_t Buffer[NUMBEROFSAMPLES];
#endif

float32_t preCIC[BLOCK_SIZE];
float32_t postCIC[BLOCK_SIZE];
float32_t fftIn[BLOCK_SIZE/DEC_CIC_FACTOR];
float32_t fftOut[BLOCK_SIZE/DEC_CIC_FACTOR];
float32_t postFIR[BLOCK_SIZE/DEC_CIC_FACTOR];

int16_t sineWave[BLOCK_SIZE];

uint16_t max_color_r;
uint16_t max_color_g;
uint16_t max_color_b;

uint16_t min_color_r;
uint16_t min_color_g;
uint16_t min_color_b;

volatile bool dmaDataReady = false;

#define MAX_COLOR 0xFFFF
#define MIN_COLOR 0x0000
#define ILI9341_VSCRDEF           0x33 // Vertical Scrolling Definition
#define ILI9341_VSCRSADD          0x37 // Vertical Scrolling Start Address
#define TEXT_HEIGHT 8 // Height of text to be printed and scrolled
#define SCREEN_VERTICAL_RESOLUTION 240
#define SCREEN_HORIZONTAL_RESOLUTION 320
#define BOT_FIXED_AREA 0 // Number of lines in bottom fixed area (lines counted from bottom of screen)
#define TOP_FIXED_AREA 16 // Number of lines in top fixed area (lines counted from top of screen)
#define YMAX 320 // Bottom of screen area

// The initial y coordinate of the top of the scrolling area
uint16_t yStart = TOP_FIXED_AREA;
// yArea must be a integral multiple of TEXT_HEIGHT
const uint16_t yArea = YMAX-TOP_FIXED_AREA-BOT_FIXED_AREA;
// The initial y coordinate of the top of the bottom text line
const uint16_t yDraw = YMAX - BOT_FIXED_AREA - TEXT_HEIGHT;
volatile uint16_t xPos;
volatile float32_t max_convert_value = 10.0;
//long drawAdjust = 10000/(SAMPLINGFREQUENCY/(NUMBEROFSAMPLES/2));
long drawAdjust = 3;
bool screenFrozen = false;
volatile long lastPressMillis = 0;
/*
uint16_t colorInterp(uint32_t val){
  //float newVal = 20*log10(val+1);
  val = (val < 3) ? 0 : val;
  max_convert_value = (val > max_convert_value) ? val : max_convert_value;
  int newVal = 63.0 / max_convert_value * val;
  newVal = (newVal < 0) ? 0 : newVal;
  newVal = (newVal > 63) ? 63 : newVal;
  uint16_t hex_r = (newVal & 0xF800);
  hex_r = 0;
  //uint16_t hex_g = (newVal & 0x07E0);
  //hex_g = 0;
  //uint16_t hex_b = (newVal & 0x001F);
  uint16_t hex_b = 0;
  uint16_t hex_g = newVal;
  
  uint16_t hex_r = byte(((max_color_r - min_color_r) * frac + min_color_r))<<11;
  uint16_t hex_g = byte(((max_color_g - min_color_g) * frac + min_color_g))<<5;
  uint16_t hex_b = byte(((max_color_b - min_color_b) * frac + min_color_b));
  
  return (hex_r + hex_g<<5 + hex_b);
  //return (color << 11) || (color << 5) || color;
}
*/

void generateSineWave(int freq){
  float32_t period = (1.0/(float)SAMPLINGFREQUENCY);
  for(int i = 0; i < BLOCK_SIZE; i++){
    sineWave[i] = (int16_t)4*arm_sin_f32(2*PI*freq*period*i);
    //Serial.println(sineWave[i]);
  }
}

uint16_t colorInterp(float32_t val){
  uint16_t hex_g = 0;
  max_convert_value = (val > max_convert_value) ? val : max_convert_value;
  float32_t float_g = (63.0/max_convert_value)*val;
  if(float_g > 63.0){
    hex_g = 63;
  }else{
    hex_g = (uint16_t)float_g;
  }
  return hex_g<<5;
}

int freqYPos(int freq){
  return freq/(SAMPLINGFREQUENCY/(NUMBEROFSAMPLES/2));
}

void placeLinePixel(uint16_t xPos_, uint16_t yPos, uint32_t signalMagnitude) {
  uint16_t colorMap;
  colorMap = colorInterp(signalMagnitude);
  tft.drawPixel(xPos_, yPos, colorMap);
}

void setupScrollArea(uint16_t tfa, uint16_t bfa) {
  tft.writecommand(ILI9341_VSCRDEF); // Vertical scroll definition
  tft.writedata(tfa >> 8);           // Top Fixed Area line count
  tft.writedata(tfa);
  tft.writedata((YMAX-tfa-bfa)>>8);  // Vertical Scrolling Area line count
  tft.writedata(YMAX-tfa-bfa);
  tft.writedata(bfa >> 8);           // Bottom Fixed Area line count
  tft.writedata(bfa);
}

void scrollAddress(uint16_t VSP) {
  tft.writecommand(ILI9341_VSCRSADD); // Vertical scrolling start address
  tft.writedata(VSP >> 8);
  tft.writedata(VSP);
}

int scroll_line(int lines) {
  int yTemp = yStart;
  //yStart+=TEXT_HEIGHT;
  for (int i = 0; i < lines; i++) {
    yStart++;
    if (yStart >= YMAX - BOT_FIXED_AREA) yStart = TOP_FIXED_AREA + (yStart - YMAX + BOT_FIXED_AREA);
    scrollAddress(yStart);
  }
  return  yTemp;
}

void ISR_userButton_Pressed(void){
  long interruptMillis = millis();
  if(interruptMillis - lastPressMillis > 100){
    screenFrozen = !screenFrozen;
  }
  lastPressMillis = interruptMillis;
}

int32_t op, res, one;
float32_t lastInSamples[CIC_FACTOR] = {0};
float32_t lastOut = 0;
uint32_t distToEnd = 0;

float complexABS(float32_t real, float32_t imag){
  return sqrtf((real*real+imag*imag));
}

void CIC_filter(float32_t * inArr, float32_t * outArr, uint16_t index){
  if(index > CIC_FACTOR){
    outArr[index] = (1.0/(float)CIC_FACTOR)*(inArr[index]-inArr[index-CIC_FACTOR])+outArr[index-1];
  }
  else if(index == 0){
    outArr[index] = (1.0/(float)CIC_FACTOR)*(inArr[index]-lastInSamples[0])+lastOut;
  }
  else{
    outArr[index] = (1.0/(float)CIC_FACTOR)*(inArr[index]-lastInSamples[index])+outArr[index-1];
  }
  distToEnd = (NUMBEROFSAMPLES-1)-index;
  if(distToEnd<CIC_FACTOR){
    for(int i = 0; i < CIC_FACTOR; i++){
      if(i != CIC_FACTOR-1){
        lastInSamples[i] = lastInSamples[i+1];
      }
      else{
        lastInSamples[CIC_FACTOR-1] = inArr[index];
      }
    }
    if(distToEnd == 0){
      lastOut = outArr[index];
    }
  }
}

extern "C" void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
  if(dmaDataReady == false){
     dmaDataReady = true;
  }
  //Serial.println(1);
}

extern "C" void DMA1_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&I2S.dmaHandle);
}

float shift = (float)(BITS_PER_SAMPLE*DEC_CIC_FACTOR/2);
//#define DEBUG_SERIAL

void setup()
{
  Serial.begin(115200);
  Serial.println("Begin");

  I2S.dmaSendSize=NUMBEROFSAMPLES;

  generateSineWave(20000);
  //while(1);

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  setupScrollArea(TOP_FIXED_AREA, BOT_FIXED_AREA);
  int ten_KHZ_Line = 10000/(SAMPLINGFREQUENCY/(NUMBEROFSAMPLES/2));
  tft.drawLine(0,ten_KHZ_Line-drawAdjust,TOP_FIXED_AREA-1,ten_KHZ_Line-drawAdjust,TFT_WHITE);
  tft.drawLine(0,freqYPos(20000)-drawAdjust,TOP_FIXED_AREA-1,freqYPos(20000)-drawAdjust,TFT_WHITE);
  tft.drawLine(0,freqYPos(40000)-drawAdjust,TOP_FIXED_AREA-1,freqYPos(40000)-drawAdjust,TFT_WHITE);
  tft.drawLine(0,freqYPos(60000)-drawAdjust,TOP_FIXED_AREA-1,freqYPos(60000)-drawAdjust,TFT_WHITE);

  max_color_r = MAX_COLOR >> 11;
  max_color_g = MAX_COLOR >> 5;
  max_color_b = MAX_COLOR & 0x001F;
  min_color_r = MIN_COLOR >> 11;
  min_color_g = MIN_COLOR >> 5;
  min_color_b = MIN_COLOR & 0x001F;

  Serial.println(I2S.begin(I2S_LEFT_JUSTIFIED_MODE, SAMPLINGFREQUENCY, BITS_PER_SAMPLE));
  I2S.setBuffer(Buffer,NUMBEROFSAMPLES);
  delay(1000);
  Serial.println("Init OK");
  Serial.println(I2S.read());
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  xPos = 16;

  arm_fir_decimate_init_f32(&fir_handler,FILTER_TAP_NUM,DEC_CIC_FACTOR,filter_taps,firStateArr,BLOCK_SIZE);
  arm_rfft_fast_init_f32(&fft_handler,BLOCK_SIZE/DEC_CIC_FACTOR);

  attachInterrupt(digitalPinToInterrupt(USER_BTN),ISR_userButton_Pressed,LOW);
}

uint16_t bitCnt = 0;
float32_t preDecimSum = 0;
String data = "";
unsigned long decayMillis = 0;

#define USE_SINE_WAVE

void loop()
{
  if(!screenFrozen){
    if(dmaDataReady){
      digitalWrite(LED_BUILTIN,HIGH);
      bitCnt = 0;
      for(int i = 0; i<NUMBEROFSAMPLES;i++){
        for (int j = 0;  j < BITS_PER_SAMPLE/BITS_PER_SUM;  ++j){
          #ifdef USE_SINE_WAVE
          for(int k = 0; k < BITS_PER_SUM; k++){
            if((sineWave[i] & (1 << (BITS_PER_SUM*j+k)))!=0){
                bitCnt += 1; // add finer bit counting here -> FIR filter on every nth bit (1st, 4th, etc)
            }
          }
          #else
          for(int k = 0; k < BITS_PER_SUM; k++){
            if((Buffer[i] & (1 << (BITS_PER_SUM*j+k)))!=0){
                bitCnt += 1; // add finer bit counting here -> FIR filter on every nth bit (1st, 4th, etc)
            }
          }
          #endif
          bitCnt += rand() % 2;
          postCIC[BITS_PER_SAMPLE/BITS_PER_SUM*i+j] = bitCnt;
          //Serial.println(String(bitCnt));
          bitCnt = 0;
        }
        //preCIC[i] = bitCnt;
        //CIC_filter(preCIC,postCIC,i);
        //postCIC[i] = preCIC[i];
        //preDecimSum = preDecimSum + postCIC[i];
        //Serial.print(bitCnt);
        //Serial.print("\t");
        //Serial.print(postCIC[i]);
        //Serial.println();
        
      }

      arm_fir_decimate_f32(&fir_handler,postCIC,postFIR,BLOCK_SIZE);
      for(int j = 0; j < BLOCK_SIZE/DEC_CIC_FACTOR; j++){
        if(j != 0){
          fftIn[j] = postFIR[j]-shift;
        }else{
          fftIn[j] = 0;
        }
        //Serial.println(postFIR[j]);
      }
      arm_rfft_fast_f32(&fft_handler,fftIn,fftOut,0);

      int mag;
      
      //ADD SCREEN STUFF HERE
      for (int i = drawAdjust; i < BLOCK_SIZE/DEC_CIC_FACTOR; i += 2) {
        mag = complexABS(fftOut[i],fftOut[i+1]);
        if(mag < 50){
          mag = 0;
        }
        //if(i/2-drawAdjust < SCREEN_VERTICAL_RESOLUTION){
        //  placeLinePixel(xPos,i/2-drawAdjust,mag);
        //}
        placeLinePixel(xPos,i/2-drawAdjust,mag);

        //data = data + "\t" + String(mag);
        
      }

      xPos = xPos + 1;
      if(xPos>SCREEN_HORIZONTAL_RESOLUTION){
        xPos = yStart;
      }
      scroll_line(1);

      if(millis() - decayMillis > 100){
        decayMillis = millis();
        if(max_convert_value > 1000){
          max_convert_value = max_convert_value - 200;
        }
        if(max_convert_value > 500){
          max_convert_value = max_convert_value - 100;
        }
        else if(max_convert_value > 200){
          //Serial.println(max_convert_value);
          max_convert_value = max_convert_value - 20; //decay max val
        }
        else if(max_convert_value > 4){
          max_convert_value = max_convert_value - 2;
        }
        //Serial.println(max_convert_value);
      }



      //Serial.println(data);



      data = "";
      dmaDataReady = false;
      I2S.read();
      
    }
    else{
      digitalWrite(LED_BUILTIN,LOW);
    }
  }
  delay(1);
}
