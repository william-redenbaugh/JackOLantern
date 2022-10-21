#include <Arduino.h>
#include "OS/OSThreadKernel.h"
#include "OS/OSMutexKernel.h"
#include "SPI.h"
#include "LedMatrix.h"
#include "arduinoFFT.h"

/////////////////// THREAD INIT BEGIN ///////////////////
#define FFT_COMPUTE_STACK_SIZE 1724
uint8_t fft_compute_stack[FFT_COMPUTE_STACK_SIZE];
os_thread_id_t fft_compute_thread_handler;

#define ANALOG_READ_STACK_SIZE 32
uint8_t analog_read_stack[ANALOG_READ_STACK_SIZE];
os_thread_id_t analog_read_thread_handler;

#define MATRIX_DISPLAY_STACK_SIZE 512
uint8_t matrix_display_stack[MATRIX_DISPLAY_STACK_SIZE];
os_thread_id_t matrix_display_thread_handler;
/////////////////// THREAD INIT END ///////////////////


/////////////////// FFT VARS BEGIN ///////////////////

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

#define NUM_SAMPLES 128
// Buffer for getting microphone samples
MutexLock mic_muttx;
uint8_t mic_input[NUM_SAMPLES];

double vReal[NUM_SAMPLES];
double vImag[NUM_SAMPLES];
arduinoFFT fft;

MutexLock data_output_muttx;
uint32_t data_output[NUM_SAMPLES];
/////////////////// FFT VARS END ///////////////////

/////////////////// MATRIX VARS BEGIN ///////////////////
#define NUMBER_OF_DEVICES 4
#define CS_PIN PA_4
LedMatrix ledMatrix = LedMatrix(NUMBER_OF_DEVICES, CS_PIN);
uint8_t data_output_scaled[NUM_SAMPLES];
/////////////////// MATRIX VARS END ///////////////////

static void analog_read_task(void *arg){
  analogReadResolution(2);
  for(;;){
    // Safely gather up samples to be used for fourier transforms
    mic_muttx.lockWaitIndefinite();
    for(int n = 0; n < NUM_SAMPLES; n++){
      mic_input[n] = analogRead(PA_1);
    }
    mic_muttx.unlock();

    os_thread_delay_ms(10);
  }
}

static void fft_compute_task(void *arg){
  // Calculate the exponenet of it all
  uint8_t exponent = fft.Exponent(NUM_SAMPLES);

  for(;;){
    // Get Mic data from mic thread safetly
    mic_muttx.lockWaitIndefinite();
    for(int n = 0; n < NUM_SAMPLES; n++){
      vReal[n] = (double)mic_input[n];
    }
    mic_muttx.unlock();

    // Clear our entry buffer
    memset(vImag, 0, sizeof(vImag));

    fft.Windowing(vReal, NUM_SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    fft.Compute(vReal, vImag, NUM_SAMPLES, FFT_FORWARD);
    fft.ComplexToMagnitude(vReal, vImag, NUM_SAMPLES);

    // Copy data over to be used by LED matrix
    data_output_muttx.lockWaitIndefinite();
    

    for(int n = 0; n < NUM_SAMPLES; n++){
      data_output[n] = int(vReal[n]);
    }
    data_output_muttx.unlock();

    os_thread_delay_ms(10);
  }
}

uint8_t matrix_length[NUMBER_OF_DEVICES * 8];

static void matrix_display_task(void *arg){
  UNUSED(arg);

  // Init ledmatrix
  ledMatrix.init();
  ledMatrix.clear();
  ledMatrix.setIntensity(5);
  memset(matrix_length, 0, sizeof(matrix_length));
  const int multiplier = NUM_SAMPLES / (NUMBER_OF_DEVICES * 8);

  for(;;){
    // Copy and scale the data over from the FFT thread
    data_output_muttx.lockWaitIndefinite();
    for(int n = 0; n < NUM_SAMPLES; n++){
      data_output_scaled[n] = data_output[n];
    }
    data_output_muttx.unlock();

    for(int x = 0; x < (NUMBER_OF_DEVICES * 8); x++){
      int y_len = data_output_scaled[multiplier * x + 2];

      if(y_len > 32)
        y_len = 32; 

      if(y_len > matrix_length[x])
        matrix_length[x] = y_len;

      if(matrix_length[x] > 0){
        matrix_length[x]--;
      }
    }

    // Clear all pixels so we start with an empty tab
    ledMatrix.clear();
    // Compute and setup display for showing the volume level
    for(int x = 0; x < (NUMBER_OF_DEVICES * 8); x++){
      int y_len = matrix_length[x];
      for(int y = 0; y < y_len; y++){
        ledMatrix.setPixel(x, y);
      }
    }

    // Push out data to the display
    ledMatrix.commit();
    os_thread_delay_ms(10);
  }
}

static inline void init_thread(void){
  analog_read_thread_handler = os_add_thread((thread_func_t)analog_read_task, NULL, ANALOG_READ_STACK_SIZE, analog_read_stack);
  fft_compute_thread_handler = os_add_thread((thread_func_t)fft_compute_task, NULL, FFT_COMPUTE_STACK_SIZE, fft_compute_stack);
  matrix_display_thread_handler = os_add_thread((thread_func_t)matrix_display_task, NULL, MATRIX_DISPLAY_STACK_SIZE, matrix_display_stack);
}

void setup() {
  HAL_Init();
  SystemClock_Config();

  os_init();
  init_thread();
}

void loop() {
  os_thread_delay_ms(100);
}