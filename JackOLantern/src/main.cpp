#include <Arduino.h>
#include "OS/OSThreadKernel.h"
#include "SPI.h"
#include "LedMatrix.h"

#define NUMBER_OF_DEVICES 4
#define CS_PIN PA_4

LedMatrix ledMatrix = LedMatrix(NUMBER_OF_DEVICES, CS_PIN);
int x = 0;

// Thread handler ID that we can use to manipulate a thread. 
os_thread_id_t target_thread; 

static void set_pixel(int x, int y){
  int matrix_x = 7-(x % 8); 
  int matrix_y = y % 8; 

  int which_matrix = x / 8;
  ledMatrix.setPixel((7 - matrix_y) + (which_matrix * 8), matrix_x);
}

static void LED_task(void *arg){
  UNUSED(arg);
  ledMatrix.init();
  ledMatrix.clear();
  ledMatrix.setIntensity(5);

  for(;;){
    for(int y = 0; y < 8; y++){
      for(int x = 0; x < 32; x++){
        set_pixel(x, y);
        ledMatrix.commit();
        os_thread_delay_ms(100);
      }
    }
    ledMatrix.clear();
  }
}

void setup() {
  HAL_Init();
  SystemClock_Config();
  
  os_init();

  target_thread = os_add_thread((thread_func_t)LED_task, 0, -1, 0);
}

void loop() {
  os_thread_delay_ms(100);
}