#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "SPI.h"
#include "LedMatrix.h"

#define NUMBER_OF_DEVICES 4
#define CS_PIN PA_4

LedMatrix ledMatrix = LedMatrix(NUMBER_OF_DEVICES, CS_PIN);
int x = 0;

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
        vTaskDelay(100/portTICK_RATE_MS);
      }
    }
    ledMatrix.clear();
  }
}

void setup() {
  HAL_Init();
  SystemClock_Config();
  
  xTaskCreate(LED_task, NULL , 128, NULL, 2, NULL );
  vTaskStartScheduler();
}

void loop() {

}