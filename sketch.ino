// Include Arduino FreeRTOS library
#include <Arduino_FreeRTOS.h>

#include <semphr.h>




SemaphoreHandle_t r1;
SemaphoreHandle_t r2;
SemaphoreHandle_t r3;

int globalCount = 0;

void setup() {

  Serial.begin(9600);

  r1 = xSemaphoreCreateCounting(1,1);
  r2 = xSemaphoreCreateCounting(1,1);
  r3 = xSemaphoreCreateCounting(1,1);

  /**
     Create tasks
  */
  int parameters1[] = {2500, 500, 500};
  int parameters2[] = {4000, 800};
  int parameters3[] = {3000, 1000};

  xTaskCreate(Task1, // Task function
              "Task1", // Task name for humans
              128, 
              parameters1, // Task parameter
              2, // Task priority
              NULL);
  xTaskCreate(Task2, // Task function
              "Task2", // Task name for humans
              128, 
              parameters2, // Task parameter
              1, // Task priority
              NULL);
  xTaskCreate(Task3, // Task function
              "Task3", // Task name for humans
              128, 
              parameters3, // Task parameter
              1, // Task priority
              NULL);

  vTaskStartScheduler();
}

void loop() {}

void Task1(int *pvParameters)
{
 TickType_t xLastWakeTime;

     
     int deadline = pvParameters[0];
     vTaskDelay(deadline/ portTICK_PERIOD_MS);
     xLastWakeTime = xTaskGetTickCount();

     
     while(1){
        while(xSemaphoreTake(r1, 10) == pdFALSE){}
        //Serial.println(("r1 taken by task 1"));
        Serial.println(("-----TASK 1(1) START-----"));
        vTaskDelay(pvParameters[1] / portTICK_PERIOD_MS);
        Serial.println(("-----TASK 1(1) END-----"));
        while(xSemaphoreTake(r2, 10) == pdFALSE){}
        //Serial.println(("r2 taken by task 1"));
        Serial.println(("-----TASK 1(2) START-----"));
        vTaskDelay(pvParameters[2] / portTICK_PERIOD_MS);
        Serial.println(("-----TASK 1(2) END-----"));
        //Serial.println(("r2 released by task 1"));
        xSemaphoreGive(r2);
        //Serial.println(("r1 released by task 1"));
        xSemaphoreGive(r1);
        int time = (xTaskGetTickCount()-xLastWakeTime)*portTICK_PERIOD_MS;
        
        if(time > deadline){
          Serial.println((String)"DEADLINE ERROR!!! time: "+ time);
          vTaskEndScheduler();
        } else {
          Serial.println((String)"1: "+ time);
        }
        vTaskDelayUntil( &xLastWakeTime, deadline /  portTICK_PERIOD_MS);
     }
 }

void Task2(int *pvParameters){
 TickType_t xLastWakeTime;


     // Initialise the xLastWakeTime variable with the current time.
     
     int deadline = pvParameters[0];
     vTaskDelay(deadline/ portTICK_PERIOD_MS);
     xLastWakeTime = xTaskGetTickCount();

  while(1){
      
      while(xSemaphoreTake(r1, 10) == pdFALSE){}
      //Serial.println(("r1 taken by task 2"));
      while(xSemaphoreTake(r3, 10) == pdFALSE){}
      //Serial.println(("r3 taken by task 2"));
      Serial.println(("-----TASK 2 START-----"));
      vTaskDelay(pvParameters[1] / portTICK_PERIOD_MS);
      Serial.println(("-----TASK 2 END-----"));
      //Serial.println(("r3 released by task 2"));
      xSemaphoreGive(r3);
      //Serial.println(("r1 released by task 2"));
      xSemaphoreGive(r1);
      int time = (xTaskGetTickCount()-xLastWakeTime)*portTICK_PERIOD_MS;
      
      if(time > deadline){
        Serial.println((String)"DEADLINE ERROR!!! time: "+ time);
        vTaskEndScheduler();
      } else {
        Serial.println((String)"2: "+ time);
      }
      vTaskDelayUntil( &xLastWakeTime, deadline /  portTICK_PERIOD_MS);
  }
}

void Task3(int *pvParameters){
 TickType_t xLastWakeTime;

     // Initialise the xLastWakeTime variable with the current time.
     
     int deadline = pvParameters[0];
     vTaskDelay(deadline/ portTICK_PERIOD_MS);
     xLastWakeTime = xTaskGetTickCount();

  while(1){
       while(xSemaphoreTake(r2, 10) == pdFALSE){}
      //Serial.println(("r2 taken by task 3"));
       while(xSemaphoreTake(r3, 10) == pdFALSE){}
      //Serial.println(("r3 taken by task 3"));
      Serial.println(("-----TASK 3 START-----"));
      vTaskDelay(pvParameters[1] / portTICK_PERIOD_MS);
      Serial.println(("-----TASK 3 END-----"));
      //Serial.println(("r3 released by task 3"));
      xSemaphoreGive(r3);
      //Serial.println(("r2 released by task 3"));
      xSemaphoreGive(r2);
      int time = (xTaskGetTickCount()-xLastWakeTime)*portTICK_PERIOD_MS;
        
      if(time > deadline){
        Serial.println((String)"DEADLINE ERROR!!! time: "+ time);
        vTaskEndScheduler();
      } else {
        Serial.println((String)"3: "+ time);
      }
      vTaskDelayUntil( &xLastWakeTime, deadline /  portTICK_PERIOD_MS);
  }
}