#include <Arduino.h>

// Brochages
#define GPIO_LED_ROUGE      5
#define GPIO_SW1            12

//Création de la sémaphore
SemaphoreHandle_t xBinarySemaphore = NULL;

void IRAM_ATTR interruptSW1()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  Serial.println("interruptSW1()");
  xSemaphoreGiveFromISR( xBinarySemaphore, &xHigherPriorityTaskWoken );
  if( xHigherPriorityTaskWoken != pdFALSE )
    portYIELD_FROM_ISR();
}

void clignoterLedRouge(void *pvParameters)
{
  const char *pcTaskName = "Task clignoterLedRouge";
  UBaseType_t uxPriority;
  uxPriority = uxTaskPriorityGet( NULL );
  for(;;)
  {
    xSemaphoreTake( xBinarySemaphore, portMAX_DELAY );
    Serial.printf("%s - core = %d (priorite %d)\n", pcTaskName, xPortGetCoreID(), uxPriority);
    // provoque un clignotement de la Led
    digitalWrite(GPIO_LED_ROUGE, HIGH);
    vTaskDelay( pdMS_TO_TICKS( 500 ) );
    digitalWrite(GPIO_LED_ROUGE, LOW);
    vTaskDelay( pdMS_TO_TICKS( 500 ) );
  }
}

void vTaskPeriodic( void *pvParameters )
{
  const char *pcTaskName = "Task periodique";
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for( ;; )
  {
    Serial.printf("%s %d\n", pcTaskName, xPortGetCoreID());
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 500 ) );
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Start");

  // Gestion matérielle
  pinMode(GPIO_LED_ROUGE, OUTPUT);
  pinMode(GPIO_SW1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GPIO_SW1), interruptSW1, FALLING);
  digitalWrite(GPIO_LED_ROUGE, LOW);

  xBinarySemaphore = xSemaphoreCreateBinary();

  xTaskCreate( vTaskPeriodic, "vTaskPeriodic", 10000, NULL, 1, NULL );
  xTaskCreate( clignoterLedRouge, "clignoterLedRouge", 10000, NULL, 1, NULL );

  for(;;); // pas de loop()
}

void loop()
{
}

