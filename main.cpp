#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

[[noreturn]] void task(void *args) {
    uint16_t stack_rem = uxTaskGetStackHighWaterMark(nullptr);
    Serial.print("(BEGIN) Remaining stack: ");
    Serial.print(stack_rem);
    Serial.println();

    TickType_t last_tick = xTaskGetTickCount();
    while (true) {
        TickType_t now = xTaskGetTickCount();
        if ((now - last_tick) > 100) {
            stack_rem = uxTaskGetStackHighWaterMark(nullptr);
            Serial.print("(PERIODIC) Remaining stack: ");
            Serial.print(stack_rem);
            Serial.println();

            last_tick = now;
        }
    }
}

void setup() {
    Serial.begin(9600);

    xTaskCreate(task, "Task", 63, nullptr, 0, nullptr);
}

void loop() {
}
