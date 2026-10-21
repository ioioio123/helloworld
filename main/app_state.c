#include "app_state.h"
#include "board_config.h"

app_state_t g_state;
static SemaphoreHandle_t s_lock;

void app_state_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    configASSERT(s_lock);
    g_state.pulse_us = WELD_PULSE_US;
    g_state.temp_c = -273.0f;
}

app_state_t *app_state_lock(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    return &g_state;
}

void app_state_unlock(void)
{
    xSemaphoreGive(s_lock);
}
