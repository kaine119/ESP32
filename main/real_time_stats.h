#include "esp_err.h"
#include "freertos/portmacro.h"

#ifndef REAL_TIME_STATS_H
#define ARRAY_SIZE_OFFSET   5   //Increase this if print_real_time_stats returns ESP_ERR_INVALID_SIZE
esp_err_t print_real_time_stats(TickType_t xTicksToWait, char *out);
#endif