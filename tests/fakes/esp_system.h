#pragma once

#include "esp_err.h"

const char *esp_err_to_name(esp_err_t err);
void esp_restart(void);
