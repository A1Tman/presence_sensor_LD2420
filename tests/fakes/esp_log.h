#pragma once
#include <stdio.h>

#define ESP_LOGI(tag, fmt, ...) do { (void)(tag); printf(fmt "\n", ##__VA_ARGS__); } while (0)
#define ESP_LOGW(tag, fmt, ...) do { (void)(tag); printf(fmt "\n", ##__VA_ARGS__); } while (0)
#define ESP_LOGE(tag, fmt, ...) do { (void)(tag); printf(fmt "\n", ##__VA_ARGS__); } while (0)
#define ESP_LOGD(tag, fmt, ...) do { (void)(tag); printf(fmt "\n", ##__VA_ARGS__); } while (0)
