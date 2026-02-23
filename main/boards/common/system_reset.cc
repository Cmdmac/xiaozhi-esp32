#include "system_reset.h"

#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include <esp_partition.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <esp_wifi.h>
#include "ssid_manager.h"

#define TAG "SystemReset"


SystemReset::SystemReset(gpio_num_t reset_nvs_pin, gpio_num_t reset_factory_pin) : reset_nvs_pin_(reset_nvs_pin), reset_factory_pin_(reset_factory_pin) {
    // Configure GPIO1, GPIO2 as INPUT, reset NVS flash if the button is pressed
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << reset_nvs_pin_) | (1ULL << reset_factory_pin_);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
}


void SystemReset::CheckButtons() {
    if (gpio_get_level(reset_factory_pin_) == 0) {
        ESP_LOGI(TAG, "Button is pressed, reset to factory");
        ResetNvsFlash();
        ResetToFactory();
    }

    if (gpio_get_level(reset_nvs_pin_) == 0) {
        ESP_LOGI(TAG, "Button is pressed, reset NVS flash");
        ResetNvsFlash();
    }
}

void SystemReset::ResetWifi() {
    ESP_LOGI(TAG, "Resetting WiFi");

    // Stop WiFi first
    esp_wifi_stop();
    ESP_LOGI(TAG, "WiFi stopped");

    // Clear SSID manager cache
    SsidManager::GetInstance().Clear();
    ESP_LOGI(TAG, "SSID manager cleared");

    // Manually erase WiFi namespace from NVS
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("wifi", NVS_READWRITE, &nvs_handle);
    if (ret == ESP_OK) {
        ret = nvs_erase_all(nvs_handle);
        if (ret == ESP_OK) {
            nvs_commit(nvs_handle);
            ESP_LOGI(TAG, "WiFi namespace erased from NVS");
        } else {
            ESP_LOGE(TAG, "Failed to erase WiFi namespace: %s", esp_err_to_name(ret));
        }
        nvs_close(nvs_handle);
    } else {
        ESP_LOGW(TAG, "WiFi namespace not found in NVS: %s", esp_err_to_name(ret));
    }

    // Also try esp_wifi_restore as a backup
    ret = esp_wifi_restore();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_restore failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "WiFi settings restored to default");
    }
}

void SystemReset::ResetNvsFlash() {
    ESP_LOGI(TAG, "Resetting NVS flash");

    // First stop WiFi and clear WiFi configuration to prevent auto-save
    ResetWifi();

    // Erase all NVS partitions
    esp_err_t ret = nvs_flash_erase();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase NVS flash: %s", esp_err_to_name(ret));
    }

    // Reinitialize NVS
    ret = nvs_flash_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS flash: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "NVS flash reset completed");
}

void SystemReset::ResetToFactory() {
    ESP_LOGI(TAG, "Resetting to factory");

    // First stop WiFi and clear WiFi configuration
    ResetNvsFlash();

    // Erase otadata partition to boot from factory partition
    const esp_partition_t* partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_OTA, NULL);
    if (partition == NULL) {
        ESP_LOGE(TAG, "Failed to find otadata partition");
    } else {
        esp_err_t ret = esp_partition_erase_range(partition, 0, partition->size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to erase otadata partition: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Erased otadata partition");
        }
    }

    // Reboot in 3 seconds
    RestartInSeconds(3);
}

void SystemReset::RestartInSeconds(int seconds) {
    // Keep WiFi stopped during the delay to prevent reconnection
    for (int i = seconds; i > 0; i--) {
        ESP_LOGI(TAG, "Resetting in %d seconds", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    esp_restart();
}
