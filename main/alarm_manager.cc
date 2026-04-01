#include "alarm_manager.h"
#include "application.h"
#include "esp_log.h"
#include "assets/lang_config.h"
#include <ctime>

static const char* TAG = "AlarmManager";

AlarmManager::AlarmManager() : next_alarm_id_(1) {
    // 创建定时器，每分钟检查一次闹钟
    esp_timer_create_args_t timer_args = {
        .callback = CheckAlarmsCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "alarm_check_timer",
        .skip_unhandled_events = true
    };
    esp_timer_create(&timer_args, &check_timer_);
    esp_timer_start_periodic(check_timer_, 60 * 1000000ULL); // 每分钟检查一次
    
    ESP_LOGI(TAG, "Alarm manager initialized");
}

AlarmManager::~AlarmManager() {
    if (check_timer_) {
        esp_timer_stop(check_timer_);
        esp_timer_delete(check_timer_);
    }
}

AlarmManager& AlarmManager::GetInstance() {
    static AlarmManager instance;
    return instance;
}

bool Alarm::ShouldTrigger(int current_hour, int current_minute) const {
    return active_ && hour_ == current_hour && minute_ == current_minute;
}

int AlarmManager::AddAlarm(int hour, int minute, const std::string& label) {
    alarms_.emplace_back(hour, minute, label);
    int alarm_id = next_alarm_id_++;
    ESP_LOGI(TAG, "Added alarm: %02d:%02d %s (ID: %d)", hour, minute, label.c_str(), alarm_id);
    return alarm_id;
}

bool AlarmManager::RemoveAlarm(int alarm_id) {
    if (alarm_id > 0 && alarm_id < next_alarm_id_) {
        // 假设 alarm_id 是按顺序分配的，这里简化处理
        // 实际项目中应该使用更可靠的 ID 管理
        if (!alarms_.empty()) {
            alarms_.pop_back();
            ESP_LOGI(TAG, "Removed alarm (ID: %d)", alarm_id);
            return true;
        }
    }
    return false;
}

const std::vector<Alarm>& AlarmManager::GetAlarms() const {
    return alarms_;
}

void AlarmManager::CheckAlarms() {
    // 获取当前时间
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    int current_hour = timeinfo.tm_hour;
    int current_minute = timeinfo.tm_min;
    
    for (const auto& alarm : alarms_) {
        if (alarm.ShouldTrigger(current_hour, current_minute)) {
            ESP_LOGI(TAG, "Alarm triggered: %02d:%02d %s", 
                     alarm.GetHour(), alarm.GetMinute(), alarm.GetLabel().c_str());
            
            // 触发闹钟
            Application::GetInstance().Alert(
                "闹钟", 
                alarm.GetLabel().empty() ? "时间到了" : alarm.GetLabel().c_str(),
                "bell",
                Lang::Sounds::OGG_EXCLAMATION
            );
        }
    }
}

void AlarmManager::CheckAlarmsCallback(void* arg) {
    AlarmManager* manager = static_cast<AlarmManager*>(arg);
    manager->CheckAlarms();
}
