#ifndef ALARM_MANAGER_H
#define ALARM_MANAGER_H

#include <string>
#include <vector>
#include <esp_timer.h>

class Alarm {
public:
    Alarm(int hour, int minute, const std::string& label = "") 
        : hour_(hour), minute_(minute), label_(label), active_(true) {}
    
    int GetHour() const { return hour_; }
    int GetMinute() const { return minute_; }
    const std::string& GetLabel() const { return label_; }
    bool IsActive() const { return active_; }
    
    void SetActive(bool active) { active_ = active; }
    void SetLabel(const std::string& label) { label_ = label; }
    
    // 检查当前时间是否触发闹钟
    bool ShouldTrigger(int current_hour, int current_minute) const;
    
private:
    int hour_;
    int minute_;
    std::string label_;
    bool active_;
};

class AlarmManager {
public:
    static AlarmManager& GetInstance();
    
    // 添加闹钟
    int AddAlarm(int hour, int minute, const std::string& label = "");
    
    // 删除闹钟
    bool RemoveAlarm(int alarm_id);
    
    // 获取所有闹钟
    const std::vector<Alarm>& GetAlarms() const;
    
    // 检查并触发闹钟
    void CheckAlarms();
    
private:
    AlarmManager();
    ~AlarmManager();
    
    std::vector<Alarm> alarms_;
    int next_alarm_id_;
    esp_timer_handle_t check_timer_;
    
    static void CheckAlarmsCallback(void* arg);
};

#endif // ALARM_MANAGER_H
