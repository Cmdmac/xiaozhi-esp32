#ifndef IR_LEARNER2_H
#define IR_LEARNER2_H

// 基于 IRremoteESP8266 的红外学习+回放
// 接收: GPIO 中断 (不占用 RMT 通道)
// 发送: RMT 硬件调制 (时序精确, 不受 WiFi 中断干扰)
// 与空调控制器共用 RMT 发送器

#include <Arduino.h>
#include <IRrecv.h>
// #include <IRsend.h>
#include <IRutils.h>
#include <vector>
#include <string>
#include <algorithm>
#include <IRSender.h>
#include <remote_transmitter.h>

// RemoteTransmitter/RemoteTransmitData 定义在 heatpump_ir_tx 命名空间中
using heatpump_ir_tx::RemoteTransmitter;
using heatpump_ir_tx::RemoteTransmitData;

const static char* TAG = "IRLearner";

// 原始红外码结构 (mark/space 交替的微秒数组)
struct IRRawKey {
    String name;
    std::vector<uint16_t> rawData;  // mark/space 交替, 单位: 微秒
    uint16_t frequency = 38000;     // 载波频率
    bool isLearned = false;
    // 解码后的协议状态 (优先使用, 时序更精确)
    decode_type_t protocol = decode_type_t::UNKNOWN;
    uint8_t state[32] = {0};        // 解码后的状态字节
    uint16_t stateLen = 0;          // 状态长度
};

// ============ IRSenderRMT: heatpumpir IRSender → RemoteTransmitter 桥接 ============
// IRLearner2 通过 IRSender 接口 (mark/space) 生成码, 这里积累成帧后用 RMT 硬件一次发送
// space(0) 是约定: 帧结束, 立即提交给 RemoteTransmitter
class IRSenderRMT : public IRSender {
public:
    explicit IRSenderRMT(RemoteTransmitter* tx) : IRSender(0), tx_(tx) {}

    void setFrequency(int frequency) override {
        data_.set_carrier_frequency(static_cast<uint32_t>(frequency) * 1000u);
    }

    void mark(int markLength) override {
        data_.mark(static_cast<uint32_t>(markLength));
    }

    void space(int spaceLength) override {
        if (spaceLength != 0) {
            data_.space(static_cast<uint32_t>(spaceLength));
        } else {
            // 帧结束: 一次性提交给 RMT 硬件发送
            if (tx_) {
                esp_err_t err = tx_->send(data_, 1, 0);
                if (err != ESP_OK) {
                    ESP_LOGE("IRSenderRMT", "发送失败: %s", esp_err_to_name(err));
                }
            }
            data_.reset();
        }
    }

private:
    RemoteTransmitter* tx_;
    RemoteTransmitData data_{};
};


class IRLearner {
private:
    IRrecv* receiver_ = nullptr;
    IRSender* irSender_ = nullptr;  // heatpumpir IRSender (IRSenderESP32)
    gpio_num_t rx_pin_;
    gpio_num_t tx_pin_;

    std::vector<IRRawKey> keys_;
    int currentKeyIndex_ = -1;
    bool isSessionActive_ = false;
    bool isSetup_ = false;
    bool irEnabled_ = false;  // 接收器是否已启用 (避免对 NULL timer 调用 disableIRIn)

    // 非阻塞延时状态机: 捕获后等待一段时间再切到下一个键
    bool waitingAfterCapture_ = false;  // 正在等待(让用户松开遥控器)
    uint32_t waitStartMs_ = 0;          // 等待起始时刻
    uint32_t lastDiagMs_ = 0;           // 上次诊断日志时间(学习模式边沿计数)

    // 最近一次状态消息 (供网页查询)
    String lastStatusMsg_ = "空闲";

    // 待发送的按键索引 (Web 任务设置, loop() 中执行, 避免时序干扰)
    volatile int32_t pendingPlayKey_ = -1;

    decode_results results_;

    // 提示用户按键
    void promptUser() {
        if (currentKeyIndex_ >= 0 && currentKeyIndex_ < (int)keys_.size()) {
            lastStatusMsg_ = "请按下遥控器的 [" + keys_[currentKeyIndex_].name + "] 键";
            ESP_LOGI(TAG, "--------------------------------");
            ESP_LOGI(TAG, ">>> 请按下遥控器的 [%s] 键 <<<",
                     keys_[currentKeyIndex_].name.c_str());
            ESP_LOGI(TAG, "--------------------------------");
        } else {
            stopLearning();
        }
    }

    // 停止学习
    void stopLearning() {
        isSessionActive_ = false;
        currentKeyIndex_ = -1;
        waitingAfterCapture_ = false;  // 取消等待状态
        // 学习结束, 关闭接收器, 避免持续捕获噪声
        if (receiver_ && irEnabled_) {
            receiver_->disableIRIn();
            irEnabled_ = false;
        }
        lastStatusMsg_ = "学习完成!";
        ESP_LOGI(TAG, "===========================");
        ESP_LOGI(TAG, "  所有按键学习完成!");
        ESP_LOGI(TAG, "===========================");
    }

public:
    IRLearner(gpio_num_t rx_pin, gpio_num_t tx_pin)
        : rx_pin_(rx_pin), tx_pin_(tx_pin) {}

    ~IRLearner() {
        if (receiver_) {
            if (irEnabled_) receiver_->disableIRIn();
            delete receiver_;
        }
    }

    // 设置 IR 发送器 (heatpumpir IRSenderESP32, 由外部创建)
    void setIRSender(IRSender* sender) {
        irSender_ = sender;
    }

    // 初始化
    void setup() {
        // 创建接收器 (GPIO 中断方式, 不用 RMT)
        // 缓冲区加大到 512 (空调协议很长, 默认100会溢出)
        // 超时加到 90ms (空调帧可达50-100ms, 默认15ms太短)
        receiver_ = new IRrecv(rx_pin_, 512, 90);
        // 发送器由外部 RMT 发送器处理, 不在此创建
        // 注意: 不在此处 enableIRIn(), 避免开机后一直捕获噪声污染缓冲区
        // 接收器只在 startLearning() 时才开启
        isSetup_ = true;
        ESP_LOGI(TAG, "初始化完成 (RX: GPIO%d, TX: GPIO%d, buf:512, timeout:90ms)",
                 rx_pin_, tx_pin_);
    }

    // 添加要学习的按键
    void addTargetKey(String keyName) {
        IRRawKey key;
        key.name = keyName;
        keys_.push_back(key);
    }

    // 开始学习
    void startLearning() {
        if (keys_.empty()) {
            ESP_LOGE(TAG, "错误: 请先添加按键!");
            return;
        }
        if (!isSetup_) setup();
        // 如果接收器之前已启用, 先关闭并清空状态机, 避免缓冲区残留噪声
        // 首次启动时 timer 为 NULL, 不能调用 disableIRIn/resume, 直接 enableIRIn 即可
        if (receiver_ && irEnabled_) {
            receiver_->disableIRIn();
            irEnabled_ = false;
        }
        if (receiver_) {
            receiver_->enableIRIn(true);  // 开启接收并启用内部上拉 (内部会调用 resume)
            irEnabled_ = true;
        }
        isSessionActive_ = true;
        currentKeyIndex_ = 0;
        waitingAfterCapture_ = false;  // 清除等待状态
        lastStatusMsg_ = "开始学习模式...";
        ESP_LOGI(TAG, "开始学习模式...");
        promptUser();
    }

    // 预设: 学习空调
    void learnAirConditioner() {
        reset();
        addTargetKey("电源");
        addTargetKey("模式");
        addTargetKey("温度+");
        addTargetKey("温度-");
        addTargetKey("风速");
        addTargetKey("上下扫风");
        addTargetKey("左右扫风");
        addTargetKey("定时");
        startLearning();
    }

    // 预设: 学习电视
    void learnTV() {
        reset();
        addTargetKey("电源");
        addTargetKey("信号源");
        addTargetKey("音量+");
        addTargetKey("音量-");
        addTargetKey("频道+");
        addTargetKey("频道-");
        addTargetKey("静音");
        addTargetKey("菜单");
        startLearning();
    }

    // 跳过当前按键
    void skipCurrentKey() {
        if (!isSessionActive_) return;
        if (currentKeyIndex_ >= 0 && currentKeyIndex_ < (int)keys_.size()) {
            lastStatusMsg_ = "跳过 [" + keys_[currentKeyIndex_].name + "]";
            ESP_LOGI(TAG, ">> 跳过按键 [%s]",
                     keys_[currentKeyIndex_].name.c_str());
            // 清空可能已捕获的噪声, 避免下一个按键误触发
            if (receiver_) receiver_->resume();
            waitingAfterCapture_ = false;  // 如果正在等待, 取消等待
            currentKeyIndex_++;
            promptUser();
        }
    }

    // 主循环 (在 Arduino loop 中调用)
    void loop() {
        // 先处理待发送的红外指令 (从主循环执行, 避免 Web 任务时序干扰)
        processPendingSend();

        if (!isSessionActive_ || !receiver_) return;

        // 捕获后等待状态: 让用户松开遥控器, 避免连发码被当作下一个键
        // 非阻塞, 期间 BOOT 按钮仍可正常响应
        if (waitingAfterCapture_) {
            if ((xTaskGetTickCount() * portTICK_PERIOD_MS) - waitStartMs_ < 1000) {
                return;  // 还没到 1 秒, 先返回让主 loop 处理其他事
            }
            // 到时间了, 清空等待期间捕获的噪声/连发码
            waitingAfterCapture_ = false;
            receiver_->resume();

            currentKeyIndex_++;
            if (currentKeyIndex_ < (int)keys_.size()) {
                promptUser();
            } else {
                stopLearning();
            }
            return;
        }

        if (receiver_->decode(&results_)) {
            uint16_t rawlen = results_.rawlen;

            // 临时调试: 打印 decode 详情, 排查 rawlen=0 问题
            ESP_LOGW(TAG, "[DBG] decode=true rawlen=%u overflow=%u type=%d bits=%u",
                     rawlen, results_.overflow, (int)results_.decode_type, results_.bits);

            if (rawlen < 5) {
                ESP_LOGW(TAG, "忽略干扰 (rawlen=%d)...", rawlen);
                receiver_->resume();
                return;
            }

            if (results_.overflow) {
                ESP_LOGW(TAG, "警告: 缓冲区溢出! rawlen=%d, 信号被截断, 请增大缓冲区", rawlen);
            }

            ESP_LOGI(TAG, "捕获成功! rawlen: %d%s", rawlen,
                     results_.overflow ? " (溢出!)" : "");

            // 保存原始数据 (rawbuf[0] 忽略, 从 rawbuf[1] 开始)
            // 每个值 * kRawTick(2) = 微秒
            if (currentKeyIndex_ >= 0 && currentKeyIndex_ < (int)keys_.size()) {
                IRRawKey& k = keys_[currentKeyIndex_];
                k.rawData.clear();
                for (uint16_t i = 1; i < rawlen; i++) {
                    k.rawData.push_back(results_.rawbuf[i] * kRawTick);
                }
                k.isLearned = true;

                // 尝试解码为已知协议 (优先用库函数发送, 时序更精确)
                k.protocol = results_.decode_type;
                k.stateLen = results_.bits / 8;
                if (k.stateLen > 0 && k.stateLen <= sizeof(k.state)) {
                    memcpy(k.state, results_.state, k.stateLen);
                    ESP_LOGI(TAG, ">> [%s] 解码: %s (%d 字节)",
                             k.name.c_str(),
                             typeToString(k.protocol).c_str(),
                             k.stateLen);
                } else {
                    ESP_LOGI(TAG, ">> [%s] 未识别协议, 使用原始码回放",
                             k.name.c_str());
                }

                lastStatusMsg_ = "[" + k.name + "] 已保存 (" + std::to_string((int)k.rawData.size()) + " 符号)";
                ESP_LOGI(TAG, ">> [%s] 已保存 (%d 符号)",
                         k.name.c_str(), (int)k.rawData.size());
            }

            receiver_->resume();
            // 进入非阻塞等待状态 (替代原来的 vTaskDelay(1000))
            waitingAfterCapture_ = true;
            waitStartMs_ = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }
    }

    // 发射指定按键 (设置标记, 在 loop() 中执行, 避免 Web 任务时序干扰)
    void playKey(int index) {
        if (index < 0 || index >= (int)keys_.size()) {
            ESP_LOGE(TAG, "索引越界!");
            return;
        }
        IRRawKey& k = keys_[index];
        if (!k.isLearned) {
            ESP_LOGW(TAG, "[%s] 尚未学习, 跳过", k.name.c_str());
            return;
        }
        if (k.rawData.empty()) {
            ESP_LOGE(TAG, "[%s] 数据为空!", k.name.c_str());
            return;
        }
        pendingPlayKey_ = index;  // 标记待发送, loop() 里执行
        ESP_LOGI(TAG, "排队发射 [%s]", k.name.c_str());
    }

    // 在 loop() 中执行实际发送 (用 heatpumpir IRSender 的 mark/space)
    void processPendingSend() {
        if (pendingPlayKey_ < 0) return;
        int index = pendingPlayKey_;
        pendingPlayKey_ = -1;  // 先清除, 防止重入

        IRRawKey& k = keys_[index];

        if (!irSender_) {
            ESP_LOGE(TAG, "IR 发送器未设置!");
            return;
        }
        if (k.rawData.empty()) {
            ESP_LOGE(TAG, "[%s] 数据为空!", k.name.c_str());
            return;
        }

        // 用 heatpumpir IRSender 的 mark/space 发送原始学习码
        irSender_->setFrequency(k.frequency / 1000);
        ESP_LOGI("IRLearner2", "发射 [%s] (%d 符号, %dHz)",
                 k.name.c_str(), (int)k.rawData.size(), k.frequency);
        for (size_t i = 0; i < k.rawData.size(); i++) {
            if (i % 2 == 0) irSender_->mark(k.rawData[i]);
            else irSender_->space(k.rawData[i]);
        }
        // 约定: space(0) 表示帧结束, 桥接发送器在此一次性提交给 RMT 硬件
        irSender_->space(0);
        ESP_LOGI(TAG, ">> [%s] 发送完成", k.name.c_str());
    }

    // 批量回放
    void replayAll() {
        if (keys_.empty()) {
            ESP_LOGW(TAG, "没有按键可回放!");
            return;
        }
        ESP_LOGI(TAG, "--- 批量回放测试 ---");
        for (int i = 0; i < (int)keys_.size(); i++) {
            playKey(i);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        ESP_LOGI(TAG, "--- 回放结束 ---");
    }

    // 重置
    void reset() {
        keys_.clear();
        isSessionActive_ = false;
        currentKeyIndex_ = -1;
        waitingAfterCapture_ = false;
        ESP_LOGI("IRLearner2", "已重置任务列表");
    }

    // 打印所有原始码详细数据
    void printAllRawData() {
        ESP_LOGI(TAG, "==========================================");
        ESP_LOGI(TAG, "所有学习的红外原始码:");
        ESP_LOGI(TAG, "==========================================");

        for (size_t i = 0; i < keys_.size(); i++) {
            IRRawKey& k = keys_[i];
            ESP_LOGI(TAG, "[%d] %s - %s (共 %d 符号, 协议: %s)",
                     (int)i, k.name.c_str(),
                     k.isLearned ? "已学习" : "未学习",
                     (int)k.rawData.size(),
                     k.stateLen > 0 ? typeToString(k.protocol).c_str() : "未知(原始码)");

            if (k.isLearned && !k.rawData.empty()) {
                // 打印详细时序 (前20个)
                for (size_t j = 0; j < std::min(k.rawData.size(), (size_t)20); j++) {
                    const char* type = (j % 2 == 0) ? "MARK" : "SPACE";
                    ESP_LOGI(TAG, "  [%02d] %s: %dus",
                             (int)j, type, k.rawData[j]);
                }
                if (k.rawData.size() > 20) {
                    ESP_LOGI(TAG, "  ... 还有 %d 个符号",
                             (int)k.rawData.size() - 20);
                }
            }
        }
    }

    // 获取按键列表
    std::vector<IRRawKey>& getKeys() { return keys_; }
    const std::vector<IRRawKey>& getKeys() const { return keys_; }

    // 是否正在学习
    bool isLearning() const { return isSessionActive_; }

    // 获取当前学习状态消息 (供网页显示)
    String getStatusMessage() const {
        if (!isSessionActive_) return lastStatusMsg_;
        // 学习中: 附加进度
        int learned = 0;
        for (const auto& k : keys_) if (k.isLearned) learned++;
        return lastStatusMsg_ + " (" + std::to_string(learned) + "/" + std::to_string((int)keys_.size()) + ")";
    }
};

#endif
