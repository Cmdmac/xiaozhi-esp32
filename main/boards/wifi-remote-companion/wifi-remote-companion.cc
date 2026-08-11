#include "wifi_board.h"
#include "adc_pdm_audio_codec.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include <esp_log.h>

#include "device_state.h"
#include "alarm_manager.h"
#include "codecs/mix_audio_codec.h"
#include "assets/lang_config.h"
#include "audio/demuxer/ogg_demuxer.h"
#include "esp_opus_dec.h"

#include "nvs_flash.h"

// ===== IR 遥控伴侣模块 (学习 + 全协议空调控制) =====
// 注意: heatpump_climate.h 必须先于 IRLearner.h 包含——IRLearner.h 通过
// Arduino 兼容层会定义 LOW/HIGH 宏, 会破坏 heatpump_climate.h 中的枚举声明
#include <remote_transmitter.h>
#include <heatpump_climate.h>
#include <IRLearner.h>

// HeatpumpIRCompat.h 定义的 LOW/HIGH 宏与代码中其它枚举值同名, 取消定义
#undef LOW
#undef HIGH

#ifdef CONFIG_ESP_HI_WEB_CONTROL_ENABLED
#include "esp_hi_web_control.h"
#endif //CONFIG_ESP_HI_WEB_CONTROL_ENABLED

#define TAG "WIFI-REMOTE-COMPANION"

// 学习结果 NVS 持久化
#define IR_NVS_NAMESPACE "ir_keys"
#define IR_NVS_KEY       "learned"
#define IR_NVS_AC_PROTOCOL_KEY "ac_protocol"

using heatpump_ir_tx::RemoteTransmitter;
using heatpump_ir_tx::RemoteTransmitData;
using heatpump_ir_tx::HeatPumpClimate;

// ===== 学习静音派生类: 继承 AdcPdmAudioCodec, 覆写 Read() 实现学习期间麦克风静音 + 模拟回授 =====
// 本地提示音(ogg)播放时扬声器声音会串入麦克风(无 AEC), 若设备处于聆听态, 回授音频会被
// ASR 识别导致模型自问自答; 学习会话期间把读回的麦克风数据置 0, 可彻底切断该回授路径。
// 同时支持"模拟回授": 把设备播放的提示音(如"下一个")直接注入上传通道, 让服务器 ASR
// 识别出提示内容, 模型因此能"听到"设备播报并配合回复(如播报"请按XX键")。
// 放在板卡代码内派生, 不修改基础 codec 实现。
class LearningAdcPdmAudioCodec : public AdcPdmAudioCodec {
private:
    volatile bool input_muted_ = false;

    // 模拟回授数据: 待注入上传通道的 PCM(已重采样到麦克风采样率 16k), 由 StartFeedback 填充
    std::vector<int16_t> feedback_pcm_;
    size_t feedback_pos_ = 0;

protected:
    virtual int Read(int16_t* dest, int samples) override {
        // 1) 有模拟回授数据待注入 → 优先用它代替麦克风数据上传 (注入期间输入处于静音)
        if (feedback_pos_ < feedback_pcm_.size()) {
            size_t n = std::min<size_t>(samples, feedback_pcm_.size() - feedback_pos_);
            memcpy(dest, feedback_pcm_.data() + feedback_pos_, n * sizeof(int16_t));
            feedback_pos_ += n;
            if (n < (size_t)samples) {
                memset(dest + n, 0, ((size_t)samples - n) * sizeof(int16_t));
            }
            return samples;
        }
        if (!input_muted_) {
            return AdcPdmAudioCodec::Read(dest, samples);
        }
        // 2) 静音: 先按真实节奏阻塞读取 (esp_codec_dev_read 等待 ADC 数据, 约 10ms/帧),
        // 再把读到的数据清零返回:
        // 1) 喇叭提示音的回授不会进入 ASR (防自问自答);
        // 2) 维持管线实时流速, 避免零数据以最高速灌入编码队列,
        //    导致 opus_codec 忙转饿死 IDLE 任务触发任务看门狗
        int ret = AdcPdmAudioCodec::Read(dest, samples);
        memset(dest, 0, samples * sizeof(int16_t));
        return ret;
    }

public:
    LearningAdcPdmAudioCodec(int input_sample_rate, int output_sample_rate,
        uint32_t adc_mic_channel, gpio_num_t pdm_speak_p, gpio_num_t pdm_speak_n, gpio_num_t pa_ctl)
        : AdcPdmAudioCodec(input_sample_rate, output_sample_rate, adc_mic_channel,
                           pdm_speak_p, pdm_speak_n, pa_ctl) {}

    void SetInputMuted(bool muted) {
        input_muted_ = muted;
        ESP_LOGI(TAG, "SetInputMuted %d (学习期间麦克风%s采集)", muted, muted ? "静音, 防喇叭回授" : "恢复");
    }

    // 注入模拟回授: pcm 为本地提示音 ogg 解码出的 48kHz int16 数据, frames 为样本数。
    // 内部线性重采样到麦克风采样率(input_sample_rate_, 本板 16k), 注入期间 Read 返回回授数据,
    // 服务器 ASR 会把它识别成提示内容(如"下一个"), 使模型"听到"设备播报
    void StartFeedback(const int16_t* pcm, size_t frames) {
        feedback_pcm_.clear();
        feedback_pos_ = 0;
        if (pcm == nullptr || frames == 0) {
            return;
        }
        const uint32_t in_sr = 48000;
        const uint32_t out_sr = input_sample_rate_;
        if (out_sr == 0) {
            return;
        }
        if (in_sr == out_sr) {
            feedback_pcm_.assign(pcm, pcm + frames);
        } else {
            // 线性插值重采样: out[i] = pcm[i*in/out] 附近按小数部分插值
            size_t out_frames = frames * out_sr / in_sr + 1;
            feedback_pcm_.reserve(out_frames);
            for (size_t i = 0; i < out_frames; i++) {
                uint64_t pos = (uint64_t)i * in_sr;
                size_t idx = (size_t)(pos / out_sr);
                if (idx + 1 >= frames) {
                    feedback_pcm_.push_back(pcm[idx]);
                    continue;
                }
                uint64_t frac = pos % out_sr;
                uint32_t frac1024 = (uint32_t)(frac * 1024 / out_sr);
                int32_t a = pcm[idx];
                int32_t b = pcm[idx + 1];
                feedback_pcm_.push_back((int16_t)((a * (1024 - (int32_t)frac1024) + b * (int32_t)frac1024) >> 10));
            }
        }
        ESP_LOGI(TAG, "StartFeedback: %d frames(48k) -> %d frames(%dHz)", (int)frames,
                 (int)feedback_pcm_.size(), (int)out_sr);
    }
};

class WifiRemoteCompanion : public WifiBoard {
private:
    Button boot_button_;
    Button audio_wake_button_;
    Button move_wake_button_;
    MixAudioCodec* mix_audio_codec_ = nullptr;

    // 红外遥控硬件
    RemoteTransmitter ir_tx_;
    HeatPumpClimate climate_;
    std::string ac_protocol_name_{""};  // 当前空调品牌协议名
    bool ac_protocol_configured_ = false;            // 是否已显式设置过品牌(NVS/set_protocol)
    IRLearner* ir_learner_ = nullptr;
    TaskHandle_t ir_task_ = nullptr;

    // 学习结束时延迟恢复麦克风: 模型对"学习完成"的总结 TTS 仍在喇叭播放时不能恢复,
    // 否则 TTS 会被回授进麦克风被 ASR 识别(自问自答)。先等 UNMUTE_DELAY_MS, 若设备
    // 仍处于 speaking(模型 TTS 还没播完)则继续等待, 直到 TTS 播完才真正恢复麦克风。
    esp_timer_handle_t unmute_timer_ = nullptr;
    static constexpr uint32_t UNMUTE_DELAY_MS = 2500;  // 首轮等待时间, 之后按 speaking 状态续等

    // 捕获按键后由模型语音驱动推进到下一个键: 不再用固定倒计时盲推,
    // 而是检测模型对"下一个"的语音回复(TTS)播完(speaking -> 非 speaking)后再 nextKey()。
    // 状态机: kWaitingCurrent(捕获时模型还在播上一个回复, 先等它播完) ->
    //         kWaitingReply(等模型开始回复"下一个") -> kReplyPlaying(回复播放中) -> 播完推进
    esp_timer_handle_t next_timer_ = nullptr;
    enum class NextAdvanceState { kWaitingCurrent, kWaitingReply, kReplyPlaying };
    NextAdvanceState advance_state_ = NextAdvanceState::kWaitingCurrent;
    int64_t capture_time_us_ = 0;   // 本次捕获时间戳, 用于兜底超时
    int64_t waiting_reply_since_us_ = 0;  // 进入"等待模型回复"的时间戳, 用于超时重发
    static constexpr uint32_t MODEL_CHECK_MS = 300;            // 状态轮询间隔
    static constexpr uint32_t MODEL_REPLY_TIMEOUT_MS = 10000;  // 模型始终没回复时的兜底超时
    static constexpr uint32_t MODEL_RETRY_MS = 3000;           // 模型迟迟没回复时重发"下一个"

    // 学习结束后延迟保存到 NVS 的倒计时 (ir_loop 每 20ms 递减):
    // "学习完成"提示音(约1.7s)刚投递播放, NVS flash 写入会阻塞单核 CPU,
    // 与播放重叠会听到卡顿, 故等提示音播完再保存
    int save_delay_ticks_ = 0;
    static constexpr int SAVE_DELAY_TICKS = 100;  // 100 * 20ms = 2s, 覆盖提示音时长+余量

    // 把"下一个"提示音直接发送到小智服务端音频通道, 服务器 ASR 识别出"下一个",
    // 模型因此"听到"并语音回复引导(如"好的, 请按模式键"); 本地不播放"下一个"人声,
    // 只有 popup 短音效作为按键反馈(在按键捕获时立即播放)
    void SendNextOggToServer() {
        const auto& ogg = Lang::Sounds::OGG_IR_NEXT;
        // 零拷贝直传嵌入资源(静态), 避免模型 TTS 播放期间堆内存紧张时拷贝分配失败崩溃
        Application::GetInstance().SendOggToServer(
            reinterpret_cast<const uint8_t*>(ogg.data()), ogg.size());
    }

    static void UnmuteTimerCallback(void* arg) {
        auto* self = static_cast<WifiRemoteCompanion*>(arg);
        // 模型对"学习完成"的总结 TTS 可能长于 UNMUTE_DELAY_MS: 若此刻仍在播放, 恢复麦克风
        // 会把喇叭声音回授进麦克风被 ASR 识别(自问自答/乱码), 故继续等待 TTS 播完(speaking
        // 结束)再恢复麦克风, 保证学习结束后用户随时说话都能被听到
        if (Application::GetInstance().GetDeviceState() == kDeviceStateSpeaking) {
            esp_timer_start_once(self->unmute_timer_, self->UNMUTE_DELAY_MS * 1000);
            return;
        }
        auto* codec = dynamic_cast<LearningAdcPdmAudioCodec*>(self->GetAudioCodec());
        if (codec) {
            codec->SetInputMuted(false);
        }
    }

    static void NextTimerCallback(void* arg) {
        auto* self = static_cast<WifiRemoteCompanion*>(arg);
        if (!self->ir_learner_) return;
        auto state = Application::GetInstance().GetDeviceState();
        switch (self->advance_state_) {
            case NextAdvanceState::kWaitingCurrent:
                // 打断后旧 TTS 尾巴还在播, 等它播完(回到非 speaking)再等模型对"下一个"的新回复。
                // "下一个"已在按键捕获时立即发出, 这里只负责排空旧音频
                if (state != kDeviceStateSpeaking) {
                    self->advance_state_ = NextAdvanceState::kWaitingReply;
                    self->waiting_reply_since_us_ = esp_timer_get_time();
                }
                break;
            case NextAdvanceState::kWaitingReply:
                // 模型开始回复"下一个"(进入 speaking)
                if (state == kDeviceStateSpeaking) {
                    self->advance_state_ = NextAdvanceState::kReplyPlaying;
                } else if (esp_timer_get_time() - self->waiting_reply_since_us_ > (int64_t)MODEL_RETRY_MS * 1000) {
                    // 模型迟迟没开始回复(ASR 偶发识别失败): 重发"下一个"补救, 并重新计时
                    self->waiting_reply_since_us_ = esp_timer_get_time();
                    ESP_LOGW(TAG, "模型迟迟未回复, 重发'下一个'给服务器");
                    self->SendNextOggToServer();
                }
                break;
            case NextAdvanceState::kReplyPlaying:
                // 模型语音回复播完(回到非 speaking) → 由模型驱动推进下一个键
                if (state != kDeviceStateSpeaking) {
                    self->advance_state_ = NextAdvanceState::kWaitingCurrent;
                    self->ir_learner_->nextKey();
                    return;  // 推进完成, 不再重启 timer
                }
                break;
        }
        // 兜底: 模型始终没回复(如 ASR 识别失败), 超时后强制推进, 防止卡死
        if (esp_timer_get_time() - self->capture_time_us_ > (int64_t)MODEL_REPLY_TIMEOUT_MS * 1000) {
            ESP_LOGW(TAG, "模型未回复, 超时兜底推进到下一个键");
            self->advance_state_ = NextAdvanceState::kWaitingCurrent;
            self->ir_learner_->nextKey();
            return;
        }
        esp_timer_start_once(self->next_timer_, MODEL_CHECK_MS * 1000);
    }

    static void IRLoopTask(void* arg) {
        auto* self = static_cast<WifiRemoteCompanion*>(arg);
        uint32_t idle_ticks = 0;
        bool was_learning = false;
        while (true) {
            if (self->ir_learner_) {
                self->ir_learner_->loop();
            }
            // 学习会话刚结束(isLearning true->false) → 延迟保存学习结果到 NVS:
            // 此刻"学习完成"提示音(约1.7s)刚被投递播放, NVS flash 写入会阻塞单核 CPU,
            // 与播放重叠会听到卡顿, 故等提示音播完再写入
            bool learning = self->ir_learner_ ? self->ir_learner_->isLearning() : false;
            if (was_learning && !learning) {
                self->save_delay_ticks_ = self->SAVE_DELAY_TICKS;
            }
            if (self->save_delay_ticks_ > 0) {
                if (--self->save_delay_ticks_ == 0) {
                    self->SaveIRKeysToNVS();
                }
            }
            was_learning = learning;

            // 无学习/回放且无待执行的延迟保存时, 空闲约 1 秒(50*20ms)后自挂起,
            // 由 EnsureIRLoopTask 唤醒, 避免一直空转; 挂起前留 1 秒余量以消除与
            // MCP 回调(启动学习)之间的竞态
            if (self->ir_learner_ && !learning && self->save_delay_ticks_ == 0) {
                if (++idle_ticks >= 50) {
                    idle_ticks = 0;
                    vTaskSuspend(nullptr);
                }
            } else {
                idle_ticks = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    // 按需创建/唤醒 ir_loop 任务 (开始学习或回放前调用)
    void EnsureIRLoopTask() {
        if (ir_task_ == nullptr) {
            xTaskCreate(IRLoopTask, "ir_loop", 6144, this, 1, &ir_task_);
        } else {
            vTaskResume(ir_task_);
        }
    }

    // ===== 空调品牌协议持久化 =====
    void SaveACProtocolToNVS(const std::string& name) {
        nvs_handle_t h;
        if (nvs_open(IR_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
        esp_err_t err = nvs_set_str(h, IR_NVS_AC_PROTOCOL_KEY, name.c_str());
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "空调品牌协议已保存: %s", name.c_str());
        } else {
            ESP_LOGE(TAG, "保存空调品牌协议失败: %s", esp_err_to_name(err));
        }
    }

    std::string LoadACProtocolFromNVS() {
        nvs_handle_t h;
        if (nvs_open(IR_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
            return "";
        }
        char buf[32] = {0};
        size_t len = sizeof(buf);
        esp_err_t err = nvs_get_str(h, IR_NVS_AC_PROTOCOL_KEY, buf, &len);
        nvs_close(h);
        if (err != ESP_OK) return "";
        return std::string(buf);
    }

    // 清除已保存的空调品牌协议
    void ClearACProtocolFromNVS() {
        nvs_handle_t h;
        if (nvs_open(IR_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
        esp_err_t err = nvs_erase_key(h, IR_NVS_AC_PROTOCOL_KEY);
        if (err == ESP_ERR_NVS_NOT_FOUND) err = ESP_OK;  // 本来就没有, 不算错误
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "空调品牌协议已清除");
        } else {
            ESP_LOGE(TAG, "清除空调品牌协议失败: %s", esp_err_to_name(err));
        }
    }

    // ===== 学习结果 NVS 持久化 =====
    static void PutU16(std::vector<uint8_t>& buf, size_t& off, uint16_t v) {
        buf[off++] = static_cast<uint8_t>(v & 0xFF);
        buf[off++] = static_cast<uint8_t>((v >> 8) & 0xFF);
    }

    static uint16_t GetU16(const std::vector<uint8_t>& buf, size_t& off) {
        uint16_t v = static_cast<uint16_t>(buf[off]) |
                     (static_cast<uint16_t>(buf[off + 1]) << 8);
        off += 2;
        return v;
    }

    // 将已学习按键(仅 isLearned 且有原始码)序列化到 NVS
    void SaveIRKeysToNVS() {
        if (ir_learner_ == nullptr) return;
        auto& keys = ir_learner_->getKeys();
        uint16_t count = 0;
        for (auto& k : keys) {
            if (k.isLearned && !k.rawData.empty()) count++;
        }
        nvs_handle_t h;
        if (nvs_open(IR_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
            ESP_LOGE(TAG, "nvs_open(%s) failed", IR_NVS_NAMESPACE);
            return;
        }
        if (count == 0) {
            nvs_erase_key(h, IR_NVS_KEY);  // 无已学习按键 → 清空
            nvs_commit(h);
            nvs_close(h);
            ESP_LOGI(TAG, "无已学习按键, 已清空 NVS");
            return;
        }
        // 布局: [count u16] 每键: [name_len u16][name][raw_count u16][raw u16...][freq u16]
        size_t total = 2;
        for (auto& k : keys) {
            if (!k.isLearned || k.rawData.empty()) continue;
            total += 2 + k.name.size() + 2 + k.rawData.size() * 2 + 2;
        }
        std::vector<uint8_t> buf(total);
        size_t off = 0;
        PutU16(buf, off, count);
        for (auto& k : keys) {
            if (!k.isLearned || k.rawData.empty()) continue;
            PutU16(buf, off, static_cast<uint16_t>(k.name.size()));
            memcpy(buf.data() + off, k.name.data(), k.name.size());
            off += k.name.size();
            PutU16(buf, off, static_cast<uint16_t>(k.rawData.size()));
            for (auto v : k.rawData) PutU16(buf, off, v);
            PutU16(buf, off, k.frequency);
        }
        esp_err_t err = nvs_set_blob(h, IR_NVS_KEY, buf.data(), total);
        if (err == ESP_OK) err = nvs_commit(h);
        nvs_close(h);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "保存按键到 NVS 失败: %s", esp_err_to_name(err));
            return;
        }
        ESP_LOGI(TAG, "已保存 %d 个按键到 NVS (%d bytes)", count, (int)total);
    }

    // 开机从 NVS 恢复上次学习结果
    void LoadIRKeysFromNVS() {
        if (ir_learner_ == nullptr) return;
        nvs_handle_t h;
        if (nvs_open(IR_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
            return;  // 从未保存过
        }
        size_t len = 0;
        if (nvs_get_blob(h, IR_NVS_KEY, nullptr, &len) != ESP_OK || len < 2) {
            nvs_close(h);
            return;
        }
        std::vector<uint8_t> buf(len);
        if (nvs_get_blob(h, IR_NVS_KEY, buf.data(), &len) != ESP_OK) {
            nvs_close(h);
            return;
        }
        nvs_close(h);

        size_t off = 0;
        uint16_t count = GetU16(buf, off);
        uint16_t loaded = 0;
        for (uint16_t i = 0; i < count && off + 2 <= len; i++) {
            uint16_t name_len = GetU16(buf, off);
            if (off + name_len > len) break;
            std::string name(reinterpret_cast<const char*>(buf.data() + off), name_len);
            off += name_len;
            if (off + 2 > len) break;
            uint16_t raw_count = GetU16(buf, off);
            if (off + static_cast<size_t>(raw_count) * 2 > len) break;
            std::vector<uint16_t> raw(raw_count);
            for (uint16_t j = 0; j < raw_count; j++) raw[j] = GetU16(buf, off);
            if (off + 2 > len) break;
            uint16_t freq = GetU16(buf, off);

            IRRawKey k;
            k.name = name;
            k.rawData = raw;
            k.frequency = freq;
            k.isLearned = true;
            ir_learner_->getKeys().push_back(k);
            loaded++;
        }
        if (loaded > 0) {
            ESP_LOGI(TAG, "已从 NVS 恢复 %d 个按键", loaded);
        }
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto &app = Application::GetInstance();
            // During startup (before connected), pressing BOOT button enters Wi-Fi config mode without reboot
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    void InitializeIR() {
        ESP_LOGI(TAG, "Initialize IR: TX=GPIO%d RX=GPIO%d", IR_TX_GPIO, IR_RX_GPIO);

        esp_err_t err = ir_tx_.begin(IR_TX_GPIO);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "IR transmitter begin failed: %s", esp_err_to_name(err));
        }
        ir_tx_.set_carrier_duty_percent(33);

        // 空调控制: 默认协议 Panasonic DKE(本机空调型号), 可通过 self.ac.set/set_protocol 切换品牌并持久化
        esp_err_t cerr = climate_.begin(&ir_tx_, heatpump_ir_tx::Protocol::PANASONIC_DKE);
        if (cerr != ESP_OK) {
            ESP_LOGE(TAG, "AC climate begin failed: %s", esp_err_to_name(cerr));
        }
        climate_.set_min_temperature(16.0f);
        climate_.set_max_temperature(30.0f);

        // 开机恢复上次保存的空调品牌协议
        std::string saved_proto = LoadACProtocolFromNVS();
        if (!saved_proto.empty()) {
            auto proto = heatpump_ir_tx::protocol_from_string(saved_proto.c_str());
            if (climate_.set_protocol(proto) == ESP_OK) {
                ac_protocol_name_ = heatpump_ir_tx::protocol_to_string(proto);
                ac_protocol_configured_ = true;  // 持久化过的品牌同样视为已配置
                ESP_LOGI(TAG, "空调品牌协议(已保存): %s", ac_protocol_name_.c_str());
            }
        }

        // 红外学习: 接收用 GPIO 中断, 回放共用 RMT 发射器
        ir_learner_ = new IRLearner(IR_RX_GPIO, IR_TX_GPIO);
        ir_learner_->setup();
        ir_learner_->setIRSender(new IRSenderRMT(&ir_tx_));
        // 捕获到按键(非最后键): 把"下一个"提示音直接发送到小智服务端音频通道,
        // 服务器 ASR 识别出"下一个", 模型因此"听到"并语音回复引导(如"好的, 请按模式键");
        // 本地播放 popup 短音效作为按键反馈(确认用户按键已被捕获), 不播放"下一个"人声,
        // 引导用户按哪个键仍由模型语音负责, 不播放 ic_ac 那类"请按XX键"提示音
        ir_learner_->setOnKeyCaptured([this]() {
            // 本地播放 popup 短音效作为按键反馈提示音(立即响应, 确认按键已被捕获)
            Application::GetInstance().PlaySound(Lang::Sounds::OGG_POPUP);

            // 以下为模拟回授方案(已弃用, 保留备查): 解码 ogg 得到 48kHz PCM, 注入 codec 模拟麦克风。
            // 注意: OggDemuxer 回调传入的是 opus 压缩包(每个音频包触发一次回调), 不是 PCM!
            // 需先用 opus 解码器解出 PCM 并累积成完整音频, 再一次性注入上传通道,
            // 否则 ASR 收到的全是压缩数据碎片, 无法识别"下一个"
            // std::vector<int16_t> pcm;
            // void* opus_dec = nullptr;
            // esp_opus_dec_cfg_t dec_cfg = {
            //     .sample_rate = 48000,
            //     .channel = ESP_AUDIO_MONO,
            //     .frame_duration = ESP_OPUS_DEC_FRAME_DURATION_INVALID,
            //     .self_delimited = false,
            // };
            // if (esp_opus_dec_open(&dec_cfg, sizeof(dec_cfg), &opus_dec) == ESP_AUDIO_ERR_OK && opus_dec != nullptr) {
            //     auto demuxer = std::make_unique<OggDemuxer>();
            //     demuxer->OnDemuxerFinished([&pcm, opus_dec](const uint8_t* data, int sample_rate, size_t size) {
            //         std::vector<int16_t> frame(4800);  // 100ms @48k 缓冲, 足够容纳单帧解码输出
            //         esp_audio_dec_in_raw_t raw = {
            //             .buffer = const_cast<uint8_t*>(data),
            //             .len = (uint32_t)size,
            //             .consumed = 0,
            //             .frame_recover = ESP_AUDIO_DEC_RECOVERY_NONE,
            //         };
            //         esp_audio_dec_out_frame_t out = {
            //             .buffer = (uint8_t*)frame.data(),
            //             .len = (uint32_t)(frame.size() * sizeof(int16_t)),
            //             .decoded_size = 0,
            //         };
            //         esp_audio_dec_info_t info = {};
            //         if (esp_opus_dec_decode(opus_dec, &raw, &out, &info) == ESP_AUDIO_ERR_OK) {
            //             size_t n = out.decoded_size / sizeof(int16_t);
            //             pcm.insert(pcm.end(), frame.begin(), frame.begin() + n);
            //         }
            //     });
            //     demuxer->Reset();
            //     demuxer->Process(reinterpret_cast<const uint8_t*>(ogg.data()), ogg.size());
            //     esp_opus_dec_close(opus_dec);
            // }
            // auto* codec = dynamic_cast<LearningAdcPdmAudioCodec*>(GetAudioCodec());
            // if (codec) {
            //     codec->StartFeedback(pcm.data(), pcm.size());
            // }
            // 本地也播放"下一个", 用户听到提示, 与上传同步
            // Application::GetInstance().PlaySound(ogg);

            // 解除模型 TTS 抑制: 让模型对"下一个"的回复(如"好的, 请按模式键")可以播报出来
            Application::GetInstance().SetSuppressNetworkAudio(false);
            // 打断模型当前语音回复: 用户按键即视为打断, 通知服务器停止当前 TTS,
            // 立即直发"下一个", 模型快速回复对下一个键的引导, 无需等旧语音播完
            Application::GetInstance().AbortSpeaking(kAbortReasonNone);
            // 立即发送"下一个"给服务器(ASR 识别出"下一个", 模型语音引导下一个键)
            SendNextOggToServer();
            // 启动"模型语音驱动推进"检查: 每次按键都从捕获时刻重新计时/复位状态机。
            // 打断后设备可能仍在播放旧音频(设备 speaking): 先排空旧 TTS 尾巴(kWaitingCurrent),
            // 再等模型对"下一个"的新回复(kWaitingReply -> kReplyPlaying -> 播完推进)
            capture_time_us_ = esp_timer_get_time();
            if (Application::GetInstance().GetDeviceState() == kDeviceStateSpeaking) {
                advance_state_ = NextAdvanceState::kWaitingCurrent;
            } else {
                advance_state_ = NextAdvanceState::kWaitingReply;
                waiting_reply_since_us_ = esp_timer_get_time();
            }
            if (next_timer_) {
                esp_timer_stop(next_timer_);
                esp_timer_start_once(next_timer_, MODEL_CHECK_MS * 1000);
            }
        });
        // 学习全部完成: 不再本地播放"学习完成"提示音, 直发"下一个"给服务端,
        // 模型收到后语音总结学习结果(如"所有按键已学习完成")
        ir_learner_->setOnLearningCompleted([]() {
            const auto& ogg = Lang::Sounds::OGG_IR_NEXT;
            Application::GetInstance().SendOggToServer(
                reinterpret_cast<const uint8_t*>(ogg.data()), ogg.size());
        });
        // 提示用户按某个键时播放对应按键的语音提示(学习空调时依次播报"电源开/电源关/模式/温度+/温度-")。
        // 提示音为 48kHz, 与 TTS 采样率一致, 不会触发解码器反复重建
        ir_learner_->setOnPromptKey([](const std::string& key) {
            const std::string_view* snd = nullptr;
            if (key == "电源" || key == "电源开" || key == "电源关") snd = &Lang::Sounds::OGG_IR_AC_POWER;
            else if (key == "模式") snd = &Lang::Sounds::OGG_IR_AC_MODE;
            else if (key == "温度+") snd = &Lang::Sounds::OGG_IR_AC_ADD_TEMP;
            else if (key == "温度-") snd = &Lang::Sounds::OGG_IR_AC_SUB_TEMP;
            else if (key == "风速") snd = &Lang::Sounds::OGG_IR_AC_WIND;
            else if (key == "上下扫风") snd = &Lang::Sounds::OGG_IR_AC_SWING_V;
            else if (key == "左右扫风") snd = &Lang::Sounds::OGG_IR_AC_SWING_H;
            else if (key == "定时") snd = &Lang::Sounds::OGG_IR_AC_TIME;
            if (snd != nullptr) {
                // 由模型语音引导用户按哪个键, 本地不再播放"请按XX键"提示音
                // Application::GetInstance().PlaySound(*snd);
            }
        });
        // 学习会话开始/结束时静音/恢复麦克风:
        // 提示音(本地 ogg)播放时扬声器声音会串入麦克风(无 AEC), 若此时设备处于聆听态,
        // 回授音频会被 ASR 识别, 导致模型误以为用户在说话而自行回应; 学习期间静音麦克风可彻底切断。
        // 注意: 学习结束时不能立即恢复——完成提示音(PlaySound 异步)还在喇叭上播放, 恢复过早
        // 会把提示音回授进麦克风被 ASR 识别(日志里 "不们。" 这类乱码); 用定时器延迟到提示音播完
        esp_timer_create_args_t unmute_timer_args = {
            .callback = &WifiRemoteCompanion::UnmuteTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "unmute_timer",
        };
        ESP_ERROR_CHECK(esp_timer_create(&unmute_timer_args, &unmute_timer_));
        esp_timer_create_args_t next_timer_args = {
            .callback = &WifiRemoteCompanion::NextTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "ir_next_timer",
        };
        ESP_ERROR_CHECK(esp_timer_create(&next_timer_args, &next_timer_));
        ir_learner_->setOnLearningStateChanged([this](bool learning) {
            auto* codec = dynamic_cast<LearningAdcPdmAudioCodec*>(GetAudioCodec());
            if (!codec) {
                return;
            }
            // 学习期间不再抑制模型 TTS: 用户引导完全由模型语音承担
            // (learn_start 回复"请按XX键"、对"下一个"的回复都需要播报), 本地不再播放提示音,
            // 不存在与本地提示音重叠的问题, 故保持默认(不抑制)
            // Application::GetInstance().SetSuppressNetworkAudio(learning);
            if (learning) {
                // 新学习会话开始: 立即静音, 并取消可能挂起的恢复/推进定时器
                if (unmute_timer_) {
                    esp_timer_stop(unmute_timer_);
                }
                if (next_timer_) {
                    esp_timer_stop(next_timer_);
                }
                advance_state_ = NextAdvanceState::kWaitingCurrent;
                codec->SetInputMuted(true);
            } else {
                // 学习结束: 延迟恢复麦克风, 等完成提示音播放完毕再解除静音
                if (unmute_timer_) {
                    esp_timer_start_once(unmute_timer_, UNMUTE_DELAY_MS * 1000);
                }
            }
        });

        // 开机从 NVS 恢复上次学习结果 (学习完成后由 ir_loop 任务自动保存)
        LoadIRKeysFromNVS();

        // 开机预置默认学习按键(空调), 保证按键列表始终已初始化;
        // 之后可通过 self.ir.learn_start 的 preset/keys 参数覆盖
        // ir_learner_->addTargetKey("电源");
        // ir_learner_->addTargetKey("模式");
        // ir_learner_->addTargetKey("温度+");
        // ir_learner_->addTargetKey("温度-");
        // ir_learner_->addTargetKey("风速");
        // ir_learner_->addTargetKey("上下扫风");
        // ir_learner_->addTargetKey("左右扫风");
        // ir_learner_->addTargetKey("定时");
        //         ESP_LOGI(TAG, "预置学习按键: 电源/模式/温度+/温度-/风速/上下扫风/左右扫风/定时");
        // 注意: ir_loop 任务不在此创建, 首次学习/回放时才按需启动 (EnsureIRLoopTask)
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();

        // ========== 空调控制 (全协议) ==========
        mcp_server.AddTool("self.ac.set_protocol",
            "设置并记住空调品牌协议(保存到 NVS, 之后 self.ac.set 无需再传 protocol)。重要约束: 只有在用户明确说出空调品牌/型号时才能调用本工具; 学习的时候不用调这个工具设置品牌；禁止猜测、推断或默认空调品牌(如用户只说'打开空调'而未告知品牌, 绝不能调用本工具, 必须先向用户询问'你的空调是什么品牌')。protocol 可选: aux/ballu/carrier_mca/carrier_nqv/daikin_arc417/daikin_arc480/daikin/electroluxyal/fuego/fujitsu/gree/greeyaa/greeyan/greeyac/greeyt/greeyap/hisense_aud/hitachi/hyundai/ivt/midea/mitsubishi_fa/mitsubishi_fd/mitsubishi_fe/mitsubishi_heavy_fdtc/mitsubishi_heavy_zj/mitsubishi_heavy_zm/mitsubishi_heavy_zmp/mitsubishi_kj/mitsubishi_msc/mitsubishi_msy/mitsubishi_sez/panasonic_ckp/panasonic_dke/panasonic_eke/panasonic_jke/panasonic_lke/panasonic_nke/samsung_aqv/samsung_fjm/sharp/toshiba_daiseikai/toshiba/zhlt01/nibe/qlima_1/qlima_2/samsung_aqv12msan/zhjg01/airway/bgh_aud/panasonic_altdke/philco_phs32/vaillantvai8/r51m",
            PropertyList({
                Property("protocol", kPropertyTypeString, std::string(""))
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string protocol = properties["protocol"].value<std::string>();
                if (protocol.empty()) {
                    return "{\"success\": false, \"message\": \"未提供空调品牌 protocol, 请先询问用户空调品牌后再调用本工具\"}";
                }
                auto proto = heatpump_ir_tx::protocol_from_string(protocol.c_str());
                esp_err_t err = climate_.set_protocol(proto);
                if (err != ESP_OK) {
                    char resp[160];
                    snprintf(resp, sizeof(resp), "{\"success\": false, \"message\": \"protocol not supported: %s\"}", protocol.c_str());
                    return std::string(resp);
                }
                ac_protocol_name_ = heatpump_ir_tx::protocol_to_string(proto);
                ac_protocol_configured_ = true;
                SaveACProtocolToNVS(ac_protocol_name_);
                char resp[256];
                snprintf(resp, sizeof(resp), "{\"success\": true, \"message\": \"空调品牌已设为 %s 并保存\"}", ac_protocol_name_.c_str());
                return std::string(resp);
            });

        mcp_server.AddTool("self.ac.set",
            "设置空调状态并通过红外发送控制信号。protocol 可空: 未传 protocol 时优先使用本地学习到的红外码发送(开机/关机→[电源]键, 切换模式→[模式]键, 调高/调低温度→[温度+]/[温度-]键); 本地没有学习码时, 再使用已设置的品牌协议, 若品牌也未设置则提示先调用 self.ac.set_protocol。protocol 也可直接指定品牌。mode: off/cool/heat/auto/fan/dry; 重要: 用户要求'打开/开启空调'时, 必须传开机模式(如 cool/heat/auto/fan), 绝不能因为 self.ac.get 返回的当前状态是 off 就发送 off(off 只在用户明确说'关闭/关机'时才传); temperature: 16-30; fan: auto/low/medium/high; swing: off/horizontal/vertical/both",
            PropertyList({
                Property("protocol", kPropertyTypeString, std::string("")),
                Property("mode", kPropertyTypeString, std::string("cool")),
                Property("temperature", kPropertyTypeInteger, 24, 16, 30),
                Property("fan", kPropertyTypeString, std::string("auto")),
                Property("swing", kPropertyTypeString, std::string("off"))
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string mode = properties["mode"].value<std::string>();
                auto& state = climate_.state();
                int target_temp = properties["temperature"].value<int>();

                // 解析 mode → 协议枚举
                heatpump_ir_tx::ClimateMode want_mode;
                if (mode == "off") {
                    want_mode = heatpump_ir_tx::ClimateMode::OFF;
                } else if (mode == "cool") {
                    want_mode = heatpump_ir_tx::ClimateMode::COOL;
                } else if (mode == "heat") {
                    want_mode = heatpump_ir_tx::ClimateMode::HEAT;
                } else if (mode == "auto") {
                    want_mode = heatpump_ir_tx::ClimateMode::HEAT_COOL;
                } else if (mode == "fan") {
                    want_mode = heatpump_ir_tx::ClimateMode::FAN_ONLY;
                } else if (mode == "dry") {
                    want_mode = heatpump_ir_tx::ClimateMode::DRY;
                } else {
                    return "{\"success\": false, \"message\": \"invalid mode, use off/cool/heat/auto/fan/dry\"}";
                }

                std::string protocol = properties["protocol"].value<std::string>();
                if (!protocol.empty()) {
                    // 指定了品牌 → 切换并记住
                    auto proto = heatpump_ir_tx::protocol_from_string(protocol.c_str());
                    esp_err_t err = climate_.set_protocol(proto);
                    if (err != ESP_OK) {
                        char resp[160];
                        snprintf(resp, sizeof(resp), "{\"success\": false, \"message\": \"protocol not supported: %s\"}", protocol.c_str());
                        return std::string(resp);
                    }
                    ac_protocol_name_ = heatpump_ir_tx::protocol_to_string(proto);
                    ac_protocol_configured_ = true;
                    SaveACProtocolToNVS(ac_protocol_name_);
                } else if (ir_learner_) {
                    // 没传 protocol → 优先使用本地学习码(用户学习过的红外按键)
                    // 指令 → 学习键 映射(按优先级)
                    std::vector<std::string> candidates;
                    if (mode == "off") {
                        candidates.push_back("电源关");  // 关机 → 电源关(分离码)
                        candidates.push_back("电源");    // 兼容旧版单码学习数据(toggle)
                    } else {
                        candidates.push_back("电源开");  // 开机 → 电源开(分离码)
                        candidates.push_back("电源");    // 兼容旧版单码学习数据(toggle)
                        if (state.mode != heatpump_ir_tx::ClimateMode::OFF &&
                            state.mode != want_mode) {
                            candidates.push_back("模式");  // 切换模式 → 模式键
                        }
                        if ((int)state.target_temperature != target_temp) {
                            candidates.push_back(target_temp > (int)state.target_temperature ? "温度+" : "温度-");
                        }
                    }
                    // 按优先级在本地学习键中查找已学习的键
                    int play_index = -1;
                    std::string play_name;
                    auto& keys = ir_learner_->getKeys();
                    for (const auto& c : candidates) {
                        for (size_t i = 0; i < keys.size(); i++) {
                            if (keys[i].isLearned && !keys[i].rawData.empty() &&
                                keys[i].name == c) {
                                play_index = (int)i;
                                play_name = c;
                                break;
                            }
                        }
                        if (play_index >= 0) break;
                    }
                    if (play_index >= 0) {
                        EnsureIRLoopTask();  // 唤醒 ir_loop 任务执行实际发送(空闲后会自挂起)
                        ir_learner_->playKey(play_index);
                        // 记录状态, 供后续温度/模式升降判断
                        state.mode = want_mode;
                        state.target_temperature = static_cast<float>(target_temp);
                        char resp[192];
                        snprintf(resp, sizeof(resp), "{\"success\": true, \"message\": \"使用本地学习码发送[%s]键\"}", play_name.c_str());
                        return std::string(resp);
                    }
                    // 本地没有学习码 → 品牌已设置则用品牌, 否则提示设置品牌
                    if (!ac_protocol_configured_) {
                        return "{\"success\": false, \"message\": \"尚未设置空调品牌，请先调用 self.ac.set_protocol 告知空调品牌(如 gree/panasonic_lke/midea/daikin)\"}";
                    }
                    protocol = ac_protocol_name_;
                } else {
                    // 学习者未初始化 → 品牌已设置则用品牌, 否则提示设置品牌
                    if (!ac_protocol_configured_) {
                        return "{\"success\": false, \"message\": \"尚未设置空调品牌，请先调用 self.ac.set_protocol 告知空调品牌(如 gree/panasonic_lke/midea/daikin)\"}";
                    }
                    protocol = ac_protocol_name_;
                }

                state.mode = want_mode;
                state.target_temperature = static_cast<float>(target_temp);

                std::string fan = properties["fan"].value<std::string>();
                if (fan == "low") {
                    state.fan = heatpump_ir_tx::ClimateFanMode::LOW;
                } else if (fan == "medium") {
                    state.fan = heatpump_ir_tx::ClimateFanMode::MEDIUM;
                } else if (fan == "high") {
                    state.fan = heatpump_ir_tx::ClimateFanMode::HIGH;
                } else {
                    state.fan = heatpump_ir_tx::ClimateFanMode::AUTO;
                }

                std::string swing = properties["swing"].value<std::string>();
                if (swing == "horizontal") {
                    state.swing = heatpump_ir_tx::ClimateSwingMode::HORIZONTAL;
                } else if (swing == "vertical") {
                    state.swing = heatpump_ir_tx::ClimateSwingMode::VERTICAL;
                } else if (swing == "both") {
                    state.swing = heatpump_ir_tx::ClimateSwingMode::BOTH;
                } else {
                    state.swing = heatpump_ir_tx::ClimateSwingMode::OFF;
                }

                esp_err_t err = climate_.transmit_state();
                ESP_LOGI("WIFI-REMOTE-COMPANION", "self.ac.set => protocol=%s mode=%s temp=%d fan=%s swing=%s, transmit=%s",
                         protocol.c_str(), mode.c_str(), properties["temperature"].value<int>(),
                         fan.c_str(), swing.c_str(), esp_err_to_name(err));
                if (err != ESP_OK) {
                    char resp[128];
                    snprintf(resp, sizeof(resp), "{\"success\": false, \"message\": \"transmit failed: %s\"}", esp_err_to_name(err));
                    return std::string(resp);
                }
                char resp[320];
                snprintf(resp, sizeof(resp), "{\"success\": true, \"message\": \"AC sent: protocol=%s mode=%s temp=%d fan=%s swing=%s\"}",
                         protocol.c_str(), mode.c_str(), properties["temperature"].value<int>(), fan.c_str(), swing.c_str());
                return std::string(resp);
            });

        mcp_server.AddTool("self.ac.get",
            "获取当前空调状态和控制来源。返回: protocol(品牌协议名, 未设置时可能为空), mode, temperature, source(控制来源: learned=使用本地学习码, protocol=使用品牌协议), learned_keys(已学习的红外按键列表, 逗号分隔)。"
            "当用户询问'按键是本地学习的还是内置协议的/当前用哪种方式控制'时, 根据 source 和 learned_keys 如实回答",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                auto& s = climate_.state();
                const char* mode = "off";
                switch (s.mode) {
                    case heatpump_ir_tx::ClimateMode::COOL: mode = "cool"; break;
                    case heatpump_ir_tx::ClimateMode::HEAT: mode = "heat"; break;
                    case heatpump_ir_tx::ClimateMode::HEAT_COOL: mode = "auto"; break;
                    case heatpump_ir_tx::ClimateMode::FAN_ONLY: mode = "fan"; break;
                    case heatpump_ir_tx::ClimateMode::DRY: mode = "dry"; break;
                    default: mode = "off"; break;
                }
                // 统计本地学习到的按键
                std::string learned_keys;
                int learned_count = 0;
                const char* source = "protocol";  // 控制来源: 本地学习码 or 品牌协议
                if (ir_learner_) {
                    auto& keys = ir_learner_->getKeys();
                    for (auto& k : keys) {
                        if (k.isLearned && !k.rawData.empty()) {
                            if (learned_count > 0) learned_keys += ",";
                            learned_keys += k.name;
                            learned_count++;
                            // 本地学过空调相关键(电源开/关/模式/温度±) → 控制来源为学习码
                            if (k.name == "电源" || k.name == "电源开" || k.name == "电源关" ||
                                k.name == "模式" || k.name == "温度+" || k.name == "温度-") {
                                source = "learned";
                            }
                        }
                    }
                }
                char resp[512];
                snprintf(resp, sizeof(resp), "{\"success\": true, \"protocol\": \"%s\", \"mode\": \"%s\", \"temperature\": %d, \"source\": \"%s\", \"learned_keys\": \"%s\"}",
                         ac_protocol_name_.c_str(), mode, (int)s.target_temperature, source, learned_keys.c_str());
                return std::string(resp);
            });

        mcp_server.AddTool("self.ac.reset",
            "清除已保存的空调品牌协议和当前空调状态(清空 NVS 中的品牌记录, 释放协议对象)。"
            "调用后 self.ac.set 将拒绝发送直到再次通过 self.ac.set_protocol 设置品牌。"
            "当用户说'清除空调数据/重新设置空调'时应调用本工具",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                ClearACProtocolFromNVS();
                climate_.reset();                     // 释放当前协议对象
                ac_protocol_name_ = "";               // 清空当前品牌
                ac_protocol_configured_ = false;      // 标记未配置
                return "{\"success\": true, \"message\": \"空调品牌协议和状态已清除\"}";
            });

        // ========== 红外学习/回放 ==========
        mcp_server.AddTool("self.ir.learn_start",
            "当用户说'学习空调/学习遥控器/学习按键/开始学习'时, 必须调用本工具启动学习, 否则设备不进入学习状态、用户按遥控器无效。"
            "调用: self.ir.learn_start(type='air_conditioner')。设备侧不播语音提示, 引导用户逐键操作完全由你(模型)负责: "
            "每学完一个键, 设备会发送'下一个'语音信号, 你听到'下一个'后, 必须按本工具返回的按键顺序语音引导用户按下一个键, 如'好的, 请按[模式]键'; "
            "禁止询问'下一个要按什么键', 禁止调用 self.ir.skip(除非用户明确说'跳过/不要这个键'), 不要问品牌/学哪些键。"
            "type: air_conditioner(默认空调, 固定顺序 电源开/电源关/模式/温度+/温度-)。注意: 空调电源键是分离码, "
            "学习[电源开]键前必须提醒用户先确认空调处于关机状态再按电源键; 学习[电源关]键前必须提醒用户先确认空调处于开机状态再按电源键; "
            "tv(电视); custom(自定义, keys 传逗号分隔按键列表)",
            PropertyList({
                Property("type", kPropertyTypeString, std::string("air_conditioner")),
                Property("keys", kPropertyTypeString, std::string(""))
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                if (ir_learner_ == nullptr) {
                    return "{\"success\": false, \"message\": \"IR learner not initialized\"}";
                }
                std::string type = properties["type"].value<std::string>();
                EnsureIRLoopTask();  // 学习需要 ir_loop 任务驱动捕获
                if (type == "tv") {
                    ir_learner_->learnTV();  // 固定顺序: 电源/信号源/音量+/音量-/频道+/频道-/静音/菜单
                    return "{\"success\": true, \"message\": \"好的, 学习顺序: 电源/信号源/音量+/音量-/频道+/频道-/静音/菜单。请用户先按[电源]键\"}";
                }
                if (type == "custom") {
                    // 解析 keys (兼容中英文逗号与空格)
                    ir_learner_->reset();
                    std::string keys = properties["keys"].value<std::string>();
                    for (auto& ch : keys) {
                        if (ch == '，' || ch == ',') ch = ',';
                    }
                    size_t pos = 0;
                    std::string keylist = "";
                    while ((pos = keys.find(',')) != std::string::npos) {
                        std::string key = keys.substr(0, pos);
                        key.erase(0, key.find_first_not_of(" \t\r\n"));
                        key.erase(key.find_last_not_of(" \t\r\n") + 1);
                        if (!key.empty()) {
                            ir_learner_->addTargetKey(key);
                            keylist += (keylist.empty() ? "" : "/") + key;
                        }
                        keys.erase(0, pos + 1);
                    }
                    keys.erase(0, keys.find_first_not_of(" \t\r\n"));
                    keys.erase(keys.find_last_not_of(" \t\r\n") + 1);
                    if (!keys.empty()) {
                        ir_learner_->addTargetKey(keys);
                        keylist += (keylist.empty() ? "" : "/") + keys;
                    }
                    if (ir_learner_->getKeys().empty()) {
                        return "{\"success\": false, \"message\": \"custom 模式需要提供 keys 按键列表\"}";
                    }
                    ir_learner_->startLearning();
                    std::string first_key = ir_learner_->getKeys().front().name;
                    return "{\"success\": true, \"message\": \"好的, 学习顺序: " + keylist + "。请用户先按[" + first_key + "]键\"}";
                }
                // 默认: 空调
                ir_learner_->learnAirConditioner();  // 固定顺序: 电源开/电源关/模式/温度+/温度-
                return "{\"success\": true, \"message\": \"好的, 学习顺序: 电源开/电源关/模式/温度+/温度-。请先确认空调已关机, 然后让用户按[电源开]键\"}";
            });

        mcp_server.AddTool("self.ir.learn_status",
            "查询红外学习状态(可选, 仅当用户询问'学到哪了/学完了吗'时使用)。学习期间设备会自动推进并语音提示, 本工具不负责推进。"
            "返回 learning(是否仍在学习), keys(按键列表), message(当前进度提示)。若 learning=false 表示学习已完成",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                if (ir_learner_ == nullptr) {
                    return "{\"success\": false, \"message\": \"IR learner not initialized\"}";
                }
                std::string msg = ir_learner_->getStatusMessage().c_str();
                auto& keys = ir_learner_->getKeys();
                std::string keylist = "[";
                for (size_t i = 0; i < keys.size(); i++) {
                    char b[64];
                    snprintf(b, sizeof(b), "%s\"%s\"", i ? "," : "", keys[i].name.c_str());
                    keylist += b;
                }
                keylist += "]";
                char resp[512];
                snprintf(resp, sizeof(resp), "{\"success\": true, \"learning\": %s, \"keys\": %s, \"message\": \"%s\"}",
                         ir_learner_->isLearning() ? "true" : "false", keylist.c_str(), msg.c_str());
                return std::string(resp);
            });

        mcp_server.AddTool("self.ir.play",
            "回放已学习的按键。index 为按键序号, 可通过 self.ir.keys 查询。空调学习顺序: 0=电源开, 1=电源关, 2=模式, 3=温度+, 4=温度-。"
            "用户说'打开空调/开机'时回放 index=0(电源开); 说'关闭空调/关机'时回放 index=1(电源关); '调高温度'回放 index=3; '调低温度'回放 index=4",
            PropertyList({
                Property("index", kPropertyTypeInteger, 0, 0, 100)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int index = properties["index"].value<int>();
                auto& keys = ir_learner_->getKeys();
                if (index < 0 || index >= (int)keys.size()) {
                    return "{\"success\": false, \"message\": \"index out of range\"}";
                }
                ir_learner_->playKey(index);
                EnsureIRLoopTask();  // 回放需要 ir_loop 任务执行实际发送
                char resp[256];
                snprintf(resp, sizeof(resp), "{\"success\": true, \"message\": \"playing [%s]\", \"index\": %d}",
                         keys[index].name.c_str(), index);
                return std::string(resp);
            });

        mcp_server.AddTool("self.ir.skip",
            "仅当用户明确说出'跳过/下一个键不要了/不用学这个键'时才调用, 跳过当前正在学习的按键。"
            "注意: 学习过程中设备自动发送的'下一个'信号是正常推进信号, 不是跳过指令, 听到'下一个'时严禁调用本工具",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                ir_learner_->skipCurrentKey();
                return "{\"success\": true, \"message\": \"skipped\"}";
            });

        mcp_server.AddTool("self.ir.reset",
            "清空所有已学习按键并停止学习",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                ir_learner_->reset();
                return "{\"success\": true, \"message\": \"reset\"}";
            });

        mcp_server.AddTool("self.ir.keys",
            "列出所有已学习/待学习的按键",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                auto& keys = ir_learner_->getKeys();
                std::string resp = "{\"success\": true, \"keys\": [";
                for (size_t i = 0; i < keys.size(); i++) {
                    char line[256];
                    snprintf(line, sizeof(line), "{\"index\": %d, \"name\": \"%s\", \"learned\": %s}",
                             (int)i, keys[i].name.c_str(), keys[i].isLearned ? "true" : "false");
                    resp += line;
                    if (i < keys.size() - 1) resp += ",";
                }
                resp += "]}";
                return resp;
            });

        // ========== 闹钟 ==========
        mcp_server.AddTool("self.alarm.add",
            "Add an alarm clock with specified time and optional label.",
            PropertyList({
                Property("hour", kPropertyTypeInteger, 0, 23),// "Hour of the alarm (0-23)"),
                Property("minute", kPropertyTypeInteger, 0, 59), // "Minute of the alarm (0-59)"),
                Property("label", kPropertyTypeString, std::string("")) // "Optional label for the alarm", "")
            }),
            [](const PropertyList& properties) -> ReturnValue {
                int hour = properties["hour"].value<int>();
                int minute = properties["minute"].value<int>();
                std::string label = properties["label"].value<std::string>();

                auto& alarm_manager = AlarmManager::GetInstance();
                int alarm_id = alarm_manager.AddAlarm(hour, minute, label);

                char response[150];
                snprintf(response, sizeof(response), "{\"success\": true, \"message\": \"Alarm set for %02d:%02d\", \"alarm_id\": %d}", hour, minute, alarm_id);
                return std::string(response);
            });

        mcp_server.AddTool("self.alarm.remove",
            "Remove an alarm by ID.",
            PropertyList({
                Property("alarm_id", kPropertyTypeInteger, 1, 100) // "Alarm ID to remove")
            }),
            [](const PropertyList& properties) -> ReturnValue {
                int alarm_id = properties["alarm_id"].value<int>();

                auto& alarm_manager = AlarmManager::GetInstance();
                bool success = alarm_manager.RemoveAlarm(alarm_id);

                if (success) {
                    return "{\"success\": true, \"message\": \"Alarm removed successfully\"}";
                } else {
                    return "{\"success\": false, \"message\": \"Failed to remove alarm\"}";
                }
            });

        mcp_server.AddTool("self.alarm.list",
            "List all configured alarms.",
            PropertyList(),
            [](const PropertyList&) -> ReturnValue {
                auto& alarm_manager = AlarmManager::GetInstance();
                const auto& alarms = alarm_manager.GetAlarms();

                if (alarms.empty()) {
                    return "{\"success\": true, \"message\": \"No alarms set\", \"alarms\": []}";
                }

                std::string response = "{\"success\": true, \"alarms\": [";
                for (size_t i = 0; i < alarms.size(); i++) {
                    const auto& alarm = alarms[i];
                    char line[100];
                    snprintf(line, sizeof(line), "{\"id\": %d, \"hour\": %d, \"minute\": %d, \"label\": \"%s\"}",
                            i + 1, alarm.GetHour(), alarm.GetMinute(), alarm.GetLabel().c_str());
                    response += line;
                    if (i < alarms.size() - 1) {
                        response += ",";
                    }
                }
                response += "]}";

                return response;
            });
    }

public:
    WifiRemoteCompanion() : boot_button_(BOOT_BUTTON_GPIO),
        audio_wake_button_(AUDIO_WAKE_BUTTON_GPIO),
        move_wake_button_(MOVE_WAKE_BUTTON_GPIO)
    {
        InitializeButtons();
        InitializeIR();
        InitializeTools();
    }

    virtual AudioCodec* GetAudioCodec() override
    {
        // if (mix_audio_codec_ == nullptr) {
            static LearningAdcPdmAudioCodec audio_codec(
                AUDIO_INPUT_SAMPLE_RATE,
                AUDIO_OUTPUT_SAMPLE_RATE,
                AUDIO_ADC_MIC_CHANNEL,
                AUDIO_PDM_SPEAK_P_GPIO,
                AUDIO_PDM_SPEAK_N_GPIO,
                AUDIO_PA_CTL_GPIO);
            return &audio_codec;
        //     mix_audio_codec_ = new MixAudioCodec(&audio_codec);
        // }
        // return mix_audio_codec_;
    }

    // 无屏幕板卡：不重写 GetDisplay()，使用基类默认的 NoDisplay
};

DECLARE_BOARD(WifiRemoteCompanion);
