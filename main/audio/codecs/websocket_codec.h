#ifndef __WEBSOCKET_CODEC_H__
#define __WEBSOCKET_CODEC_H__

#include <string>
#include <queue>
#include <mutex>
#include <condition_variable>
#include "audio_codec.h"

class WebSocket;
class OpenClawWebSocket;

class WebSocketCodec : public AudioCodec
{
private:
    std::queue<int16_t> audio_buffer_;
    std::mutex buffer_mutex_;
    std::condition_variable buffer_cv_;
    EventGroupHandle_t event_group_;
    static const char* TAG;
    bool available_;

public:
    WebSocketCodec() {
        available_ = false;
        event_group_ = xEventGroupCreate();
        input_sample_rate_ = 16000; // 默认采样率
        output_sample_rate_ = 16000;
        input_channels_ = 1;
        output_channels_ = 1;
    }
    ~WebSocketCodec() {
        vEventGroupDelete(event_group_);
        event_group_ = nullptr;
    }

    bool IsAvailable() const { return audio_buffer_.size() > 0; }
    virtual void OutputData(std::vector<int16_t>& data) override;
    virtual bool InputData(std::vector<int16_t>& data) override;    
    virtual int Read(int16_t* dest, int samples) override;
    virtual int Write(const int16_t* data, int samples) override;

    static bool IsOgg(const uint8_t* data, size_t size);
    static void ParseOgg(std::vector<std::vector<uint8_t>>& result, const uint8_t* data, size_t size);

};

struct OpusHead {
    uint8_t version;           // 版本号
    uint8_t channel_count;     // 通道数
    uint16_t pre_skip;         // 预跳过样本数
    uint32_t sample_rate;      // 输入采样率
    int16_t output_gain;       // 输出增益
    uint8_t mapping_family;    // 通道映射族
};

// ogg_parser.h
class OggParser {
public:
    
    OggParser();
    
    std::vector<std::vector<uint8_t>> Parse(const uint8_t* data, size_t size);
    bool GetOpusHead(OpusHead& head) const;
    int GetSampleRate() const { return sample_rate_; }
    
private:
    bool IsOgg(const uint8_t* data, size_t size);  
    bool ParseOpusHead(const uint8_t* data, size_t size);
    // 寻找下一个 OGG 页头
    bool FindNextPage(const uint8_t* data, size_t size, size_t& offset);
    
    // 处理单个切分好的 Opus 数据包
    void ProcessOpusPacket(const uint8_t* pkt, size_t pkt_len, 
                           std::vector<std::vector<uint8_t>>& result,
                           std::vector<uint8_t>& frame_buffer, 
                           int& local_frame_count);
    
    OpusHead opus_head_;
    int sample_rate_;
    bool seen_head_;
    bool seen_tags_;
    int frame_count_;
    static const char* TAG;
};

#endif // __WEBSOCKET_CODEC_H__