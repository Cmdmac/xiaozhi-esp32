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

    bool IsOggContainer(const uint8_t* data, size_t size);
    void ParseOggContainer(const uint8_t* data, size_t size);
    void Process8BitAudio(const uint8_t* data, size_t size);


};

#endif // __WEBSOCKET_CODEC_H__