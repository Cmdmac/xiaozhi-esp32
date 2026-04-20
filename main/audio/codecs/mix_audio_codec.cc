#pragma once
#include "audio_codec.h"
#include "websocket_codec.h"
#include "mix_audio_codec.h"
#include "codecs/no_audio_codec.h"
#include "esp_log.h"

MixAudioCodec::MixAudioCodec(AudioCodec* codec) : device_audio_codec_(codec) {
    WebSocketCodec* websocket_codec = new WebSocketCodec();
    this->websocket_codec_ = websocket_codec;

    duplex_ = device_audio_codec_->duplex();
    input_sample_rate_ = device_audio_codec_->input_sample_rate();
    output_sample_rate_ = device_audio_codec_->output_sample_rate();

    input_reference_ = device_audio_codec_->input_reference();
    input_enabled_ = device_audio_codec_->input_enabled();
    output_enabled_ = device_audio_codec_->output_enabled();
    input_channels_ = device_audio_codec_->input_channels();
    output_channels_ = device_audio_codec_->output_channels();
    output_volume_ = device_audio_codec_->output_volume();
    input_gain_ = device_audio_codec_->input_gain();

    ESP_LOGW("MixAudioCodec", "MixAudioCodec %d,%d", input_sample_rate_, output_sample_rate_);
}

void MixAudioCodec::Start() {
    ESP_LOGW("MixAudioCodec", "Start");
    device_audio_codec_->Start();
}

void MixAudioCodec::OutputData(std::vector<int16_t>& data) {
    device_audio_codec_->OutputData(data);
}

bool MixAudioCodec::InputData(std::vector<int16_t>& data) {
    if (websocket_codec_->IsAvailable()) {
        // ESP_LOGE("MixAudioCodec", "MixAudioCodec InputData %d", data.size());
        bool available = websocket_codec_->InputData(data);
        if (available) {
            return true;
        } 
    }
    return device_audio_codec_->InputData(data);
}

void MixAudioCodec::EnableInput(bool enable) {
    ESP_LOGW("MixAudioCodec", "EnableInput %d", enable);
    device_audio_codec_->EnableInput(enable);
    input_enabled_ = enable;  // 同步状态
}

void MixAudioCodec::EnableOutput(bool enable) {
    ESP_LOGW("MixAudioCodec", "EnableOutput %d", enable);
    device_audio_codec_->EnableOutput(enable);
    output_enabled_ = enable;  // 同步状态
}

int MixAudioCodec::Read(int16_t* dest, int samples) {
    ESP_LOGW("MixAudioCodec", "Read"); 
    return 0;//device_audio_codec_->Read(dest, samples);
}

int MixAudioCodec::Write(const int16_t* data, int samples) {
    ESP_LOGW("MixAudioCodec", "Write");
    return 0;//device_audio_codec_->Write(data, samples);
}

void MixAudioCodec::enableWSInput(bool enable) {
    ws_input_enabled_ = enable;
}

void MixAudioCodec::writeFromWS(const int16_t* data, int samples) {
    if (ws_input_enabled_) {
        websocket_codec_->Write(data, samples);
    }
}