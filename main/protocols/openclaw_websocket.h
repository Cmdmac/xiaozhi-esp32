// OpenClawWebsocket.h
#ifndef OPENCLAW_WEBSOCKET_H
#define OPENCLAW_WEBSOCKET_H

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <functional>
#include <vector>
#include <string>
#include <memory>
#include <web_socket.h>

enum class AudioType {
    WAV,
    OGG
};

class OpenClawWebSocket {
public:
    typedef std::function<void(const std::vector<uint8_t>&, AudioType type)> AudioDataCallback;
    
    OpenClawWebSocket();
    ~OpenClawWebSocket();
    
    bool Connect(const std::string& url);
    void Disconnect();
    bool IsConnected() const;
    
    void SetAudioDataCallback(AudioDataCallback callback);
    bool SendText(const std::string& text);
    bool SendBinary(const std::vector<uint8_t>& data);
    
private:
    std::unique_ptr<WebSocket> websocket_;
    EventGroupHandle_t event_group_;
    AudioDataCallback audio_data_callback_;

    bool isReceived_;
    AudioType binaryType_; //0-wav,1-opus
    
};

#endif