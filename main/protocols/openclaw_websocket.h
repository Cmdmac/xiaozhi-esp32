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
    WAV_STREAM,
    WAV_FILE,
    OGG_STREAM,
    OGG_FILE,
};

struct OpenClawWebSocketCallbacks {
    std::function<void(const std::vector<uint8_t>&, AudioType audioType)> on_audio_data_callback;
    std::function<void(AudioType audioType, size_t fileSize)> on_start_send_audio;
    std::function<void(AudioType audioType)> on_end_send_audio;
};

class OpenClawWebSocket {
public:    
    OpenClawWebSocket();
    ~OpenClawWebSocket();
    
    bool Connect(const std::string& url);
    void Disconnect();
    bool IsConnected() const;
    
    void SetCallbacks(OpenClawWebSocketCallbacks callback);
    bool SendText(const std::string& text);
    bool SendBinary(const std::vector<uint8_t>& data);
    
private:
    std::unique_ptr<WebSocket> websocket_;
    EventGroupHandle_t event_group_;
    OpenClawWebSocketCallbacks callbacks_;

    bool isReceiving_;
    AudioType binaryType_; //0-wav,1-opus
    
};

#endif