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

class OpenClawWebSocket {
public:
    typedef std::function<void(const std::vector<uint8_t>&)> OpusDataCallback;
    
    OpenClawWebSocket();
    ~OpenClawWebSocket();
    
    bool Connect(const std::string& url);
    void Disconnect();
    bool IsConnected() const;
    
    void SetOpusDataCallback(OpusDataCallback callback);
    bool SendText(const std::string& text);
    bool SendBinary(const std::vector<uint8_t>& data);
    
private:
    std::unique_ptr<WebSocket> websocket_;
    EventGroupHandle_t event_group_;
    OpusDataCallback opus_data_callback_;
    
    void ProcessOpusData(const uint8_t* data, size_t size);
    bool IsOggContainer(const uint8_t* data, size_t size);
    void ParseOggContainer(const uint8_t* data, size_t size);
};

#endif