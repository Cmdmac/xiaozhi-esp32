// OpenClawWebsocket.cpp
#include "openclaw_websocket.h"
#include "wifi_board.h"
#include "application.h"
#include "protocols/protocol.h"
#include <esp_log.h>
#include <cstring>
#include <sstream>

static const char* TAG = "OpenClawWebSocket";

std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    
    // 使用 getline 按指定分隔符读取
    while (std::getline(tokenStream, token, delimiter)) {
        // ESP_LOGI(TAG, "Token: %s", token.c_str());
        tokens.push_back(token);
    }
    return tokens;
}

OpenClawWebSocket::OpenClawWebSocket() {
    event_group_ = xEventGroupCreate();
}

OpenClawWebSocket::~OpenClawWebSocket() {
    Disconnect();
    if (event_group_) {
        vEventGroupDelete(event_group_);
    }
}

bool OpenClawWebSocket::Connect(const std::string& url) {
    if (websocket_) {
        Disconnect();
    }
    
    auto network = WifiBoard::GetInstance().GetNetwork();
    websocket_ = network->CreateWebSocket(2);
    if (!websocket_) {
        ESP_LOGE(TAG, "Failed to create WebSocket");
        return false;
    }
    
    websocket_->OnData([this](const char* data, size_t len, bool binary) {
        if (binary) {
            ESP_LOGW(TAG, "Received binary data: %u, type: %d", len, static_cast<int>(binaryType_));
            std::vector<uint8_t> opus_data(data, data + len);
            audio_data_callback_(opus_data, binaryType_, isReceiving_);
        } else {
            ESP_LOGI(TAG, "Received text: %.*s", len, data);
            std::string header(data, len);
            //cmd=start_send,type={type},file_size={file_size}
            std::vector<std::string> parts = split(header, ',');
            if (parts.size() < 1) {
                ESP_LOGE(TAG, "Invalid header format parts: %d", parts.size());
                return;
            }
            std::string command = parts[0];
            std::vector<std::string> cmd_parts = split(command, '=');
            //开始发送
            if (cmd_parts.size() == 2 && cmd_parts[0] == "cmd" && cmd_parts[1] == "start_send") {
                isReceiving_ = true;

                std::string type = split(parts[1], '=')[1];
                std::string file_size = split(parts[2], '=')[1];
                if (type == "wav") {
                    binaryType_ = AudioType::WAV;
                } else if (type == "ogg") {
                    binaryType_ = AudioType::OGG;
                } else {
                    ESP_LOGE(TAG, "Invalid type");
                    return;
                }
            } else if (cmd_parts.size() == 2 && cmd_parts[0] == "cmd" && cmd_parts[1] == "end_send") {
                //结束发送
                isReceiving_ = false;
            }
            
        }
    });
    
    if (!websocket_->Connect(url.c_str())) {
        ESP_LOGE(TAG, "Failed to connect to WebSocket server: %s", url.c_str());
        return false;
    }
    
    ESP_LOGI(TAG, "Connected to WebSocket server: %s", url.c_str());

    return true;
}

void OpenClawWebSocket::Disconnect() {
    if (websocket_) {
        websocket_.reset();
        ESP_LOGI(TAG, "Disconnected from WebSocket server");
    }
}

bool OpenClawWebSocket::IsConnected() const {
    return websocket_ && websocket_->IsConnected();
}

void OpenClawWebSocket::SetAudioDataCallback(AudioDataCallback callback) {
    audio_data_callback_ = callback;
}

bool OpenClawWebSocket::SendText(const std::string& text) {
    if (!websocket_ || !websocket_->IsConnected()) {
        ESP_LOGE(TAG, "WebSocket not connected");
        return false;
    }
    return websocket_->Send(text.c_str());
}

bool OpenClawWebSocket::SendBinary(const std::vector<uint8_t>& data) {
    if (!websocket_ || !websocket_->IsConnected()) {
        ESP_LOGE(TAG, "WebSocket not connected");
        return false;
    }
    return websocket_->Send(data.data(), data.size(), true);
}
