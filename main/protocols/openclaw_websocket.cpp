// OpenClawWebsocket.cpp
#include "openclaw_websocket.h"
#include "wifi_board.h"
#include "application.h"
#include "protocols/protocol.h"
#include <esp_log.h>
#include <cstring>

static const char* TAG = "OpenClawWebSocket";

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
            ESP_LOGW(TAG, "Received binary data: %u", len);
            ProcessOpusData(reinterpret_cast<const uint8_t*>(data), len);
        } else {
            ESP_LOGI(TAG, "Received text: %.*s", len, data);
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

void OpenClawWebSocket::SetOpusDataCallback(OpusDataCallback callback) {
    opus_data_callback_ = callback;
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

void OpenClawWebSocket::ProcessOpusData(const uint8_t* data, size_t size) {
    if (IsOggContainer(data, size)) {
        ESP_LOGI(TAG, "Received Ogg container: %u", size);
        ParseOggContainer(data, size);
    } else {
        // 直接处理原始 OPUS 数据
        if (opus_data_callback_) {
            std::vector<uint8_t> opus_data(data, data + size);
            opus_data_callback_(opus_data);
        }
    }
}

bool OpenClawWebSocket::IsOggContainer(const uint8_t* data, size_t size) {
    return size >= 4 && 
           data[0] == 'O' && 
           data[1] == 'g' && 
           data[2] == 'g' && 
           data[3] == 'S';
}

// void OpenClawWebSocket::ParseOggContainer(const uint8_t* data, size_t size) {
//     size_t offset = 0;
    
//     auto find_page = [&](size_t start)->size_t {
//         for (size_t i = start; i + 4 <= size; ++i) {
//             if (data[i] == 'O' && data[i+1] == 'g' && data[i+2] == 'g' && data[i+3] == 'S') return i;
//         }
//         return static_cast<size_t>(-1);
//     };
    
//     bool seen_head = false;
//     bool seen_tags = false;
    
//     while (true) {
//         size_t pos = find_page(offset);
//         if (pos == static_cast<size_t>(-1)) break;
//         offset = pos;
//         if (offset + 27 > size) break;
        
//         const uint8_t* page = data + offset;
//         uint8_t page_segments = page[26];
//         size_t seg_table_off = offset + 27;
//         if (seg_table_off + page_segments > size) break;
        
//         size_t body_size = 0;
//         for (size_t i = 0; i < page_segments; ++i) body_size += page[27 + i];
        
//         size_t body_off = seg_table_off + page_segments;
//         if (body_off + body_size > size) break;
        
//         size_t cur = body_off;
//         size_t seg_idx = 0;
//         while (seg_idx < page_segments) {
//             size_t pkt_len = 0;
//             size_t pkt_start = cur;
//             bool continued = false;
//             do {
//                 uint8_t l = page[27 + seg_idx++];
//                 pkt_len += l;
//                 cur += l;
//                 continued = (l == 255);
//             } while (continued && seg_idx < page_segments);
            
//             if (pkt_len == 0) continue;
//             const uint8_t* pkt_ptr = data + pkt_start;
            
//             if (!seen_head) {
//                 if (pkt_len >= 19 && std::memcmp(pkt_ptr, "OpusHead", 8) == 0) {
//                     seen_head = true;
//                 }
//                 continue;
//             }
//             if (!seen_tags) {
//                 if (pkt_len >= 8 && std::memcmp(pkt_ptr, "OpusTags", 8) == 0) {
//                     seen_tags = true;
//                 }
//                 continue;
//             }
            
//             if (seen_head && seen_tags) {
//                 if (opus_data_callback_) {
//                     std::vector<uint8_t> opus_data(pkt_ptr, pkt_ptr + pkt_len);
//                     opus_data_callback_(opus_data);
//                 }
//             }
//         }
        
//         offset = body_off + body_size;
//     }
// }

void OpenClawWebSocket::ParseOggContainer(const uint8_t* data, size_t size) {
    std::vector<uint8_t> accumulated_opus_data_;  // 累积的 OPUS 数据
    bool is_receiving_file_ = false;              // 是否正在接收文件
    bool is_file_complete_ = false;               // 文件是否接收完成
    
    size_t offset = 0;
    while (offset < size) {
        // 检查 Ogg 页头
        if (offset + 27 > size) {
            break;
        }

        // 验证 Ogg 魔法字节
        if (memcmp(data + offset, "OggS", 4) != 0) {
            ESP_LOGE(TAG, "Invalid Ogg header");
            break;
        }

        // 解析页头信息
        uint8_t header_type_flag = data[offset + 5];  // 第5字节是头类型标志
        bool is_last_page = (header_type_flag & 0x04) != 0;  // 第2位表示是否为最后一页
        uint32_t page_serial = *(uint32_t*)(data + offset + 14);
        uint32_t page_segments = data[offset + 26];

        // 计算页数据大小
        size_t segment_table_offset = offset + 27;
        size_t data_offset = segment_table_offset + page_segments;
        size_t page_size = 0;

        for (size_t i = 0; i < page_segments; i++) {
            page_size += data[segment_table_offset + i];
        }

        // 检查页数据是否完整
        if (data_offset + page_size > size) {
            break;
        }

        // 提取 OPUS 数据并累积
        std::vector<uint8_t> opus_data(data + data_offset, data + data_offset + page_size);
        accumulated_opus_data_.insert(accumulated_opus_data_.end(), opus_data.begin(), opus_data.end());

        // 检查是否为最后一页
        if (is_last_page) {
            is_file_complete_ = true;
            // 文件接收完成，发送累积的 OPUS 数据
            if (opus_data_callback_) {
                opus_data_callback_(accumulated_opus_data_);
            }
            // 重置状态
            accumulated_opus_data_.clear();
            is_receiving_file_ = false;
            is_file_complete_ = false;
        } else {
            // 不是最后一页，继续接收
            is_receiving_file_ = true;
        }

        // 移动到下一页
        offset = data_offset + page_size;
    }
}