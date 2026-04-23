#include "websocket_codec.h"
#include "board.h"
#include "esp_log.h"

const char* WebSocketCodec::TAG = "WebSocketCodec";
const char* OggParser::TAG = "OggParser";

OggParser::OggParser() : frame_count_(0) {
}
        
bool OggParser::IsOgg(const uint8_t* data, size_t size) {
    return size >= 4 && 
           data[0] == 'O' && 
           data[1] == 'g' && 
           data[2] == 'g' && 
           data[3] == 'S';
}

bool OggParser::ParseOpusHead(const uint8_t* data, size_t size) {
    // OpusHead 包以 "OpusHead" 标签开头（8字节）
    if (size < 19) {
        return false;
    }
    
    // 检查 OpusHead 标签
    if (memcmp(data, "OpusHead", 8) != 0) {
        return false;
    }
    
    // 解析 OpusHead 结构（小端序）
    opus_head_.version = data[8];
    opus_head_.channel_count = data[9];
    opus_head_.pre_skip = data[10] | (data[11] << 8);
    opus_head_.sample_rate = data[12] | (data[13] << 8) | (data[14] << 16) | (data[15] << 24);
    opus_head_.output_gain = data[16] | (data[17] << 8);
    opus_head_.mapping_family = data[18];
    
    return true;
}

std::vector<std::vector<uint8_t>> OggParser::Parse(const uint8_t* data, size_t size) {
    std::vector<std::vector<uint8_t>> result;
    if (!IsOgg(data, size)) {
        ESP_LOGW(TAG, "Not OGG data");
        return result;
    }

    bool seen_head = false;
    bool seen_tags = false;
    sample_rate_ = 16000; // 默认值

    int local_frame_count = 0; // 避免与成员变量 frame_count_ 名字混淆
    std::vector<uint8_t> frame_buffer; // 用于拼接连续的音频帧

    size_t offset = 0;
    
    // 只要剩余数据还够一个最小的 OGG 页头 (27字节)，就继续解析
    while (offset + 27 <= size) {
        // 1. 寻找 OggS 同步字 (扁平化查找，去掉原来的 lambda 函数)
        if (data[offset] != 'O' || data[offset+1] != 'g' || 
            data[offset+2] != 'g' || data[offset+3] != 'S') {
            offset++;
            continue;
        }

        // 2. 解析 OGG 页头
        uint8_t page_segments = data[offset + 26];
        size_t seg_table_off = offset + 27;
        if (seg_table_off + page_segments > size) break;

        // 3. 解析 Lacing 表 (段表) 并提取 Packet
        size_t body_off = seg_table_off + page_segments;
        size_t data_ptr = body_off;
        
        size_t pkt_len = 0;
        size_t pkt_start = data_ptr;

        // 一个优雅的 for 循环解决所有 Lacing 拼接逻辑
        for (int i = 0; i < page_segments; ++i) {
            uint8_t seg_len = data[seg_table_off + i];
            pkt_len += seg_len;
            data_ptr += seg_len;

            // 防御性越界检查
            if (data_ptr > size) return result;

            // 根据 OGG 规范，seg_len < 255 代表当前 Packet 结束
            if (seg_len < 255) {
                if (pkt_len > 0) {
                    const uint8_t* pkt = data + pkt_start;

                    // === 开始处理完整的 Packet ===
                    if (!seen_head) {
                        // 解析 OpusHead
                        if (pkt_len >= 19 && memcmp(pkt, "OpusHead", 8) == 0) {
                            seen_head = true;
                            sample_rate_ = pkt[12] | (pkt[13] << 8) | (pkt[14] << 16) | (pkt[15] << 24);
                            ESP_LOGI(TAG, "OpusHead: sample_rate=%d", sample_rate_);
                        }
                    } else if (!seen_tags) {
                        // 解析 OpusTags
                        if (pkt_len >= 8 && memcmp(pkt, "OpusTags", 8) == 0) {
                            seen_tags = true;
                        }
                    } else {
                        // 处理实际的音频数据帧
                        frame_count_++;
                        local_frame_count++;

                        // 将当前帧追加到 buffer 中（修复原代码丢弃前两帧的 Bug）
                        frame_buffer.insert(frame_buffer.end(), pkt, pkt + pkt_len);

                        // 攒满 3 帧 (例如 3 x 20ms = 60ms) 后输出
                        if (local_frame_count == 3) {
                            result.push_back(frame_buffer);
                            frame_buffer.clear(); // 清空 buffer 准备装下一批
                            local_frame_count = 0;
                        }
                    }
                    // === Packet 处理结束 ===
                }
                
                // 重置指针，准备提取下一个 Packet
                pkt_start = data_ptr;
                pkt_len = 0;
            }
        }
        
        // 直接跳到下一页的开头
        offset = data_ptr; 
    }

    return result;
}

void WebSocketCodec::OutputData(std::vector<int16_t>& data){
    Write(data.data(), data.size());
}

bool WebSocketCodec::InputData(std::vector<int16_t>& data){
    int samples = Read(data.data(), data.size());
    if (samples > 0) {
        return true;
    }
    return false;
}

int WebSocketCodec::Read(int16_t* dest, int samples)
{
    // std::unique_lock<std::mutex> lock(buffer_mutex_);
    
    // // 等待足够的音频数据
    // buffer_cv_.wait(lock, [this, samples]() {
    //     return audio_buffer_.size() >= samples;
    // });

    // int read_count = 0;
    // while (read_count < samples && !audio_buffer_.empty()) {
    //     dest[read_count] = audio_buffer_.front();
    //     audio_buffer_.pop();
    //     read_count++;
    // }
    std::unique_lock<std::mutex> lock(buffer_mutex_);
    
    // 非阻塞：如果没有足够数据，直接返回
    if (audio_buffer_.size() < samples) {
        return 0;  // 返回 0 表示没有足够数据
    }

    int read_count = 0;
    while (read_count < samples && !audio_buffer_.empty()) {
        dest[read_count] = audio_buffer_.front();
        audio_buffer_.pop();
        read_count++;
    }
    return read_count;
}

int WebSocketCodec::Write(const int16_t* data, int samples)
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    for (int i = 0; i < samples; i++) {
        audio_buffer_.push(data[i]);
    }
    buffer_cv_.notify_one();
    return samples;
}

