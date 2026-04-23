#include "websocket_codec.h"
#include "board.h"
#include "esp_log.h"

const char* WebSocketCodec::TAG = "WebSocketCodec";
const char* OggParser::TAG = "OggParser";

OggParser::OggParser() : seen_head_(false), seen_tags_(false), frame_count_(0) {
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

// 辅助函数 1：滑动窗口寻找下一个 OGG 页头
bool OggParser::FindNextPage(const uint8_t* data, size_t size, size_t& offset) {
    while (offset + 27 <= size) {
        if (data[offset] == 'O' && data[offset+1] == 'g' && 
            data[offset+2] == 'g' && data[offset+3] == 'S') {
            return true; // 找到了
        }
        offset++; // 没找到就往前推进 1 字节
    }
    return false;
}

// 辅助函数 2：专属的 Opus 逻辑处理器
void OggParser::ProcessOpusPacket(const uint8_t* pkt, size_t pkt_len, 
                                  std::vector<std::vector<uint8_t>>& result,
                                  std::vector<uint8_t>& frame_buffer, 
                                  int& local_frame_count) {
    if (!seen_head_) {
        // 解析 OpusHead 包
        if (pkt_len >= 19 && memcmp(pkt, "OpusHead", 8) == 0) {
            seen_head_ = true;
            sample_rate_ = pkt[12] | (pkt[13] << 8) | (pkt[14] << 16) | (pkt[15] << 24);
            ESP_LOGI(TAG, "OpusHead: sample_rate=%d", sample_rate_);
        }
        return;
    } 
    
    if (!seen_tags_) {
        // 解析 OpusTags 包 (通常是第二个包)
        if (pkt_len >= 8 && memcmp(pkt, "OpusTags", 8) == 0) {
            seen_tags_ = true;
        }
        return;
    }

    // --- 运行到这里，说明一定是纯音频数据了 ---
    frame_count_++;
    local_frame_count++;
    
    // 把当前音频帧塞进缓冲区
    frame_buffer.insert(frame_buffer.end(), pkt, pkt + pkt_len);
    
    // 攒满 3 帧 (例如 60ms) 后，打包推入结果
    if (local_frame_count == 3) {
        result.push_back(frame_buffer);
        frame_buffer.clear();
        local_frame_count = 0;
    }
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

std::vector<std::vector<uint8_t>> OggParser::Parse(const uint8_t* data, size_t size) {
    std::vector<std::vector<uint8_t>> result;
    if (!IsOgg(data, size)) {
        ESP_LOGW(TAG, "Not OGG data");
        return result;
    }

    // 初始化状态
    seen_head_ = false;
    seen_tags_ = false;
    sample_rate_ = 16000; 

    int local_frame_count = 0; 
    std::vector<uint8_t> frame_buffer; 
    size_t offset = 0;
    
    // 1. 寻找每一个 OGG 页 (Page)
    while (FindNextPage(data, size, offset)) {
        uint8_t page_segments = data[offset + 26];
        size_t seg_table_off = offset + 27;
        
        if (seg_table_off + page_segments > size) break;

        size_t body_off = seg_table_off + page_segments;
        size_t data_ptr = body_off;
        
        size_t pkt_len = 0;
        size_t pkt_start = data_ptr;

        // 2. 遍历该页的段表 (Segment Table)，切分出完整的数据包 (Packet)
        for (int i = 0; i < page_segments; ++i) {
            uint8_t seg_len = data[seg_table_off + i];
            pkt_len += seg_len;
            data_ptr += seg_len;

            if (data_ptr > size) return result; // 防御性越界检查

            // 段长度小于 255 代表一个完整 Packet 的结束
            if (seg_len < 255) {
                if (pkt_len > 0) {
                    // 3. 把切分好的完整包交给 Opus 处理器
                    ProcessOpusPacket(data + pkt_start, pkt_len, result, frame_buffer, local_frame_count);
                }
                // 重置参数，准备切下一个包
                pkt_start = data_ptr;
                pkt_len = 0;
            }
        }
        // 当前页处理完毕，指针跳到下一页开头
        offset = data_ptr; 
    }

    return result;
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

