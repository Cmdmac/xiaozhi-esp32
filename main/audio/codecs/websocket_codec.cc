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

std::vector<std::vector<uint8_t>>  OggParser::Parse(const uint8_t* data, size_t size) {

    std::vector<std::vector<uint8_t>> result;
    if (!IsOgg(data, size)) {
        ESP_LOGW(TAG, "Not OGG data");
        return result;
    }
    const uint8_t* buf = reinterpret_cast<const uint8_t*>(data);
    size_t offset = 0;

    auto find_page = [&](size_t start)->size_t {
        for (size_t i = start; i + 4 <= size; ++i) {
            if (buf[i] == 'O' && buf[i+1] == 'g' && buf[i+2] == 'g' && buf[i+3] == 'S') return i;
        }
        return static_cast<size_t>(-1);
    };

    bool seen_head = false;
    bool seen_tags = false;
    sample_rate_ = 16000; // 默认值

    int frame_count = 0;
    while (true) {
        size_t pos = find_page(offset);
        if (pos == static_cast<size_t>(-1)) break;
        offset = pos;
        if (offset + 27 > size) break;

        const uint8_t* page = buf + offset;
        uint8_t page_segments = page[26];
        size_t seg_table_off = offset + 27;
        if (seg_table_off + page_segments > size) break;

        size_t body_size = 0;
        for (size_t i = 0; i < page_segments; ++i) body_size += page[27 + i];

        size_t body_off = seg_table_off + page_segments;
        if (body_off + body_size > size) break;

        // Parse packets using lacing
        size_t cur = body_off;
        size_t seg_idx = 0;
        while (seg_idx < page_segments) {
            size_t pkt_len = 0;
            size_t pkt_start = cur;
            bool continued = false;
            do {
                uint8_t l = page[27 + seg_idx++];
                pkt_len += l;
                cur += l;
                continued = (l == 255);
            } while (continued && seg_idx < page_segments);

            if (pkt_len == 0) continue;
            const uint8_t* pkt_ptr = buf + pkt_start;

            if (!seen_head) {
                // 解析OpusHead包
                if (pkt_len >= 19 && memcmp(pkt_ptr, "OpusHead", 8) == 0) {
                    seen_head = true;
                    // OpusHead结构：[0-7] "OpusHead", [8] version, [9] channel_count, [10-11] pre_skip
                    // [12-15] input_sample_rate, [16-17] output_gain, [18] mapping_family
                    if (pkt_len >= 12) {
                        uint8_t version = pkt_ptr[8];
                        uint8_t channel_count = pkt_ptr[9];
                        if (pkt_len >= 16) {
                            // 读取输入采样率 (little-endian)
                            sample_rate_ = pkt_ptr[12] | (pkt_ptr[13] << 8) |
                                        (pkt_ptr[14] << 16) | (pkt_ptr[15] << 24);
                            ESP_LOGI(TAG, "OpusHead: version=%d, channels=%d, sample_rate=%d",
                                   version, channel_count, sample_rate_);
                        }
                    }
                }
                continue;
            }
            if (!seen_tags) {
                // Expect OpusTags in second packet
                if (pkt_len >= 8 && memcmp(pkt_ptr, "OpusTags", 8) == 0) {
                    seen_tags = true;
                }
                continue;
            }
            frame_count_++;
            frame_count++;
            // 60ms 一帧
            if (frame_count == 3) {
                std::vector<uint8_t> frame;
                frame.insert(frame.end(), pkt_ptr, pkt_ptr + pkt_len);
                result.push_back(frame);
                frame_count = 0;
            }
        }

        offset = body_off + body_size;
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

