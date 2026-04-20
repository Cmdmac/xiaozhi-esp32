#include "websocket_codec.h"
#include "board.h"
#include "esp_log.h"

const char* WebSocketCodec::TAG = "WebSocketCodec";

bool WebSocketCodec::IsOggContainer(const uint8_t* data, size_t size) {
    return size >= 4 && 
           data[0] == 'O' && 
           data[1] == 'g' && 
           data[2] == 'g' && 
           data[3] == 'S';
}

void WebSocketCodec::ParseOggContainer(const uint8_t* data, size_t size) {
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
            // if (opus_data_callback_) {
            //     opus_data_callback_(accumulated_opus_data_);
            // }
            Process8BitAudio(accumulated_opus_data_.data(), accumulated_opus_data_.size());
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

void WebSocketCodec::Process8BitAudio(const uint8_t* data, size_t size)
{
    // 检查数据大小是否为偶数（16位需要2个字节）
    if (size % 2 != 0) {
        ESP_LOGW(TAG, "Data size is odd, last byte will be ignored");
        size = size - 1;  // 忽略最后一个字节
    }
    
    size_t sample_count = size / 2;  // 16位样本数 = 8位字节数 / 2
    
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    for (size_t i = 0; i < sample_count; i++) {
        // 将2个8位字节组合成1个16位样本
        // 小端序：低字节在前，高字节在后
        int16_t pcm_value = static_cast<int16_t>(data[i * 2]) | 
                           (static_cast<int16_t>(data[i * 2 + 1]) << 8);
        audio_buffer_.push(pcm_value);
    }
    buffer_cv_.notify_one();
    ESP_LOGI(TAG, "Converted %u bytes to %u 16-bit samples", size, sample_count);
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

