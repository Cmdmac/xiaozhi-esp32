#ifndef MIX_AUDIO_CODEC_CC
#define MIX_AUDIO_CODEC_CC

#include "audio_codec.h"
#include "websocket_codec.h"
#include "codecs/no_audio_codec.h"

class MixAudioCodec : public AudioCodec {
    private:
        WebSocketCodec* websocket_codec_;
        AudioCodec* device_audio_codec_;

        bool ws_input_enabled_;
    
    public:
        MixAudioCodec(AudioCodec* codec);

        virtual void Start() override;
        virtual void OutputData(std::vector<int16_t>& data) override;
        virtual bool InputData(std::vector<int16_t>& data) override;
        virtual void EnableInput(bool enable) override;
        virtual void EnableOutput(bool enable) override;


        void enableWSInput(bool enable);
        void writeFromWS(const int16_t* data, int samples);

    protected:
        virtual int Read(int16_t* dest, int samples) override;
        virtual int Write(const int16_t* data, int samples) override;
};


#endif