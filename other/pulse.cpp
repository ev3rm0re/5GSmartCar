#include <iostream>
#include <pulse/simple.h>
#include <pulse/error.h>
#include <math.h>

#define SAMPLE_RATE 44100
#define CHANNELS 1
#define SAMPLE_FORMAT PA_SAMPLE_S16LE

int main() {
    // 配置PulseAudio音频流
    static const pa_sample_spec sampleSpec = {
        .format = SAMPLE_FORMAT,
        .rate = SAMPLE_RATE,
        .channels = CHANNELS
    };

    // 创建一个PulseAudio播放流
    int error;
    pa_simple *stream = pa_simple_new(
        NULL,               // 使用默认服务器
        "PulseAudioDemo",    // 应用名称
        PA_STREAM_PLAYBACK,  // 播放流
        NULL,               // 默认设备
        "playback",         // 流描述
        &sampleSpec,        // 样本规格
        NULL,               // 通道映射
        NULL,               // 缓冲区属性
        &error              // 错误返回
    );

    if (!stream) {
        std::cerr << "PulseAudio流创建失败: " << pa_strerror(error) << std::endl;
        return 1;
    }

    // 生成并播放一个正弦波
    const int duration = 5;  // 播放时长，单位秒
    const int amplitude = 32767;  // 最大音量
    const double frequency = 440.0;  // 音调频率 (Hz)

    for (int i = 0; i < SAMPLE_RATE * duration; ++i) {
        int16_t sample = (int16_t)(amplitude * sin(2.0 * M_PI * frequency * i / SAMPLE_RATE));
        if (pa_simple_write(stream, &sample, sizeof(sample), &error) < 0) {
            std::cerr << "写入音频数据失败: " << pa_strerror(error) << std::endl;
            pa_simple_free(stream);
            return 1;
        }
    }

    // 刷新和关闭音频流
    if (pa_simple_drain(stream, &error) < 0) {
        std::cerr << "音频流刷新失败: " << pa_strerror(error) << std::endl;
    }

    pa_simple_free(stream);
    return 0;
}
