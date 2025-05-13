
#include "alsa_recorder.h"

AlsaRecorder::AlsaRecorder(const char *name, 
                           unsigned int channels, 
                           snd_pcm_format_t format, 
                           unsigned int rate,
                           const int buffer_size, 
                           const char *p_file):
                           frames(32),
                           buffer(new char[buffer_size]), 
                           buffer_size(buffer_size) {
    
    snd_pcm_hw_params_t *params = nullptr;
    int dir;

    // 打开设备                                   
    if (snd_pcm_open(&handle, name, SND_PCM_STREAM_CAPTURE, 0) < 0) {
        ROS_ERROR_STREAM("unable to open input device");
        exit(1);
    }
    // 参数初始化    
    snd_pcm_hw_params_alloca(&params);

    // 使用默认参数
    snd_pcm_hw_params_any(handle, params);    

    // 翻译
    snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_MMAP_COMPLEX);
    
    // S32 小端模式
    snd_pcm_hw_params_set_format(handle, params, format); 
    
    // 8 通道 
    snd_pcm_hw_params_set_channels(handle, params, channels);

    // 采样率
    snd_pcm_hw_params_set_rate_near(handle, params, &rate, &dir);


    snd_pcm_hw_params_set_period_size_near(handle, params, &frames, &dir);

    if (snd_pcm_hw_params(handle, params) < 0) {
        ROS_ERROR_STREAM("unable to set hw parameters");
        exit(1);
    }
    
    snd_pcm_hw_params_get_period_size(params, &frames, &dir);
    snd_pcm_hw_params_get_period_time(params, &rate, &dir);

    if (p_file != nullptr){
        save_file = fopen(p_file, "wb+");
    }

}

AlsaRecorder::~AlsaRecorder() {
    delete buffer;
    snd_pcm_drain(handle);
    snd_pcm_close(handle);
    fclose(save_file);
    ROS_INFO_STREAM("stop recording..");

}

void AlsaRecorder::stratRecording(std::function<void (const void *, unsigned int)> rec_cb){
    int ret;
    ROS_INFO_STREAM("start recording..");
    while (ros::ok()){

        ret = snd_pcm_readi(handle, buffer, frames);
        if (ret == -EPIPE){
            ROS_INFO_STREAM("overrun occurred");
            snd_pcm_prepare(handle);
        } else if (ret < 0) {
            ROS_ERROR_STREAM("error from read");
            snd_strerror(ret);
        } else if (ret != frames){
            ROS_ERROR_STREAM("shot read frames...");
        }

        rec_cb(buffer, buffer_size);

        if (save_file != nullptr) {
            ret = fwrite(buffer, sizeof(char), buffer_size, save_file);
            if (ret != buffer_size) {
                ROS_INFO_STREAM("short write :write " << ret << " bytes");
        }
        }
        ros::spinOnce();
    }
}
