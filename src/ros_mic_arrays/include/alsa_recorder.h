

#ifndef __ALSA_RECORDER_H__
#define __ALSA_RECORDER_H__

#include <alsa/asoundlib.h>
#include <ros/ros.h>
#include <stdio.h>

class AlsaRecorder {
private:
    snd_pcm_t *handle = nullptr;
    snd_pcm_uframes_t frames;
    char *buffer = nullptr;
    FILE *save_file = nullptr;
public:
    const int buffer_size;
    AlsaRecorder(const char *name, 
                 unsigned int channels, 
                 snd_pcm_format_t format, 
                 unsigned int rate,
                 const int buffer_size, 
                 const char *p_file=nullptr);
    void stratRecording(std::function<void (const void *, unsigned ini)> rec_cb);
    ~AlsaRecorder();
};
 
#endif