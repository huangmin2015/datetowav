//
// Created by kylin on 18-2-5.
//

#ifndef HOST_LINUX_WAV_H
#define HOST_LINUX_WAV_H

#define  PCM_FILE   "/tmp/test.wav"

typedef struct {
    char    fccID[4];       //内容为""RIFF
    unsigned int dwSize;   //最后填写，WAVE格式音频的大小
    char    fccType[4];     //内容为"WAVE"
}WAVE_HEADER;

typedef struct {
    char    fccID[4];          //内容为"fmt "
    unsigned int  dwSize;     //内容为WAVE_FMT占的字节数，为16 不包括fccID和dwSize
    unsigned short wFormatTag; //如果为PCM，改值为 1
    unsigned short wChannels;  //通道数，单通道=1，双通道=2
    unsigned int  dwSamplesPerSec;//采用频率
    unsigned int  dwAvgBytesPerSec;/* ==dwSamplesPerSec*wChannels*uiBitsPerSample/8 */
    unsigned short wBlockAlign;//==wChannels*uiBitsPerSample/8
    unsigned short uiBitsPerSample;//每个采样点的bit数，8bits=8, 16bits=16
}WAVE_FMT;

typedef struct {
    char    fccID[4];       //内容为"data"
    unsigned int dwSize;   //==NumSamples*wChannels*uiBitsPerSample/8
}WAVE_DATA;

typedef  struct {
    WAVE_HEADER header;
    WAVE_FMT fmt;
    WAVE_DATA data;
}WAVE_HEAD;


/**
 * Convert PCM16LE raw data to WAVE format
 * @param pcmpath       Input PCM file.
 * @param channels      Channel number of PCM file.
 * @param sample_rate   Sample rate of PCM file.
 * @param wavepath      Output WAVE file.
 */
int simplest_pcm16le_to_wave(const char *pcmpath, int channels, int sample_rate, const char *wavepath);

/**
 * Convert PCM16LE raw data to WAVE format
 * @param head          save wave format information
 * @param bit           bit of per sample
 * @param channels      Channel number of PCM file.
 * @param sample_rate   Sample rate of PCM file.
 */
void wav_init(WAVE_HEAD *head, unsigned short bit, unsigned short channels, unsigned int sample_rate);

int wav_write(WAVE_HEAD *head, const char *wav_path, unsigned short data);
int wave_finished(WAVE_HEAD *head);

#endif //HOST_LINUX_WAV_H
