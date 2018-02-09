//
// Created by kylin on 18-2-5.
//

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "Wav.h"

#define  PCM_FILE   "/tmp/test.wav"

static  FILE *g_fd;


/*
int main()
{
    WAVE_HEAD head = {0};
    FILE *fd;
    int ret = -1;
    unsigned short pcm_data;


    fd = fopen(PCM_FILE, "rb+");
    if (fd == NULL) {
        printf("%s write error", __FUNCTION__);
        perror("msg: ");
        exit(EXIT_FAILURE);
    }
    wav_init(&head, 16, 2, 44100);

    fread(&pcm_data, sizeof(unsigned short), 1, fd);
    while(!feof(fd))
    {
        ret = wav_write(&head, "/tmp/wav.test", pcm_data);
        if (ret < 0) {
            printf("wav write error\n");
            return -1;
        }
        fread(&pcm_data, sizeof(unsigned short), 1, fd);
    }

    wave_finished(&head);
    return 0;
}
*/


void wav_init(WAVE_HEAD *head, unsigned short bits, unsigned short channels, unsigned int sample_rate)
{
    if(channels==2 || sample_rate==0)
    {
        channels = 2;
        sample_rate = 44100;
    }

    /* WAVE_HEADER */
    memcpy(head->header.fccID, "RIFF", strlen("RIFF"));
    memcpy(head->header.fccType, "WAVE", strlen("WAVE"));

    /* WAVE_FMT */
    memcpy(head->fmt.fccID, "fmt ", strlen("fmt "));
    head->fmt.dwSize = 16; //fmt struct size
    head->fmt.wFormatTag = 1; // "1" PCM data
    head->fmt.wChannels = channels;
    head->fmt.dwSamplesPerSec = sample_rate;
    head->fmt.uiBitsPerSample = bits;

    /* ==dwSamplesPerSec*wChannels*uiBitsPerSample/8 */
    head->fmt.dwAvgBytesPerSec = (head->fmt.dwSamplesPerSec) * (head->fmt.wChannels) \
                                  * (head->fmt.uiBitsPerSample) / 8;

    /* ==wChannels*uiBitsPerSample/8 */
    head->fmt.wBlockAlign = (unsigned short)((head->fmt.wChannels) \
                            * (head->fmt.uiBitsPerSample) / 8);

    /* WAVE_DATA */
    memcpy(head->data.fccID, "data", strlen("data"));
    head->data.dwSize = 0;

}

int wav_write(WAVE_HEAD *head, const char *wav_path, unsigned short data)
{

    int  ret = -1;
    do {

        if (g_fd == NULL && wav_path != NULL) {

            g_fd = fopen(wav_path , "wb+");
            if (g_fd == NULL) {
                perror("fail to open wav file ");
                break;
            }

            ret = (int)fwrite(head, sizeof(WAVE_HEAD), 1, g_fd);
            if (ret != 1) {
                printf("%s write error\n", __FUNCTION__);
                perror("msg: ");
                ret = -1;
                break;
            }

            fseek(g_fd, sizeof(WAVE_HEAD), 1);   //1=SEEK_CUR
        }

        head->data.dwSize += 2;
        ret = (int)fwrite(&data, sizeof(unsigned short), 1, g_fd);
        if (ret != 1) {
            printf("%s write error\n", __FUNCTION__);
            perror("msg: ");
            ret = -1;
            break;
        }

    } while(0);

    if (ret < 0 && g_fd != NULL) {
        fclose(g_fd);
        g_fd = NULL;
    }

    return  ret;
}

int wave_finished(WAVE_HEAD *head)
{
    int ret = -1;
    head->header.dwSize = 36 + head->data.dwSize;
    printf("dwWsize = %d\n", head->header.dwSize);

    rewind(g_fd);
    ret = (int)fwrite(&head->header, sizeof(WAVE_HEADER), 1, g_fd);
    if (ret != 1 ) {
        printf("%s write error\n", __FUNCTION__);
        perror("reason: ");
        return ret;
    }

    fseek(g_fd, sizeof(WAVE_FMT), SEEK_CUR);   //1=SEEK_CUR
    ret = (int)fwrite(&head->data, sizeof(WAVE_DATA), 1, g_fd);
    if (ret != 1 ) {
        printf("%s write error\n", __FUNCTION__);
        perror("reason: ");
        return ret;
    }

    if (g_fd != NULL) {
        fclose(g_fd);
        g_fd = NULL;
    }

    return  0;
}

