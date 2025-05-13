#!/usr/bin python3
from pydub import AudioSegment
import sys, os

MP3_ENDSWITH = '.mp3'
WAV_ENDSWITH = '.wav'

def mp3_to_wav(file):
    sound = AudioSegment.from_mp3(file)
    filename = os.path.basename(file)
    name_without_ext = os.path.splitext(filename)[0]
    sava_path = "/tmp/{}{}".format(name_without_ext, WAV_ENDSWITH)
    sound.export(sava_path, format="wav")
    print('save in {}'.format(sava_path))

if __name__ == '__main__':
    getinput = sys.argv[1:]
    if len(getinput) > 0:
        if getinput[0].endswith(MP3_ENDSWITH):
            mp3_to_wav(getinput[0])
        else:
            print('Features to be developed...')
    else:
        print('请传入需要转换的文件')
