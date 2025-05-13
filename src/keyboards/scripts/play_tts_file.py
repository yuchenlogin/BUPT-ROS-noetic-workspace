import os 
from threading import Thread
import time

class Player():
    def __init__(self):
        self.__is_playing = False
        self.path = ''

    def __play(self):
        self.__is_playing = True
        os.system("play -q " + self.path)
        self.__is_playing = False

    def play(self, path):
        # self.wait_to_play()
        self.path = path
        # Thread(target=self.__play).start()
        self.__play()

    def stop(self):
        if self.__is_playing == True:
            self.__is_playing == False
            os.system("ps -aux | grep {} | awk '{{print $2}}' | xargs kill".format(self.path))
    
    def __wait_to_play(self):
        while self.__is_playing == True:
            time.sleep(0.1)

    def choose_voice_file(self, text):
        return text + '.mp3'

    def play_complete_voice(self, voice_file_path, text):
        self.play(voice_file_path + self.choose_voice_file(text))

    def play_single_voice(self, voice_file_path, text):
        for word in text:
            if word == '.':
                word = 'dot'
            elif word == ' ':
                continue
            self.play(voice_file_path + self.choose_voice_file(word))

if __name__ == "__main__":
    # pass
    playy = Player()
    playy.play_single_voice()
