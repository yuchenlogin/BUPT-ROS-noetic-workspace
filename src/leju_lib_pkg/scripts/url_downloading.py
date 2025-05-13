#!/usr/bin/env python3
# coding=utf-8

import time
import requests
import os
import hashlib
from functools import partial

CANCEL_REDOWNLOAD = ['n', 'N']

def __downloading(url, saved_folder, file_saved_name):
    file_path = os.path.join(saved_folder, file_saved_name)

    if not os.path.exists(saved_folder):
        print("保存的文件夹不存在，将自动创建...")
        os.makedirs(saved_folder)
    try:
        response = requests.get(url, stream=True)
        if response.status_code != 200:
            print('下载失败！HTTP 状态码：{}'.format(response.status_code))
            return False
        content_size = int(response.headers.get('Content-Length', 0))
        print('正在下载 {}，文件大小: {:.2f} MB'.format(file_saved_name, content_size / 1024 / 1024))
        with open(file_path, 'wb') as file:
            progress = 0
            chunk_size = 1024
            start_time = time.time()
            for data in response.iter_content(chunk_size=chunk_size):
                file.write(data)
                progress += len(data)
                percent = progress / content_size * 100
                print('\r[下载进度]:' + '>' * int(percent / 2) + ' {:.2f}%'.format(percent), end='', flush=True)
            end_time = time.time()
            print('\n下载完毕! 用时: {:.2f}秒'.format(end_time - start_time))
            return True
    except requests.exceptions.RequestException as e:
        print('下载失败! Error: {}'.format(e))
        return False

def md5sum(filename):
    with open(filename, mode='rb') as f:
        d = hashlib.md5()
        for buf in iter(partial(f.read, 128), b''):
            d.update(buf)
    return d.hexdigest()

def download_from_url(url, saved_folder, file_saved_name, md5=None):
    file_path = os.path.join(saved_folder, file_saved_name)

    if os.path.exists(file_path):
        if md5 != None:
            current_file_md5 = md5sum(file_path)
            if current_file_md5 != md5:
                os.remove(file_path)
                return __downloading(url, saved_folder, file_saved_name)
            else:
                print("文件已存在!")
                return True
        else:
            is_redownload = input("文件存在，是否重新下载并覆盖（y/n）:")
            if is_redownload in CANCEL_REDOWNLOAD:
                return True
            else:
                print("remove")
                os.remove(file_path)
    return __downloading(url, saved_folder, file_saved_name)

if __name__ == "__main__":
    url = "https://roban.lejurobot.com/modols/digital_detection.onnx"
    saved_folder = "/home/lemon"
    file_saved_name = "digital_detection.onnx"
    md5 = "48ea1eb83e0222d19ae4763f1a8f3aad"
    download_from_url(url, saved_folder, file_saved_name, md5)
