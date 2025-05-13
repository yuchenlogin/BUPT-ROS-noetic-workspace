#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import json

CONFIG_FOLDER_PATH = "/home/lemon"
CONFIG_FILENAME = ".robanConfig.json"
CONFIG_FILENAME_2 = ".lejuconfig/robanConfig.json"
CONFIG_FILE_PATH = os.path.join(CONFIG_FOLDER_PATH, CONFIG_FILENAME)
CONFIG_FILE_PATH_2 = os.path.join(CONFIG_FOLDER_PATH, CONFIG_FILENAME_2)

def get_robot_name():
    def get_name_param():
        if "name" not in robanConfig:
            return name_option_default["name"]
        return robanConfig["name"]

    name_option_default = {"name": "roban_default"}
    try:
        with open(CONFIG_FILE_PATH, mode='r') as read_file:
            rf = read_file.read()
            robanConfig = json.loads(rf, encoding='utf-8')
        roban_name = get_name_param()
    except:
        try:
            with open(CONFIG_FILE_PATH_2, "r") as f:
                readfile = f.read()
                robanConfig = json.loads(readfile, encoding="utf-8")
            roban_name = get_name_param()
        except:
            roban_name = name_option_default["name"]
    if sys.version>'3':
        return roban_name
    return roban_name.encode('utf-8')

def write_roban_name(file_path, write_data):
    if os.path.exists(file_path):
        with open(file_path, "r") as file:
            rfile = file.read()
            try:
                data = json.loads(rfile)
                data["name"] = write_data["name"]
            except:
                data = write_data
    else:
        data = write_data
    with open(file_path, "w") as file:
        try:
            json.dump(data, file)
            return 'success'
        except:
            return 'fail'

def set_robot_name(name):
    write_data = {"name": name}
    process_result = write_roban_name(CONFIG_FILE_PATH, write_data)
    process_result_2 = write_roban_name(CONFIG_FILE_PATH_2, write_data)
    return [process_result, process_result_2]
