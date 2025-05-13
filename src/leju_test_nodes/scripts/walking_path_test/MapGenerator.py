#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
from json import loads
try:
    import xmltodict
except ImportError:
    os.system("pip install xmltodict")
    import xmltodict
import yaml

class MapGenerator(object):
    def __init__(self,marker_len=5):
        self.mark_len = marker_len
    
    def add_marker(self,index,points,status="1"):
        marker = {
            "@index":index,
            "@status":status,
            "corner":[
            ]
        }
        halflen = self.mark_len/2.0
        marker["corner"].extend(
            [
                {"@x":str(points[0]-halflen),"@y":str(points[1]-halflen),"@z":"0"},
                {"@x":str(points[0]+halflen),"@y":str(points[1]-halflen),"@z":"0"},
                {"@x":str(points[0]+halflen),"@y":str(points[1]+halflen),"@z":"0"},
                {"@x":str(points[0]-halflen),"@y":str(points[1]+halflen),"@z":"0"}
            ])
        return marker
        
    def points2map(self,json_dict,status="1"):
        """
        :json_dict {
                    "0":[0,0],
                    "4":[10,0],
                    "2":[5,0],
                    ...
                    }
        :status 地图标识
        """
        map = {"multimarker":{
            "@markers":str(len(json_dict)),
            "marker":[]
        }}
        for index in json_dict:
            pos = json_dict[index]
            map["multimarker"]["marker"].append(self.add_marker(index,pos,status))
        return self.map2xml(map)
    
    def map2xml(self,map):
        return xmltodict.unparse(map, pretty=True)
    
if __name__ == '__main__':
    SCRIPTS_PATH=os.path.split(sys.argv[0])[0]
    with open(os.path.join(SCRIPTS_PATH,"config.yaml"),"r")as f:
        config = yaml.safe_load(f)
        print(config)
        
    points = config["MAP_POINTS"]
    xmlstr = MapGenerator().points2map(points)
    print(xmlstr)
    with open("test.xml","w",encoding="utf-8") as f:
        f.write(xmlstr)

