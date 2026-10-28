#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 20 16:00:41 2020

@author: mert
"""

import os
import numpy as np
import cv2
import glob
from tkinter import Tcl
import utils

savepath='timelapse_video/timelapse.mp4'
framesPerSecond=30
cap=cv2.VideoCapture(0)
config=CFEVideoConf(cap,filepath=save_path,res='720p')
out=cv2.VideoWriter(save_path,config.video_type,framesPerSecond,config.dims)
timelapse_image_dir='/home/mert/Desktop/me407/images'
image_list=glob.glob(f"{timelapse_image_dir}/*.jpg")
sorted_images=Tcl().call('lsort', '-dict', image_list)




for imagename in sorted_images:
    image_frame=cv2.imread(imagename)
    out.write(image_frame)
    
    
class CFEVideoConf(object):
    # Standard Video Dimensions Sizes
    STD_DIMENSIONS =  {
        "360p": (480, 360),
        "480p": (640, 480),
        "720p": (1280, 720),
        "1080p": (1920, 1080),
        "4k": (3840, 2160),
    }
    # Video Encoding, might require additional installs
    # Types of Codes: http://www.fourcc.org/codecs.php
    VIDEO_TYPE = {
        'avi': cv2.VideoWriter_fourcc(*'XVID'),
        #'mp4': cv2.VideoWriter_fourcc(*'H264'),
        'mp4': cv2.VideoWriter_fourcc(*'XVID'),
    }

    width           = 640
    height          = 480
    dims            = (640, 480)
    capture         = None
    video_type      = None
    def __init__(self, capture, filepath, res="480p", *args, **kwargs):
        self.capture = capture
        self.filepath = filepath
        self.width, self.height = self.get_dims(res=res)
        self.video_type = self.get_video_type()

    # Set resolution for the video capture
    # Function adapted from https://kirr.co/0l6qmh
    def change_res(self, width, height):
        self.capture.set(3, width)
        self.capture.set(4, height)

    def get_dims(self, res='480p'):
        width, height = self.STD_DIMENSIONS['480p']
        if res in self.STD_DIMENSIONS:
            width, height = self.STD_DIMENSIONS[res]
        self.change_res(width, height)
        self.dims = (width, height)
        return width, height

    def get_video_type(self):
        filename, ext = os.path.splitext(self.filepath)
        if ext in self.VIDEO_TYPE:
          return  self.VIDEO_TYPE[ext]
        return self.VIDEO_TYPE['avi']