# !/usr/bin/env python
# -*-coding:utf-8 -*-
# @Time    : 2024/01/20 14:16
# @Author  : Skelon_Chan
# @File    : gen_gif.py
# @Version : python3.9
# @Desc    : $END$
import imageio.v2 as imageio
import matplotlib.pyplot as plt

# 将临时文件合并成gif格式
images = []
for i in range(59):
    filename = 'temp_image_' + str(i+1) + '.png'
    images.append(imageio.imread(filename))
imageio.mimsave('4.gif', images, duration=0.15)