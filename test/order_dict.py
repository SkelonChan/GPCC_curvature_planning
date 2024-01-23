'''
Author: Skelon_Chan 269529898@msn.cn
Date: 2024-01-16 10:46:09
LastEditors: Skelon_Chan 269529898@msn.cn
LastEditTime: 2024-01-16 11:24:28
FilePath: \K_planning\optimal\order_dict.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''

# 有序字典的插入、修改，查询，删除
from collections import OrderedDict
import copy
import numpy as np
# 将字符串转换为可运行的代码指令
# code_str = "print('Hello, World!')"
# eval(code_str)
#

class OrderedKeyValueStructure:
    def __init__(self):
        self.data = OrderedDict()
        self.found_counts = {}

    def add_item(self, key, value):
        self.data[key] = value
        self.data = OrderedDict(sorted(self.data.items()))
        self.found_counts[key] = self.found_counts.get(key, 0)

    def modify_key(self, old_key, new_key):
        if old_key in self.data:
            value = self.data.pop(old_key)
            self.data[new_key] = value
            self.data = OrderedDict(sorted(self.data.items()))
            self.found_counts[new_key] = self.found_counts.get(new_key, 0) + self.found_counts.pop(old_key, 0)

    def delete_item(self, key):
        if key in self.data:
            del self.data[key]
            if key in self.found_counts:
                del self.found_counts[key]

    def get_ordered_items(self):
        return self.data

    def get_min_key_value(self):
        if self.data:
            min_key = next(iter(self.data))
            self.found_counts[min_key] += 1
            res_ = copy.deepcopy(self.data[min_key])
            if self.found_counts[min_key] == 2:
                del self.data[min_key]
                del self.found_counts[min_key]
            return min_key, res_
        else:
            return None

# 创建数据结构实例
okvs = OrderedKeyValueStructure()

# 添加键值对
key_1 = 3
val = ['3']
okvs.add_item(key_1, [])
# val.append('4')
# okvs.add_item(4, val)
okvs.add_item(2,['2'])

# 获取按键从小到大排列的键值对
print("原始键值对：")
print(okvs.get_ordered_items())
print("最小键对应的键值对：")
print(okvs.get_min_key_value()[1])
# okvs.add_item(7,['7'])
# print("增添新的键值对后：")
# print(okvs.get_ordered_items())
# print("增加新的键值对之后的最小的键值对：")
# print(okvs.get_min_key_value())
# print("两次查找最小键值对之后的键值对：")
# print(okvs.get_ordered_items())
# # # 获取修改键后的键值对
# print(okvs.get_ordered_items())
# print(okvs.get_min_key_value())