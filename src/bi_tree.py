'''
Author: Skelon_Chan 269529898@msn.cn
Date: 2024-01-15 16:07:22
LastEditors: Skelon_Chan 269529898@msn.cn
LastEditTime: 2024-01-20 13:08:32
FilePath: \K_planning\optimal\bi_tree.py
'''
import warnings
import numpy as np
import copy
from collections import OrderedDict


class TreeNode:
    def __init__(self, value,seq):
        self.value = value
        self.left = None
        self.right = None
        self.seq = seq
        
class In_tree_value:
    def __init__(self, weight, path):
        self.weight = weight
        self.sequ = None
        self.path = path
        self.ori = None

class BinaryTree:
    def __init__(self):
        self.root = None

    def add_root_node(self, in_tree_value):
        if self.root is None:
            self.root = TreeNode(in_tree_value.path, in_tree_value.sequ)
            self.root.left = None  # Initialize left child
            self.root.right = None  # Initialize right child

    def add_node(self, in_tree_value):
        in_tree_value_copy = copy.deepcopy(in_tree_value)
        tmp_node = self.root
        res = []
        if in_tree_value_copy.sequ is not None:
            res = in_tree_value_copy.sequ.copy()
            for idx in in_tree_value_copy.sequ[:-1]:
                if idx == 'l':
                    tmp_node = tmp_node.left
                elif idx == 'r':
                    tmp_node = tmp_node.right
                else:
                    warnings.warn("Node sequence ERROR!")
        # tmp_node.seq = in_tree_value_copy.sequ
        if in_tree_value_copy.ori == 'l':
            tmp_node.left = TreeNode(in_tree_value_copy.path,in_tree_value_copy.sequ)
        elif in_tree_value_copy.ori == 'r':
            tmp_node.right = TreeNode(in_tree_value_copy.path,in_tree_value_copy.sequ)
        else:
            warnings.warn("Add tree Node orientation ERROR!")     
        return res

    def get_tree_path(self, in_tree_value):
        tmp_node = self.root
        res = tmp_node.value
        if in_tree_value.sequ is not None:
            for idx in in_tree_value.sequ:
                if idx == 'l':
                    tmp_node = tmp_node.left
                    res = np.concatenate((res,tmp_node.value))
                elif idx == 'r':
                    tmp_node = tmp_node.right
                    res = np.concatenate((res,tmp_node.value))
                else:
                    warnings.warn("Traversal ERROR!")
        return res
    
    def get_tree_path_sque(self, in_tree_value):
        tmp_node = self.root
        res = tmp_node.value
        # print(in_tree_value.sequ)
        if in_tree_value is not None:
            for idx in in_tree_value:
                if idx == 'l':
                    tmp_node = tmp_node.left
                    res = np.concatenate((res,tmp_node.value))
                elif idx == 'r':
                    tmp_node = tmp_node.right
                    res = np.concatenate((res,tmp_node.value))
                else:
                    warnings.warn("Traversal ERROR!")
        return res
    
    def get_node(self, in_tree_sequ):
        tmp_node = self.root
        if in_tree_sequ is not None:
            for idx in in_tree_sequ:
                if idx == 'l':
                    tmp_node = tmp_node.left
                elif idx == 'r':
                    tmp_node = tmp_node.right
                else:
                    warnings.warn("Traversal ERROR!")
        return tmp_node
    
    def print_node(self, node_sequ):
        "通过序列输出节点"
        tmp_node = self.root
        if node_sequ is not None:
            for idx in node_sequ:
                if idx == 'l':
                    tmp_node = tmp_node.left
                elif idx == 'r':
                    tmp_node = tmp_node.right
                else:
                    warnings.warn("Traversal ERROR!")
        return [tmp_node.value, tmp_node.seq]

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
            if self.found_counts[min_key] >= 2:
                del self.data[min_key]
                del self.found_counts[min_key]
            return [min_key, res_]
        else:
            return None
    















