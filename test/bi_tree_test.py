'''
Author: Skelon_Chan 269529898@msn.cn
Date: 2024-01-16 08:51:51
LastEditors: Skelon_Chan 269529898@msn.cn
LastEditTime: 2024-01-16 15:11:37
FilePath: \K_planning\optimal\bi_tree_test.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import bi_tree as bt
import numpy as np
import copy
node1 = bt.In_tree_value(1, np.ones((4, 4)) * 1)
node2 = bt.In_tree_value(2, np.ones((3, 4)) * 2)
node3 = bt.In_tree_value(3, np.ones((2, 4)) * 3)
node4 = bt.In_tree_value(4, np.ones((3,4)) * 4)
node5 = bt.In_tree_value(5, np.ones((4,4)) * 5)
node6 = bt.In_tree_value(6, np.ones((2,4)) * 6)
node7 = bt.In_tree_value(0.2, np.ones((3,4)) * 7)

btree = bt.BinaryTree()

node2.ori = 'l'
node2.sequ = ['l']

node3.ori = 'r'
node3.sequ = ['r']

node4.ori = 'l'
node4.sequ = ['r', 'l']

node5.ori = 'r'
node5.sequ = ['r','r']

node6.ori = 'r'
node6.sequ = ['r','l','r']

node7.ori = 'l'
node7.sequ = ['r','l','r','l']

btree.add_root_node(node1)
btree.add_node(node2)
btree.add_node(node3)
btree.add_node(node4)
btree.add_node(node5)
btree.add_node(node6)
btree.add_node(node7)

# res2 = btree.get_tree_path_sque(['r','l','r','l'])
# res = btree.get_tree_path(node7)
res2 = btree.print_node(['r','l','r','l'])


print(res2)


