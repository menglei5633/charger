# -*- coding: utf-8 -*-
"""
Created on Wed May 22 14:27:52 2019

@author: 拔凉拔凉冰冰凉
"""

import matplotlib.pyplot as plt
import numpy as np

result = []
filename = "./test_A_s_result.txt"
with open(filename, "r", encoding="utf-8") as f:
    lines = f.readlines()
#    print(len(lines))
    for line in lines:
        line = line.split("\n")[0]
        line = line[:-1]
#        print(line)
        line = line.split(" ")
        r = []
        for s in line:
#            print(s)
            r.append(float(s))
        result.append(r)
    f.close()
    
al_name = ['centralized(c=4)', "greedyCover", "greedyUtility", "centralized(c=1)"]
color = ["r", "b", "g", "orange"]

print(result)

x = np.arange(30, 390, 30)

plt.figure(figsize=(20, 17))
plt.xticks(x)
plt.ylim((0.3, 0.8))
for i in range(len(result)):
    plt.plot(x, result[i], c=color[i])
plt.show()