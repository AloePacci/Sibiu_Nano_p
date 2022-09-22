# -*- coding: utf-8 -*-
"""
Created on Thu May 26 13:38:35 2022

@author: Alejandro
"""

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import time

data=pd.read_csv("out.csv")

for i in data:
    fig=plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(data[i], color='tab:blue')
    plt.plot()
    plt.xlabel('medida')
    plt.ylabel(i)
    plt.title(i)
    #ax.legend(["dron3", "dron2"])
    plt.savefig(i+'.png', dpi=400)
    #print("average ",name," ",i,"=",media(measures[i]))