# -*- coding: utf-8 -*-
"""
Created on Thu May 26 13:38:35 2022

@author: Alejandro
"""

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import time

data=pd.read_csv("out.csv",index_col=0)
data["mode"]=data["mode"].replace('MANUAL',0)
data["mode"]=data["mode"].replace('ALT_HOLD',1)
data["state"]=data["state"].replace('CRITICAL',0)
data=data.groupby(data.index)
data=data.mean()

for i in data:
    fig=plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    print(i)
    if i=="mode":
        
        ax.plot(data[~data[i].isnull()][i], color='tab:blue')
        ax.set_yticklabels(["MANUAL", "DEEP_HOLD"])
    elif i=="state":
        ax.plot(data[~data[i].isnull()][i], color='tab:blue')
        ax.set_yticklabels(["CRITICAL"])
    else:
        ax.plot(data[~data[i].isnull()][i], color='tab:blue')
    plt.plot()
    plt.xlabel('tiempo')
    plt.ylabel(i)
    plt.title(i)
    #ax.legend(["dron3", "dron2"])
    plt.savefig("./images/"+i+'.png', dpi=400)
    #print("average ",name," ",i,"=",media(measures[i]))