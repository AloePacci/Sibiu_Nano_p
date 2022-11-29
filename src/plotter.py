# -*- coding: utf-8 -*-
"""
Created on Thu May 26 13:38:35 2022

@author: Alejandro
"""

import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import numpy as np
import pandas as pd
import time

data=pd.read_csv("./data/data 11.25.2022..16.35.csv",index_col=0)
data["mode"]=data["mode"].replace('MANUAL',1)
data["mode"]=data["mode"].replace('ALT_HOLD',2)
data["mode"]=data["mode"].replace('GUIDED',3)
data["state"]=data["state"].replace('CRITICAL',1)
data=data.groupby(data.index)
data=data.mean()

fig=plt.figure()
ax = fig.add_subplot(1, 1, 1)
datosx=data[~data["lat"].isnull()]["lat"]
datosy=data[~data["lon"].isnull()]["lon"]
ax.plot(datosx.values, datosy.values, color='tab:blue')
plt.xlabel("lat (º)")
plt.ylabel("lon (º)")
plt.title("position")
plt.savefig("./images/"+"position"+'.png', dpi=400)
for i in data:
    fig=plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    print(i)
    if i=="mode":
        datos=data[~data[i].isnull()][i]
        ax.plot(datos.index/1000, datos.values, color='tab:blue')
        ax.yaxis.set_ticks([1,2,3])
        ax.set_yticklabels(["MANUAL", "DEEP_HOLD", "GUIDED"])
    elif i=="state":
        datos=data[~data[i].isnull()][i]
        ax.plot(datos.index/1000, datos.values, color='tab:blue')
        ax.yaxis.set_ticks([1,2])
        ax.set_yticklabels(["CRITICAL", "Undefined"])
    else:
        datos=data[~data[i].isnull()][i]
        ax.plot(datos.index, datos.values, color='tab:blue')
    plt.plot()
    plt.xlabel('tiempo (s)')
    if i=="lat":
        plt.ylabel("x (m)")
        plt.title("x")
    elif i=="lon":
        plt.ylabel("y (m)")
        plt.title("y")
    else:
        plt.ylabel(i)
        plt.title(i)
    plt.savefig("./images/"+i+'.png', dpi=400)