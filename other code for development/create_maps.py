# -*- coding: utf-8 -*-
"""
Created on Thu Jun  9 13:40:02 2022

@author: Federico
"""
import sys
import json
import argparse
import matplotlib.pyplot as plt
import numpy as np
from sklearn.gaussian_process import GaussianProcessRegressor, kernels


def calculate_cell(coordinates, locations, width, height):
    for i in range(width):
        if locations[0][i][1] > coordinates[1]:
            startx = i
            break
    for j in range(height):
        if locations[j][0][0] < coordinates[0]:
            starty = j
            break
    return [starty, startx]


def generate_maps(map_name,Logitude,Latitude,sensor,values):

    # Load map
    if map_name=="Alamillo95x216":
        width = 216
        height = 95
    else:
        width = 393
        height = 343
    
    base_path="./"
    map_occupancy_mat = np.loadtxt(f"{base_path}/{map_name}plantilla.csv", delimiter=";")
    map_locations_mat_str = np.loadtxt(f"{base_path}/{map_name}grid.csv", dtype=str, delimiter=";")

    map_locations_mat = np.zeros((map_occupancy_mat.shape[0], map_occupancy_mat.shape[1], 2))
    for i in range(map_occupancy_mat.shape[0]):
        for j in range(map_occupancy_mat.shape[1]):
            map_locations_mat[i][j] = [float(mlms) for mlms in map_locations_mat_str[i][j].split(",")]

    points = []  # Prepare arguments
    for gps_point in [[float(y), float(x)] for y, x in zip(Latitude, Logitude)]:
        print(f" {gps_point}\n")
        points.append(calculate_cell(gps_point, map_locations_mat, width, height))
    points = np.array(points).reshape(-1, 2)
    values = np.array([float(v) for v in values])

    # Create vector of non-occupied cells
    non_occupied_cells = np.asarray(np.where(map_occupancy_mat == 0)).reshape(2, -1).T

    # GP stuff
    if sensor == "ORP":
        kernel = kernels.RBF(length_scale=10) + kernels.WhiteKernel(noise_level=0.1, noise_level_bounds='fixed')
    elif sensor == "PH":
        kernel = kernels.RBF(length_scale=10) + kernels.WhiteKernel(noise_level=0.1, noise_level_bounds='fixed')
    elif sensor == "WT":
        kernel = kernels.RBF(length_scale=10) + kernels.WhiteKernel(noise_level=0.1, noise_level_bounds='fixed')
    elif sensor == "COND":
        kernel = kernels.RBF(length_scale=10) + kernels.WhiteKernel(noise_level=0.1, noise_level_bounds='fixed')
    elif sensor == "DO":
        kernel = kernels.RBF(length_scale=10) + kernels.WhiteKernel(noise_level=0.1, noise_level_bounds='fixed')
    else:
        kernel = kernels.RBF(length_scale=10) + kernels.WhiteKernel(noise_level=0.1, noise_level_bounds='fixed')
    gp = GaussianProcessRegressor(kernel=kernel, normalize_y=True)
    gp.fit(points, values)

    # mean, std = gp.predict(non_occupied_cells, return_std=True)
    mean = gp.predict(non_occupied_cells)

    my_mean_map = np.full_like(map_occupancy_mat, np.nan)
    # my_std_map = np.full_like(map_occupancy_mat, np.nan)
    for idx, vec in enumerate(non_occupied_cells):
        my_mean_map[vec[0], vec[1]] = mean[idx]
        # my_std_map[vec[0], vec[1]] = std[idx]

    im = plt.imshow(my_mean_map, cmap='afmhot', zorder=3)
    plt.grid(True, zorder=0, color="white")
    plt.gca().set_facecolor('#eceff1')
    plt.plot(points[:, 1], points[:, 0], 'gP', zorder=3, markersize=4, label="Measurements")
    # horizontal colorbar with a title
    cb = plt.colorbar(im, orientation='horizontal', fraction=0.046, pad=0.08, label=f"{sensor} Value")

    plt.legend(fancybox=True, framealpha=1, shadow=True, ncol=1, loc="upper right")
    # plt.savefig(f'./{sensor}.png', dpi=300, bbox_inches='tight')
    plt.savefig(f'./{sensor}.png', dpi=300, bbox_inches='tight')
    plt.close()



def main():
    name=sys.argv
    #if data doesn't exist raise an error
    #♠if len(name)==1 or len(name)>2:
    #    raise
    
    #initialize variables
    
    ph=[]
    veh_num=[]
    Latitude=[]
    Longitude=[]
    smart_water_battery=[]
    Disolved_Oxygen=[]
    temperature=[]
    conductivity=[]
    oxidation_reduction_potential=[]
        
    #fetch data
    with open("test.txt", 'r') as f:
        for read in f:
            values=json.loads(read)
            #veh_num.append(values['veh_num'])
            Latitude.append(values['Latitude'])
            Longitude.append(values['Longitude'])
            #smart_water_battery.append(values['smart_water_battery'])
            ph.append(values['ph'])
            #Disolved_Oxygen.append(values['Disolved_Oxygen'])
            #temperature.append(values['temperature'])
            #conductivity.append(values['conductivity'])
            #oxidation_reduction_potential.append(values['oxidation_reduction_potential'])
            
    #generate maps
    #if Latitude[0]>37.41:
    #    mapa="Alamillo95x216"
    #else:
    #    mapa="Loyola121x239"
    mapa="zufre"
    #generate_maps(mapa,Longitude,Latitude,"ORP",oxidation_reduction_potential)
    generate_maps(mapa,Longitude,Latitude,"PH",ph)
    #generate_maps(mapa,Longitude,Latitude,"WT",temperature)
    #generate_maps(mapa,Longitude,Latitude,"COND",conductivity)
    #generate_maps(mapa,Longitude,Latitude,"DO",Disolved_Oxygen)

if __name__ == '__main__':
    main()