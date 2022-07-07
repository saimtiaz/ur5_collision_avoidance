#! /usr/bin/env python
import os
import numpy as np
from sklearn.cluster import KMeans, MiniBatchKMeans
import math

import clusterViz as clusViz

#TODO Remove when submitting final project
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time


def objWriter(objects, settingsFileName = "settings.yaml"):
    #Sort by cuboid and spheres
    cuboids = []
    spheres = []

    for object in objects:
        objClass = object['type']
        if objClass == 'sphere':
            spheres.append(object)
        elif objClass == 'cuboid':
            cuboids.append(object)

    #Remove existing settings.yaml
    #settingsFileName = "/home/simtiaz/ur5_collision_avoidance/catkin_ws/src/relaxed_ik_ros1/relaxed_ik_core/config/settings.yaml"
    #settingsFileName = "/home/simtiaz/catkin_ws/src/relaxed_ik_ros1/relaxed_ik_core/config/settings.yaml"
    #settingsFileName = "..\\..\\..\\..\\settingmisc\\settings.yaml"

    if (os.path.exists(settingsFileName)):
        os.remove(settingsFileName)

    
    #objective_mode should be set to noECA if there are no objects
    numSpheres = len(spheres)
    numCubes = len(cuboids)
    objMode = ''
    if numSpheres + numCubes == 0:
        objMode = 'noECA'
    else:
        objMode = 'ECAA'

    #Create header of the settings.yaml
    linkRadius = 0.05
    name = 'ur5_info.yaml'
    inputDevice = 'keyboard'

    f = open(settingsFileName, 'w')
    f.write('loaded_robot:\n')
    f.write('  name: ' + name + '\n')
    f.write('  link_radius: ' + str(linkRadius) + '\n')
    f.write('  objective_mode: ' + objMode + '\n')
    f.write('  input_device: ' + inputDevice + '\n')
    f.write('obstacles:\n')
    
    #For loop and write each cuboid
    if numCubes > 0:
        f.write('  cuboids:\n')
        counter = 1
        for cube in cuboids:
            cubeName = 'box' + str(counter)
            cubeScale = str(cube['scale'])
            cubeTrans = str(cube['translation'])
            cubeRot = str(cube['rotation'])
            animation = 'static'

            f.write('    - name: ' + cubeName + '\n')
            f.write('      scale: ' + cubeScale + '\n')
            f.write('      translation: ' + cubeTrans + '\n')
            f.write('      rotation: ' + cubeRot + '\n')
            f.write('      animation: ' + animation + '\n')

            counter = counter + 1
    #For loop and write each sphere
    if numSpheres > 0:
        f.write('  spheres:\n')
        counter = 1
        for sphere in spheres:
            sphereName = 'sphere' + str(counter)
            scale = sphere['scale']
            if type(scale) is list:
                scale = sphere['scale'][0]
            sphereScale = str(scale)
            sphereTrans = str(sphere['translation'])
            sphereRot = str(sphere['rotation'])
            animation = 'static'

            f.write('    - name: ' + sphereName + '\n')
            f.write('      scale: ' + sphereScale + '\n')
            f.write('      translation: ' + sphereTrans + '\n')
            f.write('      rotation: ' + sphereRot + '\n')
            f.write('      animation: ' + animation + '\n')

            counter = counter + 1

    f.close()

def pcd2cluster(fileName):
    fileHandle = open(fileName)
    lines = fileHandle.readlines()
    i = 0
    cluster = [[],[],[]]
    for line in lines:
        i = i + 1
        #TODO implement a way to tell when point data begins
        if (i < 12):
            continue
        numList = line.split()
        for j in range(0,3):
            num = float(numList[j])
            if math.isnan(num):
                continue
            cluster[j].append(float(numList[j]))
        
    return cluster

def cluster2obj(cluster, minPercentile = 10, maxPercentile = 90):
    #clusViz.boxViz(cluster)
    #clusViz.cubeVsSphereViz(cluster)
    roundingFactor = 5
    makeCubes = True
    xS = cluster[0]
    yS = cluster[1]
    zS = cluster[2]

    minX = float(np.percentile(xS, minPercentile))
    maxX = float(np.percentile(xS, maxPercentile))

    minY = float(np.percentile(yS, minPercentile))
    maxY = float(np.percentile(yS, maxPercentile))

    minZ = float(np.percentile(zS, minPercentile))
    maxZ = float(np.percentile(zS, maxPercentile))


    xTrans = round((maxX + minX)/2, roundingFactor)
    yTrans = round((maxY + minY)/2, roundingFactor)
    zTrans = round((maxZ + minZ)/2, roundingFactor)

    if makeCubes:
        xScale = round(maxX - minX, roundingFactor)
        yScale = round(maxY - minY, roundingFactor)
        zScale = round(maxZ - minZ, roundingFactor)

        cube = {'type' : 'cuboid', 'scale' : [xScale, yScale, zScale], 
                'rotation' : [0.0, 0.0, 0.0], 'translation' : [xTrans , yTrans, zTrans]}
        return cube

    else:
        scale = math.sqrt((maxX-minX)**2 + (maxY - minY)**2 + (maxZ - minZ)**2)
        sphere = {'type' : 'sphere', 'scale' : scale, 
                'rotation' : [0.0, 0.0, 0.0], 'translation' : [xTrans , yTrans, zTrans]}
        return sphere

def cluster2objKMeans(cluster, numCluster, minPercentile = 10, maxPercentile = 90, branchLevel = 1, method = 0, batch_size = 0):
    #clusViz.fullBoxViz(cluster)
    kClustersOld = [cluster]
    kClusters = []
    objList = []
    
    while branchLevel > 0:
        while len(kClustersOld) > 0:
            currCluster = kClustersOld.pop()
            newClusters = kMeansClustering(currCluster, numCluster, method = method, batch_size= batch_size)
            kClusters += newClusters
        branchLevel = branchLevel - 1
        kClustersOld = kClusters
        kClusters = []
    
    kClusters = kClustersOld


    for i in range(0, len(kClusters)):
        objList.append(cluster2obj(kClusters[i], minPercentile, maxPercentile))

    return objList
    
def kMeansClustering(cluster, numCluster, method = 0, batch_size = 0):
    X = np.column_stack((cluster[0], cluster[1], cluster[2]))

    #Could use metrics such as wcss, sihouette, or volume minimize
    kmeans = None
    if method == 0:
        kmeans = KMeans(n_clusters = numCluster).fit(X)
    elif method == 1:
        kmeans = MiniBatchKMeans(n_clusters=numCluster, batch_size=batch_size).fit(X)
    elif method == 2:
        #kmeans = BisectingKMeans(n_clusters=numCluster)
    
    kClusters = []

    for i in range(0, numCluster):
        kClusters.append([[],[],[]])
    for i in range(0, len(kmeans.labels_)):
        kClusters[kmeans.labels_[i]][0].append(cluster[0][i])
        kClusters[kmeans.labels_[i]][1].append(cluster[1][i])
        kClusters[kmeans.labels_[i]][2].append(cluster[2][i])
    return kClusters

def addCluster(fullCluster, cluster):

    xS = fullCluster[0] + cluster[0]
    yS = fullCluster[1] + cluster[1]
    zS = fullCluster[2] + cluster[2]

    newFullCluster = [xS, yS, zS]
    return newFullCluster

def range_filter(rawCluster, radius = 1.0):

    filteredTuples = list(zip(rawCluster[0], rawCluster[1], rawCluster[2]))
    filteredTuples = [x for x in filteredTuples if abs(x[0]) < radius and abs(x[1]) < radius and abs(x[2]) < radius]

    xS = [x[0] for x in filteredTuples]
    yS = [x[1] for x in filteredTuples]
    zS = [x[2] for x in filteredTuples]
    return [xS, yS, zS]

if __name__ == "__main__":

    ##PARAMETERS

    #Directory and file locations
    is_windows = True

    linux_point_cloud_dir = '/home/simtiaz/ur5_collision_avoidance/catkin_ws/pointCloudDir/'
    windows_point_cloud_dir = '..\\..\\..\\..\\pointClouds'
    point_cloud_dir = None

    linux_cik_setting_name = '/home/simtiaz/ur5_collision_avoidance/catkin_ws/src/relaxed_ik_ros1/relaxed_ik_core/config/settings.yaml'
    window_cik_setting_name = 'settings.yaml'
    cik_setting_name = None

    if is_windows:
        point_cloud_dir = windows_point_cloud_dir
        cik_setting_name = window_cik_setting_name
    else:
        point_cloud_dir = linux_point_cloud_dir
        cik_setting_name = linux_cik_setting_name
    

    #Filter parameters
    min_percentile = 3
    max_percentile = 97
    point_cloud_radius = 1.0

    #The total number of clusters is clusterSplit**branchLevel 
    #KMeans parameters
    run_standard = False
    cluster_split = 12
    branch_level = 2
    total_clusters = cluster_split ** branch_level

    #Can either be kmeans++ or random
    init_method = 'kmeans++'

    #Mini-batch Parameters
    #TODO study and implement mini_batch, online mini_batching, and bisecting kmeans
    run_mini_batch = True
    num_cores = 8
    #Batch size is ideally at least greater than 256 * number of cores on your machine
    batch_size = 256 * num_cores
    #Control early stopping based on number of iterations that inertia does not improve
    #TODO explore how this impacts run time
    max_num_improvement = 10

    #Bisecting K-means parameters
    #Number of iterations at each bisection
    run_bisection = False
    max_iter = 300

    #Keep track of runtime
    writeTic = time.perf_counter()

    ##Point Cloud File Processing
    file_list = os.listdir(point_cloud_dir)
    mega_cluster = [[], [], []]

    #Append each point cloud file
    for file in file_list:
        if(file.endswith(".pcd")):
            full_file_name = os.path.join(point_cloud_dir, file)
            cluster = pcd2cluster(full_file_name)
            mega_cluster = addCluster(mega_cluster, cluster)
    
    ##Range Filtering
    print("Size of the cluster before range filtering: ", len(mega_cluster[0]))
    mega_cluster = range_filter(mega_cluster, point_cloud_radius)
    print("Size of the cluster after range filtering: ", len(mega_cluster[0]))

    ##KMeans object extraction
    kmeans_method = 0
    if run_standard:
        kmeans_method = 0
        n_clusters = cluster_split
    elif run_mini_batch:
        kmeans_method = 1
        n_clusters = cluster_split ** branch_level
        branch_level = 1
    elif run_bisection:
        kmeans_method = 2

    obj_list = cluster2objKMeans(mega_cluster, numCluster=n_clusters, minPercentile=min_percentile, maxPercentile=max_percentile, branchLevel=branch_level, method = kmeans_method, batch_size=batch_size)

    ##Write objects to settings file
    objWriter(obj_list, settingsFileName = cik_setting_name)

    #Calculate total runtime
    writeToc = time.perf_counter()
    print(f"Extracted the objects to CIK in {writeToc - writeTic:0.4f} seconds")