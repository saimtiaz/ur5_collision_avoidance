#! /usr/bin/env python
import os
import numpy as np
from sklearn.cluster import KMeans, MiniBatchKMeans, DBSCAN
import math
from statistics import mean
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

def cluster2obj(cluster, pVal = 3, makeCubes = True):
    #clusViz.boxViz(cluster)
    #clusViz.cubeVsSphereViz(cluster)
    minPercentile = pVal
    maxPercentile = 100 - pVal
    roundingFactor = 5

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

#def cluster2objKMeans(cluster, numCluster, minPercentile = 10, maxPercentile = 90, branchLevel = 1, method = 0, batch_size = 0):
def cluster2objKMeans(cluster, pVal = 3, params = None):
    if params is None:
        params = [{
            'type' : 'kmeans', 
            'cluster_split' : 12, 
            'init_method' : 'kmeans++'
        }]

    #clusViz.fullBoxViz(cluster)
    kClustersOld = [cluster]
    kClusters = []
    objList = []
    
    for param in params:
        while len(kClustersOld) > 0:
            currCluster = kClustersOld.pop()
            newClusters = kMeansClustering(currCluster, param = param)
            kClusters += newClusters
        kClustersOld = kClusters
        kClusters = []
    
    kClusters = kClustersOld


    for i in range(0, len(kClusters)):
        objList.append(cluster2obj(kClusters[i], pVal = pVal))

    return objList
    
def kMeansClustering(cluster, param = None):
    if param is None:
        param = {
            'type' : 'kmeans', 
            'cluster_split' : 12, 
            'init_method' : 'k-means++'
        }

    method = param['type']
    

    X = np.column_stack((cluster[0], cluster[1], cluster[2]))
    #Minimum cluster size is 20 points

    if (len(cluster[0]) < 20):
        return cluster
    #Could use metrics such as wcss, sihouette, or volume minimize
    kmeans = None
    numClusters = 0
    if method == 'kmeans':
        numClusters = param['cluster_split']
        kmeans = KMeans(
            n_clusters = param['cluster_split'], 
            init = param['init_method']
        ).fit(X)

    elif method == 'mini':
        numClusters = param['cluster_split']
        kmeans = MiniBatchKMeans(
            init = param['init_method'],
            n_clusters= param['cluster_split'],
            batch_size= param['batch_size'],
            max_no_improvement= param['max_num_improvement'],
            max_iter = param['max_iter']
        ).fit(X)       
    elif method == 'dbscan':
        kmeans = DBSCAN(
            eps = param['eps'],
            min_samples = param['min_samples'],
            leaf_size = param['leaf_size'],
            n_jobs = param['n_jobs']
        ).fit(X)
        labels = kmeans.labels_
        numClusters = len(set(labels)) - (1 if -1 in labels else 0)
    
    kClusters = []

    for i in range(0, numClusters):
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
    p_val = 1
    point_cloud_radius = 1.0

    #Clustering params
    kmeans_param = {
        'type' : 'kmeans',
        'cluster_split' : 10,
        'init_method' : 'k-means++'
    }
    orig_param = {
        'type' : 'kmeans',
        'cluster_split' : 150,
        'init_method' : 'k-means++'
    }
    mini_kmeans_param = {
        'type' : 'mini',
        'init_method' : 'k-means++',
        'cluster_split' : 10,
        'batch_size' : (256*8),
        'max_num_improvement' : 10,
        'max_iter' : 100
    }
    dbscan_param = {
        'type' : 'dbscan',
        'eps' : 0.05,
        'min_samples' : 15,
        #Leaf size must be at least 1
        'leaf_size' : 30,
        'n_jobs' : -1
    }

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

    #Clustering
    k = kmeans_param
    m = mini_kmeans_param
    d = dbscan_param 
    o = orig_param
    
    listOfParamLists = [
        [o],
        [k],
        [m],
        [d],
        [k, k],
        [k, m],
        [k, d],
        [m, m],
        [m, d],
        [m, k],
        [d, m],
        [d, d],
        [d, k]
    ]
    obj_list = None

    volEfficiencies = []
    timeEfficiencies = []
    for param_list in listOfParamLists:
        loops = 1
        clusterTic = time.perf_counter()
        for i in range(0, loops):
            obj_list = cluster2objKMeans(mega_cluster, pVal = p_val, params = param_list)
        clusterToc = time.perf_counter()
        print(f"Clustered objects in {((clusterToc- clusterTic) / loops):0.4f} seconds")
        percentInBox = clusViz.pointsInBox(mega_cluster, obj_list)
        totalVol = clusViz.totalVolume(obj_list)
        efficiency = (totalVol / percentInBox) * 1000
        volEfficiencies.append(efficiency)
        timeEfficiency = (((clusterToc- clusterTic) / loops) / percentInBox) * 1000
        timeEfficiencies.append(timeEfficiency)
        print(percentInBox, totalVol, efficiency, timeEfficiency)
        for param in param_list:
            print(param['type'])

    volAvg = mean(volEfficiencies)
    timeAvg = mean(timeEfficiencies)
    trueEfficiencies = []
    for i in range(0, len(volEfficiencies)):
        trueEfficiency = ((volEfficiencies[i] / volAvg) * 0.75) + ((timeEfficiencies[i] / timeAvg) * 0.25)
        trueEfficiencies.append(trueEfficiency)
    print(trueEfficiencies)

    print(listOfParamLists[trueEfficiencies.index(min(trueEfficiencies))])
    print(listOfParamLists[volEfficiencies.index(min(volEfficiencies))])
    print(listOfParamLists[timeEfficiencies.index(min(timeEfficiencies))])

    #Visualize objects
    #clusViz.objectViz(mega_cluster, obj_list)

    ##Write objects to settings file
    objWriter(obj_list, settingsFileName = cik_setting_name)

    #Calculate total runtime
    writeToc = time.perf_counter()
    print(f"Extracted the objects to CIK in {writeToc - writeTic:0.4f} seconds")
    