from typing_extensions import runtime
import cluster2obj as clus
import time
import os

from itertools import product, combinations
import numpy as np
from sklearn.cluster import KMeans
import math

import random as random


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

def initializeCluster():
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
    point_cloud_radius = 1.0


    ##Point Cloud File Processing
    file_list = os.listdir(point_cloud_dir)
    mega_cluster = [[], [], []]

    #Append each point cloud file
    for file in file_list:
        if(file.endswith(".pcd")):
            full_file_name = os.path.join(point_cloud_dir, file)
            cluster = clus.pcd2cluster(full_file_name)
            mega_cluster = clus.addCluster(mega_cluster, cluster)

    ##Range Filtering
    print("Size of the cluster before range filtering: ", len(mega_cluster[0]))
    mega_cluster = clus.range_filter(mega_cluster, point_cloud_radius)
    print("Size of the cluster after range filtering: ", len(mega_cluster[0]))
    return mega_cluster


def kClusterHelper(cluster, params = None):
    if params is None:
        params = [{
            'type' : 'kmeans', 
            'cluster_split' : 12, 
            'init_method' : 'k-means++'
        }]

    kClustersOld = [cluster]

    kClusters = []

    clusterTic = time.perf_counter()
    for param in params:
        while len(kClustersOld) > 0:
            currCluster = kClustersOld.pop()
            newClusters = clus.kMeansClustering(currCluster, param = param)
            kClusters += newClusters
        kClustersOld = kClusters
        kClusters = []
    clusterToc = time.perf_counter()
    
    kClusters = kClustersOld
    return kClusters, (clusterToc-clusterTic)

def objListHelper(kClusters, pVal = 1):
    obj_list = []
    for i in range(0, len(kClusters)):
        obj_list.append(clus.cluster2obj(kClusters[i], pVal = pVal))
    return obj_list

def clusterSplitViz(kClusters, title = "Viz"):
    fig = plt.figure(figsize=(4,4))
    ax = fig.add_subplot(111, projection = '3d')

    
    for i in range(0, len(kClusters)):
        r = random.random()
        b = random.random()
        g = random.random()
  
        color = (r, g, b)

        currCluster = kClusters[i]
        xS = currCluster[0]
        yS = currCluster[1]
        zS = currCluster[2]
        ax.scatter(xS, yS, zS, marker = 'x', alpha = 0.3)
    ax.view_init(12, -122)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_title(title)
    plt.show()
    return

def fullPointCloudViz(cluster):
    fig = plt.figure(figsize=(4,4))
    ax = fig.add_subplot(111, projection = '3d')


    xS = cluster[0]
    yS = cluster[1]
    zS = cluster[2]

    ax.scatter(xS, yS, zS, marker = 'x', alpha = 0.3)
    ax.view_init(12, -122)
    ax.set_xlabel('x')
    ax.xaxis.label.set_size(10)
    ax.set_ylabel('y')
    ax.yaxis.label.set_size(10)
    ax.set_zlabel('z')
    ax.zaxis.label.set_size(10)
    ax.set_title("Point Cloud")

    plt.show()
    return

def clusterHelper(cluster, pVal = 1, makeCubes = True):
    roundingFactor = 5
    xS = cluster[0]
    yS = cluster[1]
    zS = cluster[2]

    minPercentile = pVal
    maxPercentile = 100 - pVal

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
        return {'x' : [minX, maxX], 'y' : [minY, maxY], 'z' : [minZ, maxZ], 'xLoc' : xTrans, 'yLoc' : yTrans, 'zLoc' : zTrans} 

    else:
        radius = math.sqrt((maxX-minX)**2 + (maxY - minY)**2 + (maxZ - minZ)**2) / 2
        return {'x' : [minX, maxX], 'y' : [minY, maxY], 'z' : [minZ, maxZ], 'xLoc' : xTrans, 'yLoc' : yTrans, 'zLoc' : zTrans, 'r' : radius}


def boxViz(cluster, kClusters, makeCubes = True, title = "Viz", runTime = 0):

    fig = plt.figure(figsize=(4,4))
    ax = fig.add_subplot(111, projection = '3d')
    xS = cluster[0]
    yS = cluster[1]
    zS = cluster[2]

    ax.scatter(xS, yS, zS, marker = 'x', alpha = 0.1)
    totalVol = 0
    for i in range(0, len(kClusters)):
        color = 'red'
        currCluster = kClusters[i]
        objParam = clusterHelper(currCluster, makeCubes= makeCubes)


        xPair = objParam['x']
        yPair = objParam['y']
        zPair = objParam['z']

        volume = (xPair[1] - xPair[0]) * (yPair[1] - yPair[0]) * (zPair[1] - zPair[0])
        totalVol = totalVol + volume

        if makeCubes:
            #Draw cube
            for s, e in combinations(np.array(list(product(xPair, yPair, zPair))), 2):
                if np.sum(np.abs(s-e)) == xPair[1]-xPair[0]:
                    ax.plot3D(*zip(s, e), color=color)
                if np.sum(np.abs(s-e)) == yPair[1]-yPair[0]:
                    ax.plot3D(*zip(s, e), color=color)
                if np.sum(np.abs(s-e)) == zPair[1]-zPair[0]:
                    ax.plot3D(*zip(s, e), color=color)
        else:
            radius = objParam['r']
            volume = math.pi * radius**3 * (4/3)
            totalVol = totalVol + volume
            xTrans = objParam['xLoc']
            yTrans = objParam['yLoc']
            zTrans = objParam['zLoc']

            #Draw sphere
            u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j] 
            x = (np.cos(u) * np.sin(v) * radius ) + xTrans
            y = (np.sin(u) * np.sin(v) * radius) + yTrans
            z = (np.cos(v) * radius) + zTrans
            ax.plot_wireframe(x, y, z, color=color, rstride = 2, cstride = 2)

    ax.view_init(12, -122)

    ax.set_title(title)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')


    volString = "Total Volume: " + str(round(totalVol*100*100, 2)) + "cm^3"
    timeString = f"Clustered objects in {(runTime):0.4f} seconds"


    ax.annotate(volString, xy=(0.5, 1),xycoords='axes fraction', fontsize=8, horizontalalignment='right', verticalalignment='top')
    ax.annotate(timeString, xy=(0.5, 0.95),xycoords='axes fraction', fontsize=8, horizontalalignment='right', verticalalignment='top')

    plt.show()


def dbMiniViz(cluster, kClusters, numClusters = 8, initRunTime = 0):

    mini_kmeans_param = [{
        'type' : 'mini',
        'init_method' : 'k-means++',
        'cluster_split' : numClusters,
        'batch_size' : (256*8),
        'max_num_improvement' : 10,
        'max_iter' : 100
    }]

    runTimeTot = initRunTime
    allKClusters = []
    for currCluster in kClusters:
        currKClusters, runTime = kClusterHelper(currCluster, params = mini_kmeans_param)
        runTimeTot = runTimeTot + runTime
        allKClusters += currKClusters
    
    #Point cloud Viz
    fullPointCloudViz(cluster)

    #Cluster Split Viz
    titleString = "Mini-Batch K-means, k = " + str(numClusters)
    clusterSplitViz(allKClusters, title = titleString)

    #Cluster Box Viz
    titleString = "Mini-Batch K-means, k = " + str(numClusters)
    boxViz(cluster, allKClusters, makeCubes= True, runTime = runTimeTot, title=titleString)

def dbscanViz(cluster):

    dbscan_param = [{
        'type' : 'dbscan',
        'eps' : 0.03,
        'min_samples' : 15,
        #Leaf size must be at least 1
        'leaf_size' : 15,
        'n_jobs' : -1
    }]

    kClusters, runtime = kClusterHelper(cluster, params = dbscan_param)

    #Point cloud Viz
    fullPointCloudViz(cluster)

    #Cluster Split Viz
    titleString = "DBSCAN Clustering"
    clusterSplitViz(kClusters, title = titleString)

    return kClusters, runtime

def kMeansViz(cluster ,numClusters = 12, pVal = 1):
    param = [{
            'type' : 'kmeans', 
            'cluster_split' : numClusters, 
            'init_method' : 'k-means++'
        }]

    kClusters, runTime = kClusterHelper(cluster, params = param)

    #Point Cloud Viz
    fullPointCloudViz(cluster)

    #Cluster Split Viz
    titleString = "K-means Clustering, k = " + str(numClusters)
    clusterSplitViz(kClusters, title = titleString)

    #Cluster Box Viz
    titleString = "K-means Bounding Boxes, k = " + str(numClusters)
    boxViz(cluster, kClusters, makeCubes= True, runTime = runTime, title = titleString)


#Create a figure that compares the bounding boxes based on pVals
def pViz(cluster, pVals):
    xS = cluster[0]
    yS = cluster[1]
    zS = cluster[2]

    fig = plt.figure(figsize=(4,4))
    ax = fig.add_subplot(111, projection = '3d')

    ax.scatter(xS, yS, zS, marker = 'x', label = 'Cluster Point Cloud')

    colors = ['green', 'red', 'blue']
    i = 0

    for pVal in pVals:
        i = i+1
        color = colors[i%3]

        minPercentile = pVal
        maxPercentile = 100 - pVal

        minX = float(np.percentile(xS, minPercentile))
        maxX = float(np.percentile(xS, maxPercentile))

        minY = float(np.percentile(yS, minPercentile))
        maxY = float(np.percentile(yS, maxPercentile))

        minZ = float(np.percentile(zS, minPercentile))
        maxZ = float(np.percentile(zS, maxPercentile))

        labelTag = 'Percentage outliers: ' + str(pVal) + "%"
        ax.scatter(minX,minY,minZ, color = color, label = labelTag, alpha = 0.3)
        #Draw cube
        for s, e in combinations(np.array(list(product([minX, maxX], [minY, maxY], [minZ, maxZ]))), 2):
            if np.sum(np.abs(s-e)) == maxX-minX:
                ax.plot3D(*zip(s, e), color=color)
            if np.sum(np.abs(s-e)) == maxY-minY:
                ax.plot3D(*zip(s, e), color=color)
            if np.sum(np.abs(s-e)) == maxZ-minZ:
                ax.plot3D(*zip(s, e), color=color)




    ax.legend()
    ax.set_title("Bounding Boxes: Percentage Outlier Value Selection")
    ax.set_xlabel('x')
    ax.xaxis.label.set_size(10)
    ax.set_ylabel('y')
    ax.yaxis.label.set_size(10)
    ax.set_zlabel('z')
    ax.zaxis.label.set_size(10)
    ax.view_init(45, 45)
    plt.show()

#Select a random cluster for pVal visualization
def randomCluster(cluster):
    param = [{
        'type' : 'kmeans', 
        'cluster_split' : 60, 
        'init_method' : 'k-means++'
    }]

    kClusters, runtime = kClusterHelper(cluster, params = param)

    return kClusters[4]


if __name__ == "__main__":

    #Initialize the point cloud for object extraction and visualtization
    cluster = initializeCluster()

    #Create a random cluster
    soloCluster = randomCluster(cluster)

    #p-val viz
    pVals = [10, 4, 1]
    pViz(soloCluster, pVals)
    
    #K selection viz
    k = [24, 48, 99]
    for numCluster in k:
        kMeansViz(cluster, numClusters= numCluster)
        
    #DBSCAN viz
    kClusters, runtime = dbscanViz(cluster)
    print("DBSCAN runtime: ", runtime)

    #DBSCAN + minibatch viz
    k = [8, 16, 33]
    for numCluster in k:
        dbMiniViz(cluster, kClusters, numClusters = numCluster, initRunTime = runtime)