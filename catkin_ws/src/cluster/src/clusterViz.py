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

def fullViz():
    return

def kmeansViz():
    return

def euclideanViz():
    return

def clusterHelper(cluster, minPercentile, maxPercentile, isCube):
    roundingFactor = 5
    makeCubes = isCube
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
        return {'x' : [minX, maxX], 'y' : [minY, maxY], 'z' : [minZ, maxZ], 'xLoc' : xTrans, 'yLoc' : yTrans, 'zLoc' : zTrans} 

    else:
        radius = math.sqrt((maxX-minX)**2 + (maxY - minY)**2 + (maxZ - minZ)**2) / 2
        return {'x' : [minX, maxX], 'y' : [minY, maxY], 'z' : [minZ, maxZ], 'xLoc' : xTrans, 'yLoc' : yTrans, 'zLoc' : zTrans, 'r' : radius} 

def cubeVsSphereTime(cluster):
    #Compare cube vs sphere time by cluster number
    minPercentile = 3
    maxPercentile = 97
    

    clusterNums = []
    timeCube = []
    timeSphere = []

    #objType = [True, False]
    objType = [True]
    color = ['red', 'blue', 'green', 'yellow']
    increment = [200, 100, 100, 100]
    increment = [
        [20, 100, 200, 500, 1000, 1500],
        [20, 49, 64, 81, 100, 144, 225, 400, 625, 900, 1024, 1225, 1444, 1521],
        [20, 27, 64, 125, 216, 343, 512, 729, 1000, 1331, 1728],
        [16, 81, 256, 625, 1296]
    ]
    for coreBranchLevel in range(2, 5):
        clusterNums = []
        timeCube = []
        currColor = color[coreBranchLevel - 1]
        currIncrement = increment[coreBranchLevel - 1]
        for totalCluster in currIncrement: 
            print(totalCluster)
            numCluster = round(totalCluster ** (1./coreBranchLevel))
            clusterNums.append(numCluster ** coreBranchLevel)
            for isCube in objType:
                branchLevel = coreBranchLevel
                tic = time.perf_counter()
                kClustersOld = [cluster]
                kClusters = []
                
                while branchLevel > 0:
                    while len(kClustersOld) > 0:
                        currCluster = kClustersOld.pop()
                        newClusters = [currCluster]
                        if len(currCluster[0]) > numCluster:
                            newClusters = clus.kMeansClustering(currCluster, numCluster)
                        kClusters += newClusters
                    branchLevel = branchLevel - 1
                    kClustersOld = kClusters
                    kClusters = []
                
                kClusters = kClustersOld

                for i in range(0, len(kClusters)):
                    currCluster = kClusters[i]
                    objParam = clusterHelper(currCluster, minPercentile, maxPercentile, isCube=isCube)
                toc = time.perf_counter()

                timeTot = toc - tic
                if isCube:
                    timeCube.append(timeTot)
                else:
                    timeSphere.append(timeTot)
        # Plot a simple line chart
        print(clusterNums, timeCube)
        plt.plot(clusterNums, timeCube, label='Cubic Time', color = currColor)

        # # Plot another line on the same chart/graph
        # plt.plot(clusterNums, timeSphere, label='Spherical Time')

    plt.legend()
    plt.show()




def fullBoxViz(cluster, numCluster = 20):
    #Given the entire point cloud, show how it is all boxed up
    #cubeVsSphereTime(cluster)
    minPercentile = 3
    maxPercentile = 97
    numCluster = 12
    branchLevel = 2
    isCube = True

    # if numCluster > 4:
    #     fullBoxViz(cluster, numCluster - 1)

    fig = plt.figure(figsize=(4,4))
    ax = fig.add_subplot(111, projection = '3d')


    kClustersOld = [cluster]
    kClusters = []
    
    while branchLevel > 0:
        while len(kClustersOld) > 0:
            currCluster = kClustersOld.pop()
            newClusters = clus.kMeansClustering(currCluster, numCluster)
            kClusters += newClusters
        branchLevel = branchLevel - 1
        kClustersOld = kClusters
        kClusters = []
    
    kClusters = kClustersOld

    totalVol = 0
    for i in range(0, len(kClusters)):
        r = random.random()
        b = random.random()
        g = random.random()
  
        color = (r, g, b)
        currCluster = kClusters[i]
        objParam = clusterHelper(currCluster, minPercentile, maxPercentile, isCube=isCube)

        if isCube:
            xPair = objParam['x']
            yPair = objParam['y']
            zPair = objParam['z']

            volume = (xPair[1] - xPair[0]) * (yPair[1] - yPair[0]) * (zPair[1] - zPair[0])
            totalVol = totalVol + volume

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
        
    
        xS = currCluster[0]
        yS = currCluster[1]
        zS = currCluster[2]
        ax.scatter(xS, yS, zS, marker = 'x', alpha = 0.2)
    #print(totalVol, numCluster)
    ax.set_title("Cube")
    plt.show()

def boxViz(cluster):
    roundingFactor = 5
    xS = cluster[0]
    yS = cluster[1]
    zS = cluster[2]

    fig = plt.figure(figsize=(4,4))
    ax = fig.add_subplot(111, projection = '3d')

    ax.scatter(xS, yS, zS, marker = 'x')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    minMax = [(10, 90), (5, 95), (1, 99)]
    colors = ['green', 'red', 'blue']
    for i in range(len(minMax)):
        pair = minMax[i]
        color = colors[i]

        minPercentile = pair[0]
        maxPercentile = pair[1]


        minX = float(np.percentile(xS, minPercentile))
        maxX = float(np.percentile(xS, maxPercentile))

        minY = float(np.percentile(yS, minPercentile))
        maxY = float(np.percentile(yS, maxPercentile))

        minZ = float(np.percentile(zS, minPercentile))
        maxZ = float(np.percentile(zS, maxPercentile))


        xTrans = round((maxX + minX)/2, roundingFactor)
        yTrans = round((maxY + minY)/2, roundingFactor)
        zTrans = round((maxZ + minZ)/2, roundingFactor)

        ax.scatter(xTrans, yTrans, zTrans, marker = '+')

        #Draw cube
        for s, e in combinations(np.array(list(product([minX, maxX], [minY, maxY], [minZ, maxZ]))), 2):
            if np.sum(np.abs(s-e)) == maxX-minX:
                ax.plot3D(*zip(s, e), color=color)
            if np.sum(np.abs(s-e)) == maxY-minY:
                ax.plot3D(*zip(s, e), color=color)
            if np.sum(np.abs(s-e)) == maxZ-minZ:
                ax.plot3D(*zip(s, e), color=color)

    ax.set_title("Cube")
    plt.show()
    #Given a cluster, return a viz 


def cubeVsSphereViz(cluster):
    roundingFactor = 5
    xS = cluster[0]
    yS = cluster[1]
    zS = cluster[2]

    fig = plt.figure(figsize=(4,4))
        
    ax = fig.add_subplot(111, projection = '3d')
    ax.set_title("Sphere")

    ax.scatter(xS, yS, zS, marker = 'x')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    minMax = [(5, 95)]
    for i in range(len(minMax)):
        pair = minMax[i]
        colorSphere = 'red'
        colorCube = 'blue'

        minPercentile = pair[0]
        maxPercentile = pair[1]


        minX = float(np.percentile(xS, minPercentile))
        maxX = float(np.percentile(xS, maxPercentile))

        minY = float(np.percentile(yS, minPercentile))
        maxY = float(np.percentile(yS, maxPercentile))

        minZ = float(np.percentile(zS, minPercentile))
        maxZ = float(np.percentile(zS, maxPercentile))


        xTrans = round((maxX + minX)/2, roundingFactor)
        yTrans = round((maxY + minY)/2, roundingFactor)
        zTrans = round((maxZ + minZ)/2, roundingFactor)

        radius = (math.sqrt((maxX-minX)**2 + (maxY - minY)**2 + (maxZ - minZ)**2))/2
        print(radius)
        ax.scatter(xTrans, yTrans, zTrans, marker = '+')

        #Draw sphere
        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j] 
        x = (np.cos(u) * np.sin(v) * radius ) + xTrans
        y = (np.sin(u) * np.sin(v) * radius) + yTrans
        z = (np.cos(v) * radius) + zTrans
        ax.plot_wireframe(x, y, z, color=colorSphere, rstride = 2, cstride = 2)

        #Draw cube
        for s, e in combinations(np.array(list(product([minX, maxX], [minY, maxY], [minZ, maxZ]))), 2):
            if np.sum(np.abs(s-e)) == maxX-minX:
                ax.plot3D(*zip(s, e), color=colorCube)
            if np.sum(np.abs(s-e)) == maxY-minY:
                ax.plot3D(*zip(s, e), color=colorCube)
            if np.sum(np.abs(s-e)) == maxZ-minZ:
                ax.plot3D(*zip(s, e), color=colorCube)




    plt.show()
    #Given a cluster, return a viz of the cluster and the sphere itself
    return

def boxOrSphereViz():
    #Given a specific cluster, show it being represented by either a box or a sphere
    #Will also would want to show how the full representation looks like with each strategy
    #tablescene may be a good idea
    return

def outlierViz():
    return

def kTreeViz():
    return

if __name__ == "__main__":
    pointCloudDir = '..\\..\\..\\..\\pointClouds'
    fileList = os.listdir(pointCloudDir)