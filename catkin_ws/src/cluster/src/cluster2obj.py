#! /usr/bin/env python
import os
import numpy as np
from sklearn.cluster import KMeans
import math

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
    settingsFileName = "/home/simtiaz/ur5_collision_avoidance/catkin_ws/src/relaxed_ik_ros1/relaxed_ik_core/config/settings.yaml"
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
    xS = cluster[0]
    yS = cluster[1]
    zS = cluster[2]

    minX = float(np.percentile(xS, minPercentile))
    maxX = float(np.percentile(xS, maxPercentile))

    minY = float(np.percentile(yS, minPercentile))
    maxY = float(np.percentile(yS, maxPercentile))

    minZ = float(np.percentile(zS, minPercentile))
    maxZ = float(np.percentile(zS, maxPercentile))

    xScale = round(maxX - minX, 4)
    yScale = round(maxY - minY, 4)
    zScale = round(maxZ - minZ, 4)

    xTrans = round((maxX + minX)/2, 4)
    yTrans = round((maxY + minY)/2, 4)
    zTrans = round((maxZ + minZ)/2, 4)

    cube = {'type' : 'cuboid', 'scale' : [xScale, yScale, zScale], 
             'rotation' : [0.0, 0.0, 0.0], 'translation' : [xTrans ,yTrans,zTrans]}
    return cube

def cluster2objKMeans(cluster, numCluster, minPercentile = 10, maxPercentile = 90, branchLevel = 1):
    kClustersOld = [cluster]
    kClusters = []
    objList = []
    
    while branchLevel > 0:
        while len(kClustersOld) > 0:
            currCluster = kClustersOld.pop()
            newClusters = kMeansClustering(currCluster, numCluster)
            kClusters += newClusters
        branchLevel = branchLevel - 1
        kClustersOld = kClusters
        kClusters = []
    
    kClusters = kClustersOld


    for i in range(0, len(kClusters)):
        objList.append(cluster2obj(kClusters[i], minPercentile, maxPercentile))

    return objList
    
def kMeansClustering(cluster, numCluster):
    X = np.column_stack((cluster[0], cluster[1], cluster[2]))

    #Could use metrics such as wcss, sihouette, or volume minimize
    kmeans = KMeans(n_clusters = numCluster).fit(X)
    kClusters = []

    for i in range(0, numCluster):
        kClusters.append([[],[],[]])
    for i in range(0, len(kmeans.labels_)):
        kClusters[kmeans.labels_[i]][0].append(cluster[0][i])
        kClusters[kmeans.labels_[i]][1].append(cluster[1][i])
        kClusters[kmeans.labels_[i]][2].append(cluster[2][i])
    return kClusters



# def cluster2objKMEANS(cluster, numCluster, minPercentile = 10, maxPercentile = 90):
#     X = np.column_stack((cluster[0], cluster[1], cluster[2]))

#     print("KMEANS TIME")
#     #Could use metrics such as wcss, sihouette, or volume minimize
#     kmeans = KMeans(n_clusters = numCluster).fit(X)
#     objList = []
#     kClusters = []

#     for i in range(0, numCluster):
#         kClusters.append([[],[],[]])
#     for i in range(0, len(kmeans.labels_)):
#         kClusters[kmeans.labels_[i]][0].append(cluster[0][i])
#         kClusters[kmeans.labels_[i]][1].append(cluster[1][i])
#         kClusters[kmeans.labels_[i]][2].append(cluster[2][i])
#     for i in range(0, numCluster):
#         objList.append(cluster2obj(kClusters[i], minPercentile, maxPercentile))

#     return objList

def rotateCluster(cluster):
    return cluster
    #TODO remove this code as it should not be necessary
    # The point cloud should be automatically rotated 
    print('Starting rotation')
    qw= 0.6760704112311099
    qx= -0.10048774764081572
    qy= 0.004741109455629532
    qz= 0.7299373490327808
    x= 0.03523307719392562
    y= -1.4418668889585609
    z= 0.6256011107718419

    transMatrix = quaternion_matrix([qx, qy, qz, qw])

    secondTransMatrix = quaternion_matrix([0, -0.7071068, 0, 0.7071068])


    
    #Q = [qw, qx, qy, qz]
    Trans = [x, y, z]
    print(transMatrix)
    transMatrix[0:3, 3] = Trans
    print(transMatrix)


    allZ = [x * -1 for x in cluster[0]]
    allX = cluster[1]
    allY = [x * -1 for x in cluster[2]]


    


    #rot = quaternion_rotation_matrix(Q)
    

    #rot = np.linalg.inv(rot)
    #transMatrix = np.identity(4)
   # transMatrix[0:3, 0:3] = rot
   # transMatrix[0:3, 3] = Trans
    #print(transMatrix)
    print('Transformation Matrix Formed')
    points = np.zeros((len(allX), 4))


    points[:,0] = allX
    points[:,1] = allY
    points[:,2] = allZ
    points[:,3] = 1

    # sel = np.random.choice(points.shape[0], size = 10000, replace = False)
    # print(sel)
    # condensePoints = points[sel]

    # points = condensePoints

    print('Finish creating np points')
    transformedPoints = np.dot(points, transMatrix)
    transformedPoints[:,0] = transformedPoints[:,0] + x
    transformedPoints[:,1] = transformedPoints[:,1] + y
    transformedPoints[:,2] = transformedPoints[:,2] + z
    transformedPoints = np.dot(points, secondTransMatrix)

    transformedPoints = np.delete(transformedPoints, 0, axis=0)
    

    # allX = points[:,0] 
    # allY = points[:,1] 
    # allZ = points[:,2] 
    transX = transformedPoints[:,0] #+ x
    transY = transformedPoints[:,1] #+ y
    transZ = transformedPoints[:,2] #+ z
    rotCluster = [transX, transY, transZ]

    # fig = plt.figure(figsize=(4,4))
    # ax = fig.add_subplot(111, projection = '3d')

    # ax.scatter(allX, allY, allZ, marker = 'x')
    # ax.set_xlabel('x')
    # ax.set_ylabel('y')
    # ax.set_zlabel('z')
    # ax.scatter(transX, transY, transZ, marker = '+')
    # ax.plot([0],[0],[0], marker = 'o')
    # plt.show()


    print('Done rotation')
    return rotCluster
    #return cluster


def addCluster(fullCluster, cluster):

    xS = fullCluster[0] + cluster[0]
    yS = fullCluster[1] + cluster[1]
    zS = fullCluster[2] + cluster[2]

    newFullCluster = [xS, yS, zS]
    return newFullCluster

def filterCluster(rawCluster, leafSize):
    filteredX = [round(((round(x/leafSize)) * leafSize), 8) for x in rawCluster[0]]
    filteredY = [round(((round(y/leafSize)) * leafSize), 8) for y in rawCluster[1]]
    filteredZ = [round(((round(z/leafSize)) * leafSize), 8) for z in rawCluster[2]]
    
    filteredTuples = list(zip(filteredX, filteredY, filteredZ))
    filteredTuples = removeDuplicates(filteredTuples)
    filteredTuples = removeOutOfRange(filteredTuples)

    xS = [x[0] for x in filteredTuples]
    yS = [x[1] for x in filteredTuples]
    zS = [x[2] for x in filteredTuples]

    #Convert tuples back to list
    filterCluster = [xS, yS, zS]
    return filterCluster

def removeOutOfRange(filteredTuples):
    filtered = [x for x in filteredTuples if abs(x[0]) < 1 and abs(x[1]) < 1 and abs(x[2]) < 1]
    return filtered

def removeDuplicates(lst):
      
    return list(set([i for i in lst]))



if __name__ == "__main__":

    pointCloudDir = '/home/simtiaz/ur5_collision_avoidance/catkin_ws/pointCloudDir/'
    #pointCloudDir = '..\\..\\..\\..\\pointClouds' THE WINDOWS POINT CLOUD DIRECTORY

    fileList = os.listdir(pointCloudDir)
    objList = []
    fullScan = True
    if fullScan:
        megaCluster = [[], [], []]
        clusterTic = time.perf_counter()
        for file in fileList:
            if (file.endswith(".pcd")):
                fullFileName = os.path.join(pointCloudDir, file)
                cluster = pcd2cluster(fullFileName)
                megaCluster = addCluster(megaCluster, cluster)
        clusterToc = time.perf_counter()
        print("Size of the cluster before filtering: ", len(megaCluster[0]))
        filterTic = time.perf_counter()
        filteredCluster = filterCluster(megaCluster, 0.005)
        #filteredCluster = megaCluster
        filterToc = time.perf_counter()
        print("Size of cluster after filtering: ", len(filteredCluster[0]))
        kmeansTic = time.perf_counter()
        kObjects = cluster2objKMEANS(megaCluster, numCluster = 300)
        kmeansToc = time.perf_counter()
        
        for object in kObjects:
            objList.append(object)
       


        print(f"Combined all the point cloud data in {clusterToc - clusterTic:0.4f} seconds")
        print(f"Filtered the clusters in {filterToc - filterTic:0.4f} seconds")
        print(f"Seperated the clusters using kmeans in {kmeansToc - kmeansTic:0.4f} seconds")
        
    else:
        for file in fileList:
            if (file.endswith(".pcd")):
                fullFileName = os.path.join(pointCloudDir, file)
                cluster = pcd2cluster(fullFileName)
                #rotCluster = rotateCluster(cluster)
                kObjects = cluster2objKMEANS(cluster, numCluster = 200)
                #obj = cluster2obj(cluster)
                for object in kObjects:
                    objList.append(object)
    writeTic = time.perf_counter()
    objWriter(objList)
    writeToc = time.perf_counter()
    print(f"Wrote the objects to CIK in {writeToc - writeTic:0.4f} seconds")