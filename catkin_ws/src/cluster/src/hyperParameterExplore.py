import cluster2obj as clus
import time
import os
import math

#clusterCount = [1000]
#minMax = [(1,99)]
kTreeVals = [True, False]
clusterCount = [100, 200, 500, 2000]
minMax = [(1, 99), (5, 95), (10, 90)]





for cc in clusterCount:
    for minMaxPair in minMax:
        for kTree in kTreeVals:
            with open("outputCCMinMaxKTree.txt", "a") as f:
                minPercentile = minMaxPair[0]
                maxPercentile = minMaxPair[1]
                print("Cluster Count: ", cc , file=f)
                print("Filter applied: ", "true" , file=f)
                print("Leaf size: ", "0.001" , file=f)
                print("Ktree Branching applied: ", kTree, file = f)
                print("Min Max: (",  minPercentile, ",", maxPercentile, ")", file=f)
                print("---------------------------------------" , file=f)

                pointCloudDir = '..\\..\\..\\..\\pointClouds'
                #pointCloudDir = '.\\pointClouds'
                fileList = os.listdir(pointCloudDir)
                objList = []
                fullScan = True
                if fullScan:
                    megaCluster = [[], [], []]
                    clusterTic = time.perf_counter()
                    for file in fileList:
                        if (file.endswith(".pcd")):
                            fullFileName = os.path.join(pointCloudDir, file)
                            cluster = clus.pcd2cluster(fullFileName)
                            megaCluster = clus.addCluster(megaCluster, cluster)
                    clusterToc = time.perf_counter()
                    print("Size of the cluster before filtering: ", len(megaCluster[0]) , file=f)
                    filterTic = time.perf_counter()

                    filteredCluster = megaCluster
                    if filter:
                        filteredCluster = clus.filterCluster(megaCluster, 0.001)

                    filterToc = time.perf_counter()
                    print("Size of cluster after filtering: ", len(filteredCluster[0]) , file=f)
                    kmeansTic = time.perf_counter()
                    kObjects = []
                    if kTree:
                        kObjects = clus.cluster2objKMeans(filteredCluster, round(math.sqrt(cc)), minPercentile= minPercentile, maxPercentile= maxPercentile, branchLevel = 2)
                    else:
                        kObjects = clus.cluster2objKMeans(filteredCluster, numCluster = cc, minPercentile= minPercentile, maxPercentile= maxPercentile, branchLevel = 1)
                    kmeansToc = time.perf_counter()
                    writeTic = time.perf_counter()
                    for object in kObjects:
                        objList.append(object)

                    #Make file name
                    uniqueString = "cluster" + str(cc) + "minMax" + str(minPercentile) + "_" + str(maxPercentile) + "ktree" + str(kTree)
                    fileName = ".\\" + uniqueString + "\\settings.yaml"
                    os.mkdir(uniqueString)
                    #Make directory
                    clus.objWriter(objList, settingsFileName= fileName)
                    writeToc = time.perf_counter()


                    print(f"Combined all the point cloud data in {clusterToc - clusterTic:0.4f} seconds" , file=f)
                    print(f"Filtered the clusters in {filterToc - filterTic:0.4f} seconds" , file=f)
                    print(f"Seperated the clusters using kmeans in {kmeansToc - kmeansTic:0.4f} seconds" , file=f)
                    print(f"Wrote the objects to CIK in {writeToc - writeTic:0.4f} seconds" , file=f)

                    print("\n\n\n\n" , file=f)
