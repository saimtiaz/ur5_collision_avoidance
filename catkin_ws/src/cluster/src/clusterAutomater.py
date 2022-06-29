import cluster2obj as clus
import time
import os

clusterCount = [2000]
filters = [True, False]
leafSize = [0.1, 0.05, 0.01, 0.005]





for cc in clusterCount:
    for filter in filters:
        for leaf in leafSize:
            with open("output2.txt", "a") as f:

                print("Cluster Count: ", cc , file=f)
                print("Filter applied: ", filter , file=f)
                print("Leaf size: ", leaf , file=f)
                print("---------------------------------------" , file=f)

                pointCloudDir = '..\\..\\..\\..\\pointClouds'
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
                        filteredCluster = clus.filterCluster(megaCluster, leaf)

                    filterToc = time.perf_counter()
                    print("Size of cluster after filtering: ", len(filteredCluster[0]) , file=f)
                    kmeansTic = time.perf_counter()
                    kObjects = clus.cluster2objKMEANS(megaCluster, numCluster = cc)
                    kmeansToc = time.perf_counter()
                    writeTic = time.perf_counter()
                    for object in kObjects:
                        objList.append(object)

                    #Make file name
                    uniqueString = "cluster" + str(cc) + "filter" + str(filter) + "leaf" + str(leaf)
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
