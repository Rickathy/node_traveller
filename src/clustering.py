#!/usr/bin/env python

import numpy, scipy
import scipy.cluster.hierarchy as hier
import scipy.spatial.distance as dist

import math

from hcluster import pdist, linkage, dendrogram
import scipy.cluster.hierarchy as hcluster
from matplotlib.pyplot import show
import matplotlib.pyplot as plt

class clustering:
    def __init__(self):
        return
    def cluster_path_times(self, recordings,display=False):
        
        X=[]

        for recording in recordings:
            X.append(recording.time.seconds+recording.time.microseconds/10**6.)
        times_of_day=[]
        for recording in recordings:
            times_of_day.append(recording.date.hour*60+recording.date.minute)
        print X
        Y=[]
        for x in X:
            temp=[]
            for x2 in X:
                temp.append(x-x2)
            Y.append(temp)
        #Y=pdist(X)
        Z=linkage(Y)
       # print('Z', Z)
        dendrogram(Z)
       # for i in range(len(X)):
        #    print('{0}, {1}'.format(i,X[i]))
       # print('Z', Z)
        clusters= self.generate_clusters(X,Z,times_of_day)
        #print('clusters are: {0}'.format(clusters))
        #for key in clusters.keys():
        #    print clusters[key]
        if display:
            show()
        return clusters
    #calculate the variances of each set of clusters as they are made
    def generate_clusters(self,data_set,linkage_matrix,times_of_day):
        clusters =[]
        #print('data set: ', data_set)
        for data in data_set:
            clusters.append([data])
        var_sum=[]
        var_temp =0
        for cluster in clusters:

            var_temp+=self.variance(cluster)/len(cluster)
        #print var_sum
        var_sum.append(var_temp)
        genes = dict()
        print linkage_matrix
        for i in range(len(clusters)):
            genes[i]= clusters[i]
        #print('genes', genes)
        for i in range(0,len(linkage_matrix)):
            #print linkage_matrix[0]
            temp =[]
            for gene in genes[linkage_matrix[i][0]]:
                temp.append(gene)
            for gene in genes[linkage_matrix[i][1]]:
                temp.append(gene)
            genes[i+1+len(linkage_matrix)]= temp
            del genes[linkage_matrix[i][0]]
            del genes[linkage_matrix[i][1]]
           # print(' at point ',i, ' genes are ', genes)
            var_temp=0
            for key in genes.keys():
              #  print('clusters: ',genes[key])
                var_temp+=self.variance(genes[key])/len(genes[key])
            var_sum.append(var_temp)
        grads=[]
        for i in range(len(var_sum)-1):
            grads.append((var_sum[i]+var_sum[i+1])/2)
        grad_av = sum(grads)/len(grads)
       # print('gradient average is ',grad_av)
       # print('grads: ',grads)
        change_point=0
        for i in range(len(grads)):
            if grads[i] > grad_av:
                change_point=i
                print(i,'  gradient is greater than average')
                
        genes = dict()  
        for i in range(len(clusters)):
            genes[i]= [(clusters[i][0],times_of_day[i])]
        for i in range(0,change_point):
            #print linkage_matrix[0]
            temp =[]
            for gene in genes[linkage_matrix[i][0]]:
                temp.append(gene)
            for gene in genes[linkage_matrix[i][1]]:
                temp.append(gene)
            genes[i+1+len(linkage_matrix)]= temp
            del genes[linkage_matrix[i][0]]
            del genes[linkage_matrix[i][1]]     
        return genes


    #calculate the variance of the provided cluster
    def variance(self, cluster):
        
        av_time=0
        print cluster
        for time in cluster:
          
            av_time+=time
        
        av_time/=len(cluster)
        sum =0
        sumSqr=0
        for time in cluster:
            dist = time-av_time
            sum+=dist
            sumSqr+=dist*dist
        var = sumSqr/len(cluster)
        return var