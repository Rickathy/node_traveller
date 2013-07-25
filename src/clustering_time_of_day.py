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
    def cluster_path_times(self, path_times,display):
        recordings = path_times.recordings
        X=[]

        for recording in recordings:
            X.append([recording.time.seconds+recording.time.microseconds/10**6.,recording.date.hour*60+recording.date.minute])
        print X
        Y=pdist(X)
        Z=linkage(Y)
        dendrogram(Z)
        for i in range(len(X)):
            print('{0}, {1}'.format(i,X[i]))
        print Z
        print self.calculate_variances(X,Z)
        if display:
            show()
    #calculate the variances of each set of clusters as they are made
    def calculate_variances(self,data_set,linkage_matrix):
        clusters =[]
        for data in data_set:
            clusters.append([data])
        var_sum =0
        for cluster in clusters:

            var_sum+=self.variance(cluster)/len(cluster)
        print var_sum
        #for (A,B,dist,gene_count) in linkage_matrix:



    #calculate the variance of the provided cluster
    def variance(self, cluster):
        av_date=0
        av_time=0
        for (date,time) in cluster:
            av_date+=date
            av_time+=time
        av_date/=len(cluster)
        av_time/=len(cluster)
        sum =0
        sumSqr=0
        for (date,time) in cluster:
            dist = math.pow(date-av_date,2) + math.pow(time-av_time,2)
            sum+=dist
            sumSqr+=dist^2
        var = sumSqr/len(cluster)
        return var