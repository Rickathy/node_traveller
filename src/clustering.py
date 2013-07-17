#!/usr/bin/env python

import numpy, scipy
import scipy.cluster.hierarchy as hier
import scipy.spatial.distance as dist

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
        Y=pdist(X)
        Z=linkage(Y)
        dendrogram(Z)
        for i in range(len(X)):
            print('{0}, {1}'.format(i,X[i]))
        print Z
        vars= Z[:,2]
        print('variances', vars)
        grads= []
        for i in range(len(vars)-1):
            grads.append((vars[i]+vars[i+1])/2)
        print('gradients', grads)
        average_grad = sum(grads)/len(grads)
        print('average grad', average_grad)
        for i in range(len(grads)):
            if average_grad<grads[i]:
                print 'last link is',i-1
                break

        if display:
            show()
    def variance(self, cluster):
        av_date=0
        av_time=0
        for (date,time) in cluster:
            av_date+=date
            av_time+=time
        av_date/=len(cluster)
        av_time/=len(cluster)