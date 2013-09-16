#!/usr/bin/env python

from clustering import clustering
from exponential import exponential
import math
import sys

class edge_interpreter:
    def __init__(self):
        return
    # given a list of edge traversals and a type corresponding to a type of interpretation, return a list of time intervals
    # and the time to traverse an edge in that interval
    def interpret(self, path_times, type,days=False):
        if type==0:
            return self.interpret_average(path_times.recordings)
        else:
            if type==1:
                return self.interpret_hourly_average(path_times.recordings)
            else:
                if type==2:
                    return self.interpret_ten_minute_averages(path_times.recordings)
                else:
                    if type==3:
                        return self.interpret_clustering_ten_minutes(path_times.recordings,True,days)
                    else:
                        if type==4:
                            return self.interpret_exponential(path_times.recordings,True,10,days=days)
                        else:
                            return None
    # Just return the average of all the times
    def interpret_average(self,times):

        sum =0.
        for time in times:
            sum+=time.time.seconds+time.time.microseconds/10**6.
        return [0,1440],sum/len(times)# it's the whole day, so we only have one time interval
    
    def interpret_hourly_average(self,times):
        average = self.interpret_average(times)[1]
        avs=[]
        partitions =[]
        for i in range(23):
            sum =0
            count=0
            for time in times:
                if time.date.hour ==i:
                    sum+=time.time.seconds+time.time.microseconds/10**6.
                    count+=1
            if count >0:
                avs.append(sum/count)
            else:
                avs.append(average)
            partitions.append(i*60)
        partitions.append(24*60)

        return partitions, avs
    def interpret_ten_minute_averages(self,times):
        average = self.interpret_average(times)[1]
        avs=[]
        partitions=[]
        for i in range(1440/10):
            sum=0
            count=0
            for time in times:
                if int((time.date.hour*60+time.date.minute)/10) ==i:
                    sum+=time.time.seconds+time.time.microseconds/10**6.
                    count+=1
            if count >0:
                avs.append(sum/count)
            else:
                avs.append(average)
            partitions.append(i*10)
        partitions.append(24*60)
        return partitions,avs
    '''
    Given a set of times clusters on those times and produces a estimated traversal time from that clustering by
    splitting the day into bins of 10 minutes
    If the final parameter is true, will also return the entropies for each bin
    '''
    def interpret_clustering_ten_minutes(self,times,entropy=False,minimum_traversals=2,minute_interval=10,days=False):
        if days==True:
                max_val = 1440*7
        else: 
                max_val=1440
        if len(times)>1:
            
            c = clustering()
            clusters=c.cluster_path_times(times,False,days)
            #print('clusters are:')
            #for key in clusters.keys():
             #   print clusters[key]
            averages=[]
            main_average=0
            total=0
            for key in clusters.keys():
                av_duration =0
                for (duration,date)in clusters[key]:
                    av_duration+=duration
                    main_average+=duration
                    total+=1
                av_duration/=len(clusters[key])
                averages.append(av_duration)
            main_average/=total
            #print averages
            #print main_average
            bins_amts = []
            bins_ests =[]
            zeroCount=[]
            partitions=[]
            #print 'days', days
           
            #print 'max_val',max_val
            for i in range(0,max_val/minute_interval):#create a bin for each ten minutes of the day
                bins_ests.append(0)
                bins_amts.append(0)
                zeroCount.append(0)
                partitions.append(i*minute_interval)
            partitions.append(max_val)
            #print clusters.keys()
            for i in range(len(clusters.keys())):
                #zeroCount=0
                for(duration,date) in clusters[clusters.keys()[i]]:
                    if duration ==0:
                        zeroCount[int(date)/minute_interval]+=1
                    else:
                        bins_amts[int(date)/minute_interval]+=1
                        bins_ests[int(date)/minute_interval]+=averages[i]
                 
                        
            for i in range(0,max_val/minute_interval):
                
                if(bins_amts[i]>0):
                   
                    bins_ests[i]/=bins_amts[i]
                else:
                    bins_ests[i]=main_average
                if zeroCount[i] > bins_amts[i]:
                    bins_ests[i]= -1 #special value for if this edge is blocked
            entropies=[]
            if entropy is True:
                entropies=[]
                for i in range(0,max_val/minute_interval):
                    bin=[]
                    for j in range(len(clusters.keys())):
                        for(duration,date) in clusters[clusters.keys()[j]]:
                            if (date>=i*minute_interval) & (date<(i+1)*(minute_interval)):
                               # print(i*minute_interval, (i+1)*minute_interval)
                                bin.append(j)
                    counts=dict()
                    total=len(bin)*1.0
                    for value in bin:
                        
                        try:
                            counts[value]+=1
                        except KeyError:
                            counts[value]=1.
                    #print counts
                    entropy=0
                    if total >= minimum_traversals:
                        for x in counts.keys():
                           # print('counts/total',counts[x]/total)
                            #print 'counts[x]',counts[x]
                            #print 'total',total
                            if counts[x] > 0:
                                entropy-=counts[x]/total*math.log(counts[x]/total)
                    else:
                        entropy=sys.maxint
                    entropies.append(entropy)
                return partitions,bins_ests,entropies
            else:   
                return partitions,bins_ests
        else:
            partitions=[]
            bin_ests=[]
            entropies=[]
            for i in range(0,max_val/minute_interval):
                partitions.append(i*minute_interval)
                bin_ests.append(sys.maxint)
                entropies.append(sys.maxint)
            if entropies:
                return partitions,bin_ests,entropies
            else:
                return partitions, bin_ests
    def interpret_exponential(self,recordings,confidence=False,minute_interval=10,days=False):
            durations=[]
            if days==True:
                max_val=1440*7
            else:
                max_val=1440
            for recording in recordings:
                try:
                    durations.append(recording.time.seconds+recording.time.microseconds/10**6.)
                except AttributeError:
                    durations.append(0)
            if (len(durations)==0):
                partitions=[]
                lambdas=[]
                if confidence==True:
                    cons=[]
                for i in range(0,max_val/minute_interval):
                    partitions.append(i*minute_interval)
                    lambdas.append(0)
                    if confidence==True:
                        cons.append(sys.maxint)
                partitions.append(max_val)
                if confidence==True:
                    return partitions, lambdas, cons
                else:
                    return partitions, lambdas
            total_average= sum(durations)/len(durations)
            times_of_day=[]
            for recording in recordings:
                if days==True:
                    times_of_day.append(recording.date.day*60*24+recording.date.hour*60+recording.date.minute)
                else:
                    times_of_day.append(recording.date.hour*60+recording.date.minute)
            e = exponential()
            if confidence == True:
                (partitions,lambdas,confidences)=e.calculate_distribution(durations,times_of_day,minute_interval,confidence,days)
            else:
                (partitions,lambdas)=e.calculate_distribution(durations,times_of_day,days=days)
            expected_values=[]
            for l in lambdas:
                if l >0:
                    expected_values.append(1/l)
                else:
                    expected_values.append(0)
            
            if confidence == True:
                return partitions, expected_values,confidences
            else:
                return partitions,expected_values
            

