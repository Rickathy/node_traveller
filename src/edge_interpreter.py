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
    def interpret(self, path_times, type):
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
                        return self.interpret_clustering_ten_minutes(path_times.recordings,True)
                    else:
                        if type==4:
                            return self.interpret_exponential(path_times.recordings)
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
    def interpret_clustering_ten_minutes(self,times,entropy=False,minimum_traversals=1,minute_interval=10):
        
        c = clustering()
        clusters=c.cluster_path_times(times,False)
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
        partitions=[]
        for i in range(0,1440/minute_interval):#create a bin for each ten minutes of the day
            bins_ests.append(0)
            bins_amts.append(0)
            partitions.append(i*minute_interval)
        partitions.append(1440)
        #print clusters.keys()
        for i in range(len(clusters.keys())):
            for(duration,date) in clusters[clusters.keys()[i]]:
                bins_amts[int(date)/minute_interval]+=1
                bins_ests[int(date)/minute_interval]+=averages[i]
             
                    
        for i in range(0,1440/minute_interval):
            
            if(bins_amts[i]>0):
               
                bins_ests[i]/=bins_amts[i]
            else:
                bins_ests[i]=main_average
        
        entropies=[]
        if entropy is True:
            entropies=[]
            for i in range(0,1440/minute_interval):
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
    def interpret_exponential(self,recordings):
        durations=[]

        for recording in recordings:
            durations.append(recording.time.seconds+recording.time.microseconds/10**6.)
        total_average= sum(durations)/len(durations)
        times_of_day=[]
        for recording in recordings:
            times_of_day.append(recording.date.hour*60+recording.date.minute)
        e = exponential()
        (partitions,lambdas)=e.calculate_distribution(durations,times_of_day)
        expected_values=[]
        for l in lambdas:
            if l >0:
                expected_values.append(1/l)
            else:
                expected_values.append(total_average)
        return partitions,expected_values
            

