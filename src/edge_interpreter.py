#!/usr/bin/env python

from clustering import clustering

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
                        return self.interpret_clustering_ten_minutes(path_times.recordings)
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
    
    def interpret_clustering_ten_minutes(self,times):
        c = clustering()
        clusters=c.cluster_path_times(times,False)
        print('clusters are:')
        for key in clusters.keys():
            print clusters[key]
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
        print averages
        print main_average
        bins_amts = []
        bins_ests =[]
        partitions=[]
        for i in range(0,144):#create a bin for each ten minutes of the day
            bins_ests.append(0)
            bins_amts.append(0)
            partitions.append(i*10)
        partitions.append(1440)
        print clusters.keys()
        for i in range(len(clusters.keys())):
            for(duration,date) in clusters[clusters.keys()[i]]:
                bins_amts[int(date)/10]+=1
                try:
                    bins_ests[int(date)/10]+=averages[i]
                except IndexError:
                    print int(date)/10
                    print len(bins_amts)
                    print clusters.keys()[i]
                    raise
                    
        for i in range(0,144):
            
            if(bins_amts[i]>0):
                print('bin amts is greater than 0 at {0}, bin ests is {1}, bin amts is {2}'.format(i,bins_ests[i],bins_amts[i]))
                bins_ests[i]/=bins_amts[i]
            else:
                bins_ests[i]=main_average
        return partitions,bins_ests
                

