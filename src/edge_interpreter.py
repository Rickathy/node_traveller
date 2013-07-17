#!/usr/bin/env python


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
