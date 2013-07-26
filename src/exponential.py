#!/usr/bin/env python


class exponential:
    def __init__(self):
        return
    def calculate_distribution(self, durations,times_of_day,minute_interval=10):
        
        partitions=[]
       
        lambdas=[]
        for i in range(0,1440/minute_interval):
            partitions.append(i*minute_interval)
            sum=0
            amt=0
            for j in range(len(times_of_day)):
               
                if (times_of_day[j]>=(i*minute_interval))& (times_of_day[j]<(i+1)*minute_interval):
                  sum+=  durations[j]
                  amt+=1
            print amt
            if amt >0:
                lambdas.append(1/(sum/amt))
                print 'lambda',1/sum/amt
                
            else:
                lambdas.append(0)
        partitions.append(1440)
        return partitions,lambdas