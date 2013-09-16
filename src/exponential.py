#!/usr/bin/env python
import math
import sys
import scipy.stats.mstats as mst
import scipy.stats as stats
import numpy as np
from numpy import asarray, array
import scipy.special as special

class exponential:
    def __init__(self):
        return
    def calculate_distribution(self, durations,times_of_day,minute_interval=10,confidence=False,days=False):
        
        partitions=[]
       
        lambdas=[]
        if confidence == True:
            cons = []
        average=0
        count=0
        for dur in durations:
            if dur>0:
                average+=dur
                count+=1
        average/=count
        if days==True:
            max_val=1440*24
        else:
            max_val=1440
        for i in range(0,max_val/minute_interval):
            partitions.append(i*minute_interval)
            sum=0
            amt=0
            zeros=0
            observed=[]
            for j in range(len(times_of_day)):
               
                if (times_of_day[j]>=(i*minute_interval))& (times_of_day[j]<(i+1)*minute_interval):
                  sum+=  durations[j]
                  
                  observed.append(durations[j])
                  if durations[j]==0:
                      zeros+=1
                  else:
                      amt+=1
            #print amt
            if amt >zeros:# there are less failed runs than normal runs
           
                lambdas.append(1/(sum/amt))
               # print 'lambda',1/sum/amt
            
            else:
                if zeros>0:# no readings from this time
                    lambdas.append(0)
                    
                else:
                    lambdas.append(1/average)
            upper =[]
            lower =[]
            for obs in observed:
                per = stats.percentileofscore(observed,obs)
                
                if per < 95:
                    lower.append(obs)
                if per > 5:
                    upper.append(obs)
            if confidence== True:
                if (amt > 0) & (sum>0) & (len(upper)>0) & (len(lower)>0):
                    expected_lower =[]
                    for i in range(len(lower)):
                        #print'lambdas', lambdas
                        try:
                            expected_lower.append(1/lambdas[len(lambdas)-1])
                        except ZeroDivisionError:
                            expected_lower.append(0)
                    expected_upper = []
                    for i in range(len(upper)):
                        try:
                            expected_upper.append(1/lambdas[len(lambdas)-1])
                        except ZeroDivisionError:
                            expected_upper.append(0)
                   # print 'observed, expected lower, expected upper',observed, expected_lower, expected_upper
                    #print 'upper, lower', upper, lower
                    
                    chi_vals_lower =mst.chisquare(np.array(lower),np.array(expected_lower))
                    chi_vals_upper =mst.chisquare(np.array(upper),np.array(expected_upper))
                    #print 'chi',chi_vals_lower, chi_vals_upper
                    if lambdas[len(lambdas)-1]>0:
                        con_int = (2*amt/lambdas[len(lambdas)-1]*chi_vals_upper[0])-(2*amt/lambdas[len(lambdas)-1]*chi_vals_lower[0])
                    else:
                        con_int = (2*amt/1*chi_vals_upper[0])-(2*amt/1*chi_vals_lower[0])
                    cons.append(abs(con_int))
                    
                    #print 'new confidence value being added', i ,cons[i]
                else:
                    cons.append(sys.maxint)
        partitions.append(1440)
        if confidence == True:
            return partitions,lambdas, cons
        else:
            return partitions,lambdas
