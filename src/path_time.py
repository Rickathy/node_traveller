#!/usr/bin/env python
class path_time:
    def __init__(self,date,time):
        self.date=date #when the reading was recorded
        self.time=time #how long the reading took
    def __rpr__(self):
        return str('Traversal at {0} took {1}'.format(self.date,self.time))
    def __str__(self):
        return str('Traversal at {0} took {1}'.format(self.date,self.time))
