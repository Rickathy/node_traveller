#!/usr/bin/env python

class travel_edge:
    def __str__(self):
        return 'A: {0}, B: {1}, cost: {2}'.format(self.A.node_num,self.B.node_num,self.cost)

    def __init__(self,args):
    # print 'args: ',args

        if len(args)==2:
            self.A=args[0]
            self.B=args[1]
            self.cost=1
        else:
            if len(args)==3:
                self.A=args[0]
                self.B=args[1]
                self.cost=args[2]
            else:
                return Exception("incorrect number of parameters for initialization of edge")

