#!/usr/bin/env python
import path_time
from datetime import datetime
class path_times:

    def __init__(self,edge):
        self.edge=edge
        self.recordings = []
    def add_recording(self, time_taken):
        self.recordings.append(path_time.path_time(datetime.today(),time_taken))
    def add_false_recording(self, duration, time):
        self.recordings.append(path_time.path_time(time,duration))
    def __str__(self):
        path_string =[]
        for recording in self.recordings:
            path_string.append(str(recording))

        return str(('Edge: {0} to {1} recordings:'.format(self.edge.A.node_num,self.edge.B.node_num),path_string))
    def data_matrix(self):
        matrix = []
        for recording in self.recordings:
            matrix.append([recording.date.hour*60+recording.date.minute,recording.time.seconds+recording.time.microseconds/10**6.])
        return matrix