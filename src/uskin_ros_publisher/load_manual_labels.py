#!/usr/bin/env python
import pandas as pd


class ManualLabelsRecord:

    def __init__(self, file_path):
        print('=================================================================================')
        print('Loading Manual Labels data')
        print('=================================================================================')
        try:
            
            self.bdf = pd.read_excel(file_path, converters={'Timestamp (Epochs)':str, 'Timestamp (Date)':str})
            self.timestamps = []
            print (self.bdf)
            # Necessary to zero left-pad the float decimal part (thank you ROS)
            # self.bdf['Timestamp (Epochs)'][:] = [float(temp[0]+"."+temp[1].zfill(9)) for temp in self.bdf['Timestamp (Epochs)']]
            for timestamp in self.bdf['Timestamp (Epochs)']:
                temp = timestamp.split('.')
                self.timestamps.append(float(temp[0]+"."+temp[1].zfill(9)))
        except:
            print ("An error has occurred")
            del self

    def getTimestampStartExperiment(self):
        return self.timestamps[0]

    def getTimestampEndExperiment(self):
        return self.timestamps[-1]
    
    def getTimestampsSlips(self):
        return self.timestamps[1:-1]

    def resetTimestamps(self, ground_timestamp):
        for index, timestamp in enumerate(self.bdf['Timestamp (Epochs)']):
            temp = timestamp.split('.')
            # Get original timestamps from bdf
            self.timestamps[index] = float(temp[0]+"."+temp[1].zfill(9))
            # Reset timestamps
            self.timestamps[index] = (self.timestamps[index] - ground_timestamp)*1000
    
    def setTimestampEndExperiment(self, timestamp):
        self.timestamps[-1] = timestamp

        

