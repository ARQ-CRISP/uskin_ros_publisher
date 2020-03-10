import h5py 
import numpy as np

def print_attrs(name): 
    print (name) 
    # for key, val in obj.attrs.items(): 
    #     print ("{}: {}".format(key, val)) 
 
f = h5py.File('/home/rodrigo/Documents/github/uskin_publisher/src/uskin_ros_publisher/files/h5py/slipDataset.h5','r') 
f.visit(print_attrs)