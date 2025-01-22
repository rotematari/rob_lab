import numpy as np
import pickle
import os
import time
import glob
import matplotlib

import matplotlib.pyplot as plt


logdir_prefix = 'lab-01'

data_path = os.path.join(os.getcwd(), 'data')

data = []

for file in glob.glob(data_path + '/*/*.pkl'):
    with open(file, 'rb') as h:
        data.append(pickle.load(h))
# print(data)

