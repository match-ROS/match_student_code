#!/usr/bin/env python
# # coding=latin-1

import numpy as np

# FIR bei gleichem a fuer alle Zeitpunkte:
# x(n) = y(n)*a + y(n-1)*a + ... + y(0)*a = y(n)*a + x(n-1)
class LowPassFilter(object):
    """IIR Filter: x(n) = y(n)*k + x(n-1)*1.
        Filter returns absolute value of data. (FOR USE WITH IMG DATA)
        Args:
            data_shape: shape of data to filter
            k = b_0 bei IIR (entspricht a bei FIR)"""
    def __init__(self, data_shape, k=0.2):
        self.integrator = np.zeros(data_shape)
        self.k = k  # omega*control_period

    def filter(self, data):
        data = np.array(data).astype("float")
        self.integrator = np.abs(self.integrator + (data - self.integrator) * self.k)
        return self.integrator
    
class MovingAvg(object):
    #TODO: add shape
    def __init__(self, filter_len, num_data=1):
        self.__filter_len = filter_len
        self.__num_data = num_data
        self.vals = np.zeros((num_data, filter_len), dtype="float")
        self.__mean = 0.0

    def update(self, data: np.ndarray):
        """update filter with new data
        #TODO: check for speed 

        Args:
            data (np.ndarray(num_data)): new data

        Returns:
            np.ndarray(num_data, dtype="float"): mean of filter
        """
        # self.mean = self.mean + (data - self.vals[0]) / self.__filter_len
        # self.vals[:-1] = self.vals[1:]
        # self.vals[-1] = data
        
        # self.vals[:-1] = self.vals[1:]
        # self.vals[-1] = data
        # self.mean = np.mean(self.vals)
        # return self.mean
    
        self.vals = np.roll(self.vals, 1, axis=1)
        self.vals[:,0] = data
        self.__mean = np.mean(self.vals,1)
    
        return self.__mean
    
    @property
    def mean(self):
        return self.__mean
    
    @property
    def filter_len(self):
        return self.__filter_len
    
    @filter_len.setter
    def filter_len(self, filter_len):
        self.__filter_len = filter_len
        vals_new = np.zeros((self.__num_data, filter_len), dtype="float") # len isnt necessary the same as filter_len
        self.vals = np.concatenate((self.vals,vals_new), axis=1)[:,:filter_len]
        
    @property
    def num_data(self):
        return self.__num_data
    
    @num_data.setter
    def num_data(self, num_data):
        self.__num_data = num_data
        vals_new = np.zeros((num_data, self.__filter_len), dtype="float")
        self.vals = np.concatenate((self.vals,vals_new), axis=0)[:num_data,:]
        


def main():
    import random
    lowPass = LowPassFilter((2, 1), 0.05)
    data = np.array([200, 200])

    for i in range(5):
        data_in = data + [random.randint(-10, 10), random.randint(-10, 10)]
        data_out = lowPass.filter(data)

    print("Data after 5 iterations")
    print(data_out)

    for i in range(100000):
        data_in = data + [random.randint(-1, 1), random.randint(-1, 1)]
        data_out = lowPass.filter(data)

    print("Data after 100000 iterations")
    print(data_out)


if __name__ == "__main__":
    main()
