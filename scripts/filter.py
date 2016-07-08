# -*- coding: utf-8 -*-
"""
Created on Tue Jun 28 12:26:29 2016

@author: tbrandeh
"""

class Filter:
    last_mean = None
    last_std = None
    
    def derivative(self,signal,window_size):
        res = [0]*(len(signal)-window_size)
        self.last_mean = None
        self.last_std = None
        weight = [1 for k in range(window_size)] #adapt weight ?
        for i in range(window_size,len(signal)):
            d = self.gma(signal[i-window_size:i],weight)
            res[i-window_size] = d
        return res
            
    def gma(self,window, weight):
        if len(window) == len(weight):
            res = 0
            mean_0 = np.mean(window)
            std_0 = np.std(window)
            if self.last_mean != None and self.last_std != None:       
                for i in range(0,len(window)):
                    dgma = weight[i] * math.log(p_theta(window[i],self.last_mean,self.last_std)/p_theta(window[i],mean_0,std_0))
                    res += dgma
            self.last_mean = mean_0
            self.last_std = std_0
            return res

def p_theta(y,mu,var):
    return math.exp(-((y-mu)**2)/2 * (var**2)) / (var * math.sqrt(2 * math.pi))
