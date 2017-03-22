#coding=utf-8
import numpy as np
import pylab as pl

data = np.loadtxt('data.txt')
x1 = range(0,200)
x2 = range(0,200)
pl.plot(x1, data[:,0], 'r',label='measure')
pl.plot(x2, data[:,1], 'g',label='kalman')
pl.xlim(0, 200)
pl.ylim(0, 35)
pl.xlabel('timer')
pl.ylabel('temperature')
pl.legend(loc='lower right')
pl.show()