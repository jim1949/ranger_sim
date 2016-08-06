#!/usr/bin/env python
#                    Simple python examples
# python/randomwalk6.py
#                    David MacKay
# This can be run from a unix shell
#
# Make a walk with a fair coin, plotting 
# with Gnuplot the points reached at times
# 0, period, 2*period, 3*period...
#
#    R      = number of walks
#    T      = duration of walk
#    period = period between points shown
#
import random
import sys
import Gnuplot, Gnuplot.funcutils

def walk(T=10, period=1):
    """random walk with a fair coin"""
    x=0;     answer= [[0,0]]
    for t in xrange(T+1):
        u = random.random()
        if (u<= 0.5): x+=1 
        else:         x-=1 
        if (t%period == 0 ) :
            answer.append([ t+1 , x ])
    return answer

def makewalks(R=10, T=100,period=1):
    """Makes walks and plots them with gnuplot"""
    g = Gnuplot.Gnuplot(persist=1) 
    g.title('Random walks') 
    g('set data style line') 

    for r in xrange(R):
        newwalk = walk(T,period)
        if(r==0): g.plot(newwalk)   
        else:     g.replot(newwalk) 

if(1):
    R=4
    T=100
    period=1
    makewalks(R, T, period )

# This program illustrates
#  - how to plot lists of lists in gnuplot
#  - how to use random.random()
#      (there are many other random functions
#       available in python; random.random()
#       makes numbers between 0 and 1)
