# A dynamic model identification framework developed for daVinci Suigical System

There are parallelograms, springs, tendon couplings, and counterweight in da Vinci surgical system so that we can not use existing tools
to solve the dynamics model identification of it. This software framework was developed to solve these problems.


## Features
### Finished
* Symbolic dynamic modelling based on DH parameters and geometric tree
    * Geometrical modelling
    * Dynamic modelling
    * Base parameter generation using QR decomposition
* Optimal excitation trajectory generation based on fourier-series

* Data processing
    * Derive velocity and acceleration from position measurements
    * Zero-phase low-pass filter for original data
    * Remove the data whose velocity is close to zero to decrease the noise for Coulomb friction
* Identification
    * Ordinary Least Square (OLS)
    * Weighted Least Square (WLS)
    * Semi-definite Programming (SDP)
* Excitation of robots, using dvrk ROS library
### Yet to be done
* Output of the identified paramters
* Output of C++ or Python code of identified dynamic model for control use 

* Regularize output print

* Identification of the dynamic model of the MTM
* ~~Identification of the dynamic model of the PSM~~


## Requirements
* Python 2.7
* Python modules
    * NumPy, SciPy, SymPy, CvxOpt, Matplotlib, PyOpt, cloudpickle

## Application and example


## Author
Yan Wang, Radian

## Reference
When developing this work, we referred a lot from the following places:
* [SymPyBotics](https://github.com/cdsousa/SymPyBotics)
* [FloBaRoID](https://github.com/kjyv/FloBaRoID)

## Some problems
When I was using PyOpt, I found some problems with it. In pySLSQP.py file, these changes should be done.
```
gg = numpy.zeros([la], numpy.float) ==> gg = numpy.zeros(la, numpy.float)

dg = numpy.zeros([la,n+1], numpy.float) ==> dg = numpy.zeros([la[0], n + 1], numpy.float)

w = numpy.zeros([lw], numpy.float) ==> w = numpy.zeros(lw, numpy.float)

jw = numpy.zeros([ljw], numpy.intc) ==> jw = numpy.zeros(ljw, numpy.intc)
```

		
