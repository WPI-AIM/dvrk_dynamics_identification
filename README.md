# A dynamic model identification framework developed for daVinci Suigical System

There are parallelograms, springs and counter weights in daVinci surgical system so that we can not use existing tools
to solve the dynamics model identification of it. This software framework was developed to solve these problems.

Uses SymPy, Numpy, PyOpt, Matplotlib and Cvxpy modules

## Features
### Finished
* Symbolic dynamic modelling based on DH parameters and geometric tree
* Optimal excitation trajectory generation based on fourier-series
### Yet to be done
* Data processing
    * Derive velocity and acceleration from position measurements
    * Zero-phase low-pass filter for original data
    * Remove the data whose velocity is close to zero to decrease the noise for Coulomb friction
* Excitation of robots, using dvrk ROS library
* Output of the identified paramters
* Output of C++ or Python code of identified dynamic model for control use 

## Requirements
* Python 2.7
* Python modules
    * numpy, scipy, sympy, cvxopt, matplotlib

## Example


## Author
Yan Wang, Radian

## Reference
When developing this work, we referred a lot from the following places:
* https://github.com/cdsousa/SymPyBotics
* https://github.com/kjyv/FloBaRoID
