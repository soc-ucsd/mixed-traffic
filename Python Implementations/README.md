# Python Implementation of Code
Here, there are some implementations in Python for the Matlab code in the repository.

In order to run this code, you will need both the [NumPy](https://numpy.org/install/), and [CVXPY](https://www.cvxpy.org/install/) libraries installed (Recommended using pip to install both).

It is recommended to run 'pip install --upgrade pip' in order to update pip to the latest release before installing. 

## Installing NumPy
  1. Ensure that Python is installed, and that pip is updated. 
  2. Run 'pip install numpy' in a terminal window.

## Installing CVXPY
  1. Ensure that Python is installed, and that pip is updated. 
  2. Install the Xcode Command Line tools - this can be done through the app store, or by running 'xcode-select --install' in a terminal window, and following the prompts for installation.
  3. Run 'pip install cvxpy' in a terminal window. 


## Installing MOSEK
In order to properly utilize CVXPY, you will need to install the [MOSEK solver](https://docs.mosek.com/latest/install/installation.html) as well, which can also be done with pip.


Alongside installing the MOSEK solver, it is required to get a free academic [MOSEK license](https://www.mosek.com/products/academic-licenses/) which is valid for 365 days. This will give you a 'mosek.lic' file, which needs to be placed in a folder titled 'mosek' in the user directory. 
