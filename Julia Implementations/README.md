# Julia Implementation of Code:
---
This file includes the Julia implementations of the Matlab code in the repository. They are Jupyter notebooks, but can be run on VS code as well. To run the code, you must first install Julia on your PC (you can use this link: https://julialang.org/downloads/). To be able to run the code on jupyter notebooks you will have to create a connection between Julia and the jupyter notebooks application by following these steps:

1) Click on the Julia application to open the Julia command prompt
2) Type in these commands:
    * Using Pkg
    * Pkg.add("IJulia")

## Additional Required Libraries:
---
You will also need to enter the command Pkg.add(" ") to install the following libraries which are used by the code:
* NBInclude
* Convex
* LinearAlgebra
* MosekTools
* PyPlot
* StatsBase
* Random
* Distributions
* CSV
* DataFrames
* Infinity

## MOSEK:
---
In order to use Mosek, you will need to obtain a license (use this link: https://www.mosek.com/). MosekTools.jl is the Julia Implementation of the MOSEK solver which is used to solve the conic optimization probelms within the simulations. 
