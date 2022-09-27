# FLS-JAX

FLS-JAX is a JAX-based Python implementation of the functional least-square algorithm. We hope the correspondence-free and permutation-invariant point set metric, named as *delta distance*, can be used as a loss function for learning-based point cloud processing pipelines.

## Tutorials

We provide three tutorials to demonstrate different use of FLS:
 - Tutorial 1 (tutorial_1.ipynb) [Colab link](https://colab.research.google.com/github/MurpheyLab/FLS/blob/master/fls-jax/tutorial_1.ipynb): This tutorial walks through orthonormal decomposition of a point cloud (as a delta-mixture function) using normalized Fourier basis functions. It also demonstrates how to use the decomposed Fourier coefficients as an implicit representation of the spatial occpuancy field of the point cloud.
 - Tutorial 2 (tutorial_2.ipynb) [Colab link](https://colab.research.google.com/github/MurpheyLab/FLS/blob/master/fls-jax/tutorial_2.ipynb): This tutorial walks through registering two point clouds (with known scale) using delta-distance as the distance metric. 
 - Tutorial 3 (tutorial_3.ipynb) [Colab link](https://colab.research.google.com/github/MurpheyLab/FLS/blob/master/fls-jax/tutorial_3.ipynb): This tutorial walks through the process of decoupled scale estimation with FLS.
