#!/bin/bash

echo "Installing packages into environment."

conda install -c conda-forge pcl=1.9.1 opencv=4.2.0 eigen=3.3.7 boost-cpp=1.72.0
pip install pandas==1.0.1 numpy==1.18.1

echo "Done!"
