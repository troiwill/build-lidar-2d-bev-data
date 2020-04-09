#!/bin/bash

# Variables for environment.
ENV_NAME="l2d_bev"
REPO_DIR="build-lidar-2d-bev-data"

# Check if conda is installed.
if [[ ! $(command -v conda) ]]; then
    printf "Cannot find conda.\n"
fi

# Check if the environment was set up.
EXISTS=0

if [[ $(conda env list | grep "$ENV_NAME") == "" ]];
then
    printf "The $ENV_NAME conda environment does not exist.\n"

    RESPONSE=""
    while [[ "$RESPONSE" != "y" && "$RESPONSE" != "n" ]];
    do
        printf "Would you like me to create the environment? (y/n) "
        read RESPONSE
        printf "\n"
    done

    if [[ "$RESPONSE" == "y" ]];
    then
        printf "Creating a conda environment for $ENV_NAME.\n"
        conda config --add channels anaconda
        conda create -n $ENV_NAME python=3.8 && EXISTS=1
        printf "Once the environment is activated, run ''install_packages.sh''\n"
    fi
else
    EXISTS=1
fi


if [[ $EXISTS == 0 ]];
then
    printf "Cannot activate $ENV_NAME because it does not exist.\n"

else
    export WORK_REPO_ENV="$HOME/repos/$REPO_DIR"
    export PYTHONPATH=$WORK_REPO_ENV:$PYTHONPATH
    source activate $ENV_NAME
    export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"
fi
