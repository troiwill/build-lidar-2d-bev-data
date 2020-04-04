# Read me
## Description
This repository builds a program that:
1. Creates 2D bird's eye view (BEV) images of online LiDAR scans,
2. Aggregates 2D BEV images to create a BEV map, and
3. Outputs a information file (info.txt) that relates each BEV scan to a pixel position in the BEV map.

## Instructions
### Building the program.
1. Install Anaconda Python.
2. Clone this repository.
3. Execute the following commands:
```
# Change into repo.
cd build-lidar-2d-bev-data

# Set up environment.
source env/start_env.sh
bash env/install_packages.sh

# Build the program.
mkdir -p build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../bin
make -j && make install
```

### Running the program.
To run the program:
1. Go to the *bin* directory in the repo. That is, `cd bin` from repo root.
2. Run `./build_kitti_bev_data --basedir <basedir> --savedir <savedir> --res <res> ...` with your choice of the remaining arguments.
