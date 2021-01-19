# Snake AI
# Current Version 1.0
## Dependencies
There exist three dependencies:

1. numpy
2. open3d
3. matplotlib
4. copy
5. pickle
6. statistics
7. scikit-spatial
8. regex

To install dependencies, run `pip3 install -r requirements.txt`

## Getting started

1. In order for the stitching to work there should be the folders data/plyfolder/external, data/plyfolder/internal, data/plyfolder/upper  containing .ply files 
2. Run stitching.py from /src directory 


## Generated Files

1. After running stitching.py  it will generate the pcd files in the /stitched folder.
2. external.pcd, internal.pcd and upper.pcd are the corresponding stitched point clouds of each side.
3. final_stiched.pcd is the point cloud after stitching external, internal and upper.
4. final_clustered is the clustered point cloud where we have removed the outliers.


