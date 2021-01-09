import re


# source_path = "../data/plyfolder/points20.ply"
# target_path = "../data/plyfolder/points0.ply"

source_path = "../stiched/stitch_99_97.pcd"
target_path = "../stiched/stitched_internal_pc_98.pcd"

# source_path = "../stiched/stitch_99_98.pcd"

regex = re.compile(r'\d+')
s = regex.findall(source_path)
p = regex.findall(target_path)

voxel_size = 0.2
threshold = 0.02
