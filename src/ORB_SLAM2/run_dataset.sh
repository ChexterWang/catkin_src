## Run Monocular example, using TUM1 dataset.
#./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml /home/${USER}/Documents/dataset/TUM/rgbd_dataset_freiburg1_xyz

# Run Monocular example, using TUM2 dataset.
#./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM2.yaml /home/${USER}/Documents/dataset/TUM/rgbd_dataset_freiburg2_xyz

# Run RGD-D example, using TUM1 dataset.
#./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml /home/${USER}/Documents/dataset/TUM/rgbd_dataset_freiburg1_xyz/ ./Examples/RGB-D/associations/fr1_xyz.txt

# Run RGD-D example, using TUM2 dataset.
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM2.yaml /home/${USER}/Documents/dataset/TUM/rgbd_dataset_freiburg2_xyz/ ./Examples/RGB-D/associations/fr2_xyz.txt

# Run RGD-D example, using TUM3 dataset.
#./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM3.yaml /home/${USER}/Documents/dataset/TUM/rgbd_dataset_freiburg3_sitting_static /home/${USER}/Documents/dataset/TUM/rgbd_dataset_freiburg3_sitting_static/associate.txt
#./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM3.yaml /home/${USER}/Documents/dataset/TUM/rgbd_dataset_freiburg3_walking_xyz /home/${USER}/Documents/dataset/TUM/rgbd_dataset_freiburg3_walking_xyz/associate.txt
