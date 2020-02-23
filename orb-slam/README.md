to build the code:

cd orb-slam
chmod +x build.sh
./build.sh




to run the code: 

./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.bin Examples/Stereo/KITTI00-02.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER 1




if there is a map saved with the name map.bin in the orb-slam directory then the slam would always start in 'only localization' mode and would load the map. 

if the last arguement is '1', the slam would save the map. 
if the last arguement is '0', the slam would not save the map.
