# Gtest Framework
I used gtest to unit test my functions. To test, go to catkin_ws and run ```catkin_make run_tests````

To help myself stay organized, I wrote json files (at test/inputs/testX.json) containing T_ab and T_bc, a few twists, and transformation of the original data. I put them in json files to make it easier for me to organize and modify later. I also wrote json files describing the transformations of the original data at test/outputs/testX.json 

# Description of Each JSON File
test0.txt/test0.json tests the reading in of frames
test1.txt/test1.json has T_ab rotating and translating and T_bc also rotating and translating <br \>
test2.txt/test2.json has each frame each being the identity <br \>
test3.json has T_ab as a translation and T_bc as a rotation <br \>
test4.json has T_ab as a pure translation and T_bc as a pure rotation <br \>
test5.json has T_ab as a pure rotation and T_bc as a pure rotation <br \>

REMEMEBR that the angles in the transforms are entered in degrees and under the hood converted to radians

