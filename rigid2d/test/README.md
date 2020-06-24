# Testing File Format
I wrote json files (at test/inputs/testX.json) containing T_ab and T_bc. The user's command line argument will specify which test/label json files to use. Main.cpp will parse these input files and compute T_ab, T_ba, T_bc, T_cb, T_ac, T_ca. I manually calculated these frames and put the labels in json files at test/outputs/testX.json. For example, ```./rigid2d_test 3``` will use test/inputs/test3.json as input and /test/outputs/test3.json as its label


# Describe Each File
test1.txt/test1.json has T_ab rotate and translate and T_bc also rotate and translate <br \>
test2.txt/test2.json has each frame each being the identity <br \>
test3.txt/test3.json has T_ab as a translation and T_bc as a rotation <br \>
 
REMEMEBR that the angles in the transforms are entered in degrees and under the hood converted to radians

