# Description

# Compiling 
I am using Eigen and it is installed at /usr/include/eigen3 so I need to link with -I 
```g++ -Wall -Wextra -g -std=c++17 -I /usr/include/eigen3 -o rigid2d_test main.cpp rigid2d.cpp -ljsoncpp```

Json parser compiling. I had too many issues with the yaml-cpp package so I used JSON. One can also install jsoncpp with the apt <br \>
```sudo apt-get install libjsoncpp-dev```
```sudo g++ jsonTest.cpp -ljsoncpp```


# Describe the testing 
Describe the folders of test files
