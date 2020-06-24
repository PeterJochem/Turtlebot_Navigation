# Description

# Compiling 
I am using Eigen and it is installed at /usr/include/eigen3 so I need to link with -I 
```g++ -Wall -Wextra -g -std=c++17 -I /usr/include/eigen3 -o rigid2d_test main.cpp rigid2d.cpp```

The compiling command for the libyaml code is
```git clone https://github.com/jbeder/yaml-cpp```
```cd yaml-cpp```
```mkdir build```
```cmake .. -DYAML_BUILD_SHARED_LIBS=ON```
```cd ..```
```make```
```sudo make install```
```sudo g++ -L/home/peter/local/yaml-cpp/build -I/home/peter/local/yaml-cpp/include -std=c++11 yamlTest.cpp -lyaml-cpp```


# Describe the testing 
Describe the folders of test files
