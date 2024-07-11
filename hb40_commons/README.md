# ROS2_MAB_COMMONS
Commons package is designed as header-only library for other packages, but also incudes some helper nodes, for visualization, development and debugging.

## How to add common include to external package
To add include, one must modify the CMakeLists.txt file by adding:
```
find_package(hb40_commons REQUIRED)
include_directory("./../hb40_commons/include")
```
before creating the executable.

Then common header files can be included into cpp/hpp files as they would be in the same package.
```
#include "kinematics_commons.hpp"
```

## Development
Due to header-only style of the library, all variables stored in commons files, must be declared as
```
const
```
, all funcions must be declared as
```
inline 
```