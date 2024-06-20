# Using VAMP inside of C++ projects

## OMPL Integration Demo

We recommend installing the latest version of OMPL to a local directory:
```bash
git clone git@github.com:ompl/ompl.git
cd ompl/
cmake -Bbuild -GNinja -DCMAKE_INSTALL_PREFIX=~/ompl_install -DOMPL_REGISTRATION=Off -DOMPL_BUILD_DEMOS=Off -DOMPL_BUILD_TESTS=Off .
cmake --build build
cmake --install build
```

The VAMP demo can be compiled with (in the VAMP directory):
```bash
cmake -Bbuild -GNinja -DVAMP_BUILD_CPP_DEMOS=On -DVAMP_OMPL_PATH=~/ompl_install .
cmake --build build
```

