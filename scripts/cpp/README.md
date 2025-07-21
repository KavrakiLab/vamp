# Using VAMP inside of C++ projects

## C++ Demo

The VAMP C++ demo can be compiled with (in the VAMP directory):
```bash
cmake -Bbuild -GNinja -DVAMP_BUILD_CPP_DEMO=On .
cmake --build build
```

## OMPL Integration Demo

We provide an example of using VAMP as a motion validator and collision checker inside the [https://ompl.kavrakilab.org/index.html](Open Motion Planning Library), given in `ompl_integration.cc`. Please note that - although offering a substantial speedup to OMPL planners - this example is not engineered for maximum performance (e.g., it relies on state copying between OMPL and VAMP representations, etc.).

If you do not have an installation of OMPL already (or wish to do local development of OMPL with VAMP as a collision checking backend), we recommend installing the latest version of OMPL to a local directory:
```bash
cd
git clone git@github.com:ompl/ompl.git
cd ompl/
# Here we are installing OMPL to a directory in the home folder, ~/ompl_install
cmake -Bbuild -GNinja -DCMAKE_INSTALL_PREFIX=~/ompl_install -DOMPL_REGISTRATION=Off -DOMPL_BUILD_DEMOS=Off -DOMPL_BUILD_TESTS=Off .
cmake --build build
cmake --install build
```

The VAMP demo can be compiled with (in the VAMP directory):
```bash
# Specify OMPL demos are to be build with -DVAMP_BUILD_OMPL_DEMOS=On
# If using a local installation, specify the OMPL path with VAMP_OMPL_PATH
cmake -Bbuild -GNinja -DVAMP_BUILD_OMPL_DEMO=On -DVAMP_OMPL_PATH=~/ompl_install .
cmake --build build
```

To run the demo with a locally installed OMPL, run the demo with a custom `LD_LIBRARY_PATH`:
```bash
LD_LIBRARY_PATH="~/ompl_install/share/;$LD_LIBRARY_PATH" ./build/vamp_ompl_integration
```
