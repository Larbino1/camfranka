# Downloading the code 

First, get permissions on the repository (from Daniel) 
Then clone it using

>git clone git@github.com:Larbino1/camfranka.git

There are 2 submodules, json (for parsing json files) and libfranka (the FCI library). These are their own git repositories, and are configured as git submodules.

They can be initialized and downloaded using:

>git submodule update --init --recursive --progress

The option --recursive is needed as libfranka has its own submodule.

# Building the code

To build the code, first make a directory in the camfranka folder called build, by using:

>mkdir build

Then, initialize the project with some flags for libfranka using

>cmake <>RELEASEFLAGS>

To build the entire project, from the build folder call.

> cmake --build .

To build a specific example, e.g. instrument_impedance_control

cmake --build . --target instrument_impedance_control

# Building for the Franka Panda (Freja)

The franka panda needs a different version of libfranka. First, navigate to the folder camfranka/submodules/libfranka

Then, checkout the correct version using 

>git checkout 0.9.0
>git submodule update

You MUST remember to update libfrankas submodules as well, or it will fail to compile with an unhelpful error.

Then, follow the normal build instructions.
