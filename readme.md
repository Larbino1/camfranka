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

>cmake -DCMAKE_BUILD_TYPE=Release ..

To build the entire project, from the build folder call.

> cmake --build .

To build a specific example, e.g. instrument_impedance_control

cmake --build . --target instrument_impedance_control


# Before running

The user profile on the compute must be part of the 'realtime' group to allow realtime programs.

This can be done with:
sudo useradd <user> realtime

where <user> is replaced with your username.

Otherwise you will get a libfranka error "libfranka: unable to set realtime scheduling: Operation not permitted"

# Building for the Franka Panda (Freja)

The franka panda needs a different version of libfranka. First, navigate to the folder camfranka/submodules/libfranka

Then, checkout the correct version using 

>git checkout 0.9.0
>git submodule update

You MUST remember to update libfrankas submodules as well, or it will fail to compile with an unhelpful error.

Then, follow the normal build instructions.


# Installing SDL2

SDL2 is a library used to communicate with the xbox controller. It can be installed on ubuntu with the following commands:
sudo apt-get install libsdl2-2.0-0
sudo apt-get install libsdl2-dev

In the event of an error, see:
https://wiki.libsdl.org/SDL2/Installation
