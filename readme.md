# Downloading the code 

First, get permissions on the repository (from Daniel) 
Then clone it using

`git clone git@github.com:Larbino1/camfranka.git`

There are 2 submodules, json (for parsing json files) and libfranka (the FCI library). These are their own git repositories, and are configured as git submodules.

They can be initialized and downloaded using:

`git submodule update --init --recursive --progress`

The option `--recursive` is needed as libfranka has its own submodules. `--init` will initialize unitinitalized submodules, and `--progress` enables progress monitoring.

# Installing Dependencies

Libfranka (which is checked out as part of camfranka) has several dependencies. These can be easily installed on linux with the following command:

`sudo apt install build-essential cmake git libpoco-dev libeigen3-dev`

For reference see [libfranka installation on linux](https://frankaemika.github.io/docs/installation_linux.html)

SDL2 is a library used to communicate with the xbox controller. It can be installed on ubuntu with the following commands:

`sudo apt-get install libsdl2-dev`

In the event of an error, see [SDL2 Installation](https://wiki.libsdl.org/SDL2/Installation)


# Building the code

To build the code, first make a directory in the camfranka folder called build, by using:

```
cd ~/camfranka
mkdir build
cd build
cp ../scripts/setup_cmake.sh ./
./setup_cmake.sh
```

To build the entire project, call `cmake --build .`

The dot specifies that you wish to build in the current folder, so make sure you are in the build folder or you will make a mess! Building the entire project can be very slow. Instead you can build a specific cmake target (and its dependencies) with the argument `--target`. For example, to build the *instrument_impedance_control* example run

`cmake --build . --target instrument_impedance_control`

# Before running

The user profile on the compute must be part of the 'realtime' group to allow realtime programs.

This can be done with:

`sudo useradd <user> realtime`

where `<user>` is replaced with your username.

Otherwise you will get a libfranka error

>`libfranka: unable to set realtime scheduling: Operation not permitted`

# Building for the Franka Panda (Freja)

The franka panda needs a different version of libfranka. First, navigate to the folder *camfranka/submodules/libfranka*

Then, checkout the correct version using 

```
git checkout 0.9.0
git submodule update --init --recursive --progress
```

You MUST remember to update libfrankas submodules as well, or it will fail to compile with an unhelpful error.

Then, follow the normal build instructions.


