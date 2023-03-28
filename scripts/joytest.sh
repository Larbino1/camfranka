#! /bin/sh
./joyInput/JoyInput > mypipe & cat < ./examples/sandbox < mypipe
