#!/bin/bash

# install if cannot run program: sudo apt-get install libx11-xcb1

LD_LIBRARY_PATH=/usr/local/lib:~/programs/edytory/qt5/5.11.1/gcc_64/lib

cd ../build
./PetriNetEditor
