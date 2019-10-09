#!/bin/bash

# example: bash compileHPNTool.sh /home/maximus/programs/edytory/qt5/5.11.1

# within build folder there should be platform folder containing libqxcb.so and libqxcb.so.debug from qt5/5.11.1/gcc_64/packages/platforms

# ustawić odpowiednią ścieżkę do Qt - miejsce w którym zainstalowano Qt5.9.1 lub wyższe
#PATH_TO_QT5_INSTALLATION=/home/max/Programs/Qt5.9.1 


PATH_TO_QT5_INSTALLATION=$1

if [ "$#" -ne 1 ]; then
	echo "Wywołanie nieprawidłowe - podaj ścieżkę do folderu instalacyjnego dla Qt - zawierającą qmake (ang. Set path to Qt, i.e. where gcc_64/bin/qmake resides)"
	PATH_TO_QT5_INSTALLATION=~/programs/edytory/qt5/5.11.1
fi 

cd ..
mkdir build
cd build
path=whoami
#make clean
#cmake .. -DCMAKE_PREFIX_PATH=$PATH_TO_QT5_INSTALLATION/gcc_64
$PATH_TO_QT5_INSTALLATION/gcc_64/bin/qmake -o Makefile ../PetriNetEditor/PetriNetEditor.pro -spec linux-g++ 
make

#./PetriNetEditor
