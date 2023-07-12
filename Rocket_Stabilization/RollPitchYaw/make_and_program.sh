#!/bin/bash

cd RollPitchYaw-udb6.X

# Recreate makefiles to make sure they match
prjMakefilesGenerator -v -mplabx-userdir=/home/jbbowen/.mplab_ide/dev/v6.05 ./

make clean
make all

# Program
pk2cmd -F dist/default/production/RollPitchYaw-udb6.X.production.hex -M -PdsPIC33FJ256GP710 -T -R -B.

# Verify
pk2cmd -F dist/default/production/RollPitchYaw-udb6.X.production.hex -PdsPIC33FJ256GP710 -Y -R -B.

cd -