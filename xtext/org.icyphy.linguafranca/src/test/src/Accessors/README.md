# Lingua Franca applications for the Accessors target

This directory contains a demonstration of the Accessors target for Lingua Franca.

To run these tests/demos, you need to use npm to install some modules. However, you
should not do this in this directory (in the Lingua Franca source tree)!!
Unfortunately, if you do it in this directory, npm will remove a large number
of files that are needed in the directories containing this one.

Hence, you should create a working copy somewhere else.
For example, assuming you start in this directory (the one
with this README file), you can do this:

    mkdir ~/linguaFrancaTest
    mkdir ~/linguaFrancaTest/src
    cp -r * ~/linguaFrancaTest/src
    cd ~/linguaFrancaTest/src
    npm install @terraswarm/accessors
    npm install @accessors-modules/text-display

These can be installed in the copy directory containing this file or
any directory above it in the file system hierarchy.

To run the code generator on the .lf files, you can use the command-line code generator
described (here)[https://github.com/icyphy/lingua-franca/wiki/Downloading-and-Building#command-line-compiler]
as follows:

    cd ~/linguaFrancaTest/src
    java -jar $LF/org.icyphy.linguafranca/build/libs/org.icyphy.linguafranca-1.0.0-SNAPSHOT-all.jar HelloWorld.lf
    cd ../src-gen
    node HelloWorld.js

