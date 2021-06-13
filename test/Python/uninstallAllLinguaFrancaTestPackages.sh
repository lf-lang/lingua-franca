#!/bin/bash

pip3 freeze | grep "LinguaFranca" > to_delete.txt
pip3 uninstall -y -r to_delete.txt
rm -f to_delete.txt
