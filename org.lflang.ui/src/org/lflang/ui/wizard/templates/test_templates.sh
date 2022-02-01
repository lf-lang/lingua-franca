#!/bin/bash
set -e
find .  -name *.lf -print0 | xargs -0 -n1 lfc
