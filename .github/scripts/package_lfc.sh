#!/usr/bin/env bash

set -euo pipefail

outname="lfc_nightly_$(date '+%Y%m%d-%H%M%S')"

# build lf compiler
./gradlew buildLfc

# assemble the files in a separate directory
mkdir -p "${outname}/bin"
mkdir -p "${outname}/lib/scripts"
mkdir -p "${outname}/lib/jars"

# move the jar
mv org.lflang.lfc/build/libs/org.lflang.lfc-*-all.jar "${outname}/lib/jars"

# copy the Bash scripts
cp -a lib/scripts "${outname}/lib/"
ln -s "../lib/scripts/launch.sh" "${outname}/bin/lfc"
# copy the PowerShell script
cp bin/lfc.ps1 "${outname}/bin/lfc.ps1"

# zip/tar everything - the files will be put into the build_upload directory
mkdir -p build_upload
zip --symlinks -r "build_upload/${outname}.zip" "${outname}"
tar cvf "build_upload/${outname}.tar.gz" "${outname}"
