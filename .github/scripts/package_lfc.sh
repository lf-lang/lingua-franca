#!/usr/bin/env bash

set -euo pipefail

cd $GITHUB_WORKSPACE

outname="lfc_nightly_$(date '+%Y%m%d-%H%M%S')"

# build lf compiler
./gradlew buildLfc

# assemble the files in a separate directory
mkdir -p "${outname}/bin"
mkdir -p "${outname}/lib"

# move the jar
mv org.lflang.lfc/build/libs/org.lflang.lfc-*-SNAPSHOT-all.jar "${outname}/lib"

# copy the Bash script
cp bin/lfc "${outname}/bin/lfc"
# copy the PowerShell script
cp bin/lfc "${outname}/bin/lfc.ps1"

# zip/tar everything - the files will be put into the build_upload directory
mkdir -p build_upload
zip -r "build_upload/${outname}.zip" "${outname}"
tar cvf "build_upload/${outname}.tar.gz" "${outname}"

