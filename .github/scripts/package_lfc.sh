#!/usr/bin/env bash

set -euo pipefail

# build lf compiler
./gradlew clean buildLfc

# find the version number
jar_path="org.lflang.lfc/build/libs/org.lflang.lfc-*-all.jar"
version="$(ls ${jar_path} | xargs -n 1 basename | sed 's/^org.lflang.lfc-\(.*\)-all.jar$/\1/')"

# use a different naming convention for nightly build artifacts
if [[ "$#" > 0 && "$1" = "nightly" ]]; then
  echo "Packaging Lingua Franca Nightly Build"
  outname="lfc_nightly_$(date '+%Y%m%d-%H%M%S')"
else
  echo "Packaging Lingua Franca v${version}"
  outname="lfc_${version}"
fi

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
