#!/usr/bin/env bash

set -euo pipefail

# build lf cli tools
./gradlew clean buildAll

# find the version number
jar_path="org.lflang/build/libs/org.lflang-*.jar"
version="$(ls ${jar_path} | xargs -n 1 basename | sed 's/^org.lflang-\(.*\).jar$/\1/')"

# use a different naming convention for nightly build artifacts
if [[ "$#" > 0 && "$1" = "nightly" ]]; then
  echo "Packaging Lingua Franca Nightly Build"
  outname="lf-cli-nightly-$(date '+%Y%m%d-%H%M%S')"
else
  echo "Packaging Lingua Franca v${version}"
  outname="lf-cli-${version}"
fi

# assemble the files in a separate directory
mkdir -p "${outname}/bin"
mkdir -p "${outname}/lib/scripts"
mkdir -p "${outname}/lib/jars"

# move the jar
mv org.lflang/build/libs/org.lflang-*.jar "${outname}/lib/jars"

# copy the Bash scripts
cp -a lib/scripts "${outname}/lib/"
ln -s "../lib/scripts/launch.sh" "${outname}/bin/lfc"
ln -s "../lib/scripts/launch.sh" "${outname}/bin/lff"
# copy the PowerShell script
cp bin/lfc.ps1 "${outname}/bin/lfc.ps1"
cp bin/lff.ps1 "${outname}/bin/lff.ps1"

# zip/tar everything - the files will be put into the build_upload directory
mkdir -p build_upload
zip --symlinks -r "build_upload/${outname}.zip" "${outname}"
tar cvf "build_upload/${outname}.tar.gz" "${outname}"
