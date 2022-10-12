# Replace reactor-c in src-gen dir 
printenv
PROJECT_ROOT=$LF_BIN_DIRECTORY/../../..
RC=$PROJECT_ROOT/org.lflang/src/lib/c/reactor-c

cp -Lr $RC/core $LF_SOURCE_GEN_DIRECTORY/
cp -Lr $RC/lib $LF_SOURCE_GEN_DIRECTORY/
cp -Lr $RC/include $LF_SOURCE_GEN_DIRECTORY/

pushd $LF_SOURCE_GEN_DIRECTORY
cmake -B build
make -C build