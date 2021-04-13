## /lib/C/util/

This directory contains source files for use by Lingua Franca programs using the
C target. To use a file in this directory, specify a target property as follows:
```
target C {
    files: ["/lib/C/util/filename", ...],
    ...
};
```
This causes the `lfc` or LF IDE to copy the file into your project's `src-gen`
directory. Hence, you can include the file in your C code as follows:
```
preamble {= 
    #include "filename"
=}
```