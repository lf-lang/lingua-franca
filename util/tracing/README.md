## util/tracing

This directory contains the source code for utilities that are standalone executables 
for post-processing tracing data created by the tracing function in Lingua Franca.

* trace\_to\_csv: Creates a comma-separated values text file from a binary trace file.
  The resulting file is suitable for analyzing in spreadsheet programs such as Excel.
  
* trace\_to\_chrome: Creates a JSON file suitable for importing into Chrome's trace
  visualizer. Point Chrome to chrome://tracing/ and load the resulting file.
  
## Installing

```
    make install
```
This puts the executables in lingua-franca/bin.