# Version 0.1.0-beta-SNAPSHOT
- LF programs with the TypeScript target can now be compiled on Windows (#850).
- In the VS Code extension, generated code is validated when an LF file is saved for all targets except C (#828). Generated C code is only validated when it is fully compiled.


- Full implementation of the Savina benchmark suite in the C and C++ targets
- Added a script for conveniently running benchmarks
- Created an VS code plugin


## Language
 
 - Introduced syntax for method definitions (Only supported by the C++ target) (#382)
 - Support widths of banks and multiports to be given as runtime parameters or target code in the C++ target (#387, #420)
 - Added syntax for interleaved connections (#416)
 - New Rust target

## Compiler
 
 - Ported the C++ code generator to Kotlin (#345)
 - Enforce LF scoping rules in generated C++ code (#375)
 - Fixed support for generic reactors in the C++ target (#467)
 - Dropped the rebuild feature of lfc (#530)
 - Fix `after` for various complex connection patterns (#541, #553, #593)
 - Improved error reporting in the standalone compiler

## Libraries
 
 - Improved performance of the C++ runtime considerably
 - Added tools for exporting dependency information from the C++ runtime

## Dependencies
 - Kotlin `1.4.10` -> `1.6.10` (#866)

# Version 0.1.0-alpha (06-04-2021)
This is a preliminary release of the Lingua Franca Compiler (`lfc`), a **command-line compiler** that translates Lingua Franca programs into target language programs, and an **Eclipse-based IDE** (integrated development environment) that provides a sophisticated editor as well as a code generator. This release supports four target languages: C, C++, Python, and Typescript. See [documentation](https://github.com/icyphy/lingua-franca/wiki). Of the four target languages, C is the most complete. It supports all documented language features including an experimental implementation of [federated execution](https://github.com/icyphy/lingua-franca/wiki/Distributed-Execution).

The IDE is suitable for the following platforms:
- Linux (`lingua-franca-rca-0.1.0-alpha-linux.gtk.x86_64.tar.gz`)
- MacOS (`lingua-franca-rca-0.1.0-alpha-macosx.cocoa.x86_64.tar.gz`)
  - **IMPORTANT NOTE**: MacOS will report that `lflang.app` is broken because it was not signed. To execute it, please run `xattr -cr lflang.app` first on the command line. Eventually, we will provide a signed download.
- Windows (`lingua-franca-rca-0.1.0-alpha-win32.x86_64.zip`)

The `lfc` command line application is suitable for:
- Linux, MacOS (`lfc-0.1.0-alpha.tar.gz`)
- Windows (`lfc-0.1.0-alpha.zip`)

### System Requirements
- Java 11 or up ([download from Oracle](https://www.oracle.com/java/technologies/javase-jdk11-downloads.html))
- Various target-specific dependencies, documented [here](https://github.com/icyphy/lingua-franca/blob/7473ae1549c2b2aeed8f5469675f328d3984cb2c/REQUIREMENTS.md)
 
### IDE Features
- code generation
- diagram synthesis
- error forwarding from target compiler
- validation

### Language Features (see [language specification](https://github.com/icyphy/lingua-franca/wiki/Language-Specification))
- imports
- banks
- multiports
- list-valued parameters
- target properties
- time type

### Targets
- [C](https://github.com/icyphy/lingua-franca/wiki/Writing-Reactors-in-C)
- [C++](https://github.com/icyphy/lingua-franca/wiki/Writing-Reactors-in-Cpp)
- [Python](https://github.com/icyphy/lingua-franca/wiki/Writing-Reactors-in-Python)
- [TypeScript](https://github.com/icyphy/lingua-franca/wiki/Writing-Reactors-in-TypeScript)
