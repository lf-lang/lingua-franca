# Version 0.1.0-beta-SNAPSHOT
- LF programs with the TypeScript target can now be compiled on Windows (#850).
- In the VS Code extension, generated code is validated when an LF file is saved for all targets except C (#828). Generated C code is only validated when it is fully compiled.

## Language

## Compiler

## Libraries

## Tools
 - added a version bump script (#829)

## Dependencies
 - eclipse.core.resources `3.15.0` -> `3.16.0` (#829)
 - eclipse.core.runtime `3.22.0` -> `3.24.0` (#829)
 - exec-maven-plugin `1.6.0` -> `3.0.0` (#829)
 - gradle `6.5` -> `7.0` (#829)
 - junit `4.12` -> `4.13.2` (#829)
 - junit-jupiter-* `5.7.2` -> `5.8.2` (#829)
 - junit-platform-* `1.7.2` -> `1.8.2` (#829)
 - kotlin `1.4.10` -> `1.6.10` (#866)
 - lsp4j `0.10.0` -> `0.12.0` (#829)
 - shadowJar `6.0.0` -> `7.1.2` (#829)
 - xtext-gradle-plugin -> `2.0.8` -> `3.0.0` (#829)
 
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

### Bug Fixes
- fixed an issue where top-level custom Python classes were being serialized
  incorrectly

### New Features
- [Python] `bank_index` (useful for banks of reactors) is now a proper parameter
  that can be passed down the reactor hierarchy via parameter assignment. For
  example, the following code snippet now works as expected:
  ```Python
  target Python;
  reactor Bar (bank_index(0), parent_bank_index(0)) {
    reaction(startup) {= 
      print(f"My parent bank index is {self.parent_bank_index}.") 
    =}
  }
  reactor Foo (bank_index(0)) {
    bar = new[2] Bar(parent_bank_index = bank_index)
  }
  main reactor {
    f = new[2] Foo()
  }
  ```
  The output will be:

  ```bash
  My parent bank index is 0.
  My parent bank index is 1.
  My parent bank index is 1.
  My parent bank index is 0.
  ```
