# Changelog

## [v0.1.0-beta](https://github.com/lf-lang/lingua-franca/releases/tag/v0.1.0-beta) (02-01-2022)

### Compiler
- Reduced size of generated C code when using banks and multiports ([#759](https://github.com/lf-lang/lingua-franca/pull/759) and [#875](https://github.com/lf-lang/lingua-franca/pull/875))
- Significantly reduced memory footprint and compilation time ([#759](https://github.com/lf-lang/lingua-franca/pull/759) and [#875](https://github.com/lf-lang/lingua-franca/pull/875))
- Ported the C++ code generator to Kotlin ([#345](https://github.com/lf-lang/lingua-franca/pull/345))
- Ported the TypeScript code generator to Kotlin ([#431](https://github.com/lf-lang/lingua-franca/pull/431), [#486](https://github.com/lf-lang/lingua-franca/pull/486))
- Added enforcement of LF scoping rules in generated C++ code ([#375](https://github.com/lf-lang/lingua-franca/pull/375))
- Fixed support for generic reactors in the C++ target ([#467](https://github.com/lf-lang/lingua-franca/pull/467))
- Dropped the rebuild feature of lfc ([#530](https://github.com/lf-lang/lingua-franca/pull/530))
- Fixed `after` for various complex connection patterns ([#541](https://github.com/lf-lang/lingua-franca/pull/541), [#553](https://github.com/lf-lang/lingua-franca/pull/553), [#593](https://github.com/lf-lang/lingua-franca/pull/593))
- Improved error reporting in the standalone compiler ([#543](https://github.com/lf-lang/lingua-franca/pull/543))
- The C target now uses CMake to compile generated code ([#402](https://github.com/lf-lang/lingua-franca/pull/402))

### Dependencies
- eclipse.core.resources `3.15.0` -> `3.16.0` ([#829](https://github.com/lf-lang/lingua-franca/pull/829))
- eclipse.core.runtime `3.22.0` -> `3.24.0` ([#829](https://github.com/lf-lang/lingua-franca/pull/829))
- exec-maven-plugin `1.6.0` -> `3.0.0` ([#829](https://github.com/lf-lang/lingua-franca/pull/829))
- gradle `6.5` -> `7.0` ([#829](https://github.com/lf-lang/lingua-franca/pull/829))
- junit `4.12` -> `4.13.2` ([#829](https://github.com/lf-lang/lingua-franca/pull/829))
- junit-jupiter-* `5.7.2` -> `5.8.2` ([#829](https://github.com/lf-lang/lingua-franca/pull/829))
- junit-platform-* `1.7.2` -> `1.8.2` ([#829](https://github.com/lf-lang/lingua-franca/pull/829))
- kotlin `1.4.10` -> `1.6.10` ([#866](https://github.com/lf-lang/lingua-franca/pull/866))
- lsp4j `0.10.0` -> `0.12.0` ([#829](https://github.com/lf-lang/lingua-franca/pull/829))
- shadowJar `6.0.0` -> `7.1.2` ([#829](https://github.com/lf-lang/lingua-franca/pull/829))
- xtext-gradle-plugin -> `2.0.8` -> `3.0.0` ([#829](https://github.com/lf-lang/lingua-franca/pull/829))

### Federation support
- Added support for federated Python programs
- Added support for federated TypeScript programs ([#538](https://github.com/lf-lang/lingua-franca/pull/538), [#596](https://github.com/lf-lang/lingua-franca/pull/596), [#646](https://github.com/lf-lang/lingua-franca/pull/646), [reactor-ts#71](https://github.com/lf-lang/reactor-ts/pull/71))
- Enhanced support for Docker containers (including for federated programs) ([#700](https://github.com/lf-lang/lingua-franca/pull/700), [#750](https://github.com/lf-lang/lingua-franca/pull/700), [#754](https://github.com/lf-lang/lingua-franca/pull/754))
- Added built-in support for ROS 2 serialization ([#449](https://github.com/lf-lang/lingua-franca/pull/449))
- RTI is now a standalone application ([#395](https://github.com/lf-lang/lingua-franca/pull/395))

### Language
 - Introduced syntax for method definitions (currently only supported by the C++ target) ([#382](https://github.com/lf-lang/lingua-franca/pull/382))
- Added support for giving widths of banks and multiports as runtime parameters or target code in the C++ target ([#387](https://github.com/lf-lang/lingua-franca/pull/387), [#420](https://github.com/lf-lang/lingua-franca/pull/420))
- Added syntax for interleaved connections ([#416](https://github.com/lf-lang/lingua-franca/pull/416))
- Created a new Rust target ([#488](https://github.com/lf-lang/lingua-franca/pull/488), [#628](https://github.com/lf-lang/lingua-franca/pull/628))
- Added the CCpp target, which accepts C++ code but is supported by the [C runtime](https://github.com/lf-lang/reactor-c) ([#513](https://github.com/lf-lang/lingua-franca/issues/531))

### Platform support
- LF programs with the TypeScript target can now be compiled on Windows ([#850](https://github.com/lf-lang/lingua-franca/pull/850)).
- Added Windows support for the C and Python targets ([#532](https://github.com/lf-lang/lingua-franca/pull/532))

### Runtime
- Implemented the Savina benchmark suite in the C, C++, and Rust target (modulo those that require mutations)
- Improved performance of the C++ runtime considerably

#### Python
- Multiports and banks are now iterable in the Python target ([#713](https://github.com/lf-lang/lingua-franca/pull/713))
- Fixed an issue where top-level custom Python classes were being serialized incorrectly
- `bank_index` (useful for banks of reactors) is now a proper parameter ([#424](https://github.com/lf-lang/lingua-franca/pull/424))
  that can be passed down the reactor hierarchy via parameter assignment.

### Tool support
- Created an Language and Diagram Server that enables our new [VS Code extension](https://github.com/lf-lang/vscode-lingua-franca)
#### VS Code extension
- Generated code is now validated when an LF file is saved for all targets except C ([#828](https://github.com/lf-lang/lingua-franca/pull/828)). Generated C code is only validated when it is fully compiled.
#### Epoch
- Added compile button as an alternative to the Eclipse automatic build feature ([#848](https://github.com/lf-lang/lingua-franca/pull/848))
- Added terminal window support ([#509](https://github.com/lf-lang/lingua-franca/pull/509))
- Updated icons

### Utilities
- Added tools for exporting dependency information from the C++ runtime ([#788](https://github.com/lf-lang/lingua-franca/pull/788))
- Added tracing support for the Python target ([#568](https://github.com/lf-lang/lingua-franca/pull/568))
- Added a script for conveniently running benchmarks ([#243](https://github.com/lf-lang/lingua-franca/pull/243))
- Added version bump script ([#829](https://github.com/lf-lang/lingua-franca/pull/870))

## [v0.1.0-alpha](https://github.com/lf-lang/lingua-franca/releases/tag/v0.1.0-alpha) (06-04-2021)
This is a preliminary release of the Lingua Franca Compiler (`lfc`), a **command-line compiler** that translates Lingua Franca programs into target language programs, and an **Eclipse-based IDE** (integrated development environment) that provides a sophisticated editor as well as a code generator. This release supports four target languages: C, C++, Python, and Typescript. See [documentation](https://github.com/icyphy/lingua-franca/wiki). Of the four target languages, C is the most complete. It supports all documented language features including an experimental implementation of [federated execution](https://github.com/icyphy/lingua-franca/wiki/Distributed-Execution).

The IDE is suitable for the following platforms:
- Linux (`lingua-franca-rca-0.1.0-alpha-linux.gtk.x86_64.tar.gz`)
- MacOS (`lingua-franca-rca-0.1.0-alpha-macosx.cocoa.x86_64.tar.gz`)
  - **IMPORTANT NOTE**: MacOS will report that `lflang.app` is broken because it was not signed. To execute it, please run `xattr -cr lflang.app` first on the command line. Eventually, we will provide a signed download.
- Windows (`lingua-franca-rca-0.1.0-alpha-win32.x86_64.zip`)

The `lfc` command line application is suitable for:
- Linux, MacOS (`lfc-0.1.0-alpha.tar.gz`)
- Windows (`lfc-0.1.0-alpha.zip`)

#### System Requirements
- Java 11 or up ([download from Oracle](https://www.oracle.com/java/technologies/javase-jdk11-downloads.html))
- Various target-specific dependencies, documented [here](https://github.com/icyphy/lingua-franca/blob/7473ae1549c2b2aeed8f5469675f328d3984cb2c/REQUIREMENTS.md)
 
#### IDE Features
- code generation
- diagram synthesis
- error forwarding from target compiler
- validation

#### Language Features (see [language specification](https://github.com/icyphy/lingua-franca/wiki/Language-Specification))
- imports
- banks
- multiports
- list-valued parameters
- target properties
- time type

#### Targets
- [C](https://github.com/icyphy/lingua-franca/wiki/Writing-Reactors-in-C)
- [C++](https://github.com/icyphy/lingua-franca/wiki/Writing-Reactors-in-Cpp)
- [Python](https://github.com/icyphy/lingua-franca/wiki/Writing-Reactors-in-Python)
- [TypeScript](https://github.com/icyphy/lingua-franca/wiki/Writing-Reactors-in-TypeScript)
