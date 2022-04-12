# Changelog
 
## [v0.1.0](https://github.com/lf-lang/lingua-franca/tree/v0.1.0) (2022-04-11)

**Highlights:**

This is the first stable release of Lingua Franca. Aside from numerous bugfixes, a number of new features have been introduced since [v0.1.0-beta](https://github.com/lf-lang/lingua-franca/releases/tag/v0.1.0-beta), including: generation of ROS2 nodes directly from LF code (C++); improved error handling; support for multiports and banks (TypeScript); modular support for runtime schedulers (C); and modal reactors (C and Python). Finally, a major refactoring has been performed in which all remaining Xtend code was ported to Java.

[Full Changelog](https://github.com/lf-lang/lingua-franca/compare/v0.1.0-beta...v0.1.0)

**Implemented enhancements:**

- Add a `-v` / `--version` flag to `lfc` [\#927](https://github.com/lf-lang/lingua-franca/issues/927)
- Produce informative message when `lfc` is run on non-LF files [\#919](https://github.com/lf-lang/lingua-franca/issues/919)
- Make Docker generation compatible with the `files` target property [\#887](https://github.com/lf-lang/lingua-franca/issues/887)
- Preconfigure Epoch with example projects [\#374](https://github.com/lf-lang/lingua-franca/issues/374)
- In banks of reactors, make `bank_index` a proper parameter [\#343](https://github.com/lf-lang/lingua-franca/issues/343)
- Align the meaning of the `threads` target property across targets [\#290](https://github.com/lf-lang/lingua-franca/issues/290)
- Improve Stability of Positioning Edges connected to Reactions in Diagrams [\#1040](https://github.com/lf-lang/lingua-franca/pull/1040) ([a-sr](https://github.com/a-sr))
- Improve layout options [\#1015](https://github.com/lf-lang/lingua-franca/pull/1015) ([soerendomroes](https://github.com/soerendomroes))
- Support modal models in Python [\#1009](https://github.com/lf-lang/lingua-franca/pull/1009) ([edwardalee](https://github.com/edwardalee))
- Replace the 'threads' target property with the 'workers' and 'threading' properties [\#993](https://github.com/lf-lang/lingua-franca/pull/993) ([cmnrd](https://github.com/cmnrd))
- Docker files property [\#987](https://github.com/lf-lang/lingua-franca/pull/987) ([housengw](https://github.com/housengw))
- Enable generation of ROS2 nodes directly from C++ code [\#984](https://github.com/lf-lang/lingua-franca/pull/984) ([cmnrd](https://github.com/cmnrd))
- Added `--version` flag to `lfc`and improved error messages [\#979](https://github.com/lf-lang/lingua-franca/pull/979) ([cmnrd](https://github.com/cmnrd))
- Improve warning message for unrecognized target parameters [\#964](https://github.com/lf-lang/lingua-franca/pull/964) ([cmnrd](https://github.com/cmnrd))
- Add bank & multiport support in TypeScript code generator with \> 30 multiport tests. [\#942](https://github.com/lf-lang/lingua-franca/pull/942) ([hokeun](https://github.com/hokeun))
- Added support for modular scheduler in C runtime [\#743](https://github.com/lf-lang/lingua-franca/pull/743) ([Soroosh129](https://github.com/Soroosh129))
- Added support for Modal Models [\#501](https://github.com/lf-lang/lingua-franca/pull/501) ([a-sr](https://github.com/a-sr))

**Fixed bugs:**

- Tracing does not get enabled in all files of reactor-c [\#1067](https://github.com/lf-lang/lingua-franca/issues/1067)
- C benchmark PingPong is broken [\#1058](https://github.com/lf-lang/lingua-franca/issues/1058)
- Command line argument -w does not work. [\#1056](https://github.com/lf-lang/lingua-franca/issues/1056)
- Maven build is broken [\#1033](https://github.com/lf-lang/lingua-franca/issues/1033)
- FilterBank benchmark in C++ has cycles [\#1031](https://github.com/lf-lang/lingua-franca/issues/1031)
- EclipseErrorReporter reports errors in the wrong file [\#1030](https://github.com/lf-lang/lingua-franca/issues/1030)
- Dependency cycle detection does not seem to work  [\#1024](https://github.com/lf-lang/lingua-franca/issues/1024)
- Spurious warnings in Epoch [\#1016](https://github.com/lf-lang/lingua-franca/issues/1016)
- Error reporting in Epoch is broken [\#994](https://github.com/lf-lang/lingua-franca/issues/994)
- Epoch fails to highlight errors on codegen [\#966](https://github.com/lf-lang/lingua-franca/issues/966)
- Some cargo errors are not reported in LF [\#936](https://github.com/lf-lang/lingua-franca/issues/936)
- Files without main don't compile if they have imports [\#913](https://github.com/lf-lang/lingua-franca/issues/913)
- Invalid time literal causees IllegalArgumentException [\#908](https://github.com/lf-lang/lingua-franca/issues/908)
- Reactor extending itself causes StackOverflowError [\#907](https://github.com/lf-lang/lingua-franca/issues/907)
- Having multiple unnamed main reactors causes nullPointerException [\#905](https://github.com/lf-lang/lingua-franca/issues/905)
- Catch errors from the TS type checker in the `examples` category [\#405](https://github.com/lf-lang/lingua-franca/issues/405)
- Fixed an issue with bank\_index [\#1087](https://github.com/lf-lang/lingua-franca/pull/1087) ([Soroosh129](https://github.com/Soroosh129))
- Fix bug where tracing does not get fully enabled [\#1068](https://github.com/lf-lang/lingua-franca/pull/1068) ([hokeun](https://github.com/hokeun))
- Fix errors in tracing util. [\#1043](https://github.com/lf-lang/lingua-franca/pull/1043) ([hokeun](https://github.com/hokeun))
- Properly recognize network message actions in TypeScript generator [\#1042](https://github.com/lf-lang/lingua-franca/pull/1042) ([hokeun](https://github.com/hokeun))
- \[error reporting\] Correct bugs reported on Epoch [\#1038](https://github.com/lf-lang/lingua-franca/pull/1038) ([petervdonovan](https://github.com/petervdonovan))
- Fix handling of ganged connections [\#1037](https://github.com/lf-lang/lingua-franca/pull/1037) ([edwardalee](https://github.com/edwardalee))
- Broaden a pylint ignore. [\#1029](https://github.com/lf-lang/lingua-franca/pull/1029) ([petervdonovan](https://github.com/petervdonovan))
- Fixed causality loop detection bug [\#1026](https://github.com/lf-lang/lingua-franca/pull/1026) ([edwardalee](https://github.com/edwardalee))
- Suppress line-too-long warnings in the Python target. [\#1018](https://github.com/lf-lang/lingua-franca/pull/1018) ([petervdonovan](https://github.com/petervdonovan))
- \[C\] do not generate code without main reactor [\#1000](https://github.com/lf-lang/lingua-franca/pull/1000) ([housengw](https://github.com/housengw))
- Check for null reactor to avoid NPE [\#998](https://github.com/lf-lang/lingua-franca/pull/998) ([edwardalee](https://github.com/edwardalee))
- Fix eclipse error reporting [\#995](https://github.com/lf-lang/lingua-franca/pull/995) ([cmnrd](https://github.com/cmnrd))
- Epoch error reporting [\#967](https://github.com/lf-lang/lingua-franca/pull/967) ([edwardalee](https://github.com/edwardalee))
- Detect and flag and as error multiple mains [\#965](https://github.com/lf-lang/lingua-franca/pull/965) ([edwardalee](https://github.com/edwardalee))
- Inheritance cleanups [\#962](https://github.com/lf-lang/lingua-franca/pull/962) ([edwardalee](https://github.com/edwardalee))
- Fixes \#768 [\#952](https://github.com/lf-lang/lingua-franca/pull/952) ([housengw](https://github.com/housengw))
- Report raw, unparsed error streams as a last resort. [\#941](https://github.com/lf-lang/lingua-franca/pull/941) ([petervdonovan](https://github.com/petervdonovan))

**Closed issues:**

- Epoch fails on valid file [\#1027](https://github.com/lf-lang/lingua-franca/issues/1027)
- List modal reactor tests in own test category [\#1020](https://github.com/lf-lang/lingua-franca/issues/1020)
- Merge `JavaAstUtils` and `ASTUtils` [\#1003](https://github.com/lf-lang/lingua-franca/issues/1003)
- Kotlin classes do not get build by `buildLfc` task \(unless the `clean` task is also run\) [\#930](https://github.com/lf-lang/lingua-franca/issues/930)
- Navigate to imported reactors in diagrams [\#889](https://github.com/lf-lang/lingua-franca/issues/889)
- Port Xtend classes to Java [\#838](https://github.com/lf-lang/lingua-franca/issues/838)
- Use the same implementation of deque accross all C benchmarks [\#765](https://github.com/lf-lang/lingua-franca/issues/765)
- Use the BenchmarkRunner reactor in all C benchmarks [\#764](https://github.com/lf-lang/lingua-franca/issues/764)
- Build both Epoch with maven and lfc with gradle in our CI workflow [\#575](https://github.com/lf-lang/lingua-franca/issues/575)
- Declutter repo and move into separate Github Organization [\#347](https://github.com/lf-lang/lingua-franca/issues/347)
- C syntax for referencing parameters and state variables [\#82](https://github.com/lf-lang/lingua-franca/issues/82)

**Merged pull requests:**

- Switched to building the Python extension module in-place [\#1088](https://github.com/lf-lang/lingua-franca/pull/1088) ([Soroosh129](https://github.com/Soroosh129))
- Remove example directory [\#1077](https://github.com/lf-lang/lingua-franca/pull/1077) ([lhstrh](https://github.com/lhstrh))
- Remove experimental directory [\#1076](https://github.com/lf-lang/lingua-franca/pull/1076) ([lhstrh](https://github.com/lhstrh))
- Accommodations for automated version changes [\#1071](https://github.com/lf-lang/lingua-franca/pull/1071) ([lhstrh](https://github.com/lhstrh))
- Enable tycho-versions-plugin to manage versioning [\#1070](https://github.com/lf-lang/lingua-franca/pull/1070) ([lhstrh](https://github.com/lhstrh))
- Commented out annotation of outputs on transitions [\#1065](https://github.com/lf-lang/lingua-franca/pull/1065) ([edwardalee](https://github.com/edwardalee))
- Fixed Problem with Mode Transitions and Edges in VS Code [\#1063](https://github.com/lf-lang/lingua-franca/pull/1063) ([a-sr](https://github.com/a-sr))
- Fix network sender reaction multiport [\#1061](https://github.com/lf-lang/lingua-franca/pull/1061) ([Soroosh129](https://github.com/Soroosh129))
- Fix missing `-w` command line argument [\#1060](https://github.com/lf-lang/lingua-franca/pull/1060) ([housengw](https://github.com/housengw))
- Improved Support for Imported Reactors in Diagrams [\#1055](https://github.com/lf-lang/lingua-franca/pull/1055) ([a-sr](https://github.com/a-sr))
- The files target property: Add support for directories [\#1053](https://github.com/lf-lang/lingua-franca/pull/1053) ([Soroosh129](https://github.com/Soroosh129))
- Removed all Xtend dependencies [\#1051](https://github.com/lf-lang/lingua-franca/pull/1051) ([housengw](https://github.com/housengw))
- Removed broken dependency on `org.eclipse.xpand.feature.group` [\#1050](https://github.com/lf-lang/lingua-franca/pull/1050) ([lhstrh](https://github.com/lhstrh))
- Port `CGenerator.xtend` to Java [\#1049](https://github.com/lf-lang/lingua-franca/pull/1049) ([housengw](https://github.com/housengw))
- Replace all tabs with spaces in the `org.lflang.diagrams` package [\#1047](https://github.com/lf-lang/lingua-franca/pull/1047) ([a-sr](https://github.com/a-sr))
- Partial refactoring of  `CGenerator.xtend` [\#1045](https://github.com/lf-lang/lingua-franca/pull/1045) ([housengw](https://github.com/housengw))
- Another refactoring of `CGenerator.xtend` [\#1035](https://github.com/lf-lang/lingua-franca/pull/1035) ([housengw](https://github.com/housengw))
- ASTUtils cleanup [\#1022](https://github.com/lf-lang/lingua-franca/pull/1022) ([cmnrd](https://github.com/cmnrd))
- Bump Xtext from 2.25.0 to 2.26.0 [\#1021](https://github.com/lf-lang/lingua-franca/pull/1021) ([lhstrh](https://github.com/lhstrh))
- Removed unnecessary `xtend-gen` folder from classpath [\#1014](https://github.com/lf-lang/lingua-franca/pull/1014) ([soerendomroes](https://github.com/soerendomroes))
- Added ROS2 serialization tests [\#1012](https://github.com/lf-lang/lingua-franca/pull/1012) ([Soroosh129](https://github.com/Soroosh129))
- Refactor `CGenerator.xtend` [\#1011](https://github.com/lf-lang/lingua-franca/pull/1011) ([housengw](https://github.com/housengw))
- Grammar cleanup [\#1008](https://github.com/lf-lang/lingua-franca/pull/1008) ([cmnrd](https://github.com/cmnrd))
- Merge `JavaAstUtils` into `ASTUtils` [\#1004](https://github.com/lf-lang/lingua-franca/pull/1004) ([housengw](https://github.com/housengw))
- Move benchmarks to separate repository [\#1001](https://github.com/lf-lang/lingua-franca/pull/1001) ([cmnrd](https://github.com/cmnrd))
- Add FPS and update README of YOLO demo [\#989](https://github.com/lf-lang/lingua-franca/pull/989) ([housengw](https://github.com/housengw))
- Carla intersection refactoring [\#982](https://github.com/lf-lang/lingua-franca/pull/982) ([housengw](https://github.com/housengw))
- FileConfig cleanups [\#978](https://github.com/lf-lang/lingua-franca/pull/978) ([cmnrd](https://github.com/cmnrd))
- Add invoke\_deadline\_handler parameter to AnytimePrime.lf. [\#976](https://github.com/lf-lang/lingua-franca/pull/976) ([hokeun](https://github.com/hokeun))
- Port `GeneratorBase.xtend` to Java [\#974](https://github.com/lf-lang/lingua-franca/pull/974) ([housengw](https://github.com/housengw))
- Add reactor-cpp as a submodule and reorganize the C++ build process [\#971](https://github.com/lf-lang/lingua-franca/pull/971) ([cmnrd](https://github.com/cmnrd))
- Simplify conditional fed codegen [\#970](https://github.com/lf-lang/lingua-franca/pull/970) ([edwardalee](https://github.com/edwardalee))
- Fix port type mismatch in AnytimePrime.lf [\#968](https://github.com/lf-lang/lingua-franca/pull/968) ([hokeun](https://github.com/hokeun))
- FileConfig cleanup [\#963](https://github.com/lf-lang/lingua-franca/pull/963) ([cmnrd](https://github.com/cmnrd))
- Validator cleanups [\#961](https://github.com/lf-lang/lingua-franca/pull/961) ([edwardalee](https://github.com/edwardalee))
- Add C code generation to store currently executed reaction, add AnytimePrime.lf as a demo for check\_deadline\(\). [\#960](https://github.com/lf-lang/lingua-franca/pull/960) ([hokeun](https://github.com/hokeun))
- Ported `PythonGenerator.xtend` to Java and started refactoring of `GeneratorBase` [\#958](https://github.com/lf-lang/lingua-franca/pull/958) ([housengw](https://github.com/housengw))
- Clean up the file writing mechanism [\#956](https://github.com/lf-lang/lingua-franca/pull/956) ([cmnrd](https://github.com/cmnrd))
-  Port  `org.lflang.federated` to java [\#950](https://github.com/lf-lang/lingua-franca/pull/950) ([housengw](https://github.com/housengw))
- Fixed bug in centralized coordination related to time advancement [\#949](https://github.com/lf-lang/lingua-franca/pull/949) ([housengw](https://github.com/housengw))
- Update reactor-cpp version [\#946](https://github.com/lf-lang/lingua-franca/pull/946) ([cmnrd](https://github.com/cmnrd))
- Ported Xtend code in diagram package to Java [\#940](https://github.com/lf-lang/lingua-franca/pull/940) ([housengw](https://github.com/housengw))
- Refactoring: Purge code generators of IFileSystemAccess2. [\#938](https://github.com/lf-lang/lingua-franca/pull/938) ([petervdonovan](https://github.com/petervdonovan))
- Fix gradle setup by setting kotlin version to 1.4.30 [\#934](https://github.com/lf-lang/lingua-franca/pull/934) ([cmnrd](https://github.com/cmnrd))
- Fixes \#932 [\#933](https://github.com/lf-lang/lingua-franca/pull/933) ([Soroosh129](https://github.com/Soroosh129))
- Carla intersection updates [\#901](https://github.com/lf-lang/lingua-franca/pull/901) ([Soroosh129](https://github.com/Soroosh129))
- Ported `LFValidator.xtend` to Java [\#886](https://github.com/lf-lang/lingua-franca/pull/886) ([housengw](https://github.com/housengw))
- Added AlarmClock example [\#780](https://github.com/lf-lang/lingua-franca/pull/780) ([revol-xut](https://github.com/revol-xut))

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
