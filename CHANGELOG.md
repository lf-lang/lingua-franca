# Changelog
 
## [v0.4.0](https://github.com/lf-lang/lingua-franca/tree/v0.4.0) (2023-03-01)

**Highlights**

This release includes substantial changes under the hood and brings a lot of new features and enhancements, ranging from performance improvements to the support of new platforms like Zephyr, Arduino, and MBED in the C target. In the C++ target, it is now also straightforward to interact with ROS2 packages using the `ros2-dependencies` target property.

**🚀 New Features**

- Multiport iterator for more efficient access to sparse inputs [\#1298](https://github.com/lf-lang/lingua-franca/pull/1298) ([edwardalee](https://github.com/edwardalee))
- Inference of logical execution times of reactions [\#1292](https://github.com/lf-lang/lingua-franca/pull/1292) ([lhstrh](https://github.com/lhstrh))
- Custom icons for reactors in diagrams [\#1300](https://github.com/lf-lang/lingua-franca/pull/1300) ([a-sr](https://github.com/a-sr))
- Support for Arduino platforms [\#1272](https://github.com/lf-lang/lingua-franca/pull/1272) ([arengarajan99](https://github.com/arengarajan99))
- Automatic LF code formatter [\#1227](https://github.com/lf-lang/lingua-franca/pull/1227) ([petervdonovan](https://github.com/petervdonovan))
- Option of using OpenSSL for authentication between RTI and federates [\#1432](https://github.com/lf-lang/lingua-franca/pull/1432) ([Jakio815](https://github.com/Jakio815))
- Safe handling of physical actions in single-threaded C runtime [\#1348](https://github.com/lf-lang/lingua-franca/pull/1348) ([lhstrh](https://github.com/lhstrh))
- Initial support for childref multiports [\#1228](https://github.com/lf-lang/lingua-franca/pull/1228) ([jhaye](https://github.com/jhaye))
- Epoch built for M1 Mac [\#1530](https://github.com/lf-lang/lingua-franca/pull/1530) ([hokeun](https://github.com/hokeun))
- Proper handling of asynchronous interactions in Rust [\#1459](https://github.com/lf-lang/lingua-franca/pull/1459) ([oowekyala](https://github.com/oowekyala))
- Support for basic scheduling enclaves in the C++ target [\#1513](https://github.com/lf-lang/lingua-franca/pull/1513) ([cmnrd](https://github.com/cmnrd))
- Zephyr support for the C target [\#1536](https://github.com/lf-lang/lingua-franca/pull/1536) ([erlingrj](https://github.com/erlingrj))
- Support for Arduino CLI [\#1532](https://github.com/lf-lang/lingua-franca/pull/1532) ([arengarajan99](https://github.com/arengarajan99))
- Arduino MBED RTOS Support  [\#1581](https://github.com/lf-lang/lingua-franca/pull/1581) ([arengarajan99](https://github.com/arengarajan99))

**✨ Enhancements**

- Added advance-message-interval option and more federated tests for TypeScript [\#1293](https://github.com/lf-lang/lingua-franca/pull/1293) ([hokeun](https://github.com/hokeun))
- Improved layout for error message node [\#1324](https://github.com/lf-lang/lingua-franca/pull/1324) ([a-sr](https://github.com/a-sr))
- C++ target performance optimizations [\#1330](https://github.com/lf-lang/lingua-franca/pull/1330) ([cmnrd](https://github.com/cmnrd))
- Improved error messaging for mode validation [\#1349](https://github.com/lf-lang/lingua-franca/pull/1349) ([a-sr](https://github.com/a-sr))
- New target property for specifying additional ROS dependencies [\#1355](https://github.com/lf-lang/lingua-franca/pull/1355) ([cmnrd](https://github.com/cmnrd))
- Support for passthrough connections in the C++ target [\#1361](https://github.com/lf-lang/lingua-franca/pull/1361) ([cmnrd](https://github.com/cmnrd))
- Support scheduling physical actions synchronously [\#1367](https://github.com/lf-lang/lingua-franca/pull/1367) ([oowekyala](https://github.com/oowekyala))
- Automatic code formatting using Spotless [\#1374](https://github.com/lf-lang/lingua-franca/pull/1374) ([petervdonovan](https://github.com/petervdonovan))
- Optimized access to sparse multiports in the C++ target [\#1312](https://github.com/lf-lang/lingua-franca/pull/1312) ([revol-xut](https://github.com/revol-xut))
- Augmented support for Arduino [\#1384](https://github.com/lf-lang/lingua-franca/pull/1384) ([arengarajan99](https://github.com/arengarajan99))
- New lff autoformatter for Lingua Franca files [\#1422](https://github.com/lf-lang/lingua-franca/pull/1422) ([cmnrd](https://github.com/cmnrd))
- Protection from code modifying the multiport container in C++ [\#1420](https://github.com/lf-lang/lingua-franca/pull/1420) ([revol-xut](https://github.com/revol-xut))
- Empty bracket pairs properly tokenized [\#1439](https://github.com/lf-lang/lingua-franca/pull/1439) ([oowekyala](https://github.com/oowekyala))
- Switched to Gradle for language and diagram server build [\#1469](https://github.com/lf-lang/lingua-franca/pull/1469) ([a-sr](https://github.com/a-sr))
- Better multiport support for Rust [\#1406](https://github.com/lf-lang/lingua-franca/pull/1406) ([oowekyala](https://github.com/oowekyala))
- Reduction of disk usage in the Rust target [\#1476](https://github.com/lf-lang/lingua-franca/pull/1476) ([oowekyala](https://github.com/oowekyala))
- Native implementation of timeout in C++ [\#1507](https://github.com/lf-lang/lingua-franca/pull/1507) ([cmnrd](https://github.com/cmnrd))
- Less verbose CMake output via suppressed install messages [\#1517](https://github.com/lf-lang/lingua-franca/pull/1517) ([cmnrd](https://github.com/cmnrd))
- Preservation of time units when formatting LF code [\#1518](https://github.com/lf-lang/lingua-franca/pull/1518) ([cmnrd](https://github.com/cmnrd))
- Check for update Rust runtime [\#1546](https://github.com/lf-lang/lingua-franca/pull/1546) ([oowekyala](https://github.com/oowekyala))
- Bugfixes and improvements in token-based memory management in C [\#1548](https://github.com/lf-lang/lingua-franca/pull/1548) ([edwardalee](https://github.com/edwardalee))
- Use of delayed and physical connections as provided by the C++ runtime [\#1583](https://github.com/lf-lang/lingua-franca/pull/1583) ([cmnrd](https://github.com/cmnrd))

**🔧 Fixes**

- Removal of error reporting markers from parameter types [\#1310](https://github.com/lf-lang/lingua-franca/pull/1310) ([Wonseo-C](https://github.com/Wonseo-C))
- Quick fix to adjust upstreamFedDelays as never tag [\#1334](https://github.com/lf-lang/lingua-franca/pull/1334) ([byeong-gil](https://github.com/byeong-gil))
- Fixed tracing in the C++ target [\#1350](https://github.com/lf-lang/lingua-franca/pull/1350) ([cmnrd](https://github.com/cmnrd))
- Reactor runtime module excluded from the Rust test modules [\#1351](https://github.com/lf-lang/lingua-franca/pull/1351) ([oowekyala](https://github.com/oowekyala))
- Problem with child dependencies in Rust fixed [\#1352](https://github.com/lf-lang/lingua-franca/pull/1352) ([oowekyala](https://github.com/oowekyala))
- Adjustment of code generator to address #1368 [\#1370](https://github.com/lf-lang/lingua-franca/pull/1370) ([lsk567](https://github.com/lsk567))
- Removal of PyDev and Kotlin plugin dependencies from Epoch [\#1371](https://github.com/lf-lang/lingua-franca/pull/1371) ([lhstrh](https://github.com/lhstrh))
- Repair of Windows diagnostic reporting [\#1391](https://github.com/lf-lang/lingua-franca/pull/1391) ([petervdonovan](https://github.com/petervdonovan))
- Fix of flaky timedwait on Windows [\#1402](https://github.com/lf-lang/lingua-franca/pull/1402) ([lhstrh](https://github.com/lhstrh))
- Use of paths not URIs in line-directives [\#1412](https://github.com/lf-lang/lingua-franca/pull/1412) ([erlingrj](https://github.com/erlingrj))
- NPE in Epoch fixed [\#1425](https://github.com/lf-lang/lingua-franca/pull/1425) ([petervdonovan](https://github.com/petervdonovan))
- JavaDoc and NPE on toString fixed [\#1445](https://github.com/lf-lang/lingua-franca/pull/1445) ([edwardalee](https://github.com/edwardalee))
- Avoid reading the same input stream twice [\#1490](https://github.com/lf-lang/lingua-franca/pull/1490) ([cmnrd](https://github.com/cmnrd))
- Tracing tools Makefile fixed [\#1485](https://github.com/lf-lang/lingua-franca/pull/1485) ([lsk567](https://github.com/lsk567))
- MalleableStrings made human readable [\#1500](https://github.com/lf-lang/lingua-franca/pull/1500) ([petervdonovan](https://github.com/petervdonovan))
- Fix for broken deadline propagation mechanism [\#1451](https://github.com/lf-lang/lingua-franca/pull/1451) ([erlingrj](https://github.com/erlingrj))
- Fix of CMake policy CMP0068 warning [\#1523](https://github.com/lf-lang/lingua-franca/pull/1523) ([cmnrd](https://github.com/cmnrd))
- rs/AsyncCallback: Join thread before creating new one [\#1542](https://github.com/lf-lang/lingua-franca/pull/1542) ([oowekyala](https://github.com/oowekyala))
- Bugfix for port access to contained generic reactors in C++ [\#1547](https://github.com/lf-lang/lingua-franca/pull/1547) ([cmnrd](https://github.com/cmnrd))
- Fixed validation of icon paths [\#1572](https://github.com/lf-lang/lingua-franca/pull/1572) ([cmnrd](https://github.com/cmnrd))
- Useful error message upon encountering missing runtime sources [\#1573](https://github.com/lf-lang/lingua-franca/pull/1573) ([cmnrd](https://github.com/cmnrd))
- [ts] Bugfix that re-enables the use of custom configuration files [\#1575](https://github.com/lf-lang/lingua-franca/pull/1575) ([lhstrh](https://github.com/lhstrh))
- Fix to let relative include helper ignore non-C files [\#1578](https://github.com/lf-lang/lingua-franca/pull/1578) ([lhstrh](https://github.com/lhstrh))
- Fix failing building of tracing tools [\#1589](https://github.com/lf-lang/lingua-franca/pull/1589) ([erlingrj](https://github.com/erlingrj))
- Fixes in relative include helper for arduino-cli [\#1586](https://github.com/lf-lang/lingua-franca/pull/1586) ([arengarajan99](https://github.com/arengarajan99))
- CLI args propagated to federates [\#1604](https://github.com/lf-lang/lingua-franca/pull/1604) ([lhstrh](https://github.com/lhstrh))
- [ts] Missing types pulled from npm [\#1609](https://github.com/lf-lang/lingua-franca/pull/1609) ([lhstrh](https://github.com/lhstrh))

**🚧 Maintenance and Refactoring**

- Use HTTPS for Rust reactor runtime submodule [\#1308](https://github.com/lf-lang/lingua-franca/pull/1308) ([jhaye](https://github.com/jhaye))
- Switch from Rust nightly to Rust stable [\#1218](https://github.com/lf-lang/lingua-franca/pull/1218) ([jhaye](https://github.com/jhaye))
- Tests in TypeScript for requesting stop in federated execution [\#1302](https://github.com/lf-lang/lingua-franca/pull/1302) ([hokeun](https://github.com/hokeun))
- More portable way to suppress unused variable warnings [\#1317](https://github.com/lf-lang/lingua-franca/pull/1317) ([edwardalee](https://github.com/edwardalee))
- Reduced verbosity in reported output [\#1323](https://github.com/lf-lang/lingua-franca/pull/1323) ([petervdonovan](https://github.com/petervdonovan))
- Reference reactor-ts as module [\#1322](https://github.com/lf-lang/lingua-franca/pull/1322) ([lhstrh](https://github.com/lhstrh))
- Removal of unnecessary build dependency to address security exceptions [\#1365](https://github.com/lf-lang/lingua-franca/pull/1365) ([cmnrd](https://github.com/cmnrd))
- Simplification of grammar that encodes mutations with the same productions as reactions [\#1318](https://github.com/lf-lang/lingua-franca/pull/1318) ([Wonseo-C](https://github.com/Wonseo-C))
- Removal of "No CMake" build option [\#1299](https://github.com/lf-lang/lingua-franca/pull/1299) ([petervdonovan](https://github.com/petervdonovan))
- Usage of a published version of reactor-ts [\#1426](https://github.com/lf-lang/lingua-franca/pull/1426) ([petervdonovan](https://github.com/petervdonovan))
- Differentiation between multi-threaded and single-threaded preprocessor directives in C [\#1411](https://github.com/lf-lang/lingua-franca/pull/1411) ([erlingrj](https://github.com/erlingrj))
- TypeScript generator cleanups [\#1457](https://github.com/lf-lang/lingua-franca/pull/1457) ([oowekyala](https://github.com/oowekyala))
- Clean up AttributeUtils [\#1470](https://github.com/lf-lang/lingua-franca/pull/1470) ([cmnrd](https://github.com/cmnrd))
- Infrastructure for AST transformations and factored out delay transformation [\#1508](https://github.com/lf-lang/lingua-franca/pull/1508) ([cmnrd](https://github.com/cmnrd))
- Cleanup initializer grammar [\#1441](https://github.com/lf-lang/lingua-franca/pull/1441) ([oowekyala](https://github.com/oowekyala))
- Serialization of compile definitions [\#1529](https://github.com/lf-lang/lingua-franca/pull/1529) ([erlingrj](https://github.com/erlingrj))
- Fix and reformat javadoc @author tag [\#1486](https://github.com/lf-lang/lingua-franca/pull/1486) ([axmmisaka](https://github.com/axmmisaka))
- Restructuring of mechanism for compiling federated programs [\#1221](https://github.com/lf-lang/lingua-franca/pull/1221) ([lhstrh](https://github.com/lhstrh))
- Removal of .c file includes in federated programs [\#1601](https://github.com/lf-lang/lingua-franca/pull/1601) ([arengarajan99](https://github.com/arengarajan99))

**🧪 Tests**

- Only record tests in test/*/src [\#1358](https://github.com/lf-lang/lingua-franca/pull/1358) ([oowekyala](https://github.com/oowekyala))
- Test for Rust main parameters [\#1366](https://github.com/lf-lang/lingua-franca/pull/1366) ([oowekyala](https://github.com/oowekyala))
- Add CLI unit tests [\#1467](https://github.com/lf-lang/lingua-franca/pull/1467) ([oowekyala](https://github.com/oowekyala))
- Correct reporting of assertion errors and exceptions during code generation [\#1498](https://github.com/lf-lang/lingua-franca/pull/1498) ([cmnrd](https://github.com/cmnrd))
- Removal of uses of deprecated reactor-c API in tests [\#1566](https://github.com/lf-lang/lingua-franca/pull/1566) ([erlingrj](https://github.com/erlingrj))

**⬆️ Updated Dependencies**

- Bump Rust runtime [\#1357](https://github.com/lf-lang/lingua-franca/pull/1357) ([jhaye](https://github.com/jhaye))
- Xtext to 2.18.0, LSP4J to 0.15.0, and Guice to 5.1.0 [\#1364](https://github.com/lf-lang/lingua-franca/pull/1364) ([lhstrh](https://github.com/lhstrh))
- Gradle bumped to version 7.5.1 [\#1408](https://github.com/lf-lang/lingua-franca/pull/1408) ([lhstrh](https://github.com/lhstrh))
- Upgrade to Gradle v7.6.1 [\#1608](https://github.com/lf-lang/lingua-franca/pull/1608) ([lhstrh](https://github.com/lhstrh))


### Submodule [lf-lang/reactor-c](http://github.com/lf-lang/reactor-c)

**🚀 New Features**

- Adaptive scheduler [\#85](https://github.com/lf-lang/reactor-c/pull/85) ([petervdonovan](https://github.com/petervdonovan))
- Support for Arduino platforms [\#89](https://github.com/lf-lang/reactor-c/pull/89) ([arengarajan99](https://github.com/arengarajan99))
- Basic authentication using HMAC for federates joining a federation [\#105](https://github.com/lf-lang/reactor-c/pull/105) ([hokeun](https://github.com/hokeun))
- Platform support for Embedded targets [\#106](https://github.com/lf-lang/reactor-c/pull/106) ([lhstrh](https://github.com/lhstrh))
- Changes to facilitate arduino-cli support [\#138](https://github.com/lf-lang/reactor-c/pull/138) ([arengarajan99](https://github.com/arengarajan99))
- Zephyr support [\#134](https://github.com/lf-lang/reactor-c/pull/134) ([erlingrj](https://github.com/erlingrj))
- Critical section functions as part of the public API [\#142](https://github.com/lf-lang/reactor-c/pull/142) ([edwardalee](https://github.com/edwardalee))
- Preliminary multi-board support for Arduino [\#141](https://github.com/lf-lang/reactor-c/pull/141) ([arengarajan99](https://github.com/arengarajan99))
- Add hi-res timing support for imxrt1170evk [\#149](https://github.com/lf-lang/reactor-c/pull/149) ([erlingrj](https://github.com/erlingrj))
- Threading support for Arduino MBED/coupling of mutexes and condition variables [\#153](https://github.com/lf-lang/reactor-c/pull/153) ([arengarajan99](https://github.com/arengarajan99))

**✨ Enhancements**

- C support for multiport iterator for sparse inputs [\#95](https://github.com/lf-lang/reactor-c/pull/95) ([edwardalee](https://github.com/edwardalee))
- Fewer warnings [\#101](https://github.com/lf-lang/reactor-c/pull/101) ([edwardalee](https://github.com/edwardalee))
- Initial style guide [\#103](https://github.com/lf-lang/reactor-c/pull/103) ([petervdonovan](https://github.com/petervdonovan))
- Memory management improvements and bugfixes [\#144](https://github.com/lf-lang/reactor-c/pull/144) ([edwardalee](https://github.com/edwardalee))
- Improved CMake configuration [\#156](https://github.com/lf-lang/reactor-c/pull/156) ([arengarajan99](https://github.com/arengarajan99))
- Small fixes to get Zephyr target compiling with threaded runtime [\#155](https://github.com/lf-lang/reactor-c/pull/155) ([erlingrj](https://github.com/erlingrj))

**🔧 Fixes**

- Fixed memory leaks in RTI [\#113](https://github.com/lf-lang/reactor-c/pull/113) ([erlingrj](https://github.com/erlingrj))
- Fix of flaky timedwait on Windows [\#115](https://github.com/lf-lang/reactor-c/pull/115) ([cmnrd](https://github.com/cmnrd))
- Token payload freed with after delay [\#124](https://github.com/lf-lang/reactor-c/pull/124) ([edwardalee](https://github.com/edwardalee))
- Consistent way of handling entry and exit of critical section [\#136](https://github.com/lf-lang/reactor-c/pull/136) ([erlingrj](https://github.com/erlingrj))
- Scheduling preprocessing fix [\#154](https://github.com/lf-lang/reactor-c/pull/154) ([erlingrj](https://github.com/erlingrj))
- Fix problem with threaded Zephyr and overflow handling in QEMU emulation [\#159](https://github.com/lf-lang/reactor-c/pull/159) ([erlingrj](https://github.com/erlingrj))
- Intended tag printed instead of last-seen tag in debug message [\#160](https://github.com/lf-lang/reactor-c/pull/160) ([byeong-gil](https://github.com/byeong-gil))
- Zephyr timing fix [\#166](https://github.com/lf-lang/reactor-c/pull/166) ([erlingrj](https://github.com/erlingrj))
- Fixed compilation of RTI without `-DAUTH` and CI job for building the RTI [\#171](https://github.com/lf-lang/reactor-c/pull/171) ([cmnrd](https://github.com/cmnrd))

**🚧 Maintenance and Refactoring**

- Removal of "No CMake" build option [\#97](https://github.com/lf-lang/reactor-c/pull/97) ([petervdonovan](https://github.com/petervdonovan))
- Improved CMake configuration [\#137](https://github.com/lf-lang/reactor-c/pull/137) ([erlingrj](https://github.com/erlingrj))
- Changes to accommodate restructuring code generation of federated programs [\#117](https://github.com/lf-lang/reactor-c/pull/117) ([lhstrh](https://github.com/lhstrh))
- Relative paths in lf_types.h [\#147](https://github.com/lf-lang/reactor-c/pull/147) ([gundralaa](https://github.com/gundralaa))

- Federated support without C include reliance [\#164](https://github.com/lf-lang/reactor-c/pull/164) ([arengarajan99](https://github.com/arengarajan99))

### Submodule [lf-lang/reactor-cpp](http://github.com/lf-lang/reactor-cpp)

**🚀 New Features**

- Support passthrough connections [\#29](https://github.com/lf-lang/reactor-cpp/pull/29) ([cmnrd](https://github.com/cmnrd))
- Enable multiple environments and schedulers to run at the same time [\#37](https://github.com/lf-lang/reactor-cpp/pull/37) ([cmnrd](https://github.com/cmnrd))
- Native runtime implementation of delayed and physical connections [\#38](https://github.com/lf-lang/reactor-cpp/pull/38) ([revol-xut](https://github.com/revol-xut))

**✨ Enhancements**

- Optimization of value pointers for trivially copyable types [\#22](https://github.com/lf-lang/reactor-cpp/pull/22) ([cmnrd](https://github.com/cmnrd))
- Avoid locking when scheduling actions [\#23](https://github.com/lf-lang/reactor-cpp/pull/23) ([cmnrd](https://github.com/cmnrd))
- Optimized Multiports [\#24](https://github.com/lf-lang/reactor-cpp/pull/24) ([revol-xut](https://github.com/revol-xut))
- Protect multiports from accidental modification by restricting Multiport and introducing new class ModifyableMultiport [\#33](https://github.com/lf-lang/reactor-cpp/pull/33) ([revol-xut](https://github.com/revol-xut))
- Native timeout implementation, sync_shutdown made threadsafe, and is_present added for timers [\#36](https://github.com/lf-lang/reactor-cpp/pull/36) ([cmnrd](https://github.com/cmnrd))
- Optimization: use std::string_view in validation functions [\#41](https://github.com/lf-lang/reactor-cpp/pull/41) ([cmnrd](https://github.com/cmnrd))
- Native runtime implementation of delayed and physical connections [\#38](https://github.com/lf-lang/reactor-cpp/pull/38) ([revol-xut](https://github.com/revol-xut))

**🔧 Fixes**

- Fixed compilation with tracing support [\#25](https://github.com/lf-lang/reactor-cpp/pull/25) ([cmnrd](https://github.com/cmnrd))
- Fix race condtions related to the scheduling of physical actions [\#31](https://github.com/lf-lang/reactor-cpp/pull/31) ([cmnrd](https://github.com/cmnrd))
- Fix compilation on 32-Bit machines [\#40](https://github.com/lf-lang/reactor-cpp/pull/40) ([cmnrd](https://github.com/cmnrd))


### Submodule [lf-lang/reactor-rs](http://github.com/lf-lang/reactor-rs)

**🚀 New Features**

- Support for synchronous scheduling of physical actions [\#30](https://github.com/lf-lang/reactor-rs/pull/30) ([oowekyala](https://github.com/oowekyala))

**✨ Enhancements**

- Replace level assignment algo [\#31](https://github.com/lf-lang/reactor-rs/pull/31) ([oowekyala](https://github.com/oowekyala))
- Switched from Rust Nightly to Rust Stable [\#32](https://github.com/lf-lang/reactor-rs/pull/32) ([jhaye](https://github.com/jhaye))
- Use of stable toolchain for lints in CI [\#35](https://github.com/lf-lang/reactor-rs/pull/35) ([oowekyala](https://github.com/oowekyala))
- Simplified access to ports and port banks [\#29](https://github.com/lf-lang/reactor-rs/pull/29) ([oowekyala](https://github.com/oowekyala))
- Use of external verified VecMap crate [\#39](https://github.com/lf-lang/reactor-rs/pull/39) ([jhaye](https://github.com/jhaye))
- Removal of scoped threads for physical actions [\#34](https://github.com/lf-lang/reactor-rs/pull/34) ([oowekyala](https://github.com/oowekyala))

**🔧 Fixes**

- Clippy fixes [\#27](https://github.com/lf-lang/reactor-rs/pull/27) ([jhaye](https://github.com/jhaye))
- Fix problem with merging of reaction plans [\#28](https://github.com/lf-lang/reactor-rs/pull/28) ([oowekyala](https://github.com/oowekyala))


 
## [v0.3.0](https://github.com/lf-lang/lingua-franca/tree/v0.3.0) (2022-07-22)

**Highlights**

This release adds broader support for methods (C and Python), syntax for annotations similar those in [Java](https://en.wikipedia.org/wiki/Java_annotation), an adaptive scheduler in the C runtime, and a new `platform` target property to furnish support for specialized execution platforms.

[Full Changelog](https://github.com/lingua-franca/lingua-franca/compare/v0.2.1...v0.3.0)

**🚀 New Features**

- Support for C methods [\#1215](https://github.com/lf-lang/lingua-franca/pull/1215) ([edwardalee](https://github.com/edwardalee))
- Support for Python methods [\#1230](https://github.com/lf-lang/lingua-franca/pull/1230) ([Soroosh129](https://github.com/Soroosh129))
- Adaptive scheduler for C target [\#1207](https://github.com/lf-lang/lingua-franca/pull/1207) ([petervdonovan](https://github.com/petervdonovan))
- Platform target property [\#1246](https://github.com/lf-lang/lingua-franca/pull/1246) ([Soroosh129](https://github.com/Soroosh129))
- Annotations [\#977](https://github.com/lf-lang/lingua-franca/pull/977) ([lsk567](https://github.com/lsk567))

**✨ Enhancements**

- New handling of startup/shutdown/reset reactions in modes [\#1169](https://github.com/lf-lang/lingua-franca/pull/1169) ([a-sr](https://github.com/a-sr))
- Diagram layout improvements [\#1206](https://github.com/lf-lang/lingua-franca/pull/1206) ([a-sr](https://github.com/a-sr))
- History transitions are now indicated using the `history` keyword [\#1247](https://github.com/lf-lang/lingua-franca/pull/1247) ([a-sr](https://github.com/a-sr))
- Compile warnings are fixed and clang-tidy is ran when compiling C++ tests in CI [\#1259](https://github.com/lf-lang/lingua-franca/pull/1259) ([cmnrd](https://github.com/cmnrd))
- New design for reset symbols in diagrams [\#1241](https://github.com/lf-lang/lingua-franca/pull/1241) ([a-sr](https://github.com/a-sr))
- Improvements of mode diagram layout [\#1282](https://github.com/lf-lang/lingua-franca/pull/1282) ([a-sr](https://github.com/a-sr))
- [cpp] Additional checks to determine whether a subdirectory should be included in compilation [\#1283](https://github.com/lf-lang/lingua-franca/pull/1283) ([cmnrd](https://github.com/cmnrd))
- [cpp] CLI parser errors are caught and a print help message is printed [\#1288](https://github.com/lf-lang/lingua-franca/pull/1288) ([cmnrd](https://github.com/cmnrd))
- [ts] Handling of physical action-triggered outputs in federated execution (previously handled by TAN - Time Advance Notice) [\#1275](https://github.com/lf-lang/lingua-franca/pull/1275) ([hokeun](https://github.com/hokeun))

**🔧 Fixes**

- TAN messages no longer used and in-transit messages recorded in the RTI [\#1074](https://github.com/lf-lang/lingua-franca/pull/1074) ([Soroosh129](https://github.com/Soroosh129))
- [c] A deadline of `0` is no longer interpreted as an _absent_ deadline but one that can never be met [\#1217](https://github.com/lf-lang/lingua-franca/pull/1217) ([billy-bao](https://github.com/billy-bao))
- Fix for deadlock in federated execution [\#1189](https://github.com/lf-lang/lingua-franca/pull/1189) ([Soroosh129](https://github.com/Soroosh129))
- [c] Removal of STP violation inheritance mechanism [\#1251](https://github.com/lf-lang/lingua-franca/pull/1251) ([Soroosh129](https://github.com/Soroosh129))
- Fix for properly handling paths with spaces in lfc launch script [\#1257](https://github.com/lf-lang/lingua-franca/pull/1257) ([cmnrd](https://github.com/cmnrd))
- Fix that repairs the ability to execute commands with Bash in case command is not found on path [\#1265](https://github.com/lf-lang/lingua-franca/pull/1265) ([petervdonovan](https://github.com/petervdonovan))
- Fix to not include downstream reactions in highlighted cycles [\#1270](https://github.com/lf-lang/lingua-franca/pull/1270) ([edwardalee](https://github.com/edwardalee))
- Correction of misplacement of self loops in diagrams [\#1274](https://github.com/lf-lang/lingua-franca/pull/1274) ([a-sr](https://github.com/a-sr))
- Fix that silences spurious error messages from Pylint [\#1280](https://github.com/lf-lang/lingua-franca/pull/1280) ([petervdonovan](https://github.com/petervdonovan))
- Patch that avoids NullPointerException in `CompileActionHandler` [\#1267](https://github.com/lf-lang/lingua-franca/pull/1267) ([lhstrh](https://github.com/lhstrh))
- Do not minimize the shadow jar [\#1285](https://github.com/lf-lang/lingua-franca/pull/1285) ([cmnrd](https://github.com/cmnrd))
- Fix for `ASTUtils.width()` returnning `-1` even when the width can be inferred from connections [\#1287](https://github.com/lf-lang/lingua-franca/pull/1287) ([hokeun](https://github.com/hokeun))
- Fix banks of modal reactors [\#1279](https://github.com/lf-lang/lingua-franca/pull/1279) ([a-sr](https://github.com/a-sr))
- Ensure that reactions consistently trigger banks [\#1289](https://github.com/lf-lang/lingua-franca/pull/1289) ([edwardalee](https://github.com/edwardalee))
- Cpp: fix triggering of reactions in multiple nested reactors [\#1286](https://github.com/lf-lang/lingua-franca/pull/1286) ([cmnrd](https://github.com/cmnrd))
- Fix to ensure the ordering of reactions relative to modes is correct [\#1303](https://github.com/lf-lang/lingua-franca/pull/1303) ([a-sr](https://github.com/a-sr))

**🚧 Maintenance and Refactoring**

- Machine-applicable refactorings [\#1224](https://github.com/lf-lang/lingua-franca/pull/1224) ([petervdonovan](https://github.com/petervdonovan))
- Inclusion of reactor-rs as a submodule [\#1296](https://github.com/lf-lang/lingua-franca/pull/1296) ([cmnrd](https://github.com/cmnrd))

**⬆️ Updated Dependencies**

- Upgrade to Xtext 2.27.0 and Lsp4j 0.14.0 [\#1234](https://github.com/lf-lang/lingua-franca/pull/1234) ([lhstrh](https://github.com/lhstrh))
- Bump Klighd version to 2.2 [\#1297](https://github.com/lf-lang/lingua-franca/pull/1297) ([a-sr](https://github.com/a-sr))


### Submodule [lf-lang/reactor-c](http://github.com/lf-lang/reactor-c)

**✨ Enhancements**

- New handling of startup/shutdown/reset reactions in modes [\#79](https://github.com/lf-lang/reactor-c/pull/79) ([a-sr](https://github.com/a-sr))
- STP violations now result in error messages [\#82](https://github.com/lf-lang/reactor-c/pull/82) ([edwardalee](https://github.com/edwardalee))
- Removal of TAN messages and new capability to record in-transit messages in the RTI [\#61](https://github.com/lf-lang/reactor-c/pull/61) ([Soroosh129](https://github.com/Soroosh129))

**🔧 Fixes**

- Patch to ensure that deadlines with zero delay are never met [\#86](https://github.com/lf-lang/reactor-c/pull/86) ([billy-bao](https://github.com/billy-bao))


### Submodule [lf-lang/reactor-cpp](http://github.com/lf-lang/reactor-cpp)

**✨ Enhancements**

- Nix tooling expanded with cachegrind, callgrind, and memtest [\#13](https://github.com/lf-lang/reactor-cpp/pull/13) ([revol-xut](https://github.com/revol-xut))
- Relocation of `unistd.h` and `execinfo.h` includes to void namespace pollution [\#14](https://github.com/lf-lang/reactor-cpp/pull/14) ([erlingrj](https://github.com/erlingrj))
- Fixes that addresses warnings reported by clang-tidy [\#15](https://github.com/lf-lang/reactor-cpp/pull/15) ([cmnrd](https://github.com/cmnrd))
- Optimized port communication for scalar types [\#17](https://github.com/lf-lang/reactor-cpp/pull/17) ([cmnrd](https://github.com/cmnrd))

**🔧 Fixes**

- Fix to allow ports to have both triggers and further bindings [\#16](https://github.com/lf-lang/reactor-cpp/pull/16) ([cmnrd](https://github.com/cmnrd))


### Submodule [lf-lang/reactor-rs](http://github.com/lf-lang/reactor-rs)

- No Changes


### Submodule [lf-lang/reactor-ts](http://github.com/lf-lang/reactor-ts)

**🚀 New Features**

- JSON serialization [\#99](https://github.com/lf-lang/reactor-ts/pull/99) ([CloverCho](https://github.com/CloverCho))

**🧪 Tests**

- Increased coverage of unit tests in bank.ts, multiport.ts and port.ts [\#100](https://github.com/lf-lang/reactor-ts/pull/100) ([goekberk](https://github.com/goekberk))


 
## [v0.2.1](https://github.com/lf-lang/lingua-franca/tree/v0.2.1) (2022-06-01)

**Highlights:**

This release includes bug fixes related to IDE tooling and federated execution. Various code cleanups and refactoring efforts have also been included in this release.

[Full Changelog](https://github.com/lf-lang/lingua-franca/compare/v0.2.0...v0.2.1)

**Implemented enhancements:**

- Introduce a thread-safe logging API for Rust [\#1138](https://github.com/lf-lang/lingua-franca/issues/1138)
- Turn warning for absence of main reactor into an informational message [\#1113](https://github.com/lf-lang/lingua-franca/issues/1113)
- Adjusted build to fall back to `python` command if `python3` is not available [\#1197](https://github.com/lf-lang/lingua-franca/pull/1197) ([Soroosh129](https://github.com/Soroosh129))
- Add more suitable default template for LF projects to Epoch [\#1180](https://github.com/lf-lang/lingua-franca/pull/1180) ([lhstrh](https://github.com/lhstrh))
- Add support for the STP handler to the Python target [\#1176](https://github.com/lf-lang/lingua-franca/pull/1176) ([Soroosh129](https://github.com/Soroosh129))
- Add federated docker file generation for TS target [\#1165](https://github.com/lf-lang/lingua-franca/pull/1165) ([housengw](https://github.com/housengw))
- Smarter default number of workers in C runtime [\#1139](https://github.com/lf-lang/lingua-franca/pull/1139) ([erlingrj](https://github.com/erlingrj))
- Add support for interleaved operator to Rust target [\#1133](https://github.com/lf-lang/lingua-franca/pull/1133) ([jhaye](https://github.com/jhaye))
- Change TS generator to support serialization of communication in federated execution [\#1125](https://github.com/lf-lang/lingua-franca/pull/1125) ([CloverCho](https://github.com/CloverCho))

**Fixed bugs:**

- Default template for "New LF Project" is federated program [\#1172](https://github.com/lf-lang/lingua-franca/issues/1172)
- Stop time is not handled correctly with decentralized coordination [\#1166](https://github.com/lf-lang/lingua-franca/issues/1166)
- Federated feedback with delay falsely detects cycles [\#1086](https://github.com/lf-lang/lingua-franca/issues/1086)
- Repair LSP tests [\#1199](https://github.com/lf-lang/lingua-franca/pull/1199) ([petervdonovan](https://github.com/petervdonovan))
- Fix segfault for internally disconnected output ports [\#1195](https://github.com/lf-lang/lingua-franca/pull/1195) ([Soroosh129](https://github.com/Soroosh129))
- Restore the ability to put the RTI on a remote machine [\#1164](https://github.com/lf-lang/lingua-franca/pull/1164) ([edwardalee](https://github.com/edwardalee))
- Fix segfault in python target argument handling [\#1161](https://github.com/lf-lang/lingua-franca/pull/1161) ([housengw](https://github.com/housengw))
- Fixed a few calls to deprecated functions that were missed [\#1131](https://github.com/lf-lang/lingua-franca/pull/1131) ([edwardalee](https://github.com/edwardalee))

**Closed issues:**

- GitHub Actions unable to install Java due to problems with Cloudflare CDN [\#1196](https://github.com/lf-lang/lingua-franca/issues/1196)
- Maven build failing on MacOS [\#1153](https://github.com/lf-lang/lingua-franca/issues/1153)
- "Required target resource not found" in Epoch 0.2.0 [\#1142](https://github.com/lf-lang/lingua-franca/issues/1142)

**Merged pull requests:**

- Augmented ErrorReporter API with informational messages [\#1192](https://github.com/lf-lang/lingua-franca/pull/1192) ([zekailin00](https://github.com/zekailin00))
- Update path to new documentation website in generated Dockerfile [\#1187](https://github.com/lf-lang/lingua-franca/pull/1187) ([erlingrj](https://github.com/erlingrj))
- Improve warning message when zero reactions in program [\#1186](https://github.com/lf-lang/lingua-franca/pull/1186) ([erlingrj](https://github.com/erlingrj))
- Fix white space issues in Lingua Franca test code [\#1185](https://github.com/lf-lang/lingua-franca/pull/1185) ([petervdonovan](https://github.com/petervdonovan))
- Fix whitespace in tests [\#1183](https://github.com/lf-lang/lingua-franca/pull/1183) ([petervdonovan](https://github.com/petervdonovan))
- Fix LSP test in expressions branch [\#1182](https://github.com/lf-lang/lingua-franca/pull/1182) ([petervdonovan](https://github.com/petervdonovan))
- Address LSP test failures. [\#1179](https://github.com/lf-lang/lingua-franca/pull/1179) ([petervdonovan](https://github.com/petervdonovan))
- Logging cleanups for VS Code [\#1174](https://github.com/lf-lang/lingua-franca/pull/1174) ([petervdonovan](https://github.com/petervdonovan))
- Added federated tests for TypeScript target [\#1159](https://github.com/lf-lang/lingua-franca/pull/1159) ([suyourice](https://github.com/suyourice))
- Update deprecated API in ui examples and tests [\#1155](https://github.com/lf-lang/lingua-franca/pull/1155) ([housengw](https://github.com/housengw))
- Remove trailing whitespace in C and Python generators [\#1154](https://github.com/lf-lang/lingua-franca/pull/1154) ([housengw](https://github.com/housengw))
- Have Epoch welcome page point to website rather than wiki page [\#1152](https://github.com/lf-lang/lingua-franca/pull/1152) ([edwardalee](https://github.com/edwardalee))
- Major refactor of the docker generators [\#1150](https://github.com/lf-lang/lingua-franca/pull/1150) ([housengw](https://github.com/housengw))
- Update references to deprecated APIs in `C` ui examples [\#1144](https://github.com/lf-lang/lingua-franca/pull/1144) ([housengw](https://github.com/housengw))
- `lf_` prefix more functions and reorganize files in `C` target [\#1143](https://github.com/lf-lang/lingua-franca/pull/1143) ([housengw](https://github.com/housengw))
- Refactor `doGenerate` in `C` generator and parts of Python and Typescript docker generators [\#1141](https://github.com/lf-lang/lingua-franca/pull/1141) ([housengw](https://github.com/housengw))
- Disable pylint `c-extension-no-member` warning [\#1140](https://github.com/lf-lang/lingua-franca/pull/1140) ([housengw](https://github.com/housengw))
- Check for Java 17 in lfc scripts and properly exit on error [\#1136](https://github.com/lf-lang/lingua-franca/pull/1136) ([cmnrd](https://github.com/cmnrd))
- Let `package_lfc` script put files in appropriately named directory [\#1135](https://github.com/lf-lang/lingua-franca/pull/1135) ([lhstrh](https://github.com/lhstrh))
- Update references to deprecated print APIs in python target [\#1134](https://github.com/lf-lang/lingua-franca/pull/1134) ([housengw](https://github.com/housengw))
- Drop `PermSize` and `MaxPermSize` options as they are not supported by Java 17 [\#1132](https://github.com/lf-lang/lingua-franca/pull/1132) ([cmnrd](https://github.com/cmnrd))
- Do not create the temporary dependency link if a connection between f… [\#1085](https://github.com/lf-lang/lingua-franca/pull/1085) ([edwardalee](https://github.com/edwardalee))
- Replace Value with Expression in the grammar [\#1023](https://github.com/lf-lang/lingua-franca/pull/1023) ([cmnrd](https://github.com/cmnrd))
- Scale back LSP tests [\#944](https://github.com/lf-lang/lingua-franca/pull/944) ([petervdonovan](https://github.com/petervdonovan))
 
## [v0.2.0](https://github.com/lf-lang/lingua-franca/tree/v0.2.0) (2022-05-01)

**Highlights:**

This release brings the minimum version requirement of Java to 17, which is a long-term support version that is also broadly supported by newer platforms (e.g., Apple silicon). Other changes included in this release are: a refactoring of the user-facing API of reactor-c, various bugfixes and enhancements of Rust, TypeScript, and C target, and minor improvements of the diagram synthesis.

[Full Changelog](https://github.com/lf-lang/lingua-franca/compare/v0.1.0...v0.2.0)

**Implemented enhancements:**

- The Rust target should support the 'workers' and 'threading' target properties [\#991](https://github.com/lf-lang/lingua-franca/issues/991)
- Optionally suppress the red No Main Reactor icon [\#1089](https://github.com/lf-lang/lingua-franca/pull/1089) ([edwardalee](https://github.com/edwardalee))

**Fixed bugs:**

- Segfault when calling `_lf_check_deadline` [\#1123](https://github.com/lf-lang/lingua-franca/issues/1123)
- In generated Rust code, logical actions in main reactors aren't mutable, making them not schedulable [\#1110](https://github.com/lf-lang/lingua-franca/issues/1110)
- Add categories explicitly to the diagram synthesis options [\#1119](https://github.com/lf-lang/lingua-franca/pull/1119) ([soerendomroes](https://github.com/soerendomroes))

**Closed issues:**

- User facing utility functions such as `info_print` should also have `lf_` prefix [\#1124](https://github.com/lf-lang/lingua-franca/issues/1124)
- Tracking the renaming of APIs in C and Python target [\#1108](https://github.com/lf-lang/lingua-franca/issues/1108)
- Remove logic for handling examples from testing framework [\#1079](https://github.com/lf-lang/lingua-franca/issues/1079)

**Merged pull requests:**

- Prefix print functions with `lf_` [\#1127](https://github.com/lf-lang/lingua-franca/pull/1127) ([housengw](https://github.com/housengw))
- Fix set executing\_reaction in self struct [\#1126](https://github.com/lf-lang/lingua-franca/pull/1126) ([housengw](https://github.com/housengw))
- Remove references to deprecated APIs in the code generator [\#1122](https://github.com/lf-lang/lingua-franca/pull/1122) ([housengw](https://github.com/housengw))
- Update Python target APIs to match the C target updates [\#1116](https://github.com/lf-lang/lingua-franca/pull/1116) ([housengw](https://github.com/housengw))
- Update `SET`, schedule and tag APIs in the C target [\#1103](https://github.com/lf-lang/lingua-franca/pull/1103) ([housengw](https://github.com/housengw))
- Remove redundant struct and related code in python target [\#1102](https://github.com/lf-lang/lingua-franca/pull/1102) ([housengw](https://github.com/housengw))
- Allow bank\_index in initializers [\#1101](https://github.com/lf-lang/lingua-franca/pull/1101) ([edwardalee](https://github.com/edwardalee))
- Added support for state variables in diagrams [\#1100](https://github.com/lf-lang/lingua-franca/pull/1100) ([a-sr](https://github.com/a-sr))
- Support inline code arguments on main reactor for Rust target [\#1099](https://github.com/lf-lang/lingua-franca/pull/1099) ([jhaye](https://github.com/jhaye))
- Add support for threading compiler flag to Rust target [\#1098](https://github.com/lf-lang/lingua-franca/pull/1098) ([jhaye](https://github.com/jhaye))
- Update `SET`, `schedule` and `tag` APIs in the C target [\#1097](https://github.com/lf-lang/lingua-franca/pull/1097) ([housengw](https://github.com/housengw))
- Java configuration bumped to version 17 [\#1094](https://github.com/lf-lang/lingua-franca/pull/1094) ([lhstrh](https://github.com/lhstrh))
- Add validation tests for ports in main or federated reactor [\#1091](https://github.com/lf-lang/lingua-franca/pull/1091) ([housengw](https://github.com/housengw))
- Update and add federated execution tests for TypeScript target [\#1062](https://github.com/lf-lang/lingua-franca/pull/1062) ([hokeun](https://github.com/hokeun))
 
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
