# Changelog

## [v0.11.0](https://github.com/lf-lang/lingua-franca/tree/v0.11.0) (2025-12-05)

**Highlights**

This release of Lingua Franca features changes to federated execution with the C and Python targets, emphasizing the decentralized coordinator. The obscure terminology of STA and STAA has been replaced by `maxwait` and `absent_after`. These are now specified as attributes on instantiations and connections, making it possible for libraries of reactors to be agnostic about whether the reactors are instantiated as federates or within a federate. In addition, the code docs for the code generator have been greatly improved and deployed at [https://lf-lang.org/lingua-franca](https://lf-lang.org/lingua-franca).

This release also makes a number of improvements to the reactor-c runtime, including the ability to set and change deadlines at runtime, support for after delays with fixed-size array types, better support for variable-size arrays in Windows, and a number of bug fixes.

This release also brings syntax and semantics of federated execution into compliance with the new (still preliminary) [`reactor-uc` runtime system and code generator](https://github.com/lf-lang/reactor-uc) that is particularly tuned for embedded platforms.

**🚀 New Features**

- Add `DeadlineUpdateDecrease.lf` & `DeadlineUpdateIncrease.lf` as an example of the new API function updating the deadline. [\#2531](https://github.com/lf-lang/lingua-franca/pull/2531) (@Jakio815)

**✨ Enhancements**

- Remove merged input ports for triggers [\#2540](https://github.com/lf-lang/lingua-franca/pull/2540) (@soerendomroes)
- Allow maxwait in place of STA and STAA [\#2541](https://github.com/lf-lang/lingua-franca/pull/2541) (@edwardalee)
- Added target uC for LSP support [\#2556](https://github.com/lf-lang/lingua-franca/pull/2556) (@edwardalee)
- New syntax for maxwait, absent_after, and tardy, replacing STA and STAA [\#2564](https://github.com/lf-lang/lingua-franca/pull/2564) (@edwardalee)

**🔧 Fixes**

- Made injector singleton and register the ide module an startup of LS [\#2539](https://github.com/lf-lang/lingua-franca/pull/2539) (@soerendomroes)
- Patch for Windows with gcc [\#2553](https://github.com/lf-lang/lingua-franca/pull/2553) (@edwardalee)
- Fix so that coral outline is used only for enclaves and align reactor-c [\#2554](https://github.com/lf-lang/lingua-franca/pull/2554) (@edwardalee)
- More doxygen warnings [\#2552](https://github.com/lf-lang/lingua-franca/pull/2552) (@edwardalee)
- Avoid copying type for Python [\#2557](https://github.com/lf-lang/lingua-franca/pull/2557) (@edwardalee)
- Use same list for length and access [\#2558](https://github.com/lf-lang/lingua-franca/pull/2558) (@edwardalee)
- Fix error when reporting errors on line 0 [\#2563](https://github.com/lf-lang/lingua-franca/pull/2563) (@edwardalee)
- Support delays of fixed-size arrays for C target [\#2567](https://github.com/lf-lang/lingua-franca/pull/2567) (@edwardalee)

**🚧 Maintenance and Refactoring**

- Docs cleanup 1 [\#2545](https://github.com/lf-lang/lingua-franca/pull/2545) (@edwardalee)
- Docs cleanup 2 [\#2546](https://github.com/lf-lang/lingua-franca/pull/2546) (@edwardalee)
- Docs cleanup 3 [\#2547](https://github.com/lf-lang/lingua-franca/pull/2547) (@edwardalee)
- Docs cleanup 4 [\#2548](https://github.com/lf-lang/lingua-franca/pull/2548) (@edwardalee)
- Doxygen warnings [\#2550](https://github.com/lf-lang/lingua-franca/pull/2550) (@edwardalee)
- Replace javadoc @code with backticks [\#2551](https://github.com/lf-lang/lingua-franca/pull/2551) (@edwardalee)
- Time syntax [\#2555](https://github.com/lf-lang/lingua-franca/pull/2555) (@edwardalee)
- Removed enclave tests for Python [\#2561](https://github.com/lf-lang/lingua-franca/pull/2561) (@edwardalee)
- Replace STA with maxwait, mark old versions deprecated [\#2568](https://github.com/lf-lang/lingua-franca/pull/2568) (@edwardalee)

**📖 Documentation**

- Docs cleanup 1 [\#2545](https://github.com/lf-lang/lingua-franca/pull/2545) (@edwardalee)
- Docs cleanup 2 [\#2546](https://github.com/lf-lang/lingua-franca/pull/2546) (@edwardalee)
- Docs cleanup 3 [\#2547](https://github.com/lf-lang/lingua-franca/pull/2547) (@edwardalee)
- Docs cleanup 4 [\#2548](https://github.com/lf-lang/lingua-franca/pull/2548) (@edwardalee)
- Use Doxygen to generate code documentation [\#2544](https://github.com/lf-lang/lingua-franca/pull/2544) (@edwardalee)
- Doxygen warnings [\#2550](https://github.com/lf-lang/lingua-franca/pull/2550) (@edwardalee)
- Replace javadoc @code with backticks [\#2551](https://github.com/lf-lang/lingua-franca/pull/2551) (@edwardalee)
- More doxygen warnings [\#2552](https://github.com/lf-lang/lingua-franca/pull/2552) (@edwardalee)
- Fix doxygen warnings and add doxygen check to CI [\#2569](https://github.com/lf-lang/lingua-franca/pull/2569) (@lsk567)


### Submodule [lf-lang/reactor-c](http://github.com/lf-lang/reactor-c)

**🚀 New Features**

- Addition of API of dynamically updating deadline of reaction. [\#538](https://github.com/lf-lang/reactor-c/pull/538) (@Jakio815)

**🔧 Fixes**

- Avoid variable size array on stack for Windows [\#541](https://github.com/lf-lang/reactor-c/pull/541) (@edwardalee)
- Patch for Windows with gcc [\#544](https://github.com/lf-lang/reactor-c/pull/544) (@edwardalee)
- Improve support for maxwait and tweak scheduling of physical actions [\#547](https://github.com/lf-lang/reactor-c/pull/547) (@edwardalee)

**🚧 Maintenance and Refactoring**

- Update checkout action to v4 [\#545](https://github.com/lf-lang/reactor-c/pull/545) (@edwardalee)


### Submodule [lf-lang/reactor-cpp](http://github.com/lf-lang/reactor-cpp)

- No Changes


### Submodule [lf-lang/reactor-rs](http://github.com/lf-lang/reactor-rs)

- No Changes



## [v0.10.1](https://github.com/lf-lang/lingua-franca/tree/v0.10.1) (2025-08-03)

**Highlights**

This patch release includes two key changes:

* A fix for an occasional segfault that could occur in federated execution during shutdown.
* A change in the default Docker configuration to build the RTI by default rather than to import it from DockerHub.

There are also a few other minor fixes, some of which are steps towards support for scheduling enclaves, which is still quite incomplete.

**✨ Enhancements**

- Make local build of the RTI the default [\#2530](https://github.com/lf-lang/lingua-franca/pull/2530) (@edwardalee)


### Submodule [lf-lang/reactor-c](http://github.com/lf-lang/reactor-c)

**✨ Enhancements**

- Preliminary support for enclaves in the C target [\#308](https://github.com/lf-lang/reactor-c/pull/308) (@erlingrj)

**🔧 Fixes**

- Fix for race condition creating a possible segfault [\#540](https://github.com/lf-lang/reactor-c/pull/540) (@edwardalee)


### Submodule [lf-lang/reactor-cpp](http://github.com/lf-lang/reactor-cpp)

- No Changes


### Submodule [lf-lang/reactor-rs](http://github.com/lf-lang/reactor-rs)

- No Changes



## [v0.10.0](https://github.com/lf-lang/lingua-franca/tree/v0.10.0) (2025-07-21)

**Highlights**

This release of Lingua Franca offers significant improvements to federated execution with the reactor-c and python targets, including now generating an RTI implementation with each federation, a number of improvements to the decentralized coordinator, an optimization for the centralized coordinator (DNET, downstream next-event tag), and support for sending statically allocated datatypes (strings and arrays) across federations.  This release also supports specifying a particular version of Python for the python target, provides support for more recent versions of Python, and fills in missing API functions for the python target.

This release also makes a number of improvements to the reactor-c runtime, including greatly improved [documentation](https://lf-lang.org/reactor-c), a new `update` policy for actions with colliding tags, making API macros available in methods, making reactor names available at runtime, federated improvements (described above), and a number of bug fixes. Note that there is a new (still preliminary) [`reactor-uc` runtime system and code generator](https://github.com/lf-lang/reactor-uc) that is particularly tuned for embedded platforms.

**🚀 New Features**

- Make macros available in methods [\#2437](https://github.com/lf-lang/lingua-franca/pull/2437) (@edwardalee)
- A target property for handling `downstream next event tag` (`DNET`) signal [\#2400](https://github.com/lf-lang/lingua-franca/pull/2400) (@byeonggiljun)
- Minimal changes to allow lff and lfd to run with reactor-uc [\#2485](https://github.com/lf-lang/lingua-franca/pull/2485) (@erlingrj)

**✨ Enhancements**

- Cleaner source directories copied to remote hosts [\#2440](https://github.com/lf-lang/lingua-franca/pull/2440) (@edwardalee)
- Use the NP scheduler when deadlines are present [\#2442](https://github.com/lf-lang/lingua-franca/pull/2442) (@lsk567)
- Make reactor names available at runtime [\#2450](https://github.com/lf-lang/lingua-franca/pull/2450) (@edwardalee)
- Allow derived classes to override parameter default values [\#2452](https://github.com/lf-lang/lingua-franca/pull/2452) (@edwardalee)
- CMake: Dont clear executable runtime path when installing [\#2454](https://github.com/lf-lang/lingua-franca/pull/2454) (@erlingrj)
- A target property for handling `downstream next event tag` (`DNET`) signal [\#2400](https://github.com/lf-lang/lingua-franca/pull/2400) (@byeonggiljun)
- Fed decentralized improvements [\#2482](https://github.com/lf-lang/lingua-franca/pull/2482) (@edwardalee)
- Do not subtract network delays from STAA and refactor code [\#2487](https://github.com/lf-lang/lingua-franca/pull/2487) (@edwardalee)
- Compile RTI for each federation [\#2492](https://github.com/lf-lang/lingua-franca/pull/2492) (@erlingrj)
- Tuned colors and validation for enclaves and deadlines [\#2495](https://github.com/lf-lang/lingua-franca/pull/2495) (@edwardalee)
- Show forever, never, and SI units in toString() [\#2498](https://github.com/lf-lang/lingua-franca/pull/2498) (@edwardalee)
- New `update` policy [\#2499](https://github.com/lf-lang/lingua-franca/pull/2499) (@lsk567)
- Fill in missing Python API functions [\#2512](https://github.com/lf-lang/lingua-franca/pull/2512) (@edwardalee)
- Upgrade to ROS2 0.7.13 [\#2513](https://github.com/lf-lang/lingua-franca/pull/2513) (@tanneberger)
- Add cmake-init-include target property [\#2432](https://github.com/lf-lang/lingua-franca/pull/2432) (@erlingrj)
- Support federated messages that are static strings and fixed-sized arrays [\#2525](https://github.com/lf-lang/lingua-franca/pull/2525) (@edwardalee)

**🔧 Fixes**

- Repaired export-dependency-graph property and dropped unsupported export-to-yaml property [\#2436](https://github.com/lf-lang/lingua-franca/pull/2436) (@cmnrd)
- Avoid unsupported -O3 option for the Patmos platform [\#2435](https://github.com/lf-lang/lingua-franca/pull/2435) (@EhsanKhodadad)
- Define LF_FILE_SEPARATOR in CMake [\#2438](https://github.com/lf-lang/lingua-franca/pull/2438) (@edwardalee)
- Patmos: better compiler flags [\#2444](https://github.com/lf-lang/lingua-franca/pull/2444) (@schoeberl)
- Python 11 and 12 support [\#2441](https://github.com/lf-lang/lingua-franca/pull/2441) (@edwardalee)
- Do not exit if there is no timeout, fast is true, and keepalive is true [\#2448](https://github.com/lf-lang/lingua-franca/pull/2448) (@edwardalee)
- Fix federated ZDC detection algorithm [\#2477](https://github.com/lf-lang/lingua-franca/pull/2477) (@erlingrj)
- Fed decentralized improvements [\#2482](https://github.com/lf-lang/lingua-franca/pull/2482) (@edwardalee)
- Do not subtract network delays from STAA and refactor code [\#2487](https://github.com/lf-lang/lingua-franca/pull/2487) (@edwardalee)
- Remove federate connections that have no effect [\#2491](https://github.com/lf-lang/lingua-franca/pull/2491) (@edwardalee)
- Fix cycle checking when calculating min delay from nearest physical action [\#2459](https://github.com/lf-lang/lingua-franca/pull/2459) (@lsk567)
- Fix issues with inheritance [\#2500](https://github.com/lf-lang/lingua-franca/pull/2500) (@edwardalee)
- Upgrade spotless and fix spotless hang [\#2504](https://github.com/lf-lang/lingua-franca/pull/2504) (@lsk567)
- Fix scope provider for parameter references. [\#2505](https://github.com/lf-lang/lingua-franca/pull/2505) (@edwardalee)
- Update async callback tests [\#2503](https://github.com/lf-lang/lingua-franca/pull/2503) (@lsk567)
- Fill in missing Python API functions [\#2512](https://github.com/lf-lang/lingua-franca/pull/2512) (@edwardalee)
- Make lf.source_directory() work for federated [\#2515](https://github.com/lf-lang/lingua-franca/pull/2515) (@edwardalee)
- Do not register atexit to mask errors [\#2518](https://github.com/lf-lang/lingua-franca/pull/2518) (@edwardalee)
- Address issue #2496 with space after {= [\#2524](https://github.com/lf-lang/lingua-franca/pull/2524) (@edwardalee)
- Fix preamble inheritance for instantiated reactors [\#2497](https://github.com/lf-lang/lingua-franca/pull/2497) (@Jakio815)
- Handle min_spacing of zero correctly [\#2527](https://github.com/lf-lang/lingua-franca/pull/2527) (@edwardalee)
- Upgrade regression tests to compile and run PATMOS tests [\#2471](https://github.com/lf-lang/lingua-franca/pull/2471) (@EhsanKhodadad)

**🚧 Maintenance and Refactoring**

- Cleaner source directories copied to remote hosts [\#2440](https://github.com/lf-lang/lingua-franca/pull/2440) (@edwardalee)
- Refactoring of socket communications. [\#2449](https://github.com/lf-lang/lingua-franca/pull/2449) (@Jakio815)
- Add `shutdown_mutex` to be initialized by code generator. [\#2474](https://github.com/lf-lang/lingua-franca/pull/2474) (@Jakio815)
- Do not subtract network delays from STAA and refactor code [\#2487](https://github.com/lf-lang/lingua-franca/pull/2487) (@edwardalee)

**📖 Documentation**

- Update contributing instructions [\#2509](https://github.com/lf-lang/lingua-franca/pull/2509) (@edwardalee)

**⬆️ Updated Dependencies**

- Bumping up reactor-ts package version. [\#2484](https://github.com/lf-lang/lingua-franca/pull/2484) (@hokeun)


### Submodule [lf-lang/reactor-c](http://github.com/lf-lang/reactor-c)

**🚀 New Features**

- Downstream next event tag (DNET), a new signal for more efficient centralized federated execution [\#349](https://github.com/lf-lang/reactor-c/pull/349) (@byeonggiljun)
- Add option for providing an external implementation of clock functions [\#516](https://github.com/lf-lang/reactor-c/pull/516) (@erlingrj)

**✨ Enhancements**

- Python 3.11, 3.12, and 3.13 support [\#501](https://github.com/lf-lang/reactor-c/pull/501) (@edwardalee)
- Make reactor names available at runtime [\#507](https://github.com/lf-lang/reactor-c/pull/507) (@edwardalee)
- Downstream next event tag (DNET), a new signal for more efficient centralized federated execution [\#349](https://github.com/lf-lang/reactor-c/pull/349) (@byeonggiljun)
- Fed decentralized improvements [\#519](https://github.com/lf-lang/reactor-c/pull/519) (@edwardalee)
- Handle STP violation over deadline when both occur [\#520](https://github.com/lf-lang/reactor-c/pull/520) (@edwardalee)
- Add semantic versioning of the RTI [\#521](https://github.com/lf-lang/reactor-c/pull/521) (@erlingrj)
- Renamed API function and allow zero argument [\#524](https://github.com/lf-lang/reactor-c/pull/524) (@edwardalee)
- New `update` policy [\#528](https://github.com/lf-lang/reactor-c/pull/528) (@lsk567)
- Fill in missing Python API functions [\#530](https://github.com/lf-lang/reactor-c/pull/530) (@edwardalee)
- Stringify CMake command-line arguments before they are added as compile definitions [\#494](https://github.com/lf-lang/reactor-c/pull/494) (@erlingrj)

**🔧 Fixes**

- Update trace_impl.c [\#498](https://github.com/lf-lang/reactor-c/pull/498) (@MoezBHH)
- Python 3.11, 3.12, and 3.13 support [\#501](https://github.com/lf-lang/reactor-c/pull/501) (@edwardalee)
- Usage of 'fixed-size' integer types in RTI code [\#453](https://github.com/lf-lang/reactor-c/pull/453) (@Jakio815)
- Fix lingua-franca-ref [\#510](https://github.com/lf-lang/reactor-c/pull/510) (@ChadliaJerad)
- Perform busy waiting when the wait duration is less than `MIN_SLEEP_DURATION` [\#514](https://github.com/lf-lang/reactor-c/pull/514) (@byeonggiljun)
- Fix memory bug when obtaining a port number [\#518](https://github.com/lf-lang/reactor-c/pull/518) (@erlingrj)
- Avoid a double join on the sensor_simulator output thread [\#517](https://github.com/lf-lang/reactor-c/pull/517) (@erlingrj)
- Fed decentralized improvements [\#519](https://github.com/lf-lang/reactor-c/pull/519) (@edwardalee)
- Handle STP violation over deadline when both occur [\#520](https://github.com/lf-lang/reactor-c/pull/520) (@edwardalee)
- Fill in missing Python API functions [\#530](https://github.com/lf-lang/reactor-c/pull/530) (@edwardalee)
- Don't print spurious warning [\#534](https://github.com/lf-lang/reactor-c/pull/534) (@edwardalee)
- Fix memory leaks [\#535](https://github.com/lf-lang/reactor-c/pull/535) (@edwardalee)
- Handle min_spacing of zero correctly [\#536](https://github.com/lf-lang/reactor-c/pull/536) (@edwardalee)

**🚧 Maintenance and Refactoring**

- Subtract time with care for overflow [\#492](https://github.com/lf-lang/reactor-c/pull/492) (@edwardalee)
- Refactoring of socket related functions to a separate socket_common.c [\#505](https://github.com/lf-lang/reactor-c/pull/505) (@Jakio815)
- Add `shutdown_socket()` function for follow up of #505 [\#506](https://github.com/lf-lang/reactor-c/pull/506) (@Jakio815)
- Compile RTI for each federation [\#523](https://github.com/lf-lang/reactor-c/pull/523) (@erlingrj)
- Use of snprintf instead of vulerable sprintf for code security best practices. [\#533](https://github.com/lf-lang/reactor-c/pull/533) (@hokeun)

**📖 Documentation**

- Clean up Doxygen documentation generation [\#529](https://github.com/lf-lang/reactor-c/pull/529) (@edwardalee)
- Fix the documentation about localy building docs [\#532](https://github.com/lf-lang/reactor-c/pull/532) (@ChadliaJerad)


### Submodule [lf-lang/reactor-cpp](http://github.com/lf-lang/reactor-cpp)

**✨ Enhancements**

- Update to ubuntu 24.04 [\#76](https://github.com/lf-lang/reactor-cpp/pull/76) (@erlingrj)


### Submodule [lf-lang/reactor-rs](http://github.com/lf-lang/reactor-rs)

- Fix all warnings for Rust 1.84.0 [\#51](https://github.com/lf-lang/reactor-rs/pull/51) (@oowekyala)
- Compat with rust 1.89 nightly [\#53](https://github.com/lf-lang/reactor-rs/pull/53) (@oowekyala)


## [v0.9.0](https://github.com/lf-lang/lingua-franca/tree/v0.9.0) (2024-10-31)

**Highlights**

This release of Lingua Franca adds support for reusable and publishable packages, providing the necessary infrastructure to develop a community-driven ecosystem of Lingua Franca packages. The VS code extension now features an integrated package explorer, and the diagrams provide colored highlighting for selected diagram edges. Other new features include C support for a new hardware platform called Patmos, several minor syntax and API improvements, and a new Alpine-based default Docker image for Python. This release also includes numerous bug fixes, such as preventing hangs during timeouts, addressing issues in decentralized coordination, improving error handling, and fixing concurrency issues in the C runtime.

**🚀 New Features**

- Colored highlighting of selected diagram edges [\#2324](https://github.com/lf-lang/lingua-franca/pull/2324) (@soerendomroes)
- Native `forever` and `never` time literal [\#2421](https://github.com/lf-lang/lingua-franca/pull/2421) (@Depetrol)
- Angular bracket imports for reusable reactor modules [\#2404](https://github.com/lf-lang/lingua-franca/pull/2404) (@vinzbarbuto)
- Support for Patmos platform [\#2383](https://github.com/lf-lang/lingua-franca/pull/2383) (@EhsanKhodadad)
- API for getting fully-qualified name in Cpp target [\#2431](https://github.com/lf-lang/lingua-franca/pull/2431) (@OmerMajNition)

**✨ Enhancements**

- Changes in the Lingua Franca Language Server to support improvements in the VSCode extension [\#2370](https://github.com/lf-lang/lingua-franca/pull/2370) (@vinzbarbuto)
- Default Docker image for Python changed to Alpine [\#2413](https://github.com/lf-lang/lingua-franca/pull/2413) (@lhstrh)
- Modal multiport [\#2422](https://github.com/lf-lang/lingua-franca/pull/2422) (@edwardalee)

**🔧 Fixes**

- Improvements in decentralized coordination [\#2394](https://github.com/lf-lang/lingua-franca/pull/2394) (@edwardalee)
- Prevent hanging on waiting for timeout time [\#2399](https://github.com/lf-lang/lingua-franca/pull/2399) (@edwardalee)
- Fix bank_index Parameter Override [\#2411](https://github.com/lf-lang/lingua-franca/pull/2411) (@Depetrol)
- Fix action is_present field not being reset  [\#2409](https://github.com/lf-lang/lingua-franca/pull/2409) (@erlingrj)
- Informative error message for platforms that do not support federated [\#2414](https://github.com/lf-lang/lingua-franca/pull/2414) (@lhstrh)
- Corrected behavior of federated code generation under `--no-compile` flag [\#2415](https://github.com/lf-lang/lingua-franca/pull/2415) (@lhstrh)
- Fix disconnected port handling [\#2416](https://github.com/lf-lang/lingua-franca/pull/2416) (@edwardalee)
- Fixed parameterized mutable inputs [\#2420](https://github.com/lf-lang/lingua-franca/pull/2420) (@edwardalee)
- Fixed concurrency bug in action scheduling in C runtime [\#2423](https://github.com/lf-lang/lingua-franca/pull/2423) (@Depetrol)
- Fix windows [\#2424](https://github.com/lf-lang/lingua-franca/pull/2424) (@edwardalee)
- File extension of TypeScript launch script adjusted on Windows [\#2427](https://github.com/lf-lang/lingua-franca/pull/2427) (@petervdonovan)
- Modal multiport [\#2422](https://github.com/lf-lang/lingua-franca/pull/2422) (@edwardalee)
- No segfault when Python reactions fail [\#2428](https://github.com/lf-lang/lingua-franca/pull/2428) (@edwardalee)
- Fixed concurrency another bug in action scheduling in C runtime [\#2429](https://github.com/lf-lang/lingua-franca/pull/2429) (@Depetrol)


### Submodule [lf-lang/reactor-c](http://github.com/lf-lang/reactor-c)

**🚀 New Features**

- lf_sleep and lf_nanosleep added to lf_patmos_support [\#478](https://github.com/lf-lang/reactor-c/pull/478) (@EhsanKhodadad)

**✨ Enhancements**

- Prevent hanging on waiting for timeout time [\#477](https://github.com/lf-lang/reactor-c/pull/477) (@edwardalee)

**🔧 Fixes**

- Bugfixes in decentralized coordination [\#476](https://github.com/lf-lang/reactor-c/pull/476) (@edwardalee)
- Fix action is_present field not being reset [\#482](https://github.com/lf-lang/reactor-c/pull/482) (@erlingrj)
- Prevent a warning when the second timer event is after the timeout time. [\#487](https://github.com/lf-lang/reactor-c/pull/487) (@edwardalee)
- Fix Unintended Action Override [\#490](https://github.com/lf-lang/reactor-c/pull/490) (@Depetrol)
- Fix Unintended Action Override [\#491](https://github.com/lf-lang/reactor-c/pull/491) (@Depetrol)

**🚧 Maintenance and Refactoring**

- Fix compiler warnings in Zephyr and FlexPRET support files [\#479](https://github.com/lf-lang/reactor-c/pull/479) (@erlingrj)
- Improved formatting in Dockerfile [\#483](https://github.com/lf-lang/reactor-c/pull/483) (@lhstrh)
- Various fixes to silence warnings on Windows [\#486](https://github.com/lf-lang/reactor-c/pull/486) (@edwardalee)


### Submodule [lf-lang/reactor-cpp](http://github.com/lf-lang/reactor-cpp)

- Fix clang-tidy configuration and tidy up code [\#60](https://github.com/lf-lang/reactor-cpp/pull/60) (@cmnrd)

### Submodule [lf-lang/reactor-rs](http://github.com/lf-lang/reactor-rs)

- No Changes



## [v0.8.2](https://github.com/lf-lang/lingua-franca/tree/v0.8.2) (2024-08-02)

**Highlights**

This patch release includes minor bugfixes and several enhancements of our Docker support. It also adds custom serialization for the Python target and support for the use of target code expressions to specify time values in C++.

**🚀 New Features**

- Docker compose override [\#2371](https://github.com/lf-lang/lingua-franca/pull/2371) (@Depetrol)

**✨ Enhancements**

- Ability to use of target code expressions for time values in C++ [\#2369](https://github.com/lf-lang/lingua-franca/pull/2369) (@cmnrd)
- Do not require libexecinfo in C++ docker images [\#2372](https://github.com/lf-lang/lingua-franca/pull/2372) (@cmnrd)
- Immediate start of federates with STP offset under decentralized coordination & fix target code STP_offset [\#2368](https://github.com/lf-lang/lingua-franca/pull/2368) (@Depetrol)
- Custom Serialization in Python Target [\#2375](https://github.com/lf-lang/lingua-franca/pull/2375) (@Depetrol)
- RTI Docker Hub Continuous Deployment with Multiplatform Support [\#2384](https://github.com/lf-lang/lingua-franca/pull/2384) (@Depetrol)

**🔧 Fixes**

- Immediate start of federates with STP offset under decentralized coordination & fix target code STP_offset [\#2368](https://github.com/lf-lang/lingua-franca/pull/2368) (@Depetrol)
- Fixed docker support for the Python target [\#2377](https://github.com/lf-lang/lingua-franca/pull/2377) (@cmnrd)
- Fix to get get all preambles in Python + updated tests [\#2381](https://github.com/lf-lang/lingua-franca/pull/2381) (@edwardalee)
- C++ raw strings allowed in target code blocks [\#2385](https://github.com/lf-lang/lingua-franca/pull/2385) (@lhstrh)

**🚧 Maintenance and Refactoring**

- Renaming `Latest Tag Completed` to `Latest Tag Confirmed` [\#2346](https://github.com/lf-lang/lingua-franca/pull/2346) (@byeonggiljun)


### Submodule [lf-lang/reactor-c](http://github.com/lf-lang/reactor-c)

**🚀 New Features**

- Support for Patmos platform [\#383](https://github.com/lf-lang/reactor-c/pull/383) (@EhsanKhodadad)

**✨ Enhancements**

- Immediate start of federates with STA offset under decentralized coordination [\#469](https://github.com/lf-lang/reactor-c/pull/469) (@Depetrol)
- Custom Serialization in Python Target [\#471](https://github.com/lf-lang/reactor-c/pull/471) (@Depetrol)
- Optimization of LTC signals [\#445](https://github.com/lf-lang/reactor-c/pull/445) (@byeonggiljun)
- RTI dockerfile support for multi-architecture builds [\#464](https://github.com/lf-lang/reactor-c/pull/464) (@elgeeko1)


### Submodule [lf-lang/reactor-cpp](http://github.com/lf-lang/reactor-cpp)

**✨ Enhancements**

- Portable backtrace mechanism [\#59](https://github.com/lf-lang/reactor-cpp/pull/59) (@cmnrd)


### Submodule [lf-lang/reactor-rs](http://github.com/lf-lang/reactor-rs)

- No Changes



## [v0.8.1](https://github.com/lf-lang/lingua-franca/tree/v0.8.1) (2024-07-14)

**Highlights**

This patch release includes several minor bugfixes and enhancements, improving Docker support for the C++ target and providing a more complete implementation of watchdogs.

**✨ Enhancements**

- API to look up source and package directory in Python [\#2331](https://github.com/lf-lang/lingua-franca/pull/2331) (@edwardalee)
- Define self variable so it can be used in instantiations [\#2353](https://github.com/lf-lang/lingua-franca/pull/2353) (@edwardalee)
- Fixed build script support in C++ docker generation [\#2357](https://github.com/lf-lang/lingua-franca/pull/2357) (@cmnrd)
- Diagram support for watchdogs [\#2356](https://github.com/lf-lang/lingua-franca/pull/2356) (@edwardalee)
- Fixed C++ docker generation for when cmake is not installed [\#2358](https://github.com/lf-lang/lingua-franca/pull/2358) (@cmnrd)
- Effects made accessible in watchdog handlers [\#2359](https://github.com/lf-lang/lingua-franca/pull/2359) (@lhstrh)

**🚧 Maintenance and Refactoring**

- Platform name changed from `Nrf52` to `nRF52` [\#2350](https://github.com/lf-lang/lingua-franca/pull/2350) (@edwardalee)


### Submodule [lf-lang/reactor-c](http://github.com/lf-lang/reactor-c)

**🚀 New Features**

- New Python functions `lf.package_directory()` and `lf.source_directory()` [\#455](https://github.com/lf-lang/reactor-c/pull/455) (@edwardalee)

**🔧 Fixes**

- Better error messages when HMAC authentication is attempted by federates when RTI does not support it [\#461](https://github.com/lf-lang/reactor-c/pull/461) (@Jakio815)


### Submodule [lf-lang/reactor-cpp](http://github.com/lf-lang/reactor-cpp)

- No Changes


### Submodule [lf-lang/reactor-rs](http://github.com/lf-lang/reactor-rs)

- Remove creusot sources and merge back vecmap into main runtime crate [\#47](https://github.com/lf-lang/reactor-rs/pull/47) (@oowekyala)


## [v0.8.0](https://github.com/lf-lang/lingua-franca/tree/v0.8.0) (2024-07-02)

**Highlights**

This release includes new features including support for the FlexPRET platform and multi-threading on the RP2040 platform. Enhancements include broader and more customizable support for Docker, as well as fixes for various bugs related to C++ ports, nRF52, and Dockerfile generation. Additionally, new tests were added, dependencies were updated, and several bug fixes and enhancements were made across various runtime implementations.

**🚀 New Features**

- Support for FlexPRET platform [\#2262](https://github.com/lf-lang/lingua-franca/pull/2262) (@magnmaeh)
- Rp2040 multithreaded target support  [\#2178](https://github.com/lf-lang/lingua-franca/pull/2178) (@sberkun)
- Support for Docker environment files [\#2349](https://github.com/lf-lang/lingua-franca/pull/2349) (@lhstrh)

**✨ Enhancements**

- Support for Python 3.9.x [\#2292](https://github.com/lf-lang/lingua-franca/pull/2292) (@jackyk02)
- Fix deadline inference, test GEDF, and remove chain ID [\#2294](https://github.com/lf-lang/lingua-franca/pull/2294) (@edwardalee)
- Improved support for Docker [\#2234](https://github.com/lf-lang/lingua-franca/pull/2234) (@lhstrh)
- Fixed docker generation for images that use /bin/sh [\#2335](https://github.com/lf-lang/lingua-franca/pull/2335) (@cmnrd)
- Safer directory creation in Docker container [\#2337](https://github.com/lf-lang/lingua-franca/pull/2337) (@lhstrh)
- Docker networking enhancements [\#2345](https://github.com/lf-lang/lingua-franca/pull/2345) (@lhstrh)

**🔧 Fixes**

- Disambiguate set(0) for C++ ports [\#2302](https://github.com/lf-lang/lingua-franca/pull/2302) (@cmnrd)
- Fixed CMake generator for pico platform [\#2303](https://github.com/lf-lang/lingua-franca/pull/2303) (@edwardalee)
- Fixed NRF52 support [\#2305](https://github.com/lf-lang/lingua-franca/pull/2305) (@edwardalee)
- No more support for Python 3.9 [\#2312](https://github.com/lf-lang/lingua-franca/pull/2312) (@lhstrh)
- Make files available in Docker runner image [\#2327](https://github.com/lf-lang/lingua-franca/pull/2327) (@petervdonovan)
- Bugfixes in handling of target properties across imports [\#2232](https://github.com/lf-lang/lingua-franca/pull/2232) (@byeonggiljun)
- Fixed bug in modes with microsteps [\#2338](https://github.com/lf-lang/lingua-franca/pull/2338) (@edwardalee)
- Add `tty:true` to docker-compose.yml [\#2344](https://github.com/lf-lang/lingua-franca/pull/2344) (@lhstrh)
- Fixed copying of multiple files in the generated dockerfiles [\#2348](https://github.com/lf-lang/lingua-franca/pull/2348) (@cmnrd)
- Fixes in adaptive scheduler for federated execution [\#2347](https://github.com/lf-lang/lingua-franca/pull/2347) (@petervdonovan)

**🧪 Tests**

- Fixed docker testing [\#2328](https://github.com/lf-lang/lingua-franca/pull/2328) (@cmnrd)
- Added test case for inheriting auth property [\#2343](https://github.com/lf-lang/lingua-franca/pull/2343) (@lhstrh)

**⬆️ Updated Dependencies**

- Bumped klighd version to 3.0.2.v20240507 [\#2301](https://github.com/lf-lang/lingua-franca/pull/2301) (@cmnrd)


### Submodule [lf-lang/reactor-c](http://github.com/lf-lang/reactor-c)

**🚀 New Features**

- Support for FlexPRET platform [\#412](https://github.com/lf-lang/reactor-c/pull/412) (@magnmaeh)
- Rp2040 multithreaded target support [\#344](https://github.com/lf-lang/reactor-c/pull/344) (@sberkun)
- Trace deadline violations [\#457](https://github.com/lf-lang/reactor-c/pull/457) (@petervdonovan)

**✨ Enhancements**

- Let pthread detect if the cpu number is valid [\#436](https://github.com/lf-lang/reactor-c/pull/436) (@erlingrj)
- Run one worker on main thread [\#437](https://github.com/lf-lang/reactor-c/pull/437) (@sberkun)
- Rp2040 multithreaded target support [\#344](https://github.com/lf-lang/reactor-c/pull/344) (@sberkun)
- Redesign of GEDF scheduler [\#433](https://github.com/lf-lang/reactor-c/pull/433) (@edwardalee)

**🔧 Fixes**

- Support for hyphens and underscores in rti host name [\#435](https://github.com/lf-lang/reactor-c/pull/435) (@erlingrj)
- Redesign of GEDF scheduler [\#433](https://github.com/lf-lang/reactor-c/pull/433) (@edwardalee)
- Fix cmake syntax [\#440](https://github.com/lf-lang/reactor-c/pull/440) (@edwardalee)
- RP2040 support based on low-level platform API [\#441](https://github.com/lf-lang/reactor-c/pull/441) (@edwardalee)
- Suppress error: cast from pointer to integer of different size [\#448](https://github.com/lf-lang/reactor-c/pull/448) (@petervdonovan)
- Fixed initialization of the master worker thread id [\#454](https://github.com/lf-lang/reactor-c/pull/454) (@cmnrd)
- Increment tag after mode switch rather than set to 1 [\#459](https://github.com/lf-lang/reactor-c/pull/459) (@edwardalee)
- Fixes to prevent memory leaks in RTI [\#446](https://github.com/lf-lang/reactor-c/pull/446) (@byeonggiljun)
- Fix support for NRF52 [\#442](https://github.com/lf-lang/reactor-c/pull/442) (@edwardalee)
- Fix adaptive scheduler [\#463](https://github.com/lf-lang/reactor-c/pull/463) (@petervdonovan)
- Make tracing usable for debugging [\#462](https://github.com/lf-lang/reactor-c/pull/462) (@petervdonovan)

**🚧 Maintenance and Refactoring**

- Redesign of GEDF scheduler [\#433](https://github.com/lf-lang/reactor-c/pull/433) (@edwardalee)


### Submodule [lf-lang/reactor-cpp](http://github.com/lf-lang/reactor-cpp)

- Run spell-checker on docs [\#56](https://github.com/lf-lang/reactor-cpp/pull/56) (@erlingrj)
- Disambiguated set(0) on ports [\#57](https://github.com/lf-lang/reactor-cpp/pull/57) (@cmnrd)

### Submodule [lf-lang/reactor-rs](http://github.com/lf-lang/reactor-rs)

- No Changes



## [v0.7.2](https://github.com/lf-lang/lingua-franca/tree/v0.7.2) (2024-05-20)

**Highlights**

This release includes patches of the C runtime only.


### Submodule [lf-lang/reactor-c](http://github.com/lf-lang/reactor-c)

**✨ Enhancements**

- Update trace-plugin API [\#428](https://github.com/lf-lang/reactor-c/pull/428) (@erlingrj)

**🔧 Fixes**

- Fixed preprocessor directives for clock sync [\#425](https://github.com/lf-lang/reactor-c/pull/425) (@edwardalee)
- Zephyr: Reset sleeping semaphore and correctly check its return values [\#431](https://github.com/lf-lang/reactor-c/pull/431) (@erlingrj)


### Submodule [lf-lang/reactor-cpp](http://github.com/lf-lang/reactor-cpp)

- No Changes


### Submodule [lf-lang/reactor-rs](http://github.com/lf-lang/reactor-rs)

- No Changes



## [v0.7.1](https://github.com/lf-lang/lingua-franca/tree/v0.7.1) (2024-05-17)

**Highlights**

This patch release includes bugfixes that address imports, tracing plugins, clock synchronization, and code generation issues.

**🔧 Fixes**

- Search for imported reactors to include reactors instantiated in modes [\#2277](https://github.com/lf-lang/lingua-franca/pull/2277) (@erlingrj)
- Extensions to the trace-plugin property [\#2275](https://github.com/lf-lang/lingua-franca/pull/2275) (@erlingrj)
- Set annotated layout options for modes. [\#2267](https://github.com/lf-lang/lingua-franca/pull/2267) (@soerendomroes)
- Fixed code generation for nested generic reactor instances [\#2284](https://github.com/lf-lang/lingua-franca/pull/2284) (@cmnrd)
- Code-generator changes required for reactor-c clock-sync fix [\#2285](https://github.com/lf-lang/lingua-franca/pull/2285) (@erlingrj)


### Submodule [lf-lang/reactor-c](http://github.com/lf-lang/reactor-c)

**🔧 Fixes**

- Fix race condition in lf_watchdog_stop [\#417](https://github.com/lf-lang/reactor-c/pull/417) (@erlingrj)
- TracePluginProperty fixes [\#420](https://github.com/lf-lang/reactor-c/pull/420) (@erlingrj)
- RTI and federate socket fixes [\#422](https://github.com/lf-lang/reactor-c/pull/422) (@erlingrj)


### Submodule [lf-lang/reactor-cpp](http://github.com/lf-lang/reactor-cpp)

- No Changes


### Submodule [lf-lang/reactor-rs](http://github.com/lf-lang/reactor-rs)

- No Changes



## [v0.7.0](https://github.com/lf-lang/lingua-franca/tree/v0.7.0) (2024-05-01)

**Highlights**

This release includes several new features, enhancements, fixes, and maintenance/refactoring updates. Notable additions include a new Plugin API for tracing, improved support for Zephyr, and enhancements of the Docker support. Fixes address issues like path conversion on Windows and null pointer exceptions in message reporting. Maintenance efforts focus on test stabilization and dependency updates, with notable enhancements to documentation and codebase cleanliness. Additionally, enhancements and fixes are applied to the reactor-c and reactor-cpp runtime, with updates to platform APIs, memory management, and bug fixes related to tracing and enclave programs. **Caution: Breaking changes are included in reactor-c's support for federated execution.**

**🚀 New Features**

- Plugin API for tracing [\#2192](https://github.com/lf-lang/lingua-franca/pull/2192) (@petervdonovan)

**✨ Enhancements**

- Use login shell so ~/.bash_profile is sourced [\#2169](https://github.com/lf-lang/lingua-franca/pull/2169) (@edwardalee)
- Have colorized outputs from C/Cpp compilers as default [\#2182](https://github.com/lf-lang/lingua-franca/pull/2182) (@erlingrj)
- Change to CLOCK_REALTIME for C target + cleanup [\#2184](https://github.com/lf-lang/lingua-franca/pull/2184) (@erlingrj)
- Add watchdogs to environment struct [\#2172](https://github.com/lf-lang/lingua-franca/pull/2172) (@erlingrj)
- Fixed clang warnings in generated C++ code [\#2201](https://github.com/lf-lang/lingua-franca/pull/2201) (@cmnrd)
- Report unsupported target properties as errors [\#2217](https://github.com/lf-lang/lingua-franca/pull/2217) (@cmnrd)
- More user-friendly Docker support [\#2198](https://github.com/lf-lang/lingua-franca/pull/2198) (@lhstrh)
- Remove list types and list initialization syntax [\#2235](https://github.com/lf-lang/lingua-franca/pull/2235) (@cmnrd)
- Make the Kernel timer the default clock for Zephyr [\#2248](https://github.com/lf-lang/lingua-franca/pull/2248) (@erlingrj)
- IntelliJ run config to start the LS [\#2260](https://github.com/lf-lang/lingua-franca/pull/2260) (@soerendomroes)

**🔧 Fixes**

- Use Zephyr's CMake extension to correctly build and link reactor-c [\#2167](https://github.com/lf-lang/lingua-franca/pull/2167) (@erlingrj)
- Fixed path conversion on Windows [\#2174](https://github.com/lf-lang/lingua-franca/pull/2174) (@cmnrd)
- Reduced scope for conflicting main reactors check [\#2180](https://github.com/lf-lang/lingua-franca/pull/2180) (@lhstrh)
- Tests that were previously failing re-enabled [\#2168](https://github.com/lf-lang/lingua-franca/pull/2168) (@edwardalee)
- Escape special characters in type arguments. [\#2186](https://github.com/lf-lang/lingua-franca/pull/2186) (@petervdonovan)
- Directory definitions corrected and made to work for federated. [\#2204](https://github.com/lf-lang/lingua-franca/pull/2204) (@edwardalee)
- Fixed `#line` directives on Windows [\#2220](https://github.com/lf-lang/lingua-franca/pull/2220) (@petervdonovan)
- Fixed null pointer exception in message reporting [\#2230](https://github.com/lf-lang/lingua-franca/pull/2230) (@cmnrd)
- Fixed code generation for connections involving enclave banks with multiports [\#2222](https://github.com/lf-lang/lingua-franca/pull/2222) (@julianrobledom)
- Include clock sync for federated programs on multiple platforms. [\#2243](https://github.com/lf-lang/lingua-franca/pull/2243) (@edwardalee)
- Fixed clean building federated programs [\#2247](https://github.com/lf-lang/lingua-franca/pull/2247) (@cmnrd)
- Fix in error handling for Docker builds [\#2249](https://github.com/lf-lang/lingua-franca/pull/2249) (@petervdonovan)
- Docker compose also create bin directory if not present [\#2251](https://github.com/lf-lang/lingua-franca/pull/2251) (@erlingrj)
- Fix NPE bug for instance in mode [\#2270](https://github.com/lf-lang/lingua-franca/pull/2270) (@edwardalee)
- Bugfixes in the thread scheduling API [\#2268](https://github.com/lf-lang/lingua-franca/pull/2268) (@erlingrj)
- Fix for validation of target properties dictionaries [\#2272](https://github.com/lf-lang/lingua-franca/pull/2272) (@magnmaeh)

**🚧 Maintenance and Refactoring**

- Tests that were previously failing re-enabled [\#2168](https://github.com/lf-lang/lingua-franca/pull/2168) (@edwardalee)
- Change to CLOCK_REALTIME for C target + cleanup [\#2184](https://github.com/lf-lang/lingua-franca/pull/2184) (@erlingrj)
- Replace use of deprecated APIs in C tests [\#2190](https://github.com/lf-lang/lingua-franca/pull/2190) (@edwardalee)
- Deflake test [\#2196](https://github.com/lf-lang/lingua-franca/pull/2196) (@edwardalee)
- Directory definitions corrected and made to work for federated. [\#2204](https://github.com/lf-lang/lingua-franca/pull/2204) (@edwardalee)
- Further cleanup [\#2210](https://github.com/lf-lang/lingua-franca/pull/2210) (@edwardalee)
- Use lf_combine_deadline_and_level function [\#2226](https://github.com/lf-lang/lingua-franca/pull/2226) (@edwardalee)
- Removal of the `compiler-flags` target property [\#2233](https://github.com/lf-lang/lingua-franca/pull/2233) (@cmnrd)
- Replacement of `javax.inject` with `com.google.inject` [\#2253](https://github.com/lf-lang/lingua-franca/pull/2253) (@soerendomroes)

**⬆️ Updated Dependencies**

- Xtext bumped to 3.34.0 [\#2264](https://github.com/lf-lang/lingua-franca/pull/2264) (@lhstrh)
- Klighd upgraded to v3.0.1 [\#2263](https://github.com/lf-lang/lingua-franca/pull/2263) (@soerendomroes)


### Submodule [lf-lang/reactor-c](http://github.com/lf-lang/reactor-c)

**🚀 New Features**

- Plugin API for tracing [\#342](https://github.com/lf-lang/reactor-c/pull/342) (@petervdonovan)
- Extend platform API with functions for setting thread scheduling policy, core affinity and priority + remove C11 as platform [\#355](https://github.com/lf-lang/reactor-c/pull/355) (@erlingrj)

**✨ Enhancements**

- Use Zephyr's CMake extension to correctly build and link reactor-c [\#339](https://github.com/lf-lang/reactor-c/pull/339) (@erlingrj)
- Put a single event on the recycle queue initially to avoid runtime malloc [\#351](https://github.com/lf-lang/reactor-c/pull/351) (@erlingrj)
- Add address query reply message [\#369](https://github.com/lf-lang/reactor-c/pull/369) (@Jakio815)
- Print more info when assertion fails [\#376](https://github.com/lf-lang/reactor-c/pull/376) (@erlingrj)
- Use SI units in reporting human-readable time [\#380](https://github.com/lf-lang/reactor-c/pull/380) (@edwardalee)
- Zephyr make kernel clock default + improvements [\#400](https://github.com/lf-lang/reactor-c/pull/400) (@erlingrj)
- Improve handling of timeouts when federates connect to RTI/each other [\#407](https://github.com/lf-lang/reactor-c/pull/407) (@erlingrj)
- Unit testing of thread scheduling API + additional fixes [\#416](https://github.com/lf-lang/reactor-c/pull/416) (@erlingrj)

**🔧 Fixes**

- Rename semaphore.h/c to lf_semaphore.h/c [\#340](https://github.com/lf-lang/reactor-c/pull/340) (@edwardalee)
- Fix data races for _lf_count_payload_allocations and _lf_count_token_allocations [\#313](https://github.com/lf-lang/reactor-c/pull/313) (@erlingrj)
- Cleanup pass removing deprecated reactor-body APIs [\#353](https://github.com/lf-lang/reactor-c/pull/353) (@edwardalee)
- Fix watchdog termination [\#341](https://github.com/lf-lang/reactor-c/pull/341) (@erlingrj)
- No use of C11 threads on Windows [\#364](https://github.com/lf-lang/reactor-c/pull/364) (@petervdonovan)
- Fix #370 [\#371](https://github.com/lf-lang/reactor-c/pull/371) (@lhstrh)
- Fix tagged message length from `int32_t` to `uint32_t` [\#368](https://github.com/lf-lang/reactor-c/pull/368) (@Jakio815)
- Fix for Clang error [\#379](https://github.com/lf-lang/reactor-c/pull/379) (@lhstrh)
- Change ADR_RQ to ADR_QR for correct line drawing in trace_svg.html [\#377](https://github.com/lf-lang/reactor-c/pull/377) (@chanijjani)
- Fix tagged message length from int32_t to uint32_t [\#391](https://github.com/lf-lang/reactor-c/pull/391) (@Jakio815)
- Add address query reply message [\#392](https://github.com/lf-lang/reactor-c/pull/392) (@Jakio815)
- Update RTI dockerfile to match file layout change [\#399](https://github.com/lf-lang/reactor-c/pull/399) (@petervdonovan)
- Proper handling of negative value passed in as the number of federates [\#411](https://github.com/lf-lang/reactor-c/pull/411) (@chanijjani)
- Fix clock sync init option [\#414](https://github.com/lf-lang/reactor-c/pull/414) (@edwardalee)
- Add fix to thread scheduling API [\#415](https://github.com/lf-lang/reactor-c/pull/415) (@erlingrj)

**🚧 Maintenance and Refactoring**

- Further cleanup of assertions [\#347](https://github.com/lf-lang/reactor-c/pull/347) (@edwardalee)
- Move to CLOCK_REALTIME introduce clock.h and lf_atomic.h [\#346](https://github.com/lf-lang/reactor-c/pull/346) (@erlingrj)
- Cleanup pass removing deprecated reactor-body APIs [\#353](https://github.com/lf-lang/reactor-c/pull/353) (@edwardalee)
- Further cleanup [\#354](https://github.com/lf-lang/reactor-c/pull/354) (@edwardalee)
- Added lf_combine_deadline_and_level function for use by code generator [\#381](https://github.com/lf-lang/reactor-c/pull/381) (@edwardalee)
- Add clang-format and run it on the code-base [\#384](https://github.com/lf-lang/reactor-c/pull/384) (@erlingrj)
- Enable all warnings and treat them as errors [\#387](https://github.com/lf-lang/reactor-c/pull/387) (@erlingrj)
- Refactoring of event queue [\#390](https://github.com/lf-lang/reactor-c/pull/390) (@byeonggiljun)

**📖 Documentation**

- Fixes to README [\#345](https://github.com/lf-lang/reactor-c/pull/345) (@lhstrh)
- Use SI units in reporting human-readable time [\#380](https://github.com/lf-lang/reactor-c/pull/380) (@edwardalee)


### Submodule [lf-lang/reactor-cpp](http://github.com/lf-lang/reactor-cpp)

**🔧 Fixes**

- Fixed bug that arises when enabling tracing in programs with enclaves. [\#55](https://github.com/lf-lang/reactor-cpp/pull/55) (@julianrobledom)


### Submodule [lf-lang/reactor-rs](http://github.com/lf-lang/reactor-rs)

- No Changes



## [v0.6.0](https://github.com/lf-lang/lingua-franca/tree/v0.6.0) (2024-01-26)

**Highlights**

This release improves the federated execution capability, makes the handling of target properties more modular and robust, and provides various bugfixes and small enhancements.

**🚀 New Features**

- Usage of high-resolution counter device on Zephyr boards. Update to v3.4.0 [\#2007](https://github.com/lf-lang/lingua-franca/pull/2007) (@erlingrj)
- Bracket list expression for initialization without escaping [\#2003](https://github.com/lf-lang/lingua-franca/pull/2003) (@oowekyala)
- [C] Self struct made available to initializers of state variables, parameters [\#2014](https://github.com/lf-lang/lingua-franca/pull/2014) (@OmerMajNition)
- New `--no-source-mapping` switch to disable line directives [\#2092](https://github.com/lf-lang/lingua-franca/pull/2092) (@lhstrh)
- Ability to specify rti image for dockerized federated programs [\#2144](https://github.com/lf-lang/lingua-franca/pull/2144) (@lhstrh)

**✨ Enhancements**

- Show reaction names in diagrams [\#2030](https://github.com/lf-lang/lingua-franca/pull/2030) (@cmnrd)
- Generate default values for the user-configurable CMake flags [\#2036](https://github.com/lf-lang/lingua-franca/pull/2036) (@erlingrj)
- Fix to generate launch script for TS target and print informational message [\#2090](https://github.com/lf-lang/lingua-franca/pull/2090) (@hokeun)
- Use more explicit include guards for Arduino [\#2102](https://github.com/lf-lang/lingua-franca/pull/2102) (@petervdonovan)
- Better placement of line directives [\#2101](https://github.com/lf-lang/lingua-franca/pull/2101) (@petervdonovan)
- String literal allowed as property name [\#2136](https://github.com/lf-lang/lingua-franca/pull/2136) (@oowekyala)
- Various bugfixes and cleanups in the support for federated programs [\#2140](https://github.com/lf-lang/lingua-franca/pull/2140) (@edwardalee)

**🔧 Fixes**

- Handling of unconnected outputs to avoid segmentation fault [\#2011](https://github.com/lf-lang/lingua-franca/pull/2011) (@edwardalee)
- Fixed timeout and shutdown in C++ [\#2012](https://github.com/lf-lang/lingua-franca/pull/2012) (@cmnrd)
- Stabilized LSP diagram genenation and added default expansion option [\#2018](https://github.com/lf-lang/lingua-franca/pull/2018) (@cmnrd)
- Avoid null pointer exception in diagrams with bodyless reactions [\#2020](https://github.com/lf-lang/lingua-franca/pull/2020) (@cmnrd)
- Removed "Reaction level" diagram synthesis option [\#2019](https://github.com/lf-lang/lingua-franca/pull/2019) (@cmnrd)
- Fixed error reporting in the verifier [\#2031](https://github.com/lf-lang/lingua-franca/pull/2031) (@cmnrd)
- Fixed trimming of the recorded test output [\#2040](https://github.com/lf-lang/lingua-franca/pull/2040) (@cmnrd)
- Fix problems with assignment serialization in diagrams [\#2038](https://github.com/lf-lang/lingua-franca/pull/2038) (@a-sr)
- Fixed bug in the C++ reaction dependency analysis [\#2050](https://github.com/lf-lang/lingua-franca/pull/2050) (@cmnrd)
- Handling of unknown width for multiports [\#2057](https://github.com/lf-lang/lingua-franca/pull/2057) (@edwardalee)
- Fixed race-condition in `_lf_replace_template_token` [\#2082](https://github.com/lf-lang/lingua-franca/pull/2082) (@erlingrj)
- Output path specified in JSON used correctly [\#2088](https://github.com/lf-lang/lingua-franca/pull/2088) (@lhstrh)
- Fix bug in C decentralized execution with small after delays [\#2032](https://github.com/lf-lang/lingua-franca/pull/2032) (@petervdonovan)
- Use more explicit include guards for Arduino [\#2102](https://github.com/lf-lang/lingua-franca/pull/2102) (@petervdonovan)
- Fixed mistake in code-generated reaction preamble [\#2105](https://github.com/lf-lang/lingua-franca/pull/2105) (@erlingrj)
- Fix undetected `scheduler` and `workers` properties [\#2110](https://github.com/lf-lang/lingua-franca/pull/2110) (@lsk567)
- Fix bug: set length on incoming token [\#2122](https://github.com/lf-lang/lingua-franca/pull/2122) (@edwardalee)
- Type check error turned into warning [\#2111](https://github.com/lf-lang/lingua-franca/pull/2111) (@lhstrh)
- Fix minimum spacing of actions and reference to freed event [\#2128](https://github.com/lf-lang/lingua-franca/pull/2128) (@edwardalee)
- Bugfix for TypeScript in dev mode [\#2130](https://github.com/lf-lang/lingua-franca/pull/2130) (@lhstrh)
- Parallel compilation disabled to avoid triggering deadlock in concurrent invocations of CMake [\#2145](https://github.com/lf-lang/lingua-franca/pull/2145) (@edwardalee)
- Python docker parent image fixed at version `3.10` [\#2151](https://github.com/lf-lang/lingua-franca/pull/2151) (@petervdonovan)
- Fix for cycle detection issue [\#2112](https://github.com/lf-lang/lingua-franca/pull/2112) (@lhstrh)
- Fixed `hashCode()` implementation in `TimeValue` class [\#2157](https://github.com/lf-lang/lingua-franca/pull/2157) (@lhstrh)
- Various bugfixes and cleanups in the support for federated programs [\#2140](https://github.com/lf-lang/lingua-franca/pull/2140) (@edwardalee)
- Removal of image-specific commands in case base image is specified [\#2158](https://github.com/lf-lang/lingua-franca/pull/2158) (@lhstrh)
- Fix some zephyr build issues [\#2160](https://github.com/lf-lang/lingua-franca/pull/2160) (@erlingrj)
- Fix for #2087  [\#2147](https://github.com/lf-lang/lingua-franca/pull/2147) (@a-sr)
- Fixed parameter forwarding for non-trivial types in C++ [\#2164](https://github.com/lf-lang/lingua-franca/pull/2164) (@cmnrd)
- Fixed generation of preambles for unused imports [\#2165](https://github.com/lf-lang/lingua-franca/pull/2165) (@cmnrd)

**🚧 Maintenance and Refactoring**

- Apply spotbugs plugin [\#2005](https://github.com/lf-lang/lingua-franca/pull/2005) (@cmnrd)
- Cleaned up test code to fix all test related spotbugs warnings [\#2006](https://github.com/lf-lang/lingua-franca/pull/2006) (@cmnrd)
- Remove deprecated C schedulers++ [\#2037](https://github.com/lf-lang/lingua-franca/pull/2037) (@erlingrj)
- Removal of `ulog` from `reactor-ts`, code generator updated to allow this change [\#2052](https://github.com/lf-lang/lingua-franca/pull/2052) (@axmmisaka)
- Refactoring of target properties and their validation [\#2008](https://github.com/lf-lang/lingua-franca/pull/2008) (@lhstrh)
- The 'threading' CLI option/build param/target property changed to 'single threaded' [\#1817](https://github.com/lf-lang/lingua-franca/pull/1817) (@patilatharva)
- Minimal changes to merge reactor-c enclaves [\#2095](https://github.com/lf-lang/lingua-franca/pull/2095) (@erlingrj)
- Fix file structure for future `networks` option [\#2070](https://github.com/lf-lang/lingua-franca/pull/2070) (@Jakio815)
- Fix minimum spacing of actions and reference to freed event [\#2128](https://github.com/lf-lang/lingua-franca/pull/2128) (@edwardalee)

**📖 Documentation**

- Updated .github/workflows/README.md [\#2076](https://github.com/lf-lang/lingua-franca/pull/2076) (@Jakio815)

**🧪 Tests**

- Fixed error reporting in the verifier [\#2031](https://github.com/lf-lang/lingua-franca/pull/2031) (@cmnrd)
- Fixed trimming of the recorded test output [\#2040](https://github.com/lf-lang/lingua-franca/pull/2040) (@cmnrd)
- Adjustment to test output to show progress [\#2137](https://github.com/lf-lang/lingua-franca/pull/2137) (@lhstrh)
- Time limits imposed on integration tests [\#2141](https://github.com/lf-lang/lingua-franca/pull/2141) (@lhstrh)

**⬆️ Updated Dependencies**

- Bumped Gradle and Gradle plugin versions [\#2024](https://github.com/lf-lang/lingua-franca/pull/2024) (@lhstrh)


### Submodule [lf-lang/reactor-c](http://github.com/lf-lang/reactor-c)

**🚀 New Features**

- Scheduling enclaves [\#242](https://github.com/lf-lang/reactor-c/pull/242) (@erlingrj)

**✨ Enhancements**

- Use real-time sockets in federated execution [\#274](https://github.com/lf-lang/reactor-c/pull/274) (@erlingrj)
- Progress towards Python 3.11 support [\#255](https://github.com/lf-lang/reactor-c/pull/255) (@cmnrd)
- Use env command rather than /usr/bin/python3 [\#297](https://github.com/lf-lang/reactor-c/pull/297) (@edwardalee)
- Priority queue refactoring [\#306](https://github.com/lf-lang/reactor-c/pull/306) (@edwardalee)
- Detect ZDC in RTI and issue PTAG only for nodes in ZDC [\#311](https://github.com/lf-lang/reactor-c/pull/311) (@edwardalee)
- Add an option for specifying the time interval of showing messages to the tracing visualizer [\#320](https://github.com/lf-lang/reactor-c/pull/320) (@byeong-gil)
- Various bugfixes and cleanups in the support for federated programs [\#323](https://github.com/lf-lang/reactor-c/pull/323) (@edwardalee)

**🔧 Fixes**

- Removal of the "waiting for tag" field [\#266](https://github.com/lf-lang/reactor-c/pull/266) (@petervdonovan)
- Mixed microsteps and time [\#269](https://github.com/lf-lang/reactor-c/pull/269) (@petervdonovan)
- Fedsd fix self_id [\#271](https://github.com/lf-lang/reactor-c/pull/271) (@ChadliaJerad)
- Guard against unconnected outputs [\#275](https://github.com/lf-lang/reactor-c/pull/275) (@edwardalee)
- Fixed argparse for `fedsd` [\#273](https://github.com/lf-lang/reactor-c/pull/273) (@erlingrj)
- Fixed `GEDF_NP` and removal of deprecated schedulers [\#282](https://github.com/lf-lang/reactor-c/pull/282) (@erlingrj)
- Fixed overflow bug affecting 32-bit Linux platforms [\#288](https://github.com/lf-lang/reactor-c/pull/288) (@erlingrj)
- Zephyr: default thread stack size updated to 2KB [\#299](https://github.com/lf-lang/reactor-c/pull/299) (@erlingrj)
- Bugfix in C decentralized execution with small after delays [\#280](https://github.com/lf-lang/reactor-c/pull/280) (@petervdonovan)
- Added return for non-void return function for arduino support [\#309](https://github.com/lf-lang/reactor-c/pull/309) (@chacalnoir)
- Handle messages arriving during initial STA wait [\#316](https://github.com/lf-lang/reactor-c/pull/316) (@edwardalee)
- Fix minimum spacing of actions and reference to freed event [\#318](https://github.com/lf-lang/reactor-c/pull/318) (@edwardalee)
- Fix LF_ASSERT [\#324](https://github.com/lf-lang/reactor-c/pull/324) (@erlingrj)
- Various bugfixes and cleanups in the support for federated programs [\#323](https://github.com/lf-lang/reactor-c/pull/323) (@edwardalee)
- Fix compiler warning in adaptive scheduler [\#335](https://github.com/lf-lang/reactor-c/pull/335) (@petervdonovan)
- Fix for silly bug in pretty printing time [\#336](https://github.com/lf-lang/reactor-c/pull/336) (@edwardalee)

**🚧 Maintenance and Refactoring**

- Do not link with thread library when we are compiling for Zephyr [\#272](https://github.com/lf-lang/reactor-c/pull/272) (@erlingrj)
- Minor cleanups [\#279](https://github.com/lf-lang/reactor-c/pull/279) (@edwardalee)
- Fixed `GEDF_NP` and removal of deprecated schedulers [\#282](https://github.com/lf-lang/reactor-c/pull/282) (@erlingrj)
- Zephyr timing implementation refactored into two separate files [\#294](https://github.com/lf-lang/reactor-c/pull/294) (@erlingrj)
- Clean up warnings and associated docs [\#296](https://github.com/lf-lang/reactor-c/pull/296) (@edwardalee)
- Use env command rather than /usr/bin/python3 [\#297](https://github.com/lf-lang/reactor-c/pull/297) (@edwardalee)
- Mentions of "unthreaded" replaced with "single-threaded" [\#250](https://github.com/lf-lang/reactor-c/pull/250) (@patilatharva)
- Priority queue refactoring [\#306](https://github.com/lf-lang/reactor-c/pull/306) (@edwardalee)
- Make new directory under `federated` named `network` [\#292](https://github.com/lf-lang/reactor-c/pull/292) (@Jakio815)
- Update lingua-franca-ref.txt to point to master [\#315](https://github.com/lf-lang/reactor-c/pull/315) (@edwardalee)
- Fix minimum spacing of actions and reference to freed event [\#318](https://github.com/lf-lang/reactor-c/pull/318) (@edwardalee)
- LF_ASSERT: Cast to void to avoid unused warnings [\#326](https://github.com/lf-lang/reactor-c/pull/326) (@erlingrj)


### Submodule [lf-lang/reactor-cpp](http://github.com/lf-lang/reactor-cpp)

**✨ Enhancements**

- Timeout implemented so that shutdown is invoked only once [\#53](https://github.com/lf-lang/reactor-cpp/pull/53) (@cmnrd)

**🔧 Fixes**

- Timeout implemented so that shutdown is invoked only once [\#53](https://github.com/lf-lang/reactor-cpp/pull/53) (@cmnrd)
- Build the dependency graph only after establishing all connections [\#54](https://github.com/lf-lang/reactor-cpp/pull/54) (@cmnrd)


### Submodule [lf-lang/reactor-rs](http://github.com/lf-lang/reactor-rs)

- No Changes



## [v0.5.1](https://github.com/lf-lang/lingua-franca/tree/v0.5.1) (2023-09-12)

**Highlights**

This release addresses several issues in the C code generator and fixes Docker support for federations.

**✨ Enhancements**

- Avoid squeezing reaction, method, or preamble bodies onto a single line [\#1984](https://github.com/lf-lang/lingua-franca/pull/1984) (@petervdonovan)

**🔧 Fixes**

- Fix for setting federates' bank index [\#1989](https://github.com/lf-lang/lingua-franca/pull/1989) (@petervdonovan)
- Default hostname for RTI in dockerized federation changed from "localhost" to "rti" [\#1993](https://github.com/lf-lang/lingua-franca/pull/1993) (@byeong-gil)
- Fix for unconnected multiport and bank reactor bug [\#1953](https://github.com/lf-lang/lingua-franca/pull/1953) (@OmerMajNition)

**🚧 Maintenance and Refactoring**

- Gradlew not longer used to run dev version of lf cli tools [\#1988](https://github.com/lf-lang/lingua-franca/pull/1988) (@axmmisaka)
- More robust dev scripts and removed util directory [\#1995](https://github.com/lf-lang/lingua-franca/pull/1995) (@cmnrd)

**🧪 Tests**

- Tests for `lf_set_array` and persistent inputs [\#1987](https://github.com/lf-lang/lingua-franca/pull/1987) (@edwardalee)
- Minor fixes for C++ tests [\#1979](https://github.com/lf-lang/lingua-franca/pull/1979) (@revol-xut)


### Submodule [lf-lang/reactor-c](http://github.com/lf-lang/reactor-c)

- No Changes


### Submodule [lf-lang/reactor-cpp](http://github.com/lf-lang/reactor-cpp)

- No Changes


### Submodule [lf-lang/reactor-rs](http://github.com/lf-lang/reactor-rs)

- No Changes



## [v0.5.0](https://github.com/lf-lang/lingua-franca/tree/v0.5.0) (2023-08-30)

**Highlights**

This release introduces new syntax for initializers, includes a renovation of the C backend to let it generate more modular code, and brings new features like a watchdog construct, support for generics in C, support for SMT-solver-based formal verification using UCLID-5, and bare-iron support for the Raspberry Pi Pico platform.

**🚀 New Features**

- Types allowed in reactor type args [\#1639](https://github.com/lf-lang/lingua-franca/pull/1639) ([@oowekyala](https://github.com/oowekyala))
- Tracing of federate interactions [\#1632](https://github.com/lf-lang/lingua-franca/pull/1632) ([@edwardalee](https://github.com/edwardalee))
- Equals initializer syntax [\#1580](https://github.com/lf-lang/lingua-franca/pull/1580) ([@oowekyala](https://github.com/oowekyala))
- `--json` and `--json-file` CLI args add to `lfc` [\#1686](https://github.com/lf-lang/lingua-franca/pull/1686) ([@patilatharva](https://github.com/patilatharva))
- Generated structs exposed in header files, reactions linkable from separate source files, and updated C target preamble visibility [\#1599](https://github.com/lf-lang/lingua-franca/pull/1599) ([@petervdonovan](https://github.com/petervdonovan))
- Preprocessor definition for `LF_PACKAGE_DIRECTORY` [\#1720](https://github.com/lf-lang/lingua-franca/pull/1720) ([@edwardalee](https://github.com/edwardalee))
- Mechanism for printing execution statistics [\#1743](https://github.com/lf-lang/lingua-franca/pull/1743) ([@cmnrd](https://github.com/cmnrd))
- Watchdog support for the C target [\#1730](https://github.com/lf-lang/lingua-franca/pull/1730) ([@Benichiwa](https://github.com/Benichiwa))
- Automatically generated .vscode/settings.json file [\#1759](https://github.com/lf-lang/lingua-franca/pull/1759) ([@edwardalee](https://github.com/edwardalee))
- C Generics [\#1681](https://github.com/lf-lang/lingua-franca/pull/1681) ([@mkhubaibumer](https://github.com/mkhubaibumer))
- `USER_THREADS` specifiable in platform options [\#1721](https://github.com/lf-lang/lingua-franca/pull/1721) ([@siljesu](https://github.com/siljesu))
- Generic params allowed as generic arguments in C target [\#1804](https://github.com/lf-lang/lingua-franca/pull/1804) ([@petervdonovan](https://github.com/petervdonovan))
- New `--check` flag for `lff` [\#1822](https://github.com/lf-lang/lingua-franca/pull/1822) ([@cmnrd](https://github.com/cmnrd))
- Environments in the C target [\#1772](https://github.com/lf-lang/lingua-franca/pull/1772) ([@erlingrj](https://github.com/erlingrj))
- `lfd` binary for generating diagrams from the command line [\#1713](https://github.com/lf-lang/lingua-franca/pull/1713) ([@cmnrd](https://github.com/cmnrd))
- `fedsd` utility updated to make the RTI optional and support enclaves visualization  [\#1870](https://github.com/lf-lang/lingua-franca/pull/1870) ([@ChadliaJerad](https://github.com/ChadliaJerad))
- C math lib always linked for C target [\#1894](https://github.com/lf-lang/lingua-franca/pull/1894) ([@cmnrd](https://github.com/cmnrd))
- Critical sections enabled outside of the context of a reactor [\#1901](https://github.com/lf-lang/lingua-franca/pull/1901) ([@edwardalee](https://github.com/edwardalee))
- Uclid5-based LF Verifier [\#1271](https://github.com/lf-lang/lingua-franca/pull/1271) ([@lsk567](https://github.com/lsk567))
- Python launch scripts [\#1914](https://github.com/lf-lang/lingua-franca/pull/1914) ([@cmnrd](https://github.com/cmnrd))
- Raspberry Pi Pico target support [\#1831](https://github.com/lf-lang/lingua-franca/pull/1831) ([@gundralaa](https://github.com/gundralaa))
- Handling cyclic dependencies for TypeScript federated execution [\#1925](https://github.com/lf-lang/lingua-franca/pull/1925) ([@byeong-gil](https://github.com/byeong-gil))

**✨ Enhancements**

- Keeplive natively inferred in the C++ runtime [\#1630](https://github.com/lf-lang/lingua-franca/pull/1630) ([@cmnrd](https://github.com/cmnrd))
- File access [\#1715](https://github.com/lf-lang/lingua-franca/pull/1715) ([@edwardalee](https://github.com/edwardalee))
- Faster building of TypeScript [\#1611](https://github.com/lf-lang/lingua-franca/pull/1611) ([@lhstrh](https://github.com/lhstrh))
- Revised mechanism for handling the `files` target property [\#1700](https://github.com/lf-lang/lingua-franca/pull/1700) ([@lhstrh](https://github.com/lhstrh))
- Validator rules to check if target supports federation or inheritance [\#1726](https://github.com/lf-lang/lingua-franca/pull/1726) ([@cmnrd](https://github.com/cmnrd))
- Optimized Gradle configuration for faster building [\#1774](https://github.com/lf-lang/lingua-franca/pull/1774) ([@axmmisaka](https://github.com/axmmisaka))
- Adjustable port sides for reactors [\#1807](https://github.com/lf-lang/lingua-franca/pull/1807) ([@a-sr](https://github.com/a-sr))
- No more space inserted after `interleaved` keyword by formatter [\#1846](https://github.com/lf-lang/lingua-franca/pull/1846) ([@cmnrd](https://github.com/cmnrd))
- More natural syntax for reaction declarations [\#1853](https://github.com/lf-lang/lingua-franca/pull/1853) ([@lhstrh](https://github.com/lhstrh))
- Fix for extra whitespace around info messages [\#1879](https://github.com/lf-lang/lingua-franca/pull/1879) ([@oowekyala](https://github.com/oowekyala))
- TypeScript runtime bumped to `v0.5.0` [\#1927](https://github.com/lf-lang/lingua-franca/pull/1927) ([@byeong-gil](https://github.com/byeong-gil))
- Support for named and bodyless reactions in C++ [\#1933](https://github.com/lf-lang/lingua-franca/pull/1933) ([@cmnrd](https://github.com/cmnrd))
- Added @layout annotation to add arbitrary layout options an elements [\#1951](https://github.com/lf-lang/lingua-franca/pull/1951) ([@soerendomroes](https://github.com/soerendomroes))

**🔧 Fixes**

- Physical connections implemented as AST transformation  [\#1596](https://github.com/lf-lang/lingua-franca/pull/1596) ([@erlingrj](https://github.com/erlingrj))
- Fix for passing of command line options from lfc to the generator [\#1631](https://github.com/lf-lang/lingua-franca/pull/1631) ([@cmnrd](https://github.com/cmnrd))
- Fix for validation of target properties [\#1629](https://github.com/lf-lang/lingua-franca/pull/1629) ([@cmnrd](https://github.com/cmnrd))
- Fix in validation so that warnings are not reported as errors [\#1643](https://github.com/lf-lang/lingua-franca/pull/1643) ([@petervdonovan](https://github.com/petervdonovan))
- Fix for NPE in lfc error reporting [\#1655](https://github.com/lf-lang/lingua-franca/pull/1655) ([@cmnrd](https://github.com/cmnrd))
- Upstream connection delays now properly handled in the TypeScript federates [\#1607](https://github.com/lf-lang/lingua-franca/pull/1607) ([@byeong-gil](https://github.com/byeong-gil))
- Fix for authenticated federation [\#1698](https://github.com/lf-lang/lingua-franca/pull/1698) ([@Jakio815](https://github.com/Jakio815))
- Bugfixes in handling of `files` target property [\#1725](https://github.com/lf-lang/lingua-franca/pull/1725) ([@lhstrh](https://github.com/lhstrh))
- Preambles properly inherited from superclasses [\#1732](https://github.com/lf-lang/lingua-franca/pull/1732) ([@edwardalee](https://github.com/edwardalee))
- Reactor classes in the same file with the same name, up to case differences, are prohibited [\#1741](https://github.com/lf-lang/lingua-franca/pull/1741) ([@petervdonovan](https://github.com/petervdonovan))
- Fix for ROS serialization [\#1755](https://github.com/lf-lang/lingua-franca/pull/1755) ([@petervdonovan](https://github.com/petervdonovan))
- Improved line adjustment logic for federated programs [\#1760](https://github.com/lf-lang/lingua-franca/pull/1760) ([@petervdonovan](https://github.com/petervdonovan))
- Fixed race condition in C++ enclave coordination [\#1778](https://github.com/lf-lang/lingua-franca/pull/1778) ([@cmnrd](https://github.com/cmnrd))
- Fix for error reporting bug [\#1787](https://github.com/lf-lang/lingua-franca/pull/1787) ([@lhstrh](https://github.com/lhstrh))
- Fix warnings reported on CliBase. [\#1793](https://github.com/lf-lang/lingua-franca/pull/1793) ([@petervdonovan](https://github.com/petervdonovan))
- Fix to allow CLI args to be passed to federates [\#1798](https://github.com/lf-lang/lingua-franca/pull/1798) ([@petervdonovan](https://github.com/petervdonovan))
- Multiple fixes for federated programs with TypeScript target [\#1752](https://github.com/lf-lang/lingua-franca/pull/1752) ([@byeong-gil](https://github.com/byeong-gil))
- Fix for race condition in `uniqueName` [\#1815](https://github.com/lf-lang/lingua-franca/pull/1815) ([@petervdonovan](https://github.com/petervdonovan))
- Fedsd compatibility with pandas 2.0 [\#1836](https://github.com/lf-lang/lingua-franca/pull/1836) ([@ChadliaJerad](https://github.com/ChadliaJerad))
- Formatter fixes [\#1840](https://github.com/lf-lang/lingua-franca/pull/1840) ([@petervdonovan](https://github.com/petervdonovan))
- Adjustments for running the LF compiler in Epoch [\#1844](https://github.com/lf-lang/lingua-franca/pull/1844) ([@a-sr](https://github.com/a-sr))
- More formatter fixes [\#1850](https://github.com/lf-lang/lingua-franca/pull/1850) ([@petervdonovan](https://github.com/petervdonovan))
- More formatter fixes [\#1851](https://github.com/lf-lang/lingua-franca/pull/1851) ([@petervdonovan](https://github.com/petervdonovan))
- Fix for naming collision when generic reactor is instantiated with different parameters [\#1864](https://github.com/lf-lang/lingua-franca/pull/1864) ([@petervdonovan](https://github.com/petervdonovan))
- Fix for inheritance problem exposed in examples [\#1891](https://github.com/lf-lang/lingua-franca/pull/1891) ([@lhstrh](https://github.com/lhstrh))
- Fix verifier error when there is no main reactor [\#1916](https://github.com/lf-lang/lingua-franca/pull/1916) ([@lsk567](https://github.com/lsk567))
- No more use of unordered reactions in federated programs; fix for deadlocks in some federated programs [\#1684](https://github.com/lf-lang/lingua-franca/pull/1684) ([@arengarajan99](https://github.com/arengarajan99))
- Keyword `extends` added to tokens allowed in reaction bodies [\#1926](https://github.com/lf-lang/lingua-franca/pull/1926) ([@lhstrh](https://github.com/lhstrh))
- Fix for edge case in which comments are dropped [\#1924](https://github.com/lf-lang/lingua-franca/pull/1924) ([@petervdonovan](https://github.com/petervdonovan))
- Fix for IllegalArgumentException in diagram synthesis [\#1932](https://github.com/lf-lang/lingua-franca/pull/1932) ([@petervdonovan](https://github.com/petervdonovan))
- Fix for STP violation [\#1935](https://github.com/lf-lang/lingua-franca/pull/1935) ([@petervdonovan](https://github.com/petervdonovan))
- Fix for after delays that use user-provided declarations [\#1959](https://github.com/lf-lang/lingua-franca/pull/1959) ([@petervdonovan](https://github.com/petervdonovan))
- Bugfix for when top-level multiport width in federation depends on parameter [\#1956](https://github.com/lf-lang/lingua-franca/pull/1956) ([@petervdonovan](https://github.com/petervdonovan))
- Fix compilation error in code for reset state variables with time type [\#1964](https://github.com/lf-lang/lingua-franca/pull/1964) ([@a-sr](https://github.com/a-sr))

**🚧 Maintenance and Refactoring**

- Migration of Epoch into separate repository [\#1482](https://github.com/lf-lang/lingua-franca/pull/1482) ([@a-sr](https://github.com/a-sr))
- Improved CLI argument handling using `picocli` [\#1534](https://github.com/lf-lang/lingua-franca/pull/1534) ([@patilatharva](https://github.com/patilatharva))
- Compile definitions no longer hardcoded in generated CMakeLists.txt [\#1622](https://github.com/lf-lang/lingua-franca/pull/1622) ([@petervdonovan](https://github.com/petervdonovan))
- Remove unchecked compilation warnings [\#1638](https://github.com/lf-lang/lingua-franca/pull/1638) ([@oowekyala](https://github.com/oowekyala))
- Refactoring of part of the federated package [\#1663](https://github.com/lf-lang/lingua-franca/pull/1663) ([@lhstrh](https://github.com/lhstrh))
- Removal of the use of non-API global variables in tests [\#1696](https://github.com/lf-lang/lingua-franca/pull/1696) ([@edwardalee](https://github.com/edwardalee))
- Gradle bumped to version 8 [\#1691](https://github.com/lf-lang/lingua-franca/pull/1691) ([@lhstrh](https://github.com/lhstrh))
- Removal of deprecated `build-lfc` script and `buildLfc` Gradle task [\#1714](https://github.com/lf-lang/lingua-franca/pull/1714) ([@cmnrd](https://github.com/cmnrd))
- Delete unnecessary complexity from `build-lf-cli` [\#1745](https://github.com/lf-lang/lingua-franca/pull/1745) ([@petervdonovan](https://github.com/petervdonovan))
- C files for Python support retrieved from reactor-c repo [\#1757](https://github.com/lf-lang/lingua-franca/pull/1757) ([@lhstrh](https://github.com/lhstrh))
- Google autoformatter applied to all files [\#1766](https://github.com/lf-lang/lingua-franca/pull/1766) ([@lhstrh](https://github.com/lhstrh))
- Struct refactoring for actions and ports [\#1756](https://github.com/lf-lang/lingua-franca/pull/1756) ([@edwardalee](https://github.com/edwardalee))
- Redesign of the repository layout and gradle configuration [\#1779](https://github.com/lf-lang/lingua-franca/pull/1779) ([@cmnrd](https://github.com/cmnrd))
- Removal of odd mechanism for loading target generators dynamically [\#1813](https://github.com/lf-lang/lingua-franca/pull/1813) ([@cmnrd](https://github.com/cmnrd))
- Default line length set to 100 for LF files [\#1389](https://github.com/lf-lang/lingua-franca/pull/1389) ([@petervdonovan](https://github.com/petervdonovan))
- KlighD bumped to `2.3.0` and now retrieved from Maven Central [\#1823](https://github.com/lf-lang/lingua-franca/pull/1823) ([@cmnrd](https://github.com/cmnrd))
- Refactor error reporter [\#1527](https://github.com/lf-lang/lingua-franca/pull/1527) ([@oowekyala](https://github.com/oowekyala))
- Unknown port types handled with `unknown` instead of `Present` [\#1857](https://github.com/lf-lang/lingua-franca/pull/1857) ([@lhstrh](https://github.com/lhstrh))
- TS code generator adjusted to appease `eslint` [\#1878](https://github.com/lf-lang/lingua-franca/pull/1878) ([@lhstrh](https://github.com/lhstrh))
- Minor fixes for the README in the test directory [\#1903](https://github.com/lf-lang/lingua-franca/pull/1903) ([@lsk567](https://github.com/lsk567))
- Declarative Port Graph in C++ Runtime [\#1848](https://github.com/lf-lang/lingua-franca/pull/1848) ([@revol-xut](https://github.com/revol-xut))
- No more use of unordered reactions in federated programs; fix for deadlocks in some federated programs [\#1684](https://github.com/lf-lang/lingua-franca/pull/1684) ([@arengarajan99](https://github.com/arengarajan99))
- Tracing utils and Zephyr run scripts moved from `util` folder [\#1948](https://github.com/lf-lang/lingua-franca/pull/1948) ([@erlingrj](https://github.com/erlingrj))

**📖 Documentation**

- Documentation about LSP tests in `README.md` [\#1587](https://github.com/lf-lang/lingua-franca/pull/1587) ([@petervdonovan](https://github.com/petervdonovan))

**🧪 Tests**

- TypeScript tests for federates with physical connections [\#1623](https://github.com/lf-lang/lingua-franca/pull/1623) ([@byeong-gil](https://github.com/byeong-gil))
- Zephyr tests pinned to particular version of docker image [\#1648](https://github.com/lf-lang/lingua-franca/pull/1648) ([@erlingrj](https://github.com/erlingrj))
- Test for the support of delayed physical connections in the TypeScript target [\#1676](https://github.com/lf-lang/lingua-franca/pull/1676) ([@byeong-gil](https://github.com/byeong-gil))
- Test for parsing CLI arguments in `lfc` [\#1668](https://github.com/lf-lang/lingua-franca/pull/1668) ([@patilatharva](https://github.com/patilatharva))
- Zephyr regression tests executed on QEMU [\#1678](https://github.com/lf-lang/lingua-franca/pull/1678) ([@erlingrj](https://github.com/erlingrj))
- CodeCov reporting for CLI tests [\#1688](https://github.com/lf-lang/lingua-franca/pull/1688) ([@lhstrh](https://github.com/lhstrh))
- Test to help ensure that level-based scheduling does not cause deadlock [\#1703](https://github.com/lf-lang/lingua-franca/pull/1703) ([@edwardalee](https://github.com/edwardalee))
- Flaky tests adjusted [\#1764](https://github.com/lf-lang/lingua-franca/pull/1764) ([@edwardalee](https://github.com/edwardalee))
- Spurious dependency example [\#1707](https://github.com/lf-lang/lingua-franca/pull/1707) ([@petervdonovan](https://github.com/petervdonovan))
- Added test of documented STP_offset parameter [\#1786](https://github.com/lf-lang/lingua-franca/pull/1786) ([@edwardalee](https://github.com/edwardalee))
- `SimpleFederatedAuthenticated.lf` test passing [\#1776](https://github.com/lf-lang/lingua-franca/pull/1776) ([@Jakio815](https://github.com/Jakio815))
- CI updates [\#1814](https://github.com/lf-lang/lingua-franca/pull/1814) ([@petervdonovan](https://github.com/petervdonovan))
- Parallel execution of round trip tests [\#1845](https://github.com/lf-lang/lingua-franca/pull/1845) ([@oowekyala](https://github.com/oowekyala))
- Tests for `lf_request_stop` with enclaves and federates [\#1871](https://github.com/lf-lang/lingua-franca/pull/1871) ([@edwardalee](https://github.com/edwardalee))
- Fixed code coverage aggregation and reporting [\#1868](https://github.com/lf-lang/lingua-franca/pull/1868) ([@cmnrd](https://github.com/cmnrd))
- New job for building `epoch` in CI [\#1974](https://github.com/lf-lang/lingua-franca/pull/1974) ([@lhstrh](https://github.com/lhstrh))

**⬆️ Updated Dependencies**

- Update to Zephyr v3.3.0 and SDK v0.16.1 [\#1825](https://github.com/lf-lang/lingua-franca/pull/1825) ([@erlingrj](https://github.com/erlingrj))
- Reactor-ts bumped to v0.4.0 [\#1749](https://github.com/lf-lang/lingua-franca/pull/1749) ([@lhstrh](https://github.com/lhstrh))
- Gradle Wrapper bumped to `8.1.1` [\#1890](https://github.com/lf-lang/lingua-franca/pull/1890) ([@lhstrh](https://github.com/lhstrh))
- Eclipse-related dependencies bumped to latest releases [\#1889](https://github.com/lf-lang/lingua-franca/pull/1889) ([@a-sr](https://github.com/a-sr))
- TypeScript runtime bumped to `v0.5.0` [\#1927](https://github.com/lf-lang/lingua-franca/pull/1927) ([@byeong-gil](https://github.com/byeong-gil))


### Submodule [lf-lang/reactor-c](http://github.com/lf-lang/reactor-c)

**🚀 New Features**

- New tracepoint for deadline misses [\#169](https://github.com/lf-lang/reactor-c/pull/169) ([@erlingrj](https://github.com/erlingrj))
- Compile definitions for federated programs [\#179](https://github.com/lf-lang/reactor-c/pull/179) ([@petervdonovan](https://github.com/petervdonovan))
- Tracing federate interactions [\#178](https://github.com/lf-lang/reactor-c/pull/178) ([@edwardalee](https://github.com/edwardalee))
- CMake definition to find `FEDERATED_AUTHENTICATED` [\#196](https://github.com/lf-lang/reactor-c/pull/196) ([@Jakio815](https://github.com/Jakio815))
- Memory reporting [\#201](https://github.com/lf-lang/reactor-c/pull/201) ([@edwardalee](https://github.com/edwardalee))
- Added `LF_PACKAGE_DIRECTORY` [\#204](https://github.com/lf-lang/reactor-c/pull/204) ([@edwardalee](https://github.com/edwardalee))
- Runtime support for watchdogs [\#177](https://github.com/lf-lang/reactor-c/pull/177) ([@Benichiwa](https://github.com/Benichiwa))
- Environments [\#212](https://github.com/lf-lang/reactor-c/pull/212) ([@erlingrj](https://github.com/erlingrj))
- Enclave request stop [\#244](https://github.com/lf-lang/reactor-c/pull/244) ([@edwardalee](https://github.com/edwardalee))
- Critical sections enabled outside of the context of a reactor [\#249](https://github.com/lf-lang/reactor-c/pull/249) ([@edwardalee](https://github.com/edwardalee))
- Rp2040 Target Support [\#253](https://github.com/lf-lang/reactor-c/pull/253) ([@gundralaa](https://github.com/gundralaa))
- Platform support for Raspberry Pi Pico [\#233](https://github.com/lf-lang/reactor-c/pull/233) ([@gundralaa](https://github.com/gundralaa))

**✨ Enhancements**

- Removal of unnecessary TAG messages [\#175](https://github.com/lf-lang/reactor-c/pull/175) ([@byeong-gil](https://github.com/byeong-gil))
- Cleaner namespace [\#189](https://github.com/lf-lang/reactor-c/pull/189) ([@petervdonovan](https://github.com/petervdonovan))
- File access and doc fixes [\#198](https://github.com/lf-lang/reactor-c/pull/198) ([@edwardalee](https://github.com/edwardalee))
- Improvements of support for watchdogs [\#209](https://github.com/lf-lang/reactor-c/pull/209) ([@edwardalee](https://github.com/edwardalee))
- Switch to more general thread creation in Zephyr support [\#194](https://github.com/lf-lang/reactor-c/pull/194) ([@siljesu](https://github.com/siljesu))
- Minor improvements to Zephyr platform [\#187](https://github.com/lf-lang/reactor-c/pull/187) ([@erlingrj](https://github.com/erlingrj))
- Output error when trying to use --auth (-a) for RTI built without -DAUTH=ON [\#222](https://github.com/lf-lang/reactor-c/pull/222) ([@hokeun](https://github.com/hokeun))
- Change nanosleep to lf_sleep in federate and RTI code [\#219](https://github.com/lf-lang/reactor-c/pull/219) ([@siljesu](https://github.com/siljesu))
- RTI exit while saving the trace file [\#228](https://github.com/lf-lang/reactor-c/pull/228) ([@ChadliaJerad](https://github.com/ChadliaJerad))
- Less namespace pollution [\#240](https://github.com/lf-lang/reactor-c/pull/240) ([@erlingrj](https://github.com/erlingrj))
- Enclaves tuning [\#243](https://github.com/lf-lang/reactor-c/pull/243) ([@edwardalee](https://github.com/edwardalee))
- If clock sync is on, link math [\#252](https://github.com/lf-lang/reactor-c/pull/252) ([@petervdonovan](https://github.com/petervdonovan))
- No more use of unordered reactions in federated programs [\#191](https://github.com/lf-lang/reactor-c/pull/191) ([@arengarajan99](https://github.com/arengarajan99))

**🔧 Fixes**

- Fix for definition of `LF_TIME_BUFFER_LENGTH` [\#197](https://github.com/lf-lang/reactor-c/pull/197) ([@edwardalee](https://github.com/edwardalee))
- Scheduler leak fix [\#200](https://github.com/lf-lang/reactor-c/pull/200) ([@edwardalee](https://github.com/edwardalee))
- Fix for Arduino to avoid duplicate definition of `timespec` [\#195](https://github.com/lf-lang/reactor-c/pull/195) ([@arengarajan99](https://github.com/arengarajan99))
- Suppression of "no symbols" warnings emitted by ranlib [\#214](https://github.com/lf-lang/reactor-c/pull/214) ([@petervdonovan](https://github.com/petervdonovan))
- Segfault fix [\#218](https://github.com/lf-lang/reactor-c/pull/218) ([@petervdonovan](https://github.com/petervdonovan))
- Zephyr fixes on thread creation and deletion [\#223](https://github.com/lf-lang/reactor-c/pull/223) ([@erlingrj](https://github.com/erlingrj))
- Minor fix of the federate id in the tracepoint [\#245](https://github.com/lf-lang/reactor-c/pull/245) ([@ChadliaJerad](https://github.com/ChadliaJerad))
- Fix protocol of HMAC authentication to start from federate. [\#231](https://github.com/lf-lang/reactor-c/pull/231) ([@Jakio815](https://github.com/Jakio815))
- Use of correct federate ID in tracing of absent messages [\#248](https://github.com/lf-lang/reactor-c/pull/248) ([@ChadliaJerad](https://github.com/ChadliaJerad))
- Memory leak in Python target fixed [\#246](https://github.com/lf-lang/reactor-c/pull/246) ([@jackykwok2024](https://github.com/jackykwok2024))
- Fix for fatal error raised during shutdown when decrementing a tag barrier that is zero [\#251](https://github.com/lf-lang/reactor-c/pull/251) ([@petervdonovan](https://github.com/petervdonovan))
- Fix for STP violation [\#257](https://github.com/lf-lang/reactor-c/pull/257) ([@petervdonovan](https://github.com/petervdonovan))

**🚧 Maintenance and Refactoring**

- Functions of rti.c moved to rti_lib.c to enable reuse [\#172](https://github.com/lf-lang/reactor-c/pull/172) ([@Jakio815](https://github.com/Jakio815))
- Code in `rti.c` made available as library [\#174](https://github.com/lf-lang/reactor-c/pull/174) ([@Jakio815](https://github.com/Jakio815))
- Documentation and code cleanup [\#193](https://github.com/lf-lang/reactor-c/pull/193) ([@edwardalee](https://github.com/edwardalee))
- Platform abstraction layer for the RTI [\#213](https://github.com/lf-lang/reactor-c/pull/213) ([@siljesu](https://github.com/siljesu))
- C files from reactor-c-py moved back into the reactor-c repo [\#217](https://github.com/lf-lang/reactor-c/pull/217) ([@lhstrh](https://github.com/lhstrh))
- Struct refactoring for actions and ports [\#216](https://github.com/lf-lang/reactor-c/pull/216) ([@edwardalee](https://github.com/edwardalee))
- Refactoring of the RTI implementation [\#224](https://github.com/lf-lang/reactor-c/pull/224) ([@ChadliaJerad](https://github.com/ChadliaJerad))
- `_lf_count_token_allocations` made `extern` instead of `static` [\#236](https://github.com/lf-lang/reactor-c/pull/236) ([@erlingrj](https://github.com/erlingrj))
- Refactoring of obsolete `gethostbyname()` in `connect_to_rti()` [\#220](https://github.com/lf-lang/reactor-c/pull/220) ([@siljesu](https://github.com/siljesu))
- No more use of unordered reactions in federated programs [\#191](https://github.com/lf-lang/reactor-c/pull/191) ([@arengarajan99](https://github.com/arengarajan99))
- Fewer warnings [\#258](https://github.com/lf-lang/reactor-c/pull/258) ([@edwardalee](https://github.com/edwardalee))

**📖 Documentation**

- Documentation and code cleanup [\#193](https://github.com/lf-lang/reactor-c/pull/193) ([@edwardalee](https://github.com/edwardalee))


### Submodule [lf-lang/reactor-cpp](http://github.com/lf-lang/reactor-cpp)

**🚀 New Features**

- Enclave connections and enclave coordination [\#44](https://github.com/lf-lang/reactor-cpp/pull/44) ([@cmnrd](https://github.com/cmnrd))
- Simple mechanism for collecting statistics during execution [\#47](https://github.com/lf-lang/reactor-cpp/pull/47) ([@cmnrd](https://github.com/cmnrd))
- Port graph [\#51](https://github.com/lf-lang/reactor-cpp/pull/51) ([@revol-xut](https://github.com/revol-xut))

**✨ Enhancements**

- Keep track of input actions in the environment [\#42](https://github.com/lf-lang/reactor-cpp/pull/42) ([@cmnrd](https://github.com/cmnrd))
- Factored event queue into its own class and fixed race condition between multiple starting enclaves [\#45](https://github.com/lf-lang/reactor-cpp/pull/45) ([@cmnrd](https://github.com/cmnrd))

**🔧 Fixes**

- Fix race condition in time barriers [\#49](https://github.com/lf-lang/reactor-cpp/pull/49) ([@cmnrd](https://github.com/cmnrd))
- Fix validate method and fix incorrect phase checks [\#50](https://github.com/lf-lang/reactor-cpp/pull/50) ([@cmnrd](https://github.com/cmnrd))


### Submodule [lf-lang/reactor-rs](http://github.com/lf-lang/reactor-rs)

- Use Cargo workspaces to directly include vecmap dependency [\#40](https://github.com/lf-lang/reactor-rs/pull/40) ([@jhaye](https://github.com/jhaye))
- Fixes for current Clippy version [\#45](https://github.com/lf-lang/reactor-rs/pull/45) ([@jhaye](https://github.com/jhaye))
- chore: Bump dependencies to avoid vulnerability in smallvec [\#44](https://github.com/lf-lang/reactor-rs/pull/44) ([@oowekyala](https://github.com/oowekyala))
- Fix use of multiport as reaction source [\#43](https://github.com/lf-lang/reactor-rs/pull/43) ([@oowekyala](https://github.com/oowekyala))


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
- `bank_index` (useful for banks of reactors) is now a proper parameter that can be passed down the reactor hierarchy via parameter assignment ([#424](https://github.com/lf-lang/lingua-franca/pull/424)).

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
