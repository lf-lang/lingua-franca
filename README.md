[![Build Status](https://github.com/icyphy/lingua-franca/workflows/CI/badge.svg)](https://github.com/icyphy/lingua-franca/actions/)
[![codecov](https://codecov.io/gh/icyphy/lingua-franca/branch/master/graph/badge.svg?token=CDDG2CNYZY)](https://codecov.io/gh/icyphy/lingua-franca)
[![Discourse status](https://img.shields.io/badge/Community-online-blue)](https://community.lf-lang.org/)

# Lingua Franca

Lingua Franca (LF) is a polyglot coordination language for concurrent and possibly time-sensitive applications ranging from low-level embedded code to distributed cloud and edge applications. An LF program specifies the interactions between components called reactors. The emphasis of the framework is on ensuring deterministic interaction with explicit management of timing. The logic of each reactor is written in one of a suite of target languages (currently C, C++, Python, and TypeScript) and can integrate legacy code in those languages. A code generator synthesizes one or more programs in the target language, which are then compiled using standard toolchains. If the application has exploitable parallelism, then it executes transparently on multiple cores without compromising determinacy. A distributed application translates into multiple programs and scripts to launch those programs on distributed machines. The communication fabric connecting components is synthesized as part of the programs.

See the main [wiki](https://github.com/icyphy/lingua-franca/wiki) for further information and detailed documentation.

See our [Publications and Presentations](https://github.com/icyphy/lingua-franca/wiki/Publications-and-Presentations).

