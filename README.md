[Website](https://lf-lang.org/) |
[Documentation](https://www.lf-lang.org/docs/handbook/) |
[Download](https://www.lf-lang.org/download/) |
[Contributing](CONTRIBUTING.md) |
[Changelog](CHANGELOG.md)

[![Build Status](https://github.com/lf-lang/lingua-franca/workflows/CI/badge.svg)](https://github.com/lf-lang/lingua-franca/actions/)
[![Nightly Build](https://github.com/lf-lang/lingua-franca/actions/workflows/nightly-build.yml/badge.svg)](https://github.com/lf-lang/lingua-franca/actions/workflows/nightly-build.yml)
[![CodeCov](https://codecov.io/gh/lf-lang/lingua-franca/branch/master/graph/badge.svg?token=b7LrpihI5a)](https://codecov.io/gh/lf-lang/lingua-franca)
[![GitHub Contributors](https://img.shields.io/github/contributors/lf-lang/lingua-franca)](https://github.com/lf-lang/lingua-franca/graphs/contributors)
[![Zulip](https://img.shields.io/badge/chat-zulip-informational)](https://lf-lang.zulipchat.com)
[![Twitter](https://img.shields.io/twitter/follow/thelflang.svg?style=social&logo=twitter)](https://twitter.com/thelflang)


# The Lingua Franca Coordination Language

Lingua Franca (LF) is a polyglot coordination language for concurrent and possibly time-sensitive applications ranging from low-level embedded code to distributed cloud and edge applications. An LF program specifies the interactions between components called reactors. The emphasis of the framework is on ensuring deterministic interaction with explicit management of timing. The logic of each reactor is written in one of a suite of target languages (currently C, C++, Python, and TypeScript) and can integrate legacy code in those languages. A code generator synthesizes one or more programs in the target language, which are then compiled using standard toolchains. If the application has exploitable parallelism, then it executes transparently on multiple cores without compromising determinacy. A distributed application translates into multiple programs and scripts to launch those programs on distributed machines. The communication fabric connecting components is synthesized as part of the programs.

See [lf-lang.org](https://lf-lang.org) for installation instructions and documentation. See also the [wiki](https://github.com/icyphy/lingua-franca/wiki) for further information on ongoing projects.

See our [Publications and Presentations](https://www.lf-lang.org/publications-and-presentations).


