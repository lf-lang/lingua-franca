[Website](https://lf-lang.org/) |
[Documentation](https://www.lf-lang.org/docs/) |
[Installation](https://www.lf-lang.org/docs/installation) |
[Contributing](CONTRIBUTING.md) |
[Changelog](CHANGELOG.md)

[![CI (targets)](https://github.com/lf-lang/lingua-franca/actions/workflows/all-targets.yml/badge.svg)](https://github.com/lf-lang/lingua-franca/actions/workflows/all-targets.yml?query=branch%3Amaster)
[![CI (misc)](https://github.com/lf-lang/lingua-franca/actions/workflows/all-misc.yml/badge.svg)](https://github.com/lf-lang/lingua-franca/actions/workflows/all-misc.yml?query=branch%3Amaster)
[![CI (misc)](https://github.com/lf-lang/lingua-franca/actions/workflows/all-embedded.yml/badge.svg)]([https://github.com/lf-lang/lingua-franca/actions/](https://github.com/lf-lang/lingua-franca/actions/workflows/all-embedded.yml?query=branch%3Amaster))

[![Open VSX Downloads](https://img.shields.io/open-vsx/dt/lf-lang/vscode-lingua-franca?label=Open%20VSX%20Registry%20%E2%A4%93)](https://open-vsx.org/extension/lf-lang/vscode-lingua-franca)
[![Visual Studio Marketplace Downloads](https://img.shields.io/visual-studio-marketplace/d/lf-lang.vscode-lingua-franca?label=VS%20Marketplace%20%E2%A4%93)](https://marketplace.visualstudio.com/items?itemName=lf-lang.vscode-lingua-franca)
[![Nightly Build](https://github.com/lf-lang/lingua-franca/actions/workflows/nightly-build.yml/badge.svg)](https://github.com/lf-lang/lingua-franca/releases/tag/nightly)

[![CodeCov](https://codecov.io/gh/lf-lang/lingua-franca/branch/master/graph/badge.svg?token=b7LrpihI5a)](https://codecov.io/gh/lf-lang/lingua-franca)
[![GitHub Contributors](https://img.shields.io/github/contributors/lf-lang/lingua-franca)](https://github.com/lf-lang/lingua-franca/graphs/contributors)
[![Zulip](https://img.shields.io/badge/chat-zulip-informational)](https://lf-lang.zulipchat.com)


# The Lingua Franca Coordination Language

Lingua Franca (LF) is a polyglot coordination language for concurrent and possibly time-sensitive applications ranging from low-level embedded code to distributed cloud and edge applications. An LF program specifies the interactions between components called reactors. The emphasis of the framework is on ensuring deterministic interaction with explicit management of timing. The logic of each reactor is written in one of a suite of target languages (currently C, C++, Python, and TypeScript) and can integrate legacy code in those languages. A code generator synthesizes one or more programs in the target language, which are then compiled using standard toolchains. If the application has exploitable parallelism, then it executes transparently on multiple cores without compromising determinacy. A distributed application translates into multiple programs and scripts to launch those programs on distributed machines. The communication fabric connecting components is synthesized as part of the programs.

See [lf-lang.org](https://lf-lang.org) for [installation instructions](https://www.lf-lang.org/docs/installation), [documentation](https://www.lf-lang.org/docs/), and [research papers](https://www.lf-lang.org/research/). See also the [wiki](https://github.com/lf-lang/lingua-franca/wiki/) for further information on ongoing projects.
