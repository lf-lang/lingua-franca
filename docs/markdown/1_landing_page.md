\mainpage Lingua Franca

Lingua Franca (LF) is a polyglot coordination language for concurrent and possibly time-sensitive applications ranging from low-level embedded code to distributed cloud and edge applications. An LF program specifies the interactions between components called reactors. The emphasis of the framework is on ensuring deterministic interaction with explicit management of timing. The logic of each reactor is written in one of a suite of target languages (currently C, C++, Python, and TypeScript) and can integrate legacy code in those languages. A code generator synthesizes one or more programs in the target language, which are then compiled using standard toolchains. If the application has exploitable parallelism, then it executes transparently on multiple cores without compromising determinacy. A distributed application translates into multiple programs and scripts to launch those programs on distributed machines. The communication fabric connecting components is synthesized as part of the programs.

See [lf-lang.org](https://lf-lang.org) for [installation instructions](https://www.lf-lang.org/docs/installation), [documentation](https://www.lf-lang.org/docs/), and [research papers](https://www.lf-lang.org/research/). See also the [wiki](https://github.com/lf-lang/lingua-franca/wiki/) for further information on ongoing projects.

The [documentation here](https://lf-lang.org/lingua-franca) is generated automatically from the C source code using [Doxygen](https://doxygen.nl).
Higher-level user documentation and getting-started information can be found on the
[lf-lang.org website](https://lf-lang.org/docs/next).

\note The most useful entry point to this documentation is likely the [topics page](topics.html).
