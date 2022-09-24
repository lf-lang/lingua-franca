# Contributing to LF

### Setup

If you plan to contribute to Lingua Franca, we recommend following the [Developer Eclipse setup with Oomph](https://github.com/lf-lang/lingua-franca/wiki/Developer-Eclipse-Setup-with-Oomph) instructions. Otherwise, you can download a release.

In addition, if you want a Kotlin-friendly developer environment using IntelliJ, you can also follow the [Developer IntelliJ Setup (for Kotlin)](https://github.com/lf-lang/lingua-franca/wiki/Developer-IntelliJ-Setup-%28for-Kotlin%29) instructions to set it up.

You can also build and run the Lingua Franca IDE without the Eclipse setup. To run Lingua Franca IDE (Epoch) with Kotlin-based code generators enabled (which is not possible with the Eclipse setup), please see the instructions in [Running Lingua Franca IDE (Epoch) with Kotlin based Code Generators Enabled (without Eclipse Environment)](https://github.com/lf-lang/lingua-franca/wiki/Running-Lingua-Franca-IDE-%28Epoch%29-with-Kotlin-based-Code-Generators-Enabled-%28without-Eclipse-Environment%29).

### Github workflow



* Develop new changes in a feature branch or in a fork (if you don't have write permission on the repository).
* Every change needs to go through a PR and get an approving review before it is merged. Pushing to master is restricted.
* All code review is conducted using the Github review system on PRs.
#### Branch naming

#### Style for issues
#### Tagging

#### Commit messages

#### PRs
 - title formatting
 - description
 - templates?

#### Code review



### Coding conventions

The Lingua Franca compiler builds on the Xtext framework and is written in Java and partly in Kotlin.

The overarching guideline we follow is to use each language's most widely used idioms and conventions, which are fortunately very similar.

A couple of more specific code hygiene guidelines are described in [code-style.md](code-style.md) (mostly for Java right now).

#### Code formatting

The following code formatters can be imported into Eclipse:
* [Java](https://github.com/lf-lang/lingua-franca/blob/master/JavaFormatterEclipse.xml)

For Intellij, formatter config files are checked in and don't need to be imported manually. Just make sure to use the project-specific formatter setting.

FIXME: how to configure it? Details.
We should probably use Spotless.


