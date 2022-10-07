# Contributing to Lingua Franca

This repository hosts the source code for the Lingua Franca compiler, which is written in Java and Kotlin.
We welcome contributions from the community and provide these guidelines in order to make contributing a smooth, efficient, and fun process.

## Reporting issues
[GitHub Issues](https://github.com/lf-lang/lingua-franca/issues) is the place to report bugs, request features, or document other issues that cannot be addressed immediately and need to be tracked over a longer period of time. We currently do not require any particular format for issues, but we do request that you properly label them to make it easier to manage them.

## Contributing code
You can work on the Lingua Franca code base in your favorite editor and build using Gradle (recommended) or Maven.
Please refer to our handbook for specific instructions for setting up a development environment in [IntelliJ](https://www.lf-lang.org/docs/handbook/intellij) or [Eclipse](https://www.lf-lang.org/docs/handbook/eclipse-oomph).

### Writing tests
An integral part of contributing code is writing tests. 

**Unit tests** for the compiler are located in the `org.lflang.tests` package and are implemented using the JUnit test framework. These tests are invoked using Gradle. For example, to run all the tests in the `org.lflang.tests.compiler` package, use the following command:
```
./gradlew test --tests org.lflang.tests.compiler.*
```
**Integration tests** consist of complete Lingua Franca programs that are located in the `test` directory in the root of the repository and are organized first by target, then by category. These tests get indexed automatically and are also invoked using Gradle. For instance, to run all the tests for the C target, use the following command:
```
 ./gradlew test --tests org.lflang.tests.runtime.CTest.*
 ```
 Tests in most categories are expected to compile without errors and return exit code `0` when executed. Tests in some categories are not attempted to run and are only expected to compile as they might require the presence of particular hardware or exotic software configurations that are not manageable in GitHub Actions, our current platform for Continuous Integration (CI). Only pushes to [feature branches](#feature-branches) associated with an active [pull request](#pull-requests) trigger CI.

### Workflow
Every change needs to go through a pull request (PR), pass all tests run in CI, and get an approving review before it is merged. Pushing to the `master` branch is restricted. All code review is conducted using the Github review system on PRs. Before requesting a code review, ensure that you have:
- applied the [code formatter](#code-style-and-formatting);
- documented your code;
- written [tests](#writing-tests) that cover your code; and
- accompanied any remaining `TODO`s or `FIXME`s with a link to an active [issue](#reporting-issues).

### Feature branches
Develop new changes in a feature branch or in a fork (if you don't have write permission on the repository). Please use an informative branch name and use kebab case (e.g., `my-new-feature`).

### Commit messages
We currently do not adhere to strict rules regarding commit messages, although that might change.

### Pull requests
When you file a PR, provide a clear and well-written description of the changes you are proposing. We use PRs to automatically construct our changelog and release notes; the text you write will be featured in them.

#### PR titles
Please make sure that the title of your PR adheres to the following rules:
1. Describe a contribution, not the activity of implementing it.
  - **Bad**: "Improve reporting of file system errors"
  - **Good**: "Improved reporting of file system errors"
  - **Bad**: "Fixes #123"
  - **Good**: "Fix for #123"
  - **Bad**: "Optimizing algorithm x"
  - **Good**: "Optimization of algorithm x"
2. Start with a capital.
  - **Bad**: "in a hurry"
  - **Good**: "Hurried new feature"
3. Do not use title case.
  - **Bad**: "Amazing Fix of a Terrible Bug"
  - **Good**: "Amazing fix of a terrible bug"
4. Do not use a period at the end.
  - **Bad**: "Titles are not sentences."
  - **Good**: "Titles are not sentences"

#### Labeling PRs
Labels are used to organize our changelog and release notes. Please label your PR to make it as clear as possible from the labels what this PR is about. If, for whatever reason, your changes should not appear in these digests, you can mark it as such using the `exclude` label.

#### Draft PRs
If a PR is not yet ready for review, mark it as **Draft**. Once it is ready for review, mark it **Ready for Review**.

#### Merging
Perform merges to bring your feature branch up-to-date with master locally (do not issue a PR). This is very easy to do.
1. Ensure you are on your feature branch (run `git branch` to find out; `git switch my-feature-branch` to switch to your feature branch).
2. Make sure you have the latest changes (run `git fetch --all`)
3. Perform the merge (run `git merge origin/master`, assuming that you want to merge the branch `master` from remote `origin` into `your-feature-branch`).

#### Addressing reviews
To address feedback from code review, implement changes and push to your feature branch. If you are certain that a reviewer's concern has been addressed by your new changes, hit the `Resolve conversation` button. If you are unsure, ask for follow up in the conversation and let the reviewer determine whether the feedback was addressed accordingly.

### Code style and formatting
The Lingua Franca compiler is implemented in Java and Kotlin. The overarching advice is to use each language's most widely used idioms and conventions, which are fortunately very similar. The code base is shipped with a [Spotless](https://github.com/diffplug/spotless) configuration to check and enforce style compliance. Lingua Franca code (e.g., tests) in this repository is also automatically formatted via Spotless.

- To check that modified files are formatted correctly, run:
```
./gradlew spotlessCheck
```
- To apply the changes recommended by the formatter, run:
```
./gradlew spotlessApply
```

More specific guidelines are described in [code-style.md](code-style.md) (mostly for Java right now). FIXME

## Reviewing code
FIXME