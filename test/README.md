# Testing Lingua Franca

System tests (complete Lingua Franca programs) are located in xtext/org.icyphy.linguafranca/src/test,
with one subdirectory per target.

## Running Tests on the Command Line

In the root directory for Lingua Franca, run this:
```
    bin/run-lf-tests
```

## Running Tests in Eclipse

In the project org.icyphy.linguafranca.tests, open the file:

`src/org/icyphy/tests/LinguaFrancaGeneratorTest.xtend`

Right click in the file (somewhere inside the `checkCTestModels()` method) and select Run As -> JUnit Test.