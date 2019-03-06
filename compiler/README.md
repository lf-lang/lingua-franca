
# Compiler for Lingua Franca

## Installation

This requires that Java and Scala be installed. On a Mac, you can install Scala using
```
brew install sbt@1
```

## Running

Tests are Makefile based and use ```sbt``` for compile and run to
have the project in a Maven style layout.
See also [Building Java Projects With Sbt](http://xerial.org/blog/2014/03/24/sbt/).
 

```
make
```

translates the LF actor Source.lf into a JavaScript actor (Source.js).

```
make gui
```

will show the parser tree of an actor in LF.

The actual actor source can be changed with variable ARG, e.g.,

```
make gui ARG=Counter
```

However, at the moment probably only Source.lf works.

## Running from the Command Line

To run the compiler from the command line:
FIXME: I only know how to do this in this directory, lingua-franca/compiler.

```
sbt "runMain org.icyphy.lf.Compiler  SOURCEFILE.lf  DESTINATION.js"
```