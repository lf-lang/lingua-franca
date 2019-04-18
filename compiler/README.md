
# Compiler for Lingua Franca

## Installation

This requires that Java and sbt being installed. On a Mac, you can install sbt using
```
brew install sbt
```

```sbt``` will install Scala.

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

## Testing

For testing ```node``` and the accessor framework need to be installed.
Following command installs a local copy of the accessor framefork:

```
make setup
```

If you have it already installed, you can set the variable ```ACCESSOR_PATH```
to your installation. This is best done in a local ```config.mk``` file, which
is included by the ```Makefile```.

Testing is run with

```
make test
```

Currently only the manual generated SimpleTest example is tested.
FIXME: change to current compiled project.

## Running from the Command Line

To run the compiler from the command line:
FIXME: I only know how to do this in this directory, lingua-franca/compiler.
FIX will be to distribute a .jar file containing the compiler.

```
sbt "runMain org.icyphy.lf.Compiler  SOURCEFILE.lf  DESTINATION.js"
```