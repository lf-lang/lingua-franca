
# Parser (Compiler) for Lingua Franca

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
