
# Parser (Compiler) for Lingua Franca

Tests are Makefile based.

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
