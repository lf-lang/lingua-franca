ANTLR=lib/antlr-4.7.2-complete.jar

ARG?=Source
SOURCE=../src/$(ARG).lf

all: lib comp lf

comp:
	java -jar $(ANTLR) -package org.icyphy.lf LinguaFranca.g4
	sbt compile

lf:
	sbt run < $(SOURCE) > $(ARG).js

gui:
	java -classpath .:$(ANTLR) org.antlr.v4.gui.TestRig LinguaFranca sys $(SOURCE) -gui

lib:
	-mkdir lib
	cd lib; curl -O https://www.antlr.org/download/antlr-4.7.2-complete.jar

clean:
	-rm *.class
	-rm *.tokens
	-rm *.interp
	-rm *.js
	-rm *.iml
	-rm -rf lib
	-rm -rf project
	-rm -rf target
	-rm LinguaFranca*.java
	-rm Expr*.java

example:
	java -jar $(ANTLR) Expr.g4
	javac -classpath $(ANTLR) Expr*.java
	java -classpath .:$(ANTLR) org.antlr.v4.gui.TestRig Expr prog -gui

# setup in your shell
useful:
	alias antlr4='java -jar $(ANTLR)'
	alias grun='java -classpath .:$(ANTLR) org.antlr.v4.gui.TestRig'
