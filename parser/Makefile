ANTLR=lib/antlr-4.7.2-complete.jar

ARG?=Source
SOURCE=../src/$(ARG).lf

all: lib comp lf

comp:
	java -jar $(ANTLR) -package org.icyphy.lf LinguaFranca.g4 -o src/main/java/org/icyphy/lf
	sbt compile

lf:
	sbt "runMain org.icyphy.lf.GenCode $(SOURCE) $(ARG).js"

gui:
	java -classpath .:$(ANTLR):target/lingua-franca-1.0-SNAPSHOT.jar org.antlr.v4.gui.TestRig org.icyphy.lf.LinguaFranca sys $(SOURCE) -gui

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
	-rm src/main/java/org/icyphy/lf/LinguaFranca*.*

# setup in your shell
useful:
	alias antlr4='java -jar $(ANTLR)'
	alias grun='java -classpath .:$(ANTLR) org.antlr.v4.gui.TestRig'
