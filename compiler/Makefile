ANTLR := lib/antlr-4.7.2-complete.jar
ARG ?= Composition
SOURCE := ../src/$(ARG).lf

all: lib comp lf

comp:
	java -jar $(ANTLR) -package org.icyphy.lf LinguaFranca.g4 -o src/main/java/org/icyphy/lf
	sbt compile

lf:
	sbt "runMain org.icyphy.lf.Compiler $(SOURCE) $(ARG).js"

gui:
	java -classpath .:$(ANTLR):target/lingua-franca-1.0-SNAPSHOT.jar org.antlr.v4.gui.TestRig org.icyphy.lf.LinguaFranca sys $(SOURCE) -gui

lib:
	mkdir -p lib
	cd lib && curl -O https://www.antlr.org/download/antlr-4.7.2-complete.jar

clean:
	$(RM) *.class *.tokens *.interp *.js *.iml src/main/java/org/icyphy/lf/LinguaFranca*.*
	$(RM) -r lib project target

# setup in your shell
useful:
	alias antlr4='java -jar $(ANTLR)'
	alias grun='java -classpath .:$(ANTLR) org.antlr.v4.gui.TestRig'
