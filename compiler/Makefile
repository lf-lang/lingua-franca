# Include user makefile for local configurations
-include config.mk

ANTLR := lib/antlr-4.7.2-complete.jar
ACCESSOR_PATH ?= accessors/web/hosts/node/
APP ?= Ping
SOURCE := ../src/$(APP).lf

all: lib comp lf

comp:
	java -jar $(ANTLR) -package org.icyphy.lf LinguaFranca.g4 -o src/main/java/org/icyphy/lf
	sbt compile

lf:
	sbt "runMain org.icyphy.lf.Compiler $(SOURCE) $(APP).js"

gui:
	java -classpath .:$(ANTLR):target/lingua-franca-1.0-SNAPSHOT.jar org.antlr.v4.gui.TestRig org.icyphy.lf.LinguaFranca sys $(SOURCE) -gui

lib:
	mkdir -p lib
	cd lib && curl -O https://www.antlr.org/download/antlr-4.7.2-complete.jar

clean:
	$(RM) *.class *.tokens *.interp *.js *.iml src/main/java/org/icyphy/lf/LinguaFranca*.*
	$(RM) -r lib project target node_modules accessors

# Install accessors node host for tests
# insallting with npm did not work for me, use the SVN method
#	npm i @accessors-hosts/node
setup:
	svn co https://repo.eecs.berkeley.edu/svn-anon/projects/terraswarm/accessors/trunk/accessors

test:
	node $(ACCESSOR_PATH)nodeHostInvoke.js $(APP)
# test the testing
#	cd ../src/SimpleTest &&  node ../../compiler/$(ACCESSOR_PATH)nodeHostInvoke.js SimpleTest


# setup in your shell
useful:
	alias antlr4='java -jar $(ANTLR)'
	alias grun='java -classpath .:$(ANTLR) org.antlr.v4.gui.TestRig'
