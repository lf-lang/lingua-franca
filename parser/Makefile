ANTLR=lib/antlr-4.7.2-complete.jar
ARG?=../src/Source.lf

all: comp gui

comp:
	java -jar $(ANTLR) LinguaFranca.g4
	javac -classpath $(ANTLR) *.java

lf:
	java -classpath .:$(ANTLR) GenCode $(ARG)

gui:
	java -classpath .:$(ANTLR) org.antlr.v4.gui.TestRig LinguaFranca sys $(ARG) -gui

init:
	-mkdir lib
	cd lib; curl -O https://www.antlr.org/download/antlr-4.7.2-complete.jar

clean:
	-rm *.class
	-rm *.tokens
	-rm *.interp
	-rm -rf lib
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
