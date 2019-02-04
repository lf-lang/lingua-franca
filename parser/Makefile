init:
	-mkdir lib
	cd lib; curl -O https://www.antlr.org/download/antlr-4.7.2-complete.jar

ANTLR=lib/antlr-4.7.2-complete.jar

example:
	java -jar $(ANTLR) Expr.g4
	javac -classpath $(ANTLR) Expr*.java
	java -classpath .:$(ANTLR) org.antlr.v4.gui.TestRig Expr prog -gui
