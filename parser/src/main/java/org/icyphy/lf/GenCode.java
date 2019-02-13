package org.icyphy.lf;

import org.antlr.v4.runtime.*;
import org.antlr.v4.runtime.tree.*;
import java.io.*;

public class GenCode {
    public static void main(String[] args) throws Exception {
        String inputFile = null;
        String outputFile = null;
        if (args.length > 0) inputFile = args[0];
        if (args.length > 1) outputFile = args[1];
        InputStream is = System.in;
        if (inputFile != null) is = new FileInputStream(inputFile);
        PrintStream ps = System.out;
        if (outputFile != null) ps = new PrintStream(outputFile);
        ANTLRInputStream input = new ANTLRInputStream(is);

        LinguaFrancaLexer lexer = new LinguaFrancaLexer(input);
        CommonTokenStream tokens = new CommonTokenStream(lexer);
        LinguaFrancaParser parser = new LinguaFrancaParser(tokens);
        ParseTree tree = parser.sys();

        ParseTreeWalker walker = new ParseTreeWalker();
        walker.walk(new GenJS(ps), tree);
        ps.println();
    }
}
