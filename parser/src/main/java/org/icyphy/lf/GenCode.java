package org.icyphy.lf;

import org.antlr.v4.runtime.*;
import org.antlr.v4.runtime.tree.*;
import java.io.FileInputStream;
import java.io.InputStream;

public class GenCode {
    public static void main(String[] args) throws Exception {
        String inputFile = null;
        if (args.length > 0) inputFile = args[0];
        InputStream is = System.in;
        if (inputFile != null) is = new FileInputStream(inputFile);
        ANTLRInputStream input = new ANTLRInputStream(is);

        LinguaFrancaLexer lexer = new LinguaFrancaLexer(input);
        CommonTokenStream tokens = new CommonTokenStream(lexer);
        LinguaFrancaParser parser = new LinguaFrancaParser(tokens);
        ParseTree tree = parser.sys();

        ParseTreeWalker walker = new ParseTreeWalker();
        walker.walk(new GenJS(), tree);
        System.out.println();
    }
}
