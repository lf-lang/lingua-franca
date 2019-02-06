import org.antlr.v4.runtime.*;
import org.antlr.v4.runtime.tree.*;

public class GenCode {
    public static void main(String[] args) throws Exception {
        ANTLRInputStream input = new ANTLRInputStream(System.in);
        LinguaFrancaLexer lexer = new LinguaFrancaLexer(input);
        CommonTokenStream tokens = new CommonTokenStream(lexer);
        LinguaFrancaParser parser = new LinguaFrancaParser(tokens);
        ParseTree tree = parser.sys();

        ParseTreeWalker walker = new ParseTreeWalker();
        walker.walk(new GenJS(), tree);
        System.out.println();
    }
}
