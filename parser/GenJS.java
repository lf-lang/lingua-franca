public class GenJS extends LinguaFrancaBaseListener {
    /** Translate { to " */
    @Override
    public void enterSys(LinguaFrancaParser.SysContext ctx) {
        System.out.println("Hello Lingua Franca");
    }
}
