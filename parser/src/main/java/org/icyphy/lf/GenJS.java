package org.icyphy.lf;

import java.util.*;

public class GenJS extends LinguaFrancaBaseListener {

    Map<String, String> param = new HashMap<String, String>();
    Map<String, String> paramType = new HashMap<String, String>();

    Map<String, String> outType = new HashMap<String, String>();

    Set<String> trigger = new HashSet<String>();

    void println(String s) {
        System.out.println(s);
    }

    @Override
    public void enterSys(LinguaFrancaParser.SysContext ctx) {

println("// Boilerplate included for all actors.");
println("var PERIODIC = true;");
println("var ONCE = false;");
println("function schedule(trigger, time, isPeriodic) {");
println("    if (isPeriodic) {");
println("        return trigger.actor.setInterval(trigger.reaction, time);");
println("    } else {");
println("        return trigger.actor.setTimeout(trigger.reaction, time);");
println("    }");
println("}");
println("function set(port, value) {");
println("    if (!port) {");
println("        throw \"Illegal reference to undeclared output.\";");
println("    }");
println("    this.send(port, value);");
println("}");
println("set.bind(this);");

    }

    @Override public void enterParam(LinguaFrancaParser.ParamContext ctx) {
        param.put(ctx.ID().getText(), ctx.def().INTVAL().getText());
        paramType.put(ctx.ID().getText(), ctx.type().getText());
    }

    @Override public void enterOutp(LinguaFrancaParser.OutpContext ctx) {
        outType.put(ctx.ID().getText(), ctx.type().getText());
    }

    @Override public void enterCode(LinguaFrancaParser.CodeContext ctx) {
        String s = ctx.CODE().getText();
        s = s.substring(2, s.length()-2);
        println(s);
    }

    @Override public void enterTrig(LinguaFrancaParser.TrigContext ctx) {
        trigger.add(ctx.ID().getText());
    }

    @Override public void enterPre(LinguaFrancaParser.PreContext ctx) {
        println("// Code generated for this particular actor.");
        println("// Trigger data structure:");
        for (String t : trigger) {
            String s = "var " + t + " = {'actor':this, 'triggerName':'" + t +
                    "', 'reaction':reaction_" + t + "};";
            println(s);
        }
    }

    @Override public void exitPre(LinguaFrancaParser.PreContext ctx) {
        println("// Generated setup function:");
        println("exports.setup = function() {");
        for (String key : param.keySet()) {
            String def = param.get(key);
            String typ = paramType.get(key);
            String s = "    this.parameter('"+key+"', {'type':'"+typ+"', 'value':"+def+"});";
            println(s);
        }
        for (String key : outType.keySet()) {
            String typ = outType.get(key);
            String s = "    this.output('" + key + "', {'type':'" + typ + "'});";
            println(s);
        }

        println("}");
    }

    void printParam() {
        for (String key : param.keySet()) {
            String s = "    var "+key+" = this.getParameter('"+key+"');";
            println(s);
        }
    }

    @Override public void enterInit(LinguaFrancaParser.InitContext ctx) {
        println("exports.initialize = function() {");
        printParam();
    }

    @Override public void exitInit(LinguaFrancaParser.InitContext ctx) {
        println("}");
    }

    @Override public void enterReact(LinguaFrancaParser.ReactContext ctx) {

        println("// Reaction ");
        String s = "function reaction_"+ctx.ID().getText()+"() {";
        println(s);
        printParam();
        // sets is really a list
        s = ctx.sets(0).ID().getText();
        s = "    var " + s + " = '"+s+"'; // FIXME in original .js code";
        println(s);

        /*
            var period = this.getParameter('period');
            var y = 'y'; // FIXME: Too easy to cheat! User could just pass a string name
            // of an output port to set().
            // *********** From reaction, verbatim:
            n = n + 1;
            set(y, n + ": Hello World!");
            // *********** End of reaction code.
        }
        */
    }

    @Override public void exitReact(LinguaFrancaParser.ReactContext ctx) {
        println("}");
        println("// Bind the reaction function to this actor.");
        String s = "reaction_"+ctx.ID().getText()+".bind(this);";
        println(s);
    }
}
