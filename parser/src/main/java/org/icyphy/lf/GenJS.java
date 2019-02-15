package org.icyphy.lf;

import java.io.*;
import java.util.*;

public class GenJS extends LinguaFrancaBaseListener {

    class TriggerData {
        String param;
        String type;
    }
    PrintStream ps;

    GenJS(PrintStream ps) {
        this.ps = ps;
    }

    Map<String, String> param = new HashMap<String, String>();
    Map<String, String> paramType = new HashMap<String, String>();

    Map<String, String> outType = new HashMap<String, String>();

    Map<String, TriggerData> trigger = new HashMap<String, TriggerData>();

    void println(String s) {
        ps.println(s);
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
        println("function setUnbound(port, value) {");
        println("    if (!port) {");
        println("        throw \"Illegal reference to undeclared output.\";");
        println("    }");
        println("    this.send(port, value);");
        println("}");
        println("var set = setUnbound.bind(this);");
    }

    @Override
    public void enterParam(LinguaFrancaParser.ParamContext ctx) {
        param.put(ctx.ID().getText(), ctx.def().INTVAL().getText());
        paramType.put(ctx.ID().getText(), ctx.type().getText());
    }

    @Override
    public void enterOutp(LinguaFrancaParser.OutpContext ctx) {
        outType.put(ctx.ID().getText(), ctx.type().getText());
    }

    @Override
    public void enterCode(LinguaFrancaParser.CodeContext ctx) {
        String s = ctx.CODE().getText();
        s = s.substring(2, s.length() - 2);
        println(s);
    }

    @Override
    public void enterTrig(LinguaFrancaParser.TrigContext ctx) {

        TriggerData td = new TriggerData();
        td.param = ctx.trigparam().ID().getText();
        td.type = ctx.trigtype().getText();

        trigger.put(ctx.ID().getText(), td);
    }

    @Override
    public void enterPre(LinguaFrancaParser.PreContext ctx) {
        println("// Code generated for this particular actor.");
        println("// Trigger data structure:");
        for (String key : trigger.keySet()) {
            String s = "var " + key + " = {'actor':this, 'triggerName':'" + key +
                    "', 'reaction':reaction_" + key + ".bind(this)};";
            println(s);
        }
    }

    @Override
    public void exitPre(LinguaFrancaParser.PreContext ctx) {
        println("// Generated setup function:");
        println("exports.setup = function() {");
        for (String key : param.keySet()) {
            String def = param.get(key);
            String typ = paramType.get(key);
            String s = "    this.parameter('" + key + "', {'type':'" + typ + "', 'value':" + def + "});";
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
            String s = "    var " + key + " = this.getParameter('" + key + "');";
            println(s);
        }
    }

    @Override
    public void enterInit(LinguaFrancaParser.InitContext ctx) {
        println("exports.initialize = function() {");
        printParam();
        for (String key : trigger.keySet()) {
            TriggerData td = trigger.get(key);
            String s = "    schedule(" + key + ", " + td.param + ", " + td.type + ");";
            println(s);
        }
    }

    @Override
    public void exitInit(LinguaFrancaParser.InitContext ctx) {
        println("}");
    }

    @Override
    public void enterReact(LinguaFrancaParser.ReactContext ctx) {

        println("// Reaction ");
        String s = "function reaction_" + ctx.ID().getText() + "() {";
        println(s);
        printParam();
        // sets is really a list
        s = ctx.sets(0).ID().getText();
        s = "    var " + s + " = '" + s + "'; // FIXME in original .js code";
        println(s);
    }

    @Override
    public void exitReact(LinguaFrancaParser.ReactContext ctx) {
        println("}");
    }
}
