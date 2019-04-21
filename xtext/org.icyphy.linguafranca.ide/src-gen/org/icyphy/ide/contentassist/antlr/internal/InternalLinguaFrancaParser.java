package org.icyphy.ide.contentassist.antlr.internal;

import java.io.InputStream;
import org.eclipse.xtext.*;
import org.eclipse.xtext.parser.*;
import org.eclipse.xtext.parser.impl.*;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.parser.antlr.XtextTokenStream;
import org.eclipse.xtext.parser.antlr.XtextTokenStream.HiddenTokens;
import org.eclipse.xtext.ide.editor.contentassist.antlr.internal.AbstractInternalContentAssistParser;
import org.eclipse.xtext.ide.editor.contentassist.antlr.internal.DFA;
import org.icyphy.services.LinguaFrancaGrammarAccess;



import org.antlr.runtime.*;
import java.util.Stack;
import java.util.List;
import java.util.ArrayList;

@SuppressWarnings("all")
public class InternalLinguaFrancaParser extends AbstractInternalContentAssistParser {
    public static final String[] tokenNames = new String[] {
        "<invalid>", "<EOR>", "<DOWN>", "<UP>", "RULE_ID", "RULE_NUMBER", "RULE_CODE", "RULE_STRING", "RULE_INT", "RULE_ML_COMMENT", "RULE_SL_COMMENT", "RULE_WS", "RULE_ANY_OTHER", "'input'", "'output'", "'clock'", "'target'", "';'", "'import'", "'reactor'", "'{'", "'}'", "'composite'", "':'", "'reaction'", "'('", "')'", "','", "'preamble'", "'constructor'", "'='", "'new'", "'->'", "'const'", "'.'"
    };
    public static final int RULE_STRING=7;
    public static final int RULE_SL_COMMENT=10;
    public static final int T__19=19;
    public static final int T__15=15;
    public static final int T__16=16;
    public static final int T__17=17;
    public static final int T__18=18;
    public static final int T__33=33;
    public static final int T__34=34;
    public static final int T__13=13;
    public static final int T__14=14;
    public static final int EOF=-1;
    public static final int T__30=30;
    public static final int T__31=31;
    public static final int T__32=32;
    public static final int RULE_ID=4;
    public static final int RULE_WS=11;
    public static final int RULE_ANY_OTHER=12;
    public static final int RULE_NUMBER=5;
    public static final int RULE_CODE=6;
    public static final int T__26=26;
    public static final int T__27=27;
    public static final int T__28=28;
    public static final int RULE_INT=8;
    public static final int T__29=29;
    public static final int T__22=22;
    public static final int RULE_ML_COMMENT=9;
    public static final int T__23=23;
    public static final int T__24=24;
    public static final int T__25=25;
    public static final int T__20=20;
    public static final int T__21=21;

    // delegates
    // delegators


        public InternalLinguaFrancaParser(TokenStream input) {
            this(input, new RecognizerSharedState());
        }
        public InternalLinguaFrancaParser(TokenStream input, RecognizerSharedState state) {
            super(input, state);
             
        }
        

    public String[] getTokenNames() { return InternalLinguaFrancaParser.tokenNames; }
    public String getGrammarFileName() { return "InternalLinguaFranca.g"; }


    	private LinguaFrancaGrammarAccess grammarAccess;

    	public void setGrammarAccess(LinguaFrancaGrammarAccess grammarAccess) {
    		this.grammarAccess = grammarAccess;
    	}

    	@Override
    	protected Grammar getGrammar() {
    		return grammarAccess.getGrammar();
    	}

    	@Override
    	protected String getValueForTokenName(String tokenName) {
    		return tokenName;
    	}



    // $ANTLR start "entryRuleModel"
    // InternalLinguaFranca.g:53:1: entryRuleModel : ruleModel EOF ;
    public final void entryRuleModel() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:54:1: ( ruleModel EOF )
            // InternalLinguaFranca.g:55:1: ruleModel EOF
            {
             before(grammarAccess.getModelRule()); 
            pushFollow(FOLLOW_1);
            ruleModel();

            state._fsp--;

             after(grammarAccess.getModelRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRuleModel"


    // $ANTLR start "ruleModel"
    // InternalLinguaFranca.g:62:1: ruleModel : ( ( rule__Model__Group__0 ) ) ;
    public final void ruleModel() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:66:2: ( ( ( rule__Model__Group__0 ) ) )
            // InternalLinguaFranca.g:67:2: ( ( rule__Model__Group__0 ) )
            {
            // InternalLinguaFranca.g:67:2: ( ( rule__Model__Group__0 ) )
            // InternalLinguaFranca.g:68:3: ( rule__Model__Group__0 )
            {
             before(grammarAccess.getModelAccess().getGroup()); 
            // InternalLinguaFranca.g:69:3: ( rule__Model__Group__0 )
            // InternalLinguaFranca.g:69:4: rule__Model__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Model__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getModelAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleModel"


    // $ANTLR start "entryRuleTarget"
    // InternalLinguaFranca.g:78:1: entryRuleTarget : ruleTarget EOF ;
    public final void entryRuleTarget() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:79:1: ( ruleTarget EOF )
            // InternalLinguaFranca.g:80:1: ruleTarget EOF
            {
             before(grammarAccess.getTargetRule()); 
            pushFollow(FOLLOW_1);
            ruleTarget();

            state._fsp--;

             after(grammarAccess.getTargetRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRuleTarget"


    // $ANTLR start "ruleTarget"
    // InternalLinguaFranca.g:87:1: ruleTarget : ( ( rule__Target__Group__0 ) ) ;
    public final void ruleTarget() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:91:2: ( ( ( rule__Target__Group__0 ) ) )
            // InternalLinguaFranca.g:92:2: ( ( rule__Target__Group__0 ) )
            {
            // InternalLinguaFranca.g:92:2: ( ( rule__Target__Group__0 ) )
            // InternalLinguaFranca.g:93:3: ( rule__Target__Group__0 )
            {
             before(grammarAccess.getTargetAccess().getGroup()); 
            // InternalLinguaFranca.g:94:3: ( rule__Target__Group__0 )
            // InternalLinguaFranca.g:94:4: rule__Target__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Target__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getTargetAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleTarget"


    // $ANTLR start "entryRuleImport"
    // InternalLinguaFranca.g:103:1: entryRuleImport : ruleImport EOF ;
    public final void entryRuleImport() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:104:1: ( ruleImport EOF )
            // InternalLinguaFranca.g:105:1: ruleImport EOF
            {
             before(grammarAccess.getImportRule()); 
            pushFollow(FOLLOW_1);
            ruleImport();

            state._fsp--;

             after(grammarAccess.getImportRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRuleImport"


    // $ANTLR start "ruleImport"
    // InternalLinguaFranca.g:112:1: ruleImport : ( ( rule__Import__Group__0 ) ) ;
    public final void ruleImport() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:116:2: ( ( ( rule__Import__Group__0 ) ) )
            // InternalLinguaFranca.g:117:2: ( ( rule__Import__Group__0 ) )
            {
            // InternalLinguaFranca.g:117:2: ( ( rule__Import__Group__0 ) )
            // InternalLinguaFranca.g:118:3: ( rule__Import__Group__0 )
            {
             before(grammarAccess.getImportAccess().getGroup()); 
            // InternalLinguaFranca.g:119:3: ( rule__Import__Group__0 )
            // InternalLinguaFranca.g:119:4: rule__Import__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Import__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getImportAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleImport"


    // $ANTLR start "entryRuleReactor"
    // InternalLinguaFranca.g:128:1: entryRuleReactor : ruleReactor EOF ;
    public final void entryRuleReactor() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:129:1: ( ruleReactor EOF )
            // InternalLinguaFranca.g:130:1: ruleReactor EOF
            {
             before(grammarAccess.getReactorRule()); 
            pushFollow(FOLLOW_1);
            ruleReactor();

            state._fsp--;

             after(grammarAccess.getReactorRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRuleReactor"


    // $ANTLR start "ruleReactor"
    // InternalLinguaFranca.g:137:1: ruleReactor : ( ( rule__Reactor__Group__0 ) ) ;
    public final void ruleReactor() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:141:2: ( ( ( rule__Reactor__Group__0 ) ) )
            // InternalLinguaFranca.g:142:2: ( ( rule__Reactor__Group__0 ) )
            {
            // InternalLinguaFranca.g:142:2: ( ( rule__Reactor__Group__0 ) )
            // InternalLinguaFranca.g:143:3: ( rule__Reactor__Group__0 )
            {
             before(grammarAccess.getReactorAccess().getGroup()); 
            // InternalLinguaFranca.g:144:3: ( rule__Reactor__Group__0 )
            // InternalLinguaFranca.g:144:4: rule__Reactor__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Reactor__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getReactorAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleReactor"


    // $ANTLR start "entryRuleComposite"
    // InternalLinguaFranca.g:153:1: entryRuleComposite : ruleComposite EOF ;
    public final void entryRuleComposite() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:154:1: ( ruleComposite EOF )
            // InternalLinguaFranca.g:155:1: ruleComposite EOF
            {
             before(grammarAccess.getCompositeRule()); 
            pushFollow(FOLLOW_1);
            ruleComposite();

            state._fsp--;

             after(grammarAccess.getCompositeRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRuleComposite"


    // $ANTLR start "ruleComposite"
    // InternalLinguaFranca.g:162:1: ruleComposite : ( ( rule__Composite__Group__0 ) ) ;
    public final void ruleComposite() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:166:2: ( ( ( rule__Composite__Group__0 ) ) )
            // InternalLinguaFranca.g:167:2: ( ( rule__Composite__Group__0 ) )
            {
            // InternalLinguaFranca.g:167:2: ( ( rule__Composite__Group__0 ) )
            // InternalLinguaFranca.g:168:3: ( rule__Composite__Group__0 )
            {
             before(grammarAccess.getCompositeAccess().getGroup()); 
            // InternalLinguaFranca.g:169:3: ( rule__Composite__Group__0 )
            // InternalLinguaFranca.g:169:4: rule__Composite__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Composite__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getCompositeAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleComposite"


    // $ANTLR start "entryRuleInput"
    // InternalLinguaFranca.g:178:1: entryRuleInput : ruleInput EOF ;
    public final void entryRuleInput() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:179:1: ( ruleInput EOF )
            // InternalLinguaFranca.g:180:1: ruleInput EOF
            {
             before(grammarAccess.getInputRule()); 
            pushFollow(FOLLOW_1);
            ruleInput();

            state._fsp--;

             after(grammarAccess.getInputRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRuleInput"


    // $ANTLR start "ruleInput"
    // InternalLinguaFranca.g:187:1: ruleInput : ( ( rule__Input__Group__0 ) ) ;
    public final void ruleInput() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:191:2: ( ( ( rule__Input__Group__0 ) ) )
            // InternalLinguaFranca.g:192:2: ( ( rule__Input__Group__0 ) )
            {
            // InternalLinguaFranca.g:192:2: ( ( rule__Input__Group__0 ) )
            // InternalLinguaFranca.g:193:3: ( rule__Input__Group__0 )
            {
             before(grammarAccess.getInputAccess().getGroup()); 
            // InternalLinguaFranca.g:194:3: ( rule__Input__Group__0 )
            // InternalLinguaFranca.g:194:4: rule__Input__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Input__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getInputAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleInput"


    // $ANTLR start "entryRuleOutput"
    // InternalLinguaFranca.g:203:1: entryRuleOutput : ruleOutput EOF ;
    public final void entryRuleOutput() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:204:1: ( ruleOutput EOF )
            // InternalLinguaFranca.g:205:1: ruleOutput EOF
            {
             before(grammarAccess.getOutputRule()); 
            pushFollow(FOLLOW_1);
            ruleOutput();

            state._fsp--;

             after(grammarAccess.getOutputRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRuleOutput"


    // $ANTLR start "ruleOutput"
    // InternalLinguaFranca.g:212:1: ruleOutput : ( ( rule__Output__Group__0 ) ) ;
    public final void ruleOutput() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:216:2: ( ( ( rule__Output__Group__0 ) ) )
            // InternalLinguaFranca.g:217:2: ( ( rule__Output__Group__0 ) )
            {
            // InternalLinguaFranca.g:217:2: ( ( rule__Output__Group__0 ) )
            // InternalLinguaFranca.g:218:3: ( rule__Output__Group__0 )
            {
             before(grammarAccess.getOutputAccess().getGroup()); 
            // InternalLinguaFranca.g:219:3: ( rule__Output__Group__0 )
            // InternalLinguaFranca.g:219:4: rule__Output__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Output__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getOutputAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleOutput"


    // $ANTLR start "entryRuleClock"
    // InternalLinguaFranca.g:228:1: entryRuleClock : ruleClock EOF ;
    public final void entryRuleClock() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:229:1: ( ruleClock EOF )
            // InternalLinguaFranca.g:230:1: ruleClock EOF
            {
             before(grammarAccess.getClockRule()); 
            pushFollow(FOLLOW_1);
            ruleClock();

            state._fsp--;

             after(grammarAccess.getClockRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRuleClock"


    // $ANTLR start "ruleClock"
    // InternalLinguaFranca.g:237:1: ruleClock : ( ( rule__Clock__Group__0 ) ) ;
    public final void ruleClock() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:241:2: ( ( ( rule__Clock__Group__0 ) ) )
            // InternalLinguaFranca.g:242:2: ( ( rule__Clock__Group__0 ) )
            {
            // InternalLinguaFranca.g:242:2: ( ( rule__Clock__Group__0 ) )
            // InternalLinguaFranca.g:243:3: ( rule__Clock__Group__0 )
            {
             before(grammarAccess.getClockAccess().getGroup()); 
            // InternalLinguaFranca.g:244:3: ( rule__Clock__Group__0 )
            // InternalLinguaFranca.g:244:4: rule__Clock__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Clock__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getClockAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleClock"


    // $ANTLR start "entryRuleReaction"
    // InternalLinguaFranca.g:253:1: entryRuleReaction : ruleReaction EOF ;
    public final void entryRuleReaction() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:254:1: ( ruleReaction EOF )
            // InternalLinguaFranca.g:255:1: ruleReaction EOF
            {
             before(grammarAccess.getReactionRule()); 
            pushFollow(FOLLOW_1);
            ruleReaction();

            state._fsp--;

             after(grammarAccess.getReactionRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRuleReaction"


    // $ANTLR start "ruleReaction"
    // InternalLinguaFranca.g:262:1: ruleReaction : ( ( rule__Reaction__Group__0 ) ) ;
    public final void ruleReaction() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:266:2: ( ( ( rule__Reaction__Group__0 ) ) )
            // InternalLinguaFranca.g:267:2: ( ( rule__Reaction__Group__0 ) )
            {
            // InternalLinguaFranca.g:267:2: ( ( rule__Reaction__Group__0 ) )
            // InternalLinguaFranca.g:268:3: ( rule__Reaction__Group__0 )
            {
             before(grammarAccess.getReactionAccess().getGroup()); 
            // InternalLinguaFranca.g:269:3: ( rule__Reaction__Group__0 )
            // InternalLinguaFranca.g:269:4: rule__Reaction__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Reaction__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getReactionAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleReaction"


    // $ANTLR start "entryRulePreamble"
    // InternalLinguaFranca.g:278:1: entryRulePreamble : rulePreamble EOF ;
    public final void entryRulePreamble() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:279:1: ( rulePreamble EOF )
            // InternalLinguaFranca.g:280:1: rulePreamble EOF
            {
             before(grammarAccess.getPreambleRule()); 
            pushFollow(FOLLOW_1);
            rulePreamble();

            state._fsp--;

             after(grammarAccess.getPreambleRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRulePreamble"


    // $ANTLR start "rulePreamble"
    // InternalLinguaFranca.g:287:1: rulePreamble : ( ( rule__Preamble__Group__0 ) ) ;
    public final void rulePreamble() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:291:2: ( ( ( rule__Preamble__Group__0 ) ) )
            // InternalLinguaFranca.g:292:2: ( ( rule__Preamble__Group__0 ) )
            {
            // InternalLinguaFranca.g:292:2: ( ( rule__Preamble__Group__0 ) )
            // InternalLinguaFranca.g:293:3: ( rule__Preamble__Group__0 )
            {
             before(grammarAccess.getPreambleAccess().getGroup()); 
            // InternalLinguaFranca.g:294:3: ( rule__Preamble__Group__0 )
            // InternalLinguaFranca.g:294:4: rule__Preamble__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Preamble__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getPreambleAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rulePreamble"


    // $ANTLR start "entryRuleConstructor"
    // InternalLinguaFranca.g:303:1: entryRuleConstructor : ruleConstructor EOF ;
    public final void entryRuleConstructor() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:304:1: ( ruleConstructor EOF )
            // InternalLinguaFranca.g:305:1: ruleConstructor EOF
            {
             before(grammarAccess.getConstructorRule()); 
            pushFollow(FOLLOW_1);
            ruleConstructor();

            state._fsp--;

             after(grammarAccess.getConstructorRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRuleConstructor"


    // $ANTLR start "ruleConstructor"
    // InternalLinguaFranca.g:312:1: ruleConstructor : ( ( rule__Constructor__Group__0 ) ) ;
    public final void ruleConstructor() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:316:2: ( ( ( rule__Constructor__Group__0 ) ) )
            // InternalLinguaFranca.g:317:2: ( ( rule__Constructor__Group__0 ) )
            {
            // InternalLinguaFranca.g:317:2: ( ( rule__Constructor__Group__0 ) )
            // InternalLinguaFranca.g:318:3: ( rule__Constructor__Group__0 )
            {
             before(grammarAccess.getConstructorAccess().getGroup()); 
            // InternalLinguaFranca.g:319:3: ( rule__Constructor__Group__0 )
            // InternalLinguaFranca.g:319:4: rule__Constructor__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Constructor__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getConstructorAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleConstructor"


    // $ANTLR start "entryRuleInstance"
    // InternalLinguaFranca.g:328:1: entryRuleInstance : ruleInstance EOF ;
    public final void entryRuleInstance() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:329:1: ( ruleInstance EOF )
            // InternalLinguaFranca.g:330:1: ruleInstance EOF
            {
             before(grammarAccess.getInstanceRule()); 
            pushFollow(FOLLOW_1);
            ruleInstance();

            state._fsp--;

             after(grammarAccess.getInstanceRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRuleInstance"


    // $ANTLR start "ruleInstance"
    // InternalLinguaFranca.g:337:1: ruleInstance : ( ( rule__Instance__Group__0 ) ) ;
    public final void ruleInstance() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:341:2: ( ( ( rule__Instance__Group__0 ) ) )
            // InternalLinguaFranca.g:342:2: ( ( rule__Instance__Group__0 ) )
            {
            // InternalLinguaFranca.g:342:2: ( ( rule__Instance__Group__0 ) )
            // InternalLinguaFranca.g:343:3: ( rule__Instance__Group__0 )
            {
             before(grammarAccess.getInstanceAccess().getGroup()); 
            // InternalLinguaFranca.g:344:3: ( rule__Instance__Group__0 )
            // InternalLinguaFranca.g:344:4: rule__Instance__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Instance__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getInstanceAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleInstance"


    // $ANTLR start "entryRuleConnection"
    // InternalLinguaFranca.g:353:1: entryRuleConnection : ruleConnection EOF ;
    public final void entryRuleConnection() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:354:1: ( ruleConnection EOF )
            // InternalLinguaFranca.g:355:1: ruleConnection EOF
            {
             before(grammarAccess.getConnectionRule()); 
            pushFollow(FOLLOW_1);
            ruleConnection();

            state._fsp--;

             after(grammarAccess.getConnectionRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRuleConnection"


    // $ANTLR start "ruleConnection"
    // InternalLinguaFranca.g:362:1: ruleConnection : ( ( rule__Connection__Group__0 ) ) ;
    public final void ruleConnection() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:366:2: ( ( ( rule__Connection__Group__0 ) ) )
            // InternalLinguaFranca.g:367:2: ( ( rule__Connection__Group__0 ) )
            {
            // InternalLinguaFranca.g:367:2: ( ( rule__Connection__Group__0 ) )
            // InternalLinguaFranca.g:368:3: ( rule__Connection__Group__0 )
            {
             before(grammarAccess.getConnectionAccess().getGroup()); 
            // InternalLinguaFranca.g:369:3: ( rule__Connection__Group__0 )
            // InternalLinguaFranca.g:369:4: rule__Connection__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Connection__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getConnectionAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleConnection"


    // $ANTLR start "entryRuleAssignments"
    // InternalLinguaFranca.g:378:1: entryRuleAssignments : ruleAssignments EOF ;
    public final void entryRuleAssignments() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:379:1: ( ruleAssignments EOF )
            // InternalLinguaFranca.g:380:1: ruleAssignments EOF
            {
             before(grammarAccess.getAssignmentsRule()); 
            pushFollow(FOLLOW_1);
            ruleAssignments();

            state._fsp--;

             after(grammarAccess.getAssignmentsRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRuleAssignments"


    // $ANTLR start "ruleAssignments"
    // InternalLinguaFranca.g:387:1: ruleAssignments : ( ( rule__Assignments__Group__0 ) ) ;
    public final void ruleAssignments() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:391:2: ( ( ( rule__Assignments__Group__0 ) ) )
            // InternalLinguaFranca.g:392:2: ( ( rule__Assignments__Group__0 ) )
            {
            // InternalLinguaFranca.g:392:2: ( ( rule__Assignments__Group__0 ) )
            // InternalLinguaFranca.g:393:3: ( rule__Assignments__Group__0 )
            {
             before(grammarAccess.getAssignmentsAccess().getGroup()); 
            // InternalLinguaFranca.g:394:3: ( rule__Assignments__Group__0 )
            // InternalLinguaFranca.g:394:4: rule__Assignments__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Assignments__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getAssignmentsAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleAssignments"


    // $ANTLR start "entryRuleAssignment"
    // InternalLinguaFranca.g:403:1: entryRuleAssignment : ruleAssignment EOF ;
    public final void entryRuleAssignment() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:404:1: ( ruleAssignment EOF )
            // InternalLinguaFranca.g:405:1: ruleAssignment EOF
            {
             before(grammarAccess.getAssignmentRule()); 
            pushFollow(FOLLOW_1);
            ruleAssignment();

            state._fsp--;

             after(grammarAccess.getAssignmentRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRuleAssignment"


    // $ANTLR start "ruleAssignment"
    // InternalLinguaFranca.g:412:1: ruleAssignment : ( ( rule__Assignment__Group__0 ) ) ;
    public final void ruleAssignment() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:416:2: ( ( ( rule__Assignment__Group__0 ) ) )
            // InternalLinguaFranca.g:417:2: ( ( rule__Assignment__Group__0 ) )
            {
            // InternalLinguaFranca.g:417:2: ( ( rule__Assignment__Group__0 ) )
            // InternalLinguaFranca.g:418:3: ( rule__Assignment__Group__0 )
            {
             before(grammarAccess.getAssignmentAccess().getGroup()); 
            // InternalLinguaFranca.g:419:3: ( rule__Assignment__Group__0 )
            // InternalLinguaFranca.g:419:4: rule__Assignment__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Assignment__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getAssignmentAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleAssignment"


    // $ANTLR start "entryRuleGets"
    // InternalLinguaFranca.g:428:1: entryRuleGets : ruleGets EOF ;
    public final void entryRuleGets() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:429:1: ( ruleGets EOF )
            // InternalLinguaFranca.g:430:1: ruleGets EOF
            {
             before(grammarAccess.getGetsRule()); 
            pushFollow(FOLLOW_1);
            ruleGets();

            state._fsp--;

             after(grammarAccess.getGetsRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRuleGets"


    // $ANTLR start "ruleGets"
    // InternalLinguaFranca.g:437:1: ruleGets : ( ( rule__Gets__Group__0 ) ) ;
    public final void ruleGets() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:441:2: ( ( ( rule__Gets__Group__0 ) ) )
            // InternalLinguaFranca.g:442:2: ( ( rule__Gets__Group__0 ) )
            {
            // InternalLinguaFranca.g:442:2: ( ( rule__Gets__Group__0 ) )
            // InternalLinguaFranca.g:443:3: ( rule__Gets__Group__0 )
            {
             before(grammarAccess.getGetsAccess().getGroup()); 
            // InternalLinguaFranca.g:444:3: ( rule__Gets__Group__0 )
            // InternalLinguaFranca.g:444:4: rule__Gets__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Gets__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getGetsAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleGets"


    // $ANTLR start "entryRuleParams"
    // InternalLinguaFranca.g:453:1: entryRuleParams : ruleParams EOF ;
    public final void entryRuleParams() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:454:1: ( ruleParams EOF )
            // InternalLinguaFranca.g:455:1: ruleParams EOF
            {
             before(grammarAccess.getParamsRule()); 
            pushFollow(FOLLOW_1);
            ruleParams();

            state._fsp--;

             after(grammarAccess.getParamsRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRuleParams"


    // $ANTLR start "ruleParams"
    // InternalLinguaFranca.g:462:1: ruleParams : ( ( rule__Params__Group__0 ) ) ;
    public final void ruleParams() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:466:2: ( ( ( rule__Params__Group__0 ) ) )
            // InternalLinguaFranca.g:467:2: ( ( rule__Params__Group__0 ) )
            {
            // InternalLinguaFranca.g:467:2: ( ( rule__Params__Group__0 ) )
            // InternalLinguaFranca.g:468:3: ( rule__Params__Group__0 )
            {
             before(grammarAccess.getParamsAccess().getGroup()); 
            // InternalLinguaFranca.g:469:3: ( rule__Params__Group__0 )
            // InternalLinguaFranca.g:469:4: rule__Params__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Params__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getParamsAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleParams"


    // $ANTLR start "entryRuleParam"
    // InternalLinguaFranca.g:478:1: entryRuleParam : ruleParam EOF ;
    public final void entryRuleParam() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:479:1: ( ruleParam EOF )
            // InternalLinguaFranca.g:480:1: ruleParam EOF
            {
             before(grammarAccess.getParamRule()); 
            pushFollow(FOLLOW_1);
            ruleParam();

            state._fsp--;

             after(grammarAccess.getParamRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRuleParam"


    // $ANTLR start "ruleParam"
    // InternalLinguaFranca.g:487:1: ruleParam : ( ( rule__Param__Group__0 ) ) ;
    public final void ruleParam() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:491:2: ( ( ( rule__Param__Group__0 ) ) )
            // InternalLinguaFranca.g:492:2: ( ( rule__Param__Group__0 ) )
            {
            // InternalLinguaFranca.g:492:2: ( ( rule__Param__Group__0 ) )
            // InternalLinguaFranca.g:493:3: ( rule__Param__Group__0 )
            {
             before(grammarAccess.getParamAccess().getGroup()); 
            // InternalLinguaFranca.g:494:3: ( rule__Param__Group__0 )
            // InternalLinguaFranca.g:494:4: rule__Param__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Param__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getParamAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleParam"


    // $ANTLR start "entryRulePeriod"
    // InternalLinguaFranca.g:503:1: entryRulePeriod : rulePeriod EOF ;
    public final void entryRulePeriod() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:504:1: ( rulePeriod EOF )
            // InternalLinguaFranca.g:505:1: rulePeriod EOF
            {
             before(grammarAccess.getPeriodRule()); 
            pushFollow(FOLLOW_1);
            rulePeriod();

            state._fsp--;

             after(grammarAccess.getPeriodRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRulePeriod"


    // $ANTLR start "rulePeriod"
    // InternalLinguaFranca.g:512:1: rulePeriod : ( ( rule__Period__Group__0 ) ) ;
    public final void rulePeriod() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:516:2: ( ( ( rule__Period__Group__0 ) ) )
            // InternalLinguaFranca.g:517:2: ( ( rule__Period__Group__0 ) )
            {
            // InternalLinguaFranca.g:517:2: ( ( rule__Period__Group__0 ) )
            // InternalLinguaFranca.g:518:3: ( rule__Period__Group__0 )
            {
             before(grammarAccess.getPeriodAccess().getGroup()); 
            // InternalLinguaFranca.g:519:3: ( rule__Period__Group__0 )
            // InternalLinguaFranca.g:519:4: rule__Period__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Period__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getPeriodAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rulePeriod"


    // $ANTLR start "entryRulePort"
    // InternalLinguaFranca.g:528:1: entryRulePort : rulePort EOF ;
    public final void entryRulePort() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:529:1: ( rulePort EOF )
            // InternalLinguaFranca.g:530:1: rulePort EOF
            {
             before(grammarAccess.getPortRule()); 
            pushFollow(FOLLOW_1);
            rulePort();

            state._fsp--;

             after(grammarAccess.getPortRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRulePort"


    // $ANTLR start "rulePort"
    // InternalLinguaFranca.g:537:1: rulePort : ( ( rule__Port__Alternatives ) ) ;
    public final void rulePort() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:541:2: ( ( ( rule__Port__Alternatives ) ) )
            // InternalLinguaFranca.g:542:2: ( ( rule__Port__Alternatives ) )
            {
            // InternalLinguaFranca.g:542:2: ( ( rule__Port__Alternatives ) )
            // InternalLinguaFranca.g:543:3: ( rule__Port__Alternatives )
            {
             before(grammarAccess.getPortAccess().getAlternatives()); 
            // InternalLinguaFranca.g:544:3: ( rule__Port__Alternatives )
            // InternalLinguaFranca.g:544:4: rule__Port__Alternatives
            {
            pushFollow(FOLLOW_2);
            rule__Port__Alternatives();

            state._fsp--;


            }

             after(grammarAccess.getPortAccess().getAlternatives()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rulePort"


    // $ANTLR start "entryRuleSets"
    // InternalLinguaFranca.g:553:1: entryRuleSets : ruleSets EOF ;
    public final void entryRuleSets() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:554:1: ( ruleSets EOF )
            // InternalLinguaFranca.g:555:1: ruleSets EOF
            {
             before(grammarAccess.getSetsRule()); 
            pushFollow(FOLLOW_1);
            ruleSets();

            state._fsp--;

             after(grammarAccess.getSetsRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRuleSets"


    // $ANTLR start "ruleSets"
    // InternalLinguaFranca.g:562:1: ruleSets : ( ( rule__Sets__Group__0 ) ) ;
    public final void ruleSets() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:566:2: ( ( ( rule__Sets__Group__0 ) ) )
            // InternalLinguaFranca.g:567:2: ( ( rule__Sets__Group__0 ) )
            {
            // InternalLinguaFranca.g:567:2: ( ( rule__Sets__Group__0 ) )
            // InternalLinguaFranca.g:568:3: ( rule__Sets__Group__0 )
            {
             before(grammarAccess.getSetsAccess().getGroup()); 
            // InternalLinguaFranca.g:569:3: ( rule__Sets__Group__0 )
            // InternalLinguaFranca.g:569:4: rule__Sets__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Sets__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getSetsAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleSets"


    // $ANTLR start "entryRuleType"
    // InternalLinguaFranca.g:578:1: entryRuleType : ruleType EOF ;
    public final void entryRuleType() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:579:1: ( ruleType EOF )
            // InternalLinguaFranca.g:580:1: ruleType EOF
            {
             before(grammarAccess.getTypeRule()); 
            pushFollow(FOLLOW_1);
            ruleType();

            state._fsp--;

             after(grammarAccess.getTypeRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRuleType"


    // $ANTLR start "ruleType"
    // InternalLinguaFranca.g:587:1: ruleType : ( ( rule__Type__Alternatives ) ) ;
    public final void ruleType() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:591:2: ( ( ( rule__Type__Alternatives ) ) )
            // InternalLinguaFranca.g:592:2: ( ( rule__Type__Alternatives ) )
            {
            // InternalLinguaFranca.g:592:2: ( ( rule__Type__Alternatives ) )
            // InternalLinguaFranca.g:593:3: ( rule__Type__Alternatives )
            {
             before(grammarAccess.getTypeAccess().getAlternatives()); 
            // InternalLinguaFranca.g:594:3: ( rule__Type__Alternatives )
            // InternalLinguaFranca.g:594:4: rule__Type__Alternatives
            {
            pushFollow(FOLLOW_2);
            rule__Type__Alternatives();

            state._fsp--;


            }

             after(grammarAccess.getTypeAccess().getAlternatives()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleType"


    // $ANTLR start "entryRuleValue"
    // InternalLinguaFranca.g:603:1: entryRuleValue : ruleValue EOF ;
    public final void entryRuleValue() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:604:1: ( ruleValue EOF )
            // InternalLinguaFranca.g:605:1: ruleValue EOF
            {
             before(grammarAccess.getValueRule()); 
            pushFollow(FOLLOW_1);
            ruleValue();

            state._fsp--;

             after(grammarAccess.getValueRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRuleValue"


    // $ANTLR start "ruleValue"
    // InternalLinguaFranca.g:612:1: ruleValue : ( ( rule__Value__Alternatives ) ) ;
    public final void ruleValue() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:616:2: ( ( ( rule__Value__Alternatives ) ) )
            // InternalLinguaFranca.g:617:2: ( ( rule__Value__Alternatives ) )
            {
            // InternalLinguaFranca.g:617:2: ( ( rule__Value__Alternatives ) )
            // InternalLinguaFranca.g:618:3: ( rule__Value__Alternatives )
            {
             before(grammarAccess.getValueAccess().getAlternatives()); 
            // InternalLinguaFranca.g:619:3: ( rule__Value__Alternatives )
            // InternalLinguaFranca.g:619:4: rule__Value__Alternatives
            {
            pushFollow(FOLLOW_2);
            rule__Value__Alternatives();

            state._fsp--;


            }

             after(grammarAccess.getValueAccess().getAlternatives()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleValue"


    // $ANTLR start "entryRulePath"
    // InternalLinguaFranca.g:628:1: entryRulePath : rulePath EOF ;
    public final void entryRulePath() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:629:1: ( rulePath EOF )
            // InternalLinguaFranca.g:630:1: rulePath EOF
            {
             before(grammarAccess.getPathRule()); 
            pushFollow(FOLLOW_1);
            rulePath();

            state._fsp--;

             after(grammarAccess.getPathRule()); 
            match(input,EOF,FOLLOW_2); 

            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {
        }
        return ;
    }
    // $ANTLR end "entryRulePath"


    // $ANTLR start "rulePath"
    // InternalLinguaFranca.g:637:1: rulePath : ( ( rule__Path__Group__0 ) ) ;
    public final void rulePath() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:641:2: ( ( ( rule__Path__Group__0 ) ) )
            // InternalLinguaFranca.g:642:2: ( ( rule__Path__Group__0 ) )
            {
            // InternalLinguaFranca.g:642:2: ( ( rule__Path__Group__0 ) )
            // InternalLinguaFranca.g:643:3: ( rule__Path__Group__0 )
            {
             before(grammarAccess.getPathAccess().getGroup()); 
            // InternalLinguaFranca.g:644:3: ( rule__Path__Group__0 )
            // InternalLinguaFranca.g:644:4: rule__Path__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Path__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getPathAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rulePath"


    // $ANTLR start "rule__Model__BlocksAlternatives_2_0"
    // InternalLinguaFranca.g:652:1: rule__Model__BlocksAlternatives_2_0 : ( ( ruleReactor ) | ( ruleComposite ) );
    public final void rule__Model__BlocksAlternatives_2_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:656:1: ( ( ruleReactor ) | ( ruleComposite ) )
            int alt1=2;
            int LA1_0 = input.LA(1);

            if ( (LA1_0==19) ) {
                alt1=1;
            }
            else if ( (LA1_0==22) ) {
                alt1=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 1, 0, input);

                throw nvae;
            }
            switch (alt1) {
                case 1 :
                    // InternalLinguaFranca.g:657:2: ( ruleReactor )
                    {
                    // InternalLinguaFranca.g:657:2: ( ruleReactor )
                    // InternalLinguaFranca.g:658:3: ruleReactor
                    {
                     before(grammarAccess.getModelAccess().getBlocksReactorParserRuleCall_2_0_0()); 
                    pushFollow(FOLLOW_2);
                    ruleReactor();

                    state._fsp--;

                     after(grammarAccess.getModelAccess().getBlocksReactorParserRuleCall_2_0_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:663:2: ( ruleComposite )
                    {
                    // InternalLinguaFranca.g:663:2: ( ruleComposite )
                    // InternalLinguaFranca.g:664:3: ruleComposite
                    {
                     before(grammarAccess.getModelAccess().getBlocksCompositeParserRuleCall_2_0_1()); 
                    pushFollow(FOLLOW_2);
                    ruleComposite();

                    state._fsp--;

                     after(grammarAccess.getModelAccess().getBlocksCompositeParserRuleCall_2_0_1()); 

                    }


                    }
                    break;

            }
        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Model__BlocksAlternatives_2_0"


    // $ANTLR start "rule__Input__NameAlternatives_1_0"
    // InternalLinguaFranca.g:673:1: rule__Input__NameAlternatives_1_0 : ( ( RULE_ID ) | ( 'input' ) );
    public final void rule__Input__NameAlternatives_1_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:677:1: ( ( RULE_ID ) | ( 'input' ) )
            int alt2=2;
            int LA2_0 = input.LA(1);

            if ( (LA2_0==RULE_ID) ) {
                alt2=1;
            }
            else if ( (LA2_0==13) ) {
                alt2=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 2, 0, input);

                throw nvae;
            }
            switch (alt2) {
                case 1 :
                    // InternalLinguaFranca.g:678:2: ( RULE_ID )
                    {
                    // InternalLinguaFranca.g:678:2: ( RULE_ID )
                    // InternalLinguaFranca.g:679:3: RULE_ID
                    {
                     before(grammarAccess.getInputAccess().getNameIDTerminalRuleCall_1_0_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getInputAccess().getNameIDTerminalRuleCall_1_0_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:684:2: ( 'input' )
                    {
                    // InternalLinguaFranca.g:684:2: ( 'input' )
                    // InternalLinguaFranca.g:685:3: 'input'
                    {
                     before(grammarAccess.getInputAccess().getNameInputKeyword_1_0_1()); 
                    match(input,13,FOLLOW_2); 
                     after(grammarAccess.getInputAccess().getNameInputKeyword_1_0_1()); 

                    }


                    }
                    break;

            }
        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Input__NameAlternatives_1_0"


    // $ANTLR start "rule__Output__NameAlternatives_1_0"
    // InternalLinguaFranca.g:694:1: rule__Output__NameAlternatives_1_0 : ( ( RULE_ID ) | ( 'output' ) );
    public final void rule__Output__NameAlternatives_1_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:698:1: ( ( RULE_ID ) | ( 'output' ) )
            int alt3=2;
            int LA3_0 = input.LA(1);

            if ( (LA3_0==RULE_ID) ) {
                alt3=1;
            }
            else if ( (LA3_0==14) ) {
                alt3=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 3, 0, input);

                throw nvae;
            }
            switch (alt3) {
                case 1 :
                    // InternalLinguaFranca.g:699:2: ( RULE_ID )
                    {
                    // InternalLinguaFranca.g:699:2: ( RULE_ID )
                    // InternalLinguaFranca.g:700:3: RULE_ID
                    {
                     before(grammarAccess.getOutputAccess().getNameIDTerminalRuleCall_1_0_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getOutputAccess().getNameIDTerminalRuleCall_1_0_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:705:2: ( 'output' )
                    {
                    // InternalLinguaFranca.g:705:2: ( 'output' )
                    // InternalLinguaFranca.g:706:3: 'output'
                    {
                     before(grammarAccess.getOutputAccess().getNameOutputKeyword_1_0_1()); 
                    match(input,14,FOLLOW_2); 
                     after(grammarAccess.getOutputAccess().getNameOutputKeyword_1_0_1()); 

                    }


                    }
                    break;

            }
        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Output__NameAlternatives_1_0"


    // $ANTLR start "rule__Clock__NameAlternatives_1_0"
    // InternalLinguaFranca.g:715:1: rule__Clock__NameAlternatives_1_0 : ( ( RULE_ID ) | ( 'clock' ) );
    public final void rule__Clock__NameAlternatives_1_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:719:1: ( ( RULE_ID ) | ( 'clock' ) )
            int alt4=2;
            int LA4_0 = input.LA(1);

            if ( (LA4_0==RULE_ID) ) {
                alt4=1;
            }
            else if ( (LA4_0==15) ) {
                alt4=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 4, 0, input);

                throw nvae;
            }
            switch (alt4) {
                case 1 :
                    // InternalLinguaFranca.g:720:2: ( RULE_ID )
                    {
                    // InternalLinguaFranca.g:720:2: ( RULE_ID )
                    // InternalLinguaFranca.g:721:3: RULE_ID
                    {
                     before(grammarAccess.getClockAccess().getNameIDTerminalRuleCall_1_0_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getClockAccess().getNameIDTerminalRuleCall_1_0_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:726:2: ( 'clock' )
                    {
                    // InternalLinguaFranca.g:726:2: ( 'clock' )
                    // InternalLinguaFranca.g:727:3: 'clock'
                    {
                     before(grammarAccess.getClockAccess().getNameClockKeyword_1_0_1()); 
                    match(input,15,FOLLOW_2); 
                     after(grammarAccess.getClockAccess().getNameClockKeyword_1_0_1()); 

                    }


                    }
                    break;

            }
        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Clock__NameAlternatives_1_0"


    // $ANTLR start "rule__Period__PeriodAlternatives_1_0"
    // InternalLinguaFranca.g:736:1: rule__Period__PeriodAlternatives_1_0 : ( ( RULE_ID ) | ( RULE_NUMBER ) );
    public final void rule__Period__PeriodAlternatives_1_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:740:1: ( ( RULE_ID ) | ( RULE_NUMBER ) )
            int alt5=2;
            int LA5_0 = input.LA(1);

            if ( (LA5_0==RULE_ID) ) {
                alt5=1;
            }
            else if ( (LA5_0==RULE_NUMBER) ) {
                alt5=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 5, 0, input);

                throw nvae;
            }
            switch (alt5) {
                case 1 :
                    // InternalLinguaFranca.g:741:2: ( RULE_ID )
                    {
                    // InternalLinguaFranca.g:741:2: ( RULE_ID )
                    // InternalLinguaFranca.g:742:3: RULE_ID
                    {
                     before(grammarAccess.getPeriodAccess().getPeriodIDTerminalRuleCall_1_0_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getPeriodAccess().getPeriodIDTerminalRuleCall_1_0_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:747:2: ( RULE_NUMBER )
                    {
                    // InternalLinguaFranca.g:747:2: ( RULE_NUMBER )
                    // InternalLinguaFranca.g:748:3: RULE_NUMBER
                    {
                     before(grammarAccess.getPeriodAccess().getPeriodNUMBERTerminalRuleCall_1_0_1()); 
                    match(input,RULE_NUMBER,FOLLOW_2); 
                     after(grammarAccess.getPeriodAccess().getPeriodNUMBERTerminalRuleCall_1_0_1()); 

                    }


                    }
                    break;

            }
        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__PeriodAlternatives_1_0"


    // $ANTLR start "rule__Period__OffsetAlternatives_2_1_0"
    // InternalLinguaFranca.g:757:1: rule__Period__OffsetAlternatives_2_1_0 : ( ( RULE_ID ) | ( RULE_NUMBER ) );
    public final void rule__Period__OffsetAlternatives_2_1_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:761:1: ( ( RULE_ID ) | ( RULE_NUMBER ) )
            int alt6=2;
            int LA6_0 = input.LA(1);

            if ( (LA6_0==RULE_ID) ) {
                alt6=1;
            }
            else if ( (LA6_0==RULE_NUMBER) ) {
                alt6=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 6, 0, input);

                throw nvae;
            }
            switch (alt6) {
                case 1 :
                    // InternalLinguaFranca.g:762:2: ( RULE_ID )
                    {
                    // InternalLinguaFranca.g:762:2: ( RULE_ID )
                    // InternalLinguaFranca.g:763:3: RULE_ID
                    {
                     before(grammarAccess.getPeriodAccess().getOffsetIDTerminalRuleCall_2_1_0_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getPeriodAccess().getOffsetIDTerminalRuleCall_2_1_0_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:768:2: ( RULE_NUMBER )
                    {
                    // InternalLinguaFranca.g:768:2: ( RULE_NUMBER )
                    // InternalLinguaFranca.g:769:3: RULE_NUMBER
                    {
                     before(grammarAccess.getPeriodAccess().getOffsetNUMBERTerminalRuleCall_2_1_0_1()); 
                    match(input,RULE_NUMBER,FOLLOW_2); 
                     after(grammarAccess.getPeriodAccess().getOffsetNUMBERTerminalRuleCall_2_1_0_1()); 

                    }


                    }
                    break;

            }
        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__OffsetAlternatives_2_1_0"


    // $ANTLR start "rule__Period__CountAlternatives_2_2_1_0"
    // InternalLinguaFranca.g:778:1: rule__Period__CountAlternatives_2_2_1_0 : ( ( RULE_ID ) | ( RULE_NUMBER ) );
    public final void rule__Period__CountAlternatives_2_2_1_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:782:1: ( ( RULE_ID ) | ( RULE_NUMBER ) )
            int alt7=2;
            int LA7_0 = input.LA(1);

            if ( (LA7_0==RULE_ID) ) {
                alt7=1;
            }
            else if ( (LA7_0==RULE_NUMBER) ) {
                alt7=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 7, 0, input);

                throw nvae;
            }
            switch (alt7) {
                case 1 :
                    // InternalLinguaFranca.g:783:2: ( RULE_ID )
                    {
                    // InternalLinguaFranca.g:783:2: ( RULE_ID )
                    // InternalLinguaFranca.g:784:3: RULE_ID
                    {
                     before(grammarAccess.getPeriodAccess().getCountIDTerminalRuleCall_2_2_1_0_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getPeriodAccess().getCountIDTerminalRuleCall_2_2_1_0_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:789:2: ( RULE_NUMBER )
                    {
                    // InternalLinguaFranca.g:789:2: ( RULE_NUMBER )
                    // InternalLinguaFranca.g:790:3: RULE_NUMBER
                    {
                     before(grammarAccess.getPeriodAccess().getCountNUMBERTerminalRuleCall_2_2_1_0_1()); 
                    match(input,RULE_NUMBER,FOLLOW_2); 
                     after(grammarAccess.getPeriodAccess().getCountNUMBERTerminalRuleCall_2_2_1_0_1()); 

                    }


                    }
                    break;

            }
        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__CountAlternatives_2_2_1_0"


    // $ANTLR start "rule__Port__Alternatives"
    // InternalLinguaFranca.g:799:1: rule__Port__Alternatives : ( ( RULE_ID ) | ( ( rule__Port__Group_1__0 ) ) );
    public final void rule__Port__Alternatives() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:803:1: ( ( RULE_ID ) | ( ( rule__Port__Group_1__0 ) ) )
            int alt8=2;
            int LA8_0 = input.LA(1);

            if ( (LA8_0==RULE_ID) ) {
                int LA8_1 = input.LA(2);

                if ( (LA8_1==34) ) {
                    alt8=2;
                }
                else if ( (LA8_1==EOF||LA8_1==17||LA8_1==32) ) {
                    alt8=1;
                }
                else {
                    NoViableAltException nvae =
                        new NoViableAltException("", 8, 1, input);

                    throw nvae;
                }
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 8, 0, input);

                throw nvae;
            }
            switch (alt8) {
                case 1 :
                    // InternalLinguaFranca.g:804:2: ( RULE_ID )
                    {
                    // InternalLinguaFranca.g:804:2: ( RULE_ID )
                    // InternalLinguaFranca.g:805:3: RULE_ID
                    {
                     before(grammarAccess.getPortAccess().getIDTerminalRuleCall_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getPortAccess().getIDTerminalRuleCall_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:810:2: ( ( rule__Port__Group_1__0 ) )
                    {
                    // InternalLinguaFranca.g:810:2: ( ( rule__Port__Group_1__0 ) )
                    // InternalLinguaFranca.g:811:3: ( rule__Port__Group_1__0 )
                    {
                     before(grammarAccess.getPortAccess().getGroup_1()); 
                    // InternalLinguaFranca.g:812:3: ( rule__Port__Group_1__0 )
                    // InternalLinguaFranca.g:812:4: rule__Port__Group_1__0
                    {
                    pushFollow(FOLLOW_2);
                    rule__Port__Group_1__0();

                    state._fsp--;


                    }

                     after(grammarAccess.getPortAccess().getGroup_1()); 

                    }


                    }
                    break;

            }
        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Port__Alternatives"


    // $ANTLR start "rule__Port__Alternatives_1_2"
    // InternalLinguaFranca.g:820:1: rule__Port__Alternatives_1_2 : ( ( RULE_ID ) | ( 'input' ) | ( 'output' ) );
    public final void rule__Port__Alternatives_1_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:824:1: ( ( RULE_ID ) | ( 'input' ) | ( 'output' ) )
            int alt9=3;
            switch ( input.LA(1) ) {
            case RULE_ID:
                {
                alt9=1;
                }
                break;
            case 13:
                {
                alt9=2;
                }
                break;
            case 14:
                {
                alt9=3;
                }
                break;
            default:
                NoViableAltException nvae =
                    new NoViableAltException("", 9, 0, input);

                throw nvae;
            }

            switch (alt9) {
                case 1 :
                    // InternalLinguaFranca.g:825:2: ( RULE_ID )
                    {
                    // InternalLinguaFranca.g:825:2: ( RULE_ID )
                    // InternalLinguaFranca.g:826:3: RULE_ID
                    {
                     before(grammarAccess.getPortAccess().getIDTerminalRuleCall_1_2_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getPortAccess().getIDTerminalRuleCall_1_2_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:831:2: ( 'input' )
                    {
                    // InternalLinguaFranca.g:831:2: ( 'input' )
                    // InternalLinguaFranca.g:832:3: 'input'
                    {
                     before(grammarAccess.getPortAccess().getInputKeyword_1_2_1()); 
                    match(input,13,FOLLOW_2); 
                     after(grammarAccess.getPortAccess().getInputKeyword_1_2_1()); 

                    }


                    }
                    break;
                case 3 :
                    // InternalLinguaFranca.g:837:2: ( 'output' )
                    {
                    // InternalLinguaFranca.g:837:2: ( 'output' )
                    // InternalLinguaFranca.g:838:3: 'output'
                    {
                     before(grammarAccess.getPortAccess().getOutputKeyword_1_2_2()); 
                    match(input,14,FOLLOW_2); 
                     after(grammarAccess.getPortAccess().getOutputKeyword_1_2_2()); 

                    }


                    }
                    break;

            }
        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Port__Alternatives_1_2"


    // $ANTLR start "rule__Type__Alternatives"
    // InternalLinguaFranca.g:847:1: rule__Type__Alternatives : ( ( RULE_ID ) | ( RULE_CODE ) );
    public final void rule__Type__Alternatives() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:851:1: ( ( RULE_ID ) | ( RULE_CODE ) )
            int alt10=2;
            int LA10_0 = input.LA(1);

            if ( (LA10_0==RULE_ID) ) {
                alt10=1;
            }
            else if ( (LA10_0==RULE_CODE) ) {
                alt10=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 10, 0, input);

                throw nvae;
            }
            switch (alt10) {
                case 1 :
                    // InternalLinguaFranca.g:852:2: ( RULE_ID )
                    {
                    // InternalLinguaFranca.g:852:2: ( RULE_ID )
                    // InternalLinguaFranca.g:853:3: RULE_ID
                    {
                     before(grammarAccess.getTypeAccess().getIDTerminalRuleCall_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getTypeAccess().getIDTerminalRuleCall_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:858:2: ( RULE_CODE )
                    {
                    // InternalLinguaFranca.g:858:2: ( RULE_CODE )
                    // InternalLinguaFranca.g:859:3: RULE_CODE
                    {
                     before(grammarAccess.getTypeAccess().getCODETerminalRuleCall_1()); 
                    match(input,RULE_CODE,FOLLOW_2); 
                     after(grammarAccess.getTypeAccess().getCODETerminalRuleCall_1()); 

                    }


                    }
                    break;

            }
        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Type__Alternatives"


    // $ANTLR start "rule__Value__Alternatives"
    // InternalLinguaFranca.g:868:1: rule__Value__Alternatives : ( ( RULE_ID ) | ( RULE_NUMBER ) | ( RULE_STRING ) | ( RULE_CODE ) );
    public final void rule__Value__Alternatives() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:872:1: ( ( RULE_ID ) | ( RULE_NUMBER ) | ( RULE_STRING ) | ( RULE_CODE ) )
            int alt11=4;
            switch ( input.LA(1) ) {
            case RULE_ID:
                {
                alt11=1;
                }
                break;
            case RULE_NUMBER:
                {
                alt11=2;
                }
                break;
            case RULE_STRING:
                {
                alt11=3;
                }
                break;
            case RULE_CODE:
                {
                alt11=4;
                }
                break;
            default:
                NoViableAltException nvae =
                    new NoViableAltException("", 11, 0, input);

                throw nvae;
            }

            switch (alt11) {
                case 1 :
                    // InternalLinguaFranca.g:873:2: ( RULE_ID )
                    {
                    // InternalLinguaFranca.g:873:2: ( RULE_ID )
                    // InternalLinguaFranca.g:874:3: RULE_ID
                    {
                     before(grammarAccess.getValueAccess().getIDTerminalRuleCall_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getValueAccess().getIDTerminalRuleCall_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:879:2: ( RULE_NUMBER )
                    {
                    // InternalLinguaFranca.g:879:2: ( RULE_NUMBER )
                    // InternalLinguaFranca.g:880:3: RULE_NUMBER
                    {
                     before(grammarAccess.getValueAccess().getNUMBERTerminalRuleCall_1()); 
                    match(input,RULE_NUMBER,FOLLOW_2); 
                     after(grammarAccess.getValueAccess().getNUMBERTerminalRuleCall_1()); 

                    }


                    }
                    break;
                case 3 :
                    // InternalLinguaFranca.g:885:2: ( RULE_STRING )
                    {
                    // InternalLinguaFranca.g:885:2: ( RULE_STRING )
                    // InternalLinguaFranca.g:886:3: RULE_STRING
                    {
                     before(grammarAccess.getValueAccess().getSTRINGTerminalRuleCall_2()); 
                    match(input,RULE_STRING,FOLLOW_2); 
                     after(grammarAccess.getValueAccess().getSTRINGTerminalRuleCall_2()); 

                    }


                    }
                    break;
                case 4 :
                    // InternalLinguaFranca.g:891:2: ( RULE_CODE )
                    {
                    // InternalLinguaFranca.g:891:2: ( RULE_CODE )
                    // InternalLinguaFranca.g:892:3: RULE_CODE
                    {
                     before(grammarAccess.getValueAccess().getCODETerminalRuleCall_3()); 
                    match(input,RULE_CODE,FOLLOW_2); 
                     after(grammarAccess.getValueAccess().getCODETerminalRuleCall_3()); 

                    }


                    }
                    break;

            }
        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Value__Alternatives"


    // $ANTLR start "rule__Model__Group__0"
    // InternalLinguaFranca.g:901:1: rule__Model__Group__0 : rule__Model__Group__0__Impl rule__Model__Group__1 ;
    public final void rule__Model__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:905:1: ( rule__Model__Group__0__Impl rule__Model__Group__1 )
            // InternalLinguaFranca.g:906:2: rule__Model__Group__0__Impl rule__Model__Group__1
            {
            pushFollow(FOLLOW_3);
            rule__Model__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Model__Group__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Model__Group__0"


    // $ANTLR start "rule__Model__Group__0__Impl"
    // InternalLinguaFranca.g:913:1: rule__Model__Group__0__Impl : ( ( rule__Model__TargetAssignment_0 ) ) ;
    public final void rule__Model__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:917:1: ( ( ( rule__Model__TargetAssignment_0 ) ) )
            // InternalLinguaFranca.g:918:1: ( ( rule__Model__TargetAssignment_0 ) )
            {
            // InternalLinguaFranca.g:918:1: ( ( rule__Model__TargetAssignment_0 ) )
            // InternalLinguaFranca.g:919:2: ( rule__Model__TargetAssignment_0 )
            {
             before(grammarAccess.getModelAccess().getTargetAssignment_0()); 
            // InternalLinguaFranca.g:920:2: ( rule__Model__TargetAssignment_0 )
            // InternalLinguaFranca.g:920:3: rule__Model__TargetAssignment_0
            {
            pushFollow(FOLLOW_2);
            rule__Model__TargetAssignment_0();

            state._fsp--;


            }

             after(grammarAccess.getModelAccess().getTargetAssignment_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Model__Group__0__Impl"


    // $ANTLR start "rule__Model__Group__1"
    // InternalLinguaFranca.g:928:1: rule__Model__Group__1 : rule__Model__Group__1__Impl rule__Model__Group__2 ;
    public final void rule__Model__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:932:1: ( rule__Model__Group__1__Impl rule__Model__Group__2 )
            // InternalLinguaFranca.g:933:2: rule__Model__Group__1__Impl rule__Model__Group__2
            {
            pushFollow(FOLLOW_3);
            rule__Model__Group__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Model__Group__2();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Model__Group__1"


    // $ANTLR start "rule__Model__Group__1__Impl"
    // InternalLinguaFranca.g:940:1: rule__Model__Group__1__Impl : ( ( rule__Model__ImportsAssignment_1 )* ) ;
    public final void rule__Model__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:944:1: ( ( ( rule__Model__ImportsAssignment_1 )* ) )
            // InternalLinguaFranca.g:945:1: ( ( rule__Model__ImportsAssignment_1 )* )
            {
            // InternalLinguaFranca.g:945:1: ( ( rule__Model__ImportsAssignment_1 )* )
            // InternalLinguaFranca.g:946:2: ( rule__Model__ImportsAssignment_1 )*
            {
             before(grammarAccess.getModelAccess().getImportsAssignment_1()); 
            // InternalLinguaFranca.g:947:2: ( rule__Model__ImportsAssignment_1 )*
            loop12:
            do {
                int alt12=2;
                int LA12_0 = input.LA(1);

                if ( (LA12_0==18) ) {
                    alt12=1;
                }


                switch (alt12) {
            	case 1 :
            	    // InternalLinguaFranca.g:947:3: rule__Model__ImportsAssignment_1
            	    {
            	    pushFollow(FOLLOW_4);
            	    rule__Model__ImportsAssignment_1();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop12;
                }
            } while (true);

             after(grammarAccess.getModelAccess().getImportsAssignment_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Model__Group__1__Impl"


    // $ANTLR start "rule__Model__Group__2"
    // InternalLinguaFranca.g:955:1: rule__Model__Group__2 : rule__Model__Group__2__Impl ;
    public final void rule__Model__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:959:1: ( rule__Model__Group__2__Impl )
            // InternalLinguaFranca.g:960:2: rule__Model__Group__2__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Model__Group__2__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Model__Group__2"


    // $ANTLR start "rule__Model__Group__2__Impl"
    // InternalLinguaFranca.g:966:1: rule__Model__Group__2__Impl : ( ( ( rule__Model__BlocksAssignment_2 ) ) ( ( rule__Model__BlocksAssignment_2 )* ) ) ;
    public final void rule__Model__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:970:1: ( ( ( ( rule__Model__BlocksAssignment_2 ) ) ( ( rule__Model__BlocksAssignment_2 )* ) ) )
            // InternalLinguaFranca.g:971:1: ( ( ( rule__Model__BlocksAssignment_2 ) ) ( ( rule__Model__BlocksAssignment_2 )* ) )
            {
            // InternalLinguaFranca.g:971:1: ( ( ( rule__Model__BlocksAssignment_2 ) ) ( ( rule__Model__BlocksAssignment_2 )* ) )
            // InternalLinguaFranca.g:972:2: ( ( rule__Model__BlocksAssignment_2 ) ) ( ( rule__Model__BlocksAssignment_2 )* )
            {
            // InternalLinguaFranca.g:972:2: ( ( rule__Model__BlocksAssignment_2 ) )
            // InternalLinguaFranca.g:973:3: ( rule__Model__BlocksAssignment_2 )
            {
             before(grammarAccess.getModelAccess().getBlocksAssignment_2()); 
            // InternalLinguaFranca.g:974:3: ( rule__Model__BlocksAssignment_2 )
            // InternalLinguaFranca.g:974:4: rule__Model__BlocksAssignment_2
            {
            pushFollow(FOLLOW_5);
            rule__Model__BlocksAssignment_2();

            state._fsp--;


            }

             after(grammarAccess.getModelAccess().getBlocksAssignment_2()); 

            }

            // InternalLinguaFranca.g:977:2: ( ( rule__Model__BlocksAssignment_2 )* )
            // InternalLinguaFranca.g:978:3: ( rule__Model__BlocksAssignment_2 )*
            {
             before(grammarAccess.getModelAccess().getBlocksAssignment_2()); 
            // InternalLinguaFranca.g:979:3: ( rule__Model__BlocksAssignment_2 )*
            loop13:
            do {
                int alt13=2;
                int LA13_0 = input.LA(1);

                if ( (LA13_0==19||LA13_0==22) ) {
                    alt13=1;
                }


                switch (alt13) {
            	case 1 :
            	    // InternalLinguaFranca.g:979:4: rule__Model__BlocksAssignment_2
            	    {
            	    pushFollow(FOLLOW_5);
            	    rule__Model__BlocksAssignment_2();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop13;
                }
            } while (true);

             after(grammarAccess.getModelAccess().getBlocksAssignment_2()); 

            }


            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Model__Group__2__Impl"


    // $ANTLR start "rule__Target__Group__0"
    // InternalLinguaFranca.g:989:1: rule__Target__Group__0 : rule__Target__Group__0__Impl rule__Target__Group__1 ;
    public final void rule__Target__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:993:1: ( rule__Target__Group__0__Impl rule__Target__Group__1 )
            // InternalLinguaFranca.g:994:2: rule__Target__Group__0__Impl rule__Target__Group__1
            {
            pushFollow(FOLLOW_6);
            rule__Target__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Target__Group__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Target__Group__0"


    // $ANTLR start "rule__Target__Group__0__Impl"
    // InternalLinguaFranca.g:1001:1: rule__Target__Group__0__Impl : ( 'target' ) ;
    public final void rule__Target__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1005:1: ( ( 'target' ) )
            // InternalLinguaFranca.g:1006:1: ( 'target' )
            {
            // InternalLinguaFranca.g:1006:1: ( 'target' )
            // InternalLinguaFranca.g:1007:2: 'target'
            {
             before(grammarAccess.getTargetAccess().getTargetKeyword_0()); 
            match(input,16,FOLLOW_2); 
             after(grammarAccess.getTargetAccess().getTargetKeyword_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Target__Group__0__Impl"


    // $ANTLR start "rule__Target__Group__1"
    // InternalLinguaFranca.g:1016:1: rule__Target__Group__1 : rule__Target__Group__1__Impl rule__Target__Group__2 ;
    public final void rule__Target__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1020:1: ( rule__Target__Group__1__Impl rule__Target__Group__2 )
            // InternalLinguaFranca.g:1021:2: rule__Target__Group__1__Impl rule__Target__Group__2
            {
            pushFollow(FOLLOW_7);
            rule__Target__Group__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Target__Group__2();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Target__Group__1"


    // $ANTLR start "rule__Target__Group__1__Impl"
    // InternalLinguaFranca.g:1028:1: rule__Target__Group__1__Impl : ( ( rule__Target__NameAssignment_1 ) ) ;
    public final void rule__Target__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1032:1: ( ( ( rule__Target__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:1033:1: ( ( rule__Target__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:1033:1: ( ( rule__Target__NameAssignment_1 ) )
            // InternalLinguaFranca.g:1034:2: ( rule__Target__NameAssignment_1 )
            {
             before(grammarAccess.getTargetAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:1035:2: ( rule__Target__NameAssignment_1 )
            // InternalLinguaFranca.g:1035:3: rule__Target__NameAssignment_1
            {
            pushFollow(FOLLOW_2);
            rule__Target__NameAssignment_1();

            state._fsp--;


            }

             after(grammarAccess.getTargetAccess().getNameAssignment_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Target__Group__1__Impl"


    // $ANTLR start "rule__Target__Group__2"
    // InternalLinguaFranca.g:1043:1: rule__Target__Group__2 : rule__Target__Group__2__Impl ;
    public final void rule__Target__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1047:1: ( rule__Target__Group__2__Impl )
            // InternalLinguaFranca.g:1048:2: rule__Target__Group__2__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Target__Group__2__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Target__Group__2"


    // $ANTLR start "rule__Target__Group__2__Impl"
    // InternalLinguaFranca.g:1054:1: rule__Target__Group__2__Impl : ( ';' ) ;
    public final void rule__Target__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1058:1: ( ( ';' ) )
            // InternalLinguaFranca.g:1059:1: ( ';' )
            {
            // InternalLinguaFranca.g:1059:1: ( ';' )
            // InternalLinguaFranca.g:1060:2: ';'
            {
             before(grammarAccess.getTargetAccess().getSemicolonKeyword_2()); 
            match(input,17,FOLLOW_2); 
             after(grammarAccess.getTargetAccess().getSemicolonKeyword_2()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Target__Group__2__Impl"


    // $ANTLR start "rule__Import__Group__0"
    // InternalLinguaFranca.g:1070:1: rule__Import__Group__0 : rule__Import__Group__0__Impl rule__Import__Group__1 ;
    public final void rule__Import__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1074:1: ( rule__Import__Group__0__Impl rule__Import__Group__1 )
            // InternalLinguaFranca.g:1075:2: rule__Import__Group__0__Impl rule__Import__Group__1
            {
            pushFollow(FOLLOW_6);
            rule__Import__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Import__Group__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Import__Group__0"


    // $ANTLR start "rule__Import__Group__0__Impl"
    // InternalLinguaFranca.g:1082:1: rule__Import__Group__0__Impl : ( 'import' ) ;
    public final void rule__Import__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1086:1: ( ( 'import' ) )
            // InternalLinguaFranca.g:1087:1: ( 'import' )
            {
            // InternalLinguaFranca.g:1087:1: ( 'import' )
            // InternalLinguaFranca.g:1088:2: 'import'
            {
             before(grammarAccess.getImportAccess().getImportKeyword_0()); 
            match(input,18,FOLLOW_2); 
             after(grammarAccess.getImportAccess().getImportKeyword_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Import__Group__0__Impl"


    // $ANTLR start "rule__Import__Group__1"
    // InternalLinguaFranca.g:1097:1: rule__Import__Group__1 : rule__Import__Group__1__Impl rule__Import__Group__2 ;
    public final void rule__Import__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1101:1: ( rule__Import__Group__1__Impl rule__Import__Group__2 )
            // InternalLinguaFranca.g:1102:2: rule__Import__Group__1__Impl rule__Import__Group__2
            {
            pushFollow(FOLLOW_7);
            rule__Import__Group__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Import__Group__2();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Import__Group__1"


    // $ANTLR start "rule__Import__Group__1__Impl"
    // InternalLinguaFranca.g:1109:1: rule__Import__Group__1__Impl : ( ( rule__Import__NameAssignment_1 ) ) ;
    public final void rule__Import__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1113:1: ( ( ( rule__Import__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:1114:1: ( ( rule__Import__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:1114:1: ( ( rule__Import__NameAssignment_1 ) )
            // InternalLinguaFranca.g:1115:2: ( rule__Import__NameAssignment_1 )
            {
             before(grammarAccess.getImportAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:1116:2: ( rule__Import__NameAssignment_1 )
            // InternalLinguaFranca.g:1116:3: rule__Import__NameAssignment_1
            {
            pushFollow(FOLLOW_2);
            rule__Import__NameAssignment_1();

            state._fsp--;


            }

             after(grammarAccess.getImportAccess().getNameAssignment_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Import__Group__1__Impl"


    // $ANTLR start "rule__Import__Group__2"
    // InternalLinguaFranca.g:1124:1: rule__Import__Group__2 : rule__Import__Group__2__Impl ;
    public final void rule__Import__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1128:1: ( rule__Import__Group__2__Impl )
            // InternalLinguaFranca.g:1129:2: rule__Import__Group__2__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Import__Group__2__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Import__Group__2"


    // $ANTLR start "rule__Import__Group__2__Impl"
    // InternalLinguaFranca.g:1135:1: rule__Import__Group__2__Impl : ( ';' ) ;
    public final void rule__Import__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1139:1: ( ( ';' ) )
            // InternalLinguaFranca.g:1140:1: ( ';' )
            {
            // InternalLinguaFranca.g:1140:1: ( ';' )
            // InternalLinguaFranca.g:1141:2: ';'
            {
             before(grammarAccess.getImportAccess().getSemicolonKeyword_2()); 
            match(input,17,FOLLOW_2); 
             after(grammarAccess.getImportAccess().getSemicolonKeyword_2()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Import__Group__2__Impl"


    // $ANTLR start "rule__Reactor__Group__0"
    // InternalLinguaFranca.g:1151:1: rule__Reactor__Group__0 : rule__Reactor__Group__0__Impl rule__Reactor__Group__1 ;
    public final void rule__Reactor__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1155:1: ( rule__Reactor__Group__0__Impl rule__Reactor__Group__1 )
            // InternalLinguaFranca.g:1156:2: rule__Reactor__Group__0__Impl rule__Reactor__Group__1
            {
            pushFollow(FOLLOW_6);
            rule__Reactor__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Reactor__Group__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__Group__0"


    // $ANTLR start "rule__Reactor__Group__0__Impl"
    // InternalLinguaFranca.g:1163:1: rule__Reactor__Group__0__Impl : ( 'reactor' ) ;
    public final void rule__Reactor__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1167:1: ( ( 'reactor' ) )
            // InternalLinguaFranca.g:1168:1: ( 'reactor' )
            {
            // InternalLinguaFranca.g:1168:1: ( 'reactor' )
            // InternalLinguaFranca.g:1169:2: 'reactor'
            {
             before(grammarAccess.getReactorAccess().getReactorKeyword_0()); 
            match(input,19,FOLLOW_2); 
             after(grammarAccess.getReactorAccess().getReactorKeyword_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__Group__0__Impl"


    // $ANTLR start "rule__Reactor__Group__1"
    // InternalLinguaFranca.g:1178:1: rule__Reactor__Group__1 : rule__Reactor__Group__1__Impl rule__Reactor__Group__2 ;
    public final void rule__Reactor__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1182:1: ( rule__Reactor__Group__1__Impl rule__Reactor__Group__2 )
            // InternalLinguaFranca.g:1183:2: rule__Reactor__Group__1__Impl rule__Reactor__Group__2
            {
            pushFollow(FOLLOW_8);
            rule__Reactor__Group__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Reactor__Group__2();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__Group__1"


    // $ANTLR start "rule__Reactor__Group__1__Impl"
    // InternalLinguaFranca.g:1190:1: rule__Reactor__Group__1__Impl : ( ( rule__Reactor__NameAssignment_1 ) ) ;
    public final void rule__Reactor__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1194:1: ( ( ( rule__Reactor__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:1195:1: ( ( rule__Reactor__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:1195:1: ( ( rule__Reactor__NameAssignment_1 ) )
            // InternalLinguaFranca.g:1196:2: ( rule__Reactor__NameAssignment_1 )
            {
             before(grammarAccess.getReactorAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:1197:2: ( rule__Reactor__NameAssignment_1 )
            // InternalLinguaFranca.g:1197:3: rule__Reactor__NameAssignment_1
            {
            pushFollow(FOLLOW_2);
            rule__Reactor__NameAssignment_1();

            state._fsp--;


            }

             after(grammarAccess.getReactorAccess().getNameAssignment_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__Group__1__Impl"


    // $ANTLR start "rule__Reactor__Group__2"
    // InternalLinguaFranca.g:1205:1: rule__Reactor__Group__2 : rule__Reactor__Group__2__Impl rule__Reactor__Group__3 ;
    public final void rule__Reactor__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1209:1: ( rule__Reactor__Group__2__Impl rule__Reactor__Group__3 )
            // InternalLinguaFranca.g:1210:2: rule__Reactor__Group__2__Impl rule__Reactor__Group__3
            {
            pushFollow(FOLLOW_8);
            rule__Reactor__Group__2__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Reactor__Group__3();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__Group__2"


    // $ANTLR start "rule__Reactor__Group__2__Impl"
    // InternalLinguaFranca.g:1217:1: rule__Reactor__Group__2__Impl : ( ( rule__Reactor__ParametersAssignment_2 )? ) ;
    public final void rule__Reactor__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1221:1: ( ( ( rule__Reactor__ParametersAssignment_2 )? ) )
            // InternalLinguaFranca.g:1222:1: ( ( rule__Reactor__ParametersAssignment_2 )? )
            {
            // InternalLinguaFranca.g:1222:1: ( ( rule__Reactor__ParametersAssignment_2 )? )
            // InternalLinguaFranca.g:1223:2: ( rule__Reactor__ParametersAssignment_2 )?
            {
             before(grammarAccess.getReactorAccess().getParametersAssignment_2()); 
            // InternalLinguaFranca.g:1224:2: ( rule__Reactor__ParametersAssignment_2 )?
            int alt14=2;
            int LA14_0 = input.LA(1);

            if ( (LA14_0==25) ) {
                alt14=1;
            }
            switch (alt14) {
                case 1 :
                    // InternalLinguaFranca.g:1224:3: rule__Reactor__ParametersAssignment_2
                    {
                    pushFollow(FOLLOW_2);
                    rule__Reactor__ParametersAssignment_2();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getReactorAccess().getParametersAssignment_2()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__Group__2__Impl"


    // $ANTLR start "rule__Reactor__Group__3"
    // InternalLinguaFranca.g:1232:1: rule__Reactor__Group__3 : rule__Reactor__Group__3__Impl rule__Reactor__Group__4 ;
    public final void rule__Reactor__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1236:1: ( rule__Reactor__Group__3__Impl rule__Reactor__Group__4 )
            // InternalLinguaFranca.g:1237:2: rule__Reactor__Group__3__Impl rule__Reactor__Group__4
            {
            pushFollow(FOLLOW_9);
            rule__Reactor__Group__3__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Reactor__Group__4();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__Group__3"


    // $ANTLR start "rule__Reactor__Group__3__Impl"
    // InternalLinguaFranca.g:1244:1: rule__Reactor__Group__3__Impl : ( '{' ) ;
    public final void rule__Reactor__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1248:1: ( ( '{' ) )
            // InternalLinguaFranca.g:1249:1: ( '{' )
            {
            // InternalLinguaFranca.g:1249:1: ( '{' )
            // InternalLinguaFranca.g:1250:2: '{'
            {
             before(grammarAccess.getReactorAccess().getLeftCurlyBracketKeyword_3()); 
            match(input,20,FOLLOW_2); 
             after(grammarAccess.getReactorAccess().getLeftCurlyBracketKeyword_3()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__Group__3__Impl"


    // $ANTLR start "rule__Reactor__Group__4"
    // InternalLinguaFranca.g:1259:1: rule__Reactor__Group__4 : rule__Reactor__Group__4__Impl rule__Reactor__Group__5 ;
    public final void rule__Reactor__Group__4() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1263:1: ( rule__Reactor__Group__4__Impl rule__Reactor__Group__5 )
            // InternalLinguaFranca.g:1264:2: rule__Reactor__Group__4__Impl rule__Reactor__Group__5
            {
            pushFollow(FOLLOW_9);
            rule__Reactor__Group__4__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Reactor__Group__5();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__Group__4"


    // $ANTLR start "rule__Reactor__Group__4__Impl"
    // InternalLinguaFranca.g:1271:1: rule__Reactor__Group__4__Impl : ( ( rule__Reactor__InputsAssignment_4 )* ) ;
    public final void rule__Reactor__Group__4__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1275:1: ( ( ( rule__Reactor__InputsAssignment_4 )* ) )
            // InternalLinguaFranca.g:1276:1: ( ( rule__Reactor__InputsAssignment_4 )* )
            {
            // InternalLinguaFranca.g:1276:1: ( ( rule__Reactor__InputsAssignment_4 )* )
            // InternalLinguaFranca.g:1277:2: ( rule__Reactor__InputsAssignment_4 )*
            {
             before(grammarAccess.getReactorAccess().getInputsAssignment_4()); 
            // InternalLinguaFranca.g:1278:2: ( rule__Reactor__InputsAssignment_4 )*
            loop15:
            do {
                int alt15=2;
                int LA15_0 = input.LA(1);

                if ( (LA15_0==13) ) {
                    alt15=1;
                }


                switch (alt15) {
            	case 1 :
            	    // InternalLinguaFranca.g:1278:3: rule__Reactor__InputsAssignment_4
            	    {
            	    pushFollow(FOLLOW_10);
            	    rule__Reactor__InputsAssignment_4();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop15;
                }
            } while (true);

             after(grammarAccess.getReactorAccess().getInputsAssignment_4()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__Group__4__Impl"


    // $ANTLR start "rule__Reactor__Group__5"
    // InternalLinguaFranca.g:1286:1: rule__Reactor__Group__5 : rule__Reactor__Group__5__Impl rule__Reactor__Group__6 ;
    public final void rule__Reactor__Group__5() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1290:1: ( rule__Reactor__Group__5__Impl rule__Reactor__Group__6 )
            // InternalLinguaFranca.g:1291:2: rule__Reactor__Group__5__Impl rule__Reactor__Group__6
            {
            pushFollow(FOLLOW_9);
            rule__Reactor__Group__5__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Reactor__Group__6();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__Group__5"


    // $ANTLR start "rule__Reactor__Group__5__Impl"
    // InternalLinguaFranca.g:1298:1: rule__Reactor__Group__5__Impl : ( ( rule__Reactor__OutputsAssignment_5 )* ) ;
    public final void rule__Reactor__Group__5__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1302:1: ( ( ( rule__Reactor__OutputsAssignment_5 )* ) )
            // InternalLinguaFranca.g:1303:1: ( ( rule__Reactor__OutputsAssignment_5 )* )
            {
            // InternalLinguaFranca.g:1303:1: ( ( rule__Reactor__OutputsAssignment_5 )* )
            // InternalLinguaFranca.g:1304:2: ( rule__Reactor__OutputsAssignment_5 )*
            {
             before(grammarAccess.getReactorAccess().getOutputsAssignment_5()); 
            // InternalLinguaFranca.g:1305:2: ( rule__Reactor__OutputsAssignment_5 )*
            loop16:
            do {
                int alt16=2;
                int LA16_0 = input.LA(1);

                if ( (LA16_0==14) ) {
                    alt16=1;
                }


                switch (alt16) {
            	case 1 :
            	    // InternalLinguaFranca.g:1305:3: rule__Reactor__OutputsAssignment_5
            	    {
            	    pushFollow(FOLLOW_11);
            	    rule__Reactor__OutputsAssignment_5();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop16;
                }
            } while (true);

             after(grammarAccess.getReactorAccess().getOutputsAssignment_5()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__Group__5__Impl"


    // $ANTLR start "rule__Reactor__Group__6"
    // InternalLinguaFranca.g:1313:1: rule__Reactor__Group__6 : rule__Reactor__Group__6__Impl rule__Reactor__Group__7 ;
    public final void rule__Reactor__Group__6() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1317:1: ( rule__Reactor__Group__6__Impl rule__Reactor__Group__7 )
            // InternalLinguaFranca.g:1318:2: rule__Reactor__Group__6__Impl rule__Reactor__Group__7
            {
            pushFollow(FOLLOW_9);
            rule__Reactor__Group__6__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Reactor__Group__7();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__Group__6"


    // $ANTLR start "rule__Reactor__Group__6__Impl"
    // InternalLinguaFranca.g:1325:1: rule__Reactor__Group__6__Impl : ( ( rule__Reactor__ClocksAssignment_6 )* ) ;
    public final void rule__Reactor__Group__6__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1329:1: ( ( ( rule__Reactor__ClocksAssignment_6 )* ) )
            // InternalLinguaFranca.g:1330:1: ( ( rule__Reactor__ClocksAssignment_6 )* )
            {
            // InternalLinguaFranca.g:1330:1: ( ( rule__Reactor__ClocksAssignment_6 )* )
            // InternalLinguaFranca.g:1331:2: ( rule__Reactor__ClocksAssignment_6 )*
            {
             before(grammarAccess.getReactorAccess().getClocksAssignment_6()); 
            // InternalLinguaFranca.g:1332:2: ( rule__Reactor__ClocksAssignment_6 )*
            loop17:
            do {
                int alt17=2;
                int LA17_0 = input.LA(1);

                if ( (LA17_0==15) ) {
                    alt17=1;
                }


                switch (alt17) {
            	case 1 :
            	    // InternalLinguaFranca.g:1332:3: rule__Reactor__ClocksAssignment_6
            	    {
            	    pushFollow(FOLLOW_12);
            	    rule__Reactor__ClocksAssignment_6();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop17;
                }
            } while (true);

             after(grammarAccess.getReactorAccess().getClocksAssignment_6()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__Group__6__Impl"


    // $ANTLR start "rule__Reactor__Group__7"
    // InternalLinguaFranca.g:1340:1: rule__Reactor__Group__7 : rule__Reactor__Group__7__Impl rule__Reactor__Group__8 ;
    public final void rule__Reactor__Group__7() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1344:1: ( rule__Reactor__Group__7__Impl rule__Reactor__Group__8 )
            // InternalLinguaFranca.g:1345:2: rule__Reactor__Group__7__Impl rule__Reactor__Group__8
            {
            pushFollow(FOLLOW_9);
            rule__Reactor__Group__7__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Reactor__Group__8();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__Group__7"


    // $ANTLR start "rule__Reactor__Group__7__Impl"
    // InternalLinguaFranca.g:1352:1: rule__Reactor__Group__7__Impl : ( ( rule__Reactor__PreambleAssignment_7 )? ) ;
    public final void rule__Reactor__Group__7__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1356:1: ( ( ( rule__Reactor__PreambleAssignment_7 )? ) )
            // InternalLinguaFranca.g:1357:1: ( ( rule__Reactor__PreambleAssignment_7 )? )
            {
            // InternalLinguaFranca.g:1357:1: ( ( rule__Reactor__PreambleAssignment_7 )? )
            // InternalLinguaFranca.g:1358:2: ( rule__Reactor__PreambleAssignment_7 )?
            {
             before(grammarAccess.getReactorAccess().getPreambleAssignment_7()); 
            // InternalLinguaFranca.g:1359:2: ( rule__Reactor__PreambleAssignment_7 )?
            int alt18=2;
            int LA18_0 = input.LA(1);

            if ( (LA18_0==28) ) {
                alt18=1;
            }
            switch (alt18) {
                case 1 :
                    // InternalLinguaFranca.g:1359:3: rule__Reactor__PreambleAssignment_7
                    {
                    pushFollow(FOLLOW_2);
                    rule__Reactor__PreambleAssignment_7();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getReactorAccess().getPreambleAssignment_7()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__Group__7__Impl"


    // $ANTLR start "rule__Reactor__Group__8"
    // InternalLinguaFranca.g:1367:1: rule__Reactor__Group__8 : rule__Reactor__Group__8__Impl rule__Reactor__Group__9 ;
    public final void rule__Reactor__Group__8() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1371:1: ( rule__Reactor__Group__8__Impl rule__Reactor__Group__9 )
            // InternalLinguaFranca.g:1372:2: rule__Reactor__Group__8__Impl rule__Reactor__Group__9
            {
            pushFollow(FOLLOW_9);
            rule__Reactor__Group__8__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Reactor__Group__9();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__Group__8"


    // $ANTLR start "rule__Reactor__Group__8__Impl"
    // InternalLinguaFranca.g:1379:1: rule__Reactor__Group__8__Impl : ( ( rule__Reactor__ConstructorAssignment_8 )? ) ;
    public final void rule__Reactor__Group__8__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1383:1: ( ( ( rule__Reactor__ConstructorAssignment_8 )? ) )
            // InternalLinguaFranca.g:1384:1: ( ( rule__Reactor__ConstructorAssignment_8 )? )
            {
            // InternalLinguaFranca.g:1384:1: ( ( rule__Reactor__ConstructorAssignment_8 )? )
            // InternalLinguaFranca.g:1385:2: ( rule__Reactor__ConstructorAssignment_8 )?
            {
             before(grammarAccess.getReactorAccess().getConstructorAssignment_8()); 
            // InternalLinguaFranca.g:1386:2: ( rule__Reactor__ConstructorAssignment_8 )?
            int alt19=2;
            int LA19_0 = input.LA(1);

            if ( (LA19_0==29) ) {
                alt19=1;
            }
            switch (alt19) {
                case 1 :
                    // InternalLinguaFranca.g:1386:3: rule__Reactor__ConstructorAssignment_8
                    {
                    pushFollow(FOLLOW_2);
                    rule__Reactor__ConstructorAssignment_8();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getReactorAccess().getConstructorAssignment_8()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__Group__8__Impl"


    // $ANTLR start "rule__Reactor__Group__9"
    // InternalLinguaFranca.g:1394:1: rule__Reactor__Group__9 : rule__Reactor__Group__9__Impl rule__Reactor__Group__10 ;
    public final void rule__Reactor__Group__9() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1398:1: ( rule__Reactor__Group__9__Impl rule__Reactor__Group__10 )
            // InternalLinguaFranca.g:1399:2: rule__Reactor__Group__9__Impl rule__Reactor__Group__10
            {
            pushFollow(FOLLOW_9);
            rule__Reactor__Group__9__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Reactor__Group__10();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__Group__9"


    // $ANTLR start "rule__Reactor__Group__9__Impl"
    // InternalLinguaFranca.g:1406:1: rule__Reactor__Group__9__Impl : ( ( rule__Reactor__ReactionsAssignment_9 )* ) ;
    public final void rule__Reactor__Group__9__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1410:1: ( ( ( rule__Reactor__ReactionsAssignment_9 )* ) )
            // InternalLinguaFranca.g:1411:1: ( ( rule__Reactor__ReactionsAssignment_9 )* )
            {
            // InternalLinguaFranca.g:1411:1: ( ( rule__Reactor__ReactionsAssignment_9 )* )
            // InternalLinguaFranca.g:1412:2: ( rule__Reactor__ReactionsAssignment_9 )*
            {
             before(grammarAccess.getReactorAccess().getReactionsAssignment_9()); 
            // InternalLinguaFranca.g:1413:2: ( rule__Reactor__ReactionsAssignment_9 )*
            loop20:
            do {
                int alt20=2;
                int LA20_0 = input.LA(1);

                if ( (LA20_0==24) ) {
                    alt20=1;
                }


                switch (alt20) {
            	case 1 :
            	    // InternalLinguaFranca.g:1413:3: rule__Reactor__ReactionsAssignment_9
            	    {
            	    pushFollow(FOLLOW_13);
            	    rule__Reactor__ReactionsAssignment_9();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop20;
                }
            } while (true);

             after(grammarAccess.getReactorAccess().getReactionsAssignment_9()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__Group__9__Impl"


    // $ANTLR start "rule__Reactor__Group__10"
    // InternalLinguaFranca.g:1421:1: rule__Reactor__Group__10 : rule__Reactor__Group__10__Impl ;
    public final void rule__Reactor__Group__10() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1425:1: ( rule__Reactor__Group__10__Impl )
            // InternalLinguaFranca.g:1426:2: rule__Reactor__Group__10__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Reactor__Group__10__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__Group__10"


    // $ANTLR start "rule__Reactor__Group__10__Impl"
    // InternalLinguaFranca.g:1432:1: rule__Reactor__Group__10__Impl : ( '}' ) ;
    public final void rule__Reactor__Group__10__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1436:1: ( ( '}' ) )
            // InternalLinguaFranca.g:1437:1: ( '}' )
            {
            // InternalLinguaFranca.g:1437:1: ( '}' )
            // InternalLinguaFranca.g:1438:2: '}'
            {
             before(grammarAccess.getReactorAccess().getRightCurlyBracketKeyword_10()); 
            match(input,21,FOLLOW_2); 
             after(grammarAccess.getReactorAccess().getRightCurlyBracketKeyword_10()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__Group__10__Impl"


    // $ANTLR start "rule__Composite__Group__0"
    // InternalLinguaFranca.g:1448:1: rule__Composite__Group__0 : rule__Composite__Group__0__Impl rule__Composite__Group__1 ;
    public final void rule__Composite__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1452:1: ( rule__Composite__Group__0__Impl rule__Composite__Group__1 )
            // InternalLinguaFranca.g:1453:2: rule__Composite__Group__0__Impl rule__Composite__Group__1
            {
            pushFollow(FOLLOW_6);
            rule__Composite__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Composite__Group__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__0"


    // $ANTLR start "rule__Composite__Group__0__Impl"
    // InternalLinguaFranca.g:1460:1: rule__Composite__Group__0__Impl : ( 'composite' ) ;
    public final void rule__Composite__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1464:1: ( ( 'composite' ) )
            // InternalLinguaFranca.g:1465:1: ( 'composite' )
            {
            // InternalLinguaFranca.g:1465:1: ( 'composite' )
            // InternalLinguaFranca.g:1466:2: 'composite'
            {
             before(grammarAccess.getCompositeAccess().getCompositeKeyword_0()); 
            match(input,22,FOLLOW_2); 
             after(grammarAccess.getCompositeAccess().getCompositeKeyword_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__0__Impl"


    // $ANTLR start "rule__Composite__Group__1"
    // InternalLinguaFranca.g:1475:1: rule__Composite__Group__1 : rule__Composite__Group__1__Impl rule__Composite__Group__2 ;
    public final void rule__Composite__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1479:1: ( rule__Composite__Group__1__Impl rule__Composite__Group__2 )
            // InternalLinguaFranca.g:1480:2: rule__Composite__Group__1__Impl rule__Composite__Group__2
            {
            pushFollow(FOLLOW_8);
            rule__Composite__Group__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Composite__Group__2();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__1"


    // $ANTLR start "rule__Composite__Group__1__Impl"
    // InternalLinguaFranca.g:1487:1: rule__Composite__Group__1__Impl : ( ( rule__Composite__NameAssignment_1 ) ) ;
    public final void rule__Composite__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1491:1: ( ( ( rule__Composite__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:1492:1: ( ( rule__Composite__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:1492:1: ( ( rule__Composite__NameAssignment_1 ) )
            // InternalLinguaFranca.g:1493:2: ( rule__Composite__NameAssignment_1 )
            {
             before(grammarAccess.getCompositeAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:1494:2: ( rule__Composite__NameAssignment_1 )
            // InternalLinguaFranca.g:1494:3: rule__Composite__NameAssignment_1
            {
            pushFollow(FOLLOW_2);
            rule__Composite__NameAssignment_1();

            state._fsp--;


            }

             after(grammarAccess.getCompositeAccess().getNameAssignment_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__1__Impl"


    // $ANTLR start "rule__Composite__Group__2"
    // InternalLinguaFranca.g:1502:1: rule__Composite__Group__2 : rule__Composite__Group__2__Impl rule__Composite__Group__3 ;
    public final void rule__Composite__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1506:1: ( rule__Composite__Group__2__Impl rule__Composite__Group__3 )
            // InternalLinguaFranca.g:1507:2: rule__Composite__Group__2__Impl rule__Composite__Group__3
            {
            pushFollow(FOLLOW_8);
            rule__Composite__Group__2__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Composite__Group__3();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__2"


    // $ANTLR start "rule__Composite__Group__2__Impl"
    // InternalLinguaFranca.g:1514:1: rule__Composite__Group__2__Impl : ( ( rule__Composite__ParametersAssignment_2 )? ) ;
    public final void rule__Composite__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1518:1: ( ( ( rule__Composite__ParametersAssignment_2 )? ) )
            // InternalLinguaFranca.g:1519:1: ( ( rule__Composite__ParametersAssignment_2 )? )
            {
            // InternalLinguaFranca.g:1519:1: ( ( rule__Composite__ParametersAssignment_2 )? )
            // InternalLinguaFranca.g:1520:2: ( rule__Composite__ParametersAssignment_2 )?
            {
             before(grammarAccess.getCompositeAccess().getParametersAssignment_2()); 
            // InternalLinguaFranca.g:1521:2: ( rule__Composite__ParametersAssignment_2 )?
            int alt21=2;
            int LA21_0 = input.LA(1);

            if ( (LA21_0==25) ) {
                alt21=1;
            }
            switch (alt21) {
                case 1 :
                    // InternalLinguaFranca.g:1521:3: rule__Composite__ParametersAssignment_2
                    {
                    pushFollow(FOLLOW_2);
                    rule__Composite__ParametersAssignment_2();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getCompositeAccess().getParametersAssignment_2()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__2__Impl"


    // $ANTLR start "rule__Composite__Group__3"
    // InternalLinguaFranca.g:1529:1: rule__Composite__Group__3 : rule__Composite__Group__3__Impl rule__Composite__Group__4 ;
    public final void rule__Composite__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1533:1: ( rule__Composite__Group__3__Impl rule__Composite__Group__4 )
            // InternalLinguaFranca.g:1534:2: rule__Composite__Group__3__Impl rule__Composite__Group__4
            {
            pushFollow(FOLLOW_14);
            rule__Composite__Group__3__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Composite__Group__4();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__3"


    // $ANTLR start "rule__Composite__Group__3__Impl"
    // InternalLinguaFranca.g:1541:1: rule__Composite__Group__3__Impl : ( '{' ) ;
    public final void rule__Composite__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1545:1: ( ( '{' ) )
            // InternalLinguaFranca.g:1546:1: ( '{' )
            {
            // InternalLinguaFranca.g:1546:1: ( '{' )
            // InternalLinguaFranca.g:1547:2: '{'
            {
             before(grammarAccess.getCompositeAccess().getLeftCurlyBracketKeyword_3()); 
            match(input,20,FOLLOW_2); 
             after(grammarAccess.getCompositeAccess().getLeftCurlyBracketKeyword_3()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__3__Impl"


    // $ANTLR start "rule__Composite__Group__4"
    // InternalLinguaFranca.g:1556:1: rule__Composite__Group__4 : rule__Composite__Group__4__Impl rule__Composite__Group__5 ;
    public final void rule__Composite__Group__4() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1560:1: ( rule__Composite__Group__4__Impl rule__Composite__Group__5 )
            // InternalLinguaFranca.g:1561:2: rule__Composite__Group__4__Impl rule__Composite__Group__5
            {
            pushFollow(FOLLOW_14);
            rule__Composite__Group__4__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Composite__Group__5();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__4"


    // $ANTLR start "rule__Composite__Group__4__Impl"
    // InternalLinguaFranca.g:1568:1: rule__Composite__Group__4__Impl : ( ( rule__Composite__InputsAssignment_4 )* ) ;
    public final void rule__Composite__Group__4__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1572:1: ( ( ( rule__Composite__InputsAssignment_4 )* ) )
            // InternalLinguaFranca.g:1573:1: ( ( rule__Composite__InputsAssignment_4 )* )
            {
            // InternalLinguaFranca.g:1573:1: ( ( rule__Composite__InputsAssignment_4 )* )
            // InternalLinguaFranca.g:1574:2: ( rule__Composite__InputsAssignment_4 )*
            {
             before(grammarAccess.getCompositeAccess().getInputsAssignment_4()); 
            // InternalLinguaFranca.g:1575:2: ( rule__Composite__InputsAssignment_4 )*
            loop22:
            do {
                int alt22=2;
                int LA22_0 = input.LA(1);

                if ( (LA22_0==13) ) {
                    alt22=1;
                }


                switch (alt22) {
            	case 1 :
            	    // InternalLinguaFranca.g:1575:3: rule__Composite__InputsAssignment_4
            	    {
            	    pushFollow(FOLLOW_10);
            	    rule__Composite__InputsAssignment_4();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop22;
                }
            } while (true);

             after(grammarAccess.getCompositeAccess().getInputsAssignment_4()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__4__Impl"


    // $ANTLR start "rule__Composite__Group__5"
    // InternalLinguaFranca.g:1583:1: rule__Composite__Group__5 : rule__Composite__Group__5__Impl rule__Composite__Group__6 ;
    public final void rule__Composite__Group__5() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1587:1: ( rule__Composite__Group__5__Impl rule__Composite__Group__6 )
            // InternalLinguaFranca.g:1588:2: rule__Composite__Group__5__Impl rule__Composite__Group__6
            {
            pushFollow(FOLLOW_14);
            rule__Composite__Group__5__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Composite__Group__6();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__5"


    // $ANTLR start "rule__Composite__Group__5__Impl"
    // InternalLinguaFranca.g:1595:1: rule__Composite__Group__5__Impl : ( ( rule__Composite__OutputsAssignment_5 )* ) ;
    public final void rule__Composite__Group__5__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1599:1: ( ( ( rule__Composite__OutputsAssignment_5 )* ) )
            // InternalLinguaFranca.g:1600:1: ( ( rule__Composite__OutputsAssignment_5 )* )
            {
            // InternalLinguaFranca.g:1600:1: ( ( rule__Composite__OutputsAssignment_5 )* )
            // InternalLinguaFranca.g:1601:2: ( rule__Composite__OutputsAssignment_5 )*
            {
             before(grammarAccess.getCompositeAccess().getOutputsAssignment_5()); 
            // InternalLinguaFranca.g:1602:2: ( rule__Composite__OutputsAssignment_5 )*
            loop23:
            do {
                int alt23=2;
                int LA23_0 = input.LA(1);

                if ( (LA23_0==14) ) {
                    alt23=1;
                }


                switch (alt23) {
            	case 1 :
            	    // InternalLinguaFranca.g:1602:3: rule__Composite__OutputsAssignment_5
            	    {
            	    pushFollow(FOLLOW_11);
            	    rule__Composite__OutputsAssignment_5();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop23;
                }
            } while (true);

             after(grammarAccess.getCompositeAccess().getOutputsAssignment_5()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__5__Impl"


    // $ANTLR start "rule__Composite__Group__6"
    // InternalLinguaFranca.g:1610:1: rule__Composite__Group__6 : rule__Composite__Group__6__Impl rule__Composite__Group__7 ;
    public final void rule__Composite__Group__6() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1614:1: ( rule__Composite__Group__6__Impl rule__Composite__Group__7 )
            // InternalLinguaFranca.g:1615:2: rule__Composite__Group__6__Impl rule__Composite__Group__7
            {
            pushFollow(FOLLOW_14);
            rule__Composite__Group__6__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Composite__Group__7();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__6"


    // $ANTLR start "rule__Composite__Group__6__Impl"
    // InternalLinguaFranca.g:1622:1: rule__Composite__Group__6__Impl : ( ( rule__Composite__ClocksAssignment_6 )* ) ;
    public final void rule__Composite__Group__6__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1626:1: ( ( ( rule__Composite__ClocksAssignment_6 )* ) )
            // InternalLinguaFranca.g:1627:1: ( ( rule__Composite__ClocksAssignment_6 )* )
            {
            // InternalLinguaFranca.g:1627:1: ( ( rule__Composite__ClocksAssignment_6 )* )
            // InternalLinguaFranca.g:1628:2: ( rule__Composite__ClocksAssignment_6 )*
            {
             before(grammarAccess.getCompositeAccess().getClocksAssignment_6()); 
            // InternalLinguaFranca.g:1629:2: ( rule__Composite__ClocksAssignment_6 )*
            loop24:
            do {
                int alt24=2;
                int LA24_0 = input.LA(1);

                if ( (LA24_0==15) ) {
                    alt24=1;
                }


                switch (alt24) {
            	case 1 :
            	    // InternalLinguaFranca.g:1629:3: rule__Composite__ClocksAssignment_6
            	    {
            	    pushFollow(FOLLOW_12);
            	    rule__Composite__ClocksAssignment_6();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop24;
                }
            } while (true);

             after(grammarAccess.getCompositeAccess().getClocksAssignment_6()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__6__Impl"


    // $ANTLR start "rule__Composite__Group__7"
    // InternalLinguaFranca.g:1637:1: rule__Composite__Group__7 : rule__Composite__Group__7__Impl rule__Composite__Group__8 ;
    public final void rule__Composite__Group__7() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1641:1: ( rule__Composite__Group__7__Impl rule__Composite__Group__8 )
            // InternalLinguaFranca.g:1642:2: rule__Composite__Group__7__Impl rule__Composite__Group__8
            {
            pushFollow(FOLLOW_14);
            rule__Composite__Group__7__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Composite__Group__8();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__7"


    // $ANTLR start "rule__Composite__Group__7__Impl"
    // InternalLinguaFranca.g:1649:1: rule__Composite__Group__7__Impl : ( ( rule__Composite__PreambleAssignment_7 )? ) ;
    public final void rule__Composite__Group__7__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1653:1: ( ( ( rule__Composite__PreambleAssignment_7 )? ) )
            // InternalLinguaFranca.g:1654:1: ( ( rule__Composite__PreambleAssignment_7 )? )
            {
            // InternalLinguaFranca.g:1654:1: ( ( rule__Composite__PreambleAssignment_7 )? )
            // InternalLinguaFranca.g:1655:2: ( rule__Composite__PreambleAssignment_7 )?
            {
             before(grammarAccess.getCompositeAccess().getPreambleAssignment_7()); 
            // InternalLinguaFranca.g:1656:2: ( rule__Composite__PreambleAssignment_7 )?
            int alt25=2;
            int LA25_0 = input.LA(1);

            if ( (LA25_0==28) ) {
                alt25=1;
            }
            switch (alt25) {
                case 1 :
                    // InternalLinguaFranca.g:1656:3: rule__Composite__PreambleAssignment_7
                    {
                    pushFollow(FOLLOW_2);
                    rule__Composite__PreambleAssignment_7();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getCompositeAccess().getPreambleAssignment_7()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__7__Impl"


    // $ANTLR start "rule__Composite__Group__8"
    // InternalLinguaFranca.g:1664:1: rule__Composite__Group__8 : rule__Composite__Group__8__Impl rule__Composite__Group__9 ;
    public final void rule__Composite__Group__8() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1668:1: ( rule__Composite__Group__8__Impl rule__Composite__Group__9 )
            // InternalLinguaFranca.g:1669:2: rule__Composite__Group__8__Impl rule__Composite__Group__9
            {
            pushFollow(FOLLOW_14);
            rule__Composite__Group__8__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Composite__Group__9();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__8"


    // $ANTLR start "rule__Composite__Group__8__Impl"
    // InternalLinguaFranca.g:1676:1: rule__Composite__Group__8__Impl : ( ( rule__Composite__ConstructorAssignment_8 )? ) ;
    public final void rule__Composite__Group__8__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1680:1: ( ( ( rule__Composite__ConstructorAssignment_8 )? ) )
            // InternalLinguaFranca.g:1681:1: ( ( rule__Composite__ConstructorAssignment_8 )? )
            {
            // InternalLinguaFranca.g:1681:1: ( ( rule__Composite__ConstructorAssignment_8 )? )
            // InternalLinguaFranca.g:1682:2: ( rule__Composite__ConstructorAssignment_8 )?
            {
             before(grammarAccess.getCompositeAccess().getConstructorAssignment_8()); 
            // InternalLinguaFranca.g:1683:2: ( rule__Composite__ConstructorAssignment_8 )?
            int alt26=2;
            int LA26_0 = input.LA(1);

            if ( (LA26_0==29) ) {
                alt26=1;
            }
            switch (alt26) {
                case 1 :
                    // InternalLinguaFranca.g:1683:3: rule__Composite__ConstructorAssignment_8
                    {
                    pushFollow(FOLLOW_2);
                    rule__Composite__ConstructorAssignment_8();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getCompositeAccess().getConstructorAssignment_8()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__8__Impl"


    // $ANTLR start "rule__Composite__Group__9"
    // InternalLinguaFranca.g:1691:1: rule__Composite__Group__9 : rule__Composite__Group__9__Impl rule__Composite__Group__10 ;
    public final void rule__Composite__Group__9() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1695:1: ( rule__Composite__Group__9__Impl rule__Composite__Group__10 )
            // InternalLinguaFranca.g:1696:2: rule__Composite__Group__9__Impl rule__Composite__Group__10
            {
            pushFollow(FOLLOW_14);
            rule__Composite__Group__9__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Composite__Group__10();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__9"


    // $ANTLR start "rule__Composite__Group__9__Impl"
    // InternalLinguaFranca.g:1703:1: rule__Composite__Group__9__Impl : ( ( rule__Composite__ReactionsAssignment_9 )* ) ;
    public final void rule__Composite__Group__9__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1707:1: ( ( ( rule__Composite__ReactionsAssignment_9 )* ) )
            // InternalLinguaFranca.g:1708:1: ( ( rule__Composite__ReactionsAssignment_9 )* )
            {
            // InternalLinguaFranca.g:1708:1: ( ( rule__Composite__ReactionsAssignment_9 )* )
            // InternalLinguaFranca.g:1709:2: ( rule__Composite__ReactionsAssignment_9 )*
            {
             before(grammarAccess.getCompositeAccess().getReactionsAssignment_9()); 
            // InternalLinguaFranca.g:1710:2: ( rule__Composite__ReactionsAssignment_9 )*
            loop27:
            do {
                int alt27=2;
                int LA27_0 = input.LA(1);

                if ( (LA27_0==24) ) {
                    alt27=1;
                }


                switch (alt27) {
            	case 1 :
            	    // InternalLinguaFranca.g:1710:3: rule__Composite__ReactionsAssignment_9
            	    {
            	    pushFollow(FOLLOW_13);
            	    rule__Composite__ReactionsAssignment_9();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop27;
                }
            } while (true);

             after(grammarAccess.getCompositeAccess().getReactionsAssignment_9()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__9__Impl"


    // $ANTLR start "rule__Composite__Group__10"
    // InternalLinguaFranca.g:1718:1: rule__Composite__Group__10 : rule__Composite__Group__10__Impl rule__Composite__Group__11 ;
    public final void rule__Composite__Group__10() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1722:1: ( rule__Composite__Group__10__Impl rule__Composite__Group__11 )
            // InternalLinguaFranca.g:1723:2: rule__Composite__Group__10__Impl rule__Composite__Group__11
            {
            pushFollow(FOLLOW_14);
            rule__Composite__Group__10__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Composite__Group__11();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__10"


    // $ANTLR start "rule__Composite__Group__10__Impl"
    // InternalLinguaFranca.g:1730:1: rule__Composite__Group__10__Impl : ( ( rule__Composite__InstancesAssignment_10 )* ) ;
    public final void rule__Composite__Group__10__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1734:1: ( ( ( rule__Composite__InstancesAssignment_10 )* ) )
            // InternalLinguaFranca.g:1735:1: ( ( rule__Composite__InstancesAssignment_10 )* )
            {
            // InternalLinguaFranca.g:1735:1: ( ( rule__Composite__InstancesAssignment_10 )* )
            // InternalLinguaFranca.g:1736:2: ( rule__Composite__InstancesAssignment_10 )*
            {
             before(grammarAccess.getCompositeAccess().getInstancesAssignment_10()); 
            // InternalLinguaFranca.g:1737:2: ( rule__Composite__InstancesAssignment_10 )*
            loop28:
            do {
                int alt28=2;
                int LA28_0 = input.LA(1);

                if ( (LA28_0==RULE_ID) ) {
                    int LA28_1 = input.LA(2);

                    if ( (LA28_1==30) ) {
                        alt28=1;
                    }


                }


                switch (alt28) {
            	case 1 :
            	    // InternalLinguaFranca.g:1737:3: rule__Composite__InstancesAssignment_10
            	    {
            	    pushFollow(FOLLOW_15);
            	    rule__Composite__InstancesAssignment_10();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop28;
                }
            } while (true);

             after(grammarAccess.getCompositeAccess().getInstancesAssignment_10()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__10__Impl"


    // $ANTLR start "rule__Composite__Group__11"
    // InternalLinguaFranca.g:1745:1: rule__Composite__Group__11 : rule__Composite__Group__11__Impl rule__Composite__Group__12 ;
    public final void rule__Composite__Group__11() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1749:1: ( rule__Composite__Group__11__Impl rule__Composite__Group__12 )
            // InternalLinguaFranca.g:1750:2: rule__Composite__Group__11__Impl rule__Composite__Group__12
            {
            pushFollow(FOLLOW_14);
            rule__Composite__Group__11__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Composite__Group__12();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__11"


    // $ANTLR start "rule__Composite__Group__11__Impl"
    // InternalLinguaFranca.g:1757:1: rule__Composite__Group__11__Impl : ( ( rule__Composite__ConnectionsAssignment_11 )* ) ;
    public final void rule__Composite__Group__11__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1761:1: ( ( ( rule__Composite__ConnectionsAssignment_11 )* ) )
            // InternalLinguaFranca.g:1762:1: ( ( rule__Composite__ConnectionsAssignment_11 )* )
            {
            // InternalLinguaFranca.g:1762:1: ( ( rule__Composite__ConnectionsAssignment_11 )* )
            // InternalLinguaFranca.g:1763:2: ( rule__Composite__ConnectionsAssignment_11 )*
            {
             before(grammarAccess.getCompositeAccess().getConnectionsAssignment_11()); 
            // InternalLinguaFranca.g:1764:2: ( rule__Composite__ConnectionsAssignment_11 )*
            loop29:
            do {
                int alt29=2;
                int LA29_0 = input.LA(1);

                if ( (LA29_0==RULE_ID) ) {
                    alt29=1;
                }


                switch (alt29) {
            	case 1 :
            	    // InternalLinguaFranca.g:1764:3: rule__Composite__ConnectionsAssignment_11
            	    {
            	    pushFollow(FOLLOW_15);
            	    rule__Composite__ConnectionsAssignment_11();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop29;
                }
            } while (true);

             after(grammarAccess.getCompositeAccess().getConnectionsAssignment_11()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__11__Impl"


    // $ANTLR start "rule__Composite__Group__12"
    // InternalLinguaFranca.g:1772:1: rule__Composite__Group__12 : rule__Composite__Group__12__Impl ;
    public final void rule__Composite__Group__12() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1776:1: ( rule__Composite__Group__12__Impl )
            // InternalLinguaFranca.g:1777:2: rule__Composite__Group__12__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Composite__Group__12__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__12"


    // $ANTLR start "rule__Composite__Group__12__Impl"
    // InternalLinguaFranca.g:1783:1: rule__Composite__Group__12__Impl : ( '}' ) ;
    public final void rule__Composite__Group__12__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1787:1: ( ( '}' ) )
            // InternalLinguaFranca.g:1788:1: ( '}' )
            {
            // InternalLinguaFranca.g:1788:1: ( '}' )
            // InternalLinguaFranca.g:1789:2: '}'
            {
             before(grammarAccess.getCompositeAccess().getRightCurlyBracketKeyword_12()); 
            match(input,21,FOLLOW_2); 
             after(grammarAccess.getCompositeAccess().getRightCurlyBracketKeyword_12()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__Group__12__Impl"


    // $ANTLR start "rule__Input__Group__0"
    // InternalLinguaFranca.g:1799:1: rule__Input__Group__0 : rule__Input__Group__0__Impl rule__Input__Group__1 ;
    public final void rule__Input__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1803:1: ( rule__Input__Group__0__Impl rule__Input__Group__1 )
            // InternalLinguaFranca.g:1804:2: rule__Input__Group__0__Impl rule__Input__Group__1
            {
            pushFollow(FOLLOW_16);
            rule__Input__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Input__Group__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Input__Group__0"


    // $ANTLR start "rule__Input__Group__0__Impl"
    // InternalLinguaFranca.g:1811:1: rule__Input__Group__0__Impl : ( 'input' ) ;
    public final void rule__Input__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1815:1: ( ( 'input' ) )
            // InternalLinguaFranca.g:1816:1: ( 'input' )
            {
            // InternalLinguaFranca.g:1816:1: ( 'input' )
            // InternalLinguaFranca.g:1817:2: 'input'
            {
             before(grammarAccess.getInputAccess().getInputKeyword_0()); 
            match(input,13,FOLLOW_2); 
             after(grammarAccess.getInputAccess().getInputKeyword_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Input__Group__0__Impl"


    // $ANTLR start "rule__Input__Group__1"
    // InternalLinguaFranca.g:1826:1: rule__Input__Group__1 : rule__Input__Group__1__Impl rule__Input__Group__2 ;
    public final void rule__Input__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1830:1: ( rule__Input__Group__1__Impl rule__Input__Group__2 )
            // InternalLinguaFranca.g:1831:2: rule__Input__Group__1__Impl rule__Input__Group__2
            {
            pushFollow(FOLLOW_17);
            rule__Input__Group__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Input__Group__2();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Input__Group__1"


    // $ANTLR start "rule__Input__Group__1__Impl"
    // InternalLinguaFranca.g:1838:1: rule__Input__Group__1__Impl : ( ( rule__Input__NameAssignment_1 ) ) ;
    public final void rule__Input__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1842:1: ( ( ( rule__Input__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:1843:1: ( ( rule__Input__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:1843:1: ( ( rule__Input__NameAssignment_1 ) )
            // InternalLinguaFranca.g:1844:2: ( rule__Input__NameAssignment_1 )
            {
             before(grammarAccess.getInputAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:1845:2: ( rule__Input__NameAssignment_1 )
            // InternalLinguaFranca.g:1845:3: rule__Input__NameAssignment_1
            {
            pushFollow(FOLLOW_2);
            rule__Input__NameAssignment_1();

            state._fsp--;


            }

             after(grammarAccess.getInputAccess().getNameAssignment_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Input__Group__1__Impl"


    // $ANTLR start "rule__Input__Group__2"
    // InternalLinguaFranca.g:1853:1: rule__Input__Group__2 : rule__Input__Group__2__Impl rule__Input__Group__3 ;
    public final void rule__Input__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1857:1: ( rule__Input__Group__2__Impl rule__Input__Group__3 )
            // InternalLinguaFranca.g:1858:2: rule__Input__Group__2__Impl rule__Input__Group__3
            {
            pushFollow(FOLLOW_17);
            rule__Input__Group__2__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Input__Group__3();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Input__Group__2"


    // $ANTLR start "rule__Input__Group__2__Impl"
    // InternalLinguaFranca.g:1865:1: rule__Input__Group__2__Impl : ( ( rule__Input__Group_2__0 )? ) ;
    public final void rule__Input__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1869:1: ( ( ( rule__Input__Group_2__0 )? ) )
            // InternalLinguaFranca.g:1870:1: ( ( rule__Input__Group_2__0 )? )
            {
            // InternalLinguaFranca.g:1870:1: ( ( rule__Input__Group_2__0 )? )
            // InternalLinguaFranca.g:1871:2: ( rule__Input__Group_2__0 )?
            {
             before(grammarAccess.getInputAccess().getGroup_2()); 
            // InternalLinguaFranca.g:1872:2: ( rule__Input__Group_2__0 )?
            int alt30=2;
            int LA30_0 = input.LA(1);

            if ( (LA30_0==23) ) {
                alt30=1;
            }
            switch (alt30) {
                case 1 :
                    // InternalLinguaFranca.g:1872:3: rule__Input__Group_2__0
                    {
                    pushFollow(FOLLOW_2);
                    rule__Input__Group_2__0();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getInputAccess().getGroup_2()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Input__Group__2__Impl"


    // $ANTLR start "rule__Input__Group__3"
    // InternalLinguaFranca.g:1880:1: rule__Input__Group__3 : rule__Input__Group__3__Impl ;
    public final void rule__Input__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1884:1: ( rule__Input__Group__3__Impl )
            // InternalLinguaFranca.g:1885:2: rule__Input__Group__3__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Input__Group__3__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Input__Group__3"


    // $ANTLR start "rule__Input__Group__3__Impl"
    // InternalLinguaFranca.g:1891:1: rule__Input__Group__3__Impl : ( ';' ) ;
    public final void rule__Input__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1895:1: ( ( ';' ) )
            // InternalLinguaFranca.g:1896:1: ( ';' )
            {
            // InternalLinguaFranca.g:1896:1: ( ';' )
            // InternalLinguaFranca.g:1897:2: ';'
            {
             before(grammarAccess.getInputAccess().getSemicolonKeyword_3()); 
            match(input,17,FOLLOW_2); 
             after(grammarAccess.getInputAccess().getSemicolonKeyword_3()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Input__Group__3__Impl"


    // $ANTLR start "rule__Input__Group_2__0"
    // InternalLinguaFranca.g:1907:1: rule__Input__Group_2__0 : rule__Input__Group_2__0__Impl rule__Input__Group_2__1 ;
    public final void rule__Input__Group_2__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1911:1: ( rule__Input__Group_2__0__Impl rule__Input__Group_2__1 )
            // InternalLinguaFranca.g:1912:2: rule__Input__Group_2__0__Impl rule__Input__Group_2__1
            {
            pushFollow(FOLLOW_18);
            rule__Input__Group_2__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Input__Group_2__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Input__Group_2__0"


    // $ANTLR start "rule__Input__Group_2__0__Impl"
    // InternalLinguaFranca.g:1919:1: rule__Input__Group_2__0__Impl : ( ':' ) ;
    public final void rule__Input__Group_2__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1923:1: ( ( ':' ) )
            // InternalLinguaFranca.g:1924:1: ( ':' )
            {
            // InternalLinguaFranca.g:1924:1: ( ':' )
            // InternalLinguaFranca.g:1925:2: ':'
            {
             before(grammarAccess.getInputAccess().getColonKeyword_2_0()); 
            match(input,23,FOLLOW_2); 
             after(grammarAccess.getInputAccess().getColonKeyword_2_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Input__Group_2__0__Impl"


    // $ANTLR start "rule__Input__Group_2__1"
    // InternalLinguaFranca.g:1934:1: rule__Input__Group_2__1 : rule__Input__Group_2__1__Impl ;
    public final void rule__Input__Group_2__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1938:1: ( rule__Input__Group_2__1__Impl )
            // InternalLinguaFranca.g:1939:2: rule__Input__Group_2__1__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Input__Group_2__1__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Input__Group_2__1"


    // $ANTLR start "rule__Input__Group_2__1__Impl"
    // InternalLinguaFranca.g:1945:1: rule__Input__Group_2__1__Impl : ( ( rule__Input__TypeAssignment_2_1 ) ) ;
    public final void rule__Input__Group_2__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1949:1: ( ( ( rule__Input__TypeAssignment_2_1 ) ) )
            // InternalLinguaFranca.g:1950:1: ( ( rule__Input__TypeAssignment_2_1 ) )
            {
            // InternalLinguaFranca.g:1950:1: ( ( rule__Input__TypeAssignment_2_1 ) )
            // InternalLinguaFranca.g:1951:2: ( rule__Input__TypeAssignment_2_1 )
            {
             before(grammarAccess.getInputAccess().getTypeAssignment_2_1()); 
            // InternalLinguaFranca.g:1952:2: ( rule__Input__TypeAssignment_2_1 )
            // InternalLinguaFranca.g:1952:3: rule__Input__TypeAssignment_2_1
            {
            pushFollow(FOLLOW_2);
            rule__Input__TypeAssignment_2_1();

            state._fsp--;


            }

             after(grammarAccess.getInputAccess().getTypeAssignment_2_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Input__Group_2__1__Impl"


    // $ANTLR start "rule__Output__Group__0"
    // InternalLinguaFranca.g:1961:1: rule__Output__Group__0 : rule__Output__Group__0__Impl rule__Output__Group__1 ;
    public final void rule__Output__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1965:1: ( rule__Output__Group__0__Impl rule__Output__Group__1 )
            // InternalLinguaFranca.g:1966:2: rule__Output__Group__0__Impl rule__Output__Group__1
            {
            pushFollow(FOLLOW_19);
            rule__Output__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Output__Group__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Output__Group__0"


    // $ANTLR start "rule__Output__Group__0__Impl"
    // InternalLinguaFranca.g:1973:1: rule__Output__Group__0__Impl : ( 'output' ) ;
    public final void rule__Output__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1977:1: ( ( 'output' ) )
            // InternalLinguaFranca.g:1978:1: ( 'output' )
            {
            // InternalLinguaFranca.g:1978:1: ( 'output' )
            // InternalLinguaFranca.g:1979:2: 'output'
            {
             before(grammarAccess.getOutputAccess().getOutputKeyword_0()); 
            match(input,14,FOLLOW_2); 
             after(grammarAccess.getOutputAccess().getOutputKeyword_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Output__Group__0__Impl"


    // $ANTLR start "rule__Output__Group__1"
    // InternalLinguaFranca.g:1988:1: rule__Output__Group__1 : rule__Output__Group__1__Impl rule__Output__Group__2 ;
    public final void rule__Output__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1992:1: ( rule__Output__Group__1__Impl rule__Output__Group__2 )
            // InternalLinguaFranca.g:1993:2: rule__Output__Group__1__Impl rule__Output__Group__2
            {
            pushFollow(FOLLOW_17);
            rule__Output__Group__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Output__Group__2();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Output__Group__1"


    // $ANTLR start "rule__Output__Group__1__Impl"
    // InternalLinguaFranca.g:2000:1: rule__Output__Group__1__Impl : ( ( rule__Output__NameAssignment_1 ) ) ;
    public final void rule__Output__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2004:1: ( ( ( rule__Output__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:2005:1: ( ( rule__Output__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:2005:1: ( ( rule__Output__NameAssignment_1 ) )
            // InternalLinguaFranca.g:2006:2: ( rule__Output__NameAssignment_1 )
            {
             before(grammarAccess.getOutputAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:2007:2: ( rule__Output__NameAssignment_1 )
            // InternalLinguaFranca.g:2007:3: rule__Output__NameAssignment_1
            {
            pushFollow(FOLLOW_2);
            rule__Output__NameAssignment_1();

            state._fsp--;


            }

             after(grammarAccess.getOutputAccess().getNameAssignment_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Output__Group__1__Impl"


    // $ANTLR start "rule__Output__Group__2"
    // InternalLinguaFranca.g:2015:1: rule__Output__Group__2 : rule__Output__Group__2__Impl rule__Output__Group__3 ;
    public final void rule__Output__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2019:1: ( rule__Output__Group__2__Impl rule__Output__Group__3 )
            // InternalLinguaFranca.g:2020:2: rule__Output__Group__2__Impl rule__Output__Group__3
            {
            pushFollow(FOLLOW_17);
            rule__Output__Group__2__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Output__Group__3();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Output__Group__2"


    // $ANTLR start "rule__Output__Group__2__Impl"
    // InternalLinguaFranca.g:2027:1: rule__Output__Group__2__Impl : ( ( rule__Output__Group_2__0 )? ) ;
    public final void rule__Output__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2031:1: ( ( ( rule__Output__Group_2__0 )? ) )
            // InternalLinguaFranca.g:2032:1: ( ( rule__Output__Group_2__0 )? )
            {
            // InternalLinguaFranca.g:2032:1: ( ( rule__Output__Group_2__0 )? )
            // InternalLinguaFranca.g:2033:2: ( rule__Output__Group_2__0 )?
            {
             before(grammarAccess.getOutputAccess().getGroup_2()); 
            // InternalLinguaFranca.g:2034:2: ( rule__Output__Group_2__0 )?
            int alt31=2;
            int LA31_0 = input.LA(1);

            if ( (LA31_0==23) ) {
                alt31=1;
            }
            switch (alt31) {
                case 1 :
                    // InternalLinguaFranca.g:2034:3: rule__Output__Group_2__0
                    {
                    pushFollow(FOLLOW_2);
                    rule__Output__Group_2__0();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getOutputAccess().getGroup_2()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Output__Group__2__Impl"


    // $ANTLR start "rule__Output__Group__3"
    // InternalLinguaFranca.g:2042:1: rule__Output__Group__3 : rule__Output__Group__3__Impl ;
    public final void rule__Output__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2046:1: ( rule__Output__Group__3__Impl )
            // InternalLinguaFranca.g:2047:2: rule__Output__Group__3__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Output__Group__3__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Output__Group__3"


    // $ANTLR start "rule__Output__Group__3__Impl"
    // InternalLinguaFranca.g:2053:1: rule__Output__Group__3__Impl : ( ';' ) ;
    public final void rule__Output__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2057:1: ( ( ';' ) )
            // InternalLinguaFranca.g:2058:1: ( ';' )
            {
            // InternalLinguaFranca.g:2058:1: ( ';' )
            // InternalLinguaFranca.g:2059:2: ';'
            {
             before(grammarAccess.getOutputAccess().getSemicolonKeyword_3()); 
            match(input,17,FOLLOW_2); 
             after(grammarAccess.getOutputAccess().getSemicolonKeyword_3()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Output__Group__3__Impl"


    // $ANTLR start "rule__Output__Group_2__0"
    // InternalLinguaFranca.g:2069:1: rule__Output__Group_2__0 : rule__Output__Group_2__0__Impl rule__Output__Group_2__1 ;
    public final void rule__Output__Group_2__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2073:1: ( rule__Output__Group_2__0__Impl rule__Output__Group_2__1 )
            // InternalLinguaFranca.g:2074:2: rule__Output__Group_2__0__Impl rule__Output__Group_2__1
            {
            pushFollow(FOLLOW_18);
            rule__Output__Group_2__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Output__Group_2__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Output__Group_2__0"


    // $ANTLR start "rule__Output__Group_2__0__Impl"
    // InternalLinguaFranca.g:2081:1: rule__Output__Group_2__0__Impl : ( ':' ) ;
    public final void rule__Output__Group_2__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2085:1: ( ( ':' ) )
            // InternalLinguaFranca.g:2086:1: ( ':' )
            {
            // InternalLinguaFranca.g:2086:1: ( ':' )
            // InternalLinguaFranca.g:2087:2: ':'
            {
             before(grammarAccess.getOutputAccess().getColonKeyword_2_0()); 
            match(input,23,FOLLOW_2); 
             after(grammarAccess.getOutputAccess().getColonKeyword_2_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Output__Group_2__0__Impl"


    // $ANTLR start "rule__Output__Group_2__1"
    // InternalLinguaFranca.g:2096:1: rule__Output__Group_2__1 : rule__Output__Group_2__1__Impl ;
    public final void rule__Output__Group_2__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2100:1: ( rule__Output__Group_2__1__Impl )
            // InternalLinguaFranca.g:2101:2: rule__Output__Group_2__1__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Output__Group_2__1__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Output__Group_2__1"


    // $ANTLR start "rule__Output__Group_2__1__Impl"
    // InternalLinguaFranca.g:2107:1: rule__Output__Group_2__1__Impl : ( ( rule__Output__TypeAssignment_2_1 ) ) ;
    public final void rule__Output__Group_2__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2111:1: ( ( ( rule__Output__TypeAssignment_2_1 ) ) )
            // InternalLinguaFranca.g:2112:1: ( ( rule__Output__TypeAssignment_2_1 ) )
            {
            // InternalLinguaFranca.g:2112:1: ( ( rule__Output__TypeAssignment_2_1 ) )
            // InternalLinguaFranca.g:2113:2: ( rule__Output__TypeAssignment_2_1 )
            {
             before(grammarAccess.getOutputAccess().getTypeAssignment_2_1()); 
            // InternalLinguaFranca.g:2114:2: ( rule__Output__TypeAssignment_2_1 )
            // InternalLinguaFranca.g:2114:3: rule__Output__TypeAssignment_2_1
            {
            pushFollow(FOLLOW_2);
            rule__Output__TypeAssignment_2_1();

            state._fsp--;


            }

             after(grammarAccess.getOutputAccess().getTypeAssignment_2_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Output__Group_2__1__Impl"


    // $ANTLR start "rule__Clock__Group__0"
    // InternalLinguaFranca.g:2123:1: rule__Clock__Group__0 : rule__Clock__Group__0__Impl rule__Clock__Group__1 ;
    public final void rule__Clock__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2127:1: ( rule__Clock__Group__0__Impl rule__Clock__Group__1 )
            // InternalLinguaFranca.g:2128:2: rule__Clock__Group__0__Impl rule__Clock__Group__1
            {
            pushFollow(FOLLOW_20);
            rule__Clock__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Clock__Group__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Clock__Group__0"


    // $ANTLR start "rule__Clock__Group__0__Impl"
    // InternalLinguaFranca.g:2135:1: rule__Clock__Group__0__Impl : ( 'clock' ) ;
    public final void rule__Clock__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2139:1: ( ( 'clock' ) )
            // InternalLinguaFranca.g:2140:1: ( 'clock' )
            {
            // InternalLinguaFranca.g:2140:1: ( 'clock' )
            // InternalLinguaFranca.g:2141:2: 'clock'
            {
             before(grammarAccess.getClockAccess().getClockKeyword_0()); 
            match(input,15,FOLLOW_2); 
             after(grammarAccess.getClockAccess().getClockKeyword_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Clock__Group__0__Impl"


    // $ANTLR start "rule__Clock__Group__1"
    // InternalLinguaFranca.g:2150:1: rule__Clock__Group__1 : rule__Clock__Group__1__Impl rule__Clock__Group__2 ;
    public final void rule__Clock__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2154:1: ( rule__Clock__Group__1__Impl rule__Clock__Group__2 )
            // InternalLinguaFranca.g:2155:2: rule__Clock__Group__1__Impl rule__Clock__Group__2
            {
            pushFollow(FOLLOW_21);
            rule__Clock__Group__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Clock__Group__2();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Clock__Group__1"


    // $ANTLR start "rule__Clock__Group__1__Impl"
    // InternalLinguaFranca.g:2162:1: rule__Clock__Group__1__Impl : ( ( rule__Clock__NameAssignment_1 ) ) ;
    public final void rule__Clock__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2166:1: ( ( ( rule__Clock__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:2167:1: ( ( rule__Clock__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:2167:1: ( ( rule__Clock__NameAssignment_1 ) )
            // InternalLinguaFranca.g:2168:2: ( rule__Clock__NameAssignment_1 )
            {
             before(grammarAccess.getClockAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:2169:2: ( rule__Clock__NameAssignment_1 )
            // InternalLinguaFranca.g:2169:3: rule__Clock__NameAssignment_1
            {
            pushFollow(FOLLOW_2);
            rule__Clock__NameAssignment_1();

            state._fsp--;


            }

             after(grammarAccess.getClockAccess().getNameAssignment_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Clock__Group__1__Impl"


    // $ANTLR start "rule__Clock__Group__2"
    // InternalLinguaFranca.g:2177:1: rule__Clock__Group__2 : rule__Clock__Group__2__Impl rule__Clock__Group__3 ;
    public final void rule__Clock__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2181:1: ( rule__Clock__Group__2__Impl rule__Clock__Group__3 )
            // InternalLinguaFranca.g:2182:2: rule__Clock__Group__2__Impl rule__Clock__Group__3
            {
            pushFollow(FOLLOW_21);
            rule__Clock__Group__2__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Clock__Group__3();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Clock__Group__2"


    // $ANTLR start "rule__Clock__Group__2__Impl"
    // InternalLinguaFranca.g:2189:1: rule__Clock__Group__2__Impl : ( ( rule__Clock__PeriodAssignment_2 )? ) ;
    public final void rule__Clock__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2193:1: ( ( ( rule__Clock__PeriodAssignment_2 )? ) )
            // InternalLinguaFranca.g:2194:1: ( ( rule__Clock__PeriodAssignment_2 )? )
            {
            // InternalLinguaFranca.g:2194:1: ( ( rule__Clock__PeriodAssignment_2 )? )
            // InternalLinguaFranca.g:2195:2: ( rule__Clock__PeriodAssignment_2 )?
            {
             before(grammarAccess.getClockAccess().getPeriodAssignment_2()); 
            // InternalLinguaFranca.g:2196:2: ( rule__Clock__PeriodAssignment_2 )?
            int alt32=2;
            int LA32_0 = input.LA(1);

            if ( (LA32_0==25) ) {
                alt32=1;
            }
            switch (alt32) {
                case 1 :
                    // InternalLinguaFranca.g:2196:3: rule__Clock__PeriodAssignment_2
                    {
                    pushFollow(FOLLOW_2);
                    rule__Clock__PeriodAssignment_2();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getClockAccess().getPeriodAssignment_2()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Clock__Group__2__Impl"


    // $ANTLR start "rule__Clock__Group__3"
    // InternalLinguaFranca.g:2204:1: rule__Clock__Group__3 : rule__Clock__Group__3__Impl ;
    public final void rule__Clock__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2208:1: ( rule__Clock__Group__3__Impl )
            // InternalLinguaFranca.g:2209:2: rule__Clock__Group__3__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Clock__Group__3__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Clock__Group__3"


    // $ANTLR start "rule__Clock__Group__3__Impl"
    // InternalLinguaFranca.g:2215:1: rule__Clock__Group__3__Impl : ( ';' ) ;
    public final void rule__Clock__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2219:1: ( ( ';' ) )
            // InternalLinguaFranca.g:2220:1: ( ';' )
            {
            // InternalLinguaFranca.g:2220:1: ( ';' )
            // InternalLinguaFranca.g:2221:2: ';'
            {
             before(grammarAccess.getClockAccess().getSemicolonKeyword_3()); 
            match(input,17,FOLLOW_2); 
             after(grammarAccess.getClockAccess().getSemicolonKeyword_3()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Clock__Group__3__Impl"


    // $ANTLR start "rule__Reaction__Group__0"
    // InternalLinguaFranca.g:2231:1: rule__Reaction__Group__0 : rule__Reaction__Group__0__Impl rule__Reaction__Group__1 ;
    public final void rule__Reaction__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2235:1: ( rule__Reaction__Group__0__Impl rule__Reaction__Group__1 )
            // InternalLinguaFranca.g:2236:2: rule__Reaction__Group__0__Impl rule__Reaction__Group__1
            {
            pushFollow(FOLLOW_22);
            rule__Reaction__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Reaction__Group__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group__0"


    // $ANTLR start "rule__Reaction__Group__0__Impl"
    // InternalLinguaFranca.g:2243:1: rule__Reaction__Group__0__Impl : ( 'reaction' ) ;
    public final void rule__Reaction__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2247:1: ( ( 'reaction' ) )
            // InternalLinguaFranca.g:2248:1: ( 'reaction' )
            {
            // InternalLinguaFranca.g:2248:1: ( 'reaction' )
            // InternalLinguaFranca.g:2249:2: 'reaction'
            {
             before(grammarAccess.getReactionAccess().getReactionKeyword_0()); 
            match(input,24,FOLLOW_2); 
             after(grammarAccess.getReactionAccess().getReactionKeyword_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group__0__Impl"


    // $ANTLR start "rule__Reaction__Group__1"
    // InternalLinguaFranca.g:2258:1: rule__Reaction__Group__1 : rule__Reaction__Group__1__Impl rule__Reaction__Group__2 ;
    public final void rule__Reaction__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2262:1: ( rule__Reaction__Group__1__Impl rule__Reaction__Group__2 )
            // InternalLinguaFranca.g:2263:2: rule__Reaction__Group__1__Impl rule__Reaction__Group__2
            {
            pushFollow(FOLLOW_22);
            rule__Reaction__Group__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Reaction__Group__2();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group__1"


    // $ANTLR start "rule__Reaction__Group__1__Impl"
    // InternalLinguaFranca.g:2270:1: rule__Reaction__Group__1__Impl : ( ( rule__Reaction__Group_1__0 )? ) ;
    public final void rule__Reaction__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2274:1: ( ( ( rule__Reaction__Group_1__0 )? ) )
            // InternalLinguaFranca.g:2275:1: ( ( rule__Reaction__Group_1__0 )? )
            {
            // InternalLinguaFranca.g:2275:1: ( ( rule__Reaction__Group_1__0 )? )
            // InternalLinguaFranca.g:2276:2: ( rule__Reaction__Group_1__0 )?
            {
             before(grammarAccess.getReactionAccess().getGroup_1()); 
            // InternalLinguaFranca.g:2277:2: ( rule__Reaction__Group_1__0 )?
            int alt33=2;
            int LA33_0 = input.LA(1);

            if ( (LA33_0==25) ) {
                alt33=1;
            }
            switch (alt33) {
                case 1 :
                    // InternalLinguaFranca.g:2277:3: rule__Reaction__Group_1__0
                    {
                    pushFollow(FOLLOW_2);
                    rule__Reaction__Group_1__0();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getReactionAccess().getGroup_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group__1__Impl"


    // $ANTLR start "rule__Reaction__Group__2"
    // InternalLinguaFranca.g:2285:1: rule__Reaction__Group__2 : rule__Reaction__Group__2__Impl rule__Reaction__Group__3 ;
    public final void rule__Reaction__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2289:1: ( rule__Reaction__Group__2__Impl rule__Reaction__Group__3 )
            // InternalLinguaFranca.g:2290:2: rule__Reaction__Group__2__Impl rule__Reaction__Group__3
            {
            pushFollow(FOLLOW_22);
            rule__Reaction__Group__2__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Reaction__Group__3();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group__2"


    // $ANTLR start "rule__Reaction__Group__2__Impl"
    // InternalLinguaFranca.g:2297:1: rule__Reaction__Group__2__Impl : ( ( rule__Reaction__GetsAssignment_2 )? ) ;
    public final void rule__Reaction__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2301:1: ( ( ( rule__Reaction__GetsAssignment_2 )? ) )
            // InternalLinguaFranca.g:2302:1: ( ( rule__Reaction__GetsAssignment_2 )? )
            {
            // InternalLinguaFranca.g:2302:1: ( ( rule__Reaction__GetsAssignment_2 )? )
            // InternalLinguaFranca.g:2303:2: ( rule__Reaction__GetsAssignment_2 )?
            {
             before(grammarAccess.getReactionAccess().getGetsAssignment_2()); 
            // InternalLinguaFranca.g:2304:2: ( rule__Reaction__GetsAssignment_2 )?
            int alt34=2;
            int LA34_0 = input.LA(1);

            if ( (LA34_0==RULE_ID) ) {
                alt34=1;
            }
            switch (alt34) {
                case 1 :
                    // InternalLinguaFranca.g:2304:3: rule__Reaction__GetsAssignment_2
                    {
                    pushFollow(FOLLOW_2);
                    rule__Reaction__GetsAssignment_2();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getReactionAccess().getGetsAssignment_2()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group__2__Impl"


    // $ANTLR start "rule__Reaction__Group__3"
    // InternalLinguaFranca.g:2312:1: rule__Reaction__Group__3 : rule__Reaction__Group__3__Impl rule__Reaction__Group__4 ;
    public final void rule__Reaction__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2316:1: ( rule__Reaction__Group__3__Impl rule__Reaction__Group__4 )
            // InternalLinguaFranca.g:2317:2: rule__Reaction__Group__3__Impl rule__Reaction__Group__4
            {
            pushFollow(FOLLOW_22);
            rule__Reaction__Group__3__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Reaction__Group__4();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group__3"


    // $ANTLR start "rule__Reaction__Group__3__Impl"
    // InternalLinguaFranca.g:2324:1: rule__Reaction__Group__3__Impl : ( ( rule__Reaction__SetsAssignment_3 )? ) ;
    public final void rule__Reaction__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2328:1: ( ( ( rule__Reaction__SetsAssignment_3 )? ) )
            // InternalLinguaFranca.g:2329:1: ( ( rule__Reaction__SetsAssignment_3 )? )
            {
            // InternalLinguaFranca.g:2329:1: ( ( rule__Reaction__SetsAssignment_3 )? )
            // InternalLinguaFranca.g:2330:2: ( rule__Reaction__SetsAssignment_3 )?
            {
             before(grammarAccess.getReactionAccess().getSetsAssignment_3()); 
            // InternalLinguaFranca.g:2331:2: ( rule__Reaction__SetsAssignment_3 )?
            int alt35=2;
            int LA35_0 = input.LA(1);

            if ( (LA35_0==32) ) {
                alt35=1;
            }
            switch (alt35) {
                case 1 :
                    // InternalLinguaFranca.g:2331:3: rule__Reaction__SetsAssignment_3
                    {
                    pushFollow(FOLLOW_2);
                    rule__Reaction__SetsAssignment_3();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getReactionAccess().getSetsAssignment_3()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group__3__Impl"


    // $ANTLR start "rule__Reaction__Group__4"
    // InternalLinguaFranca.g:2339:1: rule__Reaction__Group__4 : rule__Reaction__Group__4__Impl ;
    public final void rule__Reaction__Group__4() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2343:1: ( rule__Reaction__Group__4__Impl )
            // InternalLinguaFranca.g:2344:2: rule__Reaction__Group__4__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Reaction__Group__4__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group__4"


    // $ANTLR start "rule__Reaction__Group__4__Impl"
    // InternalLinguaFranca.g:2350:1: rule__Reaction__Group__4__Impl : ( ( rule__Reaction__CodeAssignment_4 ) ) ;
    public final void rule__Reaction__Group__4__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2354:1: ( ( ( rule__Reaction__CodeAssignment_4 ) ) )
            // InternalLinguaFranca.g:2355:1: ( ( rule__Reaction__CodeAssignment_4 ) )
            {
            // InternalLinguaFranca.g:2355:1: ( ( rule__Reaction__CodeAssignment_4 ) )
            // InternalLinguaFranca.g:2356:2: ( rule__Reaction__CodeAssignment_4 )
            {
             before(grammarAccess.getReactionAccess().getCodeAssignment_4()); 
            // InternalLinguaFranca.g:2357:2: ( rule__Reaction__CodeAssignment_4 )
            // InternalLinguaFranca.g:2357:3: rule__Reaction__CodeAssignment_4
            {
            pushFollow(FOLLOW_2);
            rule__Reaction__CodeAssignment_4();

            state._fsp--;


            }

             after(grammarAccess.getReactionAccess().getCodeAssignment_4()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group__4__Impl"


    // $ANTLR start "rule__Reaction__Group_1__0"
    // InternalLinguaFranca.g:2366:1: rule__Reaction__Group_1__0 : rule__Reaction__Group_1__0__Impl rule__Reaction__Group_1__1 ;
    public final void rule__Reaction__Group_1__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2370:1: ( rule__Reaction__Group_1__0__Impl rule__Reaction__Group_1__1 )
            // InternalLinguaFranca.g:2371:2: rule__Reaction__Group_1__0__Impl rule__Reaction__Group_1__1
            {
            pushFollow(FOLLOW_23);
            rule__Reaction__Group_1__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Reaction__Group_1__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group_1__0"


    // $ANTLR start "rule__Reaction__Group_1__0__Impl"
    // InternalLinguaFranca.g:2378:1: rule__Reaction__Group_1__0__Impl : ( '(' ) ;
    public final void rule__Reaction__Group_1__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2382:1: ( ( '(' ) )
            // InternalLinguaFranca.g:2383:1: ( '(' )
            {
            // InternalLinguaFranca.g:2383:1: ( '(' )
            // InternalLinguaFranca.g:2384:2: '('
            {
             before(grammarAccess.getReactionAccess().getLeftParenthesisKeyword_1_0()); 
            match(input,25,FOLLOW_2); 
             after(grammarAccess.getReactionAccess().getLeftParenthesisKeyword_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group_1__0__Impl"


    // $ANTLR start "rule__Reaction__Group_1__1"
    // InternalLinguaFranca.g:2393:1: rule__Reaction__Group_1__1 : rule__Reaction__Group_1__1__Impl rule__Reaction__Group_1__2 ;
    public final void rule__Reaction__Group_1__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2397:1: ( rule__Reaction__Group_1__1__Impl rule__Reaction__Group_1__2 )
            // InternalLinguaFranca.g:2398:2: rule__Reaction__Group_1__1__Impl rule__Reaction__Group_1__2
            {
            pushFollow(FOLLOW_23);
            rule__Reaction__Group_1__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Reaction__Group_1__2();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group_1__1"


    // $ANTLR start "rule__Reaction__Group_1__1__Impl"
    // InternalLinguaFranca.g:2405:1: rule__Reaction__Group_1__1__Impl : ( ( rule__Reaction__Group_1_1__0 )? ) ;
    public final void rule__Reaction__Group_1__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2409:1: ( ( ( rule__Reaction__Group_1_1__0 )? ) )
            // InternalLinguaFranca.g:2410:1: ( ( rule__Reaction__Group_1_1__0 )? )
            {
            // InternalLinguaFranca.g:2410:1: ( ( rule__Reaction__Group_1_1__0 )? )
            // InternalLinguaFranca.g:2411:2: ( rule__Reaction__Group_1_1__0 )?
            {
             before(grammarAccess.getReactionAccess().getGroup_1_1()); 
            // InternalLinguaFranca.g:2412:2: ( rule__Reaction__Group_1_1__0 )?
            int alt36=2;
            int LA36_0 = input.LA(1);

            if ( (LA36_0==RULE_ID) ) {
                alt36=1;
            }
            switch (alt36) {
                case 1 :
                    // InternalLinguaFranca.g:2412:3: rule__Reaction__Group_1_1__0
                    {
                    pushFollow(FOLLOW_2);
                    rule__Reaction__Group_1_1__0();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getReactionAccess().getGroup_1_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group_1__1__Impl"


    // $ANTLR start "rule__Reaction__Group_1__2"
    // InternalLinguaFranca.g:2420:1: rule__Reaction__Group_1__2 : rule__Reaction__Group_1__2__Impl ;
    public final void rule__Reaction__Group_1__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2424:1: ( rule__Reaction__Group_1__2__Impl )
            // InternalLinguaFranca.g:2425:2: rule__Reaction__Group_1__2__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Reaction__Group_1__2__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group_1__2"


    // $ANTLR start "rule__Reaction__Group_1__2__Impl"
    // InternalLinguaFranca.g:2431:1: rule__Reaction__Group_1__2__Impl : ( ')' ) ;
    public final void rule__Reaction__Group_1__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2435:1: ( ( ')' ) )
            // InternalLinguaFranca.g:2436:1: ( ')' )
            {
            // InternalLinguaFranca.g:2436:1: ( ')' )
            // InternalLinguaFranca.g:2437:2: ')'
            {
             before(grammarAccess.getReactionAccess().getRightParenthesisKeyword_1_2()); 
            match(input,26,FOLLOW_2); 
             after(grammarAccess.getReactionAccess().getRightParenthesisKeyword_1_2()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group_1__2__Impl"


    // $ANTLR start "rule__Reaction__Group_1_1__0"
    // InternalLinguaFranca.g:2447:1: rule__Reaction__Group_1_1__0 : rule__Reaction__Group_1_1__0__Impl rule__Reaction__Group_1_1__1 ;
    public final void rule__Reaction__Group_1_1__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2451:1: ( rule__Reaction__Group_1_1__0__Impl rule__Reaction__Group_1_1__1 )
            // InternalLinguaFranca.g:2452:2: rule__Reaction__Group_1_1__0__Impl rule__Reaction__Group_1_1__1
            {
            pushFollow(FOLLOW_24);
            rule__Reaction__Group_1_1__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Reaction__Group_1_1__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group_1_1__0"


    // $ANTLR start "rule__Reaction__Group_1_1__0__Impl"
    // InternalLinguaFranca.g:2459:1: rule__Reaction__Group_1_1__0__Impl : ( ( rule__Reaction__TriggersAssignment_1_1_0 ) ) ;
    public final void rule__Reaction__Group_1_1__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2463:1: ( ( ( rule__Reaction__TriggersAssignment_1_1_0 ) ) )
            // InternalLinguaFranca.g:2464:1: ( ( rule__Reaction__TriggersAssignment_1_1_0 ) )
            {
            // InternalLinguaFranca.g:2464:1: ( ( rule__Reaction__TriggersAssignment_1_1_0 ) )
            // InternalLinguaFranca.g:2465:2: ( rule__Reaction__TriggersAssignment_1_1_0 )
            {
             before(grammarAccess.getReactionAccess().getTriggersAssignment_1_1_0()); 
            // InternalLinguaFranca.g:2466:2: ( rule__Reaction__TriggersAssignment_1_1_0 )
            // InternalLinguaFranca.g:2466:3: rule__Reaction__TriggersAssignment_1_1_0
            {
            pushFollow(FOLLOW_2);
            rule__Reaction__TriggersAssignment_1_1_0();

            state._fsp--;


            }

             after(grammarAccess.getReactionAccess().getTriggersAssignment_1_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group_1_1__0__Impl"


    // $ANTLR start "rule__Reaction__Group_1_1__1"
    // InternalLinguaFranca.g:2474:1: rule__Reaction__Group_1_1__1 : rule__Reaction__Group_1_1__1__Impl ;
    public final void rule__Reaction__Group_1_1__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2478:1: ( rule__Reaction__Group_1_1__1__Impl )
            // InternalLinguaFranca.g:2479:2: rule__Reaction__Group_1_1__1__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Reaction__Group_1_1__1__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group_1_1__1"


    // $ANTLR start "rule__Reaction__Group_1_1__1__Impl"
    // InternalLinguaFranca.g:2485:1: rule__Reaction__Group_1_1__1__Impl : ( ( rule__Reaction__Group_1_1_1__0 )* ) ;
    public final void rule__Reaction__Group_1_1__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2489:1: ( ( ( rule__Reaction__Group_1_1_1__0 )* ) )
            // InternalLinguaFranca.g:2490:1: ( ( rule__Reaction__Group_1_1_1__0 )* )
            {
            // InternalLinguaFranca.g:2490:1: ( ( rule__Reaction__Group_1_1_1__0 )* )
            // InternalLinguaFranca.g:2491:2: ( rule__Reaction__Group_1_1_1__0 )*
            {
             before(grammarAccess.getReactionAccess().getGroup_1_1_1()); 
            // InternalLinguaFranca.g:2492:2: ( rule__Reaction__Group_1_1_1__0 )*
            loop37:
            do {
                int alt37=2;
                int LA37_0 = input.LA(1);

                if ( (LA37_0==27) ) {
                    alt37=1;
                }


                switch (alt37) {
            	case 1 :
            	    // InternalLinguaFranca.g:2492:3: rule__Reaction__Group_1_1_1__0
            	    {
            	    pushFollow(FOLLOW_25);
            	    rule__Reaction__Group_1_1_1__0();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop37;
                }
            } while (true);

             after(grammarAccess.getReactionAccess().getGroup_1_1_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group_1_1__1__Impl"


    // $ANTLR start "rule__Reaction__Group_1_1_1__0"
    // InternalLinguaFranca.g:2501:1: rule__Reaction__Group_1_1_1__0 : rule__Reaction__Group_1_1_1__0__Impl rule__Reaction__Group_1_1_1__1 ;
    public final void rule__Reaction__Group_1_1_1__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2505:1: ( rule__Reaction__Group_1_1_1__0__Impl rule__Reaction__Group_1_1_1__1 )
            // InternalLinguaFranca.g:2506:2: rule__Reaction__Group_1_1_1__0__Impl rule__Reaction__Group_1_1_1__1
            {
            pushFollow(FOLLOW_6);
            rule__Reaction__Group_1_1_1__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Reaction__Group_1_1_1__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group_1_1_1__0"


    // $ANTLR start "rule__Reaction__Group_1_1_1__0__Impl"
    // InternalLinguaFranca.g:2513:1: rule__Reaction__Group_1_1_1__0__Impl : ( ',' ) ;
    public final void rule__Reaction__Group_1_1_1__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2517:1: ( ( ',' ) )
            // InternalLinguaFranca.g:2518:1: ( ',' )
            {
            // InternalLinguaFranca.g:2518:1: ( ',' )
            // InternalLinguaFranca.g:2519:2: ','
            {
             before(grammarAccess.getReactionAccess().getCommaKeyword_1_1_1_0()); 
            match(input,27,FOLLOW_2); 
             after(grammarAccess.getReactionAccess().getCommaKeyword_1_1_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group_1_1_1__0__Impl"


    // $ANTLR start "rule__Reaction__Group_1_1_1__1"
    // InternalLinguaFranca.g:2528:1: rule__Reaction__Group_1_1_1__1 : rule__Reaction__Group_1_1_1__1__Impl ;
    public final void rule__Reaction__Group_1_1_1__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2532:1: ( rule__Reaction__Group_1_1_1__1__Impl )
            // InternalLinguaFranca.g:2533:2: rule__Reaction__Group_1_1_1__1__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Reaction__Group_1_1_1__1__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group_1_1_1__1"


    // $ANTLR start "rule__Reaction__Group_1_1_1__1__Impl"
    // InternalLinguaFranca.g:2539:1: rule__Reaction__Group_1_1_1__1__Impl : ( ( rule__Reaction__TriggersAssignment_1_1_1_1 ) ) ;
    public final void rule__Reaction__Group_1_1_1__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2543:1: ( ( ( rule__Reaction__TriggersAssignment_1_1_1_1 ) ) )
            // InternalLinguaFranca.g:2544:1: ( ( rule__Reaction__TriggersAssignment_1_1_1_1 ) )
            {
            // InternalLinguaFranca.g:2544:1: ( ( rule__Reaction__TriggersAssignment_1_1_1_1 ) )
            // InternalLinguaFranca.g:2545:2: ( rule__Reaction__TriggersAssignment_1_1_1_1 )
            {
             before(grammarAccess.getReactionAccess().getTriggersAssignment_1_1_1_1()); 
            // InternalLinguaFranca.g:2546:2: ( rule__Reaction__TriggersAssignment_1_1_1_1 )
            // InternalLinguaFranca.g:2546:3: rule__Reaction__TriggersAssignment_1_1_1_1
            {
            pushFollow(FOLLOW_2);
            rule__Reaction__TriggersAssignment_1_1_1_1();

            state._fsp--;


            }

             after(grammarAccess.getReactionAccess().getTriggersAssignment_1_1_1_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__Group_1_1_1__1__Impl"


    // $ANTLR start "rule__Preamble__Group__0"
    // InternalLinguaFranca.g:2555:1: rule__Preamble__Group__0 : rule__Preamble__Group__0__Impl rule__Preamble__Group__1 ;
    public final void rule__Preamble__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2559:1: ( rule__Preamble__Group__0__Impl rule__Preamble__Group__1 )
            // InternalLinguaFranca.g:2560:2: rule__Preamble__Group__0__Impl rule__Preamble__Group__1
            {
            pushFollow(FOLLOW_26);
            rule__Preamble__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Preamble__Group__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Preamble__Group__0"


    // $ANTLR start "rule__Preamble__Group__0__Impl"
    // InternalLinguaFranca.g:2567:1: rule__Preamble__Group__0__Impl : ( 'preamble' ) ;
    public final void rule__Preamble__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2571:1: ( ( 'preamble' ) )
            // InternalLinguaFranca.g:2572:1: ( 'preamble' )
            {
            // InternalLinguaFranca.g:2572:1: ( 'preamble' )
            // InternalLinguaFranca.g:2573:2: 'preamble'
            {
             before(grammarAccess.getPreambleAccess().getPreambleKeyword_0()); 
            match(input,28,FOLLOW_2); 
             after(grammarAccess.getPreambleAccess().getPreambleKeyword_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Preamble__Group__0__Impl"


    // $ANTLR start "rule__Preamble__Group__1"
    // InternalLinguaFranca.g:2582:1: rule__Preamble__Group__1 : rule__Preamble__Group__1__Impl ;
    public final void rule__Preamble__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2586:1: ( rule__Preamble__Group__1__Impl )
            // InternalLinguaFranca.g:2587:2: rule__Preamble__Group__1__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Preamble__Group__1__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Preamble__Group__1"


    // $ANTLR start "rule__Preamble__Group__1__Impl"
    // InternalLinguaFranca.g:2593:1: rule__Preamble__Group__1__Impl : ( ( rule__Preamble__CodeAssignment_1 ) ) ;
    public final void rule__Preamble__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2597:1: ( ( ( rule__Preamble__CodeAssignment_1 ) ) )
            // InternalLinguaFranca.g:2598:1: ( ( rule__Preamble__CodeAssignment_1 ) )
            {
            // InternalLinguaFranca.g:2598:1: ( ( rule__Preamble__CodeAssignment_1 ) )
            // InternalLinguaFranca.g:2599:2: ( rule__Preamble__CodeAssignment_1 )
            {
             before(grammarAccess.getPreambleAccess().getCodeAssignment_1()); 
            // InternalLinguaFranca.g:2600:2: ( rule__Preamble__CodeAssignment_1 )
            // InternalLinguaFranca.g:2600:3: rule__Preamble__CodeAssignment_1
            {
            pushFollow(FOLLOW_2);
            rule__Preamble__CodeAssignment_1();

            state._fsp--;


            }

             after(grammarAccess.getPreambleAccess().getCodeAssignment_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Preamble__Group__1__Impl"


    // $ANTLR start "rule__Constructor__Group__0"
    // InternalLinguaFranca.g:2609:1: rule__Constructor__Group__0 : rule__Constructor__Group__0__Impl rule__Constructor__Group__1 ;
    public final void rule__Constructor__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2613:1: ( rule__Constructor__Group__0__Impl rule__Constructor__Group__1 )
            // InternalLinguaFranca.g:2614:2: rule__Constructor__Group__0__Impl rule__Constructor__Group__1
            {
            pushFollow(FOLLOW_26);
            rule__Constructor__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Constructor__Group__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Constructor__Group__0"


    // $ANTLR start "rule__Constructor__Group__0__Impl"
    // InternalLinguaFranca.g:2621:1: rule__Constructor__Group__0__Impl : ( 'constructor' ) ;
    public final void rule__Constructor__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2625:1: ( ( 'constructor' ) )
            // InternalLinguaFranca.g:2626:1: ( 'constructor' )
            {
            // InternalLinguaFranca.g:2626:1: ( 'constructor' )
            // InternalLinguaFranca.g:2627:2: 'constructor'
            {
             before(grammarAccess.getConstructorAccess().getConstructorKeyword_0()); 
            match(input,29,FOLLOW_2); 
             after(grammarAccess.getConstructorAccess().getConstructorKeyword_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Constructor__Group__0__Impl"


    // $ANTLR start "rule__Constructor__Group__1"
    // InternalLinguaFranca.g:2636:1: rule__Constructor__Group__1 : rule__Constructor__Group__1__Impl ;
    public final void rule__Constructor__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2640:1: ( rule__Constructor__Group__1__Impl )
            // InternalLinguaFranca.g:2641:2: rule__Constructor__Group__1__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Constructor__Group__1__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Constructor__Group__1"


    // $ANTLR start "rule__Constructor__Group__1__Impl"
    // InternalLinguaFranca.g:2647:1: rule__Constructor__Group__1__Impl : ( ( rule__Constructor__CodeAssignment_1 ) ) ;
    public final void rule__Constructor__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2651:1: ( ( ( rule__Constructor__CodeAssignment_1 ) ) )
            // InternalLinguaFranca.g:2652:1: ( ( rule__Constructor__CodeAssignment_1 ) )
            {
            // InternalLinguaFranca.g:2652:1: ( ( rule__Constructor__CodeAssignment_1 ) )
            // InternalLinguaFranca.g:2653:2: ( rule__Constructor__CodeAssignment_1 )
            {
             before(grammarAccess.getConstructorAccess().getCodeAssignment_1()); 
            // InternalLinguaFranca.g:2654:2: ( rule__Constructor__CodeAssignment_1 )
            // InternalLinguaFranca.g:2654:3: rule__Constructor__CodeAssignment_1
            {
            pushFollow(FOLLOW_2);
            rule__Constructor__CodeAssignment_1();

            state._fsp--;


            }

             after(grammarAccess.getConstructorAccess().getCodeAssignment_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Constructor__Group__1__Impl"


    // $ANTLR start "rule__Instance__Group__0"
    // InternalLinguaFranca.g:2663:1: rule__Instance__Group__0 : rule__Instance__Group__0__Impl rule__Instance__Group__1 ;
    public final void rule__Instance__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2667:1: ( rule__Instance__Group__0__Impl rule__Instance__Group__1 )
            // InternalLinguaFranca.g:2668:2: rule__Instance__Group__0__Impl rule__Instance__Group__1
            {
            pushFollow(FOLLOW_27);
            rule__Instance__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Instance__Group__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Instance__Group__0"


    // $ANTLR start "rule__Instance__Group__0__Impl"
    // InternalLinguaFranca.g:2675:1: rule__Instance__Group__0__Impl : ( ( rule__Instance__NameAssignment_0 ) ) ;
    public final void rule__Instance__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2679:1: ( ( ( rule__Instance__NameAssignment_0 ) ) )
            // InternalLinguaFranca.g:2680:1: ( ( rule__Instance__NameAssignment_0 ) )
            {
            // InternalLinguaFranca.g:2680:1: ( ( rule__Instance__NameAssignment_0 ) )
            // InternalLinguaFranca.g:2681:2: ( rule__Instance__NameAssignment_0 )
            {
             before(grammarAccess.getInstanceAccess().getNameAssignment_0()); 
            // InternalLinguaFranca.g:2682:2: ( rule__Instance__NameAssignment_0 )
            // InternalLinguaFranca.g:2682:3: rule__Instance__NameAssignment_0
            {
            pushFollow(FOLLOW_2);
            rule__Instance__NameAssignment_0();

            state._fsp--;


            }

             after(grammarAccess.getInstanceAccess().getNameAssignment_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Instance__Group__0__Impl"


    // $ANTLR start "rule__Instance__Group__1"
    // InternalLinguaFranca.g:2690:1: rule__Instance__Group__1 : rule__Instance__Group__1__Impl rule__Instance__Group__2 ;
    public final void rule__Instance__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2694:1: ( rule__Instance__Group__1__Impl rule__Instance__Group__2 )
            // InternalLinguaFranca.g:2695:2: rule__Instance__Group__1__Impl rule__Instance__Group__2
            {
            pushFollow(FOLLOW_28);
            rule__Instance__Group__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Instance__Group__2();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Instance__Group__1"


    // $ANTLR start "rule__Instance__Group__1__Impl"
    // InternalLinguaFranca.g:2702:1: rule__Instance__Group__1__Impl : ( '=' ) ;
    public final void rule__Instance__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2706:1: ( ( '=' ) )
            // InternalLinguaFranca.g:2707:1: ( '=' )
            {
            // InternalLinguaFranca.g:2707:1: ( '=' )
            // InternalLinguaFranca.g:2708:2: '='
            {
             before(grammarAccess.getInstanceAccess().getEqualsSignKeyword_1()); 
            match(input,30,FOLLOW_2); 
             after(grammarAccess.getInstanceAccess().getEqualsSignKeyword_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Instance__Group__1__Impl"


    // $ANTLR start "rule__Instance__Group__2"
    // InternalLinguaFranca.g:2717:1: rule__Instance__Group__2 : rule__Instance__Group__2__Impl rule__Instance__Group__3 ;
    public final void rule__Instance__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2721:1: ( rule__Instance__Group__2__Impl rule__Instance__Group__3 )
            // InternalLinguaFranca.g:2722:2: rule__Instance__Group__2__Impl rule__Instance__Group__3
            {
            pushFollow(FOLLOW_6);
            rule__Instance__Group__2__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Instance__Group__3();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Instance__Group__2"


    // $ANTLR start "rule__Instance__Group__2__Impl"
    // InternalLinguaFranca.g:2729:1: rule__Instance__Group__2__Impl : ( 'new' ) ;
    public final void rule__Instance__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2733:1: ( ( 'new' ) )
            // InternalLinguaFranca.g:2734:1: ( 'new' )
            {
            // InternalLinguaFranca.g:2734:1: ( 'new' )
            // InternalLinguaFranca.g:2735:2: 'new'
            {
             before(grammarAccess.getInstanceAccess().getNewKeyword_2()); 
            match(input,31,FOLLOW_2); 
             after(grammarAccess.getInstanceAccess().getNewKeyword_2()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Instance__Group__2__Impl"


    // $ANTLR start "rule__Instance__Group__3"
    // InternalLinguaFranca.g:2744:1: rule__Instance__Group__3 : rule__Instance__Group__3__Impl rule__Instance__Group__4 ;
    public final void rule__Instance__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2748:1: ( rule__Instance__Group__3__Impl rule__Instance__Group__4 )
            // InternalLinguaFranca.g:2749:2: rule__Instance__Group__3__Impl rule__Instance__Group__4
            {
            pushFollow(FOLLOW_21);
            rule__Instance__Group__3__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Instance__Group__4();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Instance__Group__3"


    // $ANTLR start "rule__Instance__Group__3__Impl"
    // InternalLinguaFranca.g:2756:1: rule__Instance__Group__3__Impl : ( ( rule__Instance__ActorClassAssignment_3 ) ) ;
    public final void rule__Instance__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2760:1: ( ( ( rule__Instance__ActorClassAssignment_3 ) ) )
            // InternalLinguaFranca.g:2761:1: ( ( rule__Instance__ActorClassAssignment_3 ) )
            {
            // InternalLinguaFranca.g:2761:1: ( ( rule__Instance__ActorClassAssignment_3 ) )
            // InternalLinguaFranca.g:2762:2: ( rule__Instance__ActorClassAssignment_3 )
            {
             before(grammarAccess.getInstanceAccess().getActorClassAssignment_3()); 
            // InternalLinguaFranca.g:2763:2: ( rule__Instance__ActorClassAssignment_3 )
            // InternalLinguaFranca.g:2763:3: rule__Instance__ActorClassAssignment_3
            {
            pushFollow(FOLLOW_2);
            rule__Instance__ActorClassAssignment_3();

            state._fsp--;


            }

             after(grammarAccess.getInstanceAccess().getActorClassAssignment_3()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Instance__Group__3__Impl"


    // $ANTLR start "rule__Instance__Group__4"
    // InternalLinguaFranca.g:2771:1: rule__Instance__Group__4 : rule__Instance__Group__4__Impl rule__Instance__Group__5 ;
    public final void rule__Instance__Group__4() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2775:1: ( rule__Instance__Group__4__Impl rule__Instance__Group__5 )
            // InternalLinguaFranca.g:2776:2: rule__Instance__Group__4__Impl rule__Instance__Group__5
            {
            pushFollow(FOLLOW_21);
            rule__Instance__Group__4__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Instance__Group__5();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Instance__Group__4"


    // $ANTLR start "rule__Instance__Group__4__Impl"
    // InternalLinguaFranca.g:2783:1: rule__Instance__Group__4__Impl : ( ( rule__Instance__Group_4__0 )? ) ;
    public final void rule__Instance__Group__4__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2787:1: ( ( ( rule__Instance__Group_4__0 )? ) )
            // InternalLinguaFranca.g:2788:1: ( ( rule__Instance__Group_4__0 )? )
            {
            // InternalLinguaFranca.g:2788:1: ( ( rule__Instance__Group_4__0 )? )
            // InternalLinguaFranca.g:2789:2: ( rule__Instance__Group_4__0 )?
            {
             before(grammarAccess.getInstanceAccess().getGroup_4()); 
            // InternalLinguaFranca.g:2790:2: ( rule__Instance__Group_4__0 )?
            int alt38=2;
            int LA38_0 = input.LA(1);

            if ( (LA38_0==25) ) {
                alt38=1;
            }
            switch (alt38) {
                case 1 :
                    // InternalLinguaFranca.g:2790:3: rule__Instance__Group_4__0
                    {
                    pushFollow(FOLLOW_2);
                    rule__Instance__Group_4__0();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getInstanceAccess().getGroup_4()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Instance__Group__4__Impl"


    // $ANTLR start "rule__Instance__Group__5"
    // InternalLinguaFranca.g:2798:1: rule__Instance__Group__5 : rule__Instance__Group__5__Impl ;
    public final void rule__Instance__Group__5() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2802:1: ( rule__Instance__Group__5__Impl )
            // InternalLinguaFranca.g:2803:2: rule__Instance__Group__5__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Instance__Group__5__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Instance__Group__5"


    // $ANTLR start "rule__Instance__Group__5__Impl"
    // InternalLinguaFranca.g:2809:1: rule__Instance__Group__5__Impl : ( ';' ) ;
    public final void rule__Instance__Group__5__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2813:1: ( ( ';' ) )
            // InternalLinguaFranca.g:2814:1: ( ';' )
            {
            // InternalLinguaFranca.g:2814:1: ( ';' )
            // InternalLinguaFranca.g:2815:2: ';'
            {
             before(grammarAccess.getInstanceAccess().getSemicolonKeyword_5()); 
            match(input,17,FOLLOW_2); 
             after(grammarAccess.getInstanceAccess().getSemicolonKeyword_5()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Instance__Group__5__Impl"


    // $ANTLR start "rule__Instance__Group_4__0"
    // InternalLinguaFranca.g:2825:1: rule__Instance__Group_4__0 : rule__Instance__Group_4__0__Impl rule__Instance__Group_4__1 ;
    public final void rule__Instance__Group_4__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2829:1: ( rule__Instance__Group_4__0__Impl rule__Instance__Group_4__1 )
            // InternalLinguaFranca.g:2830:2: rule__Instance__Group_4__0__Impl rule__Instance__Group_4__1
            {
            pushFollow(FOLLOW_23);
            rule__Instance__Group_4__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Instance__Group_4__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Instance__Group_4__0"


    // $ANTLR start "rule__Instance__Group_4__0__Impl"
    // InternalLinguaFranca.g:2837:1: rule__Instance__Group_4__0__Impl : ( '(' ) ;
    public final void rule__Instance__Group_4__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2841:1: ( ( '(' ) )
            // InternalLinguaFranca.g:2842:1: ( '(' )
            {
            // InternalLinguaFranca.g:2842:1: ( '(' )
            // InternalLinguaFranca.g:2843:2: '('
            {
             before(grammarAccess.getInstanceAccess().getLeftParenthesisKeyword_4_0()); 
            match(input,25,FOLLOW_2); 
             after(grammarAccess.getInstanceAccess().getLeftParenthesisKeyword_4_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Instance__Group_4__0__Impl"


    // $ANTLR start "rule__Instance__Group_4__1"
    // InternalLinguaFranca.g:2852:1: rule__Instance__Group_4__1 : rule__Instance__Group_4__1__Impl rule__Instance__Group_4__2 ;
    public final void rule__Instance__Group_4__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2856:1: ( rule__Instance__Group_4__1__Impl rule__Instance__Group_4__2 )
            // InternalLinguaFranca.g:2857:2: rule__Instance__Group_4__1__Impl rule__Instance__Group_4__2
            {
            pushFollow(FOLLOW_23);
            rule__Instance__Group_4__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Instance__Group_4__2();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Instance__Group_4__1"


    // $ANTLR start "rule__Instance__Group_4__1__Impl"
    // InternalLinguaFranca.g:2864:1: rule__Instance__Group_4__1__Impl : ( ( rule__Instance__ParametersAssignment_4_1 )? ) ;
    public final void rule__Instance__Group_4__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2868:1: ( ( ( rule__Instance__ParametersAssignment_4_1 )? ) )
            // InternalLinguaFranca.g:2869:1: ( ( rule__Instance__ParametersAssignment_4_1 )? )
            {
            // InternalLinguaFranca.g:2869:1: ( ( rule__Instance__ParametersAssignment_4_1 )? )
            // InternalLinguaFranca.g:2870:2: ( rule__Instance__ParametersAssignment_4_1 )?
            {
             before(grammarAccess.getInstanceAccess().getParametersAssignment_4_1()); 
            // InternalLinguaFranca.g:2871:2: ( rule__Instance__ParametersAssignment_4_1 )?
            int alt39=2;
            int LA39_0 = input.LA(1);

            if ( (LA39_0==RULE_ID) ) {
                alt39=1;
            }
            switch (alt39) {
                case 1 :
                    // InternalLinguaFranca.g:2871:3: rule__Instance__ParametersAssignment_4_1
                    {
                    pushFollow(FOLLOW_2);
                    rule__Instance__ParametersAssignment_4_1();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getInstanceAccess().getParametersAssignment_4_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Instance__Group_4__1__Impl"


    // $ANTLR start "rule__Instance__Group_4__2"
    // InternalLinguaFranca.g:2879:1: rule__Instance__Group_4__2 : rule__Instance__Group_4__2__Impl ;
    public final void rule__Instance__Group_4__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2883:1: ( rule__Instance__Group_4__2__Impl )
            // InternalLinguaFranca.g:2884:2: rule__Instance__Group_4__2__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Instance__Group_4__2__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Instance__Group_4__2"


    // $ANTLR start "rule__Instance__Group_4__2__Impl"
    // InternalLinguaFranca.g:2890:1: rule__Instance__Group_4__2__Impl : ( ')' ) ;
    public final void rule__Instance__Group_4__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2894:1: ( ( ')' ) )
            // InternalLinguaFranca.g:2895:1: ( ')' )
            {
            // InternalLinguaFranca.g:2895:1: ( ')' )
            // InternalLinguaFranca.g:2896:2: ')'
            {
             before(grammarAccess.getInstanceAccess().getRightParenthesisKeyword_4_2()); 
            match(input,26,FOLLOW_2); 
             after(grammarAccess.getInstanceAccess().getRightParenthesisKeyword_4_2()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Instance__Group_4__2__Impl"


    // $ANTLR start "rule__Connection__Group__0"
    // InternalLinguaFranca.g:2906:1: rule__Connection__Group__0 : rule__Connection__Group__0__Impl rule__Connection__Group__1 ;
    public final void rule__Connection__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2910:1: ( rule__Connection__Group__0__Impl rule__Connection__Group__1 )
            // InternalLinguaFranca.g:2911:2: rule__Connection__Group__0__Impl rule__Connection__Group__1
            {
            pushFollow(FOLLOW_29);
            rule__Connection__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Connection__Group__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Connection__Group__0"


    // $ANTLR start "rule__Connection__Group__0__Impl"
    // InternalLinguaFranca.g:2918:1: rule__Connection__Group__0__Impl : ( ( rule__Connection__LeftPortAssignment_0 ) ) ;
    public final void rule__Connection__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2922:1: ( ( ( rule__Connection__LeftPortAssignment_0 ) ) )
            // InternalLinguaFranca.g:2923:1: ( ( rule__Connection__LeftPortAssignment_0 ) )
            {
            // InternalLinguaFranca.g:2923:1: ( ( rule__Connection__LeftPortAssignment_0 ) )
            // InternalLinguaFranca.g:2924:2: ( rule__Connection__LeftPortAssignment_0 )
            {
             before(grammarAccess.getConnectionAccess().getLeftPortAssignment_0()); 
            // InternalLinguaFranca.g:2925:2: ( rule__Connection__LeftPortAssignment_0 )
            // InternalLinguaFranca.g:2925:3: rule__Connection__LeftPortAssignment_0
            {
            pushFollow(FOLLOW_2);
            rule__Connection__LeftPortAssignment_0();

            state._fsp--;


            }

             after(grammarAccess.getConnectionAccess().getLeftPortAssignment_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Connection__Group__0__Impl"


    // $ANTLR start "rule__Connection__Group__1"
    // InternalLinguaFranca.g:2933:1: rule__Connection__Group__1 : rule__Connection__Group__1__Impl rule__Connection__Group__2 ;
    public final void rule__Connection__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2937:1: ( rule__Connection__Group__1__Impl rule__Connection__Group__2 )
            // InternalLinguaFranca.g:2938:2: rule__Connection__Group__1__Impl rule__Connection__Group__2
            {
            pushFollow(FOLLOW_6);
            rule__Connection__Group__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Connection__Group__2();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Connection__Group__1"


    // $ANTLR start "rule__Connection__Group__1__Impl"
    // InternalLinguaFranca.g:2945:1: rule__Connection__Group__1__Impl : ( '->' ) ;
    public final void rule__Connection__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2949:1: ( ( '->' ) )
            // InternalLinguaFranca.g:2950:1: ( '->' )
            {
            // InternalLinguaFranca.g:2950:1: ( '->' )
            // InternalLinguaFranca.g:2951:2: '->'
            {
             before(grammarAccess.getConnectionAccess().getHyphenMinusGreaterThanSignKeyword_1()); 
            match(input,32,FOLLOW_2); 
             after(grammarAccess.getConnectionAccess().getHyphenMinusGreaterThanSignKeyword_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Connection__Group__1__Impl"


    // $ANTLR start "rule__Connection__Group__2"
    // InternalLinguaFranca.g:2960:1: rule__Connection__Group__2 : rule__Connection__Group__2__Impl rule__Connection__Group__3 ;
    public final void rule__Connection__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2964:1: ( rule__Connection__Group__2__Impl rule__Connection__Group__3 )
            // InternalLinguaFranca.g:2965:2: rule__Connection__Group__2__Impl rule__Connection__Group__3
            {
            pushFollow(FOLLOW_7);
            rule__Connection__Group__2__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Connection__Group__3();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Connection__Group__2"


    // $ANTLR start "rule__Connection__Group__2__Impl"
    // InternalLinguaFranca.g:2972:1: rule__Connection__Group__2__Impl : ( ( rule__Connection__RightPortAssignment_2 ) ) ;
    public final void rule__Connection__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2976:1: ( ( ( rule__Connection__RightPortAssignment_2 ) ) )
            // InternalLinguaFranca.g:2977:1: ( ( rule__Connection__RightPortAssignment_2 ) )
            {
            // InternalLinguaFranca.g:2977:1: ( ( rule__Connection__RightPortAssignment_2 ) )
            // InternalLinguaFranca.g:2978:2: ( rule__Connection__RightPortAssignment_2 )
            {
             before(grammarAccess.getConnectionAccess().getRightPortAssignment_2()); 
            // InternalLinguaFranca.g:2979:2: ( rule__Connection__RightPortAssignment_2 )
            // InternalLinguaFranca.g:2979:3: rule__Connection__RightPortAssignment_2
            {
            pushFollow(FOLLOW_2);
            rule__Connection__RightPortAssignment_2();

            state._fsp--;


            }

             after(grammarAccess.getConnectionAccess().getRightPortAssignment_2()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Connection__Group__2__Impl"


    // $ANTLR start "rule__Connection__Group__3"
    // InternalLinguaFranca.g:2987:1: rule__Connection__Group__3 : rule__Connection__Group__3__Impl ;
    public final void rule__Connection__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2991:1: ( rule__Connection__Group__3__Impl )
            // InternalLinguaFranca.g:2992:2: rule__Connection__Group__3__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Connection__Group__3__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Connection__Group__3"


    // $ANTLR start "rule__Connection__Group__3__Impl"
    // InternalLinguaFranca.g:2998:1: rule__Connection__Group__3__Impl : ( ';' ) ;
    public final void rule__Connection__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3002:1: ( ( ';' ) )
            // InternalLinguaFranca.g:3003:1: ( ';' )
            {
            // InternalLinguaFranca.g:3003:1: ( ';' )
            // InternalLinguaFranca.g:3004:2: ';'
            {
             before(grammarAccess.getConnectionAccess().getSemicolonKeyword_3()); 
            match(input,17,FOLLOW_2); 
             after(grammarAccess.getConnectionAccess().getSemicolonKeyword_3()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Connection__Group__3__Impl"


    // $ANTLR start "rule__Assignments__Group__0"
    // InternalLinguaFranca.g:3014:1: rule__Assignments__Group__0 : rule__Assignments__Group__0__Impl rule__Assignments__Group__1 ;
    public final void rule__Assignments__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3018:1: ( rule__Assignments__Group__0__Impl rule__Assignments__Group__1 )
            // InternalLinguaFranca.g:3019:2: rule__Assignments__Group__0__Impl rule__Assignments__Group__1
            {
            pushFollow(FOLLOW_24);
            rule__Assignments__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Assignments__Group__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Assignments__Group__0"


    // $ANTLR start "rule__Assignments__Group__0__Impl"
    // InternalLinguaFranca.g:3026:1: rule__Assignments__Group__0__Impl : ( ( rule__Assignments__AssignmentsAssignment_0 ) ) ;
    public final void rule__Assignments__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3030:1: ( ( ( rule__Assignments__AssignmentsAssignment_0 ) ) )
            // InternalLinguaFranca.g:3031:1: ( ( rule__Assignments__AssignmentsAssignment_0 ) )
            {
            // InternalLinguaFranca.g:3031:1: ( ( rule__Assignments__AssignmentsAssignment_0 ) )
            // InternalLinguaFranca.g:3032:2: ( rule__Assignments__AssignmentsAssignment_0 )
            {
             before(grammarAccess.getAssignmentsAccess().getAssignmentsAssignment_0()); 
            // InternalLinguaFranca.g:3033:2: ( rule__Assignments__AssignmentsAssignment_0 )
            // InternalLinguaFranca.g:3033:3: rule__Assignments__AssignmentsAssignment_0
            {
            pushFollow(FOLLOW_2);
            rule__Assignments__AssignmentsAssignment_0();

            state._fsp--;


            }

             after(grammarAccess.getAssignmentsAccess().getAssignmentsAssignment_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Assignments__Group__0__Impl"


    // $ANTLR start "rule__Assignments__Group__1"
    // InternalLinguaFranca.g:3041:1: rule__Assignments__Group__1 : rule__Assignments__Group__1__Impl ;
    public final void rule__Assignments__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3045:1: ( rule__Assignments__Group__1__Impl )
            // InternalLinguaFranca.g:3046:2: rule__Assignments__Group__1__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Assignments__Group__1__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Assignments__Group__1"


    // $ANTLR start "rule__Assignments__Group__1__Impl"
    // InternalLinguaFranca.g:3052:1: rule__Assignments__Group__1__Impl : ( ( rule__Assignments__Group_1__0 )* ) ;
    public final void rule__Assignments__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3056:1: ( ( ( rule__Assignments__Group_1__0 )* ) )
            // InternalLinguaFranca.g:3057:1: ( ( rule__Assignments__Group_1__0 )* )
            {
            // InternalLinguaFranca.g:3057:1: ( ( rule__Assignments__Group_1__0 )* )
            // InternalLinguaFranca.g:3058:2: ( rule__Assignments__Group_1__0 )*
            {
             before(grammarAccess.getAssignmentsAccess().getGroup_1()); 
            // InternalLinguaFranca.g:3059:2: ( rule__Assignments__Group_1__0 )*
            loop40:
            do {
                int alt40=2;
                int LA40_0 = input.LA(1);

                if ( (LA40_0==27) ) {
                    alt40=1;
                }


                switch (alt40) {
            	case 1 :
            	    // InternalLinguaFranca.g:3059:3: rule__Assignments__Group_1__0
            	    {
            	    pushFollow(FOLLOW_25);
            	    rule__Assignments__Group_1__0();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop40;
                }
            } while (true);

             after(grammarAccess.getAssignmentsAccess().getGroup_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Assignments__Group__1__Impl"


    // $ANTLR start "rule__Assignments__Group_1__0"
    // InternalLinguaFranca.g:3068:1: rule__Assignments__Group_1__0 : rule__Assignments__Group_1__0__Impl rule__Assignments__Group_1__1 ;
    public final void rule__Assignments__Group_1__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3072:1: ( rule__Assignments__Group_1__0__Impl rule__Assignments__Group_1__1 )
            // InternalLinguaFranca.g:3073:2: rule__Assignments__Group_1__0__Impl rule__Assignments__Group_1__1
            {
            pushFollow(FOLLOW_6);
            rule__Assignments__Group_1__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Assignments__Group_1__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Assignments__Group_1__0"


    // $ANTLR start "rule__Assignments__Group_1__0__Impl"
    // InternalLinguaFranca.g:3080:1: rule__Assignments__Group_1__0__Impl : ( ',' ) ;
    public final void rule__Assignments__Group_1__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3084:1: ( ( ',' ) )
            // InternalLinguaFranca.g:3085:1: ( ',' )
            {
            // InternalLinguaFranca.g:3085:1: ( ',' )
            // InternalLinguaFranca.g:3086:2: ','
            {
             before(grammarAccess.getAssignmentsAccess().getCommaKeyword_1_0()); 
            match(input,27,FOLLOW_2); 
             after(grammarAccess.getAssignmentsAccess().getCommaKeyword_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Assignments__Group_1__0__Impl"


    // $ANTLR start "rule__Assignments__Group_1__1"
    // InternalLinguaFranca.g:3095:1: rule__Assignments__Group_1__1 : rule__Assignments__Group_1__1__Impl ;
    public final void rule__Assignments__Group_1__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3099:1: ( rule__Assignments__Group_1__1__Impl )
            // InternalLinguaFranca.g:3100:2: rule__Assignments__Group_1__1__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Assignments__Group_1__1__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Assignments__Group_1__1"


    // $ANTLR start "rule__Assignments__Group_1__1__Impl"
    // InternalLinguaFranca.g:3106:1: rule__Assignments__Group_1__1__Impl : ( ( rule__Assignments__AssignmentsAssignment_1_1 ) ) ;
    public final void rule__Assignments__Group_1__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3110:1: ( ( ( rule__Assignments__AssignmentsAssignment_1_1 ) ) )
            // InternalLinguaFranca.g:3111:1: ( ( rule__Assignments__AssignmentsAssignment_1_1 ) )
            {
            // InternalLinguaFranca.g:3111:1: ( ( rule__Assignments__AssignmentsAssignment_1_1 ) )
            // InternalLinguaFranca.g:3112:2: ( rule__Assignments__AssignmentsAssignment_1_1 )
            {
             before(grammarAccess.getAssignmentsAccess().getAssignmentsAssignment_1_1()); 
            // InternalLinguaFranca.g:3113:2: ( rule__Assignments__AssignmentsAssignment_1_1 )
            // InternalLinguaFranca.g:3113:3: rule__Assignments__AssignmentsAssignment_1_1
            {
            pushFollow(FOLLOW_2);
            rule__Assignments__AssignmentsAssignment_1_1();

            state._fsp--;


            }

             after(grammarAccess.getAssignmentsAccess().getAssignmentsAssignment_1_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Assignments__Group_1__1__Impl"


    // $ANTLR start "rule__Assignment__Group__0"
    // InternalLinguaFranca.g:3122:1: rule__Assignment__Group__0 : rule__Assignment__Group__0__Impl rule__Assignment__Group__1 ;
    public final void rule__Assignment__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3126:1: ( rule__Assignment__Group__0__Impl rule__Assignment__Group__1 )
            // InternalLinguaFranca.g:3127:2: rule__Assignment__Group__0__Impl rule__Assignment__Group__1
            {
            pushFollow(FOLLOW_27);
            rule__Assignment__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Assignment__Group__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Assignment__Group__0"


    // $ANTLR start "rule__Assignment__Group__0__Impl"
    // InternalLinguaFranca.g:3134:1: rule__Assignment__Group__0__Impl : ( ( rule__Assignment__NameAssignment_0 ) ) ;
    public final void rule__Assignment__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3138:1: ( ( ( rule__Assignment__NameAssignment_0 ) ) )
            // InternalLinguaFranca.g:3139:1: ( ( rule__Assignment__NameAssignment_0 ) )
            {
            // InternalLinguaFranca.g:3139:1: ( ( rule__Assignment__NameAssignment_0 ) )
            // InternalLinguaFranca.g:3140:2: ( rule__Assignment__NameAssignment_0 )
            {
             before(grammarAccess.getAssignmentAccess().getNameAssignment_0()); 
            // InternalLinguaFranca.g:3141:2: ( rule__Assignment__NameAssignment_0 )
            // InternalLinguaFranca.g:3141:3: rule__Assignment__NameAssignment_0
            {
            pushFollow(FOLLOW_2);
            rule__Assignment__NameAssignment_0();

            state._fsp--;


            }

             after(grammarAccess.getAssignmentAccess().getNameAssignment_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Assignment__Group__0__Impl"


    // $ANTLR start "rule__Assignment__Group__1"
    // InternalLinguaFranca.g:3149:1: rule__Assignment__Group__1 : rule__Assignment__Group__1__Impl rule__Assignment__Group__2 ;
    public final void rule__Assignment__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3153:1: ( rule__Assignment__Group__1__Impl rule__Assignment__Group__2 )
            // InternalLinguaFranca.g:3154:2: rule__Assignment__Group__1__Impl rule__Assignment__Group__2
            {
            pushFollow(FOLLOW_30);
            rule__Assignment__Group__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Assignment__Group__2();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Assignment__Group__1"


    // $ANTLR start "rule__Assignment__Group__1__Impl"
    // InternalLinguaFranca.g:3161:1: rule__Assignment__Group__1__Impl : ( '=' ) ;
    public final void rule__Assignment__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3165:1: ( ( '=' ) )
            // InternalLinguaFranca.g:3166:1: ( '=' )
            {
            // InternalLinguaFranca.g:3166:1: ( '=' )
            // InternalLinguaFranca.g:3167:2: '='
            {
             before(grammarAccess.getAssignmentAccess().getEqualsSignKeyword_1()); 
            match(input,30,FOLLOW_2); 
             after(grammarAccess.getAssignmentAccess().getEqualsSignKeyword_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Assignment__Group__1__Impl"


    // $ANTLR start "rule__Assignment__Group__2"
    // InternalLinguaFranca.g:3176:1: rule__Assignment__Group__2 : rule__Assignment__Group__2__Impl ;
    public final void rule__Assignment__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3180:1: ( rule__Assignment__Group__2__Impl )
            // InternalLinguaFranca.g:3181:2: rule__Assignment__Group__2__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Assignment__Group__2__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Assignment__Group__2"


    // $ANTLR start "rule__Assignment__Group__2__Impl"
    // InternalLinguaFranca.g:3187:1: rule__Assignment__Group__2__Impl : ( ( rule__Assignment__ValueAssignment_2 ) ) ;
    public final void rule__Assignment__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3191:1: ( ( ( rule__Assignment__ValueAssignment_2 ) ) )
            // InternalLinguaFranca.g:3192:1: ( ( rule__Assignment__ValueAssignment_2 ) )
            {
            // InternalLinguaFranca.g:3192:1: ( ( rule__Assignment__ValueAssignment_2 ) )
            // InternalLinguaFranca.g:3193:2: ( rule__Assignment__ValueAssignment_2 )
            {
             before(grammarAccess.getAssignmentAccess().getValueAssignment_2()); 
            // InternalLinguaFranca.g:3194:2: ( rule__Assignment__ValueAssignment_2 )
            // InternalLinguaFranca.g:3194:3: rule__Assignment__ValueAssignment_2
            {
            pushFollow(FOLLOW_2);
            rule__Assignment__ValueAssignment_2();

            state._fsp--;


            }

             after(grammarAccess.getAssignmentAccess().getValueAssignment_2()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Assignment__Group__2__Impl"


    // $ANTLR start "rule__Gets__Group__0"
    // InternalLinguaFranca.g:3203:1: rule__Gets__Group__0 : rule__Gets__Group__0__Impl rule__Gets__Group__1 ;
    public final void rule__Gets__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3207:1: ( rule__Gets__Group__0__Impl rule__Gets__Group__1 )
            // InternalLinguaFranca.g:3208:2: rule__Gets__Group__0__Impl rule__Gets__Group__1
            {
            pushFollow(FOLLOW_24);
            rule__Gets__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Gets__Group__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Gets__Group__0"


    // $ANTLR start "rule__Gets__Group__0__Impl"
    // InternalLinguaFranca.g:3215:1: rule__Gets__Group__0__Impl : ( ( rule__Gets__GetsAssignment_0 ) ) ;
    public final void rule__Gets__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3219:1: ( ( ( rule__Gets__GetsAssignment_0 ) ) )
            // InternalLinguaFranca.g:3220:1: ( ( rule__Gets__GetsAssignment_0 ) )
            {
            // InternalLinguaFranca.g:3220:1: ( ( rule__Gets__GetsAssignment_0 ) )
            // InternalLinguaFranca.g:3221:2: ( rule__Gets__GetsAssignment_0 )
            {
             before(grammarAccess.getGetsAccess().getGetsAssignment_0()); 
            // InternalLinguaFranca.g:3222:2: ( rule__Gets__GetsAssignment_0 )
            // InternalLinguaFranca.g:3222:3: rule__Gets__GetsAssignment_0
            {
            pushFollow(FOLLOW_2);
            rule__Gets__GetsAssignment_0();

            state._fsp--;


            }

             after(grammarAccess.getGetsAccess().getGetsAssignment_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Gets__Group__0__Impl"


    // $ANTLR start "rule__Gets__Group__1"
    // InternalLinguaFranca.g:3230:1: rule__Gets__Group__1 : rule__Gets__Group__1__Impl ;
    public final void rule__Gets__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3234:1: ( rule__Gets__Group__1__Impl )
            // InternalLinguaFranca.g:3235:2: rule__Gets__Group__1__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Gets__Group__1__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Gets__Group__1"


    // $ANTLR start "rule__Gets__Group__1__Impl"
    // InternalLinguaFranca.g:3241:1: rule__Gets__Group__1__Impl : ( ( rule__Gets__Group_1__0 )? ) ;
    public final void rule__Gets__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3245:1: ( ( ( rule__Gets__Group_1__0 )? ) )
            // InternalLinguaFranca.g:3246:1: ( ( rule__Gets__Group_1__0 )? )
            {
            // InternalLinguaFranca.g:3246:1: ( ( rule__Gets__Group_1__0 )? )
            // InternalLinguaFranca.g:3247:2: ( rule__Gets__Group_1__0 )?
            {
             before(grammarAccess.getGetsAccess().getGroup_1()); 
            // InternalLinguaFranca.g:3248:2: ( rule__Gets__Group_1__0 )?
            int alt41=2;
            int LA41_0 = input.LA(1);

            if ( (LA41_0==27) ) {
                alt41=1;
            }
            switch (alt41) {
                case 1 :
                    // InternalLinguaFranca.g:3248:3: rule__Gets__Group_1__0
                    {
                    pushFollow(FOLLOW_2);
                    rule__Gets__Group_1__0();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getGetsAccess().getGroup_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Gets__Group__1__Impl"


    // $ANTLR start "rule__Gets__Group_1__0"
    // InternalLinguaFranca.g:3257:1: rule__Gets__Group_1__0 : rule__Gets__Group_1__0__Impl rule__Gets__Group_1__1 ;
    public final void rule__Gets__Group_1__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3261:1: ( rule__Gets__Group_1__0__Impl rule__Gets__Group_1__1 )
            // InternalLinguaFranca.g:3262:2: rule__Gets__Group_1__0__Impl rule__Gets__Group_1__1
            {
            pushFollow(FOLLOW_6);
            rule__Gets__Group_1__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Gets__Group_1__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Gets__Group_1__0"


    // $ANTLR start "rule__Gets__Group_1__0__Impl"
    // InternalLinguaFranca.g:3269:1: rule__Gets__Group_1__0__Impl : ( ',' ) ;
    public final void rule__Gets__Group_1__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3273:1: ( ( ',' ) )
            // InternalLinguaFranca.g:3274:1: ( ',' )
            {
            // InternalLinguaFranca.g:3274:1: ( ',' )
            // InternalLinguaFranca.g:3275:2: ','
            {
             before(grammarAccess.getGetsAccess().getCommaKeyword_1_0()); 
            match(input,27,FOLLOW_2); 
             after(grammarAccess.getGetsAccess().getCommaKeyword_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Gets__Group_1__0__Impl"


    // $ANTLR start "rule__Gets__Group_1__1"
    // InternalLinguaFranca.g:3284:1: rule__Gets__Group_1__1 : rule__Gets__Group_1__1__Impl ;
    public final void rule__Gets__Group_1__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3288:1: ( rule__Gets__Group_1__1__Impl )
            // InternalLinguaFranca.g:3289:2: rule__Gets__Group_1__1__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Gets__Group_1__1__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Gets__Group_1__1"


    // $ANTLR start "rule__Gets__Group_1__1__Impl"
    // InternalLinguaFranca.g:3295:1: rule__Gets__Group_1__1__Impl : ( ( rule__Gets__GetsAssignment_1_1 ) ) ;
    public final void rule__Gets__Group_1__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3299:1: ( ( ( rule__Gets__GetsAssignment_1_1 ) ) )
            // InternalLinguaFranca.g:3300:1: ( ( rule__Gets__GetsAssignment_1_1 ) )
            {
            // InternalLinguaFranca.g:3300:1: ( ( rule__Gets__GetsAssignment_1_1 ) )
            // InternalLinguaFranca.g:3301:2: ( rule__Gets__GetsAssignment_1_1 )
            {
             before(grammarAccess.getGetsAccess().getGetsAssignment_1_1()); 
            // InternalLinguaFranca.g:3302:2: ( rule__Gets__GetsAssignment_1_1 )
            // InternalLinguaFranca.g:3302:3: rule__Gets__GetsAssignment_1_1
            {
            pushFollow(FOLLOW_2);
            rule__Gets__GetsAssignment_1_1();

            state._fsp--;


            }

             after(grammarAccess.getGetsAccess().getGetsAssignment_1_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Gets__Group_1__1__Impl"


    // $ANTLR start "rule__Params__Group__0"
    // InternalLinguaFranca.g:3311:1: rule__Params__Group__0 : rule__Params__Group__0__Impl rule__Params__Group__1 ;
    public final void rule__Params__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3315:1: ( rule__Params__Group__0__Impl rule__Params__Group__1 )
            // InternalLinguaFranca.g:3316:2: rule__Params__Group__0__Impl rule__Params__Group__1
            {
            pushFollow(FOLLOW_31);
            rule__Params__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Params__Group__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Params__Group__0"


    // $ANTLR start "rule__Params__Group__0__Impl"
    // InternalLinguaFranca.g:3323:1: rule__Params__Group__0__Impl : ( '(' ) ;
    public final void rule__Params__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3327:1: ( ( '(' ) )
            // InternalLinguaFranca.g:3328:1: ( '(' )
            {
            // InternalLinguaFranca.g:3328:1: ( '(' )
            // InternalLinguaFranca.g:3329:2: '('
            {
             before(grammarAccess.getParamsAccess().getLeftParenthesisKeyword_0()); 
            match(input,25,FOLLOW_2); 
             after(grammarAccess.getParamsAccess().getLeftParenthesisKeyword_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Params__Group__0__Impl"


    // $ANTLR start "rule__Params__Group__1"
    // InternalLinguaFranca.g:3338:1: rule__Params__Group__1 : rule__Params__Group__1__Impl rule__Params__Group__2 ;
    public final void rule__Params__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3342:1: ( rule__Params__Group__1__Impl rule__Params__Group__2 )
            // InternalLinguaFranca.g:3343:2: rule__Params__Group__1__Impl rule__Params__Group__2
            {
            pushFollow(FOLLOW_32);
            rule__Params__Group__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Params__Group__2();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Params__Group__1"


    // $ANTLR start "rule__Params__Group__1__Impl"
    // InternalLinguaFranca.g:3350:1: rule__Params__Group__1__Impl : ( ( rule__Params__ParamsAssignment_1 ) ) ;
    public final void rule__Params__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3354:1: ( ( ( rule__Params__ParamsAssignment_1 ) ) )
            // InternalLinguaFranca.g:3355:1: ( ( rule__Params__ParamsAssignment_1 ) )
            {
            // InternalLinguaFranca.g:3355:1: ( ( rule__Params__ParamsAssignment_1 ) )
            // InternalLinguaFranca.g:3356:2: ( rule__Params__ParamsAssignment_1 )
            {
             before(grammarAccess.getParamsAccess().getParamsAssignment_1()); 
            // InternalLinguaFranca.g:3357:2: ( rule__Params__ParamsAssignment_1 )
            // InternalLinguaFranca.g:3357:3: rule__Params__ParamsAssignment_1
            {
            pushFollow(FOLLOW_2);
            rule__Params__ParamsAssignment_1();

            state._fsp--;


            }

             after(grammarAccess.getParamsAccess().getParamsAssignment_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Params__Group__1__Impl"


    // $ANTLR start "rule__Params__Group__2"
    // InternalLinguaFranca.g:3365:1: rule__Params__Group__2 : rule__Params__Group__2__Impl rule__Params__Group__3 ;
    public final void rule__Params__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3369:1: ( rule__Params__Group__2__Impl rule__Params__Group__3 )
            // InternalLinguaFranca.g:3370:2: rule__Params__Group__2__Impl rule__Params__Group__3
            {
            pushFollow(FOLLOW_32);
            rule__Params__Group__2__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Params__Group__3();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Params__Group__2"


    // $ANTLR start "rule__Params__Group__2__Impl"
    // InternalLinguaFranca.g:3377:1: rule__Params__Group__2__Impl : ( ( rule__Params__Group_2__0 )* ) ;
    public final void rule__Params__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3381:1: ( ( ( rule__Params__Group_2__0 )* ) )
            // InternalLinguaFranca.g:3382:1: ( ( rule__Params__Group_2__0 )* )
            {
            // InternalLinguaFranca.g:3382:1: ( ( rule__Params__Group_2__0 )* )
            // InternalLinguaFranca.g:3383:2: ( rule__Params__Group_2__0 )*
            {
             before(grammarAccess.getParamsAccess().getGroup_2()); 
            // InternalLinguaFranca.g:3384:2: ( rule__Params__Group_2__0 )*
            loop42:
            do {
                int alt42=2;
                int LA42_0 = input.LA(1);

                if ( (LA42_0==27) ) {
                    alt42=1;
                }


                switch (alt42) {
            	case 1 :
            	    // InternalLinguaFranca.g:3384:3: rule__Params__Group_2__0
            	    {
            	    pushFollow(FOLLOW_25);
            	    rule__Params__Group_2__0();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop42;
                }
            } while (true);

             after(grammarAccess.getParamsAccess().getGroup_2()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Params__Group__2__Impl"


    // $ANTLR start "rule__Params__Group__3"
    // InternalLinguaFranca.g:3392:1: rule__Params__Group__3 : rule__Params__Group__3__Impl ;
    public final void rule__Params__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3396:1: ( rule__Params__Group__3__Impl )
            // InternalLinguaFranca.g:3397:2: rule__Params__Group__3__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Params__Group__3__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Params__Group__3"


    // $ANTLR start "rule__Params__Group__3__Impl"
    // InternalLinguaFranca.g:3403:1: rule__Params__Group__3__Impl : ( ')' ) ;
    public final void rule__Params__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3407:1: ( ( ')' ) )
            // InternalLinguaFranca.g:3408:1: ( ')' )
            {
            // InternalLinguaFranca.g:3408:1: ( ')' )
            // InternalLinguaFranca.g:3409:2: ')'
            {
             before(grammarAccess.getParamsAccess().getRightParenthesisKeyword_3()); 
            match(input,26,FOLLOW_2); 
             after(grammarAccess.getParamsAccess().getRightParenthesisKeyword_3()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Params__Group__3__Impl"


    // $ANTLR start "rule__Params__Group_2__0"
    // InternalLinguaFranca.g:3419:1: rule__Params__Group_2__0 : rule__Params__Group_2__0__Impl rule__Params__Group_2__1 ;
    public final void rule__Params__Group_2__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3423:1: ( rule__Params__Group_2__0__Impl rule__Params__Group_2__1 )
            // InternalLinguaFranca.g:3424:2: rule__Params__Group_2__0__Impl rule__Params__Group_2__1
            {
            pushFollow(FOLLOW_31);
            rule__Params__Group_2__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Params__Group_2__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Params__Group_2__0"


    // $ANTLR start "rule__Params__Group_2__0__Impl"
    // InternalLinguaFranca.g:3431:1: rule__Params__Group_2__0__Impl : ( ',' ) ;
    public final void rule__Params__Group_2__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3435:1: ( ( ',' ) )
            // InternalLinguaFranca.g:3436:1: ( ',' )
            {
            // InternalLinguaFranca.g:3436:1: ( ',' )
            // InternalLinguaFranca.g:3437:2: ','
            {
             before(grammarAccess.getParamsAccess().getCommaKeyword_2_0()); 
            match(input,27,FOLLOW_2); 
             after(grammarAccess.getParamsAccess().getCommaKeyword_2_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Params__Group_2__0__Impl"


    // $ANTLR start "rule__Params__Group_2__1"
    // InternalLinguaFranca.g:3446:1: rule__Params__Group_2__1 : rule__Params__Group_2__1__Impl ;
    public final void rule__Params__Group_2__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3450:1: ( rule__Params__Group_2__1__Impl )
            // InternalLinguaFranca.g:3451:2: rule__Params__Group_2__1__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Params__Group_2__1__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Params__Group_2__1"


    // $ANTLR start "rule__Params__Group_2__1__Impl"
    // InternalLinguaFranca.g:3457:1: rule__Params__Group_2__1__Impl : ( ( rule__Params__ParamsAssignment_2_1 ) ) ;
    public final void rule__Params__Group_2__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3461:1: ( ( ( rule__Params__ParamsAssignment_2_1 ) ) )
            // InternalLinguaFranca.g:3462:1: ( ( rule__Params__ParamsAssignment_2_1 ) )
            {
            // InternalLinguaFranca.g:3462:1: ( ( rule__Params__ParamsAssignment_2_1 ) )
            // InternalLinguaFranca.g:3463:2: ( rule__Params__ParamsAssignment_2_1 )
            {
             before(grammarAccess.getParamsAccess().getParamsAssignment_2_1()); 
            // InternalLinguaFranca.g:3464:2: ( rule__Params__ParamsAssignment_2_1 )
            // InternalLinguaFranca.g:3464:3: rule__Params__ParamsAssignment_2_1
            {
            pushFollow(FOLLOW_2);
            rule__Params__ParamsAssignment_2_1();

            state._fsp--;


            }

             after(grammarAccess.getParamsAccess().getParamsAssignment_2_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Params__Group_2__1__Impl"


    // $ANTLR start "rule__Param__Group__0"
    // InternalLinguaFranca.g:3473:1: rule__Param__Group__0 : rule__Param__Group__0__Impl rule__Param__Group__1 ;
    public final void rule__Param__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3477:1: ( rule__Param__Group__0__Impl rule__Param__Group__1 )
            // InternalLinguaFranca.g:3478:2: rule__Param__Group__0__Impl rule__Param__Group__1
            {
            pushFollow(FOLLOW_31);
            rule__Param__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Param__Group__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Param__Group__0"


    // $ANTLR start "rule__Param__Group__0__Impl"
    // InternalLinguaFranca.g:3485:1: rule__Param__Group__0__Impl : ( ( 'const' )? ) ;
    public final void rule__Param__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3489:1: ( ( ( 'const' )? ) )
            // InternalLinguaFranca.g:3490:1: ( ( 'const' )? )
            {
            // InternalLinguaFranca.g:3490:1: ( ( 'const' )? )
            // InternalLinguaFranca.g:3491:2: ( 'const' )?
            {
             before(grammarAccess.getParamAccess().getConstKeyword_0()); 
            // InternalLinguaFranca.g:3492:2: ( 'const' )?
            int alt43=2;
            int LA43_0 = input.LA(1);

            if ( (LA43_0==33) ) {
                alt43=1;
            }
            switch (alt43) {
                case 1 :
                    // InternalLinguaFranca.g:3492:3: 'const'
                    {
                    match(input,33,FOLLOW_2); 

                    }
                    break;

            }

             after(grammarAccess.getParamAccess().getConstKeyword_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Param__Group__0__Impl"


    // $ANTLR start "rule__Param__Group__1"
    // InternalLinguaFranca.g:3500:1: rule__Param__Group__1 : rule__Param__Group__1__Impl rule__Param__Group__2 ;
    public final void rule__Param__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3504:1: ( rule__Param__Group__1__Impl rule__Param__Group__2 )
            // InternalLinguaFranca.g:3505:2: rule__Param__Group__1__Impl rule__Param__Group__2
            {
            pushFollow(FOLLOW_33);
            rule__Param__Group__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Param__Group__2();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Param__Group__1"


    // $ANTLR start "rule__Param__Group__1__Impl"
    // InternalLinguaFranca.g:3512:1: rule__Param__Group__1__Impl : ( ( rule__Param__NameAssignment_1 ) ) ;
    public final void rule__Param__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3516:1: ( ( ( rule__Param__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:3517:1: ( ( rule__Param__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:3517:1: ( ( rule__Param__NameAssignment_1 ) )
            // InternalLinguaFranca.g:3518:2: ( rule__Param__NameAssignment_1 )
            {
             before(grammarAccess.getParamAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:3519:2: ( rule__Param__NameAssignment_1 )
            // InternalLinguaFranca.g:3519:3: rule__Param__NameAssignment_1
            {
            pushFollow(FOLLOW_2);
            rule__Param__NameAssignment_1();

            state._fsp--;


            }

             after(grammarAccess.getParamAccess().getNameAssignment_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Param__Group__1__Impl"


    // $ANTLR start "rule__Param__Group__2"
    // InternalLinguaFranca.g:3527:1: rule__Param__Group__2 : rule__Param__Group__2__Impl rule__Param__Group__3 ;
    public final void rule__Param__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3531:1: ( rule__Param__Group__2__Impl rule__Param__Group__3 )
            // InternalLinguaFranca.g:3532:2: rule__Param__Group__2__Impl rule__Param__Group__3
            {
            pushFollow(FOLLOW_33);
            rule__Param__Group__2__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Param__Group__3();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Param__Group__2"


    // $ANTLR start "rule__Param__Group__2__Impl"
    // InternalLinguaFranca.g:3539:1: rule__Param__Group__2__Impl : ( ( rule__Param__Group_2__0 )? ) ;
    public final void rule__Param__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3543:1: ( ( ( rule__Param__Group_2__0 )? ) )
            // InternalLinguaFranca.g:3544:1: ( ( rule__Param__Group_2__0 )? )
            {
            // InternalLinguaFranca.g:3544:1: ( ( rule__Param__Group_2__0 )? )
            // InternalLinguaFranca.g:3545:2: ( rule__Param__Group_2__0 )?
            {
             before(grammarAccess.getParamAccess().getGroup_2()); 
            // InternalLinguaFranca.g:3546:2: ( rule__Param__Group_2__0 )?
            int alt44=2;
            int LA44_0 = input.LA(1);

            if ( (LA44_0==23) ) {
                alt44=1;
            }
            switch (alt44) {
                case 1 :
                    // InternalLinguaFranca.g:3546:3: rule__Param__Group_2__0
                    {
                    pushFollow(FOLLOW_2);
                    rule__Param__Group_2__0();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getParamAccess().getGroup_2()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Param__Group__2__Impl"


    // $ANTLR start "rule__Param__Group__3"
    // InternalLinguaFranca.g:3554:1: rule__Param__Group__3 : rule__Param__Group__3__Impl ;
    public final void rule__Param__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3558:1: ( rule__Param__Group__3__Impl )
            // InternalLinguaFranca.g:3559:2: rule__Param__Group__3__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Param__Group__3__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Param__Group__3"


    // $ANTLR start "rule__Param__Group__3__Impl"
    // InternalLinguaFranca.g:3565:1: rule__Param__Group__3__Impl : ( ( rule__Param__Group_3__0 )? ) ;
    public final void rule__Param__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3569:1: ( ( ( rule__Param__Group_3__0 )? ) )
            // InternalLinguaFranca.g:3570:1: ( ( rule__Param__Group_3__0 )? )
            {
            // InternalLinguaFranca.g:3570:1: ( ( rule__Param__Group_3__0 )? )
            // InternalLinguaFranca.g:3571:2: ( rule__Param__Group_3__0 )?
            {
             before(grammarAccess.getParamAccess().getGroup_3()); 
            // InternalLinguaFranca.g:3572:2: ( rule__Param__Group_3__0 )?
            int alt45=2;
            int LA45_0 = input.LA(1);

            if ( (LA45_0==25) ) {
                alt45=1;
            }
            switch (alt45) {
                case 1 :
                    // InternalLinguaFranca.g:3572:3: rule__Param__Group_3__0
                    {
                    pushFollow(FOLLOW_2);
                    rule__Param__Group_3__0();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getParamAccess().getGroup_3()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Param__Group__3__Impl"


    // $ANTLR start "rule__Param__Group_2__0"
    // InternalLinguaFranca.g:3581:1: rule__Param__Group_2__0 : rule__Param__Group_2__0__Impl rule__Param__Group_2__1 ;
    public final void rule__Param__Group_2__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3585:1: ( rule__Param__Group_2__0__Impl rule__Param__Group_2__1 )
            // InternalLinguaFranca.g:3586:2: rule__Param__Group_2__0__Impl rule__Param__Group_2__1
            {
            pushFollow(FOLLOW_18);
            rule__Param__Group_2__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Param__Group_2__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Param__Group_2__0"


    // $ANTLR start "rule__Param__Group_2__0__Impl"
    // InternalLinguaFranca.g:3593:1: rule__Param__Group_2__0__Impl : ( ':' ) ;
    public final void rule__Param__Group_2__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3597:1: ( ( ':' ) )
            // InternalLinguaFranca.g:3598:1: ( ':' )
            {
            // InternalLinguaFranca.g:3598:1: ( ':' )
            // InternalLinguaFranca.g:3599:2: ':'
            {
             before(grammarAccess.getParamAccess().getColonKeyword_2_0()); 
            match(input,23,FOLLOW_2); 
             after(grammarAccess.getParamAccess().getColonKeyword_2_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Param__Group_2__0__Impl"


    // $ANTLR start "rule__Param__Group_2__1"
    // InternalLinguaFranca.g:3608:1: rule__Param__Group_2__1 : rule__Param__Group_2__1__Impl ;
    public final void rule__Param__Group_2__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3612:1: ( rule__Param__Group_2__1__Impl )
            // InternalLinguaFranca.g:3613:2: rule__Param__Group_2__1__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Param__Group_2__1__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Param__Group_2__1"


    // $ANTLR start "rule__Param__Group_2__1__Impl"
    // InternalLinguaFranca.g:3619:1: rule__Param__Group_2__1__Impl : ( ( rule__Param__TypeAssignment_2_1 ) ) ;
    public final void rule__Param__Group_2__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3623:1: ( ( ( rule__Param__TypeAssignment_2_1 ) ) )
            // InternalLinguaFranca.g:3624:1: ( ( rule__Param__TypeAssignment_2_1 ) )
            {
            // InternalLinguaFranca.g:3624:1: ( ( rule__Param__TypeAssignment_2_1 ) )
            // InternalLinguaFranca.g:3625:2: ( rule__Param__TypeAssignment_2_1 )
            {
             before(grammarAccess.getParamAccess().getTypeAssignment_2_1()); 
            // InternalLinguaFranca.g:3626:2: ( rule__Param__TypeAssignment_2_1 )
            // InternalLinguaFranca.g:3626:3: rule__Param__TypeAssignment_2_1
            {
            pushFollow(FOLLOW_2);
            rule__Param__TypeAssignment_2_1();

            state._fsp--;


            }

             after(grammarAccess.getParamAccess().getTypeAssignment_2_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Param__Group_2__1__Impl"


    // $ANTLR start "rule__Param__Group_3__0"
    // InternalLinguaFranca.g:3635:1: rule__Param__Group_3__0 : rule__Param__Group_3__0__Impl rule__Param__Group_3__1 ;
    public final void rule__Param__Group_3__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3639:1: ( rule__Param__Group_3__0__Impl rule__Param__Group_3__1 )
            // InternalLinguaFranca.g:3640:2: rule__Param__Group_3__0__Impl rule__Param__Group_3__1
            {
            pushFollow(FOLLOW_30);
            rule__Param__Group_3__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Param__Group_3__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Param__Group_3__0"


    // $ANTLR start "rule__Param__Group_3__0__Impl"
    // InternalLinguaFranca.g:3647:1: rule__Param__Group_3__0__Impl : ( '(' ) ;
    public final void rule__Param__Group_3__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3651:1: ( ( '(' ) )
            // InternalLinguaFranca.g:3652:1: ( '(' )
            {
            // InternalLinguaFranca.g:3652:1: ( '(' )
            // InternalLinguaFranca.g:3653:2: '('
            {
             before(grammarAccess.getParamAccess().getLeftParenthesisKeyword_3_0()); 
            match(input,25,FOLLOW_2); 
             after(grammarAccess.getParamAccess().getLeftParenthesisKeyword_3_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Param__Group_3__0__Impl"


    // $ANTLR start "rule__Param__Group_3__1"
    // InternalLinguaFranca.g:3662:1: rule__Param__Group_3__1 : rule__Param__Group_3__1__Impl rule__Param__Group_3__2 ;
    public final void rule__Param__Group_3__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3666:1: ( rule__Param__Group_3__1__Impl rule__Param__Group_3__2 )
            // InternalLinguaFranca.g:3667:2: rule__Param__Group_3__1__Impl rule__Param__Group_3__2
            {
            pushFollow(FOLLOW_34);
            rule__Param__Group_3__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Param__Group_3__2();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Param__Group_3__1"


    // $ANTLR start "rule__Param__Group_3__1__Impl"
    // InternalLinguaFranca.g:3674:1: rule__Param__Group_3__1__Impl : ( ( rule__Param__ValueAssignment_3_1 ) ) ;
    public final void rule__Param__Group_3__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3678:1: ( ( ( rule__Param__ValueAssignment_3_1 ) ) )
            // InternalLinguaFranca.g:3679:1: ( ( rule__Param__ValueAssignment_3_1 ) )
            {
            // InternalLinguaFranca.g:3679:1: ( ( rule__Param__ValueAssignment_3_1 ) )
            // InternalLinguaFranca.g:3680:2: ( rule__Param__ValueAssignment_3_1 )
            {
             before(grammarAccess.getParamAccess().getValueAssignment_3_1()); 
            // InternalLinguaFranca.g:3681:2: ( rule__Param__ValueAssignment_3_1 )
            // InternalLinguaFranca.g:3681:3: rule__Param__ValueAssignment_3_1
            {
            pushFollow(FOLLOW_2);
            rule__Param__ValueAssignment_3_1();

            state._fsp--;


            }

             after(grammarAccess.getParamAccess().getValueAssignment_3_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Param__Group_3__1__Impl"


    // $ANTLR start "rule__Param__Group_3__2"
    // InternalLinguaFranca.g:3689:1: rule__Param__Group_3__2 : rule__Param__Group_3__2__Impl ;
    public final void rule__Param__Group_3__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3693:1: ( rule__Param__Group_3__2__Impl )
            // InternalLinguaFranca.g:3694:2: rule__Param__Group_3__2__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Param__Group_3__2__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Param__Group_3__2"


    // $ANTLR start "rule__Param__Group_3__2__Impl"
    // InternalLinguaFranca.g:3700:1: rule__Param__Group_3__2__Impl : ( ')' ) ;
    public final void rule__Param__Group_3__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3704:1: ( ( ')' ) )
            // InternalLinguaFranca.g:3705:1: ( ')' )
            {
            // InternalLinguaFranca.g:3705:1: ( ')' )
            // InternalLinguaFranca.g:3706:2: ')'
            {
             before(grammarAccess.getParamAccess().getRightParenthesisKeyword_3_2()); 
            match(input,26,FOLLOW_2); 
             after(grammarAccess.getParamAccess().getRightParenthesisKeyword_3_2()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Param__Group_3__2__Impl"


    // $ANTLR start "rule__Period__Group__0"
    // InternalLinguaFranca.g:3716:1: rule__Period__Group__0 : rule__Period__Group__0__Impl rule__Period__Group__1 ;
    public final void rule__Period__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3720:1: ( rule__Period__Group__0__Impl rule__Period__Group__1 )
            // InternalLinguaFranca.g:3721:2: rule__Period__Group__0__Impl rule__Period__Group__1
            {
            pushFollow(FOLLOW_35);
            rule__Period__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Period__Group__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__Group__0"


    // $ANTLR start "rule__Period__Group__0__Impl"
    // InternalLinguaFranca.g:3728:1: rule__Period__Group__0__Impl : ( '(' ) ;
    public final void rule__Period__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3732:1: ( ( '(' ) )
            // InternalLinguaFranca.g:3733:1: ( '(' )
            {
            // InternalLinguaFranca.g:3733:1: ( '(' )
            // InternalLinguaFranca.g:3734:2: '('
            {
             before(grammarAccess.getPeriodAccess().getLeftParenthesisKeyword_0()); 
            match(input,25,FOLLOW_2); 
             after(grammarAccess.getPeriodAccess().getLeftParenthesisKeyword_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__Group__0__Impl"


    // $ANTLR start "rule__Period__Group__1"
    // InternalLinguaFranca.g:3743:1: rule__Period__Group__1 : rule__Period__Group__1__Impl rule__Period__Group__2 ;
    public final void rule__Period__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3747:1: ( rule__Period__Group__1__Impl rule__Period__Group__2 )
            // InternalLinguaFranca.g:3748:2: rule__Period__Group__1__Impl rule__Period__Group__2
            {
            pushFollow(FOLLOW_32);
            rule__Period__Group__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Period__Group__2();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__Group__1"


    // $ANTLR start "rule__Period__Group__1__Impl"
    // InternalLinguaFranca.g:3755:1: rule__Period__Group__1__Impl : ( ( rule__Period__PeriodAssignment_1 ) ) ;
    public final void rule__Period__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3759:1: ( ( ( rule__Period__PeriodAssignment_1 ) ) )
            // InternalLinguaFranca.g:3760:1: ( ( rule__Period__PeriodAssignment_1 ) )
            {
            // InternalLinguaFranca.g:3760:1: ( ( rule__Period__PeriodAssignment_1 ) )
            // InternalLinguaFranca.g:3761:2: ( rule__Period__PeriodAssignment_1 )
            {
             before(grammarAccess.getPeriodAccess().getPeriodAssignment_1()); 
            // InternalLinguaFranca.g:3762:2: ( rule__Period__PeriodAssignment_1 )
            // InternalLinguaFranca.g:3762:3: rule__Period__PeriodAssignment_1
            {
            pushFollow(FOLLOW_2);
            rule__Period__PeriodAssignment_1();

            state._fsp--;


            }

             after(grammarAccess.getPeriodAccess().getPeriodAssignment_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__Group__1__Impl"


    // $ANTLR start "rule__Period__Group__2"
    // InternalLinguaFranca.g:3770:1: rule__Period__Group__2 : rule__Period__Group__2__Impl rule__Period__Group__3 ;
    public final void rule__Period__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3774:1: ( rule__Period__Group__2__Impl rule__Period__Group__3 )
            // InternalLinguaFranca.g:3775:2: rule__Period__Group__2__Impl rule__Period__Group__3
            {
            pushFollow(FOLLOW_32);
            rule__Period__Group__2__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Period__Group__3();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__Group__2"


    // $ANTLR start "rule__Period__Group__2__Impl"
    // InternalLinguaFranca.g:3782:1: rule__Period__Group__2__Impl : ( ( rule__Period__Group_2__0 )? ) ;
    public final void rule__Period__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3786:1: ( ( ( rule__Period__Group_2__0 )? ) )
            // InternalLinguaFranca.g:3787:1: ( ( rule__Period__Group_2__0 )? )
            {
            // InternalLinguaFranca.g:3787:1: ( ( rule__Period__Group_2__0 )? )
            // InternalLinguaFranca.g:3788:2: ( rule__Period__Group_2__0 )?
            {
             before(grammarAccess.getPeriodAccess().getGroup_2()); 
            // InternalLinguaFranca.g:3789:2: ( rule__Period__Group_2__0 )?
            int alt46=2;
            int LA46_0 = input.LA(1);

            if ( (LA46_0==27) ) {
                alt46=1;
            }
            switch (alt46) {
                case 1 :
                    // InternalLinguaFranca.g:3789:3: rule__Period__Group_2__0
                    {
                    pushFollow(FOLLOW_2);
                    rule__Period__Group_2__0();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getPeriodAccess().getGroup_2()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__Group__2__Impl"


    // $ANTLR start "rule__Period__Group__3"
    // InternalLinguaFranca.g:3797:1: rule__Period__Group__3 : rule__Period__Group__3__Impl ;
    public final void rule__Period__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3801:1: ( rule__Period__Group__3__Impl )
            // InternalLinguaFranca.g:3802:2: rule__Period__Group__3__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Period__Group__3__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__Group__3"


    // $ANTLR start "rule__Period__Group__3__Impl"
    // InternalLinguaFranca.g:3808:1: rule__Period__Group__3__Impl : ( ')' ) ;
    public final void rule__Period__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3812:1: ( ( ')' ) )
            // InternalLinguaFranca.g:3813:1: ( ')' )
            {
            // InternalLinguaFranca.g:3813:1: ( ')' )
            // InternalLinguaFranca.g:3814:2: ')'
            {
             before(grammarAccess.getPeriodAccess().getRightParenthesisKeyword_3()); 
            match(input,26,FOLLOW_2); 
             after(grammarAccess.getPeriodAccess().getRightParenthesisKeyword_3()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__Group__3__Impl"


    // $ANTLR start "rule__Period__Group_2__0"
    // InternalLinguaFranca.g:3824:1: rule__Period__Group_2__0 : rule__Period__Group_2__0__Impl rule__Period__Group_2__1 ;
    public final void rule__Period__Group_2__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3828:1: ( rule__Period__Group_2__0__Impl rule__Period__Group_2__1 )
            // InternalLinguaFranca.g:3829:2: rule__Period__Group_2__0__Impl rule__Period__Group_2__1
            {
            pushFollow(FOLLOW_35);
            rule__Period__Group_2__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Period__Group_2__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__Group_2__0"


    // $ANTLR start "rule__Period__Group_2__0__Impl"
    // InternalLinguaFranca.g:3836:1: rule__Period__Group_2__0__Impl : ( ',' ) ;
    public final void rule__Period__Group_2__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3840:1: ( ( ',' ) )
            // InternalLinguaFranca.g:3841:1: ( ',' )
            {
            // InternalLinguaFranca.g:3841:1: ( ',' )
            // InternalLinguaFranca.g:3842:2: ','
            {
             before(grammarAccess.getPeriodAccess().getCommaKeyword_2_0()); 
            match(input,27,FOLLOW_2); 
             after(grammarAccess.getPeriodAccess().getCommaKeyword_2_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__Group_2__0__Impl"


    // $ANTLR start "rule__Period__Group_2__1"
    // InternalLinguaFranca.g:3851:1: rule__Period__Group_2__1 : rule__Period__Group_2__1__Impl rule__Period__Group_2__2 ;
    public final void rule__Period__Group_2__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3855:1: ( rule__Period__Group_2__1__Impl rule__Period__Group_2__2 )
            // InternalLinguaFranca.g:3856:2: rule__Period__Group_2__1__Impl rule__Period__Group_2__2
            {
            pushFollow(FOLLOW_24);
            rule__Period__Group_2__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Period__Group_2__2();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__Group_2__1"


    // $ANTLR start "rule__Period__Group_2__1__Impl"
    // InternalLinguaFranca.g:3863:1: rule__Period__Group_2__1__Impl : ( ( rule__Period__OffsetAssignment_2_1 ) ) ;
    public final void rule__Period__Group_2__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3867:1: ( ( ( rule__Period__OffsetAssignment_2_1 ) ) )
            // InternalLinguaFranca.g:3868:1: ( ( rule__Period__OffsetAssignment_2_1 ) )
            {
            // InternalLinguaFranca.g:3868:1: ( ( rule__Period__OffsetAssignment_2_1 ) )
            // InternalLinguaFranca.g:3869:2: ( rule__Period__OffsetAssignment_2_1 )
            {
             before(grammarAccess.getPeriodAccess().getOffsetAssignment_2_1()); 
            // InternalLinguaFranca.g:3870:2: ( rule__Period__OffsetAssignment_2_1 )
            // InternalLinguaFranca.g:3870:3: rule__Period__OffsetAssignment_2_1
            {
            pushFollow(FOLLOW_2);
            rule__Period__OffsetAssignment_2_1();

            state._fsp--;


            }

             after(grammarAccess.getPeriodAccess().getOffsetAssignment_2_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__Group_2__1__Impl"


    // $ANTLR start "rule__Period__Group_2__2"
    // InternalLinguaFranca.g:3878:1: rule__Period__Group_2__2 : rule__Period__Group_2__2__Impl ;
    public final void rule__Period__Group_2__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3882:1: ( rule__Period__Group_2__2__Impl )
            // InternalLinguaFranca.g:3883:2: rule__Period__Group_2__2__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Period__Group_2__2__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__Group_2__2"


    // $ANTLR start "rule__Period__Group_2__2__Impl"
    // InternalLinguaFranca.g:3889:1: rule__Period__Group_2__2__Impl : ( ( rule__Period__Group_2_2__0 )? ) ;
    public final void rule__Period__Group_2__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3893:1: ( ( ( rule__Period__Group_2_2__0 )? ) )
            // InternalLinguaFranca.g:3894:1: ( ( rule__Period__Group_2_2__0 )? )
            {
            // InternalLinguaFranca.g:3894:1: ( ( rule__Period__Group_2_2__0 )? )
            // InternalLinguaFranca.g:3895:2: ( rule__Period__Group_2_2__0 )?
            {
             before(grammarAccess.getPeriodAccess().getGroup_2_2()); 
            // InternalLinguaFranca.g:3896:2: ( rule__Period__Group_2_2__0 )?
            int alt47=2;
            int LA47_0 = input.LA(1);

            if ( (LA47_0==27) ) {
                alt47=1;
            }
            switch (alt47) {
                case 1 :
                    // InternalLinguaFranca.g:3896:3: rule__Period__Group_2_2__0
                    {
                    pushFollow(FOLLOW_2);
                    rule__Period__Group_2_2__0();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getPeriodAccess().getGroup_2_2()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__Group_2__2__Impl"


    // $ANTLR start "rule__Period__Group_2_2__0"
    // InternalLinguaFranca.g:3905:1: rule__Period__Group_2_2__0 : rule__Period__Group_2_2__0__Impl rule__Period__Group_2_2__1 ;
    public final void rule__Period__Group_2_2__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3909:1: ( rule__Period__Group_2_2__0__Impl rule__Period__Group_2_2__1 )
            // InternalLinguaFranca.g:3910:2: rule__Period__Group_2_2__0__Impl rule__Period__Group_2_2__1
            {
            pushFollow(FOLLOW_35);
            rule__Period__Group_2_2__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Period__Group_2_2__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__Group_2_2__0"


    // $ANTLR start "rule__Period__Group_2_2__0__Impl"
    // InternalLinguaFranca.g:3917:1: rule__Period__Group_2_2__0__Impl : ( ',' ) ;
    public final void rule__Period__Group_2_2__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3921:1: ( ( ',' ) )
            // InternalLinguaFranca.g:3922:1: ( ',' )
            {
            // InternalLinguaFranca.g:3922:1: ( ',' )
            // InternalLinguaFranca.g:3923:2: ','
            {
             before(grammarAccess.getPeriodAccess().getCommaKeyword_2_2_0()); 
            match(input,27,FOLLOW_2); 
             after(grammarAccess.getPeriodAccess().getCommaKeyword_2_2_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__Group_2_2__0__Impl"


    // $ANTLR start "rule__Period__Group_2_2__1"
    // InternalLinguaFranca.g:3932:1: rule__Period__Group_2_2__1 : rule__Period__Group_2_2__1__Impl ;
    public final void rule__Period__Group_2_2__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3936:1: ( rule__Period__Group_2_2__1__Impl )
            // InternalLinguaFranca.g:3937:2: rule__Period__Group_2_2__1__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Period__Group_2_2__1__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__Group_2_2__1"


    // $ANTLR start "rule__Period__Group_2_2__1__Impl"
    // InternalLinguaFranca.g:3943:1: rule__Period__Group_2_2__1__Impl : ( ( rule__Period__CountAssignment_2_2_1 ) ) ;
    public final void rule__Period__Group_2_2__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3947:1: ( ( ( rule__Period__CountAssignment_2_2_1 ) ) )
            // InternalLinguaFranca.g:3948:1: ( ( rule__Period__CountAssignment_2_2_1 ) )
            {
            // InternalLinguaFranca.g:3948:1: ( ( rule__Period__CountAssignment_2_2_1 ) )
            // InternalLinguaFranca.g:3949:2: ( rule__Period__CountAssignment_2_2_1 )
            {
             before(grammarAccess.getPeriodAccess().getCountAssignment_2_2_1()); 
            // InternalLinguaFranca.g:3950:2: ( rule__Period__CountAssignment_2_2_1 )
            // InternalLinguaFranca.g:3950:3: rule__Period__CountAssignment_2_2_1
            {
            pushFollow(FOLLOW_2);
            rule__Period__CountAssignment_2_2_1();

            state._fsp--;


            }

             after(grammarAccess.getPeriodAccess().getCountAssignment_2_2_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__Group_2_2__1__Impl"


    // $ANTLR start "rule__Port__Group_1__0"
    // InternalLinguaFranca.g:3959:1: rule__Port__Group_1__0 : rule__Port__Group_1__0__Impl rule__Port__Group_1__1 ;
    public final void rule__Port__Group_1__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3963:1: ( rule__Port__Group_1__0__Impl rule__Port__Group_1__1 )
            // InternalLinguaFranca.g:3964:2: rule__Port__Group_1__0__Impl rule__Port__Group_1__1
            {
            pushFollow(FOLLOW_36);
            rule__Port__Group_1__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Port__Group_1__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Port__Group_1__0"


    // $ANTLR start "rule__Port__Group_1__0__Impl"
    // InternalLinguaFranca.g:3971:1: rule__Port__Group_1__0__Impl : ( RULE_ID ) ;
    public final void rule__Port__Group_1__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3975:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:3976:1: ( RULE_ID )
            {
            // InternalLinguaFranca.g:3976:1: ( RULE_ID )
            // InternalLinguaFranca.g:3977:2: RULE_ID
            {
             before(grammarAccess.getPortAccess().getIDTerminalRuleCall_1_0()); 
            match(input,RULE_ID,FOLLOW_2); 
             after(grammarAccess.getPortAccess().getIDTerminalRuleCall_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Port__Group_1__0__Impl"


    // $ANTLR start "rule__Port__Group_1__1"
    // InternalLinguaFranca.g:3986:1: rule__Port__Group_1__1 : rule__Port__Group_1__1__Impl rule__Port__Group_1__2 ;
    public final void rule__Port__Group_1__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3990:1: ( rule__Port__Group_1__1__Impl rule__Port__Group_1__2 )
            // InternalLinguaFranca.g:3991:2: rule__Port__Group_1__1__Impl rule__Port__Group_1__2
            {
            pushFollow(FOLLOW_37);
            rule__Port__Group_1__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Port__Group_1__2();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Port__Group_1__1"


    // $ANTLR start "rule__Port__Group_1__1__Impl"
    // InternalLinguaFranca.g:3998:1: rule__Port__Group_1__1__Impl : ( '.' ) ;
    public final void rule__Port__Group_1__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4002:1: ( ( '.' ) )
            // InternalLinguaFranca.g:4003:1: ( '.' )
            {
            // InternalLinguaFranca.g:4003:1: ( '.' )
            // InternalLinguaFranca.g:4004:2: '.'
            {
             before(grammarAccess.getPortAccess().getFullStopKeyword_1_1()); 
            match(input,34,FOLLOW_2); 
             after(grammarAccess.getPortAccess().getFullStopKeyword_1_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Port__Group_1__1__Impl"


    // $ANTLR start "rule__Port__Group_1__2"
    // InternalLinguaFranca.g:4013:1: rule__Port__Group_1__2 : rule__Port__Group_1__2__Impl ;
    public final void rule__Port__Group_1__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4017:1: ( rule__Port__Group_1__2__Impl )
            // InternalLinguaFranca.g:4018:2: rule__Port__Group_1__2__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Port__Group_1__2__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Port__Group_1__2"


    // $ANTLR start "rule__Port__Group_1__2__Impl"
    // InternalLinguaFranca.g:4024:1: rule__Port__Group_1__2__Impl : ( ( rule__Port__Alternatives_1_2 ) ) ;
    public final void rule__Port__Group_1__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4028:1: ( ( ( rule__Port__Alternatives_1_2 ) ) )
            // InternalLinguaFranca.g:4029:1: ( ( rule__Port__Alternatives_1_2 ) )
            {
            // InternalLinguaFranca.g:4029:1: ( ( rule__Port__Alternatives_1_2 ) )
            // InternalLinguaFranca.g:4030:2: ( rule__Port__Alternatives_1_2 )
            {
             before(grammarAccess.getPortAccess().getAlternatives_1_2()); 
            // InternalLinguaFranca.g:4031:2: ( rule__Port__Alternatives_1_2 )
            // InternalLinguaFranca.g:4031:3: rule__Port__Alternatives_1_2
            {
            pushFollow(FOLLOW_2);
            rule__Port__Alternatives_1_2();

            state._fsp--;


            }

             after(grammarAccess.getPortAccess().getAlternatives_1_2()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Port__Group_1__2__Impl"


    // $ANTLR start "rule__Sets__Group__0"
    // InternalLinguaFranca.g:4040:1: rule__Sets__Group__0 : rule__Sets__Group__0__Impl rule__Sets__Group__1 ;
    public final void rule__Sets__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4044:1: ( rule__Sets__Group__0__Impl rule__Sets__Group__1 )
            // InternalLinguaFranca.g:4045:2: rule__Sets__Group__0__Impl rule__Sets__Group__1
            {
            pushFollow(FOLLOW_6);
            rule__Sets__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Sets__Group__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Sets__Group__0"


    // $ANTLR start "rule__Sets__Group__0__Impl"
    // InternalLinguaFranca.g:4052:1: rule__Sets__Group__0__Impl : ( '->' ) ;
    public final void rule__Sets__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4056:1: ( ( '->' ) )
            // InternalLinguaFranca.g:4057:1: ( '->' )
            {
            // InternalLinguaFranca.g:4057:1: ( '->' )
            // InternalLinguaFranca.g:4058:2: '->'
            {
             before(grammarAccess.getSetsAccess().getHyphenMinusGreaterThanSignKeyword_0()); 
            match(input,32,FOLLOW_2); 
             after(grammarAccess.getSetsAccess().getHyphenMinusGreaterThanSignKeyword_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Sets__Group__0__Impl"


    // $ANTLR start "rule__Sets__Group__1"
    // InternalLinguaFranca.g:4067:1: rule__Sets__Group__1 : rule__Sets__Group__1__Impl rule__Sets__Group__2 ;
    public final void rule__Sets__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4071:1: ( rule__Sets__Group__1__Impl rule__Sets__Group__2 )
            // InternalLinguaFranca.g:4072:2: rule__Sets__Group__1__Impl rule__Sets__Group__2
            {
            pushFollow(FOLLOW_24);
            rule__Sets__Group__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Sets__Group__2();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Sets__Group__1"


    // $ANTLR start "rule__Sets__Group__1__Impl"
    // InternalLinguaFranca.g:4079:1: rule__Sets__Group__1__Impl : ( ( rule__Sets__SetsAssignment_1 ) ) ;
    public final void rule__Sets__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4083:1: ( ( ( rule__Sets__SetsAssignment_1 ) ) )
            // InternalLinguaFranca.g:4084:1: ( ( rule__Sets__SetsAssignment_1 ) )
            {
            // InternalLinguaFranca.g:4084:1: ( ( rule__Sets__SetsAssignment_1 ) )
            // InternalLinguaFranca.g:4085:2: ( rule__Sets__SetsAssignment_1 )
            {
             before(grammarAccess.getSetsAccess().getSetsAssignment_1()); 
            // InternalLinguaFranca.g:4086:2: ( rule__Sets__SetsAssignment_1 )
            // InternalLinguaFranca.g:4086:3: rule__Sets__SetsAssignment_1
            {
            pushFollow(FOLLOW_2);
            rule__Sets__SetsAssignment_1();

            state._fsp--;


            }

             after(grammarAccess.getSetsAccess().getSetsAssignment_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Sets__Group__1__Impl"


    // $ANTLR start "rule__Sets__Group__2"
    // InternalLinguaFranca.g:4094:1: rule__Sets__Group__2 : rule__Sets__Group__2__Impl ;
    public final void rule__Sets__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4098:1: ( rule__Sets__Group__2__Impl )
            // InternalLinguaFranca.g:4099:2: rule__Sets__Group__2__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Sets__Group__2__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Sets__Group__2"


    // $ANTLR start "rule__Sets__Group__2__Impl"
    // InternalLinguaFranca.g:4105:1: rule__Sets__Group__2__Impl : ( ( rule__Sets__Group_2__0 )? ) ;
    public final void rule__Sets__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4109:1: ( ( ( rule__Sets__Group_2__0 )? ) )
            // InternalLinguaFranca.g:4110:1: ( ( rule__Sets__Group_2__0 )? )
            {
            // InternalLinguaFranca.g:4110:1: ( ( rule__Sets__Group_2__0 )? )
            // InternalLinguaFranca.g:4111:2: ( rule__Sets__Group_2__0 )?
            {
             before(grammarAccess.getSetsAccess().getGroup_2()); 
            // InternalLinguaFranca.g:4112:2: ( rule__Sets__Group_2__0 )?
            int alt48=2;
            int LA48_0 = input.LA(1);

            if ( (LA48_0==27) ) {
                alt48=1;
            }
            switch (alt48) {
                case 1 :
                    // InternalLinguaFranca.g:4112:3: rule__Sets__Group_2__0
                    {
                    pushFollow(FOLLOW_2);
                    rule__Sets__Group_2__0();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getSetsAccess().getGroup_2()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Sets__Group__2__Impl"


    // $ANTLR start "rule__Sets__Group_2__0"
    // InternalLinguaFranca.g:4121:1: rule__Sets__Group_2__0 : rule__Sets__Group_2__0__Impl rule__Sets__Group_2__1 ;
    public final void rule__Sets__Group_2__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4125:1: ( rule__Sets__Group_2__0__Impl rule__Sets__Group_2__1 )
            // InternalLinguaFranca.g:4126:2: rule__Sets__Group_2__0__Impl rule__Sets__Group_2__1
            {
            pushFollow(FOLLOW_6);
            rule__Sets__Group_2__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Sets__Group_2__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Sets__Group_2__0"


    // $ANTLR start "rule__Sets__Group_2__0__Impl"
    // InternalLinguaFranca.g:4133:1: rule__Sets__Group_2__0__Impl : ( ',' ) ;
    public final void rule__Sets__Group_2__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4137:1: ( ( ',' ) )
            // InternalLinguaFranca.g:4138:1: ( ',' )
            {
            // InternalLinguaFranca.g:4138:1: ( ',' )
            // InternalLinguaFranca.g:4139:2: ','
            {
             before(grammarAccess.getSetsAccess().getCommaKeyword_2_0()); 
            match(input,27,FOLLOW_2); 
             after(grammarAccess.getSetsAccess().getCommaKeyword_2_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Sets__Group_2__0__Impl"


    // $ANTLR start "rule__Sets__Group_2__1"
    // InternalLinguaFranca.g:4148:1: rule__Sets__Group_2__1 : rule__Sets__Group_2__1__Impl ;
    public final void rule__Sets__Group_2__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4152:1: ( rule__Sets__Group_2__1__Impl )
            // InternalLinguaFranca.g:4153:2: rule__Sets__Group_2__1__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Sets__Group_2__1__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Sets__Group_2__1"


    // $ANTLR start "rule__Sets__Group_2__1__Impl"
    // InternalLinguaFranca.g:4159:1: rule__Sets__Group_2__1__Impl : ( ( rule__Sets__SetsAssignment_2_1 ) ) ;
    public final void rule__Sets__Group_2__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4163:1: ( ( ( rule__Sets__SetsAssignment_2_1 ) ) )
            // InternalLinguaFranca.g:4164:1: ( ( rule__Sets__SetsAssignment_2_1 ) )
            {
            // InternalLinguaFranca.g:4164:1: ( ( rule__Sets__SetsAssignment_2_1 ) )
            // InternalLinguaFranca.g:4165:2: ( rule__Sets__SetsAssignment_2_1 )
            {
             before(grammarAccess.getSetsAccess().getSetsAssignment_2_1()); 
            // InternalLinguaFranca.g:4166:2: ( rule__Sets__SetsAssignment_2_1 )
            // InternalLinguaFranca.g:4166:3: rule__Sets__SetsAssignment_2_1
            {
            pushFollow(FOLLOW_2);
            rule__Sets__SetsAssignment_2_1();

            state._fsp--;


            }

             after(grammarAccess.getSetsAccess().getSetsAssignment_2_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Sets__Group_2__1__Impl"


    // $ANTLR start "rule__Path__Group__0"
    // InternalLinguaFranca.g:4175:1: rule__Path__Group__0 : rule__Path__Group__0__Impl rule__Path__Group__1 ;
    public final void rule__Path__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4179:1: ( rule__Path__Group__0__Impl rule__Path__Group__1 )
            // InternalLinguaFranca.g:4180:2: rule__Path__Group__0__Impl rule__Path__Group__1
            {
            pushFollow(FOLLOW_36);
            rule__Path__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Path__Group__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Path__Group__0"


    // $ANTLR start "rule__Path__Group__0__Impl"
    // InternalLinguaFranca.g:4187:1: rule__Path__Group__0__Impl : ( RULE_ID ) ;
    public final void rule__Path__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4191:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4192:1: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4192:1: ( RULE_ID )
            // InternalLinguaFranca.g:4193:2: RULE_ID
            {
             before(grammarAccess.getPathAccess().getIDTerminalRuleCall_0()); 
            match(input,RULE_ID,FOLLOW_2); 
             after(grammarAccess.getPathAccess().getIDTerminalRuleCall_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Path__Group__0__Impl"


    // $ANTLR start "rule__Path__Group__1"
    // InternalLinguaFranca.g:4202:1: rule__Path__Group__1 : rule__Path__Group__1__Impl ;
    public final void rule__Path__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4206:1: ( rule__Path__Group__1__Impl )
            // InternalLinguaFranca.g:4207:2: rule__Path__Group__1__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Path__Group__1__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Path__Group__1"


    // $ANTLR start "rule__Path__Group__1__Impl"
    // InternalLinguaFranca.g:4213:1: rule__Path__Group__1__Impl : ( ( rule__Path__Group_1__0 )* ) ;
    public final void rule__Path__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4217:1: ( ( ( rule__Path__Group_1__0 )* ) )
            // InternalLinguaFranca.g:4218:1: ( ( rule__Path__Group_1__0 )* )
            {
            // InternalLinguaFranca.g:4218:1: ( ( rule__Path__Group_1__0 )* )
            // InternalLinguaFranca.g:4219:2: ( rule__Path__Group_1__0 )*
            {
             before(grammarAccess.getPathAccess().getGroup_1()); 
            // InternalLinguaFranca.g:4220:2: ( rule__Path__Group_1__0 )*
            loop49:
            do {
                int alt49=2;
                int LA49_0 = input.LA(1);

                if ( (LA49_0==34) ) {
                    alt49=1;
                }


                switch (alt49) {
            	case 1 :
            	    // InternalLinguaFranca.g:4220:3: rule__Path__Group_1__0
            	    {
            	    pushFollow(FOLLOW_38);
            	    rule__Path__Group_1__0();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop49;
                }
            } while (true);

             after(grammarAccess.getPathAccess().getGroup_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Path__Group__1__Impl"


    // $ANTLR start "rule__Path__Group_1__0"
    // InternalLinguaFranca.g:4229:1: rule__Path__Group_1__0 : rule__Path__Group_1__0__Impl rule__Path__Group_1__1 ;
    public final void rule__Path__Group_1__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4233:1: ( rule__Path__Group_1__0__Impl rule__Path__Group_1__1 )
            // InternalLinguaFranca.g:4234:2: rule__Path__Group_1__0__Impl rule__Path__Group_1__1
            {
            pushFollow(FOLLOW_6);
            rule__Path__Group_1__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Path__Group_1__1();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Path__Group_1__0"


    // $ANTLR start "rule__Path__Group_1__0__Impl"
    // InternalLinguaFranca.g:4241:1: rule__Path__Group_1__0__Impl : ( '.' ) ;
    public final void rule__Path__Group_1__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4245:1: ( ( '.' ) )
            // InternalLinguaFranca.g:4246:1: ( '.' )
            {
            // InternalLinguaFranca.g:4246:1: ( '.' )
            // InternalLinguaFranca.g:4247:2: '.'
            {
             before(grammarAccess.getPathAccess().getFullStopKeyword_1_0()); 
            match(input,34,FOLLOW_2); 
             after(grammarAccess.getPathAccess().getFullStopKeyword_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Path__Group_1__0__Impl"


    // $ANTLR start "rule__Path__Group_1__1"
    // InternalLinguaFranca.g:4256:1: rule__Path__Group_1__1 : rule__Path__Group_1__1__Impl ;
    public final void rule__Path__Group_1__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4260:1: ( rule__Path__Group_1__1__Impl )
            // InternalLinguaFranca.g:4261:2: rule__Path__Group_1__1__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Path__Group_1__1__Impl();

            state._fsp--;


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Path__Group_1__1"


    // $ANTLR start "rule__Path__Group_1__1__Impl"
    // InternalLinguaFranca.g:4267:1: rule__Path__Group_1__1__Impl : ( RULE_ID ) ;
    public final void rule__Path__Group_1__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4271:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4272:1: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4272:1: ( RULE_ID )
            // InternalLinguaFranca.g:4273:2: RULE_ID
            {
             before(grammarAccess.getPathAccess().getIDTerminalRuleCall_1_1()); 
            match(input,RULE_ID,FOLLOW_2); 
             after(grammarAccess.getPathAccess().getIDTerminalRuleCall_1_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Path__Group_1__1__Impl"


    // $ANTLR start "rule__Model__TargetAssignment_0"
    // InternalLinguaFranca.g:4283:1: rule__Model__TargetAssignment_0 : ( ruleTarget ) ;
    public final void rule__Model__TargetAssignment_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4287:1: ( ( ruleTarget ) )
            // InternalLinguaFranca.g:4288:2: ( ruleTarget )
            {
            // InternalLinguaFranca.g:4288:2: ( ruleTarget )
            // InternalLinguaFranca.g:4289:3: ruleTarget
            {
             before(grammarAccess.getModelAccess().getTargetTargetParserRuleCall_0_0()); 
            pushFollow(FOLLOW_2);
            ruleTarget();

            state._fsp--;

             after(grammarAccess.getModelAccess().getTargetTargetParserRuleCall_0_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Model__TargetAssignment_0"


    // $ANTLR start "rule__Model__ImportsAssignment_1"
    // InternalLinguaFranca.g:4298:1: rule__Model__ImportsAssignment_1 : ( ruleImport ) ;
    public final void rule__Model__ImportsAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4302:1: ( ( ruleImport ) )
            // InternalLinguaFranca.g:4303:2: ( ruleImport )
            {
            // InternalLinguaFranca.g:4303:2: ( ruleImport )
            // InternalLinguaFranca.g:4304:3: ruleImport
            {
             before(grammarAccess.getModelAccess().getImportsImportParserRuleCall_1_0()); 
            pushFollow(FOLLOW_2);
            ruleImport();

            state._fsp--;

             after(grammarAccess.getModelAccess().getImportsImportParserRuleCall_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Model__ImportsAssignment_1"


    // $ANTLR start "rule__Model__BlocksAssignment_2"
    // InternalLinguaFranca.g:4313:1: rule__Model__BlocksAssignment_2 : ( ( rule__Model__BlocksAlternatives_2_0 ) ) ;
    public final void rule__Model__BlocksAssignment_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4317:1: ( ( ( rule__Model__BlocksAlternatives_2_0 ) ) )
            // InternalLinguaFranca.g:4318:2: ( ( rule__Model__BlocksAlternatives_2_0 ) )
            {
            // InternalLinguaFranca.g:4318:2: ( ( rule__Model__BlocksAlternatives_2_0 ) )
            // InternalLinguaFranca.g:4319:3: ( rule__Model__BlocksAlternatives_2_0 )
            {
             before(grammarAccess.getModelAccess().getBlocksAlternatives_2_0()); 
            // InternalLinguaFranca.g:4320:3: ( rule__Model__BlocksAlternatives_2_0 )
            // InternalLinguaFranca.g:4320:4: rule__Model__BlocksAlternatives_2_0
            {
            pushFollow(FOLLOW_2);
            rule__Model__BlocksAlternatives_2_0();

            state._fsp--;


            }

             after(grammarAccess.getModelAccess().getBlocksAlternatives_2_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Model__BlocksAssignment_2"


    // $ANTLR start "rule__Target__NameAssignment_1"
    // InternalLinguaFranca.g:4328:1: rule__Target__NameAssignment_1 : ( RULE_ID ) ;
    public final void rule__Target__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4332:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4333:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4333:2: ( RULE_ID )
            // InternalLinguaFranca.g:4334:3: RULE_ID
            {
             before(grammarAccess.getTargetAccess().getNameIDTerminalRuleCall_1_0()); 
            match(input,RULE_ID,FOLLOW_2); 
             after(grammarAccess.getTargetAccess().getNameIDTerminalRuleCall_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Target__NameAssignment_1"


    // $ANTLR start "rule__Import__NameAssignment_1"
    // InternalLinguaFranca.g:4343:1: rule__Import__NameAssignment_1 : ( rulePath ) ;
    public final void rule__Import__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4347:1: ( ( rulePath ) )
            // InternalLinguaFranca.g:4348:2: ( rulePath )
            {
            // InternalLinguaFranca.g:4348:2: ( rulePath )
            // InternalLinguaFranca.g:4349:3: rulePath
            {
             before(grammarAccess.getImportAccess().getNamePathParserRuleCall_1_0()); 
            pushFollow(FOLLOW_2);
            rulePath();

            state._fsp--;

             after(grammarAccess.getImportAccess().getNamePathParserRuleCall_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Import__NameAssignment_1"


    // $ANTLR start "rule__Reactor__NameAssignment_1"
    // InternalLinguaFranca.g:4358:1: rule__Reactor__NameAssignment_1 : ( RULE_ID ) ;
    public final void rule__Reactor__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4362:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4363:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4363:2: ( RULE_ID )
            // InternalLinguaFranca.g:4364:3: RULE_ID
            {
             before(grammarAccess.getReactorAccess().getNameIDTerminalRuleCall_1_0()); 
            match(input,RULE_ID,FOLLOW_2); 
             after(grammarAccess.getReactorAccess().getNameIDTerminalRuleCall_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__NameAssignment_1"


    // $ANTLR start "rule__Reactor__ParametersAssignment_2"
    // InternalLinguaFranca.g:4373:1: rule__Reactor__ParametersAssignment_2 : ( ruleParams ) ;
    public final void rule__Reactor__ParametersAssignment_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4377:1: ( ( ruleParams ) )
            // InternalLinguaFranca.g:4378:2: ( ruleParams )
            {
            // InternalLinguaFranca.g:4378:2: ( ruleParams )
            // InternalLinguaFranca.g:4379:3: ruleParams
            {
             before(grammarAccess.getReactorAccess().getParametersParamsParserRuleCall_2_0()); 
            pushFollow(FOLLOW_2);
            ruleParams();

            state._fsp--;

             after(grammarAccess.getReactorAccess().getParametersParamsParserRuleCall_2_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__ParametersAssignment_2"


    // $ANTLR start "rule__Reactor__InputsAssignment_4"
    // InternalLinguaFranca.g:4388:1: rule__Reactor__InputsAssignment_4 : ( ruleInput ) ;
    public final void rule__Reactor__InputsAssignment_4() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4392:1: ( ( ruleInput ) )
            // InternalLinguaFranca.g:4393:2: ( ruleInput )
            {
            // InternalLinguaFranca.g:4393:2: ( ruleInput )
            // InternalLinguaFranca.g:4394:3: ruleInput
            {
             before(grammarAccess.getReactorAccess().getInputsInputParserRuleCall_4_0()); 
            pushFollow(FOLLOW_2);
            ruleInput();

            state._fsp--;

             after(grammarAccess.getReactorAccess().getInputsInputParserRuleCall_4_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__InputsAssignment_4"


    // $ANTLR start "rule__Reactor__OutputsAssignment_5"
    // InternalLinguaFranca.g:4403:1: rule__Reactor__OutputsAssignment_5 : ( ruleOutput ) ;
    public final void rule__Reactor__OutputsAssignment_5() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4407:1: ( ( ruleOutput ) )
            // InternalLinguaFranca.g:4408:2: ( ruleOutput )
            {
            // InternalLinguaFranca.g:4408:2: ( ruleOutput )
            // InternalLinguaFranca.g:4409:3: ruleOutput
            {
             before(grammarAccess.getReactorAccess().getOutputsOutputParserRuleCall_5_0()); 
            pushFollow(FOLLOW_2);
            ruleOutput();

            state._fsp--;

             after(grammarAccess.getReactorAccess().getOutputsOutputParserRuleCall_5_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__OutputsAssignment_5"


    // $ANTLR start "rule__Reactor__ClocksAssignment_6"
    // InternalLinguaFranca.g:4418:1: rule__Reactor__ClocksAssignment_6 : ( ruleClock ) ;
    public final void rule__Reactor__ClocksAssignment_6() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4422:1: ( ( ruleClock ) )
            // InternalLinguaFranca.g:4423:2: ( ruleClock )
            {
            // InternalLinguaFranca.g:4423:2: ( ruleClock )
            // InternalLinguaFranca.g:4424:3: ruleClock
            {
             before(grammarAccess.getReactorAccess().getClocksClockParserRuleCall_6_0()); 
            pushFollow(FOLLOW_2);
            ruleClock();

            state._fsp--;

             after(grammarAccess.getReactorAccess().getClocksClockParserRuleCall_6_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__ClocksAssignment_6"


    // $ANTLR start "rule__Reactor__PreambleAssignment_7"
    // InternalLinguaFranca.g:4433:1: rule__Reactor__PreambleAssignment_7 : ( rulePreamble ) ;
    public final void rule__Reactor__PreambleAssignment_7() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4437:1: ( ( rulePreamble ) )
            // InternalLinguaFranca.g:4438:2: ( rulePreamble )
            {
            // InternalLinguaFranca.g:4438:2: ( rulePreamble )
            // InternalLinguaFranca.g:4439:3: rulePreamble
            {
             before(grammarAccess.getReactorAccess().getPreamblePreambleParserRuleCall_7_0()); 
            pushFollow(FOLLOW_2);
            rulePreamble();

            state._fsp--;

             after(grammarAccess.getReactorAccess().getPreamblePreambleParserRuleCall_7_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__PreambleAssignment_7"


    // $ANTLR start "rule__Reactor__ConstructorAssignment_8"
    // InternalLinguaFranca.g:4448:1: rule__Reactor__ConstructorAssignment_8 : ( ruleConstructor ) ;
    public final void rule__Reactor__ConstructorAssignment_8() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4452:1: ( ( ruleConstructor ) )
            // InternalLinguaFranca.g:4453:2: ( ruleConstructor )
            {
            // InternalLinguaFranca.g:4453:2: ( ruleConstructor )
            // InternalLinguaFranca.g:4454:3: ruleConstructor
            {
             before(grammarAccess.getReactorAccess().getConstructorConstructorParserRuleCall_8_0()); 
            pushFollow(FOLLOW_2);
            ruleConstructor();

            state._fsp--;

             after(grammarAccess.getReactorAccess().getConstructorConstructorParserRuleCall_8_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__ConstructorAssignment_8"


    // $ANTLR start "rule__Reactor__ReactionsAssignment_9"
    // InternalLinguaFranca.g:4463:1: rule__Reactor__ReactionsAssignment_9 : ( ruleReaction ) ;
    public final void rule__Reactor__ReactionsAssignment_9() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4467:1: ( ( ruleReaction ) )
            // InternalLinguaFranca.g:4468:2: ( ruleReaction )
            {
            // InternalLinguaFranca.g:4468:2: ( ruleReaction )
            // InternalLinguaFranca.g:4469:3: ruleReaction
            {
             before(grammarAccess.getReactorAccess().getReactionsReactionParserRuleCall_9_0()); 
            pushFollow(FOLLOW_2);
            ruleReaction();

            state._fsp--;

             after(grammarAccess.getReactorAccess().getReactionsReactionParserRuleCall_9_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reactor__ReactionsAssignment_9"


    // $ANTLR start "rule__Composite__NameAssignment_1"
    // InternalLinguaFranca.g:4478:1: rule__Composite__NameAssignment_1 : ( RULE_ID ) ;
    public final void rule__Composite__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4482:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4483:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4483:2: ( RULE_ID )
            // InternalLinguaFranca.g:4484:3: RULE_ID
            {
             before(grammarAccess.getCompositeAccess().getNameIDTerminalRuleCall_1_0()); 
            match(input,RULE_ID,FOLLOW_2); 
             after(grammarAccess.getCompositeAccess().getNameIDTerminalRuleCall_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__NameAssignment_1"


    // $ANTLR start "rule__Composite__ParametersAssignment_2"
    // InternalLinguaFranca.g:4493:1: rule__Composite__ParametersAssignment_2 : ( ruleParams ) ;
    public final void rule__Composite__ParametersAssignment_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4497:1: ( ( ruleParams ) )
            // InternalLinguaFranca.g:4498:2: ( ruleParams )
            {
            // InternalLinguaFranca.g:4498:2: ( ruleParams )
            // InternalLinguaFranca.g:4499:3: ruleParams
            {
             before(grammarAccess.getCompositeAccess().getParametersParamsParserRuleCall_2_0()); 
            pushFollow(FOLLOW_2);
            ruleParams();

            state._fsp--;

             after(grammarAccess.getCompositeAccess().getParametersParamsParserRuleCall_2_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__ParametersAssignment_2"


    // $ANTLR start "rule__Composite__InputsAssignment_4"
    // InternalLinguaFranca.g:4508:1: rule__Composite__InputsAssignment_4 : ( ruleInput ) ;
    public final void rule__Composite__InputsAssignment_4() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4512:1: ( ( ruleInput ) )
            // InternalLinguaFranca.g:4513:2: ( ruleInput )
            {
            // InternalLinguaFranca.g:4513:2: ( ruleInput )
            // InternalLinguaFranca.g:4514:3: ruleInput
            {
             before(grammarAccess.getCompositeAccess().getInputsInputParserRuleCall_4_0()); 
            pushFollow(FOLLOW_2);
            ruleInput();

            state._fsp--;

             after(grammarAccess.getCompositeAccess().getInputsInputParserRuleCall_4_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__InputsAssignment_4"


    // $ANTLR start "rule__Composite__OutputsAssignment_5"
    // InternalLinguaFranca.g:4523:1: rule__Composite__OutputsAssignment_5 : ( ruleOutput ) ;
    public final void rule__Composite__OutputsAssignment_5() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4527:1: ( ( ruleOutput ) )
            // InternalLinguaFranca.g:4528:2: ( ruleOutput )
            {
            // InternalLinguaFranca.g:4528:2: ( ruleOutput )
            // InternalLinguaFranca.g:4529:3: ruleOutput
            {
             before(grammarAccess.getCompositeAccess().getOutputsOutputParserRuleCall_5_0()); 
            pushFollow(FOLLOW_2);
            ruleOutput();

            state._fsp--;

             after(grammarAccess.getCompositeAccess().getOutputsOutputParserRuleCall_5_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__OutputsAssignment_5"


    // $ANTLR start "rule__Composite__ClocksAssignment_6"
    // InternalLinguaFranca.g:4538:1: rule__Composite__ClocksAssignment_6 : ( ruleClock ) ;
    public final void rule__Composite__ClocksAssignment_6() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4542:1: ( ( ruleClock ) )
            // InternalLinguaFranca.g:4543:2: ( ruleClock )
            {
            // InternalLinguaFranca.g:4543:2: ( ruleClock )
            // InternalLinguaFranca.g:4544:3: ruleClock
            {
             before(grammarAccess.getCompositeAccess().getClocksClockParserRuleCall_6_0()); 
            pushFollow(FOLLOW_2);
            ruleClock();

            state._fsp--;

             after(grammarAccess.getCompositeAccess().getClocksClockParserRuleCall_6_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__ClocksAssignment_6"


    // $ANTLR start "rule__Composite__PreambleAssignment_7"
    // InternalLinguaFranca.g:4553:1: rule__Composite__PreambleAssignment_7 : ( rulePreamble ) ;
    public final void rule__Composite__PreambleAssignment_7() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4557:1: ( ( rulePreamble ) )
            // InternalLinguaFranca.g:4558:2: ( rulePreamble )
            {
            // InternalLinguaFranca.g:4558:2: ( rulePreamble )
            // InternalLinguaFranca.g:4559:3: rulePreamble
            {
             before(grammarAccess.getCompositeAccess().getPreamblePreambleParserRuleCall_7_0()); 
            pushFollow(FOLLOW_2);
            rulePreamble();

            state._fsp--;

             after(grammarAccess.getCompositeAccess().getPreamblePreambleParserRuleCall_7_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__PreambleAssignment_7"


    // $ANTLR start "rule__Composite__ConstructorAssignment_8"
    // InternalLinguaFranca.g:4568:1: rule__Composite__ConstructorAssignment_8 : ( ruleConstructor ) ;
    public final void rule__Composite__ConstructorAssignment_8() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4572:1: ( ( ruleConstructor ) )
            // InternalLinguaFranca.g:4573:2: ( ruleConstructor )
            {
            // InternalLinguaFranca.g:4573:2: ( ruleConstructor )
            // InternalLinguaFranca.g:4574:3: ruleConstructor
            {
             before(grammarAccess.getCompositeAccess().getConstructorConstructorParserRuleCall_8_0()); 
            pushFollow(FOLLOW_2);
            ruleConstructor();

            state._fsp--;

             after(grammarAccess.getCompositeAccess().getConstructorConstructorParserRuleCall_8_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__ConstructorAssignment_8"


    // $ANTLR start "rule__Composite__ReactionsAssignment_9"
    // InternalLinguaFranca.g:4583:1: rule__Composite__ReactionsAssignment_9 : ( ruleReaction ) ;
    public final void rule__Composite__ReactionsAssignment_9() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4587:1: ( ( ruleReaction ) )
            // InternalLinguaFranca.g:4588:2: ( ruleReaction )
            {
            // InternalLinguaFranca.g:4588:2: ( ruleReaction )
            // InternalLinguaFranca.g:4589:3: ruleReaction
            {
             before(grammarAccess.getCompositeAccess().getReactionsReactionParserRuleCall_9_0()); 
            pushFollow(FOLLOW_2);
            ruleReaction();

            state._fsp--;

             after(grammarAccess.getCompositeAccess().getReactionsReactionParserRuleCall_9_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__ReactionsAssignment_9"


    // $ANTLR start "rule__Composite__InstancesAssignment_10"
    // InternalLinguaFranca.g:4598:1: rule__Composite__InstancesAssignment_10 : ( ruleInstance ) ;
    public final void rule__Composite__InstancesAssignment_10() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4602:1: ( ( ruleInstance ) )
            // InternalLinguaFranca.g:4603:2: ( ruleInstance )
            {
            // InternalLinguaFranca.g:4603:2: ( ruleInstance )
            // InternalLinguaFranca.g:4604:3: ruleInstance
            {
             before(grammarAccess.getCompositeAccess().getInstancesInstanceParserRuleCall_10_0()); 
            pushFollow(FOLLOW_2);
            ruleInstance();

            state._fsp--;

             after(grammarAccess.getCompositeAccess().getInstancesInstanceParserRuleCall_10_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__InstancesAssignment_10"


    // $ANTLR start "rule__Composite__ConnectionsAssignment_11"
    // InternalLinguaFranca.g:4613:1: rule__Composite__ConnectionsAssignment_11 : ( ruleConnection ) ;
    public final void rule__Composite__ConnectionsAssignment_11() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4617:1: ( ( ruleConnection ) )
            // InternalLinguaFranca.g:4618:2: ( ruleConnection )
            {
            // InternalLinguaFranca.g:4618:2: ( ruleConnection )
            // InternalLinguaFranca.g:4619:3: ruleConnection
            {
             before(grammarAccess.getCompositeAccess().getConnectionsConnectionParserRuleCall_11_0()); 
            pushFollow(FOLLOW_2);
            ruleConnection();

            state._fsp--;

             after(grammarAccess.getCompositeAccess().getConnectionsConnectionParserRuleCall_11_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__ConnectionsAssignment_11"


    // $ANTLR start "rule__Input__NameAssignment_1"
    // InternalLinguaFranca.g:4628:1: rule__Input__NameAssignment_1 : ( ( rule__Input__NameAlternatives_1_0 ) ) ;
    public final void rule__Input__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4632:1: ( ( ( rule__Input__NameAlternatives_1_0 ) ) )
            // InternalLinguaFranca.g:4633:2: ( ( rule__Input__NameAlternatives_1_0 ) )
            {
            // InternalLinguaFranca.g:4633:2: ( ( rule__Input__NameAlternatives_1_0 ) )
            // InternalLinguaFranca.g:4634:3: ( rule__Input__NameAlternatives_1_0 )
            {
             before(grammarAccess.getInputAccess().getNameAlternatives_1_0()); 
            // InternalLinguaFranca.g:4635:3: ( rule__Input__NameAlternatives_1_0 )
            // InternalLinguaFranca.g:4635:4: rule__Input__NameAlternatives_1_0
            {
            pushFollow(FOLLOW_2);
            rule__Input__NameAlternatives_1_0();

            state._fsp--;


            }

             after(grammarAccess.getInputAccess().getNameAlternatives_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Input__NameAssignment_1"


    // $ANTLR start "rule__Input__TypeAssignment_2_1"
    // InternalLinguaFranca.g:4643:1: rule__Input__TypeAssignment_2_1 : ( ruleType ) ;
    public final void rule__Input__TypeAssignment_2_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4647:1: ( ( ruleType ) )
            // InternalLinguaFranca.g:4648:2: ( ruleType )
            {
            // InternalLinguaFranca.g:4648:2: ( ruleType )
            // InternalLinguaFranca.g:4649:3: ruleType
            {
             before(grammarAccess.getInputAccess().getTypeTypeParserRuleCall_2_1_0()); 
            pushFollow(FOLLOW_2);
            ruleType();

            state._fsp--;

             after(grammarAccess.getInputAccess().getTypeTypeParserRuleCall_2_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Input__TypeAssignment_2_1"


    // $ANTLR start "rule__Output__NameAssignment_1"
    // InternalLinguaFranca.g:4658:1: rule__Output__NameAssignment_1 : ( ( rule__Output__NameAlternatives_1_0 ) ) ;
    public final void rule__Output__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4662:1: ( ( ( rule__Output__NameAlternatives_1_0 ) ) )
            // InternalLinguaFranca.g:4663:2: ( ( rule__Output__NameAlternatives_1_0 ) )
            {
            // InternalLinguaFranca.g:4663:2: ( ( rule__Output__NameAlternatives_1_0 ) )
            // InternalLinguaFranca.g:4664:3: ( rule__Output__NameAlternatives_1_0 )
            {
             before(grammarAccess.getOutputAccess().getNameAlternatives_1_0()); 
            // InternalLinguaFranca.g:4665:3: ( rule__Output__NameAlternatives_1_0 )
            // InternalLinguaFranca.g:4665:4: rule__Output__NameAlternatives_1_0
            {
            pushFollow(FOLLOW_2);
            rule__Output__NameAlternatives_1_0();

            state._fsp--;


            }

             after(grammarAccess.getOutputAccess().getNameAlternatives_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Output__NameAssignment_1"


    // $ANTLR start "rule__Output__TypeAssignment_2_1"
    // InternalLinguaFranca.g:4673:1: rule__Output__TypeAssignment_2_1 : ( ruleType ) ;
    public final void rule__Output__TypeAssignment_2_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4677:1: ( ( ruleType ) )
            // InternalLinguaFranca.g:4678:2: ( ruleType )
            {
            // InternalLinguaFranca.g:4678:2: ( ruleType )
            // InternalLinguaFranca.g:4679:3: ruleType
            {
             before(grammarAccess.getOutputAccess().getTypeTypeParserRuleCall_2_1_0()); 
            pushFollow(FOLLOW_2);
            ruleType();

            state._fsp--;

             after(grammarAccess.getOutputAccess().getTypeTypeParserRuleCall_2_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Output__TypeAssignment_2_1"


    // $ANTLR start "rule__Clock__NameAssignment_1"
    // InternalLinguaFranca.g:4688:1: rule__Clock__NameAssignment_1 : ( ( rule__Clock__NameAlternatives_1_0 ) ) ;
    public final void rule__Clock__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4692:1: ( ( ( rule__Clock__NameAlternatives_1_0 ) ) )
            // InternalLinguaFranca.g:4693:2: ( ( rule__Clock__NameAlternatives_1_0 ) )
            {
            // InternalLinguaFranca.g:4693:2: ( ( rule__Clock__NameAlternatives_1_0 ) )
            // InternalLinguaFranca.g:4694:3: ( rule__Clock__NameAlternatives_1_0 )
            {
             before(grammarAccess.getClockAccess().getNameAlternatives_1_0()); 
            // InternalLinguaFranca.g:4695:3: ( rule__Clock__NameAlternatives_1_0 )
            // InternalLinguaFranca.g:4695:4: rule__Clock__NameAlternatives_1_0
            {
            pushFollow(FOLLOW_2);
            rule__Clock__NameAlternatives_1_0();

            state._fsp--;


            }

             after(grammarAccess.getClockAccess().getNameAlternatives_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Clock__NameAssignment_1"


    // $ANTLR start "rule__Clock__PeriodAssignment_2"
    // InternalLinguaFranca.g:4703:1: rule__Clock__PeriodAssignment_2 : ( rulePeriod ) ;
    public final void rule__Clock__PeriodAssignment_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4707:1: ( ( rulePeriod ) )
            // InternalLinguaFranca.g:4708:2: ( rulePeriod )
            {
            // InternalLinguaFranca.g:4708:2: ( rulePeriod )
            // InternalLinguaFranca.g:4709:3: rulePeriod
            {
             before(grammarAccess.getClockAccess().getPeriodPeriodParserRuleCall_2_0()); 
            pushFollow(FOLLOW_2);
            rulePeriod();

            state._fsp--;

             after(grammarAccess.getClockAccess().getPeriodPeriodParserRuleCall_2_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Clock__PeriodAssignment_2"


    // $ANTLR start "rule__Reaction__TriggersAssignment_1_1_0"
    // InternalLinguaFranca.g:4718:1: rule__Reaction__TriggersAssignment_1_1_0 : ( RULE_ID ) ;
    public final void rule__Reaction__TriggersAssignment_1_1_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4722:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4723:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4723:2: ( RULE_ID )
            // InternalLinguaFranca.g:4724:3: RULE_ID
            {
             before(grammarAccess.getReactionAccess().getTriggersIDTerminalRuleCall_1_1_0_0()); 
            match(input,RULE_ID,FOLLOW_2); 
             after(grammarAccess.getReactionAccess().getTriggersIDTerminalRuleCall_1_1_0_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__TriggersAssignment_1_1_0"


    // $ANTLR start "rule__Reaction__TriggersAssignment_1_1_1_1"
    // InternalLinguaFranca.g:4733:1: rule__Reaction__TriggersAssignment_1_1_1_1 : ( RULE_ID ) ;
    public final void rule__Reaction__TriggersAssignment_1_1_1_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4737:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4738:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4738:2: ( RULE_ID )
            // InternalLinguaFranca.g:4739:3: RULE_ID
            {
             before(grammarAccess.getReactionAccess().getTriggersIDTerminalRuleCall_1_1_1_1_0()); 
            match(input,RULE_ID,FOLLOW_2); 
             after(grammarAccess.getReactionAccess().getTriggersIDTerminalRuleCall_1_1_1_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__TriggersAssignment_1_1_1_1"


    // $ANTLR start "rule__Reaction__GetsAssignment_2"
    // InternalLinguaFranca.g:4748:1: rule__Reaction__GetsAssignment_2 : ( ruleGets ) ;
    public final void rule__Reaction__GetsAssignment_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4752:1: ( ( ruleGets ) )
            // InternalLinguaFranca.g:4753:2: ( ruleGets )
            {
            // InternalLinguaFranca.g:4753:2: ( ruleGets )
            // InternalLinguaFranca.g:4754:3: ruleGets
            {
             before(grammarAccess.getReactionAccess().getGetsGetsParserRuleCall_2_0()); 
            pushFollow(FOLLOW_2);
            ruleGets();

            state._fsp--;

             after(grammarAccess.getReactionAccess().getGetsGetsParserRuleCall_2_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__GetsAssignment_2"


    // $ANTLR start "rule__Reaction__SetsAssignment_3"
    // InternalLinguaFranca.g:4763:1: rule__Reaction__SetsAssignment_3 : ( ruleSets ) ;
    public final void rule__Reaction__SetsAssignment_3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4767:1: ( ( ruleSets ) )
            // InternalLinguaFranca.g:4768:2: ( ruleSets )
            {
            // InternalLinguaFranca.g:4768:2: ( ruleSets )
            // InternalLinguaFranca.g:4769:3: ruleSets
            {
             before(grammarAccess.getReactionAccess().getSetsSetsParserRuleCall_3_0()); 
            pushFollow(FOLLOW_2);
            ruleSets();

            state._fsp--;

             after(grammarAccess.getReactionAccess().getSetsSetsParserRuleCall_3_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__SetsAssignment_3"


    // $ANTLR start "rule__Reaction__CodeAssignment_4"
    // InternalLinguaFranca.g:4778:1: rule__Reaction__CodeAssignment_4 : ( RULE_CODE ) ;
    public final void rule__Reaction__CodeAssignment_4() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4782:1: ( ( RULE_CODE ) )
            // InternalLinguaFranca.g:4783:2: ( RULE_CODE )
            {
            // InternalLinguaFranca.g:4783:2: ( RULE_CODE )
            // InternalLinguaFranca.g:4784:3: RULE_CODE
            {
             before(grammarAccess.getReactionAccess().getCodeCODETerminalRuleCall_4_0()); 
            match(input,RULE_CODE,FOLLOW_2); 
             after(grammarAccess.getReactionAccess().getCodeCODETerminalRuleCall_4_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Reaction__CodeAssignment_4"


    // $ANTLR start "rule__Preamble__CodeAssignment_1"
    // InternalLinguaFranca.g:4793:1: rule__Preamble__CodeAssignment_1 : ( RULE_CODE ) ;
    public final void rule__Preamble__CodeAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4797:1: ( ( RULE_CODE ) )
            // InternalLinguaFranca.g:4798:2: ( RULE_CODE )
            {
            // InternalLinguaFranca.g:4798:2: ( RULE_CODE )
            // InternalLinguaFranca.g:4799:3: RULE_CODE
            {
             before(grammarAccess.getPreambleAccess().getCodeCODETerminalRuleCall_1_0()); 
            match(input,RULE_CODE,FOLLOW_2); 
             after(grammarAccess.getPreambleAccess().getCodeCODETerminalRuleCall_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Preamble__CodeAssignment_1"


    // $ANTLR start "rule__Constructor__CodeAssignment_1"
    // InternalLinguaFranca.g:4808:1: rule__Constructor__CodeAssignment_1 : ( RULE_CODE ) ;
    public final void rule__Constructor__CodeAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4812:1: ( ( RULE_CODE ) )
            // InternalLinguaFranca.g:4813:2: ( RULE_CODE )
            {
            // InternalLinguaFranca.g:4813:2: ( RULE_CODE )
            // InternalLinguaFranca.g:4814:3: RULE_CODE
            {
             before(grammarAccess.getConstructorAccess().getCodeCODETerminalRuleCall_1_0()); 
            match(input,RULE_CODE,FOLLOW_2); 
             after(grammarAccess.getConstructorAccess().getCodeCODETerminalRuleCall_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Constructor__CodeAssignment_1"


    // $ANTLR start "rule__Instance__NameAssignment_0"
    // InternalLinguaFranca.g:4823:1: rule__Instance__NameAssignment_0 : ( RULE_ID ) ;
    public final void rule__Instance__NameAssignment_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4827:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4828:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4828:2: ( RULE_ID )
            // InternalLinguaFranca.g:4829:3: RULE_ID
            {
             before(grammarAccess.getInstanceAccess().getNameIDTerminalRuleCall_0_0()); 
            match(input,RULE_ID,FOLLOW_2); 
             after(grammarAccess.getInstanceAccess().getNameIDTerminalRuleCall_0_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Instance__NameAssignment_0"


    // $ANTLR start "rule__Instance__ActorClassAssignment_3"
    // InternalLinguaFranca.g:4838:1: rule__Instance__ActorClassAssignment_3 : ( RULE_ID ) ;
    public final void rule__Instance__ActorClassAssignment_3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4842:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4843:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4843:2: ( RULE_ID )
            // InternalLinguaFranca.g:4844:3: RULE_ID
            {
             before(grammarAccess.getInstanceAccess().getActorClassIDTerminalRuleCall_3_0()); 
            match(input,RULE_ID,FOLLOW_2); 
             after(grammarAccess.getInstanceAccess().getActorClassIDTerminalRuleCall_3_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Instance__ActorClassAssignment_3"


    // $ANTLR start "rule__Instance__ParametersAssignment_4_1"
    // InternalLinguaFranca.g:4853:1: rule__Instance__ParametersAssignment_4_1 : ( ruleAssignments ) ;
    public final void rule__Instance__ParametersAssignment_4_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4857:1: ( ( ruleAssignments ) )
            // InternalLinguaFranca.g:4858:2: ( ruleAssignments )
            {
            // InternalLinguaFranca.g:4858:2: ( ruleAssignments )
            // InternalLinguaFranca.g:4859:3: ruleAssignments
            {
             before(grammarAccess.getInstanceAccess().getParametersAssignmentsParserRuleCall_4_1_0()); 
            pushFollow(FOLLOW_2);
            ruleAssignments();

            state._fsp--;

             after(grammarAccess.getInstanceAccess().getParametersAssignmentsParserRuleCall_4_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Instance__ParametersAssignment_4_1"


    // $ANTLR start "rule__Connection__LeftPortAssignment_0"
    // InternalLinguaFranca.g:4868:1: rule__Connection__LeftPortAssignment_0 : ( rulePort ) ;
    public final void rule__Connection__LeftPortAssignment_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4872:1: ( ( rulePort ) )
            // InternalLinguaFranca.g:4873:2: ( rulePort )
            {
            // InternalLinguaFranca.g:4873:2: ( rulePort )
            // InternalLinguaFranca.g:4874:3: rulePort
            {
             before(grammarAccess.getConnectionAccess().getLeftPortPortParserRuleCall_0_0()); 
            pushFollow(FOLLOW_2);
            rulePort();

            state._fsp--;

             after(grammarAccess.getConnectionAccess().getLeftPortPortParserRuleCall_0_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Connection__LeftPortAssignment_0"


    // $ANTLR start "rule__Connection__RightPortAssignment_2"
    // InternalLinguaFranca.g:4883:1: rule__Connection__RightPortAssignment_2 : ( rulePort ) ;
    public final void rule__Connection__RightPortAssignment_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4887:1: ( ( rulePort ) )
            // InternalLinguaFranca.g:4888:2: ( rulePort )
            {
            // InternalLinguaFranca.g:4888:2: ( rulePort )
            // InternalLinguaFranca.g:4889:3: rulePort
            {
             before(grammarAccess.getConnectionAccess().getRightPortPortParserRuleCall_2_0()); 
            pushFollow(FOLLOW_2);
            rulePort();

            state._fsp--;

             after(grammarAccess.getConnectionAccess().getRightPortPortParserRuleCall_2_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Connection__RightPortAssignment_2"


    // $ANTLR start "rule__Assignments__AssignmentsAssignment_0"
    // InternalLinguaFranca.g:4898:1: rule__Assignments__AssignmentsAssignment_0 : ( ruleAssignment ) ;
    public final void rule__Assignments__AssignmentsAssignment_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4902:1: ( ( ruleAssignment ) )
            // InternalLinguaFranca.g:4903:2: ( ruleAssignment )
            {
            // InternalLinguaFranca.g:4903:2: ( ruleAssignment )
            // InternalLinguaFranca.g:4904:3: ruleAssignment
            {
             before(grammarAccess.getAssignmentsAccess().getAssignmentsAssignmentParserRuleCall_0_0()); 
            pushFollow(FOLLOW_2);
            ruleAssignment();

            state._fsp--;

             after(grammarAccess.getAssignmentsAccess().getAssignmentsAssignmentParserRuleCall_0_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Assignments__AssignmentsAssignment_0"


    // $ANTLR start "rule__Assignments__AssignmentsAssignment_1_1"
    // InternalLinguaFranca.g:4913:1: rule__Assignments__AssignmentsAssignment_1_1 : ( ruleAssignment ) ;
    public final void rule__Assignments__AssignmentsAssignment_1_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4917:1: ( ( ruleAssignment ) )
            // InternalLinguaFranca.g:4918:2: ( ruleAssignment )
            {
            // InternalLinguaFranca.g:4918:2: ( ruleAssignment )
            // InternalLinguaFranca.g:4919:3: ruleAssignment
            {
             before(grammarAccess.getAssignmentsAccess().getAssignmentsAssignmentParserRuleCall_1_1_0()); 
            pushFollow(FOLLOW_2);
            ruleAssignment();

            state._fsp--;

             after(grammarAccess.getAssignmentsAccess().getAssignmentsAssignmentParserRuleCall_1_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Assignments__AssignmentsAssignment_1_1"


    // $ANTLR start "rule__Assignment__NameAssignment_0"
    // InternalLinguaFranca.g:4928:1: rule__Assignment__NameAssignment_0 : ( RULE_ID ) ;
    public final void rule__Assignment__NameAssignment_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4932:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4933:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4933:2: ( RULE_ID )
            // InternalLinguaFranca.g:4934:3: RULE_ID
            {
             before(grammarAccess.getAssignmentAccess().getNameIDTerminalRuleCall_0_0()); 
            match(input,RULE_ID,FOLLOW_2); 
             after(grammarAccess.getAssignmentAccess().getNameIDTerminalRuleCall_0_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Assignment__NameAssignment_0"


    // $ANTLR start "rule__Assignment__ValueAssignment_2"
    // InternalLinguaFranca.g:4943:1: rule__Assignment__ValueAssignment_2 : ( ruleValue ) ;
    public final void rule__Assignment__ValueAssignment_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4947:1: ( ( ruleValue ) )
            // InternalLinguaFranca.g:4948:2: ( ruleValue )
            {
            // InternalLinguaFranca.g:4948:2: ( ruleValue )
            // InternalLinguaFranca.g:4949:3: ruleValue
            {
             before(grammarAccess.getAssignmentAccess().getValueValueParserRuleCall_2_0()); 
            pushFollow(FOLLOW_2);
            ruleValue();

            state._fsp--;

             after(grammarAccess.getAssignmentAccess().getValueValueParserRuleCall_2_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Assignment__ValueAssignment_2"


    // $ANTLR start "rule__Gets__GetsAssignment_0"
    // InternalLinguaFranca.g:4958:1: rule__Gets__GetsAssignment_0 : ( RULE_ID ) ;
    public final void rule__Gets__GetsAssignment_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4962:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4963:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4963:2: ( RULE_ID )
            // InternalLinguaFranca.g:4964:3: RULE_ID
            {
             before(grammarAccess.getGetsAccess().getGetsIDTerminalRuleCall_0_0()); 
            match(input,RULE_ID,FOLLOW_2); 
             after(grammarAccess.getGetsAccess().getGetsIDTerminalRuleCall_0_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Gets__GetsAssignment_0"


    // $ANTLR start "rule__Gets__GetsAssignment_1_1"
    // InternalLinguaFranca.g:4973:1: rule__Gets__GetsAssignment_1_1 : ( RULE_ID ) ;
    public final void rule__Gets__GetsAssignment_1_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4977:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4978:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4978:2: ( RULE_ID )
            // InternalLinguaFranca.g:4979:3: RULE_ID
            {
             before(grammarAccess.getGetsAccess().getGetsIDTerminalRuleCall_1_1_0()); 
            match(input,RULE_ID,FOLLOW_2); 
             after(grammarAccess.getGetsAccess().getGetsIDTerminalRuleCall_1_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Gets__GetsAssignment_1_1"


    // $ANTLR start "rule__Params__ParamsAssignment_1"
    // InternalLinguaFranca.g:4988:1: rule__Params__ParamsAssignment_1 : ( ruleParam ) ;
    public final void rule__Params__ParamsAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4992:1: ( ( ruleParam ) )
            // InternalLinguaFranca.g:4993:2: ( ruleParam )
            {
            // InternalLinguaFranca.g:4993:2: ( ruleParam )
            // InternalLinguaFranca.g:4994:3: ruleParam
            {
             before(grammarAccess.getParamsAccess().getParamsParamParserRuleCall_1_0()); 
            pushFollow(FOLLOW_2);
            ruleParam();

            state._fsp--;

             after(grammarAccess.getParamsAccess().getParamsParamParserRuleCall_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Params__ParamsAssignment_1"


    // $ANTLR start "rule__Params__ParamsAssignment_2_1"
    // InternalLinguaFranca.g:5003:1: rule__Params__ParamsAssignment_2_1 : ( ruleParam ) ;
    public final void rule__Params__ParamsAssignment_2_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:5007:1: ( ( ruleParam ) )
            // InternalLinguaFranca.g:5008:2: ( ruleParam )
            {
            // InternalLinguaFranca.g:5008:2: ( ruleParam )
            // InternalLinguaFranca.g:5009:3: ruleParam
            {
             before(grammarAccess.getParamsAccess().getParamsParamParserRuleCall_2_1_0()); 
            pushFollow(FOLLOW_2);
            ruleParam();

            state._fsp--;

             after(grammarAccess.getParamsAccess().getParamsParamParserRuleCall_2_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Params__ParamsAssignment_2_1"


    // $ANTLR start "rule__Param__NameAssignment_1"
    // InternalLinguaFranca.g:5018:1: rule__Param__NameAssignment_1 : ( RULE_ID ) ;
    public final void rule__Param__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:5022:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:5023:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:5023:2: ( RULE_ID )
            // InternalLinguaFranca.g:5024:3: RULE_ID
            {
             before(grammarAccess.getParamAccess().getNameIDTerminalRuleCall_1_0()); 
            match(input,RULE_ID,FOLLOW_2); 
             after(grammarAccess.getParamAccess().getNameIDTerminalRuleCall_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Param__NameAssignment_1"


    // $ANTLR start "rule__Param__TypeAssignment_2_1"
    // InternalLinguaFranca.g:5033:1: rule__Param__TypeAssignment_2_1 : ( ruleType ) ;
    public final void rule__Param__TypeAssignment_2_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:5037:1: ( ( ruleType ) )
            // InternalLinguaFranca.g:5038:2: ( ruleType )
            {
            // InternalLinguaFranca.g:5038:2: ( ruleType )
            // InternalLinguaFranca.g:5039:3: ruleType
            {
             before(grammarAccess.getParamAccess().getTypeTypeParserRuleCall_2_1_0()); 
            pushFollow(FOLLOW_2);
            ruleType();

            state._fsp--;

             after(grammarAccess.getParamAccess().getTypeTypeParserRuleCall_2_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Param__TypeAssignment_2_1"


    // $ANTLR start "rule__Param__ValueAssignment_3_1"
    // InternalLinguaFranca.g:5048:1: rule__Param__ValueAssignment_3_1 : ( ruleValue ) ;
    public final void rule__Param__ValueAssignment_3_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:5052:1: ( ( ruleValue ) )
            // InternalLinguaFranca.g:5053:2: ( ruleValue )
            {
            // InternalLinguaFranca.g:5053:2: ( ruleValue )
            // InternalLinguaFranca.g:5054:3: ruleValue
            {
             before(grammarAccess.getParamAccess().getValueValueParserRuleCall_3_1_0()); 
            pushFollow(FOLLOW_2);
            ruleValue();

            state._fsp--;

             after(grammarAccess.getParamAccess().getValueValueParserRuleCall_3_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Param__ValueAssignment_3_1"


    // $ANTLR start "rule__Period__PeriodAssignment_1"
    // InternalLinguaFranca.g:5063:1: rule__Period__PeriodAssignment_1 : ( ( rule__Period__PeriodAlternatives_1_0 ) ) ;
    public final void rule__Period__PeriodAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:5067:1: ( ( ( rule__Period__PeriodAlternatives_1_0 ) ) )
            // InternalLinguaFranca.g:5068:2: ( ( rule__Period__PeriodAlternatives_1_0 ) )
            {
            // InternalLinguaFranca.g:5068:2: ( ( rule__Period__PeriodAlternatives_1_0 ) )
            // InternalLinguaFranca.g:5069:3: ( rule__Period__PeriodAlternatives_1_0 )
            {
             before(grammarAccess.getPeriodAccess().getPeriodAlternatives_1_0()); 
            // InternalLinguaFranca.g:5070:3: ( rule__Period__PeriodAlternatives_1_0 )
            // InternalLinguaFranca.g:5070:4: rule__Period__PeriodAlternatives_1_0
            {
            pushFollow(FOLLOW_2);
            rule__Period__PeriodAlternatives_1_0();

            state._fsp--;


            }

             after(grammarAccess.getPeriodAccess().getPeriodAlternatives_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__PeriodAssignment_1"


    // $ANTLR start "rule__Period__OffsetAssignment_2_1"
    // InternalLinguaFranca.g:5078:1: rule__Period__OffsetAssignment_2_1 : ( ( rule__Period__OffsetAlternatives_2_1_0 ) ) ;
    public final void rule__Period__OffsetAssignment_2_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:5082:1: ( ( ( rule__Period__OffsetAlternatives_2_1_0 ) ) )
            // InternalLinguaFranca.g:5083:2: ( ( rule__Period__OffsetAlternatives_2_1_0 ) )
            {
            // InternalLinguaFranca.g:5083:2: ( ( rule__Period__OffsetAlternatives_2_1_0 ) )
            // InternalLinguaFranca.g:5084:3: ( rule__Period__OffsetAlternatives_2_1_0 )
            {
             before(grammarAccess.getPeriodAccess().getOffsetAlternatives_2_1_0()); 
            // InternalLinguaFranca.g:5085:3: ( rule__Period__OffsetAlternatives_2_1_0 )
            // InternalLinguaFranca.g:5085:4: rule__Period__OffsetAlternatives_2_1_0
            {
            pushFollow(FOLLOW_2);
            rule__Period__OffsetAlternatives_2_1_0();

            state._fsp--;


            }

             after(grammarAccess.getPeriodAccess().getOffsetAlternatives_2_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__OffsetAssignment_2_1"


    // $ANTLR start "rule__Period__CountAssignment_2_2_1"
    // InternalLinguaFranca.g:5093:1: rule__Period__CountAssignment_2_2_1 : ( ( rule__Period__CountAlternatives_2_2_1_0 ) ) ;
    public final void rule__Period__CountAssignment_2_2_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:5097:1: ( ( ( rule__Period__CountAlternatives_2_2_1_0 ) ) )
            // InternalLinguaFranca.g:5098:2: ( ( rule__Period__CountAlternatives_2_2_1_0 ) )
            {
            // InternalLinguaFranca.g:5098:2: ( ( rule__Period__CountAlternatives_2_2_1_0 ) )
            // InternalLinguaFranca.g:5099:3: ( rule__Period__CountAlternatives_2_2_1_0 )
            {
             before(grammarAccess.getPeriodAccess().getCountAlternatives_2_2_1_0()); 
            // InternalLinguaFranca.g:5100:3: ( rule__Period__CountAlternatives_2_2_1_0 )
            // InternalLinguaFranca.g:5100:4: rule__Period__CountAlternatives_2_2_1_0
            {
            pushFollow(FOLLOW_2);
            rule__Period__CountAlternatives_2_2_1_0();

            state._fsp--;


            }

             after(grammarAccess.getPeriodAccess().getCountAlternatives_2_2_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__CountAssignment_2_2_1"


    // $ANTLR start "rule__Sets__SetsAssignment_1"
    // InternalLinguaFranca.g:5108:1: rule__Sets__SetsAssignment_1 : ( RULE_ID ) ;
    public final void rule__Sets__SetsAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:5112:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:5113:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:5113:2: ( RULE_ID )
            // InternalLinguaFranca.g:5114:3: RULE_ID
            {
             before(grammarAccess.getSetsAccess().getSetsIDTerminalRuleCall_1_0()); 
            match(input,RULE_ID,FOLLOW_2); 
             after(grammarAccess.getSetsAccess().getSetsIDTerminalRuleCall_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Sets__SetsAssignment_1"


    // $ANTLR start "rule__Sets__SetsAssignment_2_1"
    // InternalLinguaFranca.g:5123:1: rule__Sets__SetsAssignment_2_1 : ( RULE_ID ) ;
    public final void rule__Sets__SetsAssignment_2_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:5127:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:5128:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:5128:2: ( RULE_ID )
            // InternalLinguaFranca.g:5129:3: RULE_ID
            {
             before(grammarAccess.getSetsAccess().getSetsIDTerminalRuleCall_2_1_0()); 
            match(input,RULE_ID,FOLLOW_2); 
             after(grammarAccess.getSetsAccess().getSetsIDTerminalRuleCall_2_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Sets__SetsAssignment_2_1"

    // Delegated rules


 

    public static final BitSet FOLLOW_1 = new BitSet(new long[]{0x0000000000000000L});
    public static final BitSet FOLLOW_2 = new BitSet(new long[]{0x0000000000000002L});
    public static final BitSet FOLLOW_3 = new BitSet(new long[]{0x00000000004C0000L});
    public static final BitSet FOLLOW_4 = new BitSet(new long[]{0x0000000000040002L});
    public static final BitSet FOLLOW_5 = new BitSet(new long[]{0x00000000004C0002L});
    public static final BitSet FOLLOW_6 = new BitSet(new long[]{0x0000000000000010L});
    public static final BitSet FOLLOW_7 = new BitSet(new long[]{0x0000000000020000L});
    public static final BitSet FOLLOW_8 = new BitSet(new long[]{0x0000000002100000L});
    public static final BitSet FOLLOW_9 = new BitSet(new long[]{0x000000003120E000L});
    public static final BitSet FOLLOW_10 = new BitSet(new long[]{0x0000000000002002L});
    public static final BitSet FOLLOW_11 = new BitSet(new long[]{0x0000000000004002L});
    public static final BitSet FOLLOW_12 = new BitSet(new long[]{0x0000000000008002L});
    public static final BitSet FOLLOW_13 = new BitSet(new long[]{0x0000000001000002L});
    public static final BitSet FOLLOW_14 = new BitSet(new long[]{0x000000003120E010L});
    public static final BitSet FOLLOW_15 = new BitSet(new long[]{0x0000000000000012L});
    public static final BitSet FOLLOW_16 = new BitSet(new long[]{0x0000000000002010L});
    public static final BitSet FOLLOW_17 = new BitSet(new long[]{0x0000000000820000L});
    public static final BitSet FOLLOW_18 = new BitSet(new long[]{0x0000000000000050L});
    public static final BitSet FOLLOW_19 = new BitSet(new long[]{0x0000000000004010L});
    public static final BitSet FOLLOW_20 = new BitSet(new long[]{0x0000000000008010L});
    public static final BitSet FOLLOW_21 = new BitSet(new long[]{0x0000000002020000L});
    public static final BitSet FOLLOW_22 = new BitSet(new long[]{0x0000000102000050L});
    public static final BitSet FOLLOW_23 = new BitSet(new long[]{0x0000000004000010L});
    public static final BitSet FOLLOW_24 = new BitSet(new long[]{0x0000000008000000L});
    public static final BitSet FOLLOW_25 = new BitSet(new long[]{0x0000000008000002L});
    public static final BitSet FOLLOW_26 = new BitSet(new long[]{0x0000000000000040L});
    public static final BitSet FOLLOW_27 = new BitSet(new long[]{0x0000000040000000L});
    public static final BitSet FOLLOW_28 = new BitSet(new long[]{0x0000000080000000L});
    public static final BitSet FOLLOW_29 = new BitSet(new long[]{0x0000000100000000L});
    public static final BitSet FOLLOW_30 = new BitSet(new long[]{0x00000000000000F0L});
    public static final BitSet FOLLOW_31 = new BitSet(new long[]{0x0000000200000010L});
    public static final BitSet FOLLOW_32 = new BitSet(new long[]{0x000000000C000000L});
    public static final BitSet FOLLOW_33 = new BitSet(new long[]{0x0000000002800000L});
    public static final BitSet FOLLOW_34 = new BitSet(new long[]{0x0000000004000000L});
    public static final BitSet FOLLOW_35 = new BitSet(new long[]{0x0000000000000030L});
    public static final BitSet FOLLOW_36 = new BitSet(new long[]{0x0000000400000000L});
    public static final BitSet FOLLOW_37 = new BitSet(new long[]{0x0000000000006010L});
    public static final BitSet FOLLOW_38 = new BitSet(new long[]{0x0000000400000002L});

}