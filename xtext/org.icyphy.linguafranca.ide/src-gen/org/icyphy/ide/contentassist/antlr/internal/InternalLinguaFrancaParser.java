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
        "<invalid>", "<EOR>", "<DOWN>", "<UP>", "RULE_ID", "RULE_NUMBER", "RULE_CODE", "RULE_STRING", "RULE_INT", "RULE_ML_COMMENT", "RULE_SL_COMMENT", "RULE_WS", "RULE_ANY_OTHER", "'input'", "'output'", "'timer'", "'action'", "'NOW'", "'ONCE'", "'STOP'", "'target'", "';'", "'import'", "'reactor'", "'}'", "'composite'", "'{'", "':'", "'reaction'", "'('", "')'", "','", "'preamble'", "'='", "'new'", "'->'", "'const'", "'.'"
    };
    public static final int RULE_STRING=7;
    public static final int RULE_SL_COMMENT=10;
    public static final int T__19=19;
    public static final int T__15=15;
    public static final int T__37=37;
    public static final int T__16=16;
    public static final int T__17=17;
    public static final int T__18=18;
    public static final int T__33=33;
    public static final int T__34=34;
    public static final int T__13=13;
    public static final int T__35=35;
    public static final int T__14=14;
    public static final int T__36=36;
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


    // $ANTLR start "entryRuleComponent"
    // InternalLinguaFranca.g:128:1: entryRuleComponent : ruleComponent EOF ;
    public final void entryRuleComponent() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:129:1: ( ruleComponent EOF )
            // InternalLinguaFranca.g:130:1: ruleComponent EOF
            {
             before(grammarAccess.getComponentRule()); 
            pushFollow(FOLLOW_1);
            ruleComponent();

            state._fsp--;

             after(grammarAccess.getComponentRule()); 
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
    // $ANTLR end "entryRuleComponent"


    // $ANTLR start "ruleComponent"
    // InternalLinguaFranca.g:137:1: ruleComponent : ( ( rule__Component__Alternatives ) ) ;
    public final void ruleComponent() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:141:2: ( ( ( rule__Component__Alternatives ) ) )
            // InternalLinguaFranca.g:142:2: ( ( rule__Component__Alternatives ) )
            {
            // InternalLinguaFranca.g:142:2: ( ( rule__Component__Alternatives ) )
            // InternalLinguaFranca.g:143:3: ( rule__Component__Alternatives )
            {
             before(grammarAccess.getComponentAccess().getAlternatives()); 
            // InternalLinguaFranca.g:144:3: ( rule__Component__Alternatives )
            // InternalLinguaFranca.g:144:4: rule__Component__Alternatives
            {
            pushFollow(FOLLOW_2);
            rule__Component__Alternatives();

            state._fsp--;


            }

             after(grammarAccess.getComponentAccess().getAlternatives()); 

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
    // $ANTLR end "ruleComponent"


    // $ANTLR start "entryRuleReactor"
    // InternalLinguaFranca.g:153:1: entryRuleReactor : ruleReactor EOF ;
    public final void entryRuleReactor() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:154:1: ( ruleReactor EOF )
            // InternalLinguaFranca.g:155:1: ruleReactor EOF
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
    // InternalLinguaFranca.g:162:1: ruleReactor : ( ( rule__Reactor__Group__0 ) ) ;
    public final void ruleReactor() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:166:2: ( ( ( rule__Reactor__Group__0 ) ) )
            // InternalLinguaFranca.g:167:2: ( ( rule__Reactor__Group__0 ) )
            {
            // InternalLinguaFranca.g:167:2: ( ( rule__Reactor__Group__0 ) )
            // InternalLinguaFranca.g:168:3: ( rule__Reactor__Group__0 )
            {
             before(grammarAccess.getReactorAccess().getGroup()); 
            // InternalLinguaFranca.g:169:3: ( rule__Reactor__Group__0 )
            // InternalLinguaFranca.g:169:4: rule__Reactor__Group__0
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
    // InternalLinguaFranca.g:178:1: entryRuleComposite : ruleComposite EOF ;
    public final void entryRuleComposite() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:179:1: ( ruleComposite EOF )
            // InternalLinguaFranca.g:180:1: ruleComposite EOF
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
    // InternalLinguaFranca.g:187:1: ruleComposite : ( ( rule__Composite__Group__0 ) ) ;
    public final void ruleComposite() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:191:2: ( ( ( rule__Composite__Group__0 ) ) )
            // InternalLinguaFranca.g:192:2: ( ( rule__Composite__Group__0 ) )
            {
            // InternalLinguaFranca.g:192:2: ( ( rule__Composite__Group__0 ) )
            // InternalLinguaFranca.g:193:3: ( rule__Composite__Group__0 )
            {
             before(grammarAccess.getCompositeAccess().getGroup()); 
            // InternalLinguaFranca.g:194:3: ( rule__Composite__Group__0 )
            // InternalLinguaFranca.g:194:4: rule__Composite__Group__0
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


    // $ANTLR start "entryRuleComponentBody"
    // InternalLinguaFranca.g:203:1: entryRuleComponentBody : ruleComponentBody EOF ;
    public final void entryRuleComponentBody() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:204:1: ( ruleComponentBody EOF )
            // InternalLinguaFranca.g:205:1: ruleComponentBody EOF
            {
             before(grammarAccess.getComponentBodyRule()); 
            pushFollow(FOLLOW_1);
            ruleComponentBody();

            state._fsp--;

             after(grammarAccess.getComponentBodyRule()); 
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
    // $ANTLR end "entryRuleComponentBody"


    // $ANTLR start "ruleComponentBody"
    // InternalLinguaFranca.g:212:1: ruleComponentBody : ( ( rule__ComponentBody__Group__0 ) ) ;
    public final void ruleComponentBody() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:216:2: ( ( ( rule__ComponentBody__Group__0 ) ) )
            // InternalLinguaFranca.g:217:2: ( ( rule__ComponentBody__Group__0 ) )
            {
            // InternalLinguaFranca.g:217:2: ( ( rule__ComponentBody__Group__0 ) )
            // InternalLinguaFranca.g:218:3: ( rule__ComponentBody__Group__0 )
            {
             before(grammarAccess.getComponentBodyAccess().getGroup()); 
            // InternalLinguaFranca.g:219:3: ( rule__ComponentBody__Group__0 )
            // InternalLinguaFranca.g:219:4: rule__ComponentBody__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__ComponentBody__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getComponentBodyAccess().getGroup()); 

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
    // $ANTLR end "ruleComponentBody"


    // $ANTLR start "entryRuleInput"
    // InternalLinguaFranca.g:228:1: entryRuleInput : ruleInput EOF ;
    public final void entryRuleInput() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:229:1: ( ruleInput EOF )
            // InternalLinguaFranca.g:230:1: ruleInput EOF
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
    // InternalLinguaFranca.g:237:1: ruleInput : ( ( rule__Input__Group__0 ) ) ;
    public final void ruleInput() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:241:2: ( ( ( rule__Input__Group__0 ) ) )
            // InternalLinguaFranca.g:242:2: ( ( rule__Input__Group__0 ) )
            {
            // InternalLinguaFranca.g:242:2: ( ( rule__Input__Group__0 ) )
            // InternalLinguaFranca.g:243:3: ( rule__Input__Group__0 )
            {
             before(grammarAccess.getInputAccess().getGroup()); 
            // InternalLinguaFranca.g:244:3: ( rule__Input__Group__0 )
            // InternalLinguaFranca.g:244:4: rule__Input__Group__0
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
    // InternalLinguaFranca.g:253:1: entryRuleOutput : ruleOutput EOF ;
    public final void entryRuleOutput() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:254:1: ( ruleOutput EOF )
            // InternalLinguaFranca.g:255:1: ruleOutput EOF
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
    // InternalLinguaFranca.g:262:1: ruleOutput : ( ( rule__Output__Group__0 ) ) ;
    public final void ruleOutput() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:266:2: ( ( ( rule__Output__Group__0 ) ) )
            // InternalLinguaFranca.g:267:2: ( ( rule__Output__Group__0 ) )
            {
            // InternalLinguaFranca.g:267:2: ( ( rule__Output__Group__0 ) )
            // InternalLinguaFranca.g:268:3: ( rule__Output__Group__0 )
            {
             before(grammarAccess.getOutputAccess().getGroup()); 
            // InternalLinguaFranca.g:269:3: ( rule__Output__Group__0 )
            // InternalLinguaFranca.g:269:4: rule__Output__Group__0
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


    // $ANTLR start "entryRuleTimer"
    // InternalLinguaFranca.g:278:1: entryRuleTimer : ruleTimer EOF ;
    public final void entryRuleTimer() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:279:1: ( ruleTimer EOF )
            // InternalLinguaFranca.g:280:1: ruleTimer EOF
            {
             before(grammarAccess.getTimerRule()); 
            pushFollow(FOLLOW_1);
            ruleTimer();

            state._fsp--;

             after(grammarAccess.getTimerRule()); 
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
    // $ANTLR end "entryRuleTimer"


    // $ANTLR start "ruleTimer"
    // InternalLinguaFranca.g:287:1: ruleTimer : ( ( rule__Timer__Group__0 ) ) ;
    public final void ruleTimer() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:291:2: ( ( ( rule__Timer__Group__0 ) ) )
            // InternalLinguaFranca.g:292:2: ( ( rule__Timer__Group__0 ) )
            {
            // InternalLinguaFranca.g:292:2: ( ( rule__Timer__Group__0 ) )
            // InternalLinguaFranca.g:293:3: ( rule__Timer__Group__0 )
            {
             before(grammarAccess.getTimerAccess().getGroup()); 
            // InternalLinguaFranca.g:294:3: ( rule__Timer__Group__0 )
            // InternalLinguaFranca.g:294:4: rule__Timer__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Timer__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getTimerAccess().getGroup()); 

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
    // $ANTLR end "ruleTimer"


    // $ANTLR start "entryRuleAction"
    // InternalLinguaFranca.g:303:1: entryRuleAction : ruleAction EOF ;
    public final void entryRuleAction() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:304:1: ( ruleAction EOF )
            // InternalLinguaFranca.g:305:1: ruleAction EOF
            {
             before(grammarAccess.getActionRule()); 
            pushFollow(FOLLOW_1);
            ruleAction();

            state._fsp--;

             after(grammarAccess.getActionRule()); 
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
    // $ANTLR end "entryRuleAction"


    // $ANTLR start "ruleAction"
    // InternalLinguaFranca.g:312:1: ruleAction : ( ( rule__Action__Group__0 ) ) ;
    public final void ruleAction() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:316:2: ( ( ( rule__Action__Group__0 ) ) )
            // InternalLinguaFranca.g:317:2: ( ( rule__Action__Group__0 ) )
            {
            // InternalLinguaFranca.g:317:2: ( ( rule__Action__Group__0 ) )
            // InternalLinguaFranca.g:318:3: ( rule__Action__Group__0 )
            {
             before(grammarAccess.getActionAccess().getGroup()); 
            // InternalLinguaFranca.g:319:3: ( rule__Action__Group__0 )
            // InternalLinguaFranca.g:319:4: rule__Action__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Action__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getActionAccess().getGroup()); 

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
    // $ANTLR end "ruleAction"


    // $ANTLR start "entryRuleReaction"
    // InternalLinguaFranca.g:328:1: entryRuleReaction : ruleReaction EOF ;
    public final void entryRuleReaction() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:329:1: ( ruleReaction EOF )
            // InternalLinguaFranca.g:330:1: ruleReaction EOF
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
    // InternalLinguaFranca.g:337:1: ruleReaction : ( ( rule__Reaction__Group__0 ) ) ;
    public final void ruleReaction() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:341:2: ( ( ( rule__Reaction__Group__0 ) ) )
            // InternalLinguaFranca.g:342:2: ( ( rule__Reaction__Group__0 ) )
            {
            // InternalLinguaFranca.g:342:2: ( ( rule__Reaction__Group__0 ) )
            // InternalLinguaFranca.g:343:3: ( rule__Reaction__Group__0 )
            {
             before(grammarAccess.getReactionAccess().getGroup()); 
            // InternalLinguaFranca.g:344:3: ( rule__Reaction__Group__0 )
            // InternalLinguaFranca.g:344:4: rule__Reaction__Group__0
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
    // InternalLinguaFranca.g:353:1: entryRulePreamble : rulePreamble EOF ;
    public final void entryRulePreamble() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:354:1: ( rulePreamble EOF )
            // InternalLinguaFranca.g:355:1: rulePreamble EOF
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
    // InternalLinguaFranca.g:362:1: rulePreamble : ( ( rule__Preamble__Group__0 ) ) ;
    public final void rulePreamble() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:366:2: ( ( ( rule__Preamble__Group__0 ) ) )
            // InternalLinguaFranca.g:367:2: ( ( rule__Preamble__Group__0 ) )
            {
            // InternalLinguaFranca.g:367:2: ( ( rule__Preamble__Group__0 ) )
            // InternalLinguaFranca.g:368:3: ( rule__Preamble__Group__0 )
            {
             before(grammarAccess.getPreambleAccess().getGroup()); 
            // InternalLinguaFranca.g:369:3: ( rule__Preamble__Group__0 )
            // InternalLinguaFranca.g:369:4: rule__Preamble__Group__0
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


    // $ANTLR start "entryRuleInstance"
    // InternalLinguaFranca.g:378:1: entryRuleInstance : ruleInstance EOF ;
    public final void entryRuleInstance() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:379:1: ( ruleInstance EOF )
            // InternalLinguaFranca.g:380:1: ruleInstance EOF
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
    // InternalLinguaFranca.g:387:1: ruleInstance : ( ( rule__Instance__Group__0 ) ) ;
    public final void ruleInstance() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:391:2: ( ( ( rule__Instance__Group__0 ) ) )
            // InternalLinguaFranca.g:392:2: ( ( rule__Instance__Group__0 ) )
            {
            // InternalLinguaFranca.g:392:2: ( ( rule__Instance__Group__0 ) )
            // InternalLinguaFranca.g:393:3: ( rule__Instance__Group__0 )
            {
             before(grammarAccess.getInstanceAccess().getGroup()); 
            // InternalLinguaFranca.g:394:3: ( rule__Instance__Group__0 )
            // InternalLinguaFranca.g:394:4: rule__Instance__Group__0
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
    // InternalLinguaFranca.g:403:1: entryRuleConnection : ruleConnection EOF ;
    public final void entryRuleConnection() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:404:1: ( ruleConnection EOF )
            // InternalLinguaFranca.g:405:1: ruleConnection EOF
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
    // InternalLinguaFranca.g:412:1: ruleConnection : ( ( rule__Connection__Group__0 ) ) ;
    public final void ruleConnection() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:416:2: ( ( ( rule__Connection__Group__0 ) ) )
            // InternalLinguaFranca.g:417:2: ( ( rule__Connection__Group__0 ) )
            {
            // InternalLinguaFranca.g:417:2: ( ( rule__Connection__Group__0 ) )
            // InternalLinguaFranca.g:418:3: ( rule__Connection__Group__0 )
            {
             before(grammarAccess.getConnectionAccess().getGroup()); 
            // InternalLinguaFranca.g:419:3: ( rule__Connection__Group__0 )
            // InternalLinguaFranca.g:419:4: rule__Connection__Group__0
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
    // InternalLinguaFranca.g:428:1: entryRuleAssignments : ruleAssignments EOF ;
    public final void entryRuleAssignments() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:429:1: ( ruleAssignments EOF )
            // InternalLinguaFranca.g:430:1: ruleAssignments EOF
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
    // InternalLinguaFranca.g:437:1: ruleAssignments : ( ( rule__Assignments__Group__0 ) ) ;
    public final void ruleAssignments() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:441:2: ( ( ( rule__Assignments__Group__0 ) ) )
            // InternalLinguaFranca.g:442:2: ( ( rule__Assignments__Group__0 ) )
            {
            // InternalLinguaFranca.g:442:2: ( ( rule__Assignments__Group__0 ) )
            // InternalLinguaFranca.g:443:3: ( rule__Assignments__Group__0 )
            {
             before(grammarAccess.getAssignmentsAccess().getGroup()); 
            // InternalLinguaFranca.g:444:3: ( rule__Assignments__Group__0 )
            // InternalLinguaFranca.g:444:4: rule__Assignments__Group__0
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
    // InternalLinguaFranca.g:453:1: entryRuleAssignment : ruleAssignment EOF ;
    public final void entryRuleAssignment() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:454:1: ( ruleAssignment EOF )
            // InternalLinguaFranca.g:455:1: ruleAssignment EOF
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
    // InternalLinguaFranca.g:462:1: ruleAssignment : ( ( rule__Assignment__Group__0 ) ) ;
    public final void ruleAssignment() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:466:2: ( ( ( rule__Assignment__Group__0 ) ) )
            // InternalLinguaFranca.g:467:2: ( ( rule__Assignment__Group__0 ) )
            {
            // InternalLinguaFranca.g:467:2: ( ( rule__Assignment__Group__0 ) )
            // InternalLinguaFranca.g:468:3: ( rule__Assignment__Group__0 )
            {
             before(grammarAccess.getAssignmentAccess().getGroup()); 
            // InternalLinguaFranca.g:469:3: ( rule__Assignment__Group__0 )
            // InternalLinguaFranca.g:469:4: rule__Assignment__Group__0
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
    // InternalLinguaFranca.g:478:1: entryRuleGets : ruleGets EOF ;
    public final void entryRuleGets() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:479:1: ( ruleGets EOF )
            // InternalLinguaFranca.g:480:1: ruleGets EOF
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
    // InternalLinguaFranca.g:487:1: ruleGets : ( ( rule__Gets__Group__0 ) ) ;
    public final void ruleGets() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:491:2: ( ( ( rule__Gets__Group__0 ) ) )
            // InternalLinguaFranca.g:492:2: ( ( rule__Gets__Group__0 ) )
            {
            // InternalLinguaFranca.g:492:2: ( ( rule__Gets__Group__0 ) )
            // InternalLinguaFranca.g:493:3: ( rule__Gets__Group__0 )
            {
             before(grammarAccess.getGetsAccess().getGroup()); 
            // InternalLinguaFranca.g:494:3: ( rule__Gets__Group__0 )
            // InternalLinguaFranca.g:494:4: rule__Gets__Group__0
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
    // InternalLinguaFranca.g:503:1: entryRuleParams : ruleParams EOF ;
    public final void entryRuleParams() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:504:1: ( ruleParams EOF )
            // InternalLinguaFranca.g:505:1: ruleParams EOF
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
    // InternalLinguaFranca.g:512:1: ruleParams : ( ( rule__Params__Group__0 ) ) ;
    public final void ruleParams() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:516:2: ( ( ( rule__Params__Group__0 ) ) )
            // InternalLinguaFranca.g:517:2: ( ( rule__Params__Group__0 ) )
            {
            // InternalLinguaFranca.g:517:2: ( ( rule__Params__Group__0 ) )
            // InternalLinguaFranca.g:518:3: ( rule__Params__Group__0 )
            {
             before(grammarAccess.getParamsAccess().getGroup()); 
            // InternalLinguaFranca.g:519:3: ( rule__Params__Group__0 )
            // InternalLinguaFranca.g:519:4: rule__Params__Group__0
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
    // InternalLinguaFranca.g:528:1: entryRuleParam : ruleParam EOF ;
    public final void entryRuleParam() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:529:1: ( ruleParam EOF )
            // InternalLinguaFranca.g:530:1: ruleParam EOF
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
    // InternalLinguaFranca.g:537:1: ruleParam : ( ( rule__Param__Group__0 ) ) ;
    public final void ruleParam() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:541:2: ( ( ( rule__Param__Group__0 ) ) )
            // InternalLinguaFranca.g:542:2: ( ( rule__Param__Group__0 ) )
            {
            // InternalLinguaFranca.g:542:2: ( ( rule__Param__Group__0 ) )
            // InternalLinguaFranca.g:543:3: ( rule__Param__Group__0 )
            {
             before(grammarAccess.getParamAccess().getGroup()); 
            // InternalLinguaFranca.g:544:3: ( rule__Param__Group__0 )
            // InternalLinguaFranca.g:544:4: rule__Param__Group__0
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


    // $ANTLR start "entryRuleTiming"
    // InternalLinguaFranca.g:553:1: entryRuleTiming : ruleTiming EOF ;
    public final void entryRuleTiming() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:554:1: ( ruleTiming EOF )
            // InternalLinguaFranca.g:555:1: ruleTiming EOF
            {
             before(grammarAccess.getTimingRule()); 
            pushFollow(FOLLOW_1);
            ruleTiming();

            state._fsp--;

             after(grammarAccess.getTimingRule()); 
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
    // $ANTLR end "entryRuleTiming"


    // $ANTLR start "ruleTiming"
    // InternalLinguaFranca.g:562:1: ruleTiming : ( ( rule__Timing__Group__0 ) ) ;
    public final void ruleTiming() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:566:2: ( ( ( rule__Timing__Group__0 ) ) )
            // InternalLinguaFranca.g:567:2: ( ( rule__Timing__Group__0 ) )
            {
            // InternalLinguaFranca.g:567:2: ( ( rule__Timing__Group__0 ) )
            // InternalLinguaFranca.g:568:3: ( rule__Timing__Group__0 )
            {
             before(grammarAccess.getTimingAccess().getGroup()); 
            // InternalLinguaFranca.g:569:3: ( rule__Timing__Group__0 )
            // InternalLinguaFranca.g:569:4: rule__Timing__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Timing__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getTimingAccess().getGroup()); 

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
    // $ANTLR end "ruleTiming"


    // $ANTLR start "entryRulePort"
    // InternalLinguaFranca.g:578:1: entryRulePort : rulePort EOF ;
    public final void entryRulePort() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:579:1: ( rulePort EOF )
            // InternalLinguaFranca.g:580:1: rulePort EOF
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
    // InternalLinguaFranca.g:587:1: rulePort : ( ( rule__Port__Alternatives ) ) ;
    public final void rulePort() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:591:2: ( ( ( rule__Port__Alternatives ) ) )
            // InternalLinguaFranca.g:592:2: ( ( rule__Port__Alternatives ) )
            {
            // InternalLinguaFranca.g:592:2: ( ( rule__Port__Alternatives ) )
            // InternalLinguaFranca.g:593:3: ( rule__Port__Alternatives )
            {
             before(grammarAccess.getPortAccess().getAlternatives()); 
            // InternalLinguaFranca.g:594:3: ( rule__Port__Alternatives )
            // InternalLinguaFranca.g:594:4: rule__Port__Alternatives
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
    // InternalLinguaFranca.g:603:1: entryRuleSets : ruleSets EOF ;
    public final void entryRuleSets() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:604:1: ( ruleSets EOF )
            // InternalLinguaFranca.g:605:1: ruleSets EOF
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
    // InternalLinguaFranca.g:612:1: ruleSets : ( ( rule__Sets__Group__0 ) ) ;
    public final void ruleSets() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:616:2: ( ( ( rule__Sets__Group__0 ) ) )
            // InternalLinguaFranca.g:617:2: ( ( rule__Sets__Group__0 ) )
            {
            // InternalLinguaFranca.g:617:2: ( ( rule__Sets__Group__0 ) )
            // InternalLinguaFranca.g:618:3: ( rule__Sets__Group__0 )
            {
             before(grammarAccess.getSetsAccess().getGroup()); 
            // InternalLinguaFranca.g:619:3: ( rule__Sets__Group__0 )
            // InternalLinguaFranca.g:619:4: rule__Sets__Group__0
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
    // InternalLinguaFranca.g:628:1: entryRuleType : ruleType EOF ;
    public final void entryRuleType() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:629:1: ( ruleType EOF )
            // InternalLinguaFranca.g:630:1: ruleType EOF
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
    // InternalLinguaFranca.g:637:1: ruleType : ( ( rule__Type__Alternatives ) ) ;
    public final void ruleType() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:641:2: ( ( ( rule__Type__Alternatives ) ) )
            // InternalLinguaFranca.g:642:2: ( ( rule__Type__Alternatives ) )
            {
            // InternalLinguaFranca.g:642:2: ( ( rule__Type__Alternatives ) )
            // InternalLinguaFranca.g:643:3: ( rule__Type__Alternatives )
            {
             before(grammarAccess.getTypeAccess().getAlternatives()); 
            // InternalLinguaFranca.g:644:3: ( rule__Type__Alternatives )
            // InternalLinguaFranca.g:644:4: rule__Type__Alternatives
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
    // InternalLinguaFranca.g:653:1: entryRuleValue : ruleValue EOF ;
    public final void entryRuleValue() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:654:1: ( ruleValue EOF )
            // InternalLinguaFranca.g:655:1: ruleValue EOF
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
    // InternalLinguaFranca.g:662:1: ruleValue : ( ( rule__Value__Alternatives ) ) ;
    public final void ruleValue() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:666:2: ( ( ( rule__Value__Alternatives ) ) )
            // InternalLinguaFranca.g:667:2: ( ( rule__Value__Alternatives ) )
            {
            // InternalLinguaFranca.g:667:2: ( ( rule__Value__Alternatives ) )
            // InternalLinguaFranca.g:668:3: ( rule__Value__Alternatives )
            {
             before(grammarAccess.getValueAccess().getAlternatives()); 
            // InternalLinguaFranca.g:669:3: ( rule__Value__Alternatives )
            // InternalLinguaFranca.g:669:4: rule__Value__Alternatives
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
    // InternalLinguaFranca.g:678:1: entryRulePath : rulePath EOF ;
    public final void entryRulePath() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:679:1: ( rulePath EOF )
            // InternalLinguaFranca.g:680:1: rulePath EOF
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
    // InternalLinguaFranca.g:687:1: rulePath : ( ( rule__Path__Group__0 ) ) ;
    public final void rulePath() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:691:2: ( ( ( rule__Path__Group__0 ) ) )
            // InternalLinguaFranca.g:692:2: ( ( rule__Path__Group__0 ) )
            {
            // InternalLinguaFranca.g:692:2: ( ( rule__Path__Group__0 ) )
            // InternalLinguaFranca.g:693:3: ( rule__Path__Group__0 )
            {
             before(grammarAccess.getPathAccess().getGroup()); 
            // InternalLinguaFranca.g:694:3: ( rule__Path__Group__0 )
            // InternalLinguaFranca.g:694:4: rule__Path__Group__0
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


    // $ANTLR start "rule__Component__Alternatives"
    // InternalLinguaFranca.g:702:1: rule__Component__Alternatives : ( ( ruleReactor ) | ( ruleComposite ) );
    public final void rule__Component__Alternatives() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:706:1: ( ( ruleReactor ) | ( ruleComposite ) )
            int alt1=2;
            int LA1_0 = input.LA(1);

            if ( (LA1_0==23) ) {
                alt1=1;
            }
            else if ( (LA1_0==25) ) {
                alt1=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 1, 0, input);

                throw nvae;
            }
            switch (alt1) {
                case 1 :
                    // InternalLinguaFranca.g:707:2: ( ruleReactor )
                    {
                    // InternalLinguaFranca.g:707:2: ( ruleReactor )
                    // InternalLinguaFranca.g:708:3: ruleReactor
                    {
                     before(grammarAccess.getComponentAccess().getReactorParserRuleCall_0()); 
                    pushFollow(FOLLOW_2);
                    ruleReactor();

                    state._fsp--;

                     after(grammarAccess.getComponentAccess().getReactorParserRuleCall_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:713:2: ( ruleComposite )
                    {
                    // InternalLinguaFranca.g:713:2: ( ruleComposite )
                    // InternalLinguaFranca.g:714:3: ruleComposite
                    {
                     before(grammarAccess.getComponentAccess().getCompositeParserRuleCall_1()); 
                    pushFollow(FOLLOW_2);
                    ruleComposite();

                    state._fsp--;

                     after(grammarAccess.getComponentAccess().getCompositeParserRuleCall_1()); 

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
    // $ANTLR end "rule__Component__Alternatives"


    // $ANTLR start "rule__ComponentBody__Alternatives_5"
    // InternalLinguaFranca.g:723:1: rule__ComponentBody__Alternatives_5 : ( ( ( rule__ComponentBody__TimersAssignment_5_0 ) ) | ( ( rule__ComponentBody__ActionsAssignment_5_1 ) ) );
    public final void rule__ComponentBody__Alternatives_5() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:727:1: ( ( ( rule__ComponentBody__TimersAssignment_5_0 ) ) | ( ( rule__ComponentBody__ActionsAssignment_5_1 ) ) )
            int alt2=2;
            int LA2_0 = input.LA(1);

            if ( (LA2_0==15) ) {
                alt2=1;
            }
            else if ( (LA2_0==16) ) {
                alt2=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 2, 0, input);

                throw nvae;
            }
            switch (alt2) {
                case 1 :
                    // InternalLinguaFranca.g:728:2: ( ( rule__ComponentBody__TimersAssignment_5_0 ) )
                    {
                    // InternalLinguaFranca.g:728:2: ( ( rule__ComponentBody__TimersAssignment_5_0 ) )
                    // InternalLinguaFranca.g:729:3: ( rule__ComponentBody__TimersAssignment_5_0 )
                    {
                     before(grammarAccess.getComponentBodyAccess().getTimersAssignment_5_0()); 
                    // InternalLinguaFranca.g:730:3: ( rule__ComponentBody__TimersAssignment_5_0 )
                    // InternalLinguaFranca.g:730:4: rule__ComponentBody__TimersAssignment_5_0
                    {
                    pushFollow(FOLLOW_2);
                    rule__ComponentBody__TimersAssignment_5_0();

                    state._fsp--;


                    }

                     after(grammarAccess.getComponentBodyAccess().getTimersAssignment_5_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:734:2: ( ( rule__ComponentBody__ActionsAssignment_5_1 ) )
                    {
                    // InternalLinguaFranca.g:734:2: ( ( rule__ComponentBody__ActionsAssignment_5_1 ) )
                    // InternalLinguaFranca.g:735:3: ( rule__ComponentBody__ActionsAssignment_5_1 )
                    {
                     before(grammarAccess.getComponentBodyAccess().getActionsAssignment_5_1()); 
                    // InternalLinguaFranca.g:736:3: ( rule__ComponentBody__ActionsAssignment_5_1 )
                    // InternalLinguaFranca.g:736:4: rule__ComponentBody__ActionsAssignment_5_1
                    {
                    pushFollow(FOLLOW_2);
                    rule__ComponentBody__ActionsAssignment_5_1();

                    state._fsp--;


                    }

                     after(grammarAccess.getComponentBodyAccess().getActionsAssignment_5_1()); 

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
    // $ANTLR end "rule__ComponentBody__Alternatives_5"


    // $ANTLR start "rule__Input__NameAlternatives_1_0"
    // InternalLinguaFranca.g:744:1: rule__Input__NameAlternatives_1_0 : ( ( RULE_ID ) | ( 'input' ) );
    public final void rule__Input__NameAlternatives_1_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:748:1: ( ( RULE_ID ) | ( 'input' ) )
            int alt3=2;
            int LA3_0 = input.LA(1);

            if ( (LA3_0==RULE_ID) ) {
                alt3=1;
            }
            else if ( (LA3_0==13) ) {
                alt3=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 3, 0, input);

                throw nvae;
            }
            switch (alt3) {
                case 1 :
                    // InternalLinguaFranca.g:749:2: ( RULE_ID )
                    {
                    // InternalLinguaFranca.g:749:2: ( RULE_ID )
                    // InternalLinguaFranca.g:750:3: RULE_ID
                    {
                     before(grammarAccess.getInputAccess().getNameIDTerminalRuleCall_1_0_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getInputAccess().getNameIDTerminalRuleCall_1_0_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:755:2: ( 'input' )
                    {
                    // InternalLinguaFranca.g:755:2: ( 'input' )
                    // InternalLinguaFranca.g:756:3: 'input'
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
    // InternalLinguaFranca.g:765:1: rule__Output__NameAlternatives_1_0 : ( ( RULE_ID ) | ( 'output' ) );
    public final void rule__Output__NameAlternatives_1_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:769:1: ( ( RULE_ID ) | ( 'output' ) )
            int alt4=2;
            int LA4_0 = input.LA(1);

            if ( (LA4_0==RULE_ID) ) {
                alt4=1;
            }
            else if ( (LA4_0==14) ) {
                alt4=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 4, 0, input);

                throw nvae;
            }
            switch (alt4) {
                case 1 :
                    // InternalLinguaFranca.g:770:2: ( RULE_ID )
                    {
                    // InternalLinguaFranca.g:770:2: ( RULE_ID )
                    // InternalLinguaFranca.g:771:3: RULE_ID
                    {
                     before(grammarAccess.getOutputAccess().getNameIDTerminalRuleCall_1_0_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getOutputAccess().getNameIDTerminalRuleCall_1_0_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:776:2: ( 'output' )
                    {
                    // InternalLinguaFranca.g:776:2: ( 'output' )
                    // InternalLinguaFranca.g:777:3: 'output'
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


    // $ANTLR start "rule__Timer__NameAlternatives_1_0"
    // InternalLinguaFranca.g:786:1: rule__Timer__NameAlternatives_1_0 : ( ( RULE_ID ) | ( 'timer' ) );
    public final void rule__Timer__NameAlternatives_1_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:790:1: ( ( RULE_ID ) | ( 'timer' ) )
            int alt5=2;
            int LA5_0 = input.LA(1);

            if ( (LA5_0==RULE_ID) ) {
                alt5=1;
            }
            else if ( (LA5_0==15) ) {
                alt5=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 5, 0, input);

                throw nvae;
            }
            switch (alt5) {
                case 1 :
                    // InternalLinguaFranca.g:791:2: ( RULE_ID )
                    {
                    // InternalLinguaFranca.g:791:2: ( RULE_ID )
                    // InternalLinguaFranca.g:792:3: RULE_ID
                    {
                     before(grammarAccess.getTimerAccess().getNameIDTerminalRuleCall_1_0_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getTimerAccess().getNameIDTerminalRuleCall_1_0_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:797:2: ( 'timer' )
                    {
                    // InternalLinguaFranca.g:797:2: ( 'timer' )
                    // InternalLinguaFranca.g:798:3: 'timer'
                    {
                     before(grammarAccess.getTimerAccess().getNameTimerKeyword_1_0_1()); 
                    match(input,15,FOLLOW_2); 
                     after(grammarAccess.getTimerAccess().getNameTimerKeyword_1_0_1()); 

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
    // $ANTLR end "rule__Timer__NameAlternatives_1_0"


    // $ANTLR start "rule__Action__NameAlternatives_1_0"
    // InternalLinguaFranca.g:807:1: rule__Action__NameAlternatives_1_0 : ( ( RULE_ID ) | ( 'action' ) );
    public final void rule__Action__NameAlternatives_1_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:811:1: ( ( RULE_ID ) | ( 'action' ) )
            int alt6=2;
            int LA6_0 = input.LA(1);

            if ( (LA6_0==RULE_ID) ) {
                alt6=1;
            }
            else if ( (LA6_0==16) ) {
                alt6=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 6, 0, input);

                throw nvae;
            }
            switch (alt6) {
                case 1 :
                    // InternalLinguaFranca.g:812:2: ( RULE_ID )
                    {
                    // InternalLinguaFranca.g:812:2: ( RULE_ID )
                    // InternalLinguaFranca.g:813:3: RULE_ID
                    {
                     before(grammarAccess.getActionAccess().getNameIDTerminalRuleCall_1_0_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getActionAccess().getNameIDTerminalRuleCall_1_0_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:818:2: ( 'action' )
                    {
                    // InternalLinguaFranca.g:818:2: ( 'action' )
                    // InternalLinguaFranca.g:819:3: 'action'
                    {
                     before(grammarAccess.getActionAccess().getNameActionKeyword_1_0_1()); 
                    match(input,16,FOLLOW_2); 
                     after(grammarAccess.getActionAccess().getNameActionKeyword_1_0_1()); 

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
    // $ANTLR end "rule__Action__NameAlternatives_1_0"


    // $ANTLR start "rule__Timing__OffsetAlternatives_1_0"
    // InternalLinguaFranca.g:828:1: rule__Timing__OffsetAlternatives_1_0 : ( ( 'NOW' ) | ( RULE_ID ) | ( RULE_NUMBER ) );
    public final void rule__Timing__OffsetAlternatives_1_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:832:1: ( ( 'NOW' ) | ( RULE_ID ) | ( RULE_NUMBER ) )
            int alt7=3;
            switch ( input.LA(1) ) {
            case 17:
                {
                alt7=1;
                }
                break;
            case RULE_ID:
                {
                alt7=2;
                }
                break;
            case RULE_NUMBER:
                {
                alt7=3;
                }
                break;
            default:
                NoViableAltException nvae =
                    new NoViableAltException("", 7, 0, input);

                throw nvae;
            }

            switch (alt7) {
                case 1 :
                    // InternalLinguaFranca.g:833:2: ( 'NOW' )
                    {
                    // InternalLinguaFranca.g:833:2: ( 'NOW' )
                    // InternalLinguaFranca.g:834:3: 'NOW'
                    {
                     before(grammarAccess.getTimingAccess().getOffsetNOWKeyword_1_0_0()); 
                    match(input,17,FOLLOW_2); 
                     after(grammarAccess.getTimingAccess().getOffsetNOWKeyword_1_0_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:839:2: ( RULE_ID )
                    {
                    // InternalLinguaFranca.g:839:2: ( RULE_ID )
                    // InternalLinguaFranca.g:840:3: RULE_ID
                    {
                     before(grammarAccess.getTimingAccess().getOffsetIDTerminalRuleCall_1_0_1()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getTimingAccess().getOffsetIDTerminalRuleCall_1_0_1()); 

                    }


                    }
                    break;
                case 3 :
                    // InternalLinguaFranca.g:845:2: ( RULE_NUMBER )
                    {
                    // InternalLinguaFranca.g:845:2: ( RULE_NUMBER )
                    // InternalLinguaFranca.g:846:3: RULE_NUMBER
                    {
                     before(grammarAccess.getTimingAccess().getOffsetNUMBERTerminalRuleCall_1_0_2()); 
                    match(input,RULE_NUMBER,FOLLOW_2); 
                     after(grammarAccess.getTimingAccess().getOffsetNUMBERTerminalRuleCall_1_0_2()); 

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
    // $ANTLR end "rule__Timing__OffsetAlternatives_1_0"


    // $ANTLR start "rule__Timing__PeriodAlternatives_2_1_0"
    // InternalLinguaFranca.g:855:1: rule__Timing__PeriodAlternatives_2_1_0 : ( ( 'ONCE' ) | ( 'STOP' ) | ( RULE_ID ) | ( RULE_NUMBER ) );
    public final void rule__Timing__PeriodAlternatives_2_1_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:859:1: ( ( 'ONCE' ) | ( 'STOP' ) | ( RULE_ID ) | ( RULE_NUMBER ) )
            int alt8=4;
            switch ( input.LA(1) ) {
            case 18:
                {
                alt8=1;
                }
                break;
            case 19:
                {
                alt8=2;
                }
                break;
            case RULE_ID:
                {
                alt8=3;
                }
                break;
            case RULE_NUMBER:
                {
                alt8=4;
                }
                break;
            default:
                NoViableAltException nvae =
                    new NoViableAltException("", 8, 0, input);

                throw nvae;
            }

            switch (alt8) {
                case 1 :
                    // InternalLinguaFranca.g:860:2: ( 'ONCE' )
                    {
                    // InternalLinguaFranca.g:860:2: ( 'ONCE' )
                    // InternalLinguaFranca.g:861:3: 'ONCE'
                    {
                     before(grammarAccess.getTimingAccess().getPeriodONCEKeyword_2_1_0_0()); 
                    match(input,18,FOLLOW_2); 
                     after(grammarAccess.getTimingAccess().getPeriodONCEKeyword_2_1_0_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:866:2: ( 'STOP' )
                    {
                    // InternalLinguaFranca.g:866:2: ( 'STOP' )
                    // InternalLinguaFranca.g:867:3: 'STOP'
                    {
                     before(grammarAccess.getTimingAccess().getPeriodSTOPKeyword_2_1_0_1()); 
                    match(input,19,FOLLOW_2); 
                     after(grammarAccess.getTimingAccess().getPeriodSTOPKeyword_2_1_0_1()); 

                    }


                    }
                    break;
                case 3 :
                    // InternalLinguaFranca.g:872:2: ( RULE_ID )
                    {
                    // InternalLinguaFranca.g:872:2: ( RULE_ID )
                    // InternalLinguaFranca.g:873:3: RULE_ID
                    {
                     before(grammarAccess.getTimingAccess().getPeriodIDTerminalRuleCall_2_1_0_2()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getTimingAccess().getPeriodIDTerminalRuleCall_2_1_0_2()); 

                    }


                    }
                    break;
                case 4 :
                    // InternalLinguaFranca.g:878:2: ( RULE_NUMBER )
                    {
                    // InternalLinguaFranca.g:878:2: ( RULE_NUMBER )
                    // InternalLinguaFranca.g:879:3: RULE_NUMBER
                    {
                     before(grammarAccess.getTimingAccess().getPeriodNUMBERTerminalRuleCall_2_1_0_3()); 
                    match(input,RULE_NUMBER,FOLLOW_2); 
                     after(grammarAccess.getTimingAccess().getPeriodNUMBERTerminalRuleCall_2_1_0_3()); 

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
    // $ANTLR end "rule__Timing__PeriodAlternatives_2_1_0"


    // $ANTLR start "rule__Port__Alternatives"
    // InternalLinguaFranca.g:888:1: rule__Port__Alternatives : ( ( RULE_ID ) | ( ( rule__Port__Group_1__0 ) ) );
    public final void rule__Port__Alternatives() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:892:1: ( ( RULE_ID ) | ( ( rule__Port__Group_1__0 ) ) )
            int alt9=2;
            int LA9_0 = input.LA(1);

            if ( (LA9_0==RULE_ID) ) {
                int LA9_1 = input.LA(2);

                if ( (LA9_1==EOF||LA9_1==21||LA9_1==35) ) {
                    alt9=1;
                }
                else if ( (LA9_1==37) ) {
                    alt9=2;
                }
                else {
                    NoViableAltException nvae =
                        new NoViableAltException("", 9, 1, input);

                    throw nvae;
                }
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 9, 0, input);

                throw nvae;
            }
            switch (alt9) {
                case 1 :
                    // InternalLinguaFranca.g:893:2: ( RULE_ID )
                    {
                    // InternalLinguaFranca.g:893:2: ( RULE_ID )
                    // InternalLinguaFranca.g:894:3: RULE_ID
                    {
                     before(grammarAccess.getPortAccess().getIDTerminalRuleCall_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getPortAccess().getIDTerminalRuleCall_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:899:2: ( ( rule__Port__Group_1__0 ) )
                    {
                    // InternalLinguaFranca.g:899:2: ( ( rule__Port__Group_1__0 ) )
                    // InternalLinguaFranca.g:900:3: ( rule__Port__Group_1__0 )
                    {
                     before(grammarAccess.getPortAccess().getGroup_1()); 
                    // InternalLinguaFranca.g:901:3: ( rule__Port__Group_1__0 )
                    // InternalLinguaFranca.g:901:4: rule__Port__Group_1__0
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
    // InternalLinguaFranca.g:909:1: rule__Port__Alternatives_1_2 : ( ( RULE_ID ) | ( 'input' ) | ( 'output' ) );
    public final void rule__Port__Alternatives_1_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:913:1: ( ( RULE_ID ) | ( 'input' ) | ( 'output' ) )
            int alt10=3;
            switch ( input.LA(1) ) {
            case RULE_ID:
                {
                alt10=1;
                }
                break;
            case 13:
                {
                alt10=2;
                }
                break;
            case 14:
                {
                alt10=3;
                }
                break;
            default:
                NoViableAltException nvae =
                    new NoViableAltException("", 10, 0, input);

                throw nvae;
            }

            switch (alt10) {
                case 1 :
                    // InternalLinguaFranca.g:914:2: ( RULE_ID )
                    {
                    // InternalLinguaFranca.g:914:2: ( RULE_ID )
                    // InternalLinguaFranca.g:915:3: RULE_ID
                    {
                     before(grammarAccess.getPortAccess().getIDTerminalRuleCall_1_2_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getPortAccess().getIDTerminalRuleCall_1_2_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:920:2: ( 'input' )
                    {
                    // InternalLinguaFranca.g:920:2: ( 'input' )
                    // InternalLinguaFranca.g:921:3: 'input'
                    {
                     before(grammarAccess.getPortAccess().getInputKeyword_1_2_1()); 
                    match(input,13,FOLLOW_2); 
                     after(grammarAccess.getPortAccess().getInputKeyword_1_2_1()); 

                    }


                    }
                    break;
                case 3 :
                    // InternalLinguaFranca.g:926:2: ( 'output' )
                    {
                    // InternalLinguaFranca.g:926:2: ( 'output' )
                    // InternalLinguaFranca.g:927:3: 'output'
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
    // InternalLinguaFranca.g:936:1: rule__Type__Alternatives : ( ( RULE_ID ) | ( RULE_CODE ) );
    public final void rule__Type__Alternatives() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:940:1: ( ( RULE_ID ) | ( RULE_CODE ) )
            int alt11=2;
            int LA11_0 = input.LA(1);

            if ( (LA11_0==RULE_ID) ) {
                alt11=1;
            }
            else if ( (LA11_0==RULE_CODE) ) {
                alt11=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 11, 0, input);

                throw nvae;
            }
            switch (alt11) {
                case 1 :
                    // InternalLinguaFranca.g:941:2: ( RULE_ID )
                    {
                    // InternalLinguaFranca.g:941:2: ( RULE_ID )
                    // InternalLinguaFranca.g:942:3: RULE_ID
                    {
                     before(grammarAccess.getTypeAccess().getIDTerminalRuleCall_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getTypeAccess().getIDTerminalRuleCall_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:947:2: ( RULE_CODE )
                    {
                    // InternalLinguaFranca.g:947:2: ( RULE_CODE )
                    // InternalLinguaFranca.g:948:3: RULE_CODE
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
    // InternalLinguaFranca.g:957:1: rule__Value__Alternatives : ( ( RULE_ID ) | ( RULE_NUMBER ) | ( RULE_STRING ) | ( RULE_CODE ) );
    public final void rule__Value__Alternatives() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:961:1: ( ( RULE_ID ) | ( RULE_NUMBER ) | ( RULE_STRING ) | ( RULE_CODE ) )
            int alt12=4;
            switch ( input.LA(1) ) {
            case RULE_ID:
                {
                alt12=1;
                }
                break;
            case RULE_NUMBER:
                {
                alt12=2;
                }
                break;
            case RULE_STRING:
                {
                alt12=3;
                }
                break;
            case RULE_CODE:
                {
                alt12=4;
                }
                break;
            default:
                NoViableAltException nvae =
                    new NoViableAltException("", 12, 0, input);

                throw nvae;
            }

            switch (alt12) {
                case 1 :
                    // InternalLinguaFranca.g:962:2: ( RULE_ID )
                    {
                    // InternalLinguaFranca.g:962:2: ( RULE_ID )
                    // InternalLinguaFranca.g:963:3: RULE_ID
                    {
                     before(grammarAccess.getValueAccess().getIDTerminalRuleCall_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getValueAccess().getIDTerminalRuleCall_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:968:2: ( RULE_NUMBER )
                    {
                    // InternalLinguaFranca.g:968:2: ( RULE_NUMBER )
                    // InternalLinguaFranca.g:969:3: RULE_NUMBER
                    {
                     before(grammarAccess.getValueAccess().getNUMBERTerminalRuleCall_1()); 
                    match(input,RULE_NUMBER,FOLLOW_2); 
                     after(grammarAccess.getValueAccess().getNUMBERTerminalRuleCall_1()); 

                    }


                    }
                    break;
                case 3 :
                    // InternalLinguaFranca.g:974:2: ( RULE_STRING )
                    {
                    // InternalLinguaFranca.g:974:2: ( RULE_STRING )
                    // InternalLinguaFranca.g:975:3: RULE_STRING
                    {
                     before(grammarAccess.getValueAccess().getSTRINGTerminalRuleCall_2()); 
                    match(input,RULE_STRING,FOLLOW_2); 
                     after(grammarAccess.getValueAccess().getSTRINGTerminalRuleCall_2()); 

                    }


                    }
                    break;
                case 4 :
                    // InternalLinguaFranca.g:980:2: ( RULE_CODE )
                    {
                    // InternalLinguaFranca.g:980:2: ( RULE_CODE )
                    // InternalLinguaFranca.g:981:3: RULE_CODE
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
    // InternalLinguaFranca.g:990:1: rule__Model__Group__0 : rule__Model__Group__0__Impl rule__Model__Group__1 ;
    public final void rule__Model__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:994:1: ( rule__Model__Group__0__Impl rule__Model__Group__1 )
            // InternalLinguaFranca.g:995:2: rule__Model__Group__0__Impl rule__Model__Group__1
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
    // InternalLinguaFranca.g:1002:1: rule__Model__Group__0__Impl : ( ( rule__Model__TargetAssignment_0 ) ) ;
    public final void rule__Model__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1006:1: ( ( ( rule__Model__TargetAssignment_0 ) ) )
            // InternalLinguaFranca.g:1007:1: ( ( rule__Model__TargetAssignment_0 ) )
            {
            // InternalLinguaFranca.g:1007:1: ( ( rule__Model__TargetAssignment_0 ) )
            // InternalLinguaFranca.g:1008:2: ( rule__Model__TargetAssignment_0 )
            {
             before(grammarAccess.getModelAccess().getTargetAssignment_0()); 
            // InternalLinguaFranca.g:1009:2: ( rule__Model__TargetAssignment_0 )
            // InternalLinguaFranca.g:1009:3: rule__Model__TargetAssignment_0
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
    // InternalLinguaFranca.g:1017:1: rule__Model__Group__1 : rule__Model__Group__1__Impl rule__Model__Group__2 ;
    public final void rule__Model__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1021:1: ( rule__Model__Group__1__Impl rule__Model__Group__2 )
            // InternalLinguaFranca.g:1022:2: rule__Model__Group__1__Impl rule__Model__Group__2
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
    // InternalLinguaFranca.g:1029:1: rule__Model__Group__1__Impl : ( ( rule__Model__ImportsAssignment_1 )* ) ;
    public final void rule__Model__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1033:1: ( ( ( rule__Model__ImportsAssignment_1 )* ) )
            // InternalLinguaFranca.g:1034:1: ( ( rule__Model__ImportsAssignment_1 )* )
            {
            // InternalLinguaFranca.g:1034:1: ( ( rule__Model__ImportsAssignment_1 )* )
            // InternalLinguaFranca.g:1035:2: ( rule__Model__ImportsAssignment_1 )*
            {
             before(grammarAccess.getModelAccess().getImportsAssignment_1()); 
            // InternalLinguaFranca.g:1036:2: ( rule__Model__ImportsAssignment_1 )*
            loop13:
            do {
                int alt13=2;
                int LA13_0 = input.LA(1);

                if ( (LA13_0==22) ) {
                    alt13=1;
                }


                switch (alt13) {
            	case 1 :
            	    // InternalLinguaFranca.g:1036:3: rule__Model__ImportsAssignment_1
            	    {
            	    pushFollow(FOLLOW_4);
            	    rule__Model__ImportsAssignment_1();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop13;
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
    // InternalLinguaFranca.g:1044:1: rule__Model__Group__2 : rule__Model__Group__2__Impl ;
    public final void rule__Model__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1048:1: ( rule__Model__Group__2__Impl )
            // InternalLinguaFranca.g:1049:2: rule__Model__Group__2__Impl
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
    // InternalLinguaFranca.g:1055:1: rule__Model__Group__2__Impl : ( ( ( rule__Model__ComponentsAssignment_2 ) ) ( ( rule__Model__ComponentsAssignment_2 )* ) ) ;
    public final void rule__Model__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1059:1: ( ( ( ( rule__Model__ComponentsAssignment_2 ) ) ( ( rule__Model__ComponentsAssignment_2 )* ) ) )
            // InternalLinguaFranca.g:1060:1: ( ( ( rule__Model__ComponentsAssignment_2 ) ) ( ( rule__Model__ComponentsAssignment_2 )* ) )
            {
            // InternalLinguaFranca.g:1060:1: ( ( ( rule__Model__ComponentsAssignment_2 ) ) ( ( rule__Model__ComponentsAssignment_2 )* ) )
            // InternalLinguaFranca.g:1061:2: ( ( rule__Model__ComponentsAssignment_2 ) ) ( ( rule__Model__ComponentsAssignment_2 )* )
            {
            // InternalLinguaFranca.g:1061:2: ( ( rule__Model__ComponentsAssignment_2 ) )
            // InternalLinguaFranca.g:1062:3: ( rule__Model__ComponentsAssignment_2 )
            {
             before(grammarAccess.getModelAccess().getComponentsAssignment_2()); 
            // InternalLinguaFranca.g:1063:3: ( rule__Model__ComponentsAssignment_2 )
            // InternalLinguaFranca.g:1063:4: rule__Model__ComponentsAssignment_2
            {
            pushFollow(FOLLOW_5);
            rule__Model__ComponentsAssignment_2();

            state._fsp--;


            }

             after(grammarAccess.getModelAccess().getComponentsAssignment_2()); 

            }

            // InternalLinguaFranca.g:1066:2: ( ( rule__Model__ComponentsAssignment_2 )* )
            // InternalLinguaFranca.g:1067:3: ( rule__Model__ComponentsAssignment_2 )*
            {
             before(grammarAccess.getModelAccess().getComponentsAssignment_2()); 
            // InternalLinguaFranca.g:1068:3: ( rule__Model__ComponentsAssignment_2 )*
            loop14:
            do {
                int alt14=2;
                int LA14_0 = input.LA(1);

                if ( (LA14_0==23||LA14_0==25) ) {
                    alt14=1;
                }


                switch (alt14) {
            	case 1 :
            	    // InternalLinguaFranca.g:1068:4: rule__Model__ComponentsAssignment_2
            	    {
            	    pushFollow(FOLLOW_5);
            	    rule__Model__ComponentsAssignment_2();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop14;
                }
            } while (true);

             after(grammarAccess.getModelAccess().getComponentsAssignment_2()); 

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
    // InternalLinguaFranca.g:1078:1: rule__Target__Group__0 : rule__Target__Group__0__Impl rule__Target__Group__1 ;
    public final void rule__Target__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1082:1: ( rule__Target__Group__0__Impl rule__Target__Group__1 )
            // InternalLinguaFranca.g:1083:2: rule__Target__Group__0__Impl rule__Target__Group__1
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
    // InternalLinguaFranca.g:1090:1: rule__Target__Group__0__Impl : ( 'target' ) ;
    public final void rule__Target__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1094:1: ( ( 'target' ) )
            // InternalLinguaFranca.g:1095:1: ( 'target' )
            {
            // InternalLinguaFranca.g:1095:1: ( 'target' )
            // InternalLinguaFranca.g:1096:2: 'target'
            {
             before(grammarAccess.getTargetAccess().getTargetKeyword_0()); 
            match(input,20,FOLLOW_2); 
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
    // InternalLinguaFranca.g:1105:1: rule__Target__Group__1 : rule__Target__Group__1__Impl rule__Target__Group__2 ;
    public final void rule__Target__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1109:1: ( rule__Target__Group__1__Impl rule__Target__Group__2 )
            // InternalLinguaFranca.g:1110:2: rule__Target__Group__1__Impl rule__Target__Group__2
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
    // InternalLinguaFranca.g:1117:1: rule__Target__Group__1__Impl : ( ( rule__Target__NameAssignment_1 ) ) ;
    public final void rule__Target__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1121:1: ( ( ( rule__Target__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:1122:1: ( ( rule__Target__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:1122:1: ( ( rule__Target__NameAssignment_1 ) )
            // InternalLinguaFranca.g:1123:2: ( rule__Target__NameAssignment_1 )
            {
             before(grammarAccess.getTargetAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:1124:2: ( rule__Target__NameAssignment_1 )
            // InternalLinguaFranca.g:1124:3: rule__Target__NameAssignment_1
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
    // InternalLinguaFranca.g:1132:1: rule__Target__Group__2 : rule__Target__Group__2__Impl ;
    public final void rule__Target__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1136:1: ( rule__Target__Group__2__Impl )
            // InternalLinguaFranca.g:1137:2: rule__Target__Group__2__Impl
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
    // InternalLinguaFranca.g:1143:1: rule__Target__Group__2__Impl : ( ';' ) ;
    public final void rule__Target__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1147:1: ( ( ';' ) )
            // InternalLinguaFranca.g:1148:1: ( ';' )
            {
            // InternalLinguaFranca.g:1148:1: ( ';' )
            // InternalLinguaFranca.g:1149:2: ';'
            {
             before(grammarAccess.getTargetAccess().getSemicolonKeyword_2()); 
            match(input,21,FOLLOW_2); 
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
    // InternalLinguaFranca.g:1159:1: rule__Import__Group__0 : rule__Import__Group__0__Impl rule__Import__Group__1 ;
    public final void rule__Import__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1163:1: ( rule__Import__Group__0__Impl rule__Import__Group__1 )
            // InternalLinguaFranca.g:1164:2: rule__Import__Group__0__Impl rule__Import__Group__1
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
    // InternalLinguaFranca.g:1171:1: rule__Import__Group__0__Impl : ( 'import' ) ;
    public final void rule__Import__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1175:1: ( ( 'import' ) )
            // InternalLinguaFranca.g:1176:1: ( 'import' )
            {
            // InternalLinguaFranca.g:1176:1: ( 'import' )
            // InternalLinguaFranca.g:1177:2: 'import'
            {
             before(grammarAccess.getImportAccess().getImportKeyword_0()); 
            match(input,22,FOLLOW_2); 
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
    // InternalLinguaFranca.g:1186:1: rule__Import__Group__1 : rule__Import__Group__1__Impl rule__Import__Group__2 ;
    public final void rule__Import__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1190:1: ( rule__Import__Group__1__Impl rule__Import__Group__2 )
            // InternalLinguaFranca.g:1191:2: rule__Import__Group__1__Impl rule__Import__Group__2
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
    // InternalLinguaFranca.g:1198:1: rule__Import__Group__1__Impl : ( ( rule__Import__NameAssignment_1 ) ) ;
    public final void rule__Import__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1202:1: ( ( ( rule__Import__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:1203:1: ( ( rule__Import__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:1203:1: ( ( rule__Import__NameAssignment_1 ) )
            // InternalLinguaFranca.g:1204:2: ( rule__Import__NameAssignment_1 )
            {
             before(grammarAccess.getImportAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:1205:2: ( rule__Import__NameAssignment_1 )
            // InternalLinguaFranca.g:1205:3: rule__Import__NameAssignment_1
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
    // InternalLinguaFranca.g:1213:1: rule__Import__Group__2 : rule__Import__Group__2__Impl ;
    public final void rule__Import__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1217:1: ( rule__Import__Group__2__Impl )
            // InternalLinguaFranca.g:1218:2: rule__Import__Group__2__Impl
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
    // InternalLinguaFranca.g:1224:1: rule__Import__Group__2__Impl : ( ';' ) ;
    public final void rule__Import__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1228:1: ( ( ';' ) )
            // InternalLinguaFranca.g:1229:1: ( ';' )
            {
            // InternalLinguaFranca.g:1229:1: ( ';' )
            // InternalLinguaFranca.g:1230:2: ';'
            {
             before(grammarAccess.getImportAccess().getSemicolonKeyword_2()); 
            match(input,21,FOLLOW_2); 
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
    // InternalLinguaFranca.g:1240:1: rule__Reactor__Group__0 : rule__Reactor__Group__0__Impl rule__Reactor__Group__1 ;
    public final void rule__Reactor__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1244:1: ( rule__Reactor__Group__0__Impl rule__Reactor__Group__1 )
            // InternalLinguaFranca.g:1245:2: rule__Reactor__Group__0__Impl rule__Reactor__Group__1
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
    // InternalLinguaFranca.g:1252:1: rule__Reactor__Group__0__Impl : ( 'reactor' ) ;
    public final void rule__Reactor__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1256:1: ( ( 'reactor' ) )
            // InternalLinguaFranca.g:1257:1: ( 'reactor' )
            {
            // InternalLinguaFranca.g:1257:1: ( 'reactor' )
            // InternalLinguaFranca.g:1258:2: 'reactor'
            {
             before(grammarAccess.getReactorAccess().getReactorKeyword_0()); 
            match(input,23,FOLLOW_2); 
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
    // InternalLinguaFranca.g:1267:1: rule__Reactor__Group__1 : rule__Reactor__Group__1__Impl rule__Reactor__Group__2 ;
    public final void rule__Reactor__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1271:1: ( rule__Reactor__Group__1__Impl rule__Reactor__Group__2 )
            // InternalLinguaFranca.g:1272:2: rule__Reactor__Group__1__Impl rule__Reactor__Group__2
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
    // InternalLinguaFranca.g:1279:1: rule__Reactor__Group__1__Impl : ( ( rule__Reactor__ComponentBodyAssignment_1 ) ) ;
    public final void rule__Reactor__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1283:1: ( ( ( rule__Reactor__ComponentBodyAssignment_1 ) ) )
            // InternalLinguaFranca.g:1284:1: ( ( rule__Reactor__ComponentBodyAssignment_1 ) )
            {
            // InternalLinguaFranca.g:1284:1: ( ( rule__Reactor__ComponentBodyAssignment_1 ) )
            // InternalLinguaFranca.g:1285:2: ( rule__Reactor__ComponentBodyAssignment_1 )
            {
             before(grammarAccess.getReactorAccess().getComponentBodyAssignment_1()); 
            // InternalLinguaFranca.g:1286:2: ( rule__Reactor__ComponentBodyAssignment_1 )
            // InternalLinguaFranca.g:1286:3: rule__Reactor__ComponentBodyAssignment_1
            {
            pushFollow(FOLLOW_2);
            rule__Reactor__ComponentBodyAssignment_1();

            state._fsp--;


            }

             after(grammarAccess.getReactorAccess().getComponentBodyAssignment_1()); 

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
    // InternalLinguaFranca.g:1294:1: rule__Reactor__Group__2 : rule__Reactor__Group__2__Impl ;
    public final void rule__Reactor__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1298:1: ( rule__Reactor__Group__2__Impl )
            // InternalLinguaFranca.g:1299:2: rule__Reactor__Group__2__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Reactor__Group__2__Impl();

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
    // InternalLinguaFranca.g:1305:1: rule__Reactor__Group__2__Impl : ( '}' ) ;
    public final void rule__Reactor__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1309:1: ( ( '}' ) )
            // InternalLinguaFranca.g:1310:1: ( '}' )
            {
            // InternalLinguaFranca.g:1310:1: ( '}' )
            // InternalLinguaFranca.g:1311:2: '}'
            {
             before(grammarAccess.getReactorAccess().getRightCurlyBracketKeyword_2()); 
            match(input,24,FOLLOW_2); 
             after(grammarAccess.getReactorAccess().getRightCurlyBracketKeyword_2()); 

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


    // $ANTLR start "rule__Composite__Group__0"
    // InternalLinguaFranca.g:1321:1: rule__Composite__Group__0 : rule__Composite__Group__0__Impl rule__Composite__Group__1 ;
    public final void rule__Composite__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1325:1: ( rule__Composite__Group__0__Impl rule__Composite__Group__1 )
            // InternalLinguaFranca.g:1326:2: rule__Composite__Group__0__Impl rule__Composite__Group__1
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
    // InternalLinguaFranca.g:1333:1: rule__Composite__Group__0__Impl : ( 'composite' ) ;
    public final void rule__Composite__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1337:1: ( ( 'composite' ) )
            // InternalLinguaFranca.g:1338:1: ( 'composite' )
            {
            // InternalLinguaFranca.g:1338:1: ( 'composite' )
            // InternalLinguaFranca.g:1339:2: 'composite'
            {
             before(grammarAccess.getCompositeAccess().getCompositeKeyword_0()); 
            match(input,25,FOLLOW_2); 
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
    // InternalLinguaFranca.g:1348:1: rule__Composite__Group__1 : rule__Composite__Group__1__Impl rule__Composite__Group__2 ;
    public final void rule__Composite__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1352:1: ( rule__Composite__Group__1__Impl rule__Composite__Group__2 )
            // InternalLinguaFranca.g:1353:2: rule__Composite__Group__1__Impl rule__Composite__Group__2
            {
            pushFollow(FOLLOW_9);
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
    // InternalLinguaFranca.g:1360:1: rule__Composite__Group__1__Impl : ( ( rule__Composite__ComponentBodyAssignment_1 ) ) ;
    public final void rule__Composite__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1364:1: ( ( ( rule__Composite__ComponentBodyAssignment_1 ) ) )
            // InternalLinguaFranca.g:1365:1: ( ( rule__Composite__ComponentBodyAssignment_1 ) )
            {
            // InternalLinguaFranca.g:1365:1: ( ( rule__Composite__ComponentBodyAssignment_1 ) )
            // InternalLinguaFranca.g:1366:2: ( rule__Composite__ComponentBodyAssignment_1 )
            {
             before(grammarAccess.getCompositeAccess().getComponentBodyAssignment_1()); 
            // InternalLinguaFranca.g:1367:2: ( rule__Composite__ComponentBodyAssignment_1 )
            // InternalLinguaFranca.g:1367:3: rule__Composite__ComponentBodyAssignment_1
            {
            pushFollow(FOLLOW_2);
            rule__Composite__ComponentBodyAssignment_1();

            state._fsp--;


            }

             after(grammarAccess.getCompositeAccess().getComponentBodyAssignment_1()); 

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
    // InternalLinguaFranca.g:1375:1: rule__Composite__Group__2 : rule__Composite__Group__2__Impl rule__Composite__Group__3 ;
    public final void rule__Composite__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1379:1: ( rule__Composite__Group__2__Impl rule__Composite__Group__3 )
            // InternalLinguaFranca.g:1380:2: rule__Composite__Group__2__Impl rule__Composite__Group__3
            {
            pushFollow(FOLLOW_9);
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
    // InternalLinguaFranca.g:1387:1: rule__Composite__Group__2__Impl : ( ( rule__Composite__InstancesAssignment_2 )* ) ;
    public final void rule__Composite__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1391:1: ( ( ( rule__Composite__InstancesAssignment_2 )* ) )
            // InternalLinguaFranca.g:1392:1: ( ( rule__Composite__InstancesAssignment_2 )* )
            {
            // InternalLinguaFranca.g:1392:1: ( ( rule__Composite__InstancesAssignment_2 )* )
            // InternalLinguaFranca.g:1393:2: ( rule__Composite__InstancesAssignment_2 )*
            {
             before(grammarAccess.getCompositeAccess().getInstancesAssignment_2()); 
            // InternalLinguaFranca.g:1394:2: ( rule__Composite__InstancesAssignment_2 )*
            loop15:
            do {
                int alt15=2;
                int LA15_0 = input.LA(1);

                if ( (LA15_0==RULE_ID) ) {
                    int LA15_1 = input.LA(2);

                    if ( (LA15_1==33) ) {
                        alt15=1;
                    }


                }


                switch (alt15) {
            	case 1 :
            	    // InternalLinguaFranca.g:1394:3: rule__Composite__InstancesAssignment_2
            	    {
            	    pushFollow(FOLLOW_10);
            	    rule__Composite__InstancesAssignment_2();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop15;
                }
            } while (true);

             after(grammarAccess.getCompositeAccess().getInstancesAssignment_2()); 

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
    // InternalLinguaFranca.g:1402:1: rule__Composite__Group__3 : rule__Composite__Group__3__Impl rule__Composite__Group__4 ;
    public final void rule__Composite__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1406:1: ( rule__Composite__Group__3__Impl rule__Composite__Group__4 )
            // InternalLinguaFranca.g:1407:2: rule__Composite__Group__3__Impl rule__Composite__Group__4
            {
            pushFollow(FOLLOW_9);
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
    // InternalLinguaFranca.g:1414:1: rule__Composite__Group__3__Impl : ( ( rule__Composite__ConnectionsAssignment_3 )* ) ;
    public final void rule__Composite__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1418:1: ( ( ( rule__Composite__ConnectionsAssignment_3 )* ) )
            // InternalLinguaFranca.g:1419:1: ( ( rule__Composite__ConnectionsAssignment_3 )* )
            {
            // InternalLinguaFranca.g:1419:1: ( ( rule__Composite__ConnectionsAssignment_3 )* )
            // InternalLinguaFranca.g:1420:2: ( rule__Composite__ConnectionsAssignment_3 )*
            {
             before(grammarAccess.getCompositeAccess().getConnectionsAssignment_3()); 
            // InternalLinguaFranca.g:1421:2: ( rule__Composite__ConnectionsAssignment_3 )*
            loop16:
            do {
                int alt16=2;
                int LA16_0 = input.LA(1);

                if ( (LA16_0==RULE_ID) ) {
                    alt16=1;
                }


                switch (alt16) {
            	case 1 :
            	    // InternalLinguaFranca.g:1421:3: rule__Composite__ConnectionsAssignment_3
            	    {
            	    pushFollow(FOLLOW_10);
            	    rule__Composite__ConnectionsAssignment_3();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop16;
                }
            } while (true);

             after(grammarAccess.getCompositeAccess().getConnectionsAssignment_3()); 

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
    // InternalLinguaFranca.g:1429:1: rule__Composite__Group__4 : rule__Composite__Group__4__Impl ;
    public final void rule__Composite__Group__4() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1433:1: ( rule__Composite__Group__4__Impl )
            // InternalLinguaFranca.g:1434:2: rule__Composite__Group__4__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Composite__Group__4__Impl();

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
    // InternalLinguaFranca.g:1440:1: rule__Composite__Group__4__Impl : ( '}' ) ;
    public final void rule__Composite__Group__4__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1444:1: ( ( '}' ) )
            // InternalLinguaFranca.g:1445:1: ( '}' )
            {
            // InternalLinguaFranca.g:1445:1: ( '}' )
            // InternalLinguaFranca.g:1446:2: '}'
            {
             before(grammarAccess.getCompositeAccess().getRightCurlyBracketKeyword_4()); 
            match(input,24,FOLLOW_2); 
             after(grammarAccess.getCompositeAccess().getRightCurlyBracketKeyword_4()); 

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


    // $ANTLR start "rule__ComponentBody__Group__0"
    // InternalLinguaFranca.g:1456:1: rule__ComponentBody__Group__0 : rule__ComponentBody__Group__0__Impl rule__ComponentBody__Group__1 ;
    public final void rule__ComponentBody__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1460:1: ( rule__ComponentBody__Group__0__Impl rule__ComponentBody__Group__1 )
            // InternalLinguaFranca.g:1461:2: rule__ComponentBody__Group__0__Impl rule__ComponentBody__Group__1
            {
            pushFollow(FOLLOW_11);
            rule__ComponentBody__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__ComponentBody__Group__1();

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
    // $ANTLR end "rule__ComponentBody__Group__0"


    // $ANTLR start "rule__ComponentBody__Group__0__Impl"
    // InternalLinguaFranca.g:1468:1: rule__ComponentBody__Group__0__Impl : ( ( rule__ComponentBody__NameAssignment_0 ) ) ;
    public final void rule__ComponentBody__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1472:1: ( ( ( rule__ComponentBody__NameAssignment_0 ) ) )
            // InternalLinguaFranca.g:1473:1: ( ( rule__ComponentBody__NameAssignment_0 ) )
            {
            // InternalLinguaFranca.g:1473:1: ( ( rule__ComponentBody__NameAssignment_0 ) )
            // InternalLinguaFranca.g:1474:2: ( rule__ComponentBody__NameAssignment_0 )
            {
             before(grammarAccess.getComponentBodyAccess().getNameAssignment_0()); 
            // InternalLinguaFranca.g:1475:2: ( rule__ComponentBody__NameAssignment_0 )
            // InternalLinguaFranca.g:1475:3: rule__ComponentBody__NameAssignment_0
            {
            pushFollow(FOLLOW_2);
            rule__ComponentBody__NameAssignment_0();

            state._fsp--;


            }

             after(grammarAccess.getComponentBodyAccess().getNameAssignment_0()); 

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
    // $ANTLR end "rule__ComponentBody__Group__0__Impl"


    // $ANTLR start "rule__ComponentBody__Group__1"
    // InternalLinguaFranca.g:1483:1: rule__ComponentBody__Group__1 : rule__ComponentBody__Group__1__Impl rule__ComponentBody__Group__2 ;
    public final void rule__ComponentBody__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1487:1: ( rule__ComponentBody__Group__1__Impl rule__ComponentBody__Group__2 )
            // InternalLinguaFranca.g:1488:2: rule__ComponentBody__Group__1__Impl rule__ComponentBody__Group__2
            {
            pushFollow(FOLLOW_11);
            rule__ComponentBody__Group__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__ComponentBody__Group__2();

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
    // $ANTLR end "rule__ComponentBody__Group__1"


    // $ANTLR start "rule__ComponentBody__Group__1__Impl"
    // InternalLinguaFranca.g:1495:1: rule__ComponentBody__Group__1__Impl : ( ( rule__ComponentBody__ParametersAssignment_1 )? ) ;
    public final void rule__ComponentBody__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1499:1: ( ( ( rule__ComponentBody__ParametersAssignment_1 )? ) )
            // InternalLinguaFranca.g:1500:1: ( ( rule__ComponentBody__ParametersAssignment_1 )? )
            {
            // InternalLinguaFranca.g:1500:1: ( ( rule__ComponentBody__ParametersAssignment_1 )? )
            // InternalLinguaFranca.g:1501:2: ( rule__ComponentBody__ParametersAssignment_1 )?
            {
             before(grammarAccess.getComponentBodyAccess().getParametersAssignment_1()); 
            // InternalLinguaFranca.g:1502:2: ( rule__ComponentBody__ParametersAssignment_1 )?
            int alt17=2;
            int LA17_0 = input.LA(1);

            if ( (LA17_0==29) ) {
                alt17=1;
            }
            switch (alt17) {
                case 1 :
                    // InternalLinguaFranca.g:1502:3: rule__ComponentBody__ParametersAssignment_1
                    {
                    pushFollow(FOLLOW_2);
                    rule__ComponentBody__ParametersAssignment_1();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getComponentBodyAccess().getParametersAssignment_1()); 

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
    // $ANTLR end "rule__ComponentBody__Group__1__Impl"


    // $ANTLR start "rule__ComponentBody__Group__2"
    // InternalLinguaFranca.g:1510:1: rule__ComponentBody__Group__2 : rule__ComponentBody__Group__2__Impl rule__ComponentBody__Group__3 ;
    public final void rule__ComponentBody__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1514:1: ( rule__ComponentBody__Group__2__Impl rule__ComponentBody__Group__3 )
            // InternalLinguaFranca.g:1515:2: rule__ComponentBody__Group__2__Impl rule__ComponentBody__Group__3
            {
            pushFollow(FOLLOW_12);
            rule__ComponentBody__Group__2__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__ComponentBody__Group__3();

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
    // $ANTLR end "rule__ComponentBody__Group__2"


    // $ANTLR start "rule__ComponentBody__Group__2__Impl"
    // InternalLinguaFranca.g:1522:1: rule__ComponentBody__Group__2__Impl : ( '{' ) ;
    public final void rule__ComponentBody__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1526:1: ( ( '{' ) )
            // InternalLinguaFranca.g:1527:1: ( '{' )
            {
            // InternalLinguaFranca.g:1527:1: ( '{' )
            // InternalLinguaFranca.g:1528:2: '{'
            {
             before(grammarAccess.getComponentBodyAccess().getLeftCurlyBracketKeyword_2()); 
            match(input,26,FOLLOW_2); 
             after(grammarAccess.getComponentBodyAccess().getLeftCurlyBracketKeyword_2()); 

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
    // $ANTLR end "rule__ComponentBody__Group__2__Impl"


    // $ANTLR start "rule__ComponentBody__Group__3"
    // InternalLinguaFranca.g:1537:1: rule__ComponentBody__Group__3 : rule__ComponentBody__Group__3__Impl rule__ComponentBody__Group__4 ;
    public final void rule__ComponentBody__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1541:1: ( rule__ComponentBody__Group__3__Impl rule__ComponentBody__Group__4 )
            // InternalLinguaFranca.g:1542:2: rule__ComponentBody__Group__3__Impl rule__ComponentBody__Group__4
            {
            pushFollow(FOLLOW_12);
            rule__ComponentBody__Group__3__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__ComponentBody__Group__4();

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
    // $ANTLR end "rule__ComponentBody__Group__3"


    // $ANTLR start "rule__ComponentBody__Group__3__Impl"
    // InternalLinguaFranca.g:1549:1: rule__ComponentBody__Group__3__Impl : ( ( rule__ComponentBody__InputsAssignment_3 )* ) ;
    public final void rule__ComponentBody__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1553:1: ( ( ( rule__ComponentBody__InputsAssignment_3 )* ) )
            // InternalLinguaFranca.g:1554:1: ( ( rule__ComponentBody__InputsAssignment_3 )* )
            {
            // InternalLinguaFranca.g:1554:1: ( ( rule__ComponentBody__InputsAssignment_3 )* )
            // InternalLinguaFranca.g:1555:2: ( rule__ComponentBody__InputsAssignment_3 )*
            {
             before(grammarAccess.getComponentBodyAccess().getInputsAssignment_3()); 
            // InternalLinguaFranca.g:1556:2: ( rule__ComponentBody__InputsAssignment_3 )*
            loop18:
            do {
                int alt18=2;
                int LA18_0 = input.LA(1);

                if ( (LA18_0==13) ) {
                    alt18=1;
                }


                switch (alt18) {
            	case 1 :
            	    // InternalLinguaFranca.g:1556:3: rule__ComponentBody__InputsAssignment_3
            	    {
            	    pushFollow(FOLLOW_13);
            	    rule__ComponentBody__InputsAssignment_3();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop18;
                }
            } while (true);

             after(grammarAccess.getComponentBodyAccess().getInputsAssignment_3()); 

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
    // $ANTLR end "rule__ComponentBody__Group__3__Impl"


    // $ANTLR start "rule__ComponentBody__Group__4"
    // InternalLinguaFranca.g:1564:1: rule__ComponentBody__Group__4 : rule__ComponentBody__Group__4__Impl rule__ComponentBody__Group__5 ;
    public final void rule__ComponentBody__Group__4() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1568:1: ( rule__ComponentBody__Group__4__Impl rule__ComponentBody__Group__5 )
            // InternalLinguaFranca.g:1569:2: rule__ComponentBody__Group__4__Impl rule__ComponentBody__Group__5
            {
            pushFollow(FOLLOW_12);
            rule__ComponentBody__Group__4__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__ComponentBody__Group__5();

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
    // $ANTLR end "rule__ComponentBody__Group__4"


    // $ANTLR start "rule__ComponentBody__Group__4__Impl"
    // InternalLinguaFranca.g:1576:1: rule__ComponentBody__Group__4__Impl : ( ( rule__ComponentBody__OutputsAssignment_4 )* ) ;
    public final void rule__ComponentBody__Group__4__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1580:1: ( ( ( rule__ComponentBody__OutputsAssignment_4 )* ) )
            // InternalLinguaFranca.g:1581:1: ( ( rule__ComponentBody__OutputsAssignment_4 )* )
            {
            // InternalLinguaFranca.g:1581:1: ( ( rule__ComponentBody__OutputsAssignment_4 )* )
            // InternalLinguaFranca.g:1582:2: ( rule__ComponentBody__OutputsAssignment_4 )*
            {
             before(grammarAccess.getComponentBodyAccess().getOutputsAssignment_4()); 
            // InternalLinguaFranca.g:1583:2: ( rule__ComponentBody__OutputsAssignment_4 )*
            loop19:
            do {
                int alt19=2;
                int LA19_0 = input.LA(1);

                if ( (LA19_0==14) ) {
                    alt19=1;
                }


                switch (alt19) {
            	case 1 :
            	    // InternalLinguaFranca.g:1583:3: rule__ComponentBody__OutputsAssignment_4
            	    {
            	    pushFollow(FOLLOW_14);
            	    rule__ComponentBody__OutputsAssignment_4();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop19;
                }
            } while (true);

             after(grammarAccess.getComponentBodyAccess().getOutputsAssignment_4()); 

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
    // $ANTLR end "rule__ComponentBody__Group__4__Impl"


    // $ANTLR start "rule__ComponentBody__Group__5"
    // InternalLinguaFranca.g:1591:1: rule__ComponentBody__Group__5 : rule__ComponentBody__Group__5__Impl rule__ComponentBody__Group__6 ;
    public final void rule__ComponentBody__Group__5() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1595:1: ( rule__ComponentBody__Group__5__Impl rule__ComponentBody__Group__6 )
            // InternalLinguaFranca.g:1596:2: rule__ComponentBody__Group__5__Impl rule__ComponentBody__Group__6
            {
            pushFollow(FOLLOW_12);
            rule__ComponentBody__Group__5__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__ComponentBody__Group__6();

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
    // $ANTLR end "rule__ComponentBody__Group__5"


    // $ANTLR start "rule__ComponentBody__Group__5__Impl"
    // InternalLinguaFranca.g:1603:1: rule__ComponentBody__Group__5__Impl : ( ( rule__ComponentBody__Alternatives_5 )* ) ;
    public final void rule__ComponentBody__Group__5__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1607:1: ( ( ( rule__ComponentBody__Alternatives_5 )* ) )
            // InternalLinguaFranca.g:1608:1: ( ( rule__ComponentBody__Alternatives_5 )* )
            {
            // InternalLinguaFranca.g:1608:1: ( ( rule__ComponentBody__Alternatives_5 )* )
            // InternalLinguaFranca.g:1609:2: ( rule__ComponentBody__Alternatives_5 )*
            {
             before(grammarAccess.getComponentBodyAccess().getAlternatives_5()); 
            // InternalLinguaFranca.g:1610:2: ( rule__ComponentBody__Alternatives_5 )*
            loop20:
            do {
                int alt20=2;
                int LA20_0 = input.LA(1);

                if ( ((LA20_0>=15 && LA20_0<=16)) ) {
                    alt20=1;
                }


                switch (alt20) {
            	case 1 :
            	    // InternalLinguaFranca.g:1610:3: rule__ComponentBody__Alternatives_5
            	    {
            	    pushFollow(FOLLOW_15);
            	    rule__ComponentBody__Alternatives_5();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop20;
                }
            } while (true);

             after(grammarAccess.getComponentBodyAccess().getAlternatives_5()); 

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
    // $ANTLR end "rule__ComponentBody__Group__5__Impl"


    // $ANTLR start "rule__ComponentBody__Group__6"
    // InternalLinguaFranca.g:1618:1: rule__ComponentBody__Group__6 : rule__ComponentBody__Group__6__Impl rule__ComponentBody__Group__7 ;
    public final void rule__ComponentBody__Group__6() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1622:1: ( rule__ComponentBody__Group__6__Impl rule__ComponentBody__Group__7 )
            // InternalLinguaFranca.g:1623:2: rule__ComponentBody__Group__6__Impl rule__ComponentBody__Group__7
            {
            pushFollow(FOLLOW_12);
            rule__ComponentBody__Group__6__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__ComponentBody__Group__7();

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
    // $ANTLR end "rule__ComponentBody__Group__6"


    // $ANTLR start "rule__ComponentBody__Group__6__Impl"
    // InternalLinguaFranca.g:1630:1: rule__ComponentBody__Group__6__Impl : ( ( rule__ComponentBody__PreambleAssignment_6 )? ) ;
    public final void rule__ComponentBody__Group__6__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1634:1: ( ( ( rule__ComponentBody__PreambleAssignment_6 )? ) )
            // InternalLinguaFranca.g:1635:1: ( ( rule__ComponentBody__PreambleAssignment_6 )? )
            {
            // InternalLinguaFranca.g:1635:1: ( ( rule__ComponentBody__PreambleAssignment_6 )? )
            // InternalLinguaFranca.g:1636:2: ( rule__ComponentBody__PreambleAssignment_6 )?
            {
             before(grammarAccess.getComponentBodyAccess().getPreambleAssignment_6()); 
            // InternalLinguaFranca.g:1637:2: ( rule__ComponentBody__PreambleAssignment_6 )?
            int alt21=2;
            int LA21_0 = input.LA(1);

            if ( (LA21_0==32) ) {
                alt21=1;
            }
            switch (alt21) {
                case 1 :
                    // InternalLinguaFranca.g:1637:3: rule__ComponentBody__PreambleAssignment_6
                    {
                    pushFollow(FOLLOW_2);
                    rule__ComponentBody__PreambleAssignment_6();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getComponentBodyAccess().getPreambleAssignment_6()); 

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
    // $ANTLR end "rule__ComponentBody__Group__6__Impl"


    // $ANTLR start "rule__ComponentBody__Group__7"
    // InternalLinguaFranca.g:1645:1: rule__ComponentBody__Group__7 : rule__ComponentBody__Group__7__Impl ;
    public final void rule__ComponentBody__Group__7() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1649:1: ( rule__ComponentBody__Group__7__Impl )
            // InternalLinguaFranca.g:1650:2: rule__ComponentBody__Group__7__Impl
            {
            pushFollow(FOLLOW_2);
            rule__ComponentBody__Group__7__Impl();

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
    // $ANTLR end "rule__ComponentBody__Group__7"


    // $ANTLR start "rule__ComponentBody__Group__7__Impl"
    // InternalLinguaFranca.g:1656:1: rule__ComponentBody__Group__7__Impl : ( ( rule__ComponentBody__ReactionsAssignment_7 )* ) ;
    public final void rule__ComponentBody__Group__7__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1660:1: ( ( ( rule__ComponentBody__ReactionsAssignment_7 )* ) )
            // InternalLinguaFranca.g:1661:1: ( ( rule__ComponentBody__ReactionsAssignment_7 )* )
            {
            // InternalLinguaFranca.g:1661:1: ( ( rule__ComponentBody__ReactionsAssignment_7 )* )
            // InternalLinguaFranca.g:1662:2: ( rule__ComponentBody__ReactionsAssignment_7 )*
            {
             before(grammarAccess.getComponentBodyAccess().getReactionsAssignment_7()); 
            // InternalLinguaFranca.g:1663:2: ( rule__ComponentBody__ReactionsAssignment_7 )*
            loop22:
            do {
                int alt22=2;
                int LA22_0 = input.LA(1);

                if ( (LA22_0==28) ) {
                    alt22=1;
                }


                switch (alt22) {
            	case 1 :
            	    // InternalLinguaFranca.g:1663:3: rule__ComponentBody__ReactionsAssignment_7
            	    {
            	    pushFollow(FOLLOW_16);
            	    rule__ComponentBody__ReactionsAssignment_7();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop22;
                }
            } while (true);

             after(grammarAccess.getComponentBodyAccess().getReactionsAssignment_7()); 

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
    // $ANTLR end "rule__ComponentBody__Group__7__Impl"


    // $ANTLR start "rule__Input__Group__0"
    // InternalLinguaFranca.g:1672:1: rule__Input__Group__0 : rule__Input__Group__0__Impl rule__Input__Group__1 ;
    public final void rule__Input__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1676:1: ( rule__Input__Group__0__Impl rule__Input__Group__1 )
            // InternalLinguaFranca.g:1677:2: rule__Input__Group__0__Impl rule__Input__Group__1
            {
            pushFollow(FOLLOW_17);
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
    // InternalLinguaFranca.g:1684:1: rule__Input__Group__0__Impl : ( 'input' ) ;
    public final void rule__Input__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1688:1: ( ( 'input' ) )
            // InternalLinguaFranca.g:1689:1: ( 'input' )
            {
            // InternalLinguaFranca.g:1689:1: ( 'input' )
            // InternalLinguaFranca.g:1690:2: 'input'
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
    // InternalLinguaFranca.g:1699:1: rule__Input__Group__1 : rule__Input__Group__1__Impl rule__Input__Group__2 ;
    public final void rule__Input__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1703:1: ( rule__Input__Group__1__Impl rule__Input__Group__2 )
            // InternalLinguaFranca.g:1704:2: rule__Input__Group__1__Impl rule__Input__Group__2
            {
            pushFollow(FOLLOW_18);
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
    // InternalLinguaFranca.g:1711:1: rule__Input__Group__1__Impl : ( ( rule__Input__NameAssignment_1 ) ) ;
    public final void rule__Input__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1715:1: ( ( ( rule__Input__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:1716:1: ( ( rule__Input__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:1716:1: ( ( rule__Input__NameAssignment_1 ) )
            // InternalLinguaFranca.g:1717:2: ( rule__Input__NameAssignment_1 )
            {
             before(grammarAccess.getInputAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:1718:2: ( rule__Input__NameAssignment_1 )
            // InternalLinguaFranca.g:1718:3: rule__Input__NameAssignment_1
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
    // InternalLinguaFranca.g:1726:1: rule__Input__Group__2 : rule__Input__Group__2__Impl rule__Input__Group__3 ;
    public final void rule__Input__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1730:1: ( rule__Input__Group__2__Impl rule__Input__Group__3 )
            // InternalLinguaFranca.g:1731:2: rule__Input__Group__2__Impl rule__Input__Group__3
            {
            pushFollow(FOLLOW_18);
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
    // InternalLinguaFranca.g:1738:1: rule__Input__Group__2__Impl : ( ( rule__Input__Group_2__0 )? ) ;
    public final void rule__Input__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1742:1: ( ( ( rule__Input__Group_2__0 )? ) )
            // InternalLinguaFranca.g:1743:1: ( ( rule__Input__Group_2__0 )? )
            {
            // InternalLinguaFranca.g:1743:1: ( ( rule__Input__Group_2__0 )? )
            // InternalLinguaFranca.g:1744:2: ( rule__Input__Group_2__0 )?
            {
             before(grammarAccess.getInputAccess().getGroup_2()); 
            // InternalLinguaFranca.g:1745:2: ( rule__Input__Group_2__0 )?
            int alt23=2;
            int LA23_0 = input.LA(1);

            if ( (LA23_0==27) ) {
                alt23=1;
            }
            switch (alt23) {
                case 1 :
                    // InternalLinguaFranca.g:1745:3: rule__Input__Group_2__0
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
    // InternalLinguaFranca.g:1753:1: rule__Input__Group__3 : rule__Input__Group__3__Impl ;
    public final void rule__Input__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1757:1: ( rule__Input__Group__3__Impl )
            // InternalLinguaFranca.g:1758:2: rule__Input__Group__3__Impl
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
    // InternalLinguaFranca.g:1764:1: rule__Input__Group__3__Impl : ( ';' ) ;
    public final void rule__Input__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1768:1: ( ( ';' ) )
            // InternalLinguaFranca.g:1769:1: ( ';' )
            {
            // InternalLinguaFranca.g:1769:1: ( ';' )
            // InternalLinguaFranca.g:1770:2: ';'
            {
             before(grammarAccess.getInputAccess().getSemicolonKeyword_3()); 
            match(input,21,FOLLOW_2); 
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
    // InternalLinguaFranca.g:1780:1: rule__Input__Group_2__0 : rule__Input__Group_2__0__Impl rule__Input__Group_2__1 ;
    public final void rule__Input__Group_2__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1784:1: ( rule__Input__Group_2__0__Impl rule__Input__Group_2__1 )
            // InternalLinguaFranca.g:1785:2: rule__Input__Group_2__0__Impl rule__Input__Group_2__1
            {
            pushFollow(FOLLOW_19);
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
    // InternalLinguaFranca.g:1792:1: rule__Input__Group_2__0__Impl : ( ':' ) ;
    public final void rule__Input__Group_2__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1796:1: ( ( ':' ) )
            // InternalLinguaFranca.g:1797:1: ( ':' )
            {
            // InternalLinguaFranca.g:1797:1: ( ':' )
            // InternalLinguaFranca.g:1798:2: ':'
            {
             before(grammarAccess.getInputAccess().getColonKeyword_2_0()); 
            match(input,27,FOLLOW_2); 
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
    // InternalLinguaFranca.g:1807:1: rule__Input__Group_2__1 : rule__Input__Group_2__1__Impl ;
    public final void rule__Input__Group_2__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1811:1: ( rule__Input__Group_2__1__Impl )
            // InternalLinguaFranca.g:1812:2: rule__Input__Group_2__1__Impl
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
    // InternalLinguaFranca.g:1818:1: rule__Input__Group_2__1__Impl : ( ( rule__Input__TypeAssignment_2_1 ) ) ;
    public final void rule__Input__Group_2__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1822:1: ( ( ( rule__Input__TypeAssignment_2_1 ) ) )
            // InternalLinguaFranca.g:1823:1: ( ( rule__Input__TypeAssignment_2_1 ) )
            {
            // InternalLinguaFranca.g:1823:1: ( ( rule__Input__TypeAssignment_2_1 ) )
            // InternalLinguaFranca.g:1824:2: ( rule__Input__TypeAssignment_2_1 )
            {
             before(grammarAccess.getInputAccess().getTypeAssignment_2_1()); 
            // InternalLinguaFranca.g:1825:2: ( rule__Input__TypeAssignment_2_1 )
            // InternalLinguaFranca.g:1825:3: rule__Input__TypeAssignment_2_1
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
    // InternalLinguaFranca.g:1834:1: rule__Output__Group__0 : rule__Output__Group__0__Impl rule__Output__Group__1 ;
    public final void rule__Output__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1838:1: ( rule__Output__Group__0__Impl rule__Output__Group__1 )
            // InternalLinguaFranca.g:1839:2: rule__Output__Group__0__Impl rule__Output__Group__1
            {
            pushFollow(FOLLOW_20);
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
    // InternalLinguaFranca.g:1846:1: rule__Output__Group__0__Impl : ( 'output' ) ;
    public final void rule__Output__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1850:1: ( ( 'output' ) )
            // InternalLinguaFranca.g:1851:1: ( 'output' )
            {
            // InternalLinguaFranca.g:1851:1: ( 'output' )
            // InternalLinguaFranca.g:1852:2: 'output'
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
    // InternalLinguaFranca.g:1861:1: rule__Output__Group__1 : rule__Output__Group__1__Impl rule__Output__Group__2 ;
    public final void rule__Output__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1865:1: ( rule__Output__Group__1__Impl rule__Output__Group__2 )
            // InternalLinguaFranca.g:1866:2: rule__Output__Group__1__Impl rule__Output__Group__2
            {
            pushFollow(FOLLOW_18);
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
    // InternalLinguaFranca.g:1873:1: rule__Output__Group__1__Impl : ( ( rule__Output__NameAssignment_1 ) ) ;
    public final void rule__Output__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1877:1: ( ( ( rule__Output__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:1878:1: ( ( rule__Output__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:1878:1: ( ( rule__Output__NameAssignment_1 ) )
            // InternalLinguaFranca.g:1879:2: ( rule__Output__NameAssignment_1 )
            {
             before(grammarAccess.getOutputAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:1880:2: ( rule__Output__NameAssignment_1 )
            // InternalLinguaFranca.g:1880:3: rule__Output__NameAssignment_1
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
    // InternalLinguaFranca.g:1888:1: rule__Output__Group__2 : rule__Output__Group__2__Impl rule__Output__Group__3 ;
    public final void rule__Output__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1892:1: ( rule__Output__Group__2__Impl rule__Output__Group__3 )
            // InternalLinguaFranca.g:1893:2: rule__Output__Group__2__Impl rule__Output__Group__3
            {
            pushFollow(FOLLOW_18);
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
    // InternalLinguaFranca.g:1900:1: rule__Output__Group__2__Impl : ( ( rule__Output__Group_2__0 )? ) ;
    public final void rule__Output__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1904:1: ( ( ( rule__Output__Group_2__0 )? ) )
            // InternalLinguaFranca.g:1905:1: ( ( rule__Output__Group_2__0 )? )
            {
            // InternalLinguaFranca.g:1905:1: ( ( rule__Output__Group_2__0 )? )
            // InternalLinguaFranca.g:1906:2: ( rule__Output__Group_2__0 )?
            {
             before(grammarAccess.getOutputAccess().getGroup_2()); 
            // InternalLinguaFranca.g:1907:2: ( rule__Output__Group_2__0 )?
            int alt24=2;
            int LA24_0 = input.LA(1);

            if ( (LA24_0==27) ) {
                alt24=1;
            }
            switch (alt24) {
                case 1 :
                    // InternalLinguaFranca.g:1907:3: rule__Output__Group_2__0
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
    // InternalLinguaFranca.g:1915:1: rule__Output__Group__3 : rule__Output__Group__3__Impl ;
    public final void rule__Output__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1919:1: ( rule__Output__Group__3__Impl )
            // InternalLinguaFranca.g:1920:2: rule__Output__Group__3__Impl
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
    // InternalLinguaFranca.g:1926:1: rule__Output__Group__3__Impl : ( ';' ) ;
    public final void rule__Output__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1930:1: ( ( ';' ) )
            // InternalLinguaFranca.g:1931:1: ( ';' )
            {
            // InternalLinguaFranca.g:1931:1: ( ';' )
            // InternalLinguaFranca.g:1932:2: ';'
            {
             before(grammarAccess.getOutputAccess().getSemicolonKeyword_3()); 
            match(input,21,FOLLOW_2); 
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
    // InternalLinguaFranca.g:1942:1: rule__Output__Group_2__0 : rule__Output__Group_2__0__Impl rule__Output__Group_2__1 ;
    public final void rule__Output__Group_2__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1946:1: ( rule__Output__Group_2__0__Impl rule__Output__Group_2__1 )
            // InternalLinguaFranca.g:1947:2: rule__Output__Group_2__0__Impl rule__Output__Group_2__1
            {
            pushFollow(FOLLOW_19);
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
    // InternalLinguaFranca.g:1954:1: rule__Output__Group_2__0__Impl : ( ':' ) ;
    public final void rule__Output__Group_2__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1958:1: ( ( ':' ) )
            // InternalLinguaFranca.g:1959:1: ( ':' )
            {
            // InternalLinguaFranca.g:1959:1: ( ':' )
            // InternalLinguaFranca.g:1960:2: ':'
            {
             before(grammarAccess.getOutputAccess().getColonKeyword_2_0()); 
            match(input,27,FOLLOW_2); 
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
    // InternalLinguaFranca.g:1969:1: rule__Output__Group_2__1 : rule__Output__Group_2__1__Impl ;
    public final void rule__Output__Group_2__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1973:1: ( rule__Output__Group_2__1__Impl )
            // InternalLinguaFranca.g:1974:2: rule__Output__Group_2__1__Impl
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
    // InternalLinguaFranca.g:1980:1: rule__Output__Group_2__1__Impl : ( ( rule__Output__TypeAssignment_2_1 ) ) ;
    public final void rule__Output__Group_2__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1984:1: ( ( ( rule__Output__TypeAssignment_2_1 ) ) )
            // InternalLinguaFranca.g:1985:1: ( ( rule__Output__TypeAssignment_2_1 ) )
            {
            // InternalLinguaFranca.g:1985:1: ( ( rule__Output__TypeAssignment_2_1 ) )
            // InternalLinguaFranca.g:1986:2: ( rule__Output__TypeAssignment_2_1 )
            {
             before(grammarAccess.getOutputAccess().getTypeAssignment_2_1()); 
            // InternalLinguaFranca.g:1987:2: ( rule__Output__TypeAssignment_2_1 )
            // InternalLinguaFranca.g:1987:3: rule__Output__TypeAssignment_2_1
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


    // $ANTLR start "rule__Timer__Group__0"
    // InternalLinguaFranca.g:1996:1: rule__Timer__Group__0 : rule__Timer__Group__0__Impl rule__Timer__Group__1 ;
    public final void rule__Timer__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2000:1: ( rule__Timer__Group__0__Impl rule__Timer__Group__1 )
            // InternalLinguaFranca.g:2001:2: rule__Timer__Group__0__Impl rule__Timer__Group__1
            {
            pushFollow(FOLLOW_21);
            rule__Timer__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Timer__Group__1();

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
    // $ANTLR end "rule__Timer__Group__0"


    // $ANTLR start "rule__Timer__Group__0__Impl"
    // InternalLinguaFranca.g:2008:1: rule__Timer__Group__0__Impl : ( 'timer' ) ;
    public final void rule__Timer__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2012:1: ( ( 'timer' ) )
            // InternalLinguaFranca.g:2013:1: ( 'timer' )
            {
            // InternalLinguaFranca.g:2013:1: ( 'timer' )
            // InternalLinguaFranca.g:2014:2: 'timer'
            {
             before(grammarAccess.getTimerAccess().getTimerKeyword_0()); 
            match(input,15,FOLLOW_2); 
             after(grammarAccess.getTimerAccess().getTimerKeyword_0()); 

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
    // $ANTLR end "rule__Timer__Group__0__Impl"


    // $ANTLR start "rule__Timer__Group__1"
    // InternalLinguaFranca.g:2023:1: rule__Timer__Group__1 : rule__Timer__Group__1__Impl rule__Timer__Group__2 ;
    public final void rule__Timer__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2027:1: ( rule__Timer__Group__1__Impl rule__Timer__Group__2 )
            // InternalLinguaFranca.g:2028:2: rule__Timer__Group__1__Impl rule__Timer__Group__2
            {
            pushFollow(FOLLOW_22);
            rule__Timer__Group__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Timer__Group__2();

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
    // $ANTLR end "rule__Timer__Group__1"


    // $ANTLR start "rule__Timer__Group__1__Impl"
    // InternalLinguaFranca.g:2035:1: rule__Timer__Group__1__Impl : ( ( rule__Timer__NameAssignment_1 ) ) ;
    public final void rule__Timer__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2039:1: ( ( ( rule__Timer__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:2040:1: ( ( rule__Timer__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:2040:1: ( ( rule__Timer__NameAssignment_1 ) )
            // InternalLinguaFranca.g:2041:2: ( rule__Timer__NameAssignment_1 )
            {
             before(grammarAccess.getTimerAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:2042:2: ( rule__Timer__NameAssignment_1 )
            // InternalLinguaFranca.g:2042:3: rule__Timer__NameAssignment_1
            {
            pushFollow(FOLLOW_2);
            rule__Timer__NameAssignment_1();

            state._fsp--;


            }

             after(grammarAccess.getTimerAccess().getNameAssignment_1()); 

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
    // $ANTLR end "rule__Timer__Group__1__Impl"


    // $ANTLR start "rule__Timer__Group__2"
    // InternalLinguaFranca.g:2050:1: rule__Timer__Group__2 : rule__Timer__Group__2__Impl rule__Timer__Group__3 ;
    public final void rule__Timer__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2054:1: ( rule__Timer__Group__2__Impl rule__Timer__Group__3 )
            // InternalLinguaFranca.g:2055:2: rule__Timer__Group__2__Impl rule__Timer__Group__3
            {
            pushFollow(FOLLOW_22);
            rule__Timer__Group__2__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Timer__Group__3();

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
    // $ANTLR end "rule__Timer__Group__2"


    // $ANTLR start "rule__Timer__Group__2__Impl"
    // InternalLinguaFranca.g:2062:1: rule__Timer__Group__2__Impl : ( ( rule__Timer__TimingAssignment_2 )? ) ;
    public final void rule__Timer__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2066:1: ( ( ( rule__Timer__TimingAssignment_2 )? ) )
            // InternalLinguaFranca.g:2067:1: ( ( rule__Timer__TimingAssignment_2 )? )
            {
            // InternalLinguaFranca.g:2067:1: ( ( rule__Timer__TimingAssignment_2 )? )
            // InternalLinguaFranca.g:2068:2: ( rule__Timer__TimingAssignment_2 )?
            {
             before(grammarAccess.getTimerAccess().getTimingAssignment_2()); 
            // InternalLinguaFranca.g:2069:2: ( rule__Timer__TimingAssignment_2 )?
            int alt25=2;
            int LA25_0 = input.LA(1);

            if ( (LA25_0==29) ) {
                alt25=1;
            }
            switch (alt25) {
                case 1 :
                    // InternalLinguaFranca.g:2069:3: rule__Timer__TimingAssignment_2
                    {
                    pushFollow(FOLLOW_2);
                    rule__Timer__TimingAssignment_2();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getTimerAccess().getTimingAssignment_2()); 

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
    // $ANTLR end "rule__Timer__Group__2__Impl"


    // $ANTLR start "rule__Timer__Group__3"
    // InternalLinguaFranca.g:2077:1: rule__Timer__Group__3 : rule__Timer__Group__3__Impl ;
    public final void rule__Timer__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2081:1: ( rule__Timer__Group__3__Impl )
            // InternalLinguaFranca.g:2082:2: rule__Timer__Group__3__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Timer__Group__3__Impl();

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
    // $ANTLR end "rule__Timer__Group__3"


    // $ANTLR start "rule__Timer__Group__3__Impl"
    // InternalLinguaFranca.g:2088:1: rule__Timer__Group__3__Impl : ( ';' ) ;
    public final void rule__Timer__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2092:1: ( ( ';' ) )
            // InternalLinguaFranca.g:2093:1: ( ';' )
            {
            // InternalLinguaFranca.g:2093:1: ( ';' )
            // InternalLinguaFranca.g:2094:2: ';'
            {
             before(grammarAccess.getTimerAccess().getSemicolonKeyword_3()); 
            match(input,21,FOLLOW_2); 
             after(grammarAccess.getTimerAccess().getSemicolonKeyword_3()); 

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
    // $ANTLR end "rule__Timer__Group__3__Impl"


    // $ANTLR start "rule__Action__Group__0"
    // InternalLinguaFranca.g:2104:1: rule__Action__Group__0 : rule__Action__Group__0__Impl rule__Action__Group__1 ;
    public final void rule__Action__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2108:1: ( rule__Action__Group__0__Impl rule__Action__Group__1 )
            // InternalLinguaFranca.g:2109:2: rule__Action__Group__0__Impl rule__Action__Group__1
            {
            pushFollow(FOLLOW_23);
            rule__Action__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Action__Group__1();

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
    // $ANTLR end "rule__Action__Group__0"


    // $ANTLR start "rule__Action__Group__0__Impl"
    // InternalLinguaFranca.g:2116:1: rule__Action__Group__0__Impl : ( 'action' ) ;
    public final void rule__Action__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2120:1: ( ( 'action' ) )
            // InternalLinguaFranca.g:2121:1: ( 'action' )
            {
            // InternalLinguaFranca.g:2121:1: ( 'action' )
            // InternalLinguaFranca.g:2122:2: 'action'
            {
             before(grammarAccess.getActionAccess().getActionKeyword_0()); 
            match(input,16,FOLLOW_2); 
             after(grammarAccess.getActionAccess().getActionKeyword_0()); 

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
    // $ANTLR end "rule__Action__Group__0__Impl"


    // $ANTLR start "rule__Action__Group__1"
    // InternalLinguaFranca.g:2131:1: rule__Action__Group__1 : rule__Action__Group__1__Impl rule__Action__Group__2 ;
    public final void rule__Action__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2135:1: ( rule__Action__Group__1__Impl rule__Action__Group__2 )
            // InternalLinguaFranca.g:2136:2: rule__Action__Group__1__Impl rule__Action__Group__2
            {
            pushFollow(FOLLOW_22);
            rule__Action__Group__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Action__Group__2();

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
    // $ANTLR end "rule__Action__Group__1"


    // $ANTLR start "rule__Action__Group__1__Impl"
    // InternalLinguaFranca.g:2143:1: rule__Action__Group__1__Impl : ( ( rule__Action__NameAssignment_1 ) ) ;
    public final void rule__Action__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2147:1: ( ( ( rule__Action__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:2148:1: ( ( rule__Action__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:2148:1: ( ( rule__Action__NameAssignment_1 ) )
            // InternalLinguaFranca.g:2149:2: ( rule__Action__NameAssignment_1 )
            {
             before(grammarAccess.getActionAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:2150:2: ( rule__Action__NameAssignment_1 )
            // InternalLinguaFranca.g:2150:3: rule__Action__NameAssignment_1
            {
            pushFollow(FOLLOW_2);
            rule__Action__NameAssignment_1();

            state._fsp--;


            }

             after(grammarAccess.getActionAccess().getNameAssignment_1()); 

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
    // $ANTLR end "rule__Action__Group__1__Impl"


    // $ANTLR start "rule__Action__Group__2"
    // InternalLinguaFranca.g:2158:1: rule__Action__Group__2 : rule__Action__Group__2__Impl rule__Action__Group__3 ;
    public final void rule__Action__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2162:1: ( rule__Action__Group__2__Impl rule__Action__Group__3 )
            // InternalLinguaFranca.g:2163:2: rule__Action__Group__2__Impl rule__Action__Group__3
            {
            pushFollow(FOLLOW_22);
            rule__Action__Group__2__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Action__Group__3();

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
    // $ANTLR end "rule__Action__Group__2"


    // $ANTLR start "rule__Action__Group__2__Impl"
    // InternalLinguaFranca.g:2170:1: rule__Action__Group__2__Impl : ( ( rule__Action__TimingAssignment_2 )? ) ;
    public final void rule__Action__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2174:1: ( ( ( rule__Action__TimingAssignment_2 )? ) )
            // InternalLinguaFranca.g:2175:1: ( ( rule__Action__TimingAssignment_2 )? )
            {
            // InternalLinguaFranca.g:2175:1: ( ( rule__Action__TimingAssignment_2 )? )
            // InternalLinguaFranca.g:2176:2: ( rule__Action__TimingAssignment_2 )?
            {
             before(grammarAccess.getActionAccess().getTimingAssignment_2()); 
            // InternalLinguaFranca.g:2177:2: ( rule__Action__TimingAssignment_2 )?
            int alt26=2;
            int LA26_0 = input.LA(1);

            if ( (LA26_0==29) ) {
                alt26=1;
            }
            switch (alt26) {
                case 1 :
                    // InternalLinguaFranca.g:2177:3: rule__Action__TimingAssignment_2
                    {
                    pushFollow(FOLLOW_2);
                    rule__Action__TimingAssignment_2();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getActionAccess().getTimingAssignment_2()); 

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
    // $ANTLR end "rule__Action__Group__2__Impl"


    // $ANTLR start "rule__Action__Group__3"
    // InternalLinguaFranca.g:2185:1: rule__Action__Group__3 : rule__Action__Group__3__Impl ;
    public final void rule__Action__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2189:1: ( rule__Action__Group__3__Impl )
            // InternalLinguaFranca.g:2190:2: rule__Action__Group__3__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Action__Group__3__Impl();

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
    // $ANTLR end "rule__Action__Group__3"


    // $ANTLR start "rule__Action__Group__3__Impl"
    // InternalLinguaFranca.g:2196:1: rule__Action__Group__3__Impl : ( ';' ) ;
    public final void rule__Action__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2200:1: ( ( ';' ) )
            // InternalLinguaFranca.g:2201:1: ( ';' )
            {
            // InternalLinguaFranca.g:2201:1: ( ';' )
            // InternalLinguaFranca.g:2202:2: ';'
            {
             before(grammarAccess.getActionAccess().getSemicolonKeyword_3()); 
            match(input,21,FOLLOW_2); 
             after(grammarAccess.getActionAccess().getSemicolonKeyword_3()); 

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
    // $ANTLR end "rule__Action__Group__3__Impl"


    // $ANTLR start "rule__Reaction__Group__0"
    // InternalLinguaFranca.g:2212:1: rule__Reaction__Group__0 : rule__Reaction__Group__0__Impl rule__Reaction__Group__1 ;
    public final void rule__Reaction__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2216:1: ( rule__Reaction__Group__0__Impl rule__Reaction__Group__1 )
            // InternalLinguaFranca.g:2217:2: rule__Reaction__Group__0__Impl rule__Reaction__Group__1
            {
            pushFollow(FOLLOW_24);
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
    // InternalLinguaFranca.g:2224:1: rule__Reaction__Group__0__Impl : ( 'reaction' ) ;
    public final void rule__Reaction__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2228:1: ( ( 'reaction' ) )
            // InternalLinguaFranca.g:2229:1: ( 'reaction' )
            {
            // InternalLinguaFranca.g:2229:1: ( 'reaction' )
            // InternalLinguaFranca.g:2230:2: 'reaction'
            {
             before(grammarAccess.getReactionAccess().getReactionKeyword_0()); 
            match(input,28,FOLLOW_2); 
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
    // InternalLinguaFranca.g:2239:1: rule__Reaction__Group__1 : rule__Reaction__Group__1__Impl rule__Reaction__Group__2 ;
    public final void rule__Reaction__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2243:1: ( rule__Reaction__Group__1__Impl rule__Reaction__Group__2 )
            // InternalLinguaFranca.g:2244:2: rule__Reaction__Group__1__Impl rule__Reaction__Group__2
            {
            pushFollow(FOLLOW_24);
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
    // InternalLinguaFranca.g:2251:1: rule__Reaction__Group__1__Impl : ( ( rule__Reaction__Group_1__0 )? ) ;
    public final void rule__Reaction__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2255:1: ( ( ( rule__Reaction__Group_1__0 )? ) )
            // InternalLinguaFranca.g:2256:1: ( ( rule__Reaction__Group_1__0 )? )
            {
            // InternalLinguaFranca.g:2256:1: ( ( rule__Reaction__Group_1__0 )? )
            // InternalLinguaFranca.g:2257:2: ( rule__Reaction__Group_1__0 )?
            {
             before(grammarAccess.getReactionAccess().getGroup_1()); 
            // InternalLinguaFranca.g:2258:2: ( rule__Reaction__Group_1__0 )?
            int alt27=2;
            int LA27_0 = input.LA(1);

            if ( (LA27_0==29) ) {
                alt27=1;
            }
            switch (alt27) {
                case 1 :
                    // InternalLinguaFranca.g:2258:3: rule__Reaction__Group_1__0
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
    // InternalLinguaFranca.g:2266:1: rule__Reaction__Group__2 : rule__Reaction__Group__2__Impl rule__Reaction__Group__3 ;
    public final void rule__Reaction__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2270:1: ( rule__Reaction__Group__2__Impl rule__Reaction__Group__3 )
            // InternalLinguaFranca.g:2271:2: rule__Reaction__Group__2__Impl rule__Reaction__Group__3
            {
            pushFollow(FOLLOW_24);
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
    // InternalLinguaFranca.g:2278:1: rule__Reaction__Group__2__Impl : ( ( rule__Reaction__GetsAssignment_2 )? ) ;
    public final void rule__Reaction__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2282:1: ( ( ( rule__Reaction__GetsAssignment_2 )? ) )
            // InternalLinguaFranca.g:2283:1: ( ( rule__Reaction__GetsAssignment_2 )? )
            {
            // InternalLinguaFranca.g:2283:1: ( ( rule__Reaction__GetsAssignment_2 )? )
            // InternalLinguaFranca.g:2284:2: ( rule__Reaction__GetsAssignment_2 )?
            {
             before(grammarAccess.getReactionAccess().getGetsAssignment_2()); 
            // InternalLinguaFranca.g:2285:2: ( rule__Reaction__GetsAssignment_2 )?
            int alt28=2;
            int LA28_0 = input.LA(1);

            if ( (LA28_0==RULE_ID) ) {
                alt28=1;
            }
            switch (alt28) {
                case 1 :
                    // InternalLinguaFranca.g:2285:3: rule__Reaction__GetsAssignment_2
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
    // InternalLinguaFranca.g:2293:1: rule__Reaction__Group__3 : rule__Reaction__Group__3__Impl rule__Reaction__Group__4 ;
    public final void rule__Reaction__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2297:1: ( rule__Reaction__Group__3__Impl rule__Reaction__Group__4 )
            // InternalLinguaFranca.g:2298:2: rule__Reaction__Group__3__Impl rule__Reaction__Group__4
            {
            pushFollow(FOLLOW_24);
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
    // InternalLinguaFranca.g:2305:1: rule__Reaction__Group__3__Impl : ( ( rule__Reaction__SetsAssignment_3 )? ) ;
    public final void rule__Reaction__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2309:1: ( ( ( rule__Reaction__SetsAssignment_3 )? ) )
            // InternalLinguaFranca.g:2310:1: ( ( rule__Reaction__SetsAssignment_3 )? )
            {
            // InternalLinguaFranca.g:2310:1: ( ( rule__Reaction__SetsAssignment_3 )? )
            // InternalLinguaFranca.g:2311:2: ( rule__Reaction__SetsAssignment_3 )?
            {
             before(grammarAccess.getReactionAccess().getSetsAssignment_3()); 
            // InternalLinguaFranca.g:2312:2: ( rule__Reaction__SetsAssignment_3 )?
            int alt29=2;
            int LA29_0 = input.LA(1);

            if ( (LA29_0==35) ) {
                alt29=1;
            }
            switch (alt29) {
                case 1 :
                    // InternalLinguaFranca.g:2312:3: rule__Reaction__SetsAssignment_3
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
    // InternalLinguaFranca.g:2320:1: rule__Reaction__Group__4 : rule__Reaction__Group__4__Impl ;
    public final void rule__Reaction__Group__4() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2324:1: ( rule__Reaction__Group__4__Impl )
            // InternalLinguaFranca.g:2325:2: rule__Reaction__Group__4__Impl
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
    // InternalLinguaFranca.g:2331:1: rule__Reaction__Group__4__Impl : ( ( rule__Reaction__CodeAssignment_4 ) ) ;
    public final void rule__Reaction__Group__4__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2335:1: ( ( ( rule__Reaction__CodeAssignment_4 ) ) )
            // InternalLinguaFranca.g:2336:1: ( ( rule__Reaction__CodeAssignment_4 ) )
            {
            // InternalLinguaFranca.g:2336:1: ( ( rule__Reaction__CodeAssignment_4 ) )
            // InternalLinguaFranca.g:2337:2: ( rule__Reaction__CodeAssignment_4 )
            {
             before(grammarAccess.getReactionAccess().getCodeAssignment_4()); 
            // InternalLinguaFranca.g:2338:2: ( rule__Reaction__CodeAssignment_4 )
            // InternalLinguaFranca.g:2338:3: rule__Reaction__CodeAssignment_4
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
    // InternalLinguaFranca.g:2347:1: rule__Reaction__Group_1__0 : rule__Reaction__Group_1__0__Impl rule__Reaction__Group_1__1 ;
    public final void rule__Reaction__Group_1__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2351:1: ( rule__Reaction__Group_1__0__Impl rule__Reaction__Group_1__1 )
            // InternalLinguaFranca.g:2352:2: rule__Reaction__Group_1__0__Impl rule__Reaction__Group_1__1
            {
            pushFollow(FOLLOW_25);
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
    // InternalLinguaFranca.g:2359:1: rule__Reaction__Group_1__0__Impl : ( '(' ) ;
    public final void rule__Reaction__Group_1__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2363:1: ( ( '(' ) )
            // InternalLinguaFranca.g:2364:1: ( '(' )
            {
            // InternalLinguaFranca.g:2364:1: ( '(' )
            // InternalLinguaFranca.g:2365:2: '('
            {
             before(grammarAccess.getReactionAccess().getLeftParenthesisKeyword_1_0()); 
            match(input,29,FOLLOW_2); 
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
    // InternalLinguaFranca.g:2374:1: rule__Reaction__Group_1__1 : rule__Reaction__Group_1__1__Impl rule__Reaction__Group_1__2 ;
    public final void rule__Reaction__Group_1__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2378:1: ( rule__Reaction__Group_1__1__Impl rule__Reaction__Group_1__2 )
            // InternalLinguaFranca.g:2379:2: rule__Reaction__Group_1__1__Impl rule__Reaction__Group_1__2
            {
            pushFollow(FOLLOW_25);
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
    // InternalLinguaFranca.g:2386:1: rule__Reaction__Group_1__1__Impl : ( ( rule__Reaction__Group_1_1__0 )? ) ;
    public final void rule__Reaction__Group_1__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2390:1: ( ( ( rule__Reaction__Group_1_1__0 )? ) )
            // InternalLinguaFranca.g:2391:1: ( ( rule__Reaction__Group_1_1__0 )? )
            {
            // InternalLinguaFranca.g:2391:1: ( ( rule__Reaction__Group_1_1__0 )? )
            // InternalLinguaFranca.g:2392:2: ( rule__Reaction__Group_1_1__0 )?
            {
             before(grammarAccess.getReactionAccess().getGroup_1_1()); 
            // InternalLinguaFranca.g:2393:2: ( rule__Reaction__Group_1_1__0 )?
            int alt30=2;
            int LA30_0 = input.LA(1);

            if ( (LA30_0==RULE_ID) ) {
                alt30=1;
            }
            switch (alt30) {
                case 1 :
                    // InternalLinguaFranca.g:2393:3: rule__Reaction__Group_1_1__0
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
    // InternalLinguaFranca.g:2401:1: rule__Reaction__Group_1__2 : rule__Reaction__Group_1__2__Impl ;
    public final void rule__Reaction__Group_1__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2405:1: ( rule__Reaction__Group_1__2__Impl )
            // InternalLinguaFranca.g:2406:2: rule__Reaction__Group_1__2__Impl
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
    // InternalLinguaFranca.g:2412:1: rule__Reaction__Group_1__2__Impl : ( ')' ) ;
    public final void rule__Reaction__Group_1__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2416:1: ( ( ')' ) )
            // InternalLinguaFranca.g:2417:1: ( ')' )
            {
            // InternalLinguaFranca.g:2417:1: ( ')' )
            // InternalLinguaFranca.g:2418:2: ')'
            {
             before(grammarAccess.getReactionAccess().getRightParenthesisKeyword_1_2()); 
            match(input,30,FOLLOW_2); 
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
    // InternalLinguaFranca.g:2428:1: rule__Reaction__Group_1_1__0 : rule__Reaction__Group_1_1__0__Impl rule__Reaction__Group_1_1__1 ;
    public final void rule__Reaction__Group_1_1__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2432:1: ( rule__Reaction__Group_1_1__0__Impl rule__Reaction__Group_1_1__1 )
            // InternalLinguaFranca.g:2433:2: rule__Reaction__Group_1_1__0__Impl rule__Reaction__Group_1_1__1
            {
            pushFollow(FOLLOW_26);
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
    // InternalLinguaFranca.g:2440:1: rule__Reaction__Group_1_1__0__Impl : ( ( rule__Reaction__TriggersAssignment_1_1_0 ) ) ;
    public final void rule__Reaction__Group_1_1__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2444:1: ( ( ( rule__Reaction__TriggersAssignment_1_1_0 ) ) )
            // InternalLinguaFranca.g:2445:1: ( ( rule__Reaction__TriggersAssignment_1_1_0 ) )
            {
            // InternalLinguaFranca.g:2445:1: ( ( rule__Reaction__TriggersAssignment_1_1_0 ) )
            // InternalLinguaFranca.g:2446:2: ( rule__Reaction__TriggersAssignment_1_1_0 )
            {
             before(grammarAccess.getReactionAccess().getTriggersAssignment_1_1_0()); 
            // InternalLinguaFranca.g:2447:2: ( rule__Reaction__TriggersAssignment_1_1_0 )
            // InternalLinguaFranca.g:2447:3: rule__Reaction__TriggersAssignment_1_1_0
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
    // InternalLinguaFranca.g:2455:1: rule__Reaction__Group_1_1__1 : rule__Reaction__Group_1_1__1__Impl ;
    public final void rule__Reaction__Group_1_1__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2459:1: ( rule__Reaction__Group_1_1__1__Impl )
            // InternalLinguaFranca.g:2460:2: rule__Reaction__Group_1_1__1__Impl
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
    // InternalLinguaFranca.g:2466:1: rule__Reaction__Group_1_1__1__Impl : ( ( rule__Reaction__Group_1_1_1__0 )* ) ;
    public final void rule__Reaction__Group_1_1__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2470:1: ( ( ( rule__Reaction__Group_1_1_1__0 )* ) )
            // InternalLinguaFranca.g:2471:1: ( ( rule__Reaction__Group_1_1_1__0 )* )
            {
            // InternalLinguaFranca.g:2471:1: ( ( rule__Reaction__Group_1_1_1__0 )* )
            // InternalLinguaFranca.g:2472:2: ( rule__Reaction__Group_1_1_1__0 )*
            {
             before(grammarAccess.getReactionAccess().getGroup_1_1_1()); 
            // InternalLinguaFranca.g:2473:2: ( rule__Reaction__Group_1_1_1__0 )*
            loop31:
            do {
                int alt31=2;
                int LA31_0 = input.LA(1);

                if ( (LA31_0==31) ) {
                    alt31=1;
                }


                switch (alt31) {
            	case 1 :
            	    // InternalLinguaFranca.g:2473:3: rule__Reaction__Group_1_1_1__0
            	    {
            	    pushFollow(FOLLOW_27);
            	    rule__Reaction__Group_1_1_1__0();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop31;
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
    // InternalLinguaFranca.g:2482:1: rule__Reaction__Group_1_1_1__0 : rule__Reaction__Group_1_1_1__0__Impl rule__Reaction__Group_1_1_1__1 ;
    public final void rule__Reaction__Group_1_1_1__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2486:1: ( rule__Reaction__Group_1_1_1__0__Impl rule__Reaction__Group_1_1_1__1 )
            // InternalLinguaFranca.g:2487:2: rule__Reaction__Group_1_1_1__0__Impl rule__Reaction__Group_1_1_1__1
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
    // InternalLinguaFranca.g:2494:1: rule__Reaction__Group_1_1_1__0__Impl : ( ',' ) ;
    public final void rule__Reaction__Group_1_1_1__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2498:1: ( ( ',' ) )
            // InternalLinguaFranca.g:2499:1: ( ',' )
            {
            // InternalLinguaFranca.g:2499:1: ( ',' )
            // InternalLinguaFranca.g:2500:2: ','
            {
             before(grammarAccess.getReactionAccess().getCommaKeyword_1_1_1_0()); 
            match(input,31,FOLLOW_2); 
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
    // InternalLinguaFranca.g:2509:1: rule__Reaction__Group_1_1_1__1 : rule__Reaction__Group_1_1_1__1__Impl ;
    public final void rule__Reaction__Group_1_1_1__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2513:1: ( rule__Reaction__Group_1_1_1__1__Impl )
            // InternalLinguaFranca.g:2514:2: rule__Reaction__Group_1_1_1__1__Impl
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
    // InternalLinguaFranca.g:2520:1: rule__Reaction__Group_1_1_1__1__Impl : ( ( rule__Reaction__TriggersAssignment_1_1_1_1 ) ) ;
    public final void rule__Reaction__Group_1_1_1__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2524:1: ( ( ( rule__Reaction__TriggersAssignment_1_1_1_1 ) ) )
            // InternalLinguaFranca.g:2525:1: ( ( rule__Reaction__TriggersAssignment_1_1_1_1 ) )
            {
            // InternalLinguaFranca.g:2525:1: ( ( rule__Reaction__TriggersAssignment_1_1_1_1 ) )
            // InternalLinguaFranca.g:2526:2: ( rule__Reaction__TriggersAssignment_1_1_1_1 )
            {
             before(grammarAccess.getReactionAccess().getTriggersAssignment_1_1_1_1()); 
            // InternalLinguaFranca.g:2527:2: ( rule__Reaction__TriggersAssignment_1_1_1_1 )
            // InternalLinguaFranca.g:2527:3: rule__Reaction__TriggersAssignment_1_1_1_1
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
    // InternalLinguaFranca.g:2536:1: rule__Preamble__Group__0 : rule__Preamble__Group__0__Impl rule__Preamble__Group__1 ;
    public final void rule__Preamble__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2540:1: ( rule__Preamble__Group__0__Impl rule__Preamble__Group__1 )
            // InternalLinguaFranca.g:2541:2: rule__Preamble__Group__0__Impl rule__Preamble__Group__1
            {
            pushFollow(FOLLOW_28);
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
    // InternalLinguaFranca.g:2548:1: rule__Preamble__Group__0__Impl : ( 'preamble' ) ;
    public final void rule__Preamble__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2552:1: ( ( 'preamble' ) )
            // InternalLinguaFranca.g:2553:1: ( 'preamble' )
            {
            // InternalLinguaFranca.g:2553:1: ( 'preamble' )
            // InternalLinguaFranca.g:2554:2: 'preamble'
            {
             before(grammarAccess.getPreambleAccess().getPreambleKeyword_0()); 
            match(input,32,FOLLOW_2); 
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
    // InternalLinguaFranca.g:2563:1: rule__Preamble__Group__1 : rule__Preamble__Group__1__Impl ;
    public final void rule__Preamble__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2567:1: ( rule__Preamble__Group__1__Impl )
            // InternalLinguaFranca.g:2568:2: rule__Preamble__Group__1__Impl
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
    // InternalLinguaFranca.g:2574:1: rule__Preamble__Group__1__Impl : ( ( rule__Preamble__CodeAssignment_1 ) ) ;
    public final void rule__Preamble__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2578:1: ( ( ( rule__Preamble__CodeAssignment_1 ) ) )
            // InternalLinguaFranca.g:2579:1: ( ( rule__Preamble__CodeAssignment_1 ) )
            {
            // InternalLinguaFranca.g:2579:1: ( ( rule__Preamble__CodeAssignment_1 ) )
            // InternalLinguaFranca.g:2580:2: ( rule__Preamble__CodeAssignment_1 )
            {
             before(grammarAccess.getPreambleAccess().getCodeAssignment_1()); 
            // InternalLinguaFranca.g:2581:2: ( rule__Preamble__CodeAssignment_1 )
            // InternalLinguaFranca.g:2581:3: rule__Preamble__CodeAssignment_1
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


    // $ANTLR start "rule__Instance__Group__0"
    // InternalLinguaFranca.g:2590:1: rule__Instance__Group__0 : rule__Instance__Group__0__Impl rule__Instance__Group__1 ;
    public final void rule__Instance__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2594:1: ( rule__Instance__Group__0__Impl rule__Instance__Group__1 )
            // InternalLinguaFranca.g:2595:2: rule__Instance__Group__0__Impl rule__Instance__Group__1
            {
            pushFollow(FOLLOW_29);
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
    // InternalLinguaFranca.g:2602:1: rule__Instance__Group__0__Impl : ( ( rule__Instance__NameAssignment_0 ) ) ;
    public final void rule__Instance__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2606:1: ( ( ( rule__Instance__NameAssignment_0 ) ) )
            // InternalLinguaFranca.g:2607:1: ( ( rule__Instance__NameAssignment_0 ) )
            {
            // InternalLinguaFranca.g:2607:1: ( ( rule__Instance__NameAssignment_0 ) )
            // InternalLinguaFranca.g:2608:2: ( rule__Instance__NameAssignment_0 )
            {
             before(grammarAccess.getInstanceAccess().getNameAssignment_0()); 
            // InternalLinguaFranca.g:2609:2: ( rule__Instance__NameAssignment_0 )
            // InternalLinguaFranca.g:2609:3: rule__Instance__NameAssignment_0
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
    // InternalLinguaFranca.g:2617:1: rule__Instance__Group__1 : rule__Instance__Group__1__Impl rule__Instance__Group__2 ;
    public final void rule__Instance__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2621:1: ( rule__Instance__Group__1__Impl rule__Instance__Group__2 )
            // InternalLinguaFranca.g:2622:2: rule__Instance__Group__1__Impl rule__Instance__Group__2
            {
            pushFollow(FOLLOW_30);
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
    // InternalLinguaFranca.g:2629:1: rule__Instance__Group__1__Impl : ( '=' ) ;
    public final void rule__Instance__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2633:1: ( ( '=' ) )
            // InternalLinguaFranca.g:2634:1: ( '=' )
            {
            // InternalLinguaFranca.g:2634:1: ( '=' )
            // InternalLinguaFranca.g:2635:2: '='
            {
             before(grammarAccess.getInstanceAccess().getEqualsSignKeyword_1()); 
            match(input,33,FOLLOW_2); 
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
    // InternalLinguaFranca.g:2644:1: rule__Instance__Group__2 : rule__Instance__Group__2__Impl rule__Instance__Group__3 ;
    public final void rule__Instance__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2648:1: ( rule__Instance__Group__2__Impl rule__Instance__Group__3 )
            // InternalLinguaFranca.g:2649:2: rule__Instance__Group__2__Impl rule__Instance__Group__3
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
    // InternalLinguaFranca.g:2656:1: rule__Instance__Group__2__Impl : ( 'new' ) ;
    public final void rule__Instance__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2660:1: ( ( 'new' ) )
            // InternalLinguaFranca.g:2661:1: ( 'new' )
            {
            // InternalLinguaFranca.g:2661:1: ( 'new' )
            // InternalLinguaFranca.g:2662:2: 'new'
            {
             before(grammarAccess.getInstanceAccess().getNewKeyword_2()); 
            match(input,34,FOLLOW_2); 
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
    // InternalLinguaFranca.g:2671:1: rule__Instance__Group__3 : rule__Instance__Group__3__Impl rule__Instance__Group__4 ;
    public final void rule__Instance__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2675:1: ( rule__Instance__Group__3__Impl rule__Instance__Group__4 )
            // InternalLinguaFranca.g:2676:2: rule__Instance__Group__3__Impl rule__Instance__Group__4
            {
            pushFollow(FOLLOW_22);
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
    // InternalLinguaFranca.g:2683:1: rule__Instance__Group__3__Impl : ( ( rule__Instance__ActorClassAssignment_3 ) ) ;
    public final void rule__Instance__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2687:1: ( ( ( rule__Instance__ActorClassAssignment_3 ) ) )
            // InternalLinguaFranca.g:2688:1: ( ( rule__Instance__ActorClassAssignment_3 ) )
            {
            // InternalLinguaFranca.g:2688:1: ( ( rule__Instance__ActorClassAssignment_3 ) )
            // InternalLinguaFranca.g:2689:2: ( rule__Instance__ActorClassAssignment_3 )
            {
             before(grammarAccess.getInstanceAccess().getActorClassAssignment_3()); 
            // InternalLinguaFranca.g:2690:2: ( rule__Instance__ActorClassAssignment_3 )
            // InternalLinguaFranca.g:2690:3: rule__Instance__ActorClassAssignment_3
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
    // InternalLinguaFranca.g:2698:1: rule__Instance__Group__4 : rule__Instance__Group__4__Impl rule__Instance__Group__5 ;
    public final void rule__Instance__Group__4() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2702:1: ( rule__Instance__Group__4__Impl rule__Instance__Group__5 )
            // InternalLinguaFranca.g:2703:2: rule__Instance__Group__4__Impl rule__Instance__Group__5
            {
            pushFollow(FOLLOW_22);
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
    // InternalLinguaFranca.g:2710:1: rule__Instance__Group__4__Impl : ( ( rule__Instance__Group_4__0 )? ) ;
    public final void rule__Instance__Group__4__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2714:1: ( ( ( rule__Instance__Group_4__0 )? ) )
            // InternalLinguaFranca.g:2715:1: ( ( rule__Instance__Group_4__0 )? )
            {
            // InternalLinguaFranca.g:2715:1: ( ( rule__Instance__Group_4__0 )? )
            // InternalLinguaFranca.g:2716:2: ( rule__Instance__Group_4__0 )?
            {
             before(grammarAccess.getInstanceAccess().getGroup_4()); 
            // InternalLinguaFranca.g:2717:2: ( rule__Instance__Group_4__0 )?
            int alt32=2;
            int LA32_0 = input.LA(1);

            if ( (LA32_0==29) ) {
                alt32=1;
            }
            switch (alt32) {
                case 1 :
                    // InternalLinguaFranca.g:2717:3: rule__Instance__Group_4__0
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
    // InternalLinguaFranca.g:2725:1: rule__Instance__Group__5 : rule__Instance__Group__5__Impl ;
    public final void rule__Instance__Group__5() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2729:1: ( rule__Instance__Group__5__Impl )
            // InternalLinguaFranca.g:2730:2: rule__Instance__Group__5__Impl
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
    // InternalLinguaFranca.g:2736:1: rule__Instance__Group__5__Impl : ( ';' ) ;
    public final void rule__Instance__Group__5__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2740:1: ( ( ';' ) )
            // InternalLinguaFranca.g:2741:1: ( ';' )
            {
            // InternalLinguaFranca.g:2741:1: ( ';' )
            // InternalLinguaFranca.g:2742:2: ';'
            {
             before(grammarAccess.getInstanceAccess().getSemicolonKeyword_5()); 
            match(input,21,FOLLOW_2); 
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
    // InternalLinguaFranca.g:2752:1: rule__Instance__Group_4__0 : rule__Instance__Group_4__0__Impl rule__Instance__Group_4__1 ;
    public final void rule__Instance__Group_4__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2756:1: ( rule__Instance__Group_4__0__Impl rule__Instance__Group_4__1 )
            // InternalLinguaFranca.g:2757:2: rule__Instance__Group_4__0__Impl rule__Instance__Group_4__1
            {
            pushFollow(FOLLOW_25);
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
    // InternalLinguaFranca.g:2764:1: rule__Instance__Group_4__0__Impl : ( '(' ) ;
    public final void rule__Instance__Group_4__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2768:1: ( ( '(' ) )
            // InternalLinguaFranca.g:2769:1: ( '(' )
            {
            // InternalLinguaFranca.g:2769:1: ( '(' )
            // InternalLinguaFranca.g:2770:2: '('
            {
             before(grammarAccess.getInstanceAccess().getLeftParenthesisKeyword_4_0()); 
            match(input,29,FOLLOW_2); 
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
    // InternalLinguaFranca.g:2779:1: rule__Instance__Group_4__1 : rule__Instance__Group_4__1__Impl rule__Instance__Group_4__2 ;
    public final void rule__Instance__Group_4__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2783:1: ( rule__Instance__Group_4__1__Impl rule__Instance__Group_4__2 )
            // InternalLinguaFranca.g:2784:2: rule__Instance__Group_4__1__Impl rule__Instance__Group_4__2
            {
            pushFollow(FOLLOW_25);
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
    // InternalLinguaFranca.g:2791:1: rule__Instance__Group_4__1__Impl : ( ( rule__Instance__ParametersAssignment_4_1 )? ) ;
    public final void rule__Instance__Group_4__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2795:1: ( ( ( rule__Instance__ParametersAssignment_4_1 )? ) )
            // InternalLinguaFranca.g:2796:1: ( ( rule__Instance__ParametersAssignment_4_1 )? )
            {
            // InternalLinguaFranca.g:2796:1: ( ( rule__Instance__ParametersAssignment_4_1 )? )
            // InternalLinguaFranca.g:2797:2: ( rule__Instance__ParametersAssignment_4_1 )?
            {
             before(grammarAccess.getInstanceAccess().getParametersAssignment_4_1()); 
            // InternalLinguaFranca.g:2798:2: ( rule__Instance__ParametersAssignment_4_1 )?
            int alt33=2;
            int LA33_0 = input.LA(1);

            if ( (LA33_0==RULE_ID) ) {
                alt33=1;
            }
            switch (alt33) {
                case 1 :
                    // InternalLinguaFranca.g:2798:3: rule__Instance__ParametersAssignment_4_1
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
    // InternalLinguaFranca.g:2806:1: rule__Instance__Group_4__2 : rule__Instance__Group_4__2__Impl ;
    public final void rule__Instance__Group_4__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2810:1: ( rule__Instance__Group_4__2__Impl )
            // InternalLinguaFranca.g:2811:2: rule__Instance__Group_4__2__Impl
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
    // InternalLinguaFranca.g:2817:1: rule__Instance__Group_4__2__Impl : ( ')' ) ;
    public final void rule__Instance__Group_4__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2821:1: ( ( ')' ) )
            // InternalLinguaFranca.g:2822:1: ( ')' )
            {
            // InternalLinguaFranca.g:2822:1: ( ')' )
            // InternalLinguaFranca.g:2823:2: ')'
            {
             before(grammarAccess.getInstanceAccess().getRightParenthesisKeyword_4_2()); 
            match(input,30,FOLLOW_2); 
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
    // InternalLinguaFranca.g:2833:1: rule__Connection__Group__0 : rule__Connection__Group__0__Impl rule__Connection__Group__1 ;
    public final void rule__Connection__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2837:1: ( rule__Connection__Group__0__Impl rule__Connection__Group__1 )
            // InternalLinguaFranca.g:2838:2: rule__Connection__Group__0__Impl rule__Connection__Group__1
            {
            pushFollow(FOLLOW_31);
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
    // InternalLinguaFranca.g:2845:1: rule__Connection__Group__0__Impl : ( ( rule__Connection__LeftPortAssignment_0 ) ) ;
    public final void rule__Connection__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2849:1: ( ( ( rule__Connection__LeftPortAssignment_0 ) ) )
            // InternalLinguaFranca.g:2850:1: ( ( rule__Connection__LeftPortAssignment_0 ) )
            {
            // InternalLinguaFranca.g:2850:1: ( ( rule__Connection__LeftPortAssignment_0 ) )
            // InternalLinguaFranca.g:2851:2: ( rule__Connection__LeftPortAssignment_0 )
            {
             before(grammarAccess.getConnectionAccess().getLeftPortAssignment_0()); 
            // InternalLinguaFranca.g:2852:2: ( rule__Connection__LeftPortAssignment_0 )
            // InternalLinguaFranca.g:2852:3: rule__Connection__LeftPortAssignment_0
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
    // InternalLinguaFranca.g:2860:1: rule__Connection__Group__1 : rule__Connection__Group__1__Impl rule__Connection__Group__2 ;
    public final void rule__Connection__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2864:1: ( rule__Connection__Group__1__Impl rule__Connection__Group__2 )
            // InternalLinguaFranca.g:2865:2: rule__Connection__Group__1__Impl rule__Connection__Group__2
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
    // InternalLinguaFranca.g:2872:1: rule__Connection__Group__1__Impl : ( '->' ) ;
    public final void rule__Connection__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2876:1: ( ( '->' ) )
            // InternalLinguaFranca.g:2877:1: ( '->' )
            {
            // InternalLinguaFranca.g:2877:1: ( '->' )
            // InternalLinguaFranca.g:2878:2: '->'
            {
             before(grammarAccess.getConnectionAccess().getHyphenMinusGreaterThanSignKeyword_1()); 
            match(input,35,FOLLOW_2); 
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
    // InternalLinguaFranca.g:2887:1: rule__Connection__Group__2 : rule__Connection__Group__2__Impl rule__Connection__Group__3 ;
    public final void rule__Connection__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2891:1: ( rule__Connection__Group__2__Impl rule__Connection__Group__3 )
            // InternalLinguaFranca.g:2892:2: rule__Connection__Group__2__Impl rule__Connection__Group__3
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
    // InternalLinguaFranca.g:2899:1: rule__Connection__Group__2__Impl : ( ( rule__Connection__RightPortAssignment_2 ) ) ;
    public final void rule__Connection__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2903:1: ( ( ( rule__Connection__RightPortAssignment_2 ) ) )
            // InternalLinguaFranca.g:2904:1: ( ( rule__Connection__RightPortAssignment_2 ) )
            {
            // InternalLinguaFranca.g:2904:1: ( ( rule__Connection__RightPortAssignment_2 ) )
            // InternalLinguaFranca.g:2905:2: ( rule__Connection__RightPortAssignment_2 )
            {
             before(grammarAccess.getConnectionAccess().getRightPortAssignment_2()); 
            // InternalLinguaFranca.g:2906:2: ( rule__Connection__RightPortAssignment_2 )
            // InternalLinguaFranca.g:2906:3: rule__Connection__RightPortAssignment_2
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
    // InternalLinguaFranca.g:2914:1: rule__Connection__Group__3 : rule__Connection__Group__3__Impl ;
    public final void rule__Connection__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2918:1: ( rule__Connection__Group__3__Impl )
            // InternalLinguaFranca.g:2919:2: rule__Connection__Group__3__Impl
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
    // InternalLinguaFranca.g:2925:1: rule__Connection__Group__3__Impl : ( ';' ) ;
    public final void rule__Connection__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2929:1: ( ( ';' ) )
            // InternalLinguaFranca.g:2930:1: ( ';' )
            {
            // InternalLinguaFranca.g:2930:1: ( ';' )
            // InternalLinguaFranca.g:2931:2: ';'
            {
             before(grammarAccess.getConnectionAccess().getSemicolonKeyword_3()); 
            match(input,21,FOLLOW_2); 
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
    // InternalLinguaFranca.g:2941:1: rule__Assignments__Group__0 : rule__Assignments__Group__0__Impl rule__Assignments__Group__1 ;
    public final void rule__Assignments__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2945:1: ( rule__Assignments__Group__0__Impl rule__Assignments__Group__1 )
            // InternalLinguaFranca.g:2946:2: rule__Assignments__Group__0__Impl rule__Assignments__Group__1
            {
            pushFollow(FOLLOW_26);
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
    // InternalLinguaFranca.g:2953:1: rule__Assignments__Group__0__Impl : ( ( rule__Assignments__AssignmentsAssignment_0 ) ) ;
    public final void rule__Assignments__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2957:1: ( ( ( rule__Assignments__AssignmentsAssignment_0 ) ) )
            // InternalLinguaFranca.g:2958:1: ( ( rule__Assignments__AssignmentsAssignment_0 ) )
            {
            // InternalLinguaFranca.g:2958:1: ( ( rule__Assignments__AssignmentsAssignment_0 ) )
            // InternalLinguaFranca.g:2959:2: ( rule__Assignments__AssignmentsAssignment_0 )
            {
             before(grammarAccess.getAssignmentsAccess().getAssignmentsAssignment_0()); 
            // InternalLinguaFranca.g:2960:2: ( rule__Assignments__AssignmentsAssignment_0 )
            // InternalLinguaFranca.g:2960:3: rule__Assignments__AssignmentsAssignment_0
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
    // InternalLinguaFranca.g:2968:1: rule__Assignments__Group__1 : rule__Assignments__Group__1__Impl ;
    public final void rule__Assignments__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2972:1: ( rule__Assignments__Group__1__Impl )
            // InternalLinguaFranca.g:2973:2: rule__Assignments__Group__1__Impl
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
    // InternalLinguaFranca.g:2979:1: rule__Assignments__Group__1__Impl : ( ( rule__Assignments__Group_1__0 )* ) ;
    public final void rule__Assignments__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2983:1: ( ( ( rule__Assignments__Group_1__0 )* ) )
            // InternalLinguaFranca.g:2984:1: ( ( rule__Assignments__Group_1__0 )* )
            {
            // InternalLinguaFranca.g:2984:1: ( ( rule__Assignments__Group_1__0 )* )
            // InternalLinguaFranca.g:2985:2: ( rule__Assignments__Group_1__0 )*
            {
             before(grammarAccess.getAssignmentsAccess().getGroup_1()); 
            // InternalLinguaFranca.g:2986:2: ( rule__Assignments__Group_1__0 )*
            loop34:
            do {
                int alt34=2;
                int LA34_0 = input.LA(1);

                if ( (LA34_0==31) ) {
                    alt34=1;
                }


                switch (alt34) {
            	case 1 :
            	    // InternalLinguaFranca.g:2986:3: rule__Assignments__Group_1__0
            	    {
            	    pushFollow(FOLLOW_27);
            	    rule__Assignments__Group_1__0();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop34;
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
    // InternalLinguaFranca.g:2995:1: rule__Assignments__Group_1__0 : rule__Assignments__Group_1__0__Impl rule__Assignments__Group_1__1 ;
    public final void rule__Assignments__Group_1__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2999:1: ( rule__Assignments__Group_1__0__Impl rule__Assignments__Group_1__1 )
            // InternalLinguaFranca.g:3000:2: rule__Assignments__Group_1__0__Impl rule__Assignments__Group_1__1
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
    // InternalLinguaFranca.g:3007:1: rule__Assignments__Group_1__0__Impl : ( ',' ) ;
    public final void rule__Assignments__Group_1__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3011:1: ( ( ',' ) )
            // InternalLinguaFranca.g:3012:1: ( ',' )
            {
            // InternalLinguaFranca.g:3012:1: ( ',' )
            // InternalLinguaFranca.g:3013:2: ','
            {
             before(grammarAccess.getAssignmentsAccess().getCommaKeyword_1_0()); 
            match(input,31,FOLLOW_2); 
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
    // InternalLinguaFranca.g:3022:1: rule__Assignments__Group_1__1 : rule__Assignments__Group_1__1__Impl ;
    public final void rule__Assignments__Group_1__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3026:1: ( rule__Assignments__Group_1__1__Impl )
            // InternalLinguaFranca.g:3027:2: rule__Assignments__Group_1__1__Impl
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
    // InternalLinguaFranca.g:3033:1: rule__Assignments__Group_1__1__Impl : ( ( rule__Assignments__AssignmentsAssignment_1_1 ) ) ;
    public final void rule__Assignments__Group_1__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3037:1: ( ( ( rule__Assignments__AssignmentsAssignment_1_1 ) ) )
            // InternalLinguaFranca.g:3038:1: ( ( rule__Assignments__AssignmentsAssignment_1_1 ) )
            {
            // InternalLinguaFranca.g:3038:1: ( ( rule__Assignments__AssignmentsAssignment_1_1 ) )
            // InternalLinguaFranca.g:3039:2: ( rule__Assignments__AssignmentsAssignment_1_1 )
            {
             before(grammarAccess.getAssignmentsAccess().getAssignmentsAssignment_1_1()); 
            // InternalLinguaFranca.g:3040:2: ( rule__Assignments__AssignmentsAssignment_1_1 )
            // InternalLinguaFranca.g:3040:3: rule__Assignments__AssignmentsAssignment_1_1
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
    // InternalLinguaFranca.g:3049:1: rule__Assignment__Group__0 : rule__Assignment__Group__0__Impl rule__Assignment__Group__1 ;
    public final void rule__Assignment__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3053:1: ( rule__Assignment__Group__0__Impl rule__Assignment__Group__1 )
            // InternalLinguaFranca.g:3054:2: rule__Assignment__Group__0__Impl rule__Assignment__Group__1
            {
            pushFollow(FOLLOW_29);
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
    // InternalLinguaFranca.g:3061:1: rule__Assignment__Group__0__Impl : ( ( rule__Assignment__NameAssignment_0 ) ) ;
    public final void rule__Assignment__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3065:1: ( ( ( rule__Assignment__NameAssignment_0 ) ) )
            // InternalLinguaFranca.g:3066:1: ( ( rule__Assignment__NameAssignment_0 ) )
            {
            // InternalLinguaFranca.g:3066:1: ( ( rule__Assignment__NameAssignment_0 ) )
            // InternalLinguaFranca.g:3067:2: ( rule__Assignment__NameAssignment_0 )
            {
             before(grammarAccess.getAssignmentAccess().getNameAssignment_0()); 
            // InternalLinguaFranca.g:3068:2: ( rule__Assignment__NameAssignment_0 )
            // InternalLinguaFranca.g:3068:3: rule__Assignment__NameAssignment_0
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
    // InternalLinguaFranca.g:3076:1: rule__Assignment__Group__1 : rule__Assignment__Group__1__Impl rule__Assignment__Group__2 ;
    public final void rule__Assignment__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3080:1: ( rule__Assignment__Group__1__Impl rule__Assignment__Group__2 )
            // InternalLinguaFranca.g:3081:2: rule__Assignment__Group__1__Impl rule__Assignment__Group__2
            {
            pushFollow(FOLLOW_32);
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
    // InternalLinguaFranca.g:3088:1: rule__Assignment__Group__1__Impl : ( '=' ) ;
    public final void rule__Assignment__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3092:1: ( ( '=' ) )
            // InternalLinguaFranca.g:3093:1: ( '=' )
            {
            // InternalLinguaFranca.g:3093:1: ( '=' )
            // InternalLinguaFranca.g:3094:2: '='
            {
             before(grammarAccess.getAssignmentAccess().getEqualsSignKeyword_1()); 
            match(input,33,FOLLOW_2); 
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
    // InternalLinguaFranca.g:3103:1: rule__Assignment__Group__2 : rule__Assignment__Group__2__Impl ;
    public final void rule__Assignment__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3107:1: ( rule__Assignment__Group__2__Impl )
            // InternalLinguaFranca.g:3108:2: rule__Assignment__Group__2__Impl
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
    // InternalLinguaFranca.g:3114:1: rule__Assignment__Group__2__Impl : ( ( rule__Assignment__ValueAssignment_2 ) ) ;
    public final void rule__Assignment__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3118:1: ( ( ( rule__Assignment__ValueAssignment_2 ) ) )
            // InternalLinguaFranca.g:3119:1: ( ( rule__Assignment__ValueAssignment_2 ) )
            {
            // InternalLinguaFranca.g:3119:1: ( ( rule__Assignment__ValueAssignment_2 ) )
            // InternalLinguaFranca.g:3120:2: ( rule__Assignment__ValueAssignment_2 )
            {
             before(grammarAccess.getAssignmentAccess().getValueAssignment_2()); 
            // InternalLinguaFranca.g:3121:2: ( rule__Assignment__ValueAssignment_2 )
            // InternalLinguaFranca.g:3121:3: rule__Assignment__ValueAssignment_2
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
    // InternalLinguaFranca.g:3130:1: rule__Gets__Group__0 : rule__Gets__Group__0__Impl rule__Gets__Group__1 ;
    public final void rule__Gets__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3134:1: ( rule__Gets__Group__0__Impl rule__Gets__Group__1 )
            // InternalLinguaFranca.g:3135:2: rule__Gets__Group__0__Impl rule__Gets__Group__1
            {
            pushFollow(FOLLOW_26);
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
    // InternalLinguaFranca.g:3142:1: rule__Gets__Group__0__Impl : ( ( rule__Gets__GetsAssignment_0 ) ) ;
    public final void rule__Gets__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3146:1: ( ( ( rule__Gets__GetsAssignment_0 ) ) )
            // InternalLinguaFranca.g:3147:1: ( ( rule__Gets__GetsAssignment_0 ) )
            {
            // InternalLinguaFranca.g:3147:1: ( ( rule__Gets__GetsAssignment_0 ) )
            // InternalLinguaFranca.g:3148:2: ( rule__Gets__GetsAssignment_0 )
            {
             before(grammarAccess.getGetsAccess().getGetsAssignment_0()); 
            // InternalLinguaFranca.g:3149:2: ( rule__Gets__GetsAssignment_0 )
            // InternalLinguaFranca.g:3149:3: rule__Gets__GetsAssignment_0
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
    // InternalLinguaFranca.g:3157:1: rule__Gets__Group__1 : rule__Gets__Group__1__Impl ;
    public final void rule__Gets__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3161:1: ( rule__Gets__Group__1__Impl )
            // InternalLinguaFranca.g:3162:2: rule__Gets__Group__1__Impl
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
    // InternalLinguaFranca.g:3168:1: rule__Gets__Group__1__Impl : ( ( rule__Gets__Group_1__0 )? ) ;
    public final void rule__Gets__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3172:1: ( ( ( rule__Gets__Group_1__0 )? ) )
            // InternalLinguaFranca.g:3173:1: ( ( rule__Gets__Group_1__0 )? )
            {
            // InternalLinguaFranca.g:3173:1: ( ( rule__Gets__Group_1__0 )? )
            // InternalLinguaFranca.g:3174:2: ( rule__Gets__Group_1__0 )?
            {
             before(grammarAccess.getGetsAccess().getGroup_1()); 
            // InternalLinguaFranca.g:3175:2: ( rule__Gets__Group_1__0 )?
            int alt35=2;
            int LA35_0 = input.LA(1);

            if ( (LA35_0==31) ) {
                alt35=1;
            }
            switch (alt35) {
                case 1 :
                    // InternalLinguaFranca.g:3175:3: rule__Gets__Group_1__0
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
    // InternalLinguaFranca.g:3184:1: rule__Gets__Group_1__0 : rule__Gets__Group_1__0__Impl rule__Gets__Group_1__1 ;
    public final void rule__Gets__Group_1__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3188:1: ( rule__Gets__Group_1__0__Impl rule__Gets__Group_1__1 )
            // InternalLinguaFranca.g:3189:2: rule__Gets__Group_1__0__Impl rule__Gets__Group_1__1
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
    // InternalLinguaFranca.g:3196:1: rule__Gets__Group_1__0__Impl : ( ',' ) ;
    public final void rule__Gets__Group_1__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3200:1: ( ( ',' ) )
            // InternalLinguaFranca.g:3201:1: ( ',' )
            {
            // InternalLinguaFranca.g:3201:1: ( ',' )
            // InternalLinguaFranca.g:3202:2: ','
            {
             before(grammarAccess.getGetsAccess().getCommaKeyword_1_0()); 
            match(input,31,FOLLOW_2); 
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
    // InternalLinguaFranca.g:3211:1: rule__Gets__Group_1__1 : rule__Gets__Group_1__1__Impl ;
    public final void rule__Gets__Group_1__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3215:1: ( rule__Gets__Group_1__1__Impl )
            // InternalLinguaFranca.g:3216:2: rule__Gets__Group_1__1__Impl
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
    // InternalLinguaFranca.g:3222:1: rule__Gets__Group_1__1__Impl : ( ( rule__Gets__GetsAssignment_1_1 ) ) ;
    public final void rule__Gets__Group_1__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3226:1: ( ( ( rule__Gets__GetsAssignment_1_1 ) ) )
            // InternalLinguaFranca.g:3227:1: ( ( rule__Gets__GetsAssignment_1_1 ) )
            {
            // InternalLinguaFranca.g:3227:1: ( ( rule__Gets__GetsAssignment_1_1 ) )
            // InternalLinguaFranca.g:3228:2: ( rule__Gets__GetsAssignment_1_1 )
            {
             before(grammarAccess.getGetsAccess().getGetsAssignment_1_1()); 
            // InternalLinguaFranca.g:3229:2: ( rule__Gets__GetsAssignment_1_1 )
            // InternalLinguaFranca.g:3229:3: rule__Gets__GetsAssignment_1_1
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
    // InternalLinguaFranca.g:3238:1: rule__Params__Group__0 : rule__Params__Group__0__Impl rule__Params__Group__1 ;
    public final void rule__Params__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3242:1: ( rule__Params__Group__0__Impl rule__Params__Group__1 )
            // InternalLinguaFranca.g:3243:2: rule__Params__Group__0__Impl rule__Params__Group__1
            {
            pushFollow(FOLLOW_33);
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
    // InternalLinguaFranca.g:3250:1: rule__Params__Group__0__Impl : ( '(' ) ;
    public final void rule__Params__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3254:1: ( ( '(' ) )
            // InternalLinguaFranca.g:3255:1: ( '(' )
            {
            // InternalLinguaFranca.g:3255:1: ( '(' )
            // InternalLinguaFranca.g:3256:2: '('
            {
             before(grammarAccess.getParamsAccess().getLeftParenthesisKeyword_0()); 
            match(input,29,FOLLOW_2); 
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
    // InternalLinguaFranca.g:3265:1: rule__Params__Group__1 : rule__Params__Group__1__Impl rule__Params__Group__2 ;
    public final void rule__Params__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3269:1: ( rule__Params__Group__1__Impl rule__Params__Group__2 )
            // InternalLinguaFranca.g:3270:2: rule__Params__Group__1__Impl rule__Params__Group__2
            {
            pushFollow(FOLLOW_34);
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
    // InternalLinguaFranca.g:3277:1: rule__Params__Group__1__Impl : ( ( rule__Params__ParamsAssignment_1 ) ) ;
    public final void rule__Params__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3281:1: ( ( ( rule__Params__ParamsAssignment_1 ) ) )
            // InternalLinguaFranca.g:3282:1: ( ( rule__Params__ParamsAssignment_1 ) )
            {
            // InternalLinguaFranca.g:3282:1: ( ( rule__Params__ParamsAssignment_1 ) )
            // InternalLinguaFranca.g:3283:2: ( rule__Params__ParamsAssignment_1 )
            {
             before(grammarAccess.getParamsAccess().getParamsAssignment_1()); 
            // InternalLinguaFranca.g:3284:2: ( rule__Params__ParamsAssignment_1 )
            // InternalLinguaFranca.g:3284:3: rule__Params__ParamsAssignment_1
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
    // InternalLinguaFranca.g:3292:1: rule__Params__Group__2 : rule__Params__Group__2__Impl rule__Params__Group__3 ;
    public final void rule__Params__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3296:1: ( rule__Params__Group__2__Impl rule__Params__Group__3 )
            // InternalLinguaFranca.g:3297:2: rule__Params__Group__2__Impl rule__Params__Group__3
            {
            pushFollow(FOLLOW_34);
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
    // InternalLinguaFranca.g:3304:1: rule__Params__Group__2__Impl : ( ( rule__Params__Group_2__0 )* ) ;
    public final void rule__Params__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3308:1: ( ( ( rule__Params__Group_2__0 )* ) )
            // InternalLinguaFranca.g:3309:1: ( ( rule__Params__Group_2__0 )* )
            {
            // InternalLinguaFranca.g:3309:1: ( ( rule__Params__Group_2__0 )* )
            // InternalLinguaFranca.g:3310:2: ( rule__Params__Group_2__0 )*
            {
             before(grammarAccess.getParamsAccess().getGroup_2()); 
            // InternalLinguaFranca.g:3311:2: ( rule__Params__Group_2__0 )*
            loop36:
            do {
                int alt36=2;
                int LA36_0 = input.LA(1);

                if ( (LA36_0==31) ) {
                    alt36=1;
                }


                switch (alt36) {
            	case 1 :
            	    // InternalLinguaFranca.g:3311:3: rule__Params__Group_2__0
            	    {
            	    pushFollow(FOLLOW_27);
            	    rule__Params__Group_2__0();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop36;
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
    // InternalLinguaFranca.g:3319:1: rule__Params__Group__3 : rule__Params__Group__3__Impl ;
    public final void rule__Params__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3323:1: ( rule__Params__Group__3__Impl )
            // InternalLinguaFranca.g:3324:2: rule__Params__Group__3__Impl
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
    // InternalLinguaFranca.g:3330:1: rule__Params__Group__3__Impl : ( ')' ) ;
    public final void rule__Params__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3334:1: ( ( ')' ) )
            // InternalLinguaFranca.g:3335:1: ( ')' )
            {
            // InternalLinguaFranca.g:3335:1: ( ')' )
            // InternalLinguaFranca.g:3336:2: ')'
            {
             before(grammarAccess.getParamsAccess().getRightParenthesisKeyword_3()); 
            match(input,30,FOLLOW_2); 
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
    // InternalLinguaFranca.g:3346:1: rule__Params__Group_2__0 : rule__Params__Group_2__0__Impl rule__Params__Group_2__1 ;
    public final void rule__Params__Group_2__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3350:1: ( rule__Params__Group_2__0__Impl rule__Params__Group_2__1 )
            // InternalLinguaFranca.g:3351:2: rule__Params__Group_2__0__Impl rule__Params__Group_2__1
            {
            pushFollow(FOLLOW_33);
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
    // InternalLinguaFranca.g:3358:1: rule__Params__Group_2__0__Impl : ( ',' ) ;
    public final void rule__Params__Group_2__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3362:1: ( ( ',' ) )
            // InternalLinguaFranca.g:3363:1: ( ',' )
            {
            // InternalLinguaFranca.g:3363:1: ( ',' )
            // InternalLinguaFranca.g:3364:2: ','
            {
             before(grammarAccess.getParamsAccess().getCommaKeyword_2_0()); 
            match(input,31,FOLLOW_2); 
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
    // InternalLinguaFranca.g:3373:1: rule__Params__Group_2__1 : rule__Params__Group_2__1__Impl ;
    public final void rule__Params__Group_2__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3377:1: ( rule__Params__Group_2__1__Impl )
            // InternalLinguaFranca.g:3378:2: rule__Params__Group_2__1__Impl
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
    // InternalLinguaFranca.g:3384:1: rule__Params__Group_2__1__Impl : ( ( rule__Params__ParamsAssignment_2_1 ) ) ;
    public final void rule__Params__Group_2__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3388:1: ( ( ( rule__Params__ParamsAssignment_2_1 ) ) )
            // InternalLinguaFranca.g:3389:1: ( ( rule__Params__ParamsAssignment_2_1 ) )
            {
            // InternalLinguaFranca.g:3389:1: ( ( rule__Params__ParamsAssignment_2_1 ) )
            // InternalLinguaFranca.g:3390:2: ( rule__Params__ParamsAssignment_2_1 )
            {
             before(grammarAccess.getParamsAccess().getParamsAssignment_2_1()); 
            // InternalLinguaFranca.g:3391:2: ( rule__Params__ParamsAssignment_2_1 )
            // InternalLinguaFranca.g:3391:3: rule__Params__ParamsAssignment_2_1
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
    // InternalLinguaFranca.g:3400:1: rule__Param__Group__0 : rule__Param__Group__0__Impl rule__Param__Group__1 ;
    public final void rule__Param__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3404:1: ( rule__Param__Group__0__Impl rule__Param__Group__1 )
            // InternalLinguaFranca.g:3405:2: rule__Param__Group__0__Impl rule__Param__Group__1
            {
            pushFollow(FOLLOW_33);
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
    // InternalLinguaFranca.g:3412:1: rule__Param__Group__0__Impl : ( ( 'const' )? ) ;
    public final void rule__Param__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3416:1: ( ( ( 'const' )? ) )
            // InternalLinguaFranca.g:3417:1: ( ( 'const' )? )
            {
            // InternalLinguaFranca.g:3417:1: ( ( 'const' )? )
            // InternalLinguaFranca.g:3418:2: ( 'const' )?
            {
             before(grammarAccess.getParamAccess().getConstKeyword_0()); 
            // InternalLinguaFranca.g:3419:2: ( 'const' )?
            int alt37=2;
            int LA37_0 = input.LA(1);

            if ( (LA37_0==36) ) {
                alt37=1;
            }
            switch (alt37) {
                case 1 :
                    // InternalLinguaFranca.g:3419:3: 'const'
                    {
                    match(input,36,FOLLOW_2); 

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
    // InternalLinguaFranca.g:3427:1: rule__Param__Group__1 : rule__Param__Group__1__Impl rule__Param__Group__2 ;
    public final void rule__Param__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3431:1: ( rule__Param__Group__1__Impl rule__Param__Group__2 )
            // InternalLinguaFranca.g:3432:2: rule__Param__Group__1__Impl rule__Param__Group__2
            {
            pushFollow(FOLLOW_35);
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
    // InternalLinguaFranca.g:3439:1: rule__Param__Group__1__Impl : ( ( rule__Param__NameAssignment_1 ) ) ;
    public final void rule__Param__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3443:1: ( ( ( rule__Param__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:3444:1: ( ( rule__Param__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:3444:1: ( ( rule__Param__NameAssignment_1 ) )
            // InternalLinguaFranca.g:3445:2: ( rule__Param__NameAssignment_1 )
            {
             before(grammarAccess.getParamAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:3446:2: ( rule__Param__NameAssignment_1 )
            // InternalLinguaFranca.g:3446:3: rule__Param__NameAssignment_1
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
    // InternalLinguaFranca.g:3454:1: rule__Param__Group__2 : rule__Param__Group__2__Impl rule__Param__Group__3 ;
    public final void rule__Param__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3458:1: ( rule__Param__Group__2__Impl rule__Param__Group__3 )
            // InternalLinguaFranca.g:3459:2: rule__Param__Group__2__Impl rule__Param__Group__3
            {
            pushFollow(FOLLOW_35);
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
    // InternalLinguaFranca.g:3466:1: rule__Param__Group__2__Impl : ( ( rule__Param__Group_2__0 )? ) ;
    public final void rule__Param__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3470:1: ( ( ( rule__Param__Group_2__0 )? ) )
            // InternalLinguaFranca.g:3471:1: ( ( rule__Param__Group_2__0 )? )
            {
            // InternalLinguaFranca.g:3471:1: ( ( rule__Param__Group_2__0 )? )
            // InternalLinguaFranca.g:3472:2: ( rule__Param__Group_2__0 )?
            {
             before(grammarAccess.getParamAccess().getGroup_2()); 
            // InternalLinguaFranca.g:3473:2: ( rule__Param__Group_2__0 )?
            int alt38=2;
            int LA38_0 = input.LA(1);

            if ( (LA38_0==27) ) {
                alt38=1;
            }
            switch (alt38) {
                case 1 :
                    // InternalLinguaFranca.g:3473:3: rule__Param__Group_2__0
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
    // InternalLinguaFranca.g:3481:1: rule__Param__Group__3 : rule__Param__Group__3__Impl ;
    public final void rule__Param__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3485:1: ( rule__Param__Group__3__Impl )
            // InternalLinguaFranca.g:3486:2: rule__Param__Group__3__Impl
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
    // InternalLinguaFranca.g:3492:1: rule__Param__Group__3__Impl : ( ( rule__Param__Group_3__0 )? ) ;
    public final void rule__Param__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3496:1: ( ( ( rule__Param__Group_3__0 )? ) )
            // InternalLinguaFranca.g:3497:1: ( ( rule__Param__Group_3__0 )? )
            {
            // InternalLinguaFranca.g:3497:1: ( ( rule__Param__Group_3__0 )? )
            // InternalLinguaFranca.g:3498:2: ( rule__Param__Group_3__0 )?
            {
             before(grammarAccess.getParamAccess().getGroup_3()); 
            // InternalLinguaFranca.g:3499:2: ( rule__Param__Group_3__0 )?
            int alt39=2;
            int LA39_0 = input.LA(1);

            if ( (LA39_0==29) ) {
                alt39=1;
            }
            switch (alt39) {
                case 1 :
                    // InternalLinguaFranca.g:3499:3: rule__Param__Group_3__0
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
    // InternalLinguaFranca.g:3508:1: rule__Param__Group_2__0 : rule__Param__Group_2__0__Impl rule__Param__Group_2__1 ;
    public final void rule__Param__Group_2__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3512:1: ( rule__Param__Group_2__0__Impl rule__Param__Group_2__1 )
            // InternalLinguaFranca.g:3513:2: rule__Param__Group_2__0__Impl rule__Param__Group_2__1
            {
            pushFollow(FOLLOW_19);
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
    // InternalLinguaFranca.g:3520:1: rule__Param__Group_2__0__Impl : ( ':' ) ;
    public final void rule__Param__Group_2__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3524:1: ( ( ':' ) )
            // InternalLinguaFranca.g:3525:1: ( ':' )
            {
            // InternalLinguaFranca.g:3525:1: ( ':' )
            // InternalLinguaFranca.g:3526:2: ':'
            {
             before(grammarAccess.getParamAccess().getColonKeyword_2_0()); 
            match(input,27,FOLLOW_2); 
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
    // InternalLinguaFranca.g:3535:1: rule__Param__Group_2__1 : rule__Param__Group_2__1__Impl ;
    public final void rule__Param__Group_2__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3539:1: ( rule__Param__Group_2__1__Impl )
            // InternalLinguaFranca.g:3540:2: rule__Param__Group_2__1__Impl
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
    // InternalLinguaFranca.g:3546:1: rule__Param__Group_2__1__Impl : ( ( rule__Param__TypeAssignment_2_1 ) ) ;
    public final void rule__Param__Group_2__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3550:1: ( ( ( rule__Param__TypeAssignment_2_1 ) ) )
            // InternalLinguaFranca.g:3551:1: ( ( rule__Param__TypeAssignment_2_1 ) )
            {
            // InternalLinguaFranca.g:3551:1: ( ( rule__Param__TypeAssignment_2_1 ) )
            // InternalLinguaFranca.g:3552:2: ( rule__Param__TypeAssignment_2_1 )
            {
             before(grammarAccess.getParamAccess().getTypeAssignment_2_1()); 
            // InternalLinguaFranca.g:3553:2: ( rule__Param__TypeAssignment_2_1 )
            // InternalLinguaFranca.g:3553:3: rule__Param__TypeAssignment_2_1
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
    // InternalLinguaFranca.g:3562:1: rule__Param__Group_3__0 : rule__Param__Group_3__0__Impl rule__Param__Group_3__1 ;
    public final void rule__Param__Group_3__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3566:1: ( rule__Param__Group_3__0__Impl rule__Param__Group_3__1 )
            // InternalLinguaFranca.g:3567:2: rule__Param__Group_3__0__Impl rule__Param__Group_3__1
            {
            pushFollow(FOLLOW_32);
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
    // InternalLinguaFranca.g:3574:1: rule__Param__Group_3__0__Impl : ( '(' ) ;
    public final void rule__Param__Group_3__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3578:1: ( ( '(' ) )
            // InternalLinguaFranca.g:3579:1: ( '(' )
            {
            // InternalLinguaFranca.g:3579:1: ( '(' )
            // InternalLinguaFranca.g:3580:2: '('
            {
             before(grammarAccess.getParamAccess().getLeftParenthesisKeyword_3_0()); 
            match(input,29,FOLLOW_2); 
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
    // InternalLinguaFranca.g:3589:1: rule__Param__Group_3__1 : rule__Param__Group_3__1__Impl rule__Param__Group_3__2 ;
    public final void rule__Param__Group_3__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3593:1: ( rule__Param__Group_3__1__Impl rule__Param__Group_3__2 )
            // InternalLinguaFranca.g:3594:2: rule__Param__Group_3__1__Impl rule__Param__Group_3__2
            {
            pushFollow(FOLLOW_36);
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
    // InternalLinguaFranca.g:3601:1: rule__Param__Group_3__1__Impl : ( ( rule__Param__ValueAssignment_3_1 ) ) ;
    public final void rule__Param__Group_3__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3605:1: ( ( ( rule__Param__ValueAssignment_3_1 ) ) )
            // InternalLinguaFranca.g:3606:1: ( ( rule__Param__ValueAssignment_3_1 ) )
            {
            // InternalLinguaFranca.g:3606:1: ( ( rule__Param__ValueAssignment_3_1 ) )
            // InternalLinguaFranca.g:3607:2: ( rule__Param__ValueAssignment_3_1 )
            {
             before(grammarAccess.getParamAccess().getValueAssignment_3_1()); 
            // InternalLinguaFranca.g:3608:2: ( rule__Param__ValueAssignment_3_1 )
            // InternalLinguaFranca.g:3608:3: rule__Param__ValueAssignment_3_1
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
    // InternalLinguaFranca.g:3616:1: rule__Param__Group_3__2 : rule__Param__Group_3__2__Impl ;
    public final void rule__Param__Group_3__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3620:1: ( rule__Param__Group_3__2__Impl )
            // InternalLinguaFranca.g:3621:2: rule__Param__Group_3__2__Impl
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
    // InternalLinguaFranca.g:3627:1: rule__Param__Group_3__2__Impl : ( ')' ) ;
    public final void rule__Param__Group_3__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3631:1: ( ( ')' ) )
            // InternalLinguaFranca.g:3632:1: ( ')' )
            {
            // InternalLinguaFranca.g:3632:1: ( ')' )
            // InternalLinguaFranca.g:3633:2: ')'
            {
             before(grammarAccess.getParamAccess().getRightParenthesisKeyword_3_2()); 
            match(input,30,FOLLOW_2); 
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


    // $ANTLR start "rule__Timing__Group__0"
    // InternalLinguaFranca.g:3643:1: rule__Timing__Group__0 : rule__Timing__Group__0__Impl rule__Timing__Group__1 ;
    public final void rule__Timing__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3647:1: ( rule__Timing__Group__0__Impl rule__Timing__Group__1 )
            // InternalLinguaFranca.g:3648:2: rule__Timing__Group__0__Impl rule__Timing__Group__1
            {
            pushFollow(FOLLOW_37);
            rule__Timing__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Timing__Group__1();

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
    // $ANTLR end "rule__Timing__Group__0"


    // $ANTLR start "rule__Timing__Group__0__Impl"
    // InternalLinguaFranca.g:3655:1: rule__Timing__Group__0__Impl : ( '(' ) ;
    public final void rule__Timing__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3659:1: ( ( '(' ) )
            // InternalLinguaFranca.g:3660:1: ( '(' )
            {
            // InternalLinguaFranca.g:3660:1: ( '(' )
            // InternalLinguaFranca.g:3661:2: '('
            {
             before(grammarAccess.getTimingAccess().getLeftParenthesisKeyword_0()); 
            match(input,29,FOLLOW_2); 
             after(grammarAccess.getTimingAccess().getLeftParenthesisKeyword_0()); 

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
    // $ANTLR end "rule__Timing__Group__0__Impl"


    // $ANTLR start "rule__Timing__Group__1"
    // InternalLinguaFranca.g:3670:1: rule__Timing__Group__1 : rule__Timing__Group__1__Impl rule__Timing__Group__2 ;
    public final void rule__Timing__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3674:1: ( rule__Timing__Group__1__Impl rule__Timing__Group__2 )
            // InternalLinguaFranca.g:3675:2: rule__Timing__Group__1__Impl rule__Timing__Group__2
            {
            pushFollow(FOLLOW_34);
            rule__Timing__Group__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Timing__Group__2();

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
    // $ANTLR end "rule__Timing__Group__1"


    // $ANTLR start "rule__Timing__Group__1__Impl"
    // InternalLinguaFranca.g:3682:1: rule__Timing__Group__1__Impl : ( ( rule__Timing__OffsetAssignment_1 ) ) ;
    public final void rule__Timing__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3686:1: ( ( ( rule__Timing__OffsetAssignment_1 ) ) )
            // InternalLinguaFranca.g:3687:1: ( ( rule__Timing__OffsetAssignment_1 ) )
            {
            // InternalLinguaFranca.g:3687:1: ( ( rule__Timing__OffsetAssignment_1 ) )
            // InternalLinguaFranca.g:3688:2: ( rule__Timing__OffsetAssignment_1 )
            {
             before(grammarAccess.getTimingAccess().getOffsetAssignment_1()); 
            // InternalLinguaFranca.g:3689:2: ( rule__Timing__OffsetAssignment_1 )
            // InternalLinguaFranca.g:3689:3: rule__Timing__OffsetAssignment_1
            {
            pushFollow(FOLLOW_2);
            rule__Timing__OffsetAssignment_1();

            state._fsp--;


            }

             after(grammarAccess.getTimingAccess().getOffsetAssignment_1()); 

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
    // $ANTLR end "rule__Timing__Group__1__Impl"


    // $ANTLR start "rule__Timing__Group__2"
    // InternalLinguaFranca.g:3697:1: rule__Timing__Group__2 : rule__Timing__Group__2__Impl rule__Timing__Group__3 ;
    public final void rule__Timing__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3701:1: ( rule__Timing__Group__2__Impl rule__Timing__Group__3 )
            // InternalLinguaFranca.g:3702:2: rule__Timing__Group__2__Impl rule__Timing__Group__3
            {
            pushFollow(FOLLOW_34);
            rule__Timing__Group__2__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Timing__Group__3();

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
    // $ANTLR end "rule__Timing__Group__2"


    // $ANTLR start "rule__Timing__Group__2__Impl"
    // InternalLinguaFranca.g:3709:1: rule__Timing__Group__2__Impl : ( ( rule__Timing__Group_2__0 )? ) ;
    public final void rule__Timing__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3713:1: ( ( ( rule__Timing__Group_2__0 )? ) )
            // InternalLinguaFranca.g:3714:1: ( ( rule__Timing__Group_2__0 )? )
            {
            // InternalLinguaFranca.g:3714:1: ( ( rule__Timing__Group_2__0 )? )
            // InternalLinguaFranca.g:3715:2: ( rule__Timing__Group_2__0 )?
            {
             before(grammarAccess.getTimingAccess().getGroup_2()); 
            // InternalLinguaFranca.g:3716:2: ( rule__Timing__Group_2__0 )?
            int alt40=2;
            int LA40_0 = input.LA(1);

            if ( (LA40_0==31) ) {
                alt40=1;
            }
            switch (alt40) {
                case 1 :
                    // InternalLinguaFranca.g:3716:3: rule__Timing__Group_2__0
                    {
                    pushFollow(FOLLOW_2);
                    rule__Timing__Group_2__0();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getTimingAccess().getGroup_2()); 

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
    // $ANTLR end "rule__Timing__Group__2__Impl"


    // $ANTLR start "rule__Timing__Group__3"
    // InternalLinguaFranca.g:3724:1: rule__Timing__Group__3 : rule__Timing__Group__3__Impl ;
    public final void rule__Timing__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3728:1: ( rule__Timing__Group__3__Impl )
            // InternalLinguaFranca.g:3729:2: rule__Timing__Group__3__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Timing__Group__3__Impl();

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
    // $ANTLR end "rule__Timing__Group__3"


    // $ANTLR start "rule__Timing__Group__3__Impl"
    // InternalLinguaFranca.g:3735:1: rule__Timing__Group__3__Impl : ( ')' ) ;
    public final void rule__Timing__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3739:1: ( ( ')' ) )
            // InternalLinguaFranca.g:3740:1: ( ')' )
            {
            // InternalLinguaFranca.g:3740:1: ( ')' )
            // InternalLinguaFranca.g:3741:2: ')'
            {
             before(grammarAccess.getTimingAccess().getRightParenthesisKeyword_3()); 
            match(input,30,FOLLOW_2); 
             after(grammarAccess.getTimingAccess().getRightParenthesisKeyword_3()); 

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
    // $ANTLR end "rule__Timing__Group__3__Impl"


    // $ANTLR start "rule__Timing__Group_2__0"
    // InternalLinguaFranca.g:3751:1: rule__Timing__Group_2__0 : rule__Timing__Group_2__0__Impl rule__Timing__Group_2__1 ;
    public final void rule__Timing__Group_2__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3755:1: ( rule__Timing__Group_2__0__Impl rule__Timing__Group_2__1 )
            // InternalLinguaFranca.g:3756:2: rule__Timing__Group_2__0__Impl rule__Timing__Group_2__1
            {
            pushFollow(FOLLOW_38);
            rule__Timing__Group_2__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Timing__Group_2__1();

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
    // $ANTLR end "rule__Timing__Group_2__0"


    // $ANTLR start "rule__Timing__Group_2__0__Impl"
    // InternalLinguaFranca.g:3763:1: rule__Timing__Group_2__0__Impl : ( ',' ) ;
    public final void rule__Timing__Group_2__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3767:1: ( ( ',' ) )
            // InternalLinguaFranca.g:3768:1: ( ',' )
            {
            // InternalLinguaFranca.g:3768:1: ( ',' )
            // InternalLinguaFranca.g:3769:2: ','
            {
             before(grammarAccess.getTimingAccess().getCommaKeyword_2_0()); 
            match(input,31,FOLLOW_2); 
             after(grammarAccess.getTimingAccess().getCommaKeyword_2_0()); 

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
    // $ANTLR end "rule__Timing__Group_2__0__Impl"


    // $ANTLR start "rule__Timing__Group_2__1"
    // InternalLinguaFranca.g:3778:1: rule__Timing__Group_2__1 : rule__Timing__Group_2__1__Impl ;
    public final void rule__Timing__Group_2__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3782:1: ( rule__Timing__Group_2__1__Impl )
            // InternalLinguaFranca.g:3783:2: rule__Timing__Group_2__1__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Timing__Group_2__1__Impl();

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
    // $ANTLR end "rule__Timing__Group_2__1"


    // $ANTLR start "rule__Timing__Group_2__1__Impl"
    // InternalLinguaFranca.g:3789:1: rule__Timing__Group_2__1__Impl : ( ( rule__Timing__PeriodAssignment_2_1 ) ) ;
    public final void rule__Timing__Group_2__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3793:1: ( ( ( rule__Timing__PeriodAssignment_2_1 ) ) )
            // InternalLinguaFranca.g:3794:1: ( ( rule__Timing__PeriodAssignment_2_1 ) )
            {
            // InternalLinguaFranca.g:3794:1: ( ( rule__Timing__PeriodAssignment_2_1 ) )
            // InternalLinguaFranca.g:3795:2: ( rule__Timing__PeriodAssignment_2_1 )
            {
             before(grammarAccess.getTimingAccess().getPeriodAssignment_2_1()); 
            // InternalLinguaFranca.g:3796:2: ( rule__Timing__PeriodAssignment_2_1 )
            // InternalLinguaFranca.g:3796:3: rule__Timing__PeriodAssignment_2_1
            {
            pushFollow(FOLLOW_2);
            rule__Timing__PeriodAssignment_2_1();

            state._fsp--;


            }

             after(grammarAccess.getTimingAccess().getPeriodAssignment_2_1()); 

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
    // $ANTLR end "rule__Timing__Group_2__1__Impl"


    // $ANTLR start "rule__Port__Group_1__0"
    // InternalLinguaFranca.g:3805:1: rule__Port__Group_1__0 : rule__Port__Group_1__0__Impl rule__Port__Group_1__1 ;
    public final void rule__Port__Group_1__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3809:1: ( rule__Port__Group_1__0__Impl rule__Port__Group_1__1 )
            // InternalLinguaFranca.g:3810:2: rule__Port__Group_1__0__Impl rule__Port__Group_1__1
            {
            pushFollow(FOLLOW_39);
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
    // InternalLinguaFranca.g:3817:1: rule__Port__Group_1__0__Impl : ( RULE_ID ) ;
    public final void rule__Port__Group_1__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3821:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:3822:1: ( RULE_ID )
            {
            // InternalLinguaFranca.g:3822:1: ( RULE_ID )
            // InternalLinguaFranca.g:3823:2: RULE_ID
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
    // InternalLinguaFranca.g:3832:1: rule__Port__Group_1__1 : rule__Port__Group_1__1__Impl rule__Port__Group_1__2 ;
    public final void rule__Port__Group_1__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3836:1: ( rule__Port__Group_1__1__Impl rule__Port__Group_1__2 )
            // InternalLinguaFranca.g:3837:2: rule__Port__Group_1__1__Impl rule__Port__Group_1__2
            {
            pushFollow(FOLLOW_40);
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
    // InternalLinguaFranca.g:3844:1: rule__Port__Group_1__1__Impl : ( '.' ) ;
    public final void rule__Port__Group_1__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3848:1: ( ( '.' ) )
            // InternalLinguaFranca.g:3849:1: ( '.' )
            {
            // InternalLinguaFranca.g:3849:1: ( '.' )
            // InternalLinguaFranca.g:3850:2: '.'
            {
             before(grammarAccess.getPortAccess().getFullStopKeyword_1_1()); 
            match(input,37,FOLLOW_2); 
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
    // InternalLinguaFranca.g:3859:1: rule__Port__Group_1__2 : rule__Port__Group_1__2__Impl ;
    public final void rule__Port__Group_1__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3863:1: ( rule__Port__Group_1__2__Impl )
            // InternalLinguaFranca.g:3864:2: rule__Port__Group_1__2__Impl
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
    // InternalLinguaFranca.g:3870:1: rule__Port__Group_1__2__Impl : ( ( rule__Port__Alternatives_1_2 ) ) ;
    public final void rule__Port__Group_1__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3874:1: ( ( ( rule__Port__Alternatives_1_2 ) ) )
            // InternalLinguaFranca.g:3875:1: ( ( rule__Port__Alternatives_1_2 ) )
            {
            // InternalLinguaFranca.g:3875:1: ( ( rule__Port__Alternatives_1_2 ) )
            // InternalLinguaFranca.g:3876:2: ( rule__Port__Alternatives_1_2 )
            {
             before(grammarAccess.getPortAccess().getAlternatives_1_2()); 
            // InternalLinguaFranca.g:3877:2: ( rule__Port__Alternatives_1_2 )
            // InternalLinguaFranca.g:3877:3: rule__Port__Alternatives_1_2
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
    // InternalLinguaFranca.g:3886:1: rule__Sets__Group__0 : rule__Sets__Group__0__Impl rule__Sets__Group__1 ;
    public final void rule__Sets__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3890:1: ( rule__Sets__Group__0__Impl rule__Sets__Group__1 )
            // InternalLinguaFranca.g:3891:2: rule__Sets__Group__0__Impl rule__Sets__Group__1
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
    // InternalLinguaFranca.g:3898:1: rule__Sets__Group__0__Impl : ( '->' ) ;
    public final void rule__Sets__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3902:1: ( ( '->' ) )
            // InternalLinguaFranca.g:3903:1: ( '->' )
            {
            // InternalLinguaFranca.g:3903:1: ( '->' )
            // InternalLinguaFranca.g:3904:2: '->'
            {
             before(grammarAccess.getSetsAccess().getHyphenMinusGreaterThanSignKeyword_0()); 
            match(input,35,FOLLOW_2); 
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
    // InternalLinguaFranca.g:3913:1: rule__Sets__Group__1 : rule__Sets__Group__1__Impl rule__Sets__Group__2 ;
    public final void rule__Sets__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3917:1: ( rule__Sets__Group__1__Impl rule__Sets__Group__2 )
            // InternalLinguaFranca.g:3918:2: rule__Sets__Group__1__Impl rule__Sets__Group__2
            {
            pushFollow(FOLLOW_26);
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
    // InternalLinguaFranca.g:3925:1: rule__Sets__Group__1__Impl : ( ( rule__Sets__SetsAssignment_1 ) ) ;
    public final void rule__Sets__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3929:1: ( ( ( rule__Sets__SetsAssignment_1 ) ) )
            // InternalLinguaFranca.g:3930:1: ( ( rule__Sets__SetsAssignment_1 ) )
            {
            // InternalLinguaFranca.g:3930:1: ( ( rule__Sets__SetsAssignment_1 ) )
            // InternalLinguaFranca.g:3931:2: ( rule__Sets__SetsAssignment_1 )
            {
             before(grammarAccess.getSetsAccess().getSetsAssignment_1()); 
            // InternalLinguaFranca.g:3932:2: ( rule__Sets__SetsAssignment_1 )
            // InternalLinguaFranca.g:3932:3: rule__Sets__SetsAssignment_1
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
    // InternalLinguaFranca.g:3940:1: rule__Sets__Group__2 : rule__Sets__Group__2__Impl ;
    public final void rule__Sets__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3944:1: ( rule__Sets__Group__2__Impl )
            // InternalLinguaFranca.g:3945:2: rule__Sets__Group__2__Impl
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
    // InternalLinguaFranca.g:3951:1: rule__Sets__Group__2__Impl : ( ( rule__Sets__Group_2__0 )? ) ;
    public final void rule__Sets__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3955:1: ( ( ( rule__Sets__Group_2__0 )? ) )
            // InternalLinguaFranca.g:3956:1: ( ( rule__Sets__Group_2__0 )? )
            {
            // InternalLinguaFranca.g:3956:1: ( ( rule__Sets__Group_2__0 )? )
            // InternalLinguaFranca.g:3957:2: ( rule__Sets__Group_2__0 )?
            {
             before(grammarAccess.getSetsAccess().getGroup_2()); 
            // InternalLinguaFranca.g:3958:2: ( rule__Sets__Group_2__0 )?
            int alt41=2;
            int LA41_0 = input.LA(1);

            if ( (LA41_0==31) ) {
                alt41=1;
            }
            switch (alt41) {
                case 1 :
                    // InternalLinguaFranca.g:3958:3: rule__Sets__Group_2__0
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
    // InternalLinguaFranca.g:3967:1: rule__Sets__Group_2__0 : rule__Sets__Group_2__0__Impl rule__Sets__Group_2__1 ;
    public final void rule__Sets__Group_2__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3971:1: ( rule__Sets__Group_2__0__Impl rule__Sets__Group_2__1 )
            // InternalLinguaFranca.g:3972:2: rule__Sets__Group_2__0__Impl rule__Sets__Group_2__1
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
    // InternalLinguaFranca.g:3979:1: rule__Sets__Group_2__0__Impl : ( ',' ) ;
    public final void rule__Sets__Group_2__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3983:1: ( ( ',' ) )
            // InternalLinguaFranca.g:3984:1: ( ',' )
            {
            // InternalLinguaFranca.g:3984:1: ( ',' )
            // InternalLinguaFranca.g:3985:2: ','
            {
             before(grammarAccess.getSetsAccess().getCommaKeyword_2_0()); 
            match(input,31,FOLLOW_2); 
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
    // InternalLinguaFranca.g:3994:1: rule__Sets__Group_2__1 : rule__Sets__Group_2__1__Impl ;
    public final void rule__Sets__Group_2__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3998:1: ( rule__Sets__Group_2__1__Impl )
            // InternalLinguaFranca.g:3999:2: rule__Sets__Group_2__1__Impl
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
    // InternalLinguaFranca.g:4005:1: rule__Sets__Group_2__1__Impl : ( ( rule__Sets__SetsAssignment_2_1 ) ) ;
    public final void rule__Sets__Group_2__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4009:1: ( ( ( rule__Sets__SetsAssignment_2_1 ) ) )
            // InternalLinguaFranca.g:4010:1: ( ( rule__Sets__SetsAssignment_2_1 ) )
            {
            // InternalLinguaFranca.g:4010:1: ( ( rule__Sets__SetsAssignment_2_1 ) )
            // InternalLinguaFranca.g:4011:2: ( rule__Sets__SetsAssignment_2_1 )
            {
             before(grammarAccess.getSetsAccess().getSetsAssignment_2_1()); 
            // InternalLinguaFranca.g:4012:2: ( rule__Sets__SetsAssignment_2_1 )
            // InternalLinguaFranca.g:4012:3: rule__Sets__SetsAssignment_2_1
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
    // InternalLinguaFranca.g:4021:1: rule__Path__Group__0 : rule__Path__Group__0__Impl rule__Path__Group__1 ;
    public final void rule__Path__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4025:1: ( rule__Path__Group__0__Impl rule__Path__Group__1 )
            // InternalLinguaFranca.g:4026:2: rule__Path__Group__0__Impl rule__Path__Group__1
            {
            pushFollow(FOLLOW_39);
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
    // InternalLinguaFranca.g:4033:1: rule__Path__Group__0__Impl : ( RULE_ID ) ;
    public final void rule__Path__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4037:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4038:1: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4038:1: ( RULE_ID )
            // InternalLinguaFranca.g:4039:2: RULE_ID
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
    // InternalLinguaFranca.g:4048:1: rule__Path__Group__1 : rule__Path__Group__1__Impl ;
    public final void rule__Path__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4052:1: ( rule__Path__Group__1__Impl )
            // InternalLinguaFranca.g:4053:2: rule__Path__Group__1__Impl
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
    // InternalLinguaFranca.g:4059:1: rule__Path__Group__1__Impl : ( ( rule__Path__Group_1__0 )* ) ;
    public final void rule__Path__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4063:1: ( ( ( rule__Path__Group_1__0 )* ) )
            // InternalLinguaFranca.g:4064:1: ( ( rule__Path__Group_1__0 )* )
            {
            // InternalLinguaFranca.g:4064:1: ( ( rule__Path__Group_1__0 )* )
            // InternalLinguaFranca.g:4065:2: ( rule__Path__Group_1__0 )*
            {
             before(grammarAccess.getPathAccess().getGroup_1()); 
            // InternalLinguaFranca.g:4066:2: ( rule__Path__Group_1__0 )*
            loop42:
            do {
                int alt42=2;
                int LA42_0 = input.LA(1);

                if ( (LA42_0==37) ) {
                    alt42=1;
                }


                switch (alt42) {
            	case 1 :
            	    // InternalLinguaFranca.g:4066:3: rule__Path__Group_1__0
            	    {
            	    pushFollow(FOLLOW_41);
            	    rule__Path__Group_1__0();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop42;
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
    // InternalLinguaFranca.g:4075:1: rule__Path__Group_1__0 : rule__Path__Group_1__0__Impl rule__Path__Group_1__1 ;
    public final void rule__Path__Group_1__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4079:1: ( rule__Path__Group_1__0__Impl rule__Path__Group_1__1 )
            // InternalLinguaFranca.g:4080:2: rule__Path__Group_1__0__Impl rule__Path__Group_1__1
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
    // InternalLinguaFranca.g:4087:1: rule__Path__Group_1__0__Impl : ( '.' ) ;
    public final void rule__Path__Group_1__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4091:1: ( ( '.' ) )
            // InternalLinguaFranca.g:4092:1: ( '.' )
            {
            // InternalLinguaFranca.g:4092:1: ( '.' )
            // InternalLinguaFranca.g:4093:2: '.'
            {
             before(grammarAccess.getPathAccess().getFullStopKeyword_1_0()); 
            match(input,37,FOLLOW_2); 
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
    // InternalLinguaFranca.g:4102:1: rule__Path__Group_1__1 : rule__Path__Group_1__1__Impl ;
    public final void rule__Path__Group_1__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4106:1: ( rule__Path__Group_1__1__Impl )
            // InternalLinguaFranca.g:4107:2: rule__Path__Group_1__1__Impl
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
    // InternalLinguaFranca.g:4113:1: rule__Path__Group_1__1__Impl : ( RULE_ID ) ;
    public final void rule__Path__Group_1__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4117:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4118:1: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4118:1: ( RULE_ID )
            // InternalLinguaFranca.g:4119:2: RULE_ID
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
    // InternalLinguaFranca.g:4129:1: rule__Model__TargetAssignment_0 : ( ruleTarget ) ;
    public final void rule__Model__TargetAssignment_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4133:1: ( ( ruleTarget ) )
            // InternalLinguaFranca.g:4134:2: ( ruleTarget )
            {
            // InternalLinguaFranca.g:4134:2: ( ruleTarget )
            // InternalLinguaFranca.g:4135:3: ruleTarget
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
    // InternalLinguaFranca.g:4144:1: rule__Model__ImportsAssignment_1 : ( ruleImport ) ;
    public final void rule__Model__ImportsAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4148:1: ( ( ruleImport ) )
            // InternalLinguaFranca.g:4149:2: ( ruleImport )
            {
            // InternalLinguaFranca.g:4149:2: ( ruleImport )
            // InternalLinguaFranca.g:4150:3: ruleImport
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


    // $ANTLR start "rule__Model__ComponentsAssignment_2"
    // InternalLinguaFranca.g:4159:1: rule__Model__ComponentsAssignment_2 : ( ruleComponent ) ;
    public final void rule__Model__ComponentsAssignment_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4163:1: ( ( ruleComponent ) )
            // InternalLinguaFranca.g:4164:2: ( ruleComponent )
            {
            // InternalLinguaFranca.g:4164:2: ( ruleComponent )
            // InternalLinguaFranca.g:4165:3: ruleComponent
            {
             before(grammarAccess.getModelAccess().getComponentsComponentParserRuleCall_2_0()); 
            pushFollow(FOLLOW_2);
            ruleComponent();

            state._fsp--;

             after(grammarAccess.getModelAccess().getComponentsComponentParserRuleCall_2_0()); 

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
    // $ANTLR end "rule__Model__ComponentsAssignment_2"


    // $ANTLR start "rule__Target__NameAssignment_1"
    // InternalLinguaFranca.g:4174:1: rule__Target__NameAssignment_1 : ( RULE_ID ) ;
    public final void rule__Target__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4178:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4179:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4179:2: ( RULE_ID )
            // InternalLinguaFranca.g:4180:3: RULE_ID
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
    // InternalLinguaFranca.g:4189:1: rule__Import__NameAssignment_1 : ( rulePath ) ;
    public final void rule__Import__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4193:1: ( ( rulePath ) )
            // InternalLinguaFranca.g:4194:2: ( rulePath )
            {
            // InternalLinguaFranca.g:4194:2: ( rulePath )
            // InternalLinguaFranca.g:4195:3: rulePath
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


    // $ANTLR start "rule__Reactor__ComponentBodyAssignment_1"
    // InternalLinguaFranca.g:4204:1: rule__Reactor__ComponentBodyAssignment_1 : ( ruleComponentBody ) ;
    public final void rule__Reactor__ComponentBodyAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4208:1: ( ( ruleComponentBody ) )
            // InternalLinguaFranca.g:4209:2: ( ruleComponentBody )
            {
            // InternalLinguaFranca.g:4209:2: ( ruleComponentBody )
            // InternalLinguaFranca.g:4210:3: ruleComponentBody
            {
             before(grammarAccess.getReactorAccess().getComponentBodyComponentBodyParserRuleCall_1_0()); 
            pushFollow(FOLLOW_2);
            ruleComponentBody();

            state._fsp--;

             after(grammarAccess.getReactorAccess().getComponentBodyComponentBodyParserRuleCall_1_0()); 

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
    // $ANTLR end "rule__Reactor__ComponentBodyAssignment_1"


    // $ANTLR start "rule__Composite__ComponentBodyAssignment_1"
    // InternalLinguaFranca.g:4219:1: rule__Composite__ComponentBodyAssignment_1 : ( ruleComponentBody ) ;
    public final void rule__Composite__ComponentBodyAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4223:1: ( ( ruleComponentBody ) )
            // InternalLinguaFranca.g:4224:2: ( ruleComponentBody )
            {
            // InternalLinguaFranca.g:4224:2: ( ruleComponentBody )
            // InternalLinguaFranca.g:4225:3: ruleComponentBody
            {
             before(grammarAccess.getCompositeAccess().getComponentBodyComponentBodyParserRuleCall_1_0()); 
            pushFollow(FOLLOW_2);
            ruleComponentBody();

            state._fsp--;

             after(grammarAccess.getCompositeAccess().getComponentBodyComponentBodyParserRuleCall_1_0()); 

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
    // $ANTLR end "rule__Composite__ComponentBodyAssignment_1"


    // $ANTLR start "rule__Composite__InstancesAssignment_2"
    // InternalLinguaFranca.g:4234:1: rule__Composite__InstancesAssignment_2 : ( ruleInstance ) ;
    public final void rule__Composite__InstancesAssignment_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4238:1: ( ( ruleInstance ) )
            // InternalLinguaFranca.g:4239:2: ( ruleInstance )
            {
            // InternalLinguaFranca.g:4239:2: ( ruleInstance )
            // InternalLinguaFranca.g:4240:3: ruleInstance
            {
             before(grammarAccess.getCompositeAccess().getInstancesInstanceParserRuleCall_2_0()); 
            pushFollow(FOLLOW_2);
            ruleInstance();

            state._fsp--;

             after(grammarAccess.getCompositeAccess().getInstancesInstanceParserRuleCall_2_0()); 

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
    // $ANTLR end "rule__Composite__InstancesAssignment_2"


    // $ANTLR start "rule__Composite__ConnectionsAssignment_3"
    // InternalLinguaFranca.g:4249:1: rule__Composite__ConnectionsAssignment_3 : ( ruleConnection ) ;
    public final void rule__Composite__ConnectionsAssignment_3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4253:1: ( ( ruleConnection ) )
            // InternalLinguaFranca.g:4254:2: ( ruleConnection )
            {
            // InternalLinguaFranca.g:4254:2: ( ruleConnection )
            // InternalLinguaFranca.g:4255:3: ruleConnection
            {
             before(grammarAccess.getCompositeAccess().getConnectionsConnectionParserRuleCall_3_0()); 
            pushFollow(FOLLOW_2);
            ruleConnection();

            state._fsp--;

             after(grammarAccess.getCompositeAccess().getConnectionsConnectionParserRuleCall_3_0()); 

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
    // $ANTLR end "rule__Composite__ConnectionsAssignment_3"


    // $ANTLR start "rule__ComponentBody__NameAssignment_0"
    // InternalLinguaFranca.g:4264:1: rule__ComponentBody__NameAssignment_0 : ( RULE_ID ) ;
    public final void rule__ComponentBody__NameAssignment_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4268:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4269:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4269:2: ( RULE_ID )
            // InternalLinguaFranca.g:4270:3: RULE_ID
            {
             before(grammarAccess.getComponentBodyAccess().getNameIDTerminalRuleCall_0_0()); 
            match(input,RULE_ID,FOLLOW_2); 
             after(grammarAccess.getComponentBodyAccess().getNameIDTerminalRuleCall_0_0()); 

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
    // $ANTLR end "rule__ComponentBody__NameAssignment_0"


    // $ANTLR start "rule__ComponentBody__ParametersAssignment_1"
    // InternalLinguaFranca.g:4279:1: rule__ComponentBody__ParametersAssignment_1 : ( ruleParams ) ;
    public final void rule__ComponentBody__ParametersAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4283:1: ( ( ruleParams ) )
            // InternalLinguaFranca.g:4284:2: ( ruleParams )
            {
            // InternalLinguaFranca.g:4284:2: ( ruleParams )
            // InternalLinguaFranca.g:4285:3: ruleParams
            {
             before(grammarAccess.getComponentBodyAccess().getParametersParamsParserRuleCall_1_0()); 
            pushFollow(FOLLOW_2);
            ruleParams();

            state._fsp--;

             after(grammarAccess.getComponentBodyAccess().getParametersParamsParserRuleCall_1_0()); 

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
    // $ANTLR end "rule__ComponentBody__ParametersAssignment_1"


    // $ANTLR start "rule__ComponentBody__InputsAssignment_3"
    // InternalLinguaFranca.g:4294:1: rule__ComponentBody__InputsAssignment_3 : ( ruleInput ) ;
    public final void rule__ComponentBody__InputsAssignment_3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4298:1: ( ( ruleInput ) )
            // InternalLinguaFranca.g:4299:2: ( ruleInput )
            {
            // InternalLinguaFranca.g:4299:2: ( ruleInput )
            // InternalLinguaFranca.g:4300:3: ruleInput
            {
             before(grammarAccess.getComponentBodyAccess().getInputsInputParserRuleCall_3_0()); 
            pushFollow(FOLLOW_2);
            ruleInput();

            state._fsp--;

             after(grammarAccess.getComponentBodyAccess().getInputsInputParserRuleCall_3_0()); 

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
    // $ANTLR end "rule__ComponentBody__InputsAssignment_3"


    // $ANTLR start "rule__ComponentBody__OutputsAssignment_4"
    // InternalLinguaFranca.g:4309:1: rule__ComponentBody__OutputsAssignment_4 : ( ruleOutput ) ;
    public final void rule__ComponentBody__OutputsAssignment_4() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4313:1: ( ( ruleOutput ) )
            // InternalLinguaFranca.g:4314:2: ( ruleOutput )
            {
            // InternalLinguaFranca.g:4314:2: ( ruleOutput )
            // InternalLinguaFranca.g:4315:3: ruleOutput
            {
             before(grammarAccess.getComponentBodyAccess().getOutputsOutputParserRuleCall_4_0()); 
            pushFollow(FOLLOW_2);
            ruleOutput();

            state._fsp--;

             after(grammarAccess.getComponentBodyAccess().getOutputsOutputParserRuleCall_4_0()); 

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
    // $ANTLR end "rule__ComponentBody__OutputsAssignment_4"


    // $ANTLR start "rule__ComponentBody__TimersAssignment_5_0"
    // InternalLinguaFranca.g:4324:1: rule__ComponentBody__TimersAssignment_5_0 : ( ruleTimer ) ;
    public final void rule__ComponentBody__TimersAssignment_5_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4328:1: ( ( ruleTimer ) )
            // InternalLinguaFranca.g:4329:2: ( ruleTimer )
            {
            // InternalLinguaFranca.g:4329:2: ( ruleTimer )
            // InternalLinguaFranca.g:4330:3: ruleTimer
            {
             before(grammarAccess.getComponentBodyAccess().getTimersTimerParserRuleCall_5_0_0()); 
            pushFollow(FOLLOW_2);
            ruleTimer();

            state._fsp--;

             after(grammarAccess.getComponentBodyAccess().getTimersTimerParserRuleCall_5_0_0()); 

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
    // $ANTLR end "rule__ComponentBody__TimersAssignment_5_0"


    // $ANTLR start "rule__ComponentBody__ActionsAssignment_5_1"
    // InternalLinguaFranca.g:4339:1: rule__ComponentBody__ActionsAssignment_5_1 : ( ruleAction ) ;
    public final void rule__ComponentBody__ActionsAssignment_5_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4343:1: ( ( ruleAction ) )
            // InternalLinguaFranca.g:4344:2: ( ruleAction )
            {
            // InternalLinguaFranca.g:4344:2: ( ruleAction )
            // InternalLinguaFranca.g:4345:3: ruleAction
            {
             before(grammarAccess.getComponentBodyAccess().getActionsActionParserRuleCall_5_1_0()); 
            pushFollow(FOLLOW_2);
            ruleAction();

            state._fsp--;

             after(grammarAccess.getComponentBodyAccess().getActionsActionParserRuleCall_5_1_0()); 

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
    // $ANTLR end "rule__ComponentBody__ActionsAssignment_5_1"


    // $ANTLR start "rule__ComponentBody__PreambleAssignment_6"
    // InternalLinguaFranca.g:4354:1: rule__ComponentBody__PreambleAssignment_6 : ( rulePreamble ) ;
    public final void rule__ComponentBody__PreambleAssignment_6() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4358:1: ( ( rulePreamble ) )
            // InternalLinguaFranca.g:4359:2: ( rulePreamble )
            {
            // InternalLinguaFranca.g:4359:2: ( rulePreamble )
            // InternalLinguaFranca.g:4360:3: rulePreamble
            {
             before(grammarAccess.getComponentBodyAccess().getPreamblePreambleParserRuleCall_6_0()); 
            pushFollow(FOLLOW_2);
            rulePreamble();

            state._fsp--;

             after(grammarAccess.getComponentBodyAccess().getPreamblePreambleParserRuleCall_6_0()); 

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
    // $ANTLR end "rule__ComponentBody__PreambleAssignment_6"


    // $ANTLR start "rule__ComponentBody__ReactionsAssignment_7"
    // InternalLinguaFranca.g:4369:1: rule__ComponentBody__ReactionsAssignment_7 : ( ruleReaction ) ;
    public final void rule__ComponentBody__ReactionsAssignment_7() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4373:1: ( ( ruleReaction ) )
            // InternalLinguaFranca.g:4374:2: ( ruleReaction )
            {
            // InternalLinguaFranca.g:4374:2: ( ruleReaction )
            // InternalLinguaFranca.g:4375:3: ruleReaction
            {
             before(grammarAccess.getComponentBodyAccess().getReactionsReactionParserRuleCall_7_0()); 
            pushFollow(FOLLOW_2);
            ruleReaction();

            state._fsp--;

             after(grammarAccess.getComponentBodyAccess().getReactionsReactionParserRuleCall_7_0()); 

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
    // $ANTLR end "rule__ComponentBody__ReactionsAssignment_7"


    // $ANTLR start "rule__Input__NameAssignment_1"
    // InternalLinguaFranca.g:4384:1: rule__Input__NameAssignment_1 : ( ( rule__Input__NameAlternatives_1_0 ) ) ;
    public final void rule__Input__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4388:1: ( ( ( rule__Input__NameAlternatives_1_0 ) ) )
            // InternalLinguaFranca.g:4389:2: ( ( rule__Input__NameAlternatives_1_0 ) )
            {
            // InternalLinguaFranca.g:4389:2: ( ( rule__Input__NameAlternatives_1_0 ) )
            // InternalLinguaFranca.g:4390:3: ( rule__Input__NameAlternatives_1_0 )
            {
             before(grammarAccess.getInputAccess().getNameAlternatives_1_0()); 
            // InternalLinguaFranca.g:4391:3: ( rule__Input__NameAlternatives_1_0 )
            // InternalLinguaFranca.g:4391:4: rule__Input__NameAlternatives_1_0
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
    // InternalLinguaFranca.g:4399:1: rule__Input__TypeAssignment_2_1 : ( ruleType ) ;
    public final void rule__Input__TypeAssignment_2_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4403:1: ( ( ruleType ) )
            // InternalLinguaFranca.g:4404:2: ( ruleType )
            {
            // InternalLinguaFranca.g:4404:2: ( ruleType )
            // InternalLinguaFranca.g:4405:3: ruleType
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
    // InternalLinguaFranca.g:4414:1: rule__Output__NameAssignment_1 : ( ( rule__Output__NameAlternatives_1_0 ) ) ;
    public final void rule__Output__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4418:1: ( ( ( rule__Output__NameAlternatives_1_0 ) ) )
            // InternalLinguaFranca.g:4419:2: ( ( rule__Output__NameAlternatives_1_0 ) )
            {
            // InternalLinguaFranca.g:4419:2: ( ( rule__Output__NameAlternatives_1_0 ) )
            // InternalLinguaFranca.g:4420:3: ( rule__Output__NameAlternatives_1_0 )
            {
             before(grammarAccess.getOutputAccess().getNameAlternatives_1_0()); 
            // InternalLinguaFranca.g:4421:3: ( rule__Output__NameAlternatives_1_0 )
            // InternalLinguaFranca.g:4421:4: rule__Output__NameAlternatives_1_0
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
    // InternalLinguaFranca.g:4429:1: rule__Output__TypeAssignment_2_1 : ( ruleType ) ;
    public final void rule__Output__TypeAssignment_2_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4433:1: ( ( ruleType ) )
            // InternalLinguaFranca.g:4434:2: ( ruleType )
            {
            // InternalLinguaFranca.g:4434:2: ( ruleType )
            // InternalLinguaFranca.g:4435:3: ruleType
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


    // $ANTLR start "rule__Timer__NameAssignment_1"
    // InternalLinguaFranca.g:4444:1: rule__Timer__NameAssignment_1 : ( ( rule__Timer__NameAlternatives_1_0 ) ) ;
    public final void rule__Timer__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4448:1: ( ( ( rule__Timer__NameAlternatives_1_0 ) ) )
            // InternalLinguaFranca.g:4449:2: ( ( rule__Timer__NameAlternatives_1_0 ) )
            {
            // InternalLinguaFranca.g:4449:2: ( ( rule__Timer__NameAlternatives_1_0 ) )
            // InternalLinguaFranca.g:4450:3: ( rule__Timer__NameAlternatives_1_0 )
            {
             before(grammarAccess.getTimerAccess().getNameAlternatives_1_0()); 
            // InternalLinguaFranca.g:4451:3: ( rule__Timer__NameAlternatives_1_0 )
            // InternalLinguaFranca.g:4451:4: rule__Timer__NameAlternatives_1_0
            {
            pushFollow(FOLLOW_2);
            rule__Timer__NameAlternatives_1_0();

            state._fsp--;


            }

             after(grammarAccess.getTimerAccess().getNameAlternatives_1_0()); 

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
    // $ANTLR end "rule__Timer__NameAssignment_1"


    // $ANTLR start "rule__Timer__TimingAssignment_2"
    // InternalLinguaFranca.g:4459:1: rule__Timer__TimingAssignment_2 : ( ruleTiming ) ;
    public final void rule__Timer__TimingAssignment_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4463:1: ( ( ruleTiming ) )
            // InternalLinguaFranca.g:4464:2: ( ruleTiming )
            {
            // InternalLinguaFranca.g:4464:2: ( ruleTiming )
            // InternalLinguaFranca.g:4465:3: ruleTiming
            {
             before(grammarAccess.getTimerAccess().getTimingTimingParserRuleCall_2_0()); 
            pushFollow(FOLLOW_2);
            ruleTiming();

            state._fsp--;

             after(grammarAccess.getTimerAccess().getTimingTimingParserRuleCall_2_0()); 

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
    // $ANTLR end "rule__Timer__TimingAssignment_2"


    // $ANTLR start "rule__Action__NameAssignment_1"
    // InternalLinguaFranca.g:4474:1: rule__Action__NameAssignment_1 : ( ( rule__Action__NameAlternatives_1_0 ) ) ;
    public final void rule__Action__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4478:1: ( ( ( rule__Action__NameAlternatives_1_0 ) ) )
            // InternalLinguaFranca.g:4479:2: ( ( rule__Action__NameAlternatives_1_0 ) )
            {
            // InternalLinguaFranca.g:4479:2: ( ( rule__Action__NameAlternatives_1_0 ) )
            // InternalLinguaFranca.g:4480:3: ( rule__Action__NameAlternatives_1_0 )
            {
             before(grammarAccess.getActionAccess().getNameAlternatives_1_0()); 
            // InternalLinguaFranca.g:4481:3: ( rule__Action__NameAlternatives_1_0 )
            // InternalLinguaFranca.g:4481:4: rule__Action__NameAlternatives_1_0
            {
            pushFollow(FOLLOW_2);
            rule__Action__NameAlternatives_1_0();

            state._fsp--;


            }

             after(grammarAccess.getActionAccess().getNameAlternatives_1_0()); 

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
    // $ANTLR end "rule__Action__NameAssignment_1"


    // $ANTLR start "rule__Action__TimingAssignment_2"
    // InternalLinguaFranca.g:4489:1: rule__Action__TimingAssignment_2 : ( ruleTiming ) ;
    public final void rule__Action__TimingAssignment_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4493:1: ( ( ruleTiming ) )
            // InternalLinguaFranca.g:4494:2: ( ruleTiming )
            {
            // InternalLinguaFranca.g:4494:2: ( ruleTiming )
            // InternalLinguaFranca.g:4495:3: ruleTiming
            {
             before(grammarAccess.getActionAccess().getTimingTimingParserRuleCall_2_0()); 
            pushFollow(FOLLOW_2);
            ruleTiming();

            state._fsp--;

             after(grammarAccess.getActionAccess().getTimingTimingParserRuleCall_2_0()); 

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
    // $ANTLR end "rule__Action__TimingAssignment_2"


    // $ANTLR start "rule__Reaction__TriggersAssignment_1_1_0"
    // InternalLinguaFranca.g:4504:1: rule__Reaction__TriggersAssignment_1_1_0 : ( RULE_ID ) ;
    public final void rule__Reaction__TriggersAssignment_1_1_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4508:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4509:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4509:2: ( RULE_ID )
            // InternalLinguaFranca.g:4510:3: RULE_ID
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
    // InternalLinguaFranca.g:4519:1: rule__Reaction__TriggersAssignment_1_1_1_1 : ( RULE_ID ) ;
    public final void rule__Reaction__TriggersAssignment_1_1_1_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4523:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4524:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4524:2: ( RULE_ID )
            // InternalLinguaFranca.g:4525:3: RULE_ID
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
    // InternalLinguaFranca.g:4534:1: rule__Reaction__GetsAssignment_2 : ( ruleGets ) ;
    public final void rule__Reaction__GetsAssignment_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4538:1: ( ( ruleGets ) )
            // InternalLinguaFranca.g:4539:2: ( ruleGets )
            {
            // InternalLinguaFranca.g:4539:2: ( ruleGets )
            // InternalLinguaFranca.g:4540:3: ruleGets
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
    // InternalLinguaFranca.g:4549:1: rule__Reaction__SetsAssignment_3 : ( ruleSets ) ;
    public final void rule__Reaction__SetsAssignment_3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4553:1: ( ( ruleSets ) )
            // InternalLinguaFranca.g:4554:2: ( ruleSets )
            {
            // InternalLinguaFranca.g:4554:2: ( ruleSets )
            // InternalLinguaFranca.g:4555:3: ruleSets
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
    // InternalLinguaFranca.g:4564:1: rule__Reaction__CodeAssignment_4 : ( RULE_CODE ) ;
    public final void rule__Reaction__CodeAssignment_4() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4568:1: ( ( RULE_CODE ) )
            // InternalLinguaFranca.g:4569:2: ( RULE_CODE )
            {
            // InternalLinguaFranca.g:4569:2: ( RULE_CODE )
            // InternalLinguaFranca.g:4570:3: RULE_CODE
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
    // InternalLinguaFranca.g:4579:1: rule__Preamble__CodeAssignment_1 : ( RULE_CODE ) ;
    public final void rule__Preamble__CodeAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4583:1: ( ( RULE_CODE ) )
            // InternalLinguaFranca.g:4584:2: ( RULE_CODE )
            {
            // InternalLinguaFranca.g:4584:2: ( RULE_CODE )
            // InternalLinguaFranca.g:4585:3: RULE_CODE
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


    // $ANTLR start "rule__Instance__NameAssignment_0"
    // InternalLinguaFranca.g:4594:1: rule__Instance__NameAssignment_0 : ( RULE_ID ) ;
    public final void rule__Instance__NameAssignment_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4598:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4599:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4599:2: ( RULE_ID )
            // InternalLinguaFranca.g:4600:3: RULE_ID
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
    // InternalLinguaFranca.g:4609:1: rule__Instance__ActorClassAssignment_3 : ( RULE_ID ) ;
    public final void rule__Instance__ActorClassAssignment_3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4613:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4614:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4614:2: ( RULE_ID )
            // InternalLinguaFranca.g:4615:3: RULE_ID
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
    // InternalLinguaFranca.g:4624:1: rule__Instance__ParametersAssignment_4_1 : ( ruleAssignments ) ;
    public final void rule__Instance__ParametersAssignment_4_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4628:1: ( ( ruleAssignments ) )
            // InternalLinguaFranca.g:4629:2: ( ruleAssignments )
            {
            // InternalLinguaFranca.g:4629:2: ( ruleAssignments )
            // InternalLinguaFranca.g:4630:3: ruleAssignments
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
    // InternalLinguaFranca.g:4639:1: rule__Connection__LeftPortAssignment_0 : ( rulePort ) ;
    public final void rule__Connection__LeftPortAssignment_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4643:1: ( ( rulePort ) )
            // InternalLinguaFranca.g:4644:2: ( rulePort )
            {
            // InternalLinguaFranca.g:4644:2: ( rulePort )
            // InternalLinguaFranca.g:4645:3: rulePort
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
    // InternalLinguaFranca.g:4654:1: rule__Connection__RightPortAssignment_2 : ( rulePort ) ;
    public final void rule__Connection__RightPortAssignment_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4658:1: ( ( rulePort ) )
            // InternalLinguaFranca.g:4659:2: ( rulePort )
            {
            // InternalLinguaFranca.g:4659:2: ( rulePort )
            // InternalLinguaFranca.g:4660:3: rulePort
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
    // InternalLinguaFranca.g:4669:1: rule__Assignments__AssignmentsAssignment_0 : ( ruleAssignment ) ;
    public final void rule__Assignments__AssignmentsAssignment_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4673:1: ( ( ruleAssignment ) )
            // InternalLinguaFranca.g:4674:2: ( ruleAssignment )
            {
            // InternalLinguaFranca.g:4674:2: ( ruleAssignment )
            // InternalLinguaFranca.g:4675:3: ruleAssignment
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
    // InternalLinguaFranca.g:4684:1: rule__Assignments__AssignmentsAssignment_1_1 : ( ruleAssignment ) ;
    public final void rule__Assignments__AssignmentsAssignment_1_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4688:1: ( ( ruleAssignment ) )
            // InternalLinguaFranca.g:4689:2: ( ruleAssignment )
            {
            // InternalLinguaFranca.g:4689:2: ( ruleAssignment )
            // InternalLinguaFranca.g:4690:3: ruleAssignment
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
    // InternalLinguaFranca.g:4699:1: rule__Assignment__NameAssignment_0 : ( RULE_ID ) ;
    public final void rule__Assignment__NameAssignment_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4703:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4704:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4704:2: ( RULE_ID )
            // InternalLinguaFranca.g:4705:3: RULE_ID
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
    // InternalLinguaFranca.g:4714:1: rule__Assignment__ValueAssignment_2 : ( ruleValue ) ;
    public final void rule__Assignment__ValueAssignment_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4718:1: ( ( ruleValue ) )
            // InternalLinguaFranca.g:4719:2: ( ruleValue )
            {
            // InternalLinguaFranca.g:4719:2: ( ruleValue )
            // InternalLinguaFranca.g:4720:3: ruleValue
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
    // InternalLinguaFranca.g:4729:1: rule__Gets__GetsAssignment_0 : ( RULE_ID ) ;
    public final void rule__Gets__GetsAssignment_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4733:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4734:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4734:2: ( RULE_ID )
            // InternalLinguaFranca.g:4735:3: RULE_ID
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
    // InternalLinguaFranca.g:4744:1: rule__Gets__GetsAssignment_1_1 : ( RULE_ID ) ;
    public final void rule__Gets__GetsAssignment_1_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4748:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4749:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4749:2: ( RULE_ID )
            // InternalLinguaFranca.g:4750:3: RULE_ID
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
    // InternalLinguaFranca.g:4759:1: rule__Params__ParamsAssignment_1 : ( ruleParam ) ;
    public final void rule__Params__ParamsAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4763:1: ( ( ruleParam ) )
            // InternalLinguaFranca.g:4764:2: ( ruleParam )
            {
            // InternalLinguaFranca.g:4764:2: ( ruleParam )
            // InternalLinguaFranca.g:4765:3: ruleParam
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
    // InternalLinguaFranca.g:4774:1: rule__Params__ParamsAssignment_2_1 : ( ruleParam ) ;
    public final void rule__Params__ParamsAssignment_2_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4778:1: ( ( ruleParam ) )
            // InternalLinguaFranca.g:4779:2: ( ruleParam )
            {
            // InternalLinguaFranca.g:4779:2: ( ruleParam )
            // InternalLinguaFranca.g:4780:3: ruleParam
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
    // InternalLinguaFranca.g:4789:1: rule__Param__NameAssignment_1 : ( RULE_ID ) ;
    public final void rule__Param__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4793:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4794:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4794:2: ( RULE_ID )
            // InternalLinguaFranca.g:4795:3: RULE_ID
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
    // InternalLinguaFranca.g:4804:1: rule__Param__TypeAssignment_2_1 : ( ruleType ) ;
    public final void rule__Param__TypeAssignment_2_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4808:1: ( ( ruleType ) )
            // InternalLinguaFranca.g:4809:2: ( ruleType )
            {
            // InternalLinguaFranca.g:4809:2: ( ruleType )
            // InternalLinguaFranca.g:4810:3: ruleType
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
    // InternalLinguaFranca.g:4819:1: rule__Param__ValueAssignment_3_1 : ( ruleValue ) ;
    public final void rule__Param__ValueAssignment_3_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4823:1: ( ( ruleValue ) )
            // InternalLinguaFranca.g:4824:2: ( ruleValue )
            {
            // InternalLinguaFranca.g:4824:2: ( ruleValue )
            // InternalLinguaFranca.g:4825:3: ruleValue
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


    // $ANTLR start "rule__Timing__OffsetAssignment_1"
    // InternalLinguaFranca.g:4834:1: rule__Timing__OffsetAssignment_1 : ( ( rule__Timing__OffsetAlternatives_1_0 ) ) ;
    public final void rule__Timing__OffsetAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4838:1: ( ( ( rule__Timing__OffsetAlternatives_1_0 ) ) )
            // InternalLinguaFranca.g:4839:2: ( ( rule__Timing__OffsetAlternatives_1_0 ) )
            {
            // InternalLinguaFranca.g:4839:2: ( ( rule__Timing__OffsetAlternatives_1_0 ) )
            // InternalLinguaFranca.g:4840:3: ( rule__Timing__OffsetAlternatives_1_0 )
            {
             before(grammarAccess.getTimingAccess().getOffsetAlternatives_1_0()); 
            // InternalLinguaFranca.g:4841:3: ( rule__Timing__OffsetAlternatives_1_0 )
            // InternalLinguaFranca.g:4841:4: rule__Timing__OffsetAlternatives_1_0
            {
            pushFollow(FOLLOW_2);
            rule__Timing__OffsetAlternatives_1_0();

            state._fsp--;


            }

             after(grammarAccess.getTimingAccess().getOffsetAlternatives_1_0()); 

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
    // $ANTLR end "rule__Timing__OffsetAssignment_1"


    // $ANTLR start "rule__Timing__PeriodAssignment_2_1"
    // InternalLinguaFranca.g:4849:1: rule__Timing__PeriodAssignment_2_1 : ( ( rule__Timing__PeriodAlternatives_2_1_0 ) ) ;
    public final void rule__Timing__PeriodAssignment_2_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4853:1: ( ( ( rule__Timing__PeriodAlternatives_2_1_0 ) ) )
            // InternalLinguaFranca.g:4854:2: ( ( rule__Timing__PeriodAlternatives_2_1_0 ) )
            {
            // InternalLinguaFranca.g:4854:2: ( ( rule__Timing__PeriodAlternatives_2_1_0 ) )
            // InternalLinguaFranca.g:4855:3: ( rule__Timing__PeriodAlternatives_2_1_0 )
            {
             before(grammarAccess.getTimingAccess().getPeriodAlternatives_2_1_0()); 
            // InternalLinguaFranca.g:4856:3: ( rule__Timing__PeriodAlternatives_2_1_0 )
            // InternalLinguaFranca.g:4856:4: rule__Timing__PeriodAlternatives_2_1_0
            {
            pushFollow(FOLLOW_2);
            rule__Timing__PeriodAlternatives_2_1_0();

            state._fsp--;


            }

             after(grammarAccess.getTimingAccess().getPeriodAlternatives_2_1_0()); 

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
    // $ANTLR end "rule__Timing__PeriodAssignment_2_1"


    // $ANTLR start "rule__Sets__SetsAssignment_1"
    // InternalLinguaFranca.g:4864:1: rule__Sets__SetsAssignment_1 : ( RULE_ID ) ;
    public final void rule__Sets__SetsAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4868:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4869:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4869:2: ( RULE_ID )
            // InternalLinguaFranca.g:4870:3: RULE_ID
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
    // InternalLinguaFranca.g:4879:1: rule__Sets__SetsAssignment_2_1 : ( RULE_ID ) ;
    public final void rule__Sets__SetsAssignment_2_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4883:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4884:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4884:2: ( RULE_ID )
            // InternalLinguaFranca.g:4885:3: RULE_ID
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
    public static final BitSet FOLLOW_3 = new BitSet(new long[]{0x0000000002C00000L});
    public static final BitSet FOLLOW_4 = new BitSet(new long[]{0x0000000000400002L});
    public static final BitSet FOLLOW_5 = new BitSet(new long[]{0x0000000002C00002L});
    public static final BitSet FOLLOW_6 = new BitSet(new long[]{0x0000000000000010L});
    public static final BitSet FOLLOW_7 = new BitSet(new long[]{0x0000000000200000L});
    public static final BitSet FOLLOW_8 = new BitSet(new long[]{0x0000000001000000L});
    public static final BitSet FOLLOW_9 = new BitSet(new long[]{0x0000000001000010L});
    public static final BitSet FOLLOW_10 = new BitSet(new long[]{0x0000000000000012L});
    public static final BitSet FOLLOW_11 = new BitSet(new long[]{0x0000000024000000L});
    public static final BitSet FOLLOW_12 = new BitSet(new long[]{0x000000011001E000L});
    public static final BitSet FOLLOW_13 = new BitSet(new long[]{0x0000000000002002L});
    public static final BitSet FOLLOW_14 = new BitSet(new long[]{0x0000000000004002L});
    public static final BitSet FOLLOW_15 = new BitSet(new long[]{0x0000000000018002L});
    public static final BitSet FOLLOW_16 = new BitSet(new long[]{0x0000000010000002L});
    public static final BitSet FOLLOW_17 = new BitSet(new long[]{0x0000000000002010L});
    public static final BitSet FOLLOW_18 = new BitSet(new long[]{0x0000000008200000L});
    public static final BitSet FOLLOW_19 = new BitSet(new long[]{0x0000000000000050L});
    public static final BitSet FOLLOW_20 = new BitSet(new long[]{0x0000000000004010L});
    public static final BitSet FOLLOW_21 = new BitSet(new long[]{0x0000000000008010L});
    public static final BitSet FOLLOW_22 = new BitSet(new long[]{0x0000000020200000L});
    public static final BitSet FOLLOW_23 = new BitSet(new long[]{0x0000000000010010L});
    public static final BitSet FOLLOW_24 = new BitSet(new long[]{0x0000000820000050L});
    public static final BitSet FOLLOW_25 = new BitSet(new long[]{0x0000000040000010L});
    public static final BitSet FOLLOW_26 = new BitSet(new long[]{0x0000000080000000L});
    public static final BitSet FOLLOW_27 = new BitSet(new long[]{0x0000000080000002L});
    public static final BitSet FOLLOW_28 = new BitSet(new long[]{0x0000000000000040L});
    public static final BitSet FOLLOW_29 = new BitSet(new long[]{0x0000000200000000L});
    public static final BitSet FOLLOW_30 = new BitSet(new long[]{0x0000000400000000L});
    public static final BitSet FOLLOW_31 = new BitSet(new long[]{0x0000000800000000L});
    public static final BitSet FOLLOW_32 = new BitSet(new long[]{0x00000000000000F0L});
    public static final BitSet FOLLOW_33 = new BitSet(new long[]{0x0000001000000010L});
    public static final BitSet FOLLOW_34 = new BitSet(new long[]{0x00000000C0000000L});
    public static final BitSet FOLLOW_35 = new BitSet(new long[]{0x0000000028000000L});
    public static final BitSet FOLLOW_36 = new BitSet(new long[]{0x0000000040000000L});
    public static final BitSet FOLLOW_37 = new BitSet(new long[]{0x0000000000020030L});
    public static final BitSet FOLLOW_38 = new BitSet(new long[]{0x00000000000C0030L});
    public static final BitSet FOLLOW_39 = new BitSet(new long[]{0x0000002000000000L});
    public static final BitSet FOLLOW_40 = new BitSet(new long[]{0x0000000000006010L});
    public static final BitSet FOLLOW_41 = new BitSet(new long[]{0x0000002000000002L});

}