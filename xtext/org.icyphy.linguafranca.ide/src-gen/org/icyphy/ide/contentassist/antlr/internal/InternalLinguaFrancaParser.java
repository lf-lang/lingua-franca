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
        "<invalid>", "<EOR>", "<DOWN>", "<UP>", "RULE_ID", "RULE_NUMBER", "RULE_CODE", "RULE_STRING", "RULE_INT", "RULE_ML_COMMENT", "RULE_SL_COMMENT", "RULE_WS", "RULE_ANY_OTHER", "'input'", "'output'", "'trigger'", "'target'", "';'", "'import'", "'actor'", "'{'", "'}'", "'composite'", "':'", "'reaction'", "'('", "')'", "','", "'preamble'", "'initialize'", "'instance'", "'='", "'->'", "'const'", "'.'", "'PERIODIC'", "'ONCE'"
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


    // $ANTLR start "entryRuleActor"
    // InternalLinguaFranca.g:128:1: entryRuleActor : ruleActor EOF ;
    public final void entryRuleActor() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:129:1: ( ruleActor EOF )
            // InternalLinguaFranca.g:130:1: ruleActor EOF
            {
             before(grammarAccess.getActorRule()); 
            pushFollow(FOLLOW_1);
            ruleActor();

            state._fsp--;

             after(grammarAccess.getActorRule()); 
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
    // $ANTLR end "entryRuleActor"


    // $ANTLR start "ruleActor"
    // InternalLinguaFranca.g:137:1: ruleActor : ( ( rule__Actor__Group__0 ) ) ;
    public final void ruleActor() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:141:2: ( ( ( rule__Actor__Group__0 ) ) )
            // InternalLinguaFranca.g:142:2: ( ( rule__Actor__Group__0 ) )
            {
            // InternalLinguaFranca.g:142:2: ( ( rule__Actor__Group__0 ) )
            // InternalLinguaFranca.g:143:3: ( rule__Actor__Group__0 )
            {
             before(grammarAccess.getActorAccess().getGroup()); 
            // InternalLinguaFranca.g:144:3: ( rule__Actor__Group__0 )
            // InternalLinguaFranca.g:144:4: rule__Actor__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Actor__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getActorAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleActor"


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


    // $ANTLR start "entryRuleTrigger"
    // InternalLinguaFranca.g:228:1: entryRuleTrigger : ruleTrigger EOF ;
    public final void entryRuleTrigger() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:229:1: ( ruleTrigger EOF )
            // InternalLinguaFranca.g:230:1: ruleTrigger EOF
            {
             before(grammarAccess.getTriggerRule()); 
            pushFollow(FOLLOW_1);
            ruleTrigger();

            state._fsp--;

             after(grammarAccess.getTriggerRule()); 
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
    // $ANTLR end "entryRuleTrigger"


    // $ANTLR start "ruleTrigger"
    // InternalLinguaFranca.g:237:1: ruleTrigger : ( ( rule__Trigger__Group__0 ) ) ;
    public final void ruleTrigger() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:241:2: ( ( ( rule__Trigger__Group__0 ) ) )
            // InternalLinguaFranca.g:242:2: ( ( rule__Trigger__Group__0 ) )
            {
            // InternalLinguaFranca.g:242:2: ( ( rule__Trigger__Group__0 ) )
            // InternalLinguaFranca.g:243:3: ( rule__Trigger__Group__0 )
            {
             before(grammarAccess.getTriggerAccess().getGroup()); 
            // InternalLinguaFranca.g:244:3: ( rule__Trigger__Group__0 )
            // InternalLinguaFranca.g:244:4: rule__Trigger__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Trigger__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getTriggerAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleTrigger"


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


    // $ANTLR start "entryRuleInitialize"
    // InternalLinguaFranca.g:303:1: entryRuleInitialize : ruleInitialize EOF ;
    public final void entryRuleInitialize() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:304:1: ( ruleInitialize EOF )
            // InternalLinguaFranca.g:305:1: ruleInitialize EOF
            {
             before(grammarAccess.getInitializeRule()); 
            pushFollow(FOLLOW_1);
            ruleInitialize();

            state._fsp--;

             after(grammarAccess.getInitializeRule()); 
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
    // $ANTLR end "entryRuleInitialize"


    // $ANTLR start "ruleInitialize"
    // InternalLinguaFranca.g:312:1: ruleInitialize : ( ( rule__Initialize__Group__0 ) ) ;
    public final void ruleInitialize() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:316:2: ( ( ( rule__Initialize__Group__0 ) ) )
            // InternalLinguaFranca.g:317:2: ( ( rule__Initialize__Group__0 ) )
            {
            // InternalLinguaFranca.g:317:2: ( ( rule__Initialize__Group__0 ) )
            // InternalLinguaFranca.g:318:3: ( rule__Initialize__Group__0 )
            {
             before(grammarAccess.getInitializeAccess().getGroup()); 
            // InternalLinguaFranca.g:319:3: ( rule__Initialize__Group__0 )
            // InternalLinguaFranca.g:319:4: rule__Initialize__Group__0
            {
            pushFollow(FOLLOW_2);
            rule__Initialize__Group__0();

            state._fsp--;


            }

             after(grammarAccess.getInitializeAccess().getGroup()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "ruleInitialize"


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
    // InternalLinguaFranca.g:652:1: rule__Model__BlocksAlternatives_2_0 : ( ( ruleActor ) | ( ruleComposite ) );
    public final void rule__Model__BlocksAlternatives_2_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:656:1: ( ( ruleActor ) | ( ruleComposite ) )
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
                    // InternalLinguaFranca.g:657:2: ( ruleActor )
                    {
                    // InternalLinguaFranca.g:657:2: ( ruleActor )
                    // InternalLinguaFranca.g:658:3: ruleActor
                    {
                     before(grammarAccess.getModelAccess().getBlocksActorParserRuleCall_2_0_0()); 
                    pushFollow(FOLLOW_2);
                    ruleActor();

                    state._fsp--;

                     after(grammarAccess.getModelAccess().getBlocksActorParserRuleCall_2_0_0()); 

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


    // $ANTLR start "rule__Trigger__NameAlternatives_1_0"
    // InternalLinguaFranca.g:715:1: rule__Trigger__NameAlternatives_1_0 : ( ( RULE_ID ) | ( 'trigger' ) );
    public final void rule__Trigger__NameAlternatives_1_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:719:1: ( ( RULE_ID ) | ( 'trigger' ) )
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
                     before(grammarAccess.getTriggerAccess().getNameIDTerminalRuleCall_1_0_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getTriggerAccess().getNameIDTerminalRuleCall_1_0_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:726:2: ( 'trigger' )
                    {
                    // InternalLinguaFranca.g:726:2: ( 'trigger' )
                    // InternalLinguaFranca.g:727:3: 'trigger'
                    {
                     before(grammarAccess.getTriggerAccess().getNameTriggerKeyword_1_0_1()); 
                    match(input,15,FOLLOW_2); 
                     after(grammarAccess.getTriggerAccess().getNameTriggerKeyword_1_0_1()); 

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
    // $ANTLR end "rule__Trigger__NameAlternatives_1_0"


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


    // $ANTLR start "rule__Period__Alternatives_2_1"
    // InternalLinguaFranca.g:757:1: rule__Period__Alternatives_2_1 : ( ( ( rule__Period__PeriodicAssignment_2_1_0 ) ) | ( ( rule__Period__OnceAssignment_2_1_1 ) ) );
    public final void rule__Period__Alternatives_2_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:761:1: ( ( ( rule__Period__PeriodicAssignment_2_1_0 ) ) | ( ( rule__Period__OnceAssignment_2_1_1 ) ) )
            int alt6=2;
            int LA6_0 = input.LA(1);

            if ( (LA6_0==35) ) {
                alt6=1;
            }
            else if ( (LA6_0==36) ) {
                alt6=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 6, 0, input);

                throw nvae;
            }
            switch (alt6) {
                case 1 :
                    // InternalLinguaFranca.g:762:2: ( ( rule__Period__PeriodicAssignment_2_1_0 ) )
                    {
                    // InternalLinguaFranca.g:762:2: ( ( rule__Period__PeriodicAssignment_2_1_0 ) )
                    // InternalLinguaFranca.g:763:3: ( rule__Period__PeriodicAssignment_2_1_0 )
                    {
                     before(grammarAccess.getPeriodAccess().getPeriodicAssignment_2_1_0()); 
                    // InternalLinguaFranca.g:764:3: ( rule__Period__PeriodicAssignment_2_1_0 )
                    // InternalLinguaFranca.g:764:4: rule__Period__PeriodicAssignment_2_1_0
                    {
                    pushFollow(FOLLOW_2);
                    rule__Period__PeriodicAssignment_2_1_0();

                    state._fsp--;


                    }

                     after(grammarAccess.getPeriodAccess().getPeriodicAssignment_2_1_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:768:2: ( ( rule__Period__OnceAssignment_2_1_1 ) )
                    {
                    // InternalLinguaFranca.g:768:2: ( ( rule__Period__OnceAssignment_2_1_1 ) )
                    // InternalLinguaFranca.g:769:3: ( rule__Period__OnceAssignment_2_1_1 )
                    {
                     before(grammarAccess.getPeriodAccess().getOnceAssignment_2_1_1()); 
                    // InternalLinguaFranca.g:770:3: ( rule__Period__OnceAssignment_2_1_1 )
                    // InternalLinguaFranca.g:770:4: rule__Period__OnceAssignment_2_1_1
                    {
                    pushFollow(FOLLOW_2);
                    rule__Period__OnceAssignment_2_1_1();

                    state._fsp--;


                    }

                     after(grammarAccess.getPeriodAccess().getOnceAssignment_2_1_1()); 

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
    // $ANTLR end "rule__Period__Alternatives_2_1"


    // $ANTLR start "rule__Port__Alternatives"
    // InternalLinguaFranca.g:778:1: rule__Port__Alternatives : ( ( RULE_ID ) | ( ( rule__Port__Group_1__0 ) ) );
    public final void rule__Port__Alternatives() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:782:1: ( ( RULE_ID ) | ( ( rule__Port__Group_1__0 ) ) )
            int alt7=2;
            int LA7_0 = input.LA(1);

            if ( (LA7_0==RULE_ID) ) {
                int LA7_1 = input.LA(2);

                if ( (LA7_1==34) ) {
                    alt7=2;
                }
                else if ( (LA7_1==EOF||LA7_1==17||LA7_1==32) ) {
                    alt7=1;
                }
                else {
                    NoViableAltException nvae =
                        new NoViableAltException("", 7, 1, input);

                    throw nvae;
                }
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
                     before(grammarAccess.getPortAccess().getIDTerminalRuleCall_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getPortAccess().getIDTerminalRuleCall_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:789:2: ( ( rule__Port__Group_1__0 ) )
                    {
                    // InternalLinguaFranca.g:789:2: ( ( rule__Port__Group_1__0 ) )
                    // InternalLinguaFranca.g:790:3: ( rule__Port__Group_1__0 )
                    {
                     before(grammarAccess.getPortAccess().getGroup_1()); 
                    // InternalLinguaFranca.g:791:3: ( rule__Port__Group_1__0 )
                    // InternalLinguaFranca.g:791:4: rule__Port__Group_1__0
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
    // InternalLinguaFranca.g:799:1: rule__Port__Alternatives_1_2 : ( ( RULE_ID ) | ( 'input' ) | ( 'output' ) );
    public final void rule__Port__Alternatives_1_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:803:1: ( ( RULE_ID ) | ( 'input' ) | ( 'output' ) )
            int alt8=3;
            switch ( input.LA(1) ) {
            case RULE_ID:
                {
                alt8=1;
                }
                break;
            case 13:
                {
                alt8=2;
                }
                break;
            case 14:
                {
                alt8=3;
                }
                break;
            default:
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
                     before(grammarAccess.getPortAccess().getIDTerminalRuleCall_1_2_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getPortAccess().getIDTerminalRuleCall_1_2_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:810:2: ( 'input' )
                    {
                    // InternalLinguaFranca.g:810:2: ( 'input' )
                    // InternalLinguaFranca.g:811:3: 'input'
                    {
                     before(grammarAccess.getPortAccess().getInputKeyword_1_2_1()); 
                    match(input,13,FOLLOW_2); 
                     after(grammarAccess.getPortAccess().getInputKeyword_1_2_1()); 

                    }


                    }
                    break;
                case 3 :
                    // InternalLinguaFranca.g:816:2: ( 'output' )
                    {
                    // InternalLinguaFranca.g:816:2: ( 'output' )
                    // InternalLinguaFranca.g:817:3: 'output'
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
    // InternalLinguaFranca.g:826:1: rule__Type__Alternatives : ( ( RULE_ID ) | ( RULE_CODE ) );
    public final void rule__Type__Alternatives() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:830:1: ( ( RULE_ID ) | ( RULE_CODE ) )
            int alt9=2;
            int LA9_0 = input.LA(1);

            if ( (LA9_0==RULE_ID) ) {
                alt9=1;
            }
            else if ( (LA9_0==RULE_CODE) ) {
                alt9=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 9, 0, input);

                throw nvae;
            }
            switch (alt9) {
                case 1 :
                    // InternalLinguaFranca.g:831:2: ( RULE_ID )
                    {
                    // InternalLinguaFranca.g:831:2: ( RULE_ID )
                    // InternalLinguaFranca.g:832:3: RULE_ID
                    {
                     before(grammarAccess.getTypeAccess().getIDTerminalRuleCall_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getTypeAccess().getIDTerminalRuleCall_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:837:2: ( RULE_CODE )
                    {
                    // InternalLinguaFranca.g:837:2: ( RULE_CODE )
                    // InternalLinguaFranca.g:838:3: RULE_CODE
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
    // InternalLinguaFranca.g:847:1: rule__Value__Alternatives : ( ( RULE_ID ) | ( RULE_NUMBER ) | ( RULE_STRING ) | ( RULE_CODE ) );
    public final void rule__Value__Alternatives() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:851:1: ( ( RULE_ID ) | ( RULE_NUMBER ) | ( RULE_STRING ) | ( RULE_CODE ) )
            int alt10=4;
            switch ( input.LA(1) ) {
            case RULE_ID:
                {
                alt10=1;
                }
                break;
            case RULE_NUMBER:
                {
                alt10=2;
                }
                break;
            case RULE_STRING:
                {
                alt10=3;
                }
                break;
            case RULE_CODE:
                {
                alt10=4;
                }
                break;
            default:
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
                     before(grammarAccess.getValueAccess().getIDTerminalRuleCall_0()); 
                    match(input,RULE_ID,FOLLOW_2); 
                     after(grammarAccess.getValueAccess().getIDTerminalRuleCall_0()); 

                    }


                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:858:2: ( RULE_NUMBER )
                    {
                    // InternalLinguaFranca.g:858:2: ( RULE_NUMBER )
                    // InternalLinguaFranca.g:859:3: RULE_NUMBER
                    {
                     before(grammarAccess.getValueAccess().getNUMBERTerminalRuleCall_1()); 
                    match(input,RULE_NUMBER,FOLLOW_2); 
                     after(grammarAccess.getValueAccess().getNUMBERTerminalRuleCall_1()); 

                    }


                    }
                    break;
                case 3 :
                    // InternalLinguaFranca.g:864:2: ( RULE_STRING )
                    {
                    // InternalLinguaFranca.g:864:2: ( RULE_STRING )
                    // InternalLinguaFranca.g:865:3: RULE_STRING
                    {
                     before(grammarAccess.getValueAccess().getSTRINGTerminalRuleCall_2()); 
                    match(input,RULE_STRING,FOLLOW_2); 
                     after(grammarAccess.getValueAccess().getSTRINGTerminalRuleCall_2()); 

                    }


                    }
                    break;
                case 4 :
                    // InternalLinguaFranca.g:870:2: ( RULE_CODE )
                    {
                    // InternalLinguaFranca.g:870:2: ( RULE_CODE )
                    // InternalLinguaFranca.g:871:3: RULE_CODE
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
    // InternalLinguaFranca.g:880:1: rule__Model__Group__0 : rule__Model__Group__0__Impl rule__Model__Group__1 ;
    public final void rule__Model__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:884:1: ( rule__Model__Group__0__Impl rule__Model__Group__1 )
            // InternalLinguaFranca.g:885:2: rule__Model__Group__0__Impl rule__Model__Group__1
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
    // InternalLinguaFranca.g:892:1: rule__Model__Group__0__Impl : ( ( rule__Model__TargetAssignment_0 ) ) ;
    public final void rule__Model__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:896:1: ( ( ( rule__Model__TargetAssignment_0 ) ) )
            // InternalLinguaFranca.g:897:1: ( ( rule__Model__TargetAssignment_0 ) )
            {
            // InternalLinguaFranca.g:897:1: ( ( rule__Model__TargetAssignment_0 ) )
            // InternalLinguaFranca.g:898:2: ( rule__Model__TargetAssignment_0 )
            {
             before(grammarAccess.getModelAccess().getTargetAssignment_0()); 
            // InternalLinguaFranca.g:899:2: ( rule__Model__TargetAssignment_0 )
            // InternalLinguaFranca.g:899:3: rule__Model__TargetAssignment_0
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
    // InternalLinguaFranca.g:907:1: rule__Model__Group__1 : rule__Model__Group__1__Impl rule__Model__Group__2 ;
    public final void rule__Model__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:911:1: ( rule__Model__Group__1__Impl rule__Model__Group__2 )
            // InternalLinguaFranca.g:912:2: rule__Model__Group__1__Impl rule__Model__Group__2
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
    // InternalLinguaFranca.g:919:1: rule__Model__Group__1__Impl : ( ( rule__Model__ImportsAssignment_1 )* ) ;
    public final void rule__Model__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:923:1: ( ( ( rule__Model__ImportsAssignment_1 )* ) )
            // InternalLinguaFranca.g:924:1: ( ( rule__Model__ImportsAssignment_1 )* )
            {
            // InternalLinguaFranca.g:924:1: ( ( rule__Model__ImportsAssignment_1 )* )
            // InternalLinguaFranca.g:925:2: ( rule__Model__ImportsAssignment_1 )*
            {
             before(grammarAccess.getModelAccess().getImportsAssignment_1()); 
            // InternalLinguaFranca.g:926:2: ( rule__Model__ImportsAssignment_1 )*
            loop11:
            do {
                int alt11=2;
                int LA11_0 = input.LA(1);

                if ( (LA11_0==18) ) {
                    alt11=1;
                }


                switch (alt11) {
            	case 1 :
            	    // InternalLinguaFranca.g:926:3: rule__Model__ImportsAssignment_1
            	    {
            	    pushFollow(FOLLOW_4);
            	    rule__Model__ImportsAssignment_1();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop11;
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
    // InternalLinguaFranca.g:934:1: rule__Model__Group__2 : rule__Model__Group__2__Impl ;
    public final void rule__Model__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:938:1: ( rule__Model__Group__2__Impl )
            // InternalLinguaFranca.g:939:2: rule__Model__Group__2__Impl
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
    // InternalLinguaFranca.g:945:1: rule__Model__Group__2__Impl : ( ( ( rule__Model__BlocksAssignment_2 ) ) ( ( rule__Model__BlocksAssignment_2 )* ) ) ;
    public final void rule__Model__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:949:1: ( ( ( ( rule__Model__BlocksAssignment_2 ) ) ( ( rule__Model__BlocksAssignment_2 )* ) ) )
            // InternalLinguaFranca.g:950:1: ( ( ( rule__Model__BlocksAssignment_2 ) ) ( ( rule__Model__BlocksAssignment_2 )* ) )
            {
            // InternalLinguaFranca.g:950:1: ( ( ( rule__Model__BlocksAssignment_2 ) ) ( ( rule__Model__BlocksAssignment_2 )* ) )
            // InternalLinguaFranca.g:951:2: ( ( rule__Model__BlocksAssignment_2 ) ) ( ( rule__Model__BlocksAssignment_2 )* )
            {
            // InternalLinguaFranca.g:951:2: ( ( rule__Model__BlocksAssignment_2 ) )
            // InternalLinguaFranca.g:952:3: ( rule__Model__BlocksAssignment_2 )
            {
             before(grammarAccess.getModelAccess().getBlocksAssignment_2()); 
            // InternalLinguaFranca.g:953:3: ( rule__Model__BlocksAssignment_2 )
            // InternalLinguaFranca.g:953:4: rule__Model__BlocksAssignment_2
            {
            pushFollow(FOLLOW_5);
            rule__Model__BlocksAssignment_2();

            state._fsp--;


            }

             after(grammarAccess.getModelAccess().getBlocksAssignment_2()); 

            }

            // InternalLinguaFranca.g:956:2: ( ( rule__Model__BlocksAssignment_2 )* )
            // InternalLinguaFranca.g:957:3: ( rule__Model__BlocksAssignment_2 )*
            {
             before(grammarAccess.getModelAccess().getBlocksAssignment_2()); 
            // InternalLinguaFranca.g:958:3: ( rule__Model__BlocksAssignment_2 )*
            loop12:
            do {
                int alt12=2;
                int LA12_0 = input.LA(1);

                if ( (LA12_0==19||LA12_0==22) ) {
                    alt12=1;
                }


                switch (alt12) {
            	case 1 :
            	    // InternalLinguaFranca.g:958:4: rule__Model__BlocksAssignment_2
            	    {
            	    pushFollow(FOLLOW_5);
            	    rule__Model__BlocksAssignment_2();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop12;
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
    // InternalLinguaFranca.g:968:1: rule__Target__Group__0 : rule__Target__Group__0__Impl rule__Target__Group__1 ;
    public final void rule__Target__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:972:1: ( rule__Target__Group__0__Impl rule__Target__Group__1 )
            // InternalLinguaFranca.g:973:2: rule__Target__Group__0__Impl rule__Target__Group__1
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
    // InternalLinguaFranca.g:980:1: rule__Target__Group__0__Impl : ( 'target' ) ;
    public final void rule__Target__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:984:1: ( ( 'target' ) )
            // InternalLinguaFranca.g:985:1: ( 'target' )
            {
            // InternalLinguaFranca.g:985:1: ( 'target' )
            // InternalLinguaFranca.g:986:2: 'target'
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
    // InternalLinguaFranca.g:995:1: rule__Target__Group__1 : rule__Target__Group__1__Impl rule__Target__Group__2 ;
    public final void rule__Target__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:999:1: ( rule__Target__Group__1__Impl rule__Target__Group__2 )
            // InternalLinguaFranca.g:1000:2: rule__Target__Group__1__Impl rule__Target__Group__2
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
    // InternalLinguaFranca.g:1007:1: rule__Target__Group__1__Impl : ( ( rule__Target__NameAssignment_1 ) ) ;
    public final void rule__Target__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1011:1: ( ( ( rule__Target__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:1012:1: ( ( rule__Target__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:1012:1: ( ( rule__Target__NameAssignment_1 ) )
            // InternalLinguaFranca.g:1013:2: ( rule__Target__NameAssignment_1 )
            {
             before(grammarAccess.getTargetAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:1014:2: ( rule__Target__NameAssignment_1 )
            // InternalLinguaFranca.g:1014:3: rule__Target__NameAssignment_1
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
    // InternalLinguaFranca.g:1022:1: rule__Target__Group__2 : rule__Target__Group__2__Impl ;
    public final void rule__Target__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1026:1: ( rule__Target__Group__2__Impl )
            // InternalLinguaFranca.g:1027:2: rule__Target__Group__2__Impl
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
    // InternalLinguaFranca.g:1033:1: rule__Target__Group__2__Impl : ( ';' ) ;
    public final void rule__Target__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1037:1: ( ( ';' ) )
            // InternalLinguaFranca.g:1038:1: ( ';' )
            {
            // InternalLinguaFranca.g:1038:1: ( ';' )
            // InternalLinguaFranca.g:1039:2: ';'
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
    // InternalLinguaFranca.g:1049:1: rule__Import__Group__0 : rule__Import__Group__0__Impl rule__Import__Group__1 ;
    public final void rule__Import__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1053:1: ( rule__Import__Group__0__Impl rule__Import__Group__1 )
            // InternalLinguaFranca.g:1054:2: rule__Import__Group__0__Impl rule__Import__Group__1
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
    // InternalLinguaFranca.g:1061:1: rule__Import__Group__0__Impl : ( 'import' ) ;
    public final void rule__Import__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1065:1: ( ( 'import' ) )
            // InternalLinguaFranca.g:1066:1: ( 'import' )
            {
            // InternalLinguaFranca.g:1066:1: ( 'import' )
            // InternalLinguaFranca.g:1067:2: 'import'
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
    // InternalLinguaFranca.g:1076:1: rule__Import__Group__1 : rule__Import__Group__1__Impl rule__Import__Group__2 ;
    public final void rule__Import__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1080:1: ( rule__Import__Group__1__Impl rule__Import__Group__2 )
            // InternalLinguaFranca.g:1081:2: rule__Import__Group__1__Impl rule__Import__Group__2
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
    // InternalLinguaFranca.g:1088:1: rule__Import__Group__1__Impl : ( ( rule__Import__NameAssignment_1 ) ) ;
    public final void rule__Import__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1092:1: ( ( ( rule__Import__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:1093:1: ( ( rule__Import__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:1093:1: ( ( rule__Import__NameAssignment_1 ) )
            // InternalLinguaFranca.g:1094:2: ( rule__Import__NameAssignment_1 )
            {
             before(grammarAccess.getImportAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:1095:2: ( rule__Import__NameAssignment_1 )
            // InternalLinguaFranca.g:1095:3: rule__Import__NameAssignment_1
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
    // InternalLinguaFranca.g:1103:1: rule__Import__Group__2 : rule__Import__Group__2__Impl ;
    public final void rule__Import__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1107:1: ( rule__Import__Group__2__Impl )
            // InternalLinguaFranca.g:1108:2: rule__Import__Group__2__Impl
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
    // InternalLinguaFranca.g:1114:1: rule__Import__Group__2__Impl : ( ';' ) ;
    public final void rule__Import__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1118:1: ( ( ';' ) )
            // InternalLinguaFranca.g:1119:1: ( ';' )
            {
            // InternalLinguaFranca.g:1119:1: ( ';' )
            // InternalLinguaFranca.g:1120:2: ';'
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


    // $ANTLR start "rule__Actor__Group__0"
    // InternalLinguaFranca.g:1130:1: rule__Actor__Group__0 : rule__Actor__Group__0__Impl rule__Actor__Group__1 ;
    public final void rule__Actor__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1134:1: ( rule__Actor__Group__0__Impl rule__Actor__Group__1 )
            // InternalLinguaFranca.g:1135:2: rule__Actor__Group__0__Impl rule__Actor__Group__1
            {
            pushFollow(FOLLOW_6);
            rule__Actor__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Actor__Group__1();

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
    // $ANTLR end "rule__Actor__Group__0"


    // $ANTLR start "rule__Actor__Group__0__Impl"
    // InternalLinguaFranca.g:1142:1: rule__Actor__Group__0__Impl : ( 'actor' ) ;
    public final void rule__Actor__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1146:1: ( ( 'actor' ) )
            // InternalLinguaFranca.g:1147:1: ( 'actor' )
            {
            // InternalLinguaFranca.g:1147:1: ( 'actor' )
            // InternalLinguaFranca.g:1148:2: 'actor'
            {
             before(grammarAccess.getActorAccess().getActorKeyword_0()); 
            match(input,19,FOLLOW_2); 
             after(grammarAccess.getActorAccess().getActorKeyword_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Actor__Group__0__Impl"


    // $ANTLR start "rule__Actor__Group__1"
    // InternalLinguaFranca.g:1157:1: rule__Actor__Group__1 : rule__Actor__Group__1__Impl rule__Actor__Group__2 ;
    public final void rule__Actor__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1161:1: ( rule__Actor__Group__1__Impl rule__Actor__Group__2 )
            // InternalLinguaFranca.g:1162:2: rule__Actor__Group__1__Impl rule__Actor__Group__2
            {
            pushFollow(FOLLOW_8);
            rule__Actor__Group__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Actor__Group__2();

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
    // $ANTLR end "rule__Actor__Group__1"


    // $ANTLR start "rule__Actor__Group__1__Impl"
    // InternalLinguaFranca.g:1169:1: rule__Actor__Group__1__Impl : ( ( rule__Actor__NameAssignment_1 ) ) ;
    public final void rule__Actor__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1173:1: ( ( ( rule__Actor__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:1174:1: ( ( rule__Actor__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:1174:1: ( ( rule__Actor__NameAssignment_1 ) )
            // InternalLinguaFranca.g:1175:2: ( rule__Actor__NameAssignment_1 )
            {
             before(grammarAccess.getActorAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:1176:2: ( rule__Actor__NameAssignment_1 )
            // InternalLinguaFranca.g:1176:3: rule__Actor__NameAssignment_1
            {
            pushFollow(FOLLOW_2);
            rule__Actor__NameAssignment_1();

            state._fsp--;


            }

             after(grammarAccess.getActorAccess().getNameAssignment_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Actor__Group__1__Impl"


    // $ANTLR start "rule__Actor__Group__2"
    // InternalLinguaFranca.g:1184:1: rule__Actor__Group__2 : rule__Actor__Group__2__Impl rule__Actor__Group__3 ;
    public final void rule__Actor__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1188:1: ( rule__Actor__Group__2__Impl rule__Actor__Group__3 )
            // InternalLinguaFranca.g:1189:2: rule__Actor__Group__2__Impl rule__Actor__Group__3
            {
            pushFollow(FOLLOW_8);
            rule__Actor__Group__2__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Actor__Group__3();

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
    // $ANTLR end "rule__Actor__Group__2"


    // $ANTLR start "rule__Actor__Group__2__Impl"
    // InternalLinguaFranca.g:1196:1: rule__Actor__Group__2__Impl : ( ( rule__Actor__ParametersAssignment_2 )? ) ;
    public final void rule__Actor__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1200:1: ( ( ( rule__Actor__ParametersAssignment_2 )? ) )
            // InternalLinguaFranca.g:1201:1: ( ( rule__Actor__ParametersAssignment_2 )? )
            {
            // InternalLinguaFranca.g:1201:1: ( ( rule__Actor__ParametersAssignment_2 )? )
            // InternalLinguaFranca.g:1202:2: ( rule__Actor__ParametersAssignment_2 )?
            {
             before(grammarAccess.getActorAccess().getParametersAssignment_2()); 
            // InternalLinguaFranca.g:1203:2: ( rule__Actor__ParametersAssignment_2 )?
            int alt13=2;
            int LA13_0 = input.LA(1);

            if ( (LA13_0==25) ) {
                alt13=1;
            }
            switch (alt13) {
                case 1 :
                    // InternalLinguaFranca.g:1203:3: rule__Actor__ParametersAssignment_2
                    {
                    pushFollow(FOLLOW_2);
                    rule__Actor__ParametersAssignment_2();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getActorAccess().getParametersAssignment_2()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Actor__Group__2__Impl"


    // $ANTLR start "rule__Actor__Group__3"
    // InternalLinguaFranca.g:1211:1: rule__Actor__Group__3 : rule__Actor__Group__3__Impl rule__Actor__Group__4 ;
    public final void rule__Actor__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1215:1: ( rule__Actor__Group__3__Impl rule__Actor__Group__4 )
            // InternalLinguaFranca.g:1216:2: rule__Actor__Group__3__Impl rule__Actor__Group__4
            {
            pushFollow(FOLLOW_9);
            rule__Actor__Group__3__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Actor__Group__4();

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
    // $ANTLR end "rule__Actor__Group__3"


    // $ANTLR start "rule__Actor__Group__3__Impl"
    // InternalLinguaFranca.g:1223:1: rule__Actor__Group__3__Impl : ( '{' ) ;
    public final void rule__Actor__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1227:1: ( ( '{' ) )
            // InternalLinguaFranca.g:1228:1: ( '{' )
            {
            // InternalLinguaFranca.g:1228:1: ( '{' )
            // InternalLinguaFranca.g:1229:2: '{'
            {
             before(grammarAccess.getActorAccess().getLeftCurlyBracketKeyword_3()); 
            match(input,20,FOLLOW_2); 
             after(grammarAccess.getActorAccess().getLeftCurlyBracketKeyword_3()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Actor__Group__3__Impl"


    // $ANTLR start "rule__Actor__Group__4"
    // InternalLinguaFranca.g:1238:1: rule__Actor__Group__4 : rule__Actor__Group__4__Impl rule__Actor__Group__5 ;
    public final void rule__Actor__Group__4() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1242:1: ( rule__Actor__Group__4__Impl rule__Actor__Group__5 )
            // InternalLinguaFranca.g:1243:2: rule__Actor__Group__4__Impl rule__Actor__Group__5
            {
            pushFollow(FOLLOW_9);
            rule__Actor__Group__4__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Actor__Group__5();

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
    // $ANTLR end "rule__Actor__Group__4"


    // $ANTLR start "rule__Actor__Group__4__Impl"
    // InternalLinguaFranca.g:1250:1: rule__Actor__Group__4__Impl : ( ( rule__Actor__InputsAssignment_4 )* ) ;
    public final void rule__Actor__Group__4__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1254:1: ( ( ( rule__Actor__InputsAssignment_4 )* ) )
            // InternalLinguaFranca.g:1255:1: ( ( rule__Actor__InputsAssignment_4 )* )
            {
            // InternalLinguaFranca.g:1255:1: ( ( rule__Actor__InputsAssignment_4 )* )
            // InternalLinguaFranca.g:1256:2: ( rule__Actor__InputsAssignment_4 )*
            {
             before(grammarAccess.getActorAccess().getInputsAssignment_4()); 
            // InternalLinguaFranca.g:1257:2: ( rule__Actor__InputsAssignment_4 )*
            loop14:
            do {
                int alt14=2;
                int LA14_0 = input.LA(1);

                if ( (LA14_0==13) ) {
                    alt14=1;
                }


                switch (alt14) {
            	case 1 :
            	    // InternalLinguaFranca.g:1257:3: rule__Actor__InputsAssignment_4
            	    {
            	    pushFollow(FOLLOW_10);
            	    rule__Actor__InputsAssignment_4();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop14;
                }
            } while (true);

             after(grammarAccess.getActorAccess().getInputsAssignment_4()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Actor__Group__4__Impl"


    // $ANTLR start "rule__Actor__Group__5"
    // InternalLinguaFranca.g:1265:1: rule__Actor__Group__5 : rule__Actor__Group__5__Impl rule__Actor__Group__6 ;
    public final void rule__Actor__Group__5() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1269:1: ( rule__Actor__Group__5__Impl rule__Actor__Group__6 )
            // InternalLinguaFranca.g:1270:2: rule__Actor__Group__5__Impl rule__Actor__Group__6
            {
            pushFollow(FOLLOW_9);
            rule__Actor__Group__5__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Actor__Group__6();

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
    // $ANTLR end "rule__Actor__Group__5"


    // $ANTLR start "rule__Actor__Group__5__Impl"
    // InternalLinguaFranca.g:1277:1: rule__Actor__Group__5__Impl : ( ( rule__Actor__OutputsAssignment_5 )* ) ;
    public final void rule__Actor__Group__5__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1281:1: ( ( ( rule__Actor__OutputsAssignment_5 )* ) )
            // InternalLinguaFranca.g:1282:1: ( ( rule__Actor__OutputsAssignment_5 )* )
            {
            // InternalLinguaFranca.g:1282:1: ( ( rule__Actor__OutputsAssignment_5 )* )
            // InternalLinguaFranca.g:1283:2: ( rule__Actor__OutputsAssignment_5 )*
            {
             before(grammarAccess.getActorAccess().getOutputsAssignment_5()); 
            // InternalLinguaFranca.g:1284:2: ( rule__Actor__OutputsAssignment_5 )*
            loop15:
            do {
                int alt15=2;
                int LA15_0 = input.LA(1);

                if ( (LA15_0==14) ) {
                    alt15=1;
                }


                switch (alt15) {
            	case 1 :
            	    // InternalLinguaFranca.g:1284:3: rule__Actor__OutputsAssignment_5
            	    {
            	    pushFollow(FOLLOW_11);
            	    rule__Actor__OutputsAssignment_5();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop15;
                }
            } while (true);

             after(grammarAccess.getActorAccess().getOutputsAssignment_5()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Actor__Group__5__Impl"


    // $ANTLR start "rule__Actor__Group__6"
    // InternalLinguaFranca.g:1292:1: rule__Actor__Group__6 : rule__Actor__Group__6__Impl rule__Actor__Group__7 ;
    public final void rule__Actor__Group__6() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1296:1: ( rule__Actor__Group__6__Impl rule__Actor__Group__7 )
            // InternalLinguaFranca.g:1297:2: rule__Actor__Group__6__Impl rule__Actor__Group__7
            {
            pushFollow(FOLLOW_9);
            rule__Actor__Group__6__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Actor__Group__7();

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
    // $ANTLR end "rule__Actor__Group__6"


    // $ANTLR start "rule__Actor__Group__6__Impl"
    // InternalLinguaFranca.g:1304:1: rule__Actor__Group__6__Impl : ( ( rule__Actor__TriggersAssignment_6 )* ) ;
    public final void rule__Actor__Group__6__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1308:1: ( ( ( rule__Actor__TriggersAssignment_6 )* ) )
            // InternalLinguaFranca.g:1309:1: ( ( rule__Actor__TriggersAssignment_6 )* )
            {
            // InternalLinguaFranca.g:1309:1: ( ( rule__Actor__TriggersAssignment_6 )* )
            // InternalLinguaFranca.g:1310:2: ( rule__Actor__TriggersAssignment_6 )*
            {
             before(grammarAccess.getActorAccess().getTriggersAssignment_6()); 
            // InternalLinguaFranca.g:1311:2: ( rule__Actor__TriggersAssignment_6 )*
            loop16:
            do {
                int alt16=2;
                int LA16_0 = input.LA(1);

                if ( (LA16_0==15) ) {
                    alt16=1;
                }


                switch (alt16) {
            	case 1 :
            	    // InternalLinguaFranca.g:1311:3: rule__Actor__TriggersAssignment_6
            	    {
            	    pushFollow(FOLLOW_12);
            	    rule__Actor__TriggersAssignment_6();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop16;
                }
            } while (true);

             after(grammarAccess.getActorAccess().getTriggersAssignment_6()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Actor__Group__6__Impl"


    // $ANTLR start "rule__Actor__Group__7"
    // InternalLinguaFranca.g:1319:1: rule__Actor__Group__7 : rule__Actor__Group__7__Impl rule__Actor__Group__8 ;
    public final void rule__Actor__Group__7() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1323:1: ( rule__Actor__Group__7__Impl rule__Actor__Group__8 )
            // InternalLinguaFranca.g:1324:2: rule__Actor__Group__7__Impl rule__Actor__Group__8
            {
            pushFollow(FOLLOW_9);
            rule__Actor__Group__7__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Actor__Group__8();

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
    // $ANTLR end "rule__Actor__Group__7"


    // $ANTLR start "rule__Actor__Group__7__Impl"
    // InternalLinguaFranca.g:1331:1: rule__Actor__Group__7__Impl : ( ( rule__Actor__PreambleAssignment_7 )? ) ;
    public final void rule__Actor__Group__7__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1335:1: ( ( ( rule__Actor__PreambleAssignment_7 )? ) )
            // InternalLinguaFranca.g:1336:1: ( ( rule__Actor__PreambleAssignment_7 )? )
            {
            // InternalLinguaFranca.g:1336:1: ( ( rule__Actor__PreambleAssignment_7 )? )
            // InternalLinguaFranca.g:1337:2: ( rule__Actor__PreambleAssignment_7 )?
            {
             before(grammarAccess.getActorAccess().getPreambleAssignment_7()); 
            // InternalLinguaFranca.g:1338:2: ( rule__Actor__PreambleAssignment_7 )?
            int alt17=2;
            int LA17_0 = input.LA(1);

            if ( (LA17_0==28) ) {
                alt17=1;
            }
            switch (alt17) {
                case 1 :
                    // InternalLinguaFranca.g:1338:3: rule__Actor__PreambleAssignment_7
                    {
                    pushFollow(FOLLOW_2);
                    rule__Actor__PreambleAssignment_7();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getActorAccess().getPreambleAssignment_7()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Actor__Group__7__Impl"


    // $ANTLR start "rule__Actor__Group__8"
    // InternalLinguaFranca.g:1346:1: rule__Actor__Group__8 : rule__Actor__Group__8__Impl rule__Actor__Group__9 ;
    public final void rule__Actor__Group__8() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1350:1: ( rule__Actor__Group__8__Impl rule__Actor__Group__9 )
            // InternalLinguaFranca.g:1351:2: rule__Actor__Group__8__Impl rule__Actor__Group__9
            {
            pushFollow(FOLLOW_9);
            rule__Actor__Group__8__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Actor__Group__9();

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
    // $ANTLR end "rule__Actor__Group__8"


    // $ANTLR start "rule__Actor__Group__8__Impl"
    // InternalLinguaFranca.g:1358:1: rule__Actor__Group__8__Impl : ( ( rule__Actor__InitializeAssignment_8 )? ) ;
    public final void rule__Actor__Group__8__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1362:1: ( ( ( rule__Actor__InitializeAssignment_8 )? ) )
            // InternalLinguaFranca.g:1363:1: ( ( rule__Actor__InitializeAssignment_8 )? )
            {
            // InternalLinguaFranca.g:1363:1: ( ( rule__Actor__InitializeAssignment_8 )? )
            // InternalLinguaFranca.g:1364:2: ( rule__Actor__InitializeAssignment_8 )?
            {
             before(grammarAccess.getActorAccess().getInitializeAssignment_8()); 
            // InternalLinguaFranca.g:1365:2: ( rule__Actor__InitializeAssignment_8 )?
            int alt18=2;
            int LA18_0 = input.LA(1);

            if ( (LA18_0==29) ) {
                alt18=1;
            }
            switch (alt18) {
                case 1 :
                    // InternalLinguaFranca.g:1365:3: rule__Actor__InitializeAssignment_8
                    {
                    pushFollow(FOLLOW_2);
                    rule__Actor__InitializeAssignment_8();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getActorAccess().getInitializeAssignment_8()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Actor__Group__8__Impl"


    // $ANTLR start "rule__Actor__Group__9"
    // InternalLinguaFranca.g:1373:1: rule__Actor__Group__9 : rule__Actor__Group__9__Impl rule__Actor__Group__10 ;
    public final void rule__Actor__Group__9() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1377:1: ( rule__Actor__Group__9__Impl rule__Actor__Group__10 )
            // InternalLinguaFranca.g:1378:2: rule__Actor__Group__9__Impl rule__Actor__Group__10
            {
            pushFollow(FOLLOW_9);
            rule__Actor__Group__9__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Actor__Group__10();

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
    // $ANTLR end "rule__Actor__Group__9"


    // $ANTLR start "rule__Actor__Group__9__Impl"
    // InternalLinguaFranca.g:1385:1: rule__Actor__Group__9__Impl : ( ( rule__Actor__ReactionsAssignment_9 )* ) ;
    public final void rule__Actor__Group__9__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1389:1: ( ( ( rule__Actor__ReactionsAssignment_9 )* ) )
            // InternalLinguaFranca.g:1390:1: ( ( rule__Actor__ReactionsAssignment_9 )* )
            {
            // InternalLinguaFranca.g:1390:1: ( ( rule__Actor__ReactionsAssignment_9 )* )
            // InternalLinguaFranca.g:1391:2: ( rule__Actor__ReactionsAssignment_9 )*
            {
             before(grammarAccess.getActorAccess().getReactionsAssignment_9()); 
            // InternalLinguaFranca.g:1392:2: ( rule__Actor__ReactionsAssignment_9 )*
            loop19:
            do {
                int alt19=2;
                int LA19_0 = input.LA(1);

                if ( (LA19_0==24) ) {
                    alt19=1;
                }


                switch (alt19) {
            	case 1 :
            	    // InternalLinguaFranca.g:1392:3: rule__Actor__ReactionsAssignment_9
            	    {
            	    pushFollow(FOLLOW_13);
            	    rule__Actor__ReactionsAssignment_9();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop19;
                }
            } while (true);

             after(grammarAccess.getActorAccess().getReactionsAssignment_9()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Actor__Group__9__Impl"


    // $ANTLR start "rule__Actor__Group__10"
    // InternalLinguaFranca.g:1400:1: rule__Actor__Group__10 : rule__Actor__Group__10__Impl ;
    public final void rule__Actor__Group__10() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1404:1: ( rule__Actor__Group__10__Impl )
            // InternalLinguaFranca.g:1405:2: rule__Actor__Group__10__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Actor__Group__10__Impl();

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
    // $ANTLR end "rule__Actor__Group__10"


    // $ANTLR start "rule__Actor__Group__10__Impl"
    // InternalLinguaFranca.g:1411:1: rule__Actor__Group__10__Impl : ( '}' ) ;
    public final void rule__Actor__Group__10__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1415:1: ( ( '}' ) )
            // InternalLinguaFranca.g:1416:1: ( '}' )
            {
            // InternalLinguaFranca.g:1416:1: ( '}' )
            // InternalLinguaFranca.g:1417:2: '}'
            {
             before(grammarAccess.getActorAccess().getRightCurlyBracketKeyword_10()); 
            match(input,21,FOLLOW_2); 
             after(grammarAccess.getActorAccess().getRightCurlyBracketKeyword_10()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Actor__Group__10__Impl"


    // $ANTLR start "rule__Composite__Group__0"
    // InternalLinguaFranca.g:1427:1: rule__Composite__Group__0 : rule__Composite__Group__0__Impl rule__Composite__Group__1 ;
    public final void rule__Composite__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1431:1: ( rule__Composite__Group__0__Impl rule__Composite__Group__1 )
            // InternalLinguaFranca.g:1432:2: rule__Composite__Group__0__Impl rule__Composite__Group__1
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
    // InternalLinguaFranca.g:1439:1: rule__Composite__Group__0__Impl : ( 'composite' ) ;
    public final void rule__Composite__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1443:1: ( ( 'composite' ) )
            // InternalLinguaFranca.g:1444:1: ( 'composite' )
            {
            // InternalLinguaFranca.g:1444:1: ( 'composite' )
            // InternalLinguaFranca.g:1445:2: 'composite'
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
    // InternalLinguaFranca.g:1454:1: rule__Composite__Group__1 : rule__Composite__Group__1__Impl rule__Composite__Group__2 ;
    public final void rule__Composite__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1458:1: ( rule__Composite__Group__1__Impl rule__Composite__Group__2 )
            // InternalLinguaFranca.g:1459:2: rule__Composite__Group__1__Impl rule__Composite__Group__2
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
    // InternalLinguaFranca.g:1466:1: rule__Composite__Group__1__Impl : ( ( rule__Composite__NameAssignment_1 ) ) ;
    public final void rule__Composite__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1470:1: ( ( ( rule__Composite__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:1471:1: ( ( rule__Composite__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:1471:1: ( ( rule__Composite__NameAssignment_1 ) )
            // InternalLinguaFranca.g:1472:2: ( rule__Composite__NameAssignment_1 )
            {
             before(grammarAccess.getCompositeAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:1473:2: ( rule__Composite__NameAssignment_1 )
            // InternalLinguaFranca.g:1473:3: rule__Composite__NameAssignment_1
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
    // InternalLinguaFranca.g:1481:1: rule__Composite__Group__2 : rule__Composite__Group__2__Impl rule__Composite__Group__3 ;
    public final void rule__Composite__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1485:1: ( rule__Composite__Group__2__Impl rule__Composite__Group__3 )
            // InternalLinguaFranca.g:1486:2: rule__Composite__Group__2__Impl rule__Composite__Group__3
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
    // InternalLinguaFranca.g:1493:1: rule__Composite__Group__2__Impl : ( ( rule__Composite__ParametersAssignment_2 )? ) ;
    public final void rule__Composite__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1497:1: ( ( ( rule__Composite__ParametersAssignment_2 )? ) )
            // InternalLinguaFranca.g:1498:1: ( ( rule__Composite__ParametersAssignment_2 )? )
            {
            // InternalLinguaFranca.g:1498:1: ( ( rule__Composite__ParametersAssignment_2 )? )
            // InternalLinguaFranca.g:1499:2: ( rule__Composite__ParametersAssignment_2 )?
            {
             before(grammarAccess.getCompositeAccess().getParametersAssignment_2()); 
            // InternalLinguaFranca.g:1500:2: ( rule__Composite__ParametersAssignment_2 )?
            int alt20=2;
            int LA20_0 = input.LA(1);

            if ( (LA20_0==25) ) {
                alt20=1;
            }
            switch (alt20) {
                case 1 :
                    // InternalLinguaFranca.g:1500:3: rule__Composite__ParametersAssignment_2
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
    // InternalLinguaFranca.g:1508:1: rule__Composite__Group__3 : rule__Composite__Group__3__Impl rule__Composite__Group__4 ;
    public final void rule__Composite__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1512:1: ( rule__Composite__Group__3__Impl rule__Composite__Group__4 )
            // InternalLinguaFranca.g:1513:2: rule__Composite__Group__3__Impl rule__Composite__Group__4
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
    // InternalLinguaFranca.g:1520:1: rule__Composite__Group__3__Impl : ( '{' ) ;
    public final void rule__Composite__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1524:1: ( ( '{' ) )
            // InternalLinguaFranca.g:1525:1: ( '{' )
            {
            // InternalLinguaFranca.g:1525:1: ( '{' )
            // InternalLinguaFranca.g:1526:2: '{'
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
    // InternalLinguaFranca.g:1535:1: rule__Composite__Group__4 : rule__Composite__Group__4__Impl rule__Composite__Group__5 ;
    public final void rule__Composite__Group__4() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1539:1: ( rule__Composite__Group__4__Impl rule__Composite__Group__5 )
            // InternalLinguaFranca.g:1540:2: rule__Composite__Group__4__Impl rule__Composite__Group__5
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
    // InternalLinguaFranca.g:1547:1: rule__Composite__Group__4__Impl : ( ( rule__Composite__InputsAssignment_4 )* ) ;
    public final void rule__Composite__Group__4__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1551:1: ( ( ( rule__Composite__InputsAssignment_4 )* ) )
            // InternalLinguaFranca.g:1552:1: ( ( rule__Composite__InputsAssignment_4 )* )
            {
            // InternalLinguaFranca.g:1552:1: ( ( rule__Composite__InputsAssignment_4 )* )
            // InternalLinguaFranca.g:1553:2: ( rule__Composite__InputsAssignment_4 )*
            {
             before(grammarAccess.getCompositeAccess().getInputsAssignment_4()); 
            // InternalLinguaFranca.g:1554:2: ( rule__Composite__InputsAssignment_4 )*
            loop21:
            do {
                int alt21=2;
                int LA21_0 = input.LA(1);

                if ( (LA21_0==13) ) {
                    alt21=1;
                }


                switch (alt21) {
            	case 1 :
            	    // InternalLinguaFranca.g:1554:3: rule__Composite__InputsAssignment_4
            	    {
            	    pushFollow(FOLLOW_10);
            	    rule__Composite__InputsAssignment_4();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop21;
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
    // InternalLinguaFranca.g:1562:1: rule__Composite__Group__5 : rule__Composite__Group__5__Impl rule__Composite__Group__6 ;
    public final void rule__Composite__Group__5() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1566:1: ( rule__Composite__Group__5__Impl rule__Composite__Group__6 )
            // InternalLinguaFranca.g:1567:2: rule__Composite__Group__5__Impl rule__Composite__Group__6
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
    // InternalLinguaFranca.g:1574:1: rule__Composite__Group__5__Impl : ( ( rule__Composite__OutputsAssignment_5 )* ) ;
    public final void rule__Composite__Group__5__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1578:1: ( ( ( rule__Composite__OutputsAssignment_5 )* ) )
            // InternalLinguaFranca.g:1579:1: ( ( rule__Composite__OutputsAssignment_5 )* )
            {
            // InternalLinguaFranca.g:1579:1: ( ( rule__Composite__OutputsAssignment_5 )* )
            // InternalLinguaFranca.g:1580:2: ( rule__Composite__OutputsAssignment_5 )*
            {
             before(grammarAccess.getCompositeAccess().getOutputsAssignment_5()); 
            // InternalLinguaFranca.g:1581:2: ( rule__Composite__OutputsAssignment_5 )*
            loop22:
            do {
                int alt22=2;
                int LA22_0 = input.LA(1);

                if ( (LA22_0==14) ) {
                    alt22=1;
                }


                switch (alt22) {
            	case 1 :
            	    // InternalLinguaFranca.g:1581:3: rule__Composite__OutputsAssignment_5
            	    {
            	    pushFollow(FOLLOW_11);
            	    rule__Composite__OutputsAssignment_5();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop22;
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
    // InternalLinguaFranca.g:1589:1: rule__Composite__Group__6 : rule__Composite__Group__6__Impl rule__Composite__Group__7 ;
    public final void rule__Composite__Group__6() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1593:1: ( rule__Composite__Group__6__Impl rule__Composite__Group__7 )
            // InternalLinguaFranca.g:1594:2: rule__Composite__Group__6__Impl rule__Composite__Group__7
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
    // InternalLinguaFranca.g:1601:1: rule__Composite__Group__6__Impl : ( ( rule__Composite__TriggersAssignment_6 )* ) ;
    public final void rule__Composite__Group__6__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1605:1: ( ( ( rule__Composite__TriggersAssignment_6 )* ) )
            // InternalLinguaFranca.g:1606:1: ( ( rule__Composite__TriggersAssignment_6 )* )
            {
            // InternalLinguaFranca.g:1606:1: ( ( rule__Composite__TriggersAssignment_6 )* )
            // InternalLinguaFranca.g:1607:2: ( rule__Composite__TriggersAssignment_6 )*
            {
             before(grammarAccess.getCompositeAccess().getTriggersAssignment_6()); 
            // InternalLinguaFranca.g:1608:2: ( rule__Composite__TriggersAssignment_6 )*
            loop23:
            do {
                int alt23=2;
                int LA23_0 = input.LA(1);

                if ( (LA23_0==15) ) {
                    alt23=1;
                }


                switch (alt23) {
            	case 1 :
            	    // InternalLinguaFranca.g:1608:3: rule__Composite__TriggersAssignment_6
            	    {
            	    pushFollow(FOLLOW_12);
            	    rule__Composite__TriggersAssignment_6();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop23;
                }
            } while (true);

             after(grammarAccess.getCompositeAccess().getTriggersAssignment_6()); 

            }


            }

        }
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
    // InternalLinguaFranca.g:1616:1: rule__Composite__Group__7 : rule__Composite__Group__7__Impl rule__Composite__Group__8 ;
    public final void rule__Composite__Group__7() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1620:1: ( rule__Composite__Group__7__Impl rule__Composite__Group__8 )
            // InternalLinguaFranca.g:1621:2: rule__Composite__Group__7__Impl rule__Composite__Group__8
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
    // InternalLinguaFranca.g:1628:1: rule__Composite__Group__7__Impl : ( ( rule__Composite__PreambleAssignment_7 )? ) ;
    public final void rule__Composite__Group__7__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1632:1: ( ( ( rule__Composite__PreambleAssignment_7 )? ) )
            // InternalLinguaFranca.g:1633:1: ( ( rule__Composite__PreambleAssignment_7 )? )
            {
            // InternalLinguaFranca.g:1633:1: ( ( rule__Composite__PreambleAssignment_7 )? )
            // InternalLinguaFranca.g:1634:2: ( rule__Composite__PreambleAssignment_7 )?
            {
             before(grammarAccess.getCompositeAccess().getPreambleAssignment_7()); 
            // InternalLinguaFranca.g:1635:2: ( rule__Composite__PreambleAssignment_7 )?
            int alt24=2;
            int LA24_0 = input.LA(1);

            if ( (LA24_0==28) ) {
                alt24=1;
            }
            switch (alt24) {
                case 1 :
                    // InternalLinguaFranca.g:1635:3: rule__Composite__PreambleAssignment_7
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
    // InternalLinguaFranca.g:1643:1: rule__Composite__Group__8 : rule__Composite__Group__8__Impl rule__Composite__Group__9 ;
    public final void rule__Composite__Group__8() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1647:1: ( rule__Composite__Group__8__Impl rule__Composite__Group__9 )
            // InternalLinguaFranca.g:1648:2: rule__Composite__Group__8__Impl rule__Composite__Group__9
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
    // InternalLinguaFranca.g:1655:1: rule__Composite__Group__8__Impl : ( ( rule__Composite__InitializeAssignment_8 )? ) ;
    public final void rule__Composite__Group__8__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1659:1: ( ( ( rule__Composite__InitializeAssignment_8 )? ) )
            // InternalLinguaFranca.g:1660:1: ( ( rule__Composite__InitializeAssignment_8 )? )
            {
            // InternalLinguaFranca.g:1660:1: ( ( rule__Composite__InitializeAssignment_8 )? )
            // InternalLinguaFranca.g:1661:2: ( rule__Composite__InitializeAssignment_8 )?
            {
             before(grammarAccess.getCompositeAccess().getInitializeAssignment_8()); 
            // InternalLinguaFranca.g:1662:2: ( rule__Composite__InitializeAssignment_8 )?
            int alt25=2;
            int LA25_0 = input.LA(1);

            if ( (LA25_0==29) ) {
                alt25=1;
            }
            switch (alt25) {
                case 1 :
                    // InternalLinguaFranca.g:1662:3: rule__Composite__InitializeAssignment_8
                    {
                    pushFollow(FOLLOW_2);
                    rule__Composite__InitializeAssignment_8();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getCompositeAccess().getInitializeAssignment_8()); 

            }


            }

        }
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
    // InternalLinguaFranca.g:1670:1: rule__Composite__Group__9 : rule__Composite__Group__9__Impl rule__Composite__Group__10 ;
    public final void rule__Composite__Group__9() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1674:1: ( rule__Composite__Group__9__Impl rule__Composite__Group__10 )
            // InternalLinguaFranca.g:1675:2: rule__Composite__Group__9__Impl rule__Composite__Group__10
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
    // InternalLinguaFranca.g:1682:1: rule__Composite__Group__9__Impl : ( ( rule__Composite__ReactionsAssignment_9 )* ) ;
    public final void rule__Composite__Group__9__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1686:1: ( ( ( rule__Composite__ReactionsAssignment_9 )* ) )
            // InternalLinguaFranca.g:1687:1: ( ( rule__Composite__ReactionsAssignment_9 )* )
            {
            // InternalLinguaFranca.g:1687:1: ( ( rule__Composite__ReactionsAssignment_9 )* )
            // InternalLinguaFranca.g:1688:2: ( rule__Composite__ReactionsAssignment_9 )*
            {
             before(grammarAccess.getCompositeAccess().getReactionsAssignment_9()); 
            // InternalLinguaFranca.g:1689:2: ( rule__Composite__ReactionsAssignment_9 )*
            loop26:
            do {
                int alt26=2;
                int LA26_0 = input.LA(1);

                if ( (LA26_0==24) ) {
                    alt26=1;
                }


                switch (alt26) {
            	case 1 :
            	    // InternalLinguaFranca.g:1689:3: rule__Composite__ReactionsAssignment_9
            	    {
            	    pushFollow(FOLLOW_13);
            	    rule__Composite__ReactionsAssignment_9();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop26;
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
    // InternalLinguaFranca.g:1697:1: rule__Composite__Group__10 : rule__Composite__Group__10__Impl rule__Composite__Group__11 ;
    public final void rule__Composite__Group__10() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1701:1: ( rule__Composite__Group__10__Impl rule__Composite__Group__11 )
            // InternalLinguaFranca.g:1702:2: rule__Composite__Group__10__Impl rule__Composite__Group__11
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
    // InternalLinguaFranca.g:1709:1: rule__Composite__Group__10__Impl : ( ( rule__Composite__InstancesAssignment_10 )* ) ;
    public final void rule__Composite__Group__10__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1713:1: ( ( ( rule__Composite__InstancesAssignment_10 )* ) )
            // InternalLinguaFranca.g:1714:1: ( ( rule__Composite__InstancesAssignment_10 )* )
            {
            // InternalLinguaFranca.g:1714:1: ( ( rule__Composite__InstancesAssignment_10 )* )
            // InternalLinguaFranca.g:1715:2: ( rule__Composite__InstancesAssignment_10 )*
            {
             before(grammarAccess.getCompositeAccess().getInstancesAssignment_10()); 
            // InternalLinguaFranca.g:1716:2: ( rule__Composite__InstancesAssignment_10 )*
            loop27:
            do {
                int alt27=2;
                int LA27_0 = input.LA(1);

                if ( (LA27_0==30) ) {
                    alt27=1;
                }


                switch (alt27) {
            	case 1 :
            	    // InternalLinguaFranca.g:1716:3: rule__Composite__InstancesAssignment_10
            	    {
            	    pushFollow(FOLLOW_15);
            	    rule__Composite__InstancesAssignment_10();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop27;
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
    // InternalLinguaFranca.g:1724:1: rule__Composite__Group__11 : rule__Composite__Group__11__Impl rule__Composite__Group__12 ;
    public final void rule__Composite__Group__11() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1728:1: ( rule__Composite__Group__11__Impl rule__Composite__Group__12 )
            // InternalLinguaFranca.g:1729:2: rule__Composite__Group__11__Impl rule__Composite__Group__12
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
    // InternalLinguaFranca.g:1736:1: rule__Composite__Group__11__Impl : ( ( rule__Composite__ConnectionsAssignment_11 )* ) ;
    public final void rule__Composite__Group__11__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1740:1: ( ( ( rule__Composite__ConnectionsAssignment_11 )* ) )
            // InternalLinguaFranca.g:1741:1: ( ( rule__Composite__ConnectionsAssignment_11 )* )
            {
            // InternalLinguaFranca.g:1741:1: ( ( rule__Composite__ConnectionsAssignment_11 )* )
            // InternalLinguaFranca.g:1742:2: ( rule__Composite__ConnectionsAssignment_11 )*
            {
             before(grammarAccess.getCompositeAccess().getConnectionsAssignment_11()); 
            // InternalLinguaFranca.g:1743:2: ( rule__Composite__ConnectionsAssignment_11 )*
            loop28:
            do {
                int alt28=2;
                int LA28_0 = input.LA(1);

                if ( (LA28_0==RULE_ID) ) {
                    alt28=1;
                }


                switch (alt28) {
            	case 1 :
            	    // InternalLinguaFranca.g:1743:3: rule__Composite__ConnectionsAssignment_11
            	    {
            	    pushFollow(FOLLOW_16);
            	    rule__Composite__ConnectionsAssignment_11();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop28;
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
    // InternalLinguaFranca.g:1751:1: rule__Composite__Group__12 : rule__Composite__Group__12__Impl ;
    public final void rule__Composite__Group__12() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1755:1: ( rule__Composite__Group__12__Impl )
            // InternalLinguaFranca.g:1756:2: rule__Composite__Group__12__Impl
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
    // InternalLinguaFranca.g:1762:1: rule__Composite__Group__12__Impl : ( '}' ) ;
    public final void rule__Composite__Group__12__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1766:1: ( ( '}' ) )
            // InternalLinguaFranca.g:1767:1: ( '}' )
            {
            // InternalLinguaFranca.g:1767:1: ( '}' )
            // InternalLinguaFranca.g:1768:2: '}'
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
    // InternalLinguaFranca.g:1778:1: rule__Input__Group__0 : rule__Input__Group__0__Impl rule__Input__Group__1 ;
    public final void rule__Input__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1782:1: ( rule__Input__Group__0__Impl rule__Input__Group__1 )
            // InternalLinguaFranca.g:1783:2: rule__Input__Group__0__Impl rule__Input__Group__1
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
    // InternalLinguaFranca.g:1790:1: rule__Input__Group__0__Impl : ( 'input' ) ;
    public final void rule__Input__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1794:1: ( ( 'input' ) )
            // InternalLinguaFranca.g:1795:1: ( 'input' )
            {
            // InternalLinguaFranca.g:1795:1: ( 'input' )
            // InternalLinguaFranca.g:1796:2: 'input'
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
    // InternalLinguaFranca.g:1805:1: rule__Input__Group__1 : rule__Input__Group__1__Impl rule__Input__Group__2 ;
    public final void rule__Input__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1809:1: ( rule__Input__Group__1__Impl rule__Input__Group__2 )
            // InternalLinguaFranca.g:1810:2: rule__Input__Group__1__Impl rule__Input__Group__2
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
    // InternalLinguaFranca.g:1817:1: rule__Input__Group__1__Impl : ( ( rule__Input__NameAssignment_1 ) ) ;
    public final void rule__Input__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1821:1: ( ( ( rule__Input__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:1822:1: ( ( rule__Input__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:1822:1: ( ( rule__Input__NameAssignment_1 ) )
            // InternalLinguaFranca.g:1823:2: ( rule__Input__NameAssignment_1 )
            {
             before(grammarAccess.getInputAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:1824:2: ( rule__Input__NameAssignment_1 )
            // InternalLinguaFranca.g:1824:3: rule__Input__NameAssignment_1
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
    // InternalLinguaFranca.g:1832:1: rule__Input__Group__2 : rule__Input__Group__2__Impl rule__Input__Group__3 ;
    public final void rule__Input__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1836:1: ( rule__Input__Group__2__Impl rule__Input__Group__3 )
            // InternalLinguaFranca.g:1837:2: rule__Input__Group__2__Impl rule__Input__Group__3
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
    // InternalLinguaFranca.g:1844:1: rule__Input__Group__2__Impl : ( ( rule__Input__Group_2__0 )? ) ;
    public final void rule__Input__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1848:1: ( ( ( rule__Input__Group_2__0 )? ) )
            // InternalLinguaFranca.g:1849:1: ( ( rule__Input__Group_2__0 )? )
            {
            // InternalLinguaFranca.g:1849:1: ( ( rule__Input__Group_2__0 )? )
            // InternalLinguaFranca.g:1850:2: ( rule__Input__Group_2__0 )?
            {
             before(grammarAccess.getInputAccess().getGroup_2()); 
            // InternalLinguaFranca.g:1851:2: ( rule__Input__Group_2__0 )?
            int alt29=2;
            int LA29_0 = input.LA(1);

            if ( (LA29_0==23) ) {
                alt29=1;
            }
            switch (alt29) {
                case 1 :
                    // InternalLinguaFranca.g:1851:3: rule__Input__Group_2__0
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
    // InternalLinguaFranca.g:1859:1: rule__Input__Group__3 : rule__Input__Group__3__Impl ;
    public final void rule__Input__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1863:1: ( rule__Input__Group__3__Impl )
            // InternalLinguaFranca.g:1864:2: rule__Input__Group__3__Impl
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
    // InternalLinguaFranca.g:1870:1: rule__Input__Group__3__Impl : ( ';' ) ;
    public final void rule__Input__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1874:1: ( ( ';' ) )
            // InternalLinguaFranca.g:1875:1: ( ';' )
            {
            // InternalLinguaFranca.g:1875:1: ( ';' )
            // InternalLinguaFranca.g:1876:2: ';'
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
    // InternalLinguaFranca.g:1886:1: rule__Input__Group_2__0 : rule__Input__Group_2__0__Impl rule__Input__Group_2__1 ;
    public final void rule__Input__Group_2__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1890:1: ( rule__Input__Group_2__0__Impl rule__Input__Group_2__1 )
            // InternalLinguaFranca.g:1891:2: rule__Input__Group_2__0__Impl rule__Input__Group_2__1
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
    // InternalLinguaFranca.g:1898:1: rule__Input__Group_2__0__Impl : ( ':' ) ;
    public final void rule__Input__Group_2__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1902:1: ( ( ':' ) )
            // InternalLinguaFranca.g:1903:1: ( ':' )
            {
            // InternalLinguaFranca.g:1903:1: ( ':' )
            // InternalLinguaFranca.g:1904:2: ':'
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
    // InternalLinguaFranca.g:1913:1: rule__Input__Group_2__1 : rule__Input__Group_2__1__Impl ;
    public final void rule__Input__Group_2__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1917:1: ( rule__Input__Group_2__1__Impl )
            // InternalLinguaFranca.g:1918:2: rule__Input__Group_2__1__Impl
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
    // InternalLinguaFranca.g:1924:1: rule__Input__Group_2__1__Impl : ( ( rule__Input__TypeAssignment_2_1 ) ) ;
    public final void rule__Input__Group_2__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1928:1: ( ( ( rule__Input__TypeAssignment_2_1 ) ) )
            // InternalLinguaFranca.g:1929:1: ( ( rule__Input__TypeAssignment_2_1 ) )
            {
            // InternalLinguaFranca.g:1929:1: ( ( rule__Input__TypeAssignment_2_1 ) )
            // InternalLinguaFranca.g:1930:2: ( rule__Input__TypeAssignment_2_1 )
            {
             before(grammarAccess.getInputAccess().getTypeAssignment_2_1()); 
            // InternalLinguaFranca.g:1931:2: ( rule__Input__TypeAssignment_2_1 )
            // InternalLinguaFranca.g:1931:3: rule__Input__TypeAssignment_2_1
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
    // InternalLinguaFranca.g:1940:1: rule__Output__Group__0 : rule__Output__Group__0__Impl rule__Output__Group__1 ;
    public final void rule__Output__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1944:1: ( rule__Output__Group__0__Impl rule__Output__Group__1 )
            // InternalLinguaFranca.g:1945:2: rule__Output__Group__0__Impl rule__Output__Group__1
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
    // InternalLinguaFranca.g:1952:1: rule__Output__Group__0__Impl : ( 'output' ) ;
    public final void rule__Output__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1956:1: ( ( 'output' ) )
            // InternalLinguaFranca.g:1957:1: ( 'output' )
            {
            // InternalLinguaFranca.g:1957:1: ( 'output' )
            // InternalLinguaFranca.g:1958:2: 'output'
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
    // InternalLinguaFranca.g:1967:1: rule__Output__Group__1 : rule__Output__Group__1__Impl rule__Output__Group__2 ;
    public final void rule__Output__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1971:1: ( rule__Output__Group__1__Impl rule__Output__Group__2 )
            // InternalLinguaFranca.g:1972:2: rule__Output__Group__1__Impl rule__Output__Group__2
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
    // InternalLinguaFranca.g:1979:1: rule__Output__Group__1__Impl : ( ( rule__Output__NameAssignment_1 ) ) ;
    public final void rule__Output__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1983:1: ( ( ( rule__Output__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:1984:1: ( ( rule__Output__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:1984:1: ( ( rule__Output__NameAssignment_1 ) )
            // InternalLinguaFranca.g:1985:2: ( rule__Output__NameAssignment_1 )
            {
             before(grammarAccess.getOutputAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:1986:2: ( rule__Output__NameAssignment_1 )
            // InternalLinguaFranca.g:1986:3: rule__Output__NameAssignment_1
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
    // InternalLinguaFranca.g:1994:1: rule__Output__Group__2 : rule__Output__Group__2__Impl rule__Output__Group__3 ;
    public final void rule__Output__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:1998:1: ( rule__Output__Group__2__Impl rule__Output__Group__3 )
            // InternalLinguaFranca.g:1999:2: rule__Output__Group__2__Impl rule__Output__Group__3
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
    // InternalLinguaFranca.g:2006:1: rule__Output__Group__2__Impl : ( ( rule__Output__Group_2__0 )? ) ;
    public final void rule__Output__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2010:1: ( ( ( rule__Output__Group_2__0 )? ) )
            // InternalLinguaFranca.g:2011:1: ( ( rule__Output__Group_2__0 )? )
            {
            // InternalLinguaFranca.g:2011:1: ( ( rule__Output__Group_2__0 )? )
            // InternalLinguaFranca.g:2012:2: ( rule__Output__Group_2__0 )?
            {
             before(grammarAccess.getOutputAccess().getGroup_2()); 
            // InternalLinguaFranca.g:2013:2: ( rule__Output__Group_2__0 )?
            int alt30=2;
            int LA30_0 = input.LA(1);

            if ( (LA30_0==23) ) {
                alt30=1;
            }
            switch (alt30) {
                case 1 :
                    // InternalLinguaFranca.g:2013:3: rule__Output__Group_2__0
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
    // InternalLinguaFranca.g:2021:1: rule__Output__Group__3 : rule__Output__Group__3__Impl ;
    public final void rule__Output__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2025:1: ( rule__Output__Group__3__Impl )
            // InternalLinguaFranca.g:2026:2: rule__Output__Group__3__Impl
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
    // InternalLinguaFranca.g:2032:1: rule__Output__Group__3__Impl : ( ';' ) ;
    public final void rule__Output__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2036:1: ( ( ';' ) )
            // InternalLinguaFranca.g:2037:1: ( ';' )
            {
            // InternalLinguaFranca.g:2037:1: ( ';' )
            // InternalLinguaFranca.g:2038:2: ';'
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
    // InternalLinguaFranca.g:2048:1: rule__Output__Group_2__0 : rule__Output__Group_2__0__Impl rule__Output__Group_2__1 ;
    public final void rule__Output__Group_2__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2052:1: ( rule__Output__Group_2__0__Impl rule__Output__Group_2__1 )
            // InternalLinguaFranca.g:2053:2: rule__Output__Group_2__0__Impl rule__Output__Group_2__1
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
    // InternalLinguaFranca.g:2060:1: rule__Output__Group_2__0__Impl : ( ':' ) ;
    public final void rule__Output__Group_2__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2064:1: ( ( ':' ) )
            // InternalLinguaFranca.g:2065:1: ( ':' )
            {
            // InternalLinguaFranca.g:2065:1: ( ':' )
            // InternalLinguaFranca.g:2066:2: ':'
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
    // InternalLinguaFranca.g:2075:1: rule__Output__Group_2__1 : rule__Output__Group_2__1__Impl ;
    public final void rule__Output__Group_2__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2079:1: ( rule__Output__Group_2__1__Impl )
            // InternalLinguaFranca.g:2080:2: rule__Output__Group_2__1__Impl
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
    // InternalLinguaFranca.g:2086:1: rule__Output__Group_2__1__Impl : ( ( rule__Output__TypeAssignment_2_1 ) ) ;
    public final void rule__Output__Group_2__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2090:1: ( ( ( rule__Output__TypeAssignment_2_1 ) ) )
            // InternalLinguaFranca.g:2091:1: ( ( rule__Output__TypeAssignment_2_1 ) )
            {
            // InternalLinguaFranca.g:2091:1: ( ( rule__Output__TypeAssignment_2_1 ) )
            // InternalLinguaFranca.g:2092:2: ( rule__Output__TypeAssignment_2_1 )
            {
             before(grammarAccess.getOutputAccess().getTypeAssignment_2_1()); 
            // InternalLinguaFranca.g:2093:2: ( rule__Output__TypeAssignment_2_1 )
            // InternalLinguaFranca.g:2093:3: rule__Output__TypeAssignment_2_1
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


    // $ANTLR start "rule__Trigger__Group__0"
    // InternalLinguaFranca.g:2102:1: rule__Trigger__Group__0 : rule__Trigger__Group__0__Impl rule__Trigger__Group__1 ;
    public final void rule__Trigger__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2106:1: ( rule__Trigger__Group__0__Impl rule__Trigger__Group__1 )
            // InternalLinguaFranca.g:2107:2: rule__Trigger__Group__0__Impl rule__Trigger__Group__1
            {
            pushFollow(FOLLOW_21);
            rule__Trigger__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Trigger__Group__1();

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
    // $ANTLR end "rule__Trigger__Group__0"


    // $ANTLR start "rule__Trigger__Group__0__Impl"
    // InternalLinguaFranca.g:2114:1: rule__Trigger__Group__0__Impl : ( 'trigger' ) ;
    public final void rule__Trigger__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2118:1: ( ( 'trigger' ) )
            // InternalLinguaFranca.g:2119:1: ( 'trigger' )
            {
            // InternalLinguaFranca.g:2119:1: ( 'trigger' )
            // InternalLinguaFranca.g:2120:2: 'trigger'
            {
             before(grammarAccess.getTriggerAccess().getTriggerKeyword_0()); 
            match(input,15,FOLLOW_2); 
             after(grammarAccess.getTriggerAccess().getTriggerKeyword_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Trigger__Group__0__Impl"


    // $ANTLR start "rule__Trigger__Group__1"
    // InternalLinguaFranca.g:2129:1: rule__Trigger__Group__1 : rule__Trigger__Group__1__Impl rule__Trigger__Group__2 ;
    public final void rule__Trigger__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2133:1: ( rule__Trigger__Group__1__Impl rule__Trigger__Group__2 )
            // InternalLinguaFranca.g:2134:2: rule__Trigger__Group__1__Impl rule__Trigger__Group__2
            {
            pushFollow(FOLLOW_22);
            rule__Trigger__Group__1__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Trigger__Group__2();

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
    // $ANTLR end "rule__Trigger__Group__1"


    // $ANTLR start "rule__Trigger__Group__1__Impl"
    // InternalLinguaFranca.g:2141:1: rule__Trigger__Group__1__Impl : ( ( rule__Trigger__NameAssignment_1 ) ) ;
    public final void rule__Trigger__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2145:1: ( ( ( rule__Trigger__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:2146:1: ( ( rule__Trigger__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:2146:1: ( ( rule__Trigger__NameAssignment_1 ) )
            // InternalLinguaFranca.g:2147:2: ( rule__Trigger__NameAssignment_1 )
            {
             before(grammarAccess.getTriggerAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:2148:2: ( rule__Trigger__NameAssignment_1 )
            // InternalLinguaFranca.g:2148:3: rule__Trigger__NameAssignment_1
            {
            pushFollow(FOLLOW_2);
            rule__Trigger__NameAssignment_1();

            state._fsp--;


            }

             after(grammarAccess.getTriggerAccess().getNameAssignment_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Trigger__Group__1__Impl"


    // $ANTLR start "rule__Trigger__Group__2"
    // InternalLinguaFranca.g:2156:1: rule__Trigger__Group__2 : rule__Trigger__Group__2__Impl rule__Trigger__Group__3 ;
    public final void rule__Trigger__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2160:1: ( rule__Trigger__Group__2__Impl rule__Trigger__Group__3 )
            // InternalLinguaFranca.g:2161:2: rule__Trigger__Group__2__Impl rule__Trigger__Group__3
            {
            pushFollow(FOLLOW_22);
            rule__Trigger__Group__2__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Trigger__Group__3();

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
    // $ANTLR end "rule__Trigger__Group__2"


    // $ANTLR start "rule__Trigger__Group__2__Impl"
    // InternalLinguaFranca.g:2168:1: rule__Trigger__Group__2__Impl : ( ( rule__Trigger__PeriodAssignment_2 )? ) ;
    public final void rule__Trigger__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2172:1: ( ( ( rule__Trigger__PeriodAssignment_2 )? ) )
            // InternalLinguaFranca.g:2173:1: ( ( rule__Trigger__PeriodAssignment_2 )? )
            {
            // InternalLinguaFranca.g:2173:1: ( ( rule__Trigger__PeriodAssignment_2 )? )
            // InternalLinguaFranca.g:2174:2: ( rule__Trigger__PeriodAssignment_2 )?
            {
             before(grammarAccess.getTriggerAccess().getPeriodAssignment_2()); 
            // InternalLinguaFranca.g:2175:2: ( rule__Trigger__PeriodAssignment_2 )?
            int alt31=2;
            int LA31_0 = input.LA(1);

            if ( (LA31_0==25) ) {
                alt31=1;
            }
            switch (alt31) {
                case 1 :
                    // InternalLinguaFranca.g:2175:3: rule__Trigger__PeriodAssignment_2
                    {
                    pushFollow(FOLLOW_2);
                    rule__Trigger__PeriodAssignment_2();

                    state._fsp--;


                    }
                    break;

            }

             after(grammarAccess.getTriggerAccess().getPeriodAssignment_2()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Trigger__Group__2__Impl"


    // $ANTLR start "rule__Trigger__Group__3"
    // InternalLinguaFranca.g:2183:1: rule__Trigger__Group__3 : rule__Trigger__Group__3__Impl ;
    public final void rule__Trigger__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2187:1: ( rule__Trigger__Group__3__Impl )
            // InternalLinguaFranca.g:2188:2: rule__Trigger__Group__3__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Trigger__Group__3__Impl();

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
    // $ANTLR end "rule__Trigger__Group__3"


    // $ANTLR start "rule__Trigger__Group__3__Impl"
    // InternalLinguaFranca.g:2194:1: rule__Trigger__Group__3__Impl : ( ';' ) ;
    public final void rule__Trigger__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2198:1: ( ( ';' ) )
            // InternalLinguaFranca.g:2199:1: ( ';' )
            {
            // InternalLinguaFranca.g:2199:1: ( ';' )
            // InternalLinguaFranca.g:2200:2: ';'
            {
             before(grammarAccess.getTriggerAccess().getSemicolonKeyword_3()); 
            match(input,17,FOLLOW_2); 
             after(grammarAccess.getTriggerAccess().getSemicolonKeyword_3()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Trigger__Group__3__Impl"


    // $ANTLR start "rule__Reaction__Group__0"
    // InternalLinguaFranca.g:2210:1: rule__Reaction__Group__0 : rule__Reaction__Group__0__Impl rule__Reaction__Group__1 ;
    public final void rule__Reaction__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2214:1: ( rule__Reaction__Group__0__Impl rule__Reaction__Group__1 )
            // InternalLinguaFranca.g:2215:2: rule__Reaction__Group__0__Impl rule__Reaction__Group__1
            {
            pushFollow(FOLLOW_23);
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
    // InternalLinguaFranca.g:2222:1: rule__Reaction__Group__0__Impl : ( 'reaction' ) ;
    public final void rule__Reaction__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2226:1: ( ( 'reaction' ) )
            // InternalLinguaFranca.g:2227:1: ( 'reaction' )
            {
            // InternalLinguaFranca.g:2227:1: ( 'reaction' )
            // InternalLinguaFranca.g:2228:2: 'reaction'
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
    // InternalLinguaFranca.g:2237:1: rule__Reaction__Group__1 : rule__Reaction__Group__1__Impl rule__Reaction__Group__2 ;
    public final void rule__Reaction__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2241:1: ( rule__Reaction__Group__1__Impl rule__Reaction__Group__2 )
            // InternalLinguaFranca.g:2242:2: rule__Reaction__Group__1__Impl rule__Reaction__Group__2
            {
            pushFollow(FOLLOW_23);
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
    // InternalLinguaFranca.g:2249:1: rule__Reaction__Group__1__Impl : ( ( rule__Reaction__Group_1__0 )? ) ;
    public final void rule__Reaction__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2253:1: ( ( ( rule__Reaction__Group_1__0 )? ) )
            // InternalLinguaFranca.g:2254:1: ( ( rule__Reaction__Group_1__0 )? )
            {
            // InternalLinguaFranca.g:2254:1: ( ( rule__Reaction__Group_1__0 )? )
            // InternalLinguaFranca.g:2255:2: ( rule__Reaction__Group_1__0 )?
            {
             before(grammarAccess.getReactionAccess().getGroup_1()); 
            // InternalLinguaFranca.g:2256:2: ( rule__Reaction__Group_1__0 )?
            int alt32=2;
            int LA32_0 = input.LA(1);

            if ( (LA32_0==25) ) {
                alt32=1;
            }
            switch (alt32) {
                case 1 :
                    // InternalLinguaFranca.g:2256:3: rule__Reaction__Group_1__0
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
    // InternalLinguaFranca.g:2264:1: rule__Reaction__Group__2 : rule__Reaction__Group__2__Impl rule__Reaction__Group__3 ;
    public final void rule__Reaction__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2268:1: ( rule__Reaction__Group__2__Impl rule__Reaction__Group__3 )
            // InternalLinguaFranca.g:2269:2: rule__Reaction__Group__2__Impl rule__Reaction__Group__3
            {
            pushFollow(FOLLOW_23);
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
    // InternalLinguaFranca.g:2276:1: rule__Reaction__Group__2__Impl : ( ( rule__Reaction__GetsAssignment_2 )? ) ;
    public final void rule__Reaction__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2280:1: ( ( ( rule__Reaction__GetsAssignment_2 )? ) )
            // InternalLinguaFranca.g:2281:1: ( ( rule__Reaction__GetsAssignment_2 )? )
            {
            // InternalLinguaFranca.g:2281:1: ( ( rule__Reaction__GetsAssignment_2 )? )
            // InternalLinguaFranca.g:2282:2: ( rule__Reaction__GetsAssignment_2 )?
            {
             before(grammarAccess.getReactionAccess().getGetsAssignment_2()); 
            // InternalLinguaFranca.g:2283:2: ( rule__Reaction__GetsAssignment_2 )?
            int alt33=2;
            int LA33_0 = input.LA(1);

            if ( (LA33_0==RULE_ID) ) {
                alt33=1;
            }
            switch (alt33) {
                case 1 :
                    // InternalLinguaFranca.g:2283:3: rule__Reaction__GetsAssignment_2
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
    // InternalLinguaFranca.g:2291:1: rule__Reaction__Group__3 : rule__Reaction__Group__3__Impl rule__Reaction__Group__4 ;
    public final void rule__Reaction__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2295:1: ( rule__Reaction__Group__3__Impl rule__Reaction__Group__4 )
            // InternalLinguaFranca.g:2296:2: rule__Reaction__Group__3__Impl rule__Reaction__Group__4
            {
            pushFollow(FOLLOW_23);
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
    // InternalLinguaFranca.g:2303:1: rule__Reaction__Group__3__Impl : ( ( rule__Reaction__SetsAssignment_3 )? ) ;
    public final void rule__Reaction__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2307:1: ( ( ( rule__Reaction__SetsAssignment_3 )? ) )
            // InternalLinguaFranca.g:2308:1: ( ( rule__Reaction__SetsAssignment_3 )? )
            {
            // InternalLinguaFranca.g:2308:1: ( ( rule__Reaction__SetsAssignment_3 )? )
            // InternalLinguaFranca.g:2309:2: ( rule__Reaction__SetsAssignment_3 )?
            {
             before(grammarAccess.getReactionAccess().getSetsAssignment_3()); 
            // InternalLinguaFranca.g:2310:2: ( rule__Reaction__SetsAssignment_3 )?
            int alt34=2;
            int LA34_0 = input.LA(1);

            if ( (LA34_0==32) ) {
                alt34=1;
            }
            switch (alt34) {
                case 1 :
                    // InternalLinguaFranca.g:2310:3: rule__Reaction__SetsAssignment_3
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
    // InternalLinguaFranca.g:2318:1: rule__Reaction__Group__4 : rule__Reaction__Group__4__Impl ;
    public final void rule__Reaction__Group__4() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2322:1: ( rule__Reaction__Group__4__Impl )
            // InternalLinguaFranca.g:2323:2: rule__Reaction__Group__4__Impl
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
    // InternalLinguaFranca.g:2329:1: rule__Reaction__Group__4__Impl : ( ( rule__Reaction__CodeAssignment_4 ) ) ;
    public final void rule__Reaction__Group__4__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2333:1: ( ( ( rule__Reaction__CodeAssignment_4 ) ) )
            // InternalLinguaFranca.g:2334:1: ( ( rule__Reaction__CodeAssignment_4 ) )
            {
            // InternalLinguaFranca.g:2334:1: ( ( rule__Reaction__CodeAssignment_4 ) )
            // InternalLinguaFranca.g:2335:2: ( rule__Reaction__CodeAssignment_4 )
            {
             before(grammarAccess.getReactionAccess().getCodeAssignment_4()); 
            // InternalLinguaFranca.g:2336:2: ( rule__Reaction__CodeAssignment_4 )
            // InternalLinguaFranca.g:2336:3: rule__Reaction__CodeAssignment_4
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
    // InternalLinguaFranca.g:2345:1: rule__Reaction__Group_1__0 : rule__Reaction__Group_1__0__Impl rule__Reaction__Group_1__1 ;
    public final void rule__Reaction__Group_1__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2349:1: ( rule__Reaction__Group_1__0__Impl rule__Reaction__Group_1__1 )
            // InternalLinguaFranca.g:2350:2: rule__Reaction__Group_1__0__Impl rule__Reaction__Group_1__1
            {
            pushFollow(FOLLOW_24);
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
    // InternalLinguaFranca.g:2357:1: rule__Reaction__Group_1__0__Impl : ( '(' ) ;
    public final void rule__Reaction__Group_1__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2361:1: ( ( '(' ) )
            // InternalLinguaFranca.g:2362:1: ( '(' )
            {
            // InternalLinguaFranca.g:2362:1: ( '(' )
            // InternalLinguaFranca.g:2363:2: '('
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
    // InternalLinguaFranca.g:2372:1: rule__Reaction__Group_1__1 : rule__Reaction__Group_1__1__Impl rule__Reaction__Group_1__2 ;
    public final void rule__Reaction__Group_1__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2376:1: ( rule__Reaction__Group_1__1__Impl rule__Reaction__Group_1__2 )
            // InternalLinguaFranca.g:2377:2: rule__Reaction__Group_1__1__Impl rule__Reaction__Group_1__2
            {
            pushFollow(FOLLOW_24);
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
    // InternalLinguaFranca.g:2384:1: rule__Reaction__Group_1__1__Impl : ( ( rule__Reaction__Group_1_1__0 )? ) ;
    public final void rule__Reaction__Group_1__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2388:1: ( ( ( rule__Reaction__Group_1_1__0 )? ) )
            // InternalLinguaFranca.g:2389:1: ( ( rule__Reaction__Group_1_1__0 )? )
            {
            // InternalLinguaFranca.g:2389:1: ( ( rule__Reaction__Group_1_1__0 )? )
            // InternalLinguaFranca.g:2390:2: ( rule__Reaction__Group_1_1__0 )?
            {
             before(grammarAccess.getReactionAccess().getGroup_1_1()); 
            // InternalLinguaFranca.g:2391:2: ( rule__Reaction__Group_1_1__0 )?
            int alt35=2;
            int LA35_0 = input.LA(1);

            if ( (LA35_0==RULE_ID) ) {
                alt35=1;
            }
            switch (alt35) {
                case 1 :
                    // InternalLinguaFranca.g:2391:3: rule__Reaction__Group_1_1__0
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
    // InternalLinguaFranca.g:2399:1: rule__Reaction__Group_1__2 : rule__Reaction__Group_1__2__Impl ;
    public final void rule__Reaction__Group_1__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2403:1: ( rule__Reaction__Group_1__2__Impl )
            // InternalLinguaFranca.g:2404:2: rule__Reaction__Group_1__2__Impl
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
    // InternalLinguaFranca.g:2410:1: rule__Reaction__Group_1__2__Impl : ( ')' ) ;
    public final void rule__Reaction__Group_1__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2414:1: ( ( ')' ) )
            // InternalLinguaFranca.g:2415:1: ( ')' )
            {
            // InternalLinguaFranca.g:2415:1: ( ')' )
            // InternalLinguaFranca.g:2416:2: ')'
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
    // InternalLinguaFranca.g:2426:1: rule__Reaction__Group_1_1__0 : rule__Reaction__Group_1_1__0__Impl rule__Reaction__Group_1_1__1 ;
    public final void rule__Reaction__Group_1_1__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2430:1: ( rule__Reaction__Group_1_1__0__Impl rule__Reaction__Group_1_1__1 )
            // InternalLinguaFranca.g:2431:2: rule__Reaction__Group_1_1__0__Impl rule__Reaction__Group_1_1__1
            {
            pushFollow(FOLLOW_25);
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
    // InternalLinguaFranca.g:2438:1: rule__Reaction__Group_1_1__0__Impl : ( ( rule__Reaction__TriggersAssignment_1_1_0 ) ) ;
    public final void rule__Reaction__Group_1_1__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2442:1: ( ( ( rule__Reaction__TriggersAssignment_1_1_0 ) ) )
            // InternalLinguaFranca.g:2443:1: ( ( rule__Reaction__TriggersAssignment_1_1_0 ) )
            {
            // InternalLinguaFranca.g:2443:1: ( ( rule__Reaction__TriggersAssignment_1_1_0 ) )
            // InternalLinguaFranca.g:2444:2: ( rule__Reaction__TriggersAssignment_1_1_0 )
            {
             before(grammarAccess.getReactionAccess().getTriggersAssignment_1_1_0()); 
            // InternalLinguaFranca.g:2445:2: ( rule__Reaction__TriggersAssignment_1_1_0 )
            // InternalLinguaFranca.g:2445:3: rule__Reaction__TriggersAssignment_1_1_0
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
    // InternalLinguaFranca.g:2453:1: rule__Reaction__Group_1_1__1 : rule__Reaction__Group_1_1__1__Impl ;
    public final void rule__Reaction__Group_1_1__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2457:1: ( rule__Reaction__Group_1_1__1__Impl )
            // InternalLinguaFranca.g:2458:2: rule__Reaction__Group_1_1__1__Impl
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
    // InternalLinguaFranca.g:2464:1: rule__Reaction__Group_1_1__1__Impl : ( ( rule__Reaction__Group_1_1_1__0 )* ) ;
    public final void rule__Reaction__Group_1_1__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2468:1: ( ( ( rule__Reaction__Group_1_1_1__0 )* ) )
            // InternalLinguaFranca.g:2469:1: ( ( rule__Reaction__Group_1_1_1__0 )* )
            {
            // InternalLinguaFranca.g:2469:1: ( ( rule__Reaction__Group_1_1_1__0 )* )
            // InternalLinguaFranca.g:2470:2: ( rule__Reaction__Group_1_1_1__0 )*
            {
             before(grammarAccess.getReactionAccess().getGroup_1_1_1()); 
            // InternalLinguaFranca.g:2471:2: ( rule__Reaction__Group_1_1_1__0 )*
            loop36:
            do {
                int alt36=2;
                int LA36_0 = input.LA(1);

                if ( (LA36_0==27) ) {
                    alt36=1;
                }


                switch (alt36) {
            	case 1 :
            	    // InternalLinguaFranca.g:2471:3: rule__Reaction__Group_1_1_1__0
            	    {
            	    pushFollow(FOLLOW_26);
            	    rule__Reaction__Group_1_1_1__0();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop36;
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
    // InternalLinguaFranca.g:2480:1: rule__Reaction__Group_1_1_1__0 : rule__Reaction__Group_1_1_1__0__Impl rule__Reaction__Group_1_1_1__1 ;
    public final void rule__Reaction__Group_1_1_1__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2484:1: ( rule__Reaction__Group_1_1_1__0__Impl rule__Reaction__Group_1_1_1__1 )
            // InternalLinguaFranca.g:2485:2: rule__Reaction__Group_1_1_1__0__Impl rule__Reaction__Group_1_1_1__1
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
    // InternalLinguaFranca.g:2492:1: rule__Reaction__Group_1_1_1__0__Impl : ( ',' ) ;
    public final void rule__Reaction__Group_1_1_1__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2496:1: ( ( ',' ) )
            // InternalLinguaFranca.g:2497:1: ( ',' )
            {
            // InternalLinguaFranca.g:2497:1: ( ',' )
            // InternalLinguaFranca.g:2498:2: ','
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
    // InternalLinguaFranca.g:2507:1: rule__Reaction__Group_1_1_1__1 : rule__Reaction__Group_1_1_1__1__Impl ;
    public final void rule__Reaction__Group_1_1_1__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2511:1: ( rule__Reaction__Group_1_1_1__1__Impl )
            // InternalLinguaFranca.g:2512:2: rule__Reaction__Group_1_1_1__1__Impl
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
    // InternalLinguaFranca.g:2518:1: rule__Reaction__Group_1_1_1__1__Impl : ( ( rule__Reaction__TriggersAssignment_1_1_1_1 ) ) ;
    public final void rule__Reaction__Group_1_1_1__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2522:1: ( ( ( rule__Reaction__TriggersAssignment_1_1_1_1 ) ) )
            // InternalLinguaFranca.g:2523:1: ( ( rule__Reaction__TriggersAssignment_1_1_1_1 ) )
            {
            // InternalLinguaFranca.g:2523:1: ( ( rule__Reaction__TriggersAssignment_1_1_1_1 ) )
            // InternalLinguaFranca.g:2524:2: ( rule__Reaction__TriggersAssignment_1_1_1_1 )
            {
             before(grammarAccess.getReactionAccess().getTriggersAssignment_1_1_1_1()); 
            // InternalLinguaFranca.g:2525:2: ( rule__Reaction__TriggersAssignment_1_1_1_1 )
            // InternalLinguaFranca.g:2525:3: rule__Reaction__TriggersAssignment_1_1_1_1
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
    // InternalLinguaFranca.g:2534:1: rule__Preamble__Group__0 : rule__Preamble__Group__0__Impl rule__Preamble__Group__1 ;
    public final void rule__Preamble__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2538:1: ( rule__Preamble__Group__0__Impl rule__Preamble__Group__1 )
            // InternalLinguaFranca.g:2539:2: rule__Preamble__Group__0__Impl rule__Preamble__Group__1
            {
            pushFollow(FOLLOW_27);
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
    // InternalLinguaFranca.g:2546:1: rule__Preamble__Group__0__Impl : ( 'preamble' ) ;
    public final void rule__Preamble__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2550:1: ( ( 'preamble' ) )
            // InternalLinguaFranca.g:2551:1: ( 'preamble' )
            {
            // InternalLinguaFranca.g:2551:1: ( 'preamble' )
            // InternalLinguaFranca.g:2552:2: 'preamble'
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
    // InternalLinguaFranca.g:2561:1: rule__Preamble__Group__1 : rule__Preamble__Group__1__Impl ;
    public final void rule__Preamble__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2565:1: ( rule__Preamble__Group__1__Impl )
            // InternalLinguaFranca.g:2566:2: rule__Preamble__Group__1__Impl
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
    // InternalLinguaFranca.g:2572:1: rule__Preamble__Group__1__Impl : ( ( rule__Preamble__CodeAssignment_1 ) ) ;
    public final void rule__Preamble__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2576:1: ( ( ( rule__Preamble__CodeAssignment_1 ) ) )
            // InternalLinguaFranca.g:2577:1: ( ( rule__Preamble__CodeAssignment_1 ) )
            {
            // InternalLinguaFranca.g:2577:1: ( ( rule__Preamble__CodeAssignment_1 ) )
            // InternalLinguaFranca.g:2578:2: ( rule__Preamble__CodeAssignment_1 )
            {
             before(grammarAccess.getPreambleAccess().getCodeAssignment_1()); 
            // InternalLinguaFranca.g:2579:2: ( rule__Preamble__CodeAssignment_1 )
            // InternalLinguaFranca.g:2579:3: rule__Preamble__CodeAssignment_1
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


    // $ANTLR start "rule__Initialize__Group__0"
    // InternalLinguaFranca.g:2588:1: rule__Initialize__Group__0 : rule__Initialize__Group__0__Impl rule__Initialize__Group__1 ;
    public final void rule__Initialize__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2592:1: ( rule__Initialize__Group__0__Impl rule__Initialize__Group__1 )
            // InternalLinguaFranca.g:2593:2: rule__Initialize__Group__0__Impl rule__Initialize__Group__1
            {
            pushFollow(FOLLOW_27);
            rule__Initialize__Group__0__Impl();

            state._fsp--;

            pushFollow(FOLLOW_2);
            rule__Initialize__Group__1();

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
    // $ANTLR end "rule__Initialize__Group__0"


    // $ANTLR start "rule__Initialize__Group__0__Impl"
    // InternalLinguaFranca.g:2600:1: rule__Initialize__Group__0__Impl : ( 'initialize' ) ;
    public final void rule__Initialize__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2604:1: ( ( 'initialize' ) )
            // InternalLinguaFranca.g:2605:1: ( 'initialize' )
            {
            // InternalLinguaFranca.g:2605:1: ( 'initialize' )
            // InternalLinguaFranca.g:2606:2: 'initialize'
            {
             before(grammarAccess.getInitializeAccess().getInitializeKeyword_0()); 
            match(input,29,FOLLOW_2); 
             after(grammarAccess.getInitializeAccess().getInitializeKeyword_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Initialize__Group__0__Impl"


    // $ANTLR start "rule__Initialize__Group__1"
    // InternalLinguaFranca.g:2615:1: rule__Initialize__Group__1 : rule__Initialize__Group__1__Impl ;
    public final void rule__Initialize__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2619:1: ( rule__Initialize__Group__1__Impl )
            // InternalLinguaFranca.g:2620:2: rule__Initialize__Group__1__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Initialize__Group__1__Impl();

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
    // $ANTLR end "rule__Initialize__Group__1"


    // $ANTLR start "rule__Initialize__Group__1__Impl"
    // InternalLinguaFranca.g:2626:1: rule__Initialize__Group__1__Impl : ( ( rule__Initialize__CodeAssignment_1 ) ) ;
    public final void rule__Initialize__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2630:1: ( ( ( rule__Initialize__CodeAssignment_1 ) ) )
            // InternalLinguaFranca.g:2631:1: ( ( rule__Initialize__CodeAssignment_1 ) )
            {
            // InternalLinguaFranca.g:2631:1: ( ( rule__Initialize__CodeAssignment_1 ) )
            // InternalLinguaFranca.g:2632:2: ( rule__Initialize__CodeAssignment_1 )
            {
             before(grammarAccess.getInitializeAccess().getCodeAssignment_1()); 
            // InternalLinguaFranca.g:2633:2: ( rule__Initialize__CodeAssignment_1 )
            // InternalLinguaFranca.g:2633:3: rule__Initialize__CodeAssignment_1
            {
            pushFollow(FOLLOW_2);
            rule__Initialize__CodeAssignment_1();

            state._fsp--;


            }

             after(grammarAccess.getInitializeAccess().getCodeAssignment_1()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Initialize__Group__1__Impl"


    // $ANTLR start "rule__Instance__Group__0"
    // InternalLinguaFranca.g:2642:1: rule__Instance__Group__0 : rule__Instance__Group__0__Impl rule__Instance__Group__1 ;
    public final void rule__Instance__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2646:1: ( rule__Instance__Group__0__Impl rule__Instance__Group__1 )
            // InternalLinguaFranca.g:2647:2: rule__Instance__Group__0__Impl rule__Instance__Group__1
            {
            pushFollow(FOLLOW_6);
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
    // InternalLinguaFranca.g:2654:1: rule__Instance__Group__0__Impl : ( 'instance' ) ;
    public final void rule__Instance__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2658:1: ( ( 'instance' ) )
            // InternalLinguaFranca.g:2659:1: ( 'instance' )
            {
            // InternalLinguaFranca.g:2659:1: ( 'instance' )
            // InternalLinguaFranca.g:2660:2: 'instance'
            {
             before(grammarAccess.getInstanceAccess().getInstanceKeyword_0()); 
            match(input,30,FOLLOW_2); 
             after(grammarAccess.getInstanceAccess().getInstanceKeyword_0()); 

            }


            }

        }
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
    // InternalLinguaFranca.g:2669:1: rule__Instance__Group__1 : rule__Instance__Group__1__Impl rule__Instance__Group__2 ;
    public final void rule__Instance__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2673:1: ( rule__Instance__Group__1__Impl rule__Instance__Group__2 )
            // InternalLinguaFranca.g:2674:2: rule__Instance__Group__1__Impl rule__Instance__Group__2
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
    // InternalLinguaFranca.g:2681:1: rule__Instance__Group__1__Impl : ( ( rule__Instance__NameAssignment_1 ) ) ;
    public final void rule__Instance__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2685:1: ( ( ( rule__Instance__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:2686:1: ( ( rule__Instance__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:2686:1: ( ( rule__Instance__NameAssignment_1 ) )
            // InternalLinguaFranca.g:2687:2: ( rule__Instance__NameAssignment_1 )
            {
             before(grammarAccess.getInstanceAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:2688:2: ( rule__Instance__NameAssignment_1 )
            // InternalLinguaFranca.g:2688:3: rule__Instance__NameAssignment_1
            {
            pushFollow(FOLLOW_2);
            rule__Instance__NameAssignment_1();

            state._fsp--;


            }

             after(grammarAccess.getInstanceAccess().getNameAssignment_1()); 

            }


            }

        }
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
    // InternalLinguaFranca.g:2696:1: rule__Instance__Group__2 : rule__Instance__Group__2__Impl rule__Instance__Group__3 ;
    public final void rule__Instance__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2700:1: ( rule__Instance__Group__2__Impl rule__Instance__Group__3 )
            // InternalLinguaFranca.g:2701:2: rule__Instance__Group__2__Impl rule__Instance__Group__3
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
    // InternalLinguaFranca.g:2708:1: rule__Instance__Group__2__Impl : ( '=' ) ;
    public final void rule__Instance__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2712:1: ( ( '=' ) )
            // InternalLinguaFranca.g:2713:1: ( '=' )
            {
            // InternalLinguaFranca.g:2713:1: ( '=' )
            // InternalLinguaFranca.g:2714:2: '='
            {
             before(grammarAccess.getInstanceAccess().getEqualsSignKeyword_2()); 
            match(input,31,FOLLOW_2); 
             after(grammarAccess.getInstanceAccess().getEqualsSignKeyword_2()); 

            }


            }

        }
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
    // InternalLinguaFranca.g:2723:1: rule__Instance__Group__3 : rule__Instance__Group__3__Impl rule__Instance__Group__4 ;
    public final void rule__Instance__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2727:1: ( rule__Instance__Group__3__Impl rule__Instance__Group__4 )
            // InternalLinguaFranca.g:2728:2: rule__Instance__Group__3__Impl rule__Instance__Group__4
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
    // InternalLinguaFranca.g:2735:1: rule__Instance__Group__3__Impl : ( ( rule__Instance__ActorClassAssignment_3 ) ) ;
    public final void rule__Instance__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2739:1: ( ( ( rule__Instance__ActorClassAssignment_3 ) ) )
            // InternalLinguaFranca.g:2740:1: ( ( rule__Instance__ActorClassAssignment_3 ) )
            {
            // InternalLinguaFranca.g:2740:1: ( ( rule__Instance__ActorClassAssignment_3 ) )
            // InternalLinguaFranca.g:2741:2: ( rule__Instance__ActorClassAssignment_3 )
            {
             before(grammarAccess.getInstanceAccess().getActorClassAssignment_3()); 
            // InternalLinguaFranca.g:2742:2: ( rule__Instance__ActorClassAssignment_3 )
            // InternalLinguaFranca.g:2742:3: rule__Instance__ActorClassAssignment_3
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
    // InternalLinguaFranca.g:2750:1: rule__Instance__Group__4 : rule__Instance__Group__4__Impl rule__Instance__Group__5 ;
    public final void rule__Instance__Group__4() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2754:1: ( rule__Instance__Group__4__Impl rule__Instance__Group__5 )
            // InternalLinguaFranca.g:2755:2: rule__Instance__Group__4__Impl rule__Instance__Group__5
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
    // InternalLinguaFranca.g:2762:1: rule__Instance__Group__4__Impl : ( ( rule__Instance__Group_4__0 )? ) ;
    public final void rule__Instance__Group__4__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2766:1: ( ( ( rule__Instance__Group_4__0 )? ) )
            // InternalLinguaFranca.g:2767:1: ( ( rule__Instance__Group_4__0 )? )
            {
            // InternalLinguaFranca.g:2767:1: ( ( rule__Instance__Group_4__0 )? )
            // InternalLinguaFranca.g:2768:2: ( rule__Instance__Group_4__0 )?
            {
             before(grammarAccess.getInstanceAccess().getGroup_4()); 
            // InternalLinguaFranca.g:2769:2: ( rule__Instance__Group_4__0 )?
            int alt37=2;
            int LA37_0 = input.LA(1);

            if ( (LA37_0==25) ) {
                alt37=1;
            }
            switch (alt37) {
                case 1 :
                    // InternalLinguaFranca.g:2769:3: rule__Instance__Group_4__0
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
    // InternalLinguaFranca.g:2777:1: rule__Instance__Group__5 : rule__Instance__Group__5__Impl ;
    public final void rule__Instance__Group__5() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2781:1: ( rule__Instance__Group__5__Impl )
            // InternalLinguaFranca.g:2782:2: rule__Instance__Group__5__Impl
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
    // InternalLinguaFranca.g:2788:1: rule__Instance__Group__5__Impl : ( ';' ) ;
    public final void rule__Instance__Group__5__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2792:1: ( ( ';' ) )
            // InternalLinguaFranca.g:2793:1: ( ';' )
            {
            // InternalLinguaFranca.g:2793:1: ( ';' )
            // InternalLinguaFranca.g:2794:2: ';'
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
    // InternalLinguaFranca.g:2804:1: rule__Instance__Group_4__0 : rule__Instance__Group_4__0__Impl rule__Instance__Group_4__1 ;
    public final void rule__Instance__Group_4__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2808:1: ( rule__Instance__Group_4__0__Impl rule__Instance__Group_4__1 )
            // InternalLinguaFranca.g:2809:2: rule__Instance__Group_4__0__Impl rule__Instance__Group_4__1
            {
            pushFollow(FOLLOW_24);
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
    // InternalLinguaFranca.g:2816:1: rule__Instance__Group_4__0__Impl : ( '(' ) ;
    public final void rule__Instance__Group_4__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2820:1: ( ( '(' ) )
            // InternalLinguaFranca.g:2821:1: ( '(' )
            {
            // InternalLinguaFranca.g:2821:1: ( '(' )
            // InternalLinguaFranca.g:2822:2: '('
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
    // InternalLinguaFranca.g:2831:1: rule__Instance__Group_4__1 : rule__Instance__Group_4__1__Impl rule__Instance__Group_4__2 ;
    public final void rule__Instance__Group_4__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2835:1: ( rule__Instance__Group_4__1__Impl rule__Instance__Group_4__2 )
            // InternalLinguaFranca.g:2836:2: rule__Instance__Group_4__1__Impl rule__Instance__Group_4__2
            {
            pushFollow(FOLLOW_24);
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
    // InternalLinguaFranca.g:2843:1: rule__Instance__Group_4__1__Impl : ( ( rule__Instance__ParametersAssignment_4_1 )? ) ;
    public final void rule__Instance__Group_4__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2847:1: ( ( ( rule__Instance__ParametersAssignment_4_1 )? ) )
            // InternalLinguaFranca.g:2848:1: ( ( rule__Instance__ParametersAssignment_4_1 )? )
            {
            // InternalLinguaFranca.g:2848:1: ( ( rule__Instance__ParametersAssignment_4_1 )? )
            // InternalLinguaFranca.g:2849:2: ( rule__Instance__ParametersAssignment_4_1 )?
            {
             before(grammarAccess.getInstanceAccess().getParametersAssignment_4_1()); 
            // InternalLinguaFranca.g:2850:2: ( rule__Instance__ParametersAssignment_4_1 )?
            int alt38=2;
            int LA38_0 = input.LA(1);

            if ( (LA38_0==RULE_ID) ) {
                alt38=1;
            }
            switch (alt38) {
                case 1 :
                    // InternalLinguaFranca.g:2850:3: rule__Instance__ParametersAssignment_4_1
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
    // InternalLinguaFranca.g:2858:1: rule__Instance__Group_4__2 : rule__Instance__Group_4__2__Impl ;
    public final void rule__Instance__Group_4__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2862:1: ( rule__Instance__Group_4__2__Impl )
            // InternalLinguaFranca.g:2863:2: rule__Instance__Group_4__2__Impl
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
    // InternalLinguaFranca.g:2869:1: rule__Instance__Group_4__2__Impl : ( ')' ) ;
    public final void rule__Instance__Group_4__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2873:1: ( ( ')' ) )
            // InternalLinguaFranca.g:2874:1: ( ')' )
            {
            // InternalLinguaFranca.g:2874:1: ( ')' )
            // InternalLinguaFranca.g:2875:2: ')'
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
    // InternalLinguaFranca.g:2885:1: rule__Connection__Group__0 : rule__Connection__Group__0__Impl rule__Connection__Group__1 ;
    public final void rule__Connection__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2889:1: ( rule__Connection__Group__0__Impl rule__Connection__Group__1 )
            // InternalLinguaFranca.g:2890:2: rule__Connection__Group__0__Impl rule__Connection__Group__1
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
    // InternalLinguaFranca.g:2897:1: rule__Connection__Group__0__Impl : ( ( rule__Connection__LeftPortAssignment_0 ) ) ;
    public final void rule__Connection__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2901:1: ( ( ( rule__Connection__LeftPortAssignment_0 ) ) )
            // InternalLinguaFranca.g:2902:1: ( ( rule__Connection__LeftPortAssignment_0 ) )
            {
            // InternalLinguaFranca.g:2902:1: ( ( rule__Connection__LeftPortAssignment_0 ) )
            // InternalLinguaFranca.g:2903:2: ( rule__Connection__LeftPortAssignment_0 )
            {
             before(grammarAccess.getConnectionAccess().getLeftPortAssignment_0()); 
            // InternalLinguaFranca.g:2904:2: ( rule__Connection__LeftPortAssignment_0 )
            // InternalLinguaFranca.g:2904:3: rule__Connection__LeftPortAssignment_0
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
    // InternalLinguaFranca.g:2912:1: rule__Connection__Group__1 : rule__Connection__Group__1__Impl rule__Connection__Group__2 ;
    public final void rule__Connection__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2916:1: ( rule__Connection__Group__1__Impl rule__Connection__Group__2 )
            // InternalLinguaFranca.g:2917:2: rule__Connection__Group__1__Impl rule__Connection__Group__2
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
    // InternalLinguaFranca.g:2924:1: rule__Connection__Group__1__Impl : ( '->' ) ;
    public final void rule__Connection__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2928:1: ( ( '->' ) )
            // InternalLinguaFranca.g:2929:1: ( '->' )
            {
            // InternalLinguaFranca.g:2929:1: ( '->' )
            // InternalLinguaFranca.g:2930:2: '->'
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
    // InternalLinguaFranca.g:2939:1: rule__Connection__Group__2 : rule__Connection__Group__2__Impl rule__Connection__Group__3 ;
    public final void rule__Connection__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2943:1: ( rule__Connection__Group__2__Impl rule__Connection__Group__3 )
            // InternalLinguaFranca.g:2944:2: rule__Connection__Group__2__Impl rule__Connection__Group__3
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
    // InternalLinguaFranca.g:2951:1: rule__Connection__Group__2__Impl : ( ( rule__Connection__RightPortAssignment_2 ) ) ;
    public final void rule__Connection__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2955:1: ( ( ( rule__Connection__RightPortAssignment_2 ) ) )
            // InternalLinguaFranca.g:2956:1: ( ( rule__Connection__RightPortAssignment_2 ) )
            {
            // InternalLinguaFranca.g:2956:1: ( ( rule__Connection__RightPortAssignment_2 ) )
            // InternalLinguaFranca.g:2957:2: ( rule__Connection__RightPortAssignment_2 )
            {
             before(grammarAccess.getConnectionAccess().getRightPortAssignment_2()); 
            // InternalLinguaFranca.g:2958:2: ( rule__Connection__RightPortAssignment_2 )
            // InternalLinguaFranca.g:2958:3: rule__Connection__RightPortAssignment_2
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
    // InternalLinguaFranca.g:2966:1: rule__Connection__Group__3 : rule__Connection__Group__3__Impl ;
    public final void rule__Connection__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2970:1: ( rule__Connection__Group__3__Impl )
            // InternalLinguaFranca.g:2971:2: rule__Connection__Group__3__Impl
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
    // InternalLinguaFranca.g:2977:1: rule__Connection__Group__3__Impl : ( ';' ) ;
    public final void rule__Connection__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2981:1: ( ( ';' ) )
            // InternalLinguaFranca.g:2982:1: ( ';' )
            {
            // InternalLinguaFranca.g:2982:1: ( ';' )
            // InternalLinguaFranca.g:2983:2: ';'
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
    // InternalLinguaFranca.g:2993:1: rule__Assignments__Group__0 : rule__Assignments__Group__0__Impl rule__Assignments__Group__1 ;
    public final void rule__Assignments__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:2997:1: ( rule__Assignments__Group__0__Impl rule__Assignments__Group__1 )
            // InternalLinguaFranca.g:2998:2: rule__Assignments__Group__0__Impl rule__Assignments__Group__1
            {
            pushFollow(FOLLOW_25);
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
    // InternalLinguaFranca.g:3005:1: rule__Assignments__Group__0__Impl : ( ( rule__Assignments__AssignmentsAssignment_0 ) ) ;
    public final void rule__Assignments__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3009:1: ( ( ( rule__Assignments__AssignmentsAssignment_0 ) ) )
            // InternalLinguaFranca.g:3010:1: ( ( rule__Assignments__AssignmentsAssignment_0 ) )
            {
            // InternalLinguaFranca.g:3010:1: ( ( rule__Assignments__AssignmentsAssignment_0 ) )
            // InternalLinguaFranca.g:3011:2: ( rule__Assignments__AssignmentsAssignment_0 )
            {
             before(grammarAccess.getAssignmentsAccess().getAssignmentsAssignment_0()); 
            // InternalLinguaFranca.g:3012:2: ( rule__Assignments__AssignmentsAssignment_0 )
            // InternalLinguaFranca.g:3012:3: rule__Assignments__AssignmentsAssignment_0
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
    // InternalLinguaFranca.g:3020:1: rule__Assignments__Group__1 : rule__Assignments__Group__1__Impl ;
    public final void rule__Assignments__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3024:1: ( rule__Assignments__Group__1__Impl )
            // InternalLinguaFranca.g:3025:2: rule__Assignments__Group__1__Impl
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
    // InternalLinguaFranca.g:3031:1: rule__Assignments__Group__1__Impl : ( ( rule__Assignments__Group_1__0 )* ) ;
    public final void rule__Assignments__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3035:1: ( ( ( rule__Assignments__Group_1__0 )* ) )
            // InternalLinguaFranca.g:3036:1: ( ( rule__Assignments__Group_1__0 )* )
            {
            // InternalLinguaFranca.g:3036:1: ( ( rule__Assignments__Group_1__0 )* )
            // InternalLinguaFranca.g:3037:2: ( rule__Assignments__Group_1__0 )*
            {
             before(grammarAccess.getAssignmentsAccess().getGroup_1()); 
            // InternalLinguaFranca.g:3038:2: ( rule__Assignments__Group_1__0 )*
            loop39:
            do {
                int alt39=2;
                int LA39_0 = input.LA(1);

                if ( (LA39_0==27) ) {
                    alt39=1;
                }


                switch (alt39) {
            	case 1 :
            	    // InternalLinguaFranca.g:3038:3: rule__Assignments__Group_1__0
            	    {
            	    pushFollow(FOLLOW_26);
            	    rule__Assignments__Group_1__0();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop39;
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
    // InternalLinguaFranca.g:3047:1: rule__Assignments__Group_1__0 : rule__Assignments__Group_1__0__Impl rule__Assignments__Group_1__1 ;
    public final void rule__Assignments__Group_1__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3051:1: ( rule__Assignments__Group_1__0__Impl rule__Assignments__Group_1__1 )
            // InternalLinguaFranca.g:3052:2: rule__Assignments__Group_1__0__Impl rule__Assignments__Group_1__1
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
    // InternalLinguaFranca.g:3059:1: rule__Assignments__Group_1__0__Impl : ( ',' ) ;
    public final void rule__Assignments__Group_1__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3063:1: ( ( ',' ) )
            // InternalLinguaFranca.g:3064:1: ( ',' )
            {
            // InternalLinguaFranca.g:3064:1: ( ',' )
            // InternalLinguaFranca.g:3065:2: ','
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
    // InternalLinguaFranca.g:3074:1: rule__Assignments__Group_1__1 : rule__Assignments__Group_1__1__Impl ;
    public final void rule__Assignments__Group_1__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3078:1: ( rule__Assignments__Group_1__1__Impl )
            // InternalLinguaFranca.g:3079:2: rule__Assignments__Group_1__1__Impl
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
    // InternalLinguaFranca.g:3085:1: rule__Assignments__Group_1__1__Impl : ( ( rule__Assignments__AssignmentsAssignment_1_1 ) ) ;
    public final void rule__Assignments__Group_1__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3089:1: ( ( ( rule__Assignments__AssignmentsAssignment_1_1 ) ) )
            // InternalLinguaFranca.g:3090:1: ( ( rule__Assignments__AssignmentsAssignment_1_1 ) )
            {
            // InternalLinguaFranca.g:3090:1: ( ( rule__Assignments__AssignmentsAssignment_1_1 ) )
            // InternalLinguaFranca.g:3091:2: ( rule__Assignments__AssignmentsAssignment_1_1 )
            {
             before(grammarAccess.getAssignmentsAccess().getAssignmentsAssignment_1_1()); 
            // InternalLinguaFranca.g:3092:2: ( rule__Assignments__AssignmentsAssignment_1_1 )
            // InternalLinguaFranca.g:3092:3: rule__Assignments__AssignmentsAssignment_1_1
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
    // InternalLinguaFranca.g:3101:1: rule__Assignment__Group__0 : rule__Assignment__Group__0__Impl rule__Assignment__Group__1 ;
    public final void rule__Assignment__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3105:1: ( rule__Assignment__Group__0__Impl rule__Assignment__Group__1 )
            // InternalLinguaFranca.g:3106:2: rule__Assignment__Group__0__Impl rule__Assignment__Group__1
            {
            pushFollow(FOLLOW_28);
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
    // InternalLinguaFranca.g:3113:1: rule__Assignment__Group__0__Impl : ( ( rule__Assignment__NameAssignment_0 ) ) ;
    public final void rule__Assignment__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3117:1: ( ( ( rule__Assignment__NameAssignment_0 ) ) )
            // InternalLinguaFranca.g:3118:1: ( ( rule__Assignment__NameAssignment_0 ) )
            {
            // InternalLinguaFranca.g:3118:1: ( ( rule__Assignment__NameAssignment_0 ) )
            // InternalLinguaFranca.g:3119:2: ( rule__Assignment__NameAssignment_0 )
            {
             before(grammarAccess.getAssignmentAccess().getNameAssignment_0()); 
            // InternalLinguaFranca.g:3120:2: ( rule__Assignment__NameAssignment_0 )
            // InternalLinguaFranca.g:3120:3: rule__Assignment__NameAssignment_0
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
    // InternalLinguaFranca.g:3128:1: rule__Assignment__Group__1 : rule__Assignment__Group__1__Impl rule__Assignment__Group__2 ;
    public final void rule__Assignment__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3132:1: ( rule__Assignment__Group__1__Impl rule__Assignment__Group__2 )
            // InternalLinguaFranca.g:3133:2: rule__Assignment__Group__1__Impl rule__Assignment__Group__2
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
    // InternalLinguaFranca.g:3140:1: rule__Assignment__Group__1__Impl : ( '=' ) ;
    public final void rule__Assignment__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3144:1: ( ( '=' ) )
            // InternalLinguaFranca.g:3145:1: ( '=' )
            {
            // InternalLinguaFranca.g:3145:1: ( '=' )
            // InternalLinguaFranca.g:3146:2: '='
            {
             before(grammarAccess.getAssignmentAccess().getEqualsSignKeyword_1()); 
            match(input,31,FOLLOW_2); 
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
    // InternalLinguaFranca.g:3155:1: rule__Assignment__Group__2 : rule__Assignment__Group__2__Impl ;
    public final void rule__Assignment__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3159:1: ( rule__Assignment__Group__2__Impl )
            // InternalLinguaFranca.g:3160:2: rule__Assignment__Group__2__Impl
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
    // InternalLinguaFranca.g:3166:1: rule__Assignment__Group__2__Impl : ( ( rule__Assignment__ValueAssignment_2 ) ) ;
    public final void rule__Assignment__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3170:1: ( ( ( rule__Assignment__ValueAssignment_2 ) ) )
            // InternalLinguaFranca.g:3171:1: ( ( rule__Assignment__ValueAssignment_2 ) )
            {
            // InternalLinguaFranca.g:3171:1: ( ( rule__Assignment__ValueAssignment_2 ) )
            // InternalLinguaFranca.g:3172:2: ( rule__Assignment__ValueAssignment_2 )
            {
             before(grammarAccess.getAssignmentAccess().getValueAssignment_2()); 
            // InternalLinguaFranca.g:3173:2: ( rule__Assignment__ValueAssignment_2 )
            // InternalLinguaFranca.g:3173:3: rule__Assignment__ValueAssignment_2
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
    // InternalLinguaFranca.g:3182:1: rule__Gets__Group__0 : rule__Gets__Group__0__Impl rule__Gets__Group__1 ;
    public final void rule__Gets__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3186:1: ( rule__Gets__Group__0__Impl rule__Gets__Group__1 )
            // InternalLinguaFranca.g:3187:2: rule__Gets__Group__0__Impl rule__Gets__Group__1
            {
            pushFollow(FOLLOW_25);
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
    // InternalLinguaFranca.g:3194:1: rule__Gets__Group__0__Impl : ( ( rule__Gets__GetsAssignment_0 ) ) ;
    public final void rule__Gets__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3198:1: ( ( ( rule__Gets__GetsAssignment_0 ) ) )
            // InternalLinguaFranca.g:3199:1: ( ( rule__Gets__GetsAssignment_0 ) )
            {
            // InternalLinguaFranca.g:3199:1: ( ( rule__Gets__GetsAssignment_0 ) )
            // InternalLinguaFranca.g:3200:2: ( rule__Gets__GetsAssignment_0 )
            {
             before(grammarAccess.getGetsAccess().getGetsAssignment_0()); 
            // InternalLinguaFranca.g:3201:2: ( rule__Gets__GetsAssignment_0 )
            // InternalLinguaFranca.g:3201:3: rule__Gets__GetsAssignment_0
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
    // InternalLinguaFranca.g:3209:1: rule__Gets__Group__1 : rule__Gets__Group__1__Impl ;
    public final void rule__Gets__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3213:1: ( rule__Gets__Group__1__Impl )
            // InternalLinguaFranca.g:3214:2: rule__Gets__Group__1__Impl
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
    // InternalLinguaFranca.g:3220:1: rule__Gets__Group__1__Impl : ( ( rule__Gets__Group_1__0 )? ) ;
    public final void rule__Gets__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3224:1: ( ( ( rule__Gets__Group_1__0 )? ) )
            // InternalLinguaFranca.g:3225:1: ( ( rule__Gets__Group_1__0 )? )
            {
            // InternalLinguaFranca.g:3225:1: ( ( rule__Gets__Group_1__0 )? )
            // InternalLinguaFranca.g:3226:2: ( rule__Gets__Group_1__0 )?
            {
             before(grammarAccess.getGetsAccess().getGroup_1()); 
            // InternalLinguaFranca.g:3227:2: ( rule__Gets__Group_1__0 )?
            int alt40=2;
            int LA40_0 = input.LA(1);

            if ( (LA40_0==27) ) {
                alt40=1;
            }
            switch (alt40) {
                case 1 :
                    // InternalLinguaFranca.g:3227:3: rule__Gets__Group_1__0
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
    // InternalLinguaFranca.g:3236:1: rule__Gets__Group_1__0 : rule__Gets__Group_1__0__Impl rule__Gets__Group_1__1 ;
    public final void rule__Gets__Group_1__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3240:1: ( rule__Gets__Group_1__0__Impl rule__Gets__Group_1__1 )
            // InternalLinguaFranca.g:3241:2: rule__Gets__Group_1__0__Impl rule__Gets__Group_1__1
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
    // InternalLinguaFranca.g:3248:1: rule__Gets__Group_1__0__Impl : ( ',' ) ;
    public final void rule__Gets__Group_1__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3252:1: ( ( ',' ) )
            // InternalLinguaFranca.g:3253:1: ( ',' )
            {
            // InternalLinguaFranca.g:3253:1: ( ',' )
            // InternalLinguaFranca.g:3254:2: ','
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
    // InternalLinguaFranca.g:3263:1: rule__Gets__Group_1__1 : rule__Gets__Group_1__1__Impl ;
    public final void rule__Gets__Group_1__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3267:1: ( rule__Gets__Group_1__1__Impl )
            // InternalLinguaFranca.g:3268:2: rule__Gets__Group_1__1__Impl
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
    // InternalLinguaFranca.g:3274:1: rule__Gets__Group_1__1__Impl : ( ( rule__Gets__GetsAssignment_1_1 ) ) ;
    public final void rule__Gets__Group_1__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3278:1: ( ( ( rule__Gets__GetsAssignment_1_1 ) ) )
            // InternalLinguaFranca.g:3279:1: ( ( rule__Gets__GetsAssignment_1_1 ) )
            {
            // InternalLinguaFranca.g:3279:1: ( ( rule__Gets__GetsAssignment_1_1 ) )
            // InternalLinguaFranca.g:3280:2: ( rule__Gets__GetsAssignment_1_1 )
            {
             before(grammarAccess.getGetsAccess().getGetsAssignment_1_1()); 
            // InternalLinguaFranca.g:3281:2: ( rule__Gets__GetsAssignment_1_1 )
            // InternalLinguaFranca.g:3281:3: rule__Gets__GetsAssignment_1_1
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
    // InternalLinguaFranca.g:3290:1: rule__Params__Group__0 : rule__Params__Group__0__Impl rule__Params__Group__1 ;
    public final void rule__Params__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3294:1: ( rule__Params__Group__0__Impl rule__Params__Group__1 )
            // InternalLinguaFranca.g:3295:2: rule__Params__Group__0__Impl rule__Params__Group__1
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
    // InternalLinguaFranca.g:3302:1: rule__Params__Group__0__Impl : ( '(' ) ;
    public final void rule__Params__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3306:1: ( ( '(' ) )
            // InternalLinguaFranca.g:3307:1: ( '(' )
            {
            // InternalLinguaFranca.g:3307:1: ( '(' )
            // InternalLinguaFranca.g:3308:2: '('
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
    // InternalLinguaFranca.g:3317:1: rule__Params__Group__1 : rule__Params__Group__1__Impl rule__Params__Group__2 ;
    public final void rule__Params__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3321:1: ( rule__Params__Group__1__Impl rule__Params__Group__2 )
            // InternalLinguaFranca.g:3322:2: rule__Params__Group__1__Impl rule__Params__Group__2
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
    // InternalLinguaFranca.g:3329:1: rule__Params__Group__1__Impl : ( ( rule__Params__ParamsAssignment_1 ) ) ;
    public final void rule__Params__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3333:1: ( ( ( rule__Params__ParamsAssignment_1 ) ) )
            // InternalLinguaFranca.g:3334:1: ( ( rule__Params__ParamsAssignment_1 ) )
            {
            // InternalLinguaFranca.g:3334:1: ( ( rule__Params__ParamsAssignment_1 ) )
            // InternalLinguaFranca.g:3335:2: ( rule__Params__ParamsAssignment_1 )
            {
             before(grammarAccess.getParamsAccess().getParamsAssignment_1()); 
            // InternalLinguaFranca.g:3336:2: ( rule__Params__ParamsAssignment_1 )
            // InternalLinguaFranca.g:3336:3: rule__Params__ParamsAssignment_1
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
    // InternalLinguaFranca.g:3344:1: rule__Params__Group__2 : rule__Params__Group__2__Impl rule__Params__Group__3 ;
    public final void rule__Params__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3348:1: ( rule__Params__Group__2__Impl rule__Params__Group__3 )
            // InternalLinguaFranca.g:3349:2: rule__Params__Group__2__Impl rule__Params__Group__3
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
    // InternalLinguaFranca.g:3356:1: rule__Params__Group__2__Impl : ( ( rule__Params__Group_2__0 )* ) ;
    public final void rule__Params__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3360:1: ( ( ( rule__Params__Group_2__0 )* ) )
            // InternalLinguaFranca.g:3361:1: ( ( rule__Params__Group_2__0 )* )
            {
            // InternalLinguaFranca.g:3361:1: ( ( rule__Params__Group_2__0 )* )
            // InternalLinguaFranca.g:3362:2: ( rule__Params__Group_2__0 )*
            {
             before(grammarAccess.getParamsAccess().getGroup_2()); 
            // InternalLinguaFranca.g:3363:2: ( rule__Params__Group_2__0 )*
            loop41:
            do {
                int alt41=2;
                int LA41_0 = input.LA(1);

                if ( (LA41_0==27) ) {
                    alt41=1;
                }


                switch (alt41) {
            	case 1 :
            	    // InternalLinguaFranca.g:3363:3: rule__Params__Group_2__0
            	    {
            	    pushFollow(FOLLOW_26);
            	    rule__Params__Group_2__0();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop41;
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
    // InternalLinguaFranca.g:3371:1: rule__Params__Group__3 : rule__Params__Group__3__Impl ;
    public final void rule__Params__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3375:1: ( rule__Params__Group__3__Impl )
            // InternalLinguaFranca.g:3376:2: rule__Params__Group__3__Impl
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
    // InternalLinguaFranca.g:3382:1: rule__Params__Group__3__Impl : ( ')' ) ;
    public final void rule__Params__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3386:1: ( ( ')' ) )
            // InternalLinguaFranca.g:3387:1: ( ')' )
            {
            // InternalLinguaFranca.g:3387:1: ( ')' )
            // InternalLinguaFranca.g:3388:2: ')'
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
    // InternalLinguaFranca.g:3398:1: rule__Params__Group_2__0 : rule__Params__Group_2__0__Impl rule__Params__Group_2__1 ;
    public final void rule__Params__Group_2__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3402:1: ( rule__Params__Group_2__0__Impl rule__Params__Group_2__1 )
            // InternalLinguaFranca.g:3403:2: rule__Params__Group_2__0__Impl rule__Params__Group_2__1
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
    // InternalLinguaFranca.g:3410:1: rule__Params__Group_2__0__Impl : ( ',' ) ;
    public final void rule__Params__Group_2__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3414:1: ( ( ',' ) )
            // InternalLinguaFranca.g:3415:1: ( ',' )
            {
            // InternalLinguaFranca.g:3415:1: ( ',' )
            // InternalLinguaFranca.g:3416:2: ','
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
    // InternalLinguaFranca.g:3425:1: rule__Params__Group_2__1 : rule__Params__Group_2__1__Impl ;
    public final void rule__Params__Group_2__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3429:1: ( rule__Params__Group_2__1__Impl )
            // InternalLinguaFranca.g:3430:2: rule__Params__Group_2__1__Impl
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
    // InternalLinguaFranca.g:3436:1: rule__Params__Group_2__1__Impl : ( ( rule__Params__ParamsAssignment_2_1 ) ) ;
    public final void rule__Params__Group_2__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3440:1: ( ( ( rule__Params__ParamsAssignment_2_1 ) ) )
            // InternalLinguaFranca.g:3441:1: ( ( rule__Params__ParamsAssignment_2_1 ) )
            {
            // InternalLinguaFranca.g:3441:1: ( ( rule__Params__ParamsAssignment_2_1 ) )
            // InternalLinguaFranca.g:3442:2: ( rule__Params__ParamsAssignment_2_1 )
            {
             before(grammarAccess.getParamsAccess().getParamsAssignment_2_1()); 
            // InternalLinguaFranca.g:3443:2: ( rule__Params__ParamsAssignment_2_1 )
            // InternalLinguaFranca.g:3443:3: rule__Params__ParamsAssignment_2_1
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
    // InternalLinguaFranca.g:3452:1: rule__Param__Group__0 : rule__Param__Group__0__Impl rule__Param__Group__1 ;
    public final void rule__Param__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3456:1: ( rule__Param__Group__0__Impl rule__Param__Group__1 )
            // InternalLinguaFranca.g:3457:2: rule__Param__Group__0__Impl rule__Param__Group__1
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
    // InternalLinguaFranca.g:3464:1: rule__Param__Group__0__Impl : ( ( 'const' )? ) ;
    public final void rule__Param__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3468:1: ( ( ( 'const' )? ) )
            // InternalLinguaFranca.g:3469:1: ( ( 'const' )? )
            {
            // InternalLinguaFranca.g:3469:1: ( ( 'const' )? )
            // InternalLinguaFranca.g:3470:2: ( 'const' )?
            {
             before(grammarAccess.getParamAccess().getConstKeyword_0()); 
            // InternalLinguaFranca.g:3471:2: ( 'const' )?
            int alt42=2;
            int LA42_0 = input.LA(1);

            if ( (LA42_0==33) ) {
                alt42=1;
            }
            switch (alt42) {
                case 1 :
                    // InternalLinguaFranca.g:3471:3: 'const'
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
    // InternalLinguaFranca.g:3479:1: rule__Param__Group__1 : rule__Param__Group__1__Impl rule__Param__Group__2 ;
    public final void rule__Param__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3483:1: ( rule__Param__Group__1__Impl rule__Param__Group__2 )
            // InternalLinguaFranca.g:3484:2: rule__Param__Group__1__Impl rule__Param__Group__2
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
    // InternalLinguaFranca.g:3491:1: rule__Param__Group__1__Impl : ( ( rule__Param__NameAssignment_1 ) ) ;
    public final void rule__Param__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3495:1: ( ( ( rule__Param__NameAssignment_1 ) ) )
            // InternalLinguaFranca.g:3496:1: ( ( rule__Param__NameAssignment_1 ) )
            {
            // InternalLinguaFranca.g:3496:1: ( ( rule__Param__NameAssignment_1 ) )
            // InternalLinguaFranca.g:3497:2: ( rule__Param__NameAssignment_1 )
            {
             before(grammarAccess.getParamAccess().getNameAssignment_1()); 
            // InternalLinguaFranca.g:3498:2: ( rule__Param__NameAssignment_1 )
            // InternalLinguaFranca.g:3498:3: rule__Param__NameAssignment_1
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
    // InternalLinguaFranca.g:3506:1: rule__Param__Group__2 : rule__Param__Group__2__Impl rule__Param__Group__3 ;
    public final void rule__Param__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3510:1: ( rule__Param__Group__2__Impl rule__Param__Group__3 )
            // InternalLinguaFranca.g:3511:2: rule__Param__Group__2__Impl rule__Param__Group__3
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
    // InternalLinguaFranca.g:3518:1: rule__Param__Group__2__Impl : ( ( rule__Param__Group_2__0 )? ) ;
    public final void rule__Param__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3522:1: ( ( ( rule__Param__Group_2__0 )? ) )
            // InternalLinguaFranca.g:3523:1: ( ( rule__Param__Group_2__0 )? )
            {
            // InternalLinguaFranca.g:3523:1: ( ( rule__Param__Group_2__0 )? )
            // InternalLinguaFranca.g:3524:2: ( rule__Param__Group_2__0 )?
            {
             before(grammarAccess.getParamAccess().getGroup_2()); 
            // InternalLinguaFranca.g:3525:2: ( rule__Param__Group_2__0 )?
            int alt43=2;
            int LA43_0 = input.LA(1);

            if ( (LA43_0==23) ) {
                alt43=1;
            }
            switch (alt43) {
                case 1 :
                    // InternalLinguaFranca.g:3525:3: rule__Param__Group_2__0
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
    // InternalLinguaFranca.g:3533:1: rule__Param__Group__3 : rule__Param__Group__3__Impl ;
    public final void rule__Param__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3537:1: ( rule__Param__Group__3__Impl )
            // InternalLinguaFranca.g:3538:2: rule__Param__Group__3__Impl
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
    // InternalLinguaFranca.g:3544:1: rule__Param__Group__3__Impl : ( ( rule__Param__Group_3__0 )? ) ;
    public final void rule__Param__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3548:1: ( ( ( rule__Param__Group_3__0 )? ) )
            // InternalLinguaFranca.g:3549:1: ( ( rule__Param__Group_3__0 )? )
            {
            // InternalLinguaFranca.g:3549:1: ( ( rule__Param__Group_3__0 )? )
            // InternalLinguaFranca.g:3550:2: ( rule__Param__Group_3__0 )?
            {
             before(grammarAccess.getParamAccess().getGroup_3()); 
            // InternalLinguaFranca.g:3551:2: ( rule__Param__Group_3__0 )?
            int alt44=2;
            int LA44_0 = input.LA(1);

            if ( (LA44_0==25) ) {
                alt44=1;
            }
            switch (alt44) {
                case 1 :
                    // InternalLinguaFranca.g:3551:3: rule__Param__Group_3__0
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
    // InternalLinguaFranca.g:3560:1: rule__Param__Group_2__0 : rule__Param__Group_2__0__Impl rule__Param__Group_2__1 ;
    public final void rule__Param__Group_2__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3564:1: ( rule__Param__Group_2__0__Impl rule__Param__Group_2__1 )
            // InternalLinguaFranca.g:3565:2: rule__Param__Group_2__0__Impl rule__Param__Group_2__1
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
    // InternalLinguaFranca.g:3572:1: rule__Param__Group_2__0__Impl : ( ':' ) ;
    public final void rule__Param__Group_2__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3576:1: ( ( ':' ) )
            // InternalLinguaFranca.g:3577:1: ( ':' )
            {
            // InternalLinguaFranca.g:3577:1: ( ':' )
            // InternalLinguaFranca.g:3578:2: ':'
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
    // InternalLinguaFranca.g:3587:1: rule__Param__Group_2__1 : rule__Param__Group_2__1__Impl ;
    public final void rule__Param__Group_2__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3591:1: ( rule__Param__Group_2__1__Impl )
            // InternalLinguaFranca.g:3592:2: rule__Param__Group_2__1__Impl
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
    // InternalLinguaFranca.g:3598:1: rule__Param__Group_2__1__Impl : ( ( rule__Param__TypeAssignment_2_1 ) ) ;
    public final void rule__Param__Group_2__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3602:1: ( ( ( rule__Param__TypeAssignment_2_1 ) ) )
            // InternalLinguaFranca.g:3603:1: ( ( rule__Param__TypeAssignment_2_1 ) )
            {
            // InternalLinguaFranca.g:3603:1: ( ( rule__Param__TypeAssignment_2_1 ) )
            // InternalLinguaFranca.g:3604:2: ( rule__Param__TypeAssignment_2_1 )
            {
             before(grammarAccess.getParamAccess().getTypeAssignment_2_1()); 
            // InternalLinguaFranca.g:3605:2: ( rule__Param__TypeAssignment_2_1 )
            // InternalLinguaFranca.g:3605:3: rule__Param__TypeAssignment_2_1
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
    // InternalLinguaFranca.g:3614:1: rule__Param__Group_3__0 : rule__Param__Group_3__0__Impl rule__Param__Group_3__1 ;
    public final void rule__Param__Group_3__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3618:1: ( rule__Param__Group_3__0__Impl rule__Param__Group_3__1 )
            // InternalLinguaFranca.g:3619:2: rule__Param__Group_3__0__Impl rule__Param__Group_3__1
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
    // InternalLinguaFranca.g:3626:1: rule__Param__Group_3__0__Impl : ( '(' ) ;
    public final void rule__Param__Group_3__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3630:1: ( ( '(' ) )
            // InternalLinguaFranca.g:3631:1: ( '(' )
            {
            // InternalLinguaFranca.g:3631:1: ( '(' )
            // InternalLinguaFranca.g:3632:2: '('
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
    // InternalLinguaFranca.g:3641:1: rule__Param__Group_3__1 : rule__Param__Group_3__1__Impl rule__Param__Group_3__2 ;
    public final void rule__Param__Group_3__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3645:1: ( rule__Param__Group_3__1__Impl rule__Param__Group_3__2 )
            // InternalLinguaFranca.g:3646:2: rule__Param__Group_3__1__Impl rule__Param__Group_3__2
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
    // InternalLinguaFranca.g:3653:1: rule__Param__Group_3__1__Impl : ( ( rule__Param__ValueAssignment_3_1 ) ) ;
    public final void rule__Param__Group_3__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3657:1: ( ( ( rule__Param__ValueAssignment_3_1 ) ) )
            // InternalLinguaFranca.g:3658:1: ( ( rule__Param__ValueAssignment_3_1 ) )
            {
            // InternalLinguaFranca.g:3658:1: ( ( rule__Param__ValueAssignment_3_1 ) )
            // InternalLinguaFranca.g:3659:2: ( rule__Param__ValueAssignment_3_1 )
            {
             before(grammarAccess.getParamAccess().getValueAssignment_3_1()); 
            // InternalLinguaFranca.g:3660:2: ( rule__Param__ValueAssignment_3_1 )
            // InternalLinguaFranca.g:3660:3: rule__Param__ValueAssignment_3_1
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
    // InternalLinguaFranca.g:3668:1: rule__Param__Group_3__2 : rule__Param__Group_3__2__Impl ;
    public final void rule__Param__Group_3__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3672:1: ( rule__Param__Group_3__2__Impl )
            // InternalLinguaFranca.g:3673:2: rule__Param__Group_3__2__Impl
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
    // InternalLinguaFranca.g:3679:1: rule__Param__Group_3__2__Impl : ( ')' ) ;
    public final void rule__Param__Group_3__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3683:1: ( ( ')' ) )
            // InternalLinguaFranca.g:3684:1: ( ')' )
            {
            // InternalLinguaFranca.g:3684:1: ( ')' )
            // InternalLinguaFranca.g:3685:2: ')'
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
    // InternalLinguaFranca.g:3695:1: rule__Period__Group__0 : rule__Period__Group__0__Impl rule__Period__Group__1 ;
    public final void rule__Period__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3699:1: ( rule__Period__Group__0__Impl rule__Period__Group__1 )
            // InternalLinguaFranca.g:3700:2: rule__Period__Group__0__Impl rule__Period__Group__1
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
    // InternalLinguaFranca.g:3707:1: rule__Period__Group__0__Impl : ( '(' ) ;
    public final void rule__Period__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3711:1: ( ( '(' ) )
            // InternalLinguaFranca.g:3712:1: ( '(' )
            {
            // InternalLinguaFranca.g:3712:1: ( '(' )
            // InternalLinguaFranca.g:3713:2: '('
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
    // InternalLinguaFranca.g:3722:1: rule__Period__Group__1 : rule__Period__Group__1__Impl rule__Period__Group__2 ;
    public final void rule__Period__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3726:1: ( rule__Period__Group__1__Impl rule__Period__Group__2 )
            // InternalLinguaFranca.g:3727:2: rule__Period__Group__1__Impl rule__Period__Group__2
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
    // InternalLinguaFranca.g:3734:1: rule__Period__Group__1__Impl : ( ( rule__Period__PeriodAssignment_1 ) ) ;
    public final void rule__Period__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3738:1: ( ( ( rule__Period__PeriodAssignment_1 ) ) )
            // InternalLinguaFranca.g:3739:1: ( ( rule__Period__PeriodAssignment_1 ) )
            {
            // InternalLinguaFranca.g:3739:1: ( ( rule__Period__PeriodAssignment_1 ) )
            // InternalLinguaFranca.g:3740:2: ( rule__Period__PeriodAssignment_1 )
            {
             before(grammarAccess.getPeriodAccess().getPeriodAssignment_1()); 
            // InternalLinguaFranca.g:3741:2: ( rule__Period__PeriodAssignment_1 )
            // InternalLinguaFranca.g:3741:3: rule__Period__PeriodAssignment_1
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
    // InternalLinguaFranca.g:3749:1: rule__Period__Group__2 : rule__Period__Group__2__Impl rule__Period__Group__3 ;
    public final void rule__Period__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3753:1: ( rule__Period__Group__2__Impl rule__Period__Group__3 )
            // InternalLinguaFranca.g:3754:2: rule__Period__Group__2__Impl rule__Period__Group__3
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
    // InternalLinguaFranca.g:3761:1: rule__Period__Group__2__Impl : ( ( rule__Period__Group_2__0 )? ) ;
    public final void rule__Period__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3765:1: ( ( ( rule__Period__Group_2__0 )? ) )
            // InternalLinguaFranca.g:3766:1: ( ( rule__Period__Group_2__0 )? )
            {
            // InternalLinguaFranca.g:3766:1: ( ( rule__Period__Group_2__0 )? )
            // InternalLinguaFranca.g:3767:2: ( rule__Period__Group_2__0 )?
            {
             before(grammarAccess.getPeriodAccess().getGroup_2()); 
            // InternalLinguaFranca.g:3768:2: ( rule__Period__Group_2__0 )?
            int alt45=2;
            int LA45_0 = input.LA(1);

            if ( (LA45_0==27) ) {
                alt45=1;
            }
            switch (alt45) {
                case 1 :
                    // InternalLinguaFranca.g:3768:3: rule__Period__Group_2__0
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
    // InternalLinguaFranca.g:3776:1: rule__Period__Group__3 : rule__Period__Group__3__Impl ;
    public final void rule__Period__Group__3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3780:1: ( rule__Period__Group__3__Impl )
            // InternalLinguaFranca.g:3781:2: rule__Period__Group__3__Impl
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
    // InternalLinguaFranca.g:3787:1: rule__Period__Group__3__Impl : ( ')' ) ;
    public final void rule__Period__Group__3__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3791:1: ( ( ')' ) )
            // InternalLinguaFranca.g:3792:1: ( ')' )
            {
            // InternalLinguaFranca.g:3792:1: ( ')' )
            // InternalLinguaFranca.g:3793:2: ')'
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
    // InternalLinguaFranca.g:3803:1: rule__Period__Group_2__0 : rule__Period__Group_2__0__Impl rule__Period__Group_2__1 ;
    public final void rule__Period__Group_2__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3807:1: ( rule__Period__Group_2__0__Impl rule__Period__Group_2__1 )
            // InternalLinguaFranca.g:3808:2: rule__Period__Group_2__0__Impl rule__Period__Group_2__1
            {
            pushFollow(FOLLOW_36);
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
    // InternalLinguaFranca.g:3815:1: rule__Period__Group_2__0__Impl : ( ',' ) ;
    public final void rule__Period__Group_2__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3819:1: ( ( ',' ) )
            // InternalLinguaFranca.g:3820:1: ( ',' )
            {
            // InternalLinguaFranca.g:3820:1: ( ',' )
            // InternalLinguaFranca.g:3821:2: ','
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
    // InternalLinguaFranca.g:3830:1: rule__Period__Group_2__1 : rule__Period__Group_2__1__Impl ;
    public final void rule__Period__Group_2__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3834:1: ( rule__Period__Group_2__1__Impl )
            // InternalLinguaFranca.g:3835:2: rule__Period__Group_2__1__Impl
            {
            pushFollow(FOLLOW_2);
            rule__Period__Group_2__1__Impl();

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
    // InternalLinguaFranca.g:3841:1: rule__Period__Group_2__1__Impl : ( ( rule__Period__Alternatives_2_1 ) ) ;
    public final void rule__Period__Group_2__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3845:1: ( ( ( rule__Period__Alternatives_2_1 ) ) )
            // InternalLinguaFranca.g:3846:1: ( ( rule__Period__Alternatives_2_1 ) )
            {
            // InternalLinguaFranca.g:3846:1: ( ( rule__Period__Alternatives_2_1 ) )
            // InternalLinguaFranca.g:3847:2: ( rule__Period__Alternatives_2_1 )
            {
             before(grammarAccess.getPeriodAccess().getAlternatives_2_1()); 
            // InternalLinguaFranca.g:3848:2: ( rule__Period__Alternatives_2_1 )
            // InternalLinguaFranca.g:3848:3: rule__Period__Alternatives_2_1
            {
            pushFollow(FOLLOW_2);
            rule__Period__Alternatives_2_1();

            state._fsp--;


            }

             after(grammarAccess.getPeriodAccess().getAlternatives_2_1()); 

            }


            }

        }
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


    // $ANTLR start "rule__Port__Group_1__0"
    // InternalLinguaFranca.g:3857:1: rule__Port__Group_1__0 : rule__Port__Group_1__0__Impl rule__Port__Group_1__1 ;
    public final void rule__Port__Group_1__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3861:1: ( rule__Port__Group_1__0__Impl rule__Port__Group_1__1 )
            // InternalLinguaFranca.g:3862:2: rule__Port__Group_1__0__Impl rule__Port__Group_1__1
            {
            pushFollow(FOLLOW_37);
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
    // InternalLinguaFranca.g:3869:1: rule__Port__Group_1__0__Impl : ( RULE_ID ) ;
    public final void rule__Port__Group_1__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3873:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:3874:1: ( RULE_ID )
            {
            // InternalLinguaFranca.g:3874:1: ( RULE_ID )
            // InternalLinguaFranca.g:3875:2: RULE_ID
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
    // InternalLinguaFranca.g:3884:1: rule__Port__Group_1__1 : rule__Port__Group_1__1__Impl rule__Port__Group_1__2 ;
    public final void rule__Port__Group_1__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3888:1: ( rule__Port__Group_1__1__Impl rule__Port__Group_1__2 )
            // InternalLinguaFranca.g:3889:2: rule__Port__Group_1__1__Impl rule__Port__Group_1__2
            {
            pushFollow(FOLLOW_38);
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
    // InternalLinguaFranca.g:3896:1: rule__Port__Group_1__1__Impl : ( '.' ) ;
    public final void rule__Port__Group_1__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3900:1: ( ( '.' ) )
            // InternalLinguaFranca.g:3901:1: ( '.' )
            {
            // InternalLinguaFranca.g:3901:1: ( '.' )
            // InternalLinguaFranca.g:3902:2: '.'
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
    // InternalLinguaFranca.g:3911:1: rule__Port__Group_1__2 : rule__Port__Group_1__2__Impl ;
    public final void rule__Port__Group_1__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3915:1: ( rule__Port__Group_1__2__Impl )
            // InternalLinguaFranca.g:3916:2: rule__Port__Group_1__2__Impl
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
    // InternalLinguaFranca.g:3922:1: rule__Port__Group_1__2__Impl : ( ( rule__Port__Alternatives_1_2 ) ) ;
    public final void rule__Port__Group_1__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3926:1: ( ( ( rule__Port__Alternatives_1_2 ) ) )
            // InternalLinguaFranca.g:3927:1: ( ( rule__Port__Alternatives_1_2 ) )
            {
            // InternalLinguaFranca.g:3927:1: ( ( rule__Port__Alternatives_1_2 ) )
            // InternalLinguaFranca.g:3928:2: ( rule__Port__Alternatives_1_2 )
            {
             before(grammarAccess.getPortAccess().getAlternatives_1_2()); 
            // InternalLinguaFranca.g:3929:2: ( rule__Port__Alternatives_1_2 )
            // InternalLinguaFranca.g:3929:3: rule__Port__Alternatives_1_2
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
    // InternalLinguaFranca.g:3938:1: rule__Sets__Group__0 : rule__Sets__Group__0__Impl rule__Sets__Group__1 ;
    public final void rule__Sets__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3942:1: ( rule__Sets__Group__0__Impl rule__Sets__Group__1 )
            // InternalLinguaFranca.g:3943:2: rule__Sets__Group__0__Impl rule__Sets__Group__1
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
    // InternalLinguaFranca.g:3950:1: rule__Sets__Group__0__Impl : ( '->' ) ;
    public final void rule__Sets__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3954:1: ( ( '->' ) )
            // InternalLinguaFranca.g:3955:1: ( '->' )
            {
            // InternalLinguaFranca.g:3955:1: ( '->' )
            // InternalLinguaFranca.g:3956:2: '->'
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
    // InternalLinguaFranca.g:3965:1: rule__Sets__Group__1 : rule__Sets__Group__1__Impl rule__Sets__Group__2 ;
    public final void rule__Sets__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3969:1: ( rule__Sets__Group__1__Impl rule__Sets__Group__2 )
            // InternalLinguaFranca.g:3970:2: rule__Sets__Group__1__Impl rule__Sets__Group__2
            {
            pushFollow(FOLLOW_25);
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
    // InternalLinguaFranca.g:3977:1: rule__Sets__Group__1__Impl : ( ( rule__Sets__SetsAssignment_1 ) ) ;
    public final void rule__Sets__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3981:1: ( ( ( rule__Sets__SetsAssignment_1 ) ) )
            // InternalLinguaFranca.g:3982:1: ( ( rule__Sets__SetsAssignment_1 ) )
            {
            // InternalLinguaFranca.g:3982:1: ( ( rule__Sets__SetsAssignment_1 ) )
            // InternalLinguaFranca.g:3983:2: ( rule__Sets__SetsAssignment_1 )
            {
             before(grammarAccess.getSetsAccess().getSetsAssignment_1()); 
            // InternalLinguaFranca.g:3984:2: ( rule__Sets__SetsAssignment_1 )
            // InternalLinguaFranca.g:3984:3: rule__Sets__SetsAssignment_1
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
    // InternalLinguaFranca.g:3992:1: rule__Sets__Group__2 : rule__Sets__Group__2__Impl ;
    public final void rule__Sets__Group__2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:3996:1: ( rule__Sets__Group__2__Impl )
            // InternalLinguaFranca.g:3997:2: rule__Sets__Group__2__Impl
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
    // InternalLinguaFranca.g:4003:1: rule__Sets__Group__2__Impl : ( ( rule__Sets__Group_2__0 )? ) ;
    public final void rule__Sets__Group__2__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4007:1: ( ( ( rule__Sets__Group_2__0 )? ) )
            // InternalLinguaFranca.g:4008:1: ( ( rule__Sets__Group_2__0 )? )
            {
            // InternalLinguaFranca.g:4008:1: ( ( rule__Sets__Group_2__0 )? )
            // InternalLinguaFranca.g:4009:2: ( rule__Sets__Group_2__0 )?
            {
             before(grammarAccess.getSetsAccess().getGroup_2()); 
            // InternalLinguaFranca.g:4010:2: ( rule__Sets__Group_2__0 )?
            int alt46=2;
            int LA46_0 = input.LA(1);

            if ( (LA46_0==27) ) {
                alt46=1;
            }
            switch (alt46) {
                case 1 :
                    // InternalLinguaFranca.g:4010:3: rule__Sets__Group_2__0
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
    // InternalLinguaFranca.g:4019:1: rule__Sets__Group_2__0 : rule__Sets__Group_2__0__Impl rule__Sets__Group_2__1 ;
    public final void rule__Sets__Group_2__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4023:1: ( rule__Sets__Group_2__0__Impl rule__Sets__Group_2__1 )
            // InternalLinguaFranca.g:4024:2: rule__Sets__Group_2__0__Impl rule__Sets__Group_2__1
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
    // InternalLinguaFranca.g:4031:1: rule__Sets__Group_2__0__Impl : ( ',' ) ;
    public final void rule__Sets__Group_2__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4035:1: ( ( ',' ) )
            // InternalLinguaFranca.g:4036:1: ( ',' )
            {
            // InternalLinguaFranca.g:4036:1: ( ',' )
            // InternalLinguaFranca.g:4037:2: ','
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
    // InternalLinguaFranca.g:4046:1: rule__Sets__Group_2__1 : rule__Sets__Group_2__1__Impl ;
    public final void rule__Sets__Group_2__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4050:1: ( rule__Sets__Group_2__1__Impl )
            // InternalLinguaFranca.g:4051:2: rule__Sets__Group_2__1__Impl
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
    // InternalLinguaFranca.g:4057:1: rule__Sets__Group_2__1__Impl : ( ( rule__Sets__SetsAssignment_2_1 ) ) ;
    public final void rule__Sets__Group_2__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4061:1: ( ( ( rule__Sets__SetsAssignment_2_1 ) ) )
            // InternalLinguaFranca.g:4062:1: ( ( rule__Sets__SetsAssignment_2_1 ) )
            {
            // InternalLinguaFranca.g:4062:1: ( ( rule__Sets__SetsAssignment_2_1 ) )
            // InternalLinguaFranca.g:4063:2: ( rule__Sets__SetsAssignment_2_1 )
            {
             before(grammarAccess.getSetsAccess().getSetsAssignment_2_1()); 
            // InternalLinguaFranca.g:4064:2: ( rule__Sets__SetsAssignment_2_1 )
            // InternalLinguaFranca.g:4064:3: rule__Sets__SetsAssignment_2_1
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
    // InternalLinguaFranca.g:4073:1: rule__Path__Group__0 : rule__Path__Group__0__Impl rule__Path__Group__1 ;
    public final void rule__Path__Group__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4077:1: ( rule__Path__Group__0__Impl rule__Path__Group__1 )
            // InternalLinguaFranca.g:4078:2: rule__Path__Group__0__Impl rule__Path__Group__1
            {
            pushFollow(FOLLOW_37);
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
    // InternalLinguaFranca.g:4085:1: rule__Path__Group__0__Impl : ( RULE_ID ) ;
    public final void rule__Path__Group__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4089:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4090:1: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4090:1: ( RULE_ID )
            // InternalLinguaFranca.g:4091:2: RULE_ID
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
    // InternalLinguaFranca.g:4100:1: rule__Path__Group__1 : rule__Path__Group__1__Impl ;
    public final void rule__Path__Group__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4104:1: ( rule__Path__Group__1__Impl )
            // InternalLinguaFranca.g:4105:2: rule__Path__Group__1__Impl
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
    // InternalLinguaFranca.g:4111:1: rule__Path__Group__1__Impl : ( ( rule__Path__Group_1__0 )* ) ;
    public final void rule__Path__Group__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4115:1: ( ( ( rule__Path__Group_1__0 )* ) )
            // InternalLinguaFranca.g:4116:1: ( ( rule__Path__Group_1__0 )* )
            {
            // InternalLinguaFranca.g:4116:1: ( ( rule__Path__Group_1__0 )* )
            // InternalLinguaFranca.g:4117:2: ( rule__Path__Group_1__0 )*
            {
             before(grammarAccess.getPathAccess().getGroup_1()); 
            // InternalLinguaFranca.g:4118:2: ( rule__Path__Group_1__0 )*
            loop47:
            do {
                int alt47=2;
                int LA47_0 = input.LA(1);

                if ( (LA47_0==34) ) {
                    alt47=1;
                }


                switch (alt47) {
            	case 1 :
            	    // InternalLinguaFranca.g:4118:3: rule__Path__Group_1__0
            	    {
            	    pushFollow(FOLLOW_39);
            	    rule__Path__Group_1__0();

            	    state._fsp--;


            	    }
            	    break;

            	default :
            	    break loop47;
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
    // InternalLinguaFranca.g:4127:1: rule__Path__Group_1__0 : rule__Path__Group_1__0__Impl rule__Path__Group_1__1 ;
    public final void rule__Path__Group_1__0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4131:1: ( rule__Path__Group_1__0__Impl rule__Path__Group_1__1 )
            // InternalLinguaFranca.g:4132:2: rule__Path__Group_1__0__Impl rule__Path__Group_1__1
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
    // InternalLinguaFranca.g:4139:1: rule__Path__Group_1__0__Impl : ( '.' ) ;
    public final void rule__Path__Group_1__0__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4143:1: ( ( '.' ) )
            // InternalLinguaFranca.g:4144:1: ( '.' )
            {
            // InternalLinguaFranca.g:4144:1: ( '.' )
            // InternalLinguaFranca.g:4145:2: '.'
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
    // InternalLinguaFranca.g:4154:1: rule__Path__Group_1__1 : rule__Path__Group_1__1__Impl ;
    public final void rule__Path__Group_1__1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4158:1: ( rule__Path__Group_1__1__Impl )
            // InternalLinguaFranca.g:4159:2: rule__Path__Group_1__1__Impl
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
    // InternalLinguaFranca.g:4165:1: rule__Path__Group_1__1__Impl : ( RULE_ID ) ;
    public final void rule__Path__Group_1__1__Impl() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4169:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4170:1: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4170:1: ( RULE_ID )
            // InternalLinguaFranca.g:4171:2: RULE_ID
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
    // InternalLinguaFranca.g:4181:1: rule__Model__TargetAssignment_0 : ( ruleTarget ) ;
    public final void rule__Model__TargetAssignment_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4185:1: ( ( ruleTarget ) )
            // InternalLinguaFranca.g:4186:2: ( ruleTarget )
            {
            // InternalLinguaFranca.g:4186:2: ( ruleTarget )
            // InternalLinguaFranca.g:4187:3: ruleTarget
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
    // InternalLinguaFranca.g:4196:1: rule__Model__ImportsAssignment_1 : ( ruleImport ) ;
    public final void rule__Model__ImportsAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4200:1: ( ( ruleImport ) )
            // InternalLinguaFranca.g:4201:2: ( ruleImport )
            {
            // InternalLinguaFranca.g:4201:2: ( ruleImport )
            // InternalLinguaFranca.g:4202:3: ruleImport
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
    // InternalLinguaFranca.g:4211:1: rule__Model__BlocksAssignment_2 : ( ( rule__Model__BlocksAlternatives_2_0 ) ) ;
    public final void rule__Model__BlocksAssignment_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4215:1: ( ( ( rule__Model__BlocksAlternatives_2_0 ) ) )
            // InternalLinguaFranca.g:4216:2: ( ( rule__Model__BlocksAlternatives_2_0 ) )
            {
            // InternalLinguaFranca.g:4216:2: ( ( rule__Model__BlocksAlternatives_2_0 ) )
            // InternalLinguaFranca.g:4217:3: ( rule__Model__BlocksAlternatives_2_0 )
            {
             before(grammarAccess.getModelAccess().getBlocksAlternatives_2_0()); 
            // InternalLinguaFranca.g:4218:3: ( rule__Model__BlocksAlternatives_2_0 )
            // InternalLinguaFranca.g:4218:4: rule__Model__BlocksAlternatives_2_0
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
    // InternalLinguaFranca.g:4226:1: rule__Target__NameAssignment_1 : ( RULE_ID ) ;
    public final void rule__Target__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4230:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4231:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4231:2: ( RULE_ID )
            // InternalLinguaFranca.g:4232:3: RULE_ID
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
    // InternalLinguaFranca.g:4241:1: rule__Import__NameAssignment_1 : ( rulePath ) ;
    public final void rule__Import__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4245:1: ( ( rulePath ) )
            // InternalLinguaFranca.g:4246:2: ( rulePath )
            {
            // InternalLinguaFranca.g:4246:2: ( rulePath )
            // InternalLinguaFranca.g:4247:3: rulePath
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


    // $ANTLR start "rule__Actor__NameAssignment_1"
    // InternalLinguaFranca.g:4256:1: rule__Actor__NameAssignment_1 : ( RULE_ID ) ;
    public final void rule__Actor__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4260:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4261:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4261:2: ( RULE_ID )
            // InternalLinguaFranca.g:4262:3: RULE_ID
            {
             before(grammarAccess.getActorAccess().getNameIDTerminalRuleCall_1_0()); 
            match(input,RULE_ID,FOLLOW_2); 
             after(grammarAccess.getActorAccess().getNameIDTerminalRuleCall_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Actor__NameAssignment_1"


    // $ANTLR start "rule__Actor__ParametersAssignment_2"
    // InternalLinguaFranca.g:4271:1: rule__Actor__ParametersAssignment_2 : ( ruleParams ) ;
    public final void rule__Actor__ParametersAssignment_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4275:1: ( ( ruleParams ) )
            // InternalLinguaFranca.g:4276:2: ( ruleParams )
            {
            // InternalLinguaFranca.g:4276:2: ( ruleParams )
            // InternalLinguaFranca.g:4277:3: ruleParams
            {
             before(grammarAccess.getActorAccess().getParametersParamsParserRuleCall_2_0()); 
            pushFollow(FOLLOW_2);
            ruleParams();

            state._fsp--;

             after(grammarAccess.getActorAccess().getParametersParamsParserRuleCall_2_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Actor__ParametersAssignment_2"


    // $ANTLR start "rule__Actor__InputsAssignment_4"
    // InternalLinguaFranca.g:4286:1: rule__Actor__InputsAssignment_4 : ( ruleInput ) ;
    public final void rule__Actor__InputsAssignment_4() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4290:1: ( ( ruleInput ) )
            // InternalLinguaFranca.g:4291:2: ( ruleInput )
            {
            // InternalLinguaFranca.g:4291:2: ( ruleInput )
            // InternalLinguaFranca.g:4292:3: ruleInput
            {
             before(grammarAccess.getActorAccess().getInputsInputParserRuleCall_4_0()); 
            pushFollow(FOLLOW_2);
            ruleInput();

            state._fsp--;

             after(grammarAccess.getActorAccess().getInputsInputParserRuleCall_4_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Actor__InputsAssignment_4"


    // $ANTLR start "rule__Actor__OutputsAssignment_5"
    // InternalLinguaFranca.g:4301:1: rule__Actor__OutputsAssignment_5 : ( ruleOutput ) ;
    public final void rule__Actor__OutputsAssignment_5() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4305:1: ( ( ruleOutput ) )
            // InternalLinguaFranca.g:4306:2: ( ruleOutput )
            {
            // InternalLinguaFranca.g:4306:2: ( ruleOutput )
            // InternalLinguaFranca.g:4307:3: ruleOutput
            {
             before(grammarAccess.getActorAccess().getOutputsOutputParserRuleCall_5_0()); 
            pushFollow(FOLLOW_2);
            ruleOutput();

            state._fsp--;

             after(grammarAccess.getActorAccess().getOutputsOutputParserRuleCall_5_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Actor__OutputsAssignment_5"


    // $ANTLR start "rule__Actor__TriggersAssignment_6"
    // InternalLinguaFranca.g:4316:1: rule__Actor__TriggersAssignment_6 : ( ruleTrigger ) ;
    public final void rule__Actor__TriggersAssignment_6() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4320:1: ( ( ruleTrigger ) )
            // InternalLinguaFranca.g:4321:2: ( ruleTrigger )
            {
            // InternalLinguaFranca.g:4321:2: ( ruleTrigger )
            // InternalLinguaFranca.g:4322:3: ruleTrigger
            {
             before(grammarAccess.getActorAccess().getTriggersTriggerParserRuleCall_6_0()); 
            pushFollow(FOLLOW_2);
            ruleTrigger();

            state._fsp--;

             after(grammarAccess.getActorAccess().getTriggersTriggerParserRuleCall_6_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Actor__TriggersAssignment_6"


    // $ANTLR start "rule__Actor__PreambleAssignment_7"
    // InternalLinguaFranca.g:4331:1: rule__Actor__PreambleAssignment_7 : ( rulePreamble ) ;
    public final void rule__Actor__PreambleAssignment_7() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4335:1: ( ( rulePreamble ) )
            // InternalLinguaFranca.g:4336:2: ( rulePreamble )
            {
            // InternalLinguaFranca.g:4336:2: ( rulePreamble )
            // InternalLinguaFranca.g:4337:3: rulePreamble
            {
             before(grammarAccess.getActorAccess().getPreamblePreambleParserRuleCall_7_0()); 
            pushFollow(FOLLOW_2);
            rulePreamble();

            state._fsp--;

             after(grammarAccess.getActorAccess().getPreamblePreambleParserRuleCall_7_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Actor__PreambleAssignment_7"


    // $ANTLR start "rule__Actor__InitializeAssignment_8"
    // InternalLinguaFranca.g:4346:1: rule__Actor__InitializeAssignment_8 : ( ruleInitialize ) ;
    public final void rule__Actor__InitializeAssignment_8() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4350:1: ( ( ruleInitialize ) )
            // InternalLinguaFranca.g:4351:2: ( ruleInitialize )
            {
            // InternalLinguaFranca.g:4351:2: ( ruleInitialize )
            // InternalLinguaFranca.g:4352:3: ruleInitialize
            {
             before(grammarAccess.getActorAccess().getInitializeInitializeParserRuleCall_8_0()); 
            pushFollow(FOLLOW_2);
            ruleInitialize();

            state._fsp--;

             after(grammarAccess.getActorAccess().getInitializeInitializeParserRuleCall_8_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Actor__InitializeAssignment_8"


    // $ANTLR start "rule__Actor__ReactionsAssignment_9"
    // InternalLinguaFranca.g:4361:1: rule__Actor__ReactionsAssignment_9 : ( ruleReaction ) ;
    public final void rule__Actor__ReactionsAssignment_9() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4365:1: ( ( ruleReaction ) )
            // InternalLinguaFranca.g:4366:2: ( ruleReaction )
            {
            // InternalLinguaFranca.g:4366:2: ( ruleReaction )
            // InternalLinguaFranca.g:4367:3: ruleReaction
            {
             before(grammarAccess.getActorAccess().getReactionsReactionParserRuleCall_9_0()); 
            pushFollow(FOLLOW_2);
            ruleReaction();

            state._fsp--;

             after(grammarAccess.getActorAccess().getReactionsReactionParserRuleCall_9_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Actor__ReactionsAssignment_9"


    // $ANTLR start "rule__Composite__NameAssignment_1"
    // InternalLinguaFranca.g:4376:1: rule__Composite__NameAssignment_1 : ( RULE_ID ) ;
    public final void rule__Composite__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4380:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4381:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4381:2: ( RULE_ID )
            // InternalLinguaFranca.g:4382:3: RULE_ID
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
    // InternalLinguaFranca.g:4391:1: rule__Composite__ParametersAssignment_2 : ( ruleParams ) ;
    public final void rule__Composite__ParametersAssignment_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4395:1: ( ( ruleParams ) )
            // InternalLinguaFranca.g:4396:2: ( ruleParams )
            {
            // InternalLinguaFranca.g:4396:2: ( ruleParams )
            // InternalLinguaFranca.g:4397:3: ruleParams
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
    // InternalLinguaFranca.g:4406:1: rule__Composite__InputsAssignment_4 : ( ruleInput ) ;
    public final void rule__Composite__InputsAssignment_4() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4410:1: ( ( ruleInput ) )
            // InternalLinguaFranca.g:4411:2: ( ruleInput )
            {
            // InternalLinguaFranca.g:4411:2: ( ruleInput )
            // InternalLinguaFranca.g:4412:3: ruleInput
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
    // InternalLinguaFranca.g:4421:1: rule__Composite__OutputsAssignment_5 : ( ruleOutput ) ;
    public final void rule__Composite__OutputsAssignment_5() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4425:1: ( ( ruleOutput ) )
            // InternalLinguaFranca.g:4426:2: ( ruleOutput )
            {
            // InternalLinguaFranca.g:4426:2: ( ruleOutput )
            // InternalLinguaFranca.g:4427:3: ruleOutput
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


    // $ANTLR start "rule__Composite__TriggersAssignment_6"
    // InternalLinguaFranca.g:4436:1: rule__Composite__TriggersAssignment_6 : ( ruleTrigger ) ;
    public final void rule__Composite__TriggersAssignment_6() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4440:1: ( ( ruleTrigger ) )
            // InternalLinguaFranca.g:4441:2: ( ruleTrigger )
            {
            // InternalLinguaFranca.g:4441:2: ( ruleTrigger )
            // InternalLinguaFranca.g:4442:3: ruleTrigger
            {
             before(grammarAccess.getCompositeAccess().getTriggersTriggerParserRuleCall_6_0()); 
            pushFollow(FOLLOW_2);
            ruleTrigger();

            state._fsp--;

             after(grammarAccess.getCompositeAccess().getTriggersTriggerParserRuleCall_6_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__TriggersAssignment_6"


    // $ANTLR start "rule__Composite__PreambleAssignment_7"
    // InternalLinguaFranca.g:4451:1: rule__Composite__PreambleAssignment_7 : ( rulePreamble ) ;
    public final void rule__Composite__PreambleAssignment_7() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4455:1: ( ( rulePreamble ) )
            // InternalLinguaFranca.g:4456:2: ( rulePreamble )
            {
            // InternalLinguaFranca.g:4456:2: ( rulePreamble )
            // InternalLinguaFranca.g:4457:3: rulePreamble
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


    // $ANTLR start "rule__Composite__InitializeAssignment_8"
    // InternalLinguaFranca.g:4466:1: rule__Composite__InitializeAssignment_8 : ( ruleInitialize ) ;
    public final void rule__Composite__InitializeAssignment_8() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4470:1: ( ( ruleInitialize ) )
            // InternalLinguaFranca.g:4471:2: ( ruleInitialize )
            {
            // InternalLinguaFranca.g:4471:2: ( ruleInitialize )
            // InternalLinguaFranca.g:4472:3: ruleInitialize
            {
             before(grammarAccess.getCompositeAccess().getInitializeInitializeParserRuleCall_8_0()); 
            pushFollow(FOLLOW_2);
            ruleInitialize();

            state._fsp--;

             after(grammarAccess.getCompositeAccess().getInitializeInitializeParserRuleCall_8_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Composite__InitializeAssignment_8"


    // $ANTLR start "rule__Composite__ReactionsAssignment_9"
    // InternalLinguaFranca.g:4481:1: rule__Composite__ReactionsAssignment_9 : ( ruleReaction ) ;
    public final void rule__Composite__ReactionsAssignment_9() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4485:1: ( ( ruleReaction ) )
            // InternalLinguaFranca.g:4486:2: ( ruleReaction )
            {
            // InternalLinguaFranca.g:4486:2: ( ruleReaction )
            // InternalLinguaFranca.g:4487:3: ruleReaction
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
    // InternalLinguaFranca.g:4496:1: rule__Composite__InstancesAssignment_10 : ( ruleInstance ) ;
    public final void rule__Composite__InstancesAssignment_10() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4500:1: ( ( ruleInstance ) )
            // InternalLinguaFranca.g:4501:2: ( ruleInstance )
            {
            // InternalLinguaFranca.g:4501:2: ( ruleInstance )
            // InternalLinguaFranca.g:4502:3: ruleInstance
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
    // InternalLinguaFranca.g:4511:1: rule__Composite__ConnectionsAssignment_11 : ( ruleConnection ) ;
    public final void rule__Composite__ConnectionsAssignment_11() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4515:1: ( ( ruleConnection ) )
            // InternalLinguaFranca.g:4516:2: ( ruleConnection )
            {
            // InternalLinguaFranca.g:4516:2: ( ruleConnection )
            // InternalLinguaFranca.g:4517:3: ruleConnection
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
    // InternalLinguaFranca.g:4526:1: rule__Input__NameAssignment_1 : ( ( rule__Input__NameAlternatives_1_0 ) ) ;
    public final void rule__Input__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4530:1: ( ( ( rule__Input__NameAlternatives_1_0 ) ) )
            // InternalLinguaFranca.g:4531:2: ( ( rule__Input__NameAlternatives_1_0 ) )
            {
            // InternalLinguaFranca.g:4531:2: ( ( rule__Input__NameAlternatives_1_0 ) )
            // InternalLinguaFranca.g:4532:3: ( rule__Input__NameAlternatives_1_0 )
            {
             before(grammarAccess.getInputAccess().getNameAlternatives_1_0()); 
            // InternalLinguaFranca.g:4533:3: ( rule__Input__NameAlternatives_1_0 )
            // InternalLinguaFranca.g:4533:4: rule__Input__NameAlternatives_1_0
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
    // InternalLinguaFranca.g:4541:1: rule__Input__TypeAssignment_2_1 : ( ruleType ) ;
    public final void rule__Input__TypeAssignment_2_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4545:1: ( ( ruleType ) )
            // InternalLinguaFranca.g:4546:2: ( ruleType )
            {
            // InternalLinguaFranca.g:4546:2: ( ruleType )
            // InternalLinguaFranca.g:4547:3: ruleType
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
    // InternalLinguaFranca.g:4556:1: rule__Output__NameAssignment_1 : ( ( rule__Output__NameAlternatives_1_0 ) ) ;
    public final void rule__Output__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4560:1: ( ( ( rule__Output__NameAlternatives_1_0 ) ) )
            // InternalLinguaFranca.g:4561:2: ( ( rule__Output__NameAlternatives_1_0 ) )
            {
            // InternalLinguaFranca.g:4561:2: ( ( rule__Output__NameAlternatives_1_0 ) )
            // InternalLinguaFranca.g:4562:3: ( rule__Output__NameAlternatives_1_0 )
            {
             before(grammarAccess.getOutputAccess().getNameAlternatives_1_0()); 
            // InternalLinguaFranca.g:4563:3: ( rule__Output__NameAlternatives_1_0 )
            // InternalLinguaFranca.g:4563:4: rule__Output__NameAlternatives_1_0
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
    // InternalLinguaFranca.g:4571:1: rule__Output__TypeAssignment_2_1 : ( ruleType ) ;
    public final void rule__Output__TypeAssignment_2_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4575:1: ( ( ruleType ) )
            // InternalLinguaFranca.g:4576:2: ( ruleType )
            {
            // InternalLinguaFranca.g:4576:2: ( ruleType )
            // InternalLinguaFranca.g:4577:3: ruleType
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


    // $ANTLR start "rule__Trigger__NameAssignment_1"
    // InternalLinguaFranca.g:4586:1: rule__Trigger__NameAssignment_1 : ( ( rule__Trigger__NameAlternatives_1_0 ) ) ;
    public final void rule__Trigger__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4590:1: ( ( ( rule__Trigger__NameAlternatives_1_0 ) ) )
            // InternalLinguaFranca.g:4591:2: ( ( rule__Trigger__NameAlternatives_1_0 ) )
            {
            // InternalLinguaFranca.g:4591:2: ( ( rule__Trigger__NameAlternatives_1_0 ) )
            // InternalLinguaFranca.g:4592:3: ( rule__Trigger__NameAlternatives_1_0 )
            {
             before(grammarAccess.getTriggerAccess().getNameAlternatives_1_0()); 
            // InternalLinguaFranca.g:4593:3: ( rule__Trigger__NameAlternatives_1_0 )
            // InternalLinguaFranca.g:4593:4: rule__Trigger__NameAlternatives_1_0
            {
            pushFollow(FOLLOW_2);
            rule__Trigger__NameAlternatives_1_0();

            state._fsp--;


            }

             after(grammarAccess.getTriggerAccess().getNameAlternatives_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Trigger__NameAssignment_1"


    // $ANTLR start "rule__Trigger__PeriodAssignment_2"
    // InternalLinguaFranca.g:4601:1: rule__Trigger__PeriodAssignment_2 : ( rulePeriod ) ;
    public final void rule__Trigger__PeriodAssignment_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4605:1: ( ( rulePeriod ) )
            // InternalLinguaFranca.g:4606:2: ( rulePeriod )
            {
            // InternalLinguaFranca.g:4606:2: ( rulePeriod )
            // InternalLinguaFranca.g:4607:3: rulePeriod
            {
             before(grammarAccess.getTriggerAccess().getPeriodPeriodParserRuleCall_2_0()); 
            pushFollow(FOLLOW_2);
            rulePeriod();

            state._fsp--;

             after(grammarAccess.getTriggerAccess().getPeriodPeriodParserRuleCall_2_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Trigger__PeriodAssignment_2"


    // $ANTLR start "rule__Reaction__TriggersAssignment_1_1_0"
    // InternalLinguaFranca.g:4616:1: rule__Reaction__TriggersAssignment_1_1_0 : ( RULE_ID ) ;
    public final void rule__Reaction__TriggersAssignment_1_1_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4620:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4621:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4621:2: ( RULE_ID )
            // InternalLinguaFranca.g:4622:3: RULE_ID
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
    // InternalLinguaFranca.g:4631:1: rule__Reaction__TriggersAssignment_1_1_1_1 : ( RULE_ID ) ;
    public final void rule__Reaction__TriggersAssignment_1_1_1_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4635:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4636:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4636:2: ( RULE_ID )
            // InternalLinguaFranca.g:4637:3: RULE_ID
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
    // InternalLinguaFranca.g:4646:1: rule__Reaction__GetsAssignment_2 : ( ruleGets ) ;
    public final void rule__Reaction__GetsAssignment_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4650:1: ( ( ruleGets ) )
            // InternalLinguaFranca.g:4651:2: ( ruleGets )
            {
            // InternalLinguaFranca.g:4651:2: ( ruleGets )
            // InternalLinguaFranca.g:4652:3: ruleGets
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
    // InternalLinguaFranca.g:4661:1: rule__Reaction__SetsAssignment_3 : ( ruleSets ) ;
    public final void rule__Reaction__SetsAssignment_3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4665:1: ( ( ruleSets ) )
            // InternalLinguaFranca.g:4666:2: ( ruleSets )
            {
            // InternalLinguaFranca.g:4666:2: ( ruleSets )
            // InternalLinguaFranca.g:4667:3: ruleSets
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
    // InternalLinguaFranca.g:4676:1: rule__Reaction__CodeAssignment_4 : ( RULE_CODE ) ;
    public final void rule__Reaction__CodeAssignment_4() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4680:1: ( ( RULE_CODE ) )
            // InternalLinguaFranca.g:4681:2: ( RULE_CODE )
            {
            // InternalLinguaFranca.g:4681:2: ( RULE_CODE )
            // InternalLinguaFranca.g:4682:3: RULE_CODE
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
    // InternalLinguaFranca.g:4691:1: rule__Preamble__CodeAssignment_1 : ( RULE_CODE ) ;
    public final void rule__Preamble__CodeAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4695:1: ( ( RULE_CODE ) )
            // InternalLinguaFranca.g:4696:2: ( RULE_CODE )
            {
            // InternalLinguaFranca.g:4696:2: ( RULE_CODE )
            // InternalLinguaFranca.g:4697:3: RULE_CODE
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


    // $ANTLR start "rule__Initialize__CodeAssignment_1"
    // InternalLinguaFranca.g:4706:1: rule__Initialize__CodeAssignment_1 : ( RULE_CODE ) ;
    public final void rule__Initialize__CodeAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4710:1: ( ( RULE_CODE ) )
            // InternalLinguaFranca.g:4711:2: ( RULE_CODE )
            {
            // InternalLinguaFranca.g:4711:2: ( RULE_CODE )
            // InternalLinguaFranca.g:4712:3: RULE_CODE
            {
             before(grammarAccess.getInitializeAccess().getCodeCODETerminalRuleCall_1_0()); 
            match(input,RULE_CODE,FOLLOW_2); 
             after(grammarAccess.getInitializeAccess().getCodeCODETerminalRuleCall_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Initialize__CodeAssignment_1"


    // $ANTLR start "rule__Instance__NameAssignment_1"
    // InternalLinguaFranca.g:4721:1: rule__Instance__NameAssignment_1 : ( RULE_ID ) ;
    public final void rule__Instance__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4725:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4726:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4726:2: ( RULE_ID )
            // InternalLinguaFranca.g:4727:3: RULE_ID
            {
             before(grammarAccess.getInstanceAccess().getNameIDTerminalRuleCall_1_0()); 
            match(input,RULE_ID,FOLLOW_2); 
             after(grammarAccess.getInstanceAccess().getNameIDTerminalRuleCall_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Instance__NameAssignment_1"


    // $ANTLR start "rule__Instance__ActorClassAssignment_3"
    // InternalLinguaFranca.g:4736:1: rule__Instance__ActorClassAssignment_3 : ( RULE_ID ) ;
    public final void rule__Instance__ActorClassAssignment_3() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4740:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4741:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4741:2: ( RULE_ID )
            // InternalLinguaFranca.g:4742:3: RULE_ID
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
    // InternalLinguaFranca.g:4751:1: rule__Instance__ParametersAssignment_4_1 : ( ruleAssignments ) ;
    public final void rule__Instance__ParametersAssignment_4_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4755:1: ( ( ruleAssignments ) )
            // InternalLinguaFranca.g:4756:2: ( ruleAssignments )
            {
            // InternalLinguaFranca.g:4756:2: ( ruleAssignments )
            // InternalLinguaFranca.g:4757:3: ruleAssignments
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
    // InternalLinguaFranca.g:4766:1: rule__Connection__LeftPortAssignment_0 : ( rulePort ) ;
    public final void rule__Connection__LeftPortAssignment_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4770:1: ( ( rulePort ) )
            // InternalLinguaFranca.g:4771:2: ( rulePort )
            {
            // InternalLinguaFranca.g:4771:2: ( rulePort )
            // InternalLinguaFranca.g:4772:3: rulePort
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
    // InternalLinguaFranca.g:4781:1: rule__Connection__RightPortAssignment_2 : ( rulePort ) ;
    public final void rule__Connection__RightPortAssignment_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4785:1: ( ( rulePort ) )
            // InternalLinguaFranca.g:4786:2: ( rulePort )
            {
            // InternalLinguaFranca.g:4786:2: ( rulePort )
            // InternalLinguaFranca.g:4787:3: rulePort
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
    // InternalLinguaFranca.g:4796:1: rule__Assignments__AssignmentsAssignment_0 : ( ruleAssignment ) ;
    public final void rule__Assignments__AssignmentsAssignment_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4800:1: ( ( ruleAssignment ) )
            // InternalLinguaFranca.g:4801:2: ( ruleAssignment )
            {
            // InternalLinguaFranca.g:4801:2: ( ruleAssignment )
            // InternalLinguaFranca.g:4802:3: ruleAssignment
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
    // InternalLinguaFranca.g:4811:1: rule__Assignments__AssignmentsAssignment_1_1 : ( ruleAssignment ) ;
    public final void rule__Assignments__AssignmentsAssignment_1_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4815:1: ( ( ruleAssignment ) )
            // InternalLinguaFranca.g:4816:2: ( ruleAssignment )
            {
            // InternalLinguaFranca.g:4816:2: ( ruleAssignment )
            // InternalLinguaFranca.g:4817:3: ruleAssignment
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
    // InternalLinguaFranca.g:4826:1: rule__Assignment__NameAssignment_0 : ( RULE_ID ) ;
    public final void rule__Assignment__NameAssignment_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4830:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4831:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4831:2: ( RULE_ID )
            // InternalLinguaFranca.g:4832:3: RULE_ID
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
    // InternalLinguaFranca.g:4841:1: rule__Assignment__ValueAssignment_2 : ( ruleValue ) ;
    public final void rule__Assignment__ValueAssignment_2() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4845:1: ( ( ruleValue ) )
            // InternalLinguaFranca.g:4846:2: ( ruleValue )
            {
            // InternalLinguaFranca.g:4846:2: ( ruleValue )
            // InternalLinguaFranca.g:4847:3: ruleValue
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
    // InternalLinguaFranca.g:4856:1: rule__Gets__GetsAssignment_0 : ( RULE_ID ) ;
    public final void rule__Gets__GetsAssignment_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4860:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4861:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4861:2: ( RULE_ID )
            // InternalLinguaFranca.g:4862:3: RULE_ID
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
    // InternalLinguaFranca.g:4871:1: rule__Gets__GetsAssignment_1_1 : ( RULE_ID ) ;
    public final void rule__Gets__GetsAssignment_1_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4875:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4876:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4876:2: ( RULE_ID )
            // InternalLinguaFranca.g:4877:3: RULE_ID
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
    // InternalLinguaFranca.g:4886:1: rule__Params__ParamsAssignment_1 : ( ruleParam ) ;
    public final void rule__Params__ParamsAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4890:1: ( ( ruleParam ) )
            // InternalLinguaFranca.g:4891:2: ( ruleParam )
            {
            // InternalLinguaFranca.g:4891:2: ( ruleParam )
            // InternalLinguaFranca.g:4892:3: ruleParam
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
    // InternalLinguaFranca.g:4901:1: rule__Params__ParamsAssignment_2_1 : ( ruleParam ) ;
    public final void rule__Params__ParamsAssignment_2_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4905:1: ( ( ruleParam ) )
            // InternalLinguaFranca.g:4906:2: ( ruleParam )
            {
            // InternalLinguaFranca.g:4906:2: ( ruleParam )
            // InternalLinguaFranca.g:4907:3: ruleParam
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
    // InternalLinguaFranca.g:4916:1: rule__Param__NameAssignment_1 : ( RULE_ID ) ;
    public final void rule__Param__NameAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4920:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:4921:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:4921:2: ( RULE_ID )
            // InternalLinguaFranca.g:4922:3: RULE_ID
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
    // InternalLinguaFranca.g:4931:1: rule__Param__TypeAssignment_2_1 : ( ruleType ) ;
    public final void rule__Param__TypeAssignment_2_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4935:1: ( ( ruleType ) )
            // InternalLinguaFranca.g:4936:2: ( ruleType )
            {
            // InternalLinguaFranca.g:4936:2: ( ruleType )
            // InternalLinguaFranca.g:4937:3: ruleType
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
    // InternalLinguaFranca.g:4946:1: rule__Param__ValueAssignment_3_1 : ( ruleValue ) ;
    public final void rule__Param__ValueAssignment_3_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4950:1: ( ( ruleValue ) )
            // InternalLinguaFranca.g:4951:2: ( ruleValue )
            {
            // InternalLinguaFranca.g:4951:2: ( ruleValue )
            // InternalLinguaFranca.g:4952:3: ruleValue
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
    // InternalLinguaFranca.g:4961:1: rule__Period__PeriodAssignment_1 : ( ( rule__Period__PeriodAlternatives_1_0 ) ) ;
    public final void rule__Period__PeriodAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4965:1: ( ( ( rule__Period__PeriodAlternatives_1_0 ) ) )
            // InternalLinguaFranca.g:4966:2: ( ( rule__Period__PeriodAlternatives_1_0 ) )
            {
            // InternalLinguaFranca.g:4966:2: ( ( rule__Period__PeriodAlternatives_1_0 ) )
            // InternalLinguaFranca.g:4967:3: ( rule__Period__PeriodAlternatives_1_0 )
            {
             before(grammarAccess.getPeriodAccess().getPeriodAlternatives_1_0()); 
            // InternalLinguaFranca.g:4968:3: ( rule__Period__PeriodAlternatives_1_0 )
            // InternalLinguaFranca.g:4968:4: rule__Period__PeriodAlternatives_1_0
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


    // $ANTLR start "rule__Period__PeriodicAssignment_2_1_0"
    // InternalLinguaFranca.g:4976:1: rule__Period__PeriodicAssignment_2_1_0 : ( ( 'PERIODIC' ) ) ;
    public final void rule__Period__PeriodicAssignment_2_1_0() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4980:1: ( ( ( 'PERIODIC' ) ) )
            // InternalLinguaFranca.g:4981:2: ( ( 'PERIODIC' ) )
            {
            // InternalLinguaFranca.g:4981:2: ( ( 'PERIODIC' ) )
            // InternalLinguaFranca.g:4982:3: ( 'PERIODIC' )
            {
             before(grammarAccess.getPeriodAccess().getPeriodicPERIODICKeyword_2_1_0_0()); 
            // InternalLinguaFranca.g:4983:3: ( 'PERIODIC' )
            // InternalLinguaFranca.g:4984:4: 'PERIODIC'
            {
             before(grammarAccess.getPeriodAccess().getPeriodicPERIODICKeyword_2_1_0_0()); 
            match(input,35,FOLLOW_2); 
             after(grammarAccess.getPeriodAccess().getPeriodicPERIODICKeyword_2_1_0_0()); 

            }

             after(grammarAccess.getPeriodAccess().getPeriodicPERIODICKeyword_2_1_0_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__PeriodicAssignment_2_1_0"


    // $ANTLR start "rule__Period__OnceAssignment_2_1_1"
    // InternalLinguaFranca.g:4995:1: rule__Period__OnceAssignment_2_1_1 : ( ( 'ONCE' ) ) ;
    public final void rule__Period__OnceAssignment_2_1_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:4999:1: ( ( ( 'ONCE' ) ) )
            // InternalLinguaFranca.g:5000:2: ( ( 'ONCE' ) )
            {
            // InternalLinguaFranca.g:5000:2: ( ( 'ONCE' ) )
            // InternalLinguaFranca.g:5001:3: ( 'ONCE' )
            {
             before(grammarAccess.getPeriodAccess().getOnceONCEKeyword_2_1_1_0()); 
            // InternalLinguaFranca.g:5002:3: ( 'ONCE' )
            // InternalLinguaFranca.g:5003:4: 'ONCE'
            {
             before(grammarAccess.getPeriodAccess().getOnceONCEKeyword_2_1_1_0()); 
            match(input,36,FOLLOW_2); 
             after(grammarAccess.getPeriodAccess().getOnceONCEKeyword_2_1_1_0()); 

            }

             after(grammarAccess.getPeriodAccess().getOnceONCEKeyword_2_1_1_0()); 

            }


            }

        }
        catch (RecognitionException re) {
            reportError(re);
            recover(input,re);
        }
        finally {

            	restoreStackSize(stackSize);

        }
        return ;
    }
    // $ANTLR end "rule__Period__OnceAssignment_2_1_1"


    // $ANTLR start "rule__Sets__SetsAssignment_1"
    // InternalLinguaFranca.g:5014:1: rule__Sets__SetsAssignment_1 : ( RULE_ID ) ;
    public final void rule__Sets__SetsAssignment_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:5018:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:5019:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:5019:2: ( RULE_ID )
            // InternalLinguaFranca.g:5020:3: RULE_ID
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
    // InternalLinguaFranca.g:5029:1: rule__Sets__SetsAssignment_2_1 : ( RULE_ID ) ;
    public final void rule__Sets__SetsAssignment_2_1() throws RecognitionException {

        		int stackSize = keepStackSize();
        	
        try {
            // InternalLinguaFranca.g:5033:1: ( ( RULE_ID ) )
            // InternalLinguaFranca.g:5034:2: ( RULE_ID )
            {
            // InternalLinguaFranca.g:5034:2: ( RULE_ID )
            // InternalLinguaFranca.g:5035:3: RULE_ID
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
    public static final BitSet FOLLOW_14 = new BitSet(new long[]{0x000000007120E010L});
    public static final BitSet FOLLOW_15 = new BitSet(new long[]{0x0000000040000002L});
    public static final BitSet FOLLOW_16 = new BitSet(new long[]{0x0000000000000012L});
    public static final BitSet FOLLOW_17 = new BitSet(new long[]{0x0000000000002010L});
    public static final BitSet FOLLOW_18 = new BitSet(new long[]{0x0000000000820000L});
    public static final BitSet FOLLOW_19 = new BitSet(new long[]{0x0000000000000050L});
    public static final BitSet FOLLOW_20 = new BitSet(new long[]{0x0000000000004010L});
    public static final BitSet FOLLOW_21 = new BitSet(new long[]{0x0000000000008010L});
    public static final BitSet FOLLOW_22 = new BitSet(new long[]{0x0000000002020000L});
    public static final BitSet FOLLOW_23 = new BitSet(new long[]{0x0000000102000050L});
    public static final BitSet FOLLOW_24 = new BitSet(new long[]{0x0000000004000010L});
    public static final BitSet FOLLOW_25 = new BitSet(new long[]{0x0000000008000000L});
    public static final BitSet FOLLOW_26 = new BitSet(new long[]{0x0000000008000002L});
    public static final BitSet FOLLOW_27 = new BitSet(new long[]{0x0000000000000040L});
    public static final BitSet FOLLOW_28 = new BitSet(new long[]{0x0000000080000000L});
    public static final BitSet FOLLOW_29 = new BitSet(new long[]{0x0000000100000000L});
    public static final BitSet FOLLOW_30 = new BitSet(new long[]{0x00000000000000F0L});
    public static final BitSet FOLLOW_31 = new BitSet(new long[]{0x0000000200000010L});
    public static final BitSet FOLLOW_32 = new BitSet(new long[]{0x000000000C000000L});
    public static final BitSet FOLLOW_33 = new BitSet(new long[]{0x0000000002800000L});
    public static final BitSet FOLLOW_34 = new BitSet(new long[]{0x0000000004000000L});
    public static final BitSet FOLLOW_35 = new BitSet(new long[]{0x0000000000000030L});
    public static final BitSet FOLLOW_36 = new BitSet(new long[]{0x0000001800000000L});
    public static final BitSet FOLLOW_37 = new BitSet(new long[]{0x0000000400000000L});
    public static final BitSet FOLLOW_38 = new BitSet(new long[]{0x0000000000006010L});
    public static final BitSet FOLLOW_39 = new BitSet(new long[]{0x0000000400000002L});

}