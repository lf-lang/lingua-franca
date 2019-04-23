package org.icyphy.parser.antlr.internal;

import org.eclipse.xtext.*;
import org.eclipse.xtext.parser.*;
import org.eclipse.xtext.parser.impl.*;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.parser.antlr.AbstractInternalAntlrParser;
import org.eclipse.xtext.parser.antlr.XtextTokenStream;
import org.eclipse.xtext.parser.antlr.XtextTokenStream.HiddenTokens;
import org.eclipse.xtext.parser.antlr.AntlrDatatypeRuleToken;
import org.icyphy.services.LinguaFrancaGrammarAccess;



import org.antlr.runtime.*;
import java.util.Stack;
import java.util.List;
import java.util.ArrayList;

@SuppressWarnings("all")
public class InternalLinguaFrancaParser extends AbstractInternalAntlrParser {
    public static final String[] tokenNames = new String[] {
        "<invalid>", "<EOR>", "<DOWN>", "<UP>", "RULE_ID", "RULE_CODE", "RULE_NUMBER", "RULE_STRING", "RULE_INT", "RULE_ML_COMMENT", "RULE_SL_COMMENT", "RULE_WS", "RULE_ANY_OTHER", "'target'", "';'", "'import'", "'reactor'", "'}'", "'composite'", "'{'", "'input'", "':'", "'output'", "'timer'", "'action'", "'reaction'", "'('", "','", "')'", "'preamble'", "'='", "'new'", "'->'", "'const'", "'NOW'", "'ONCE'", "'STOP'", "'.'"
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
    public static final int RULE_NUMBER=6;
    public static final int RULE_CODE=5;
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

        public InternalLinguaFrancaParser(TokenStream input, LinguaFrancaGrammarAccess grammarAccess) {
            this(input);
            this.grammarAccess = grammarAccess;
            registerRules(grammarAccess.getGrammar());
        }

        @Override
        protected String getFirstRuleName() {
        	return "Model";
       	}

       	@Override
       	protected LinguaFrancaGrammarAccess getGrammarAccess() {
       		return grammarAccess;
       	}




    // $ANTLR start "entryRuleModel"
    // InternalLinguaFranca.g:64:1: entryRuleModel returns [EObject current=null] : iv_ruleModel= ruleModel EOF ;
    public final EObject entryRuleModel() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleModel = null;


        try {
            // InternalLinguaFranca.g:64:46: (iv_ruleModel= ruleModel EOF )
            // InternalLinguaFranca.g:65:2: iv_ruleModel= ruleModel EOF
            {
             newCompositeNode(grammarAccess.getModelRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleModel=ruleModel();

            state._fsp--;

             current =iv_ruleModel; 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleModel"


    // $ANTLR start "ruleModel"
    // InternalLinguaFranca.g:71:1: ruleModel returns [EObject current=null] : ( ( (lv_target_0_0= ruleTarget ) ) ( (lv_imports_1_0= ruleImport ) )* ( (lv_components_2_0= ruleComponent ) )+ ) ;
    public final EObject ruleModel() throws RecognitionException {
        EObject current = null;

        EObject lv_target_0_0 = null;

        EObject lv_imports_1_0 = null;

        EObject lv_components_2_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:77:2: ( ( ( (lv_target_0_0= ruleTarget ) ) ( (lv_imports_1_0= ruleImport ) )* ( (lv_components_2_0= ruleComponent ) )+ ) )
            // InternalLinguaFranca.g:78:2: ( ( (lv_target_0_0= ruleTarget ) ) ( (lv_imports_1_0= ruleImport ) )* ( (lv_components_2_0= ruleComponent ) )+ )
            {
            // InternalLinguaFranca.g:78:2: ( ( (lv_target_0_0= ruleTarget ) ) ( (lv_imports_1_0= ruleImport ) )* ( (lv_components_2_0= ruleComponent ) )+ )
            // InternalLinguaFranca.g:79:3: ( (lv_target_0_0= ruleTarget ) ) ( (lv_imports_1_0= ruleImport ) )* ( (lv_components_2_0= ruleComponent ) )+
            {
            // InternalLinguaFranca.g:79:3: ( (lv_target_0_0= ruleTarget ) )
            // InternalLinguaFranca.g:80:4: (lv_target_0_0= ruleTarget )
            {
            // InternalLinguaFranca.g:80:4: (lv_target_0_0= ruleTarget )
            // InternalLinguaFranca.g:81:5: lv_target_0_0= ruleTarget
            {

            					newCompositeNode(grammarAccess.getModelAccess().getTargetTargetParserRuleCall_0_0());
            				
            pushFollow(FOLLOW_3);
            lv_target_0_0=ruleTarget();

            state._fsp--;


            					if (current==null) {
            						current = createModelElementForParent(grammarAccess.getModelRule());
            					}
            					set(
            						current,
            						"target",
            						lv_target_0_0,
            						"org.icyphy.LinguaFranca.Target");
            					afterParserOrEnumRuleCall();
            				

            }


            }

            // InternalLinguaFranca.g:98:3: ( (lv_imports_1_0= ruleImport ) )*
            loop1:
            do {
                int alt1=2;
                int LA1_0 = input.LA(1);

                if ( (LA1_0==15) ) {
                    alt1=1;
                }


                switch (alt1) {
            	case 1 :
            	    // InternalLinguaFranca.g:99:4: (lv_imports_1_0= ruleImport )
            	    {
            	    // InternalLinguaFranca.g:99:4: (lv_imports_1_0= ruleImport )
            	    // InternalLinguaFranca.g:100:5: lv_imports_1_0= ruleImport
            	    {

            	    					newCompositeNode(grammarAccess.getModelAccess().getImportsImportParserRuleCall_1_0());
            	    				
            	    pushFollow(FOLLOW_3);
            	    lv_imports_1_0=ruleImport();

            	    state._fsp--;


            	    					if (current==null) {
            	    						current = createModelElementForParent(grammarAccess.getModelRule());
            	    					}
            	    					add(
            	    						current,
            	    						"imports",
            	    						lv_imports_1_0,
            	    						"org.icyphy.LinguaFranca.Import");
            	    					afterParserOrEnumRuleCall();
            	    				

            	    }


            	    }
            	    break;

            	default :
            	    break loop1;
                }
            } while (true);

            // InternalLinguaFranca.g:117:3: ( (lv_components_2_0= ruleComponent ) )+
            int cnt2=0;
            loop2:
            do {
                int alt2=2;
                int LA2_0 = input.LA(1);

                if ( (LA2_0==16||LA2_0==18) ) {
                    alt2=1;
                }


                switch (alt2) {
            	case 1 :
            	    // InternalLinguaFranca.g:118:4: (lv_components_2_0= ruleComponent )
            	    {
            	    // InternalLinguaFranca.g:118:4: (lv_components_2_0= ruleComponent )
            	    // InternalLinguaFranca.g:119:5: lv_components_2_0= ruleComponent
            	    {

            	    					newCompositeNode(grammarAccess.getModelAccess().getComponentsComponentParserRuleCall_2_0());
            	    				
            	    pushFollow(FOLLOW_4);
            	    lv_components_2_0=ruleComponent();

            	    state._fsp--;


            	    					if (current==null) {
            	    						current = createModelElementForParent(grammarAccess.getModelRule());
            	    					}
            	    					add(
            	    						current,
            	    						"components",
            	    						lv_components_2_0,
            	    						"org.icyphy.LinguaFranca.Component");
            	    					afterParserOrEnumRuleCall();
            	    				

            	    }


            	    }
            	    break;

            	default :
            	    if ( cnt2 >= 1 ) break loop2;
                        EarlyExitException eee =
                            new EarlyExitException(2, input);
                        throw eee;
                }
                cnt2++;
            } while (true);


            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleModel"


    // $ANTLR start "entryRuleTarget"
    // InternalLinguaFranca.g:140:1: entryRuleTarget returns [EObject current=null] : iv_ruleTarget= ruleTarget EOF ;
    public final EObject entryRuleTarget() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleTarget = null;


        try {
            // InternalLinguaFranca.g:140:47: (iv_ruleTarget= ruleTarget EOF )
            // InternalLinguaFranca.g:141:2: iv_ruleTarget= ruleTarget EOF
            {
             newCompositeNode(grammarAccess.getTargetRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleTarget=ruleTarget();

            state._fsp--;

             current =iv_ruleTarget; 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleTarget"


    // $ANTLR start "ruleTarget"
    // InternalLinguaFranca.g:147:1: ruleTarget returns [EObject current=null] : (otherlv_0= 'target' ( (lv_name_1_0= RULE_ID ) ) otherlv_2= ';' ) ;
    public final EObject ruleTarget() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token lv_name_1_0=null;
        Token otherlv_2=null;


        	enterRule();

        try {
            // InternalLinguaFranca.g:153:2: ( (otherlv_0= 'target' ( (lv_name_1_0= RULE_ID ) ) otherlv_2= ';' ) )
            // InternalLinguaFranca.g:154:2: (otherlv_0= 'target' ( (lv_name_1_0= RULE_ID ) ) otherlv_2= ';' )
            {
            // InternalLinguaFranca.g:154:2: (otherlv_0= 'target' ( (lv_name_1_0= RULE_ID ) ) otherlv_2= ';' )
            // InternalLinguaFranca.g:155:3: otherlv_0= 'target' ( (lv_name_1_0= RULE_ID ) ) otherlv_2= ';'
            {
            otherlv_0=(Token)match(input,13,FOLLOW_5); 

            			newLeafNode(otherlv_0, grammarAccess.getTargetAccess().getTargetKeyword_0());
            		
            // InternalLinguaFranca.g:159:3: ( (lv_name_1_0= RULE_ID ) )
            // InternalLinguaFranca.g:160:4: (lv_name_1_0= RULE_ID )
            {
            // InternalLinguaFranca.g:160:4: (lv_name_1_0= RULE_ID )
            // InternalLinguaFranca.g:161:5: lv_name_1_0= RULE_ID
            {
            lv_name_1_0=(Token)match(input,RULE_ID,FOLLOW_6); 

            					newLeafNode(lv_name_1_0, grammarAccess.getTargetAccess().getNameIDTerminalRuleCall_1_0());
            				

            					if (current==null) {
            						current = createModelElement(grammarAccess.getTargetRule());
            					}
            					setWithLastConsumed(
            						current,
            						"name",
            						lv_name_1_0,
            						"org.eclipse.xtext.common.Terminals.ID");
            				

            }


            }

            otherlv_2=(Token)match(input,14,FOLLOW_2); 

            			newLeafNode(otherlv_2, grammarAccess.getTargetAccess().getSemicolonKeyword_2());
            		

            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleTarget"


    // $ANTLR start "entryRuleImport"
    // InternalLinguaFranca.g:185:1: entryRuleImport returns [EObject current=null] : iv_ruleImport= ruleImport EOF ;
    public final EObject entryRuleImport() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleImport = null;


        try {
            // InternalLinguaFranca.g:185:47: (iv_ruleImport= ruleImport EOF )
            // InternalLinguaFranca.g:186:2: iv_ruleImport= ruleImport EOF
            {
             newCompositeNode(grammarAccess.getImportRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleImport=ruleImport();

            state._fsp--;

             current =iv_ruleImport; 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleImport"


    // $ANTLR start "ruleImport"
    // InternalLinguaFranca.g:192:1: ruleImport returns [EObject current=null] : (otherlv_0= 'import' ( (lv_name_1_0= rulePath ) ) otherlv_2= ';' ) ;
    public final EObject ruleImport() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token otherlv_2=null;
        AntlrDatatypeRuleToken lv_name_1_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:198:2: ( (otherlv_0= 'import' ( (lv_name_1_0= rulePath ) ) otherlv_2= ';' ) )
            // InternalLinguaFranca.g:199:2: (otherlv_0= 'import' ( (lv_name_1_0= rulePath ) ) otherlv_2= ';' )
            {
            // InternalLinguaFranca.g:199:2: (otherlv_0= 'import' ( (lv_name_1_0= rulePath ) ) otherlv_2= ';' )
            // InternalLinguaFranca.g:200:3: otherlv_0= 'import' ( (lv_name_1_0= rulePath ) ) otherlv_2= ';'
            {
            otherlv_0=(Token)match(input,15,FOLLOW_5); 

            			newLeafNode(otherlv_0, grammarAccess.getImportAccess().getImportKeyword_0());
            		
            // InternalLinguaFranca.g:204:3: ( (lv_name_1_0= rulePath ) )
            // InternalLinguaFranca.g:205:4: (lv_name_1_0= rulePath )
            {
            // InternalLinguaFranca.g:205:4: (lv_name_1_0= rulePath )
            // InternalLinguaFranca.g:206:5: lv_name_1_0= rulePath
            {

            					newCompositeNode(grammarAccess.getImportAccess().getNamePathParserRuleCall_1_0());
            				
            pushFollow(FOLLOW_6);
            lv_name_1_0=rulePath();

            state._fsp--;


            					if (current==null) {
            						current = createModelElementForParent(grammarAccess.getImportRule());
            					}
            					set(
            						current,
            						"name",
            						lv_name_1_0,
            						"org.icyphy.LinguaFranca.Path");
            					afterParserOrEnumRuleCall();
            				

            }


            }

            otherlv_2=(Token)match(input,14,FOLLOW_2); 

            			newLeafNode(otherlv_2, grammarAccess.getImportAccess().getSemicolonKeyword_2());
            		

            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleImport"


    // $ANTLR start "entryRuleComponent"
    // InternalLinguaFranca.g:231:1: entryRuleComponent returns [EObject current=null] : iv_ruleComponent= ruleComponent EOF ;
    public final EObject entryRuleComponent() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleComponent = null;


        try {
            // InternalLinguaFranca.g:231:50: (iv_ruleComponent= ruleComponent EOF )
            // InternalLinguaFranca.g:232:2: iv_ruleComponent= ruleComponent EOF
            {
             newCompositeNode(grammarAccess.getComponentRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleComponent=ruleComponent();

            state._fsp--;

             current =iv_ruleComponent; 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleComponent"


    // $ANTLR start "ruleComponent"
    // InternalLinguaFranca.g:238:1: ruleComponent returns [EObject current=null] : (this_Reactor_0= ruleReactor | this_Composite_1= ruleComposite ) ;
    public final EObject ruleComponent() throws RecognitionException {
        EObject current = null;

        EObject this_Reactor_0 = null;

        EObject this_Composite_1 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:244:2: ( (this_Reactor_0= ruleReactor | this_Composite_1= ruleComposite ) )
            // InternalLinguaFranca.g:245:2: (this_Reactor_0= ruleReactor | this_Composite_1= ruleComposite )
            {
            // InternalLinguaFranca.g:245:2: (this_Reactor_0= ruleReactor | this_Composite_1= ruleComposite )
            int alt3=2;
            int LA3_0 = input.LA(1);

            if ( (LA3_0==16) ) {
                alt3=1;
            }
            else if ( (LA3_0==18) ) {
                alt3=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 3, 0, input);

                throw nvae;
            }
            switch (alt3) {
                case 1 :
                    // InternalLinguaFranca.g:246:3: this_Reactor_0= ruleReactor
                    {

                    			newCompositeNode(grammarAccess.getComponentAccess().getReactorParserRuleCall_0());
                    		
                    pushFollow(FOLLOW_2);
                    this_Reactor_0=ruleReactor();

                    state._fsp--;


                    			current = this_Reactor_0;
                    			afterParserOrEnumRuleCall();
                    		

                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:255:3: this_Composite_1= ruleComposite
                    {

                    			newCompositeNode(grammarAccess.getComponentAccess().getCompositeParserRuleCall_1());
                    		
                    pushFollow(FOLLOW_2);
                    this_Composite_1=ruleComposite();

                    state._fsp--;


                    			current = this_Composite_1;
                    			afterParserOrEnumRuleCall();
                    		

                    }
                    break;

            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleComponent"


    // $ANTLR start "entryRuleReactor"
    // InternalLinguaFranca.g:267:1: entryRuleReactor returns [EObject current=null] : iv_ruleReactor= ruleReactor EOF ;
    public final EObject entryRuleReactor() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleReactor = null;


        try {
            // InternalLinguaFranca.g:267:48: (iv_ruleReactor= ruleReactor EOF )
            // InternalLinguaFranca.g:268:2: iv_ruleReactor= ruleReactor EOF
            {
             newCompositeNode(grammarAccess.getReactorRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleReactor=ruleReactor();

            state._fsp--;

             current =iv_ruleReactor; 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleReactor"


    // $ANTLR start "ruleReactor"
    // InternalLinguaFranca.g:274:1: ruleReactor returns [EObject current=null] : (otherlv_0= 'reactor' ( (lv_componentBody_1_0= ruleComponentBody ) ) otherlv_2= '}' ) ;
    public final EObject ruleReactor() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token otherlv_2=null;
        EObject lv_componentBody_1_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:280:2: ( (otherlv_0= 'reactor' ( (lv_componentBody_1_0= ruleComponentBody ) ) otherlv_2= '}' ) )
            // InternalLinguaFranca.g:281:2: (otherlv_0= 'reactor' ( (lv_componentBody_1_0= ruleComponentBody ) ) otherlv_2= '}' )
            {
            // InternalLinguaFranca.g:281:2: (otherlv_0= 'reactor' ( (lv_componentBody_1_0= ruleComponentBody ) ) otherlv_2= '}' )
            // InternalLinguaFranca.g:282:3: otherlv_0= 'reactor' ( (lv_componentBody_1_0= ruleComponentBody ) ) otherlv_2= '}'
            {
            otherlv_0=(Token)match(input,16,FOLLOW_5); 

            			newLeafNode(otherlv_0, grammarAccess.getReactorAccess().getReactorKeyword_0());
            		
            // InternalLinguaFranca.g:286:3: ( (lv_componentBody_1_0= ruleComponentBody ) )
            // InternalLinguaFranca.g:287:4: (lv_componentBody_1_0= ruleComponentBody )
            {
            // InternalLinguaFranca.g:287:4: (lv_componentBody_1_0= ruleComponentBody )
            // InternalLinguaFranca.g:288:5: lv_componentBody_1_0= ruleComponentBody
            {

            					newCompositeNode(grammarAccess.getReactorAccess().getComponentBodyComponentBodyParserRuleCall_1_0());
            				
            pushFollow(FOLLOW_7);
            lv_componentBody_1_0=ruleComponentBody();

            state._fsp--;


            					if (current==null) {
            						current = createModelElementForParent(grammarAccess.getReactorRule());
            					}
            					set(
            						current,
            						"componentBody",
            						lv_componentBody_1_0,
            						"org.icyphy.LinguaFranca.ComponentBody");
            					afterParserOrEnumRuleCall();
            				

            }


            }

            otherlv_2=(Token)match(input,17,FOLLOW_2); 

            			newLeafNode(otherlv_2, grammarAccess.getReactorAccess().getRightCurlyBracketKeyword_2());
            		

            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleReactor"


    // $ANTLR start "entryRuleComposite"
    // InternalLinguaFranca.g:313:1: entryRuleComposite returns [EObject current=null] : iv_ruleComposite= ruleComposite EOF ;
    public final EObject entryRuleComposite() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleComposite = null;


        try {
            // InternalLinguaFranca.g:313:50: (iv_ruleComposite= ruleComposite EOF )
            // InternalLinguaFranca.g:314:2: iv_ruleComposite= ruleComposite EOF
            {
             newCompositeNode(grammarAccess.getCompositeRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleComposite=ruleComposite();

            state._fsp--;

             current =iv_ruleComposite; 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleComposite"


    // $ANTLR start "ruleComposite"
    // InternalLinguaFranca.g:320:1: ruleComposite returns [EObject current=null] : (otherlv_0= 'composite' ( (lv_componentBody_1_0= ruleComponentBody ) ) ( (lv_instances_2_0= ruleInstance ) )* ( (lv_connections_3_0= ruleConnection ) )* otherlv_4= '}' ) ;
    public final EObject ruleComposite() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token otherlv_4=null;
        EObject lv_componentBody_1_0 = null;

        EObject lv_instances_2_0 = null;

        EObject lv_connections_3_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:326:2: ( (otherlv_0= 'composite' ( (lv_componentBody_1_0= ruleComponentBody ) ) ( (lv_instances_2_0= ruleInstance ) )* ( (lv_connections_3_0= ruleConnection ) )* otherlv_4= '}' ) )
            // InternalLinguaFranca.g:327:2: (otherlv_0= 'composite' ( (lv_componentBody_1_0= ruleComponentBody ) ) ( (lv_instances_2_0= ruleInstance ) )* ( (lv_connections_3_0= ruleConnection ) )* otherlv_4= '}' )
            {
            // InternalLinguaFranca.g:327:2: (otherlv_0= 'composite' ( (lv_componentBody_1_0= ruleComponentBody ) ) ( (lv_instances_2_0= ruleInstance ) )* ( (lv_connections_3_0= ruleConnection ) )* otherlv_4= '}' )
            // InternalLinguaFranca.g:328:3: otherlv_0= 'composite' ( (lv_componentBody_1_0= ruleComponentBody ) ) ( (lv_instances_2_0= ruleInstance ) )* ( (lv_connections_3_0= ruleConnection ) )* otherlv_4= '}'
            {
            otherlv_0=(Token)match(input,18,FOLLOW_5); 

            			newLeafNode(otherlv_0, grammarAccess.getCompositeAccess().getCompositeKeyword_0());
            		
            // InternalLinguaFranca.g:332:3: ( (lv_componentBody_1_0= ruleComponentBody ) )
            // InternalLinguaFranca.g:333:4: (lv_componentBody_1_0= ruleComponentBody )
            {
            // InternalLinguaFranca.g:333:4: (lv_componentBody_1_0= ruleComponentBody )
            // InternalLinguaFranca.g:334:5: lv_componentBody_1_0= ruleComponentBody
            {

            					newCompositeNode(grammarAccess.getCompositeAccess().getComponentBodyComponentBodyParserRuleCall_1_0());
            				
            pushFollow(FOLLOW_8);
            lv_componentBody_1_0=ruleComponentBody();

            state._fsp--;


            					if (current==null) {
            						current = createModelElementForParent(grammarAccess.getCompositeRule());
            					}
            					set(
            						current,
            						"componentBody",
            						lv_componentBody_1_0,
            						"org.icyphy.LinguaFranca.ComponentBody");
            					afterParserOrEnumRuleCall();
            				

            }


            }

            // InternalLinguaFranca.g:351:3: ( (lv_instances_2_0= ruleInstance ) )*
            loop4:
            do {
                int alt4=2;
                int LA4_0 = input.LA(1);

                if ( (LA4_0==RULE_ID) ) {
                    int LA4_1 = input.LA(2);

                    if ( (LA4_1==30) ) {
                        alt4=1;
                    }


                }


                switch (alt4) {
            	case 1 :
            	    // InternalLinguaFranca.g:352:4: (lv_instances_2_0= ruleInstance )
            	    {
            	    // InternalLinguaFranca.g:352:4: (lv_instances_2_0= ruleInstance )
            	    // InternalLinguaFranca.g:353:5: lv_instances_2_0= ruleInstance
            	    {

            	    					newCompositeNode(grammarAccess.getCompositeAccess().getInstancesInstanceParserRuleCall_2_0());
            	    				
            	    pushFollow(FOLLOW_8);
            	    lv_instances_2_0=ruleInstance();

            	    state._fsp--;


            	    					if (current==null) {
            	    						current = createModelElementForParent(grammarAccess.getCompositeRule());
            	    					}
            	    					add(
            	    						current,
            	    						"instances",
            	    						lv_instances_2_0,
            	    						"org.icyphy.LinguaFranca.Instance");
            	    					afterParserOrEnumRuleCall();
            	    				

            	    }


            	    }
            	    break;

            	default :
            	    break loop4;
                }
            } while (true);

            // InternalLinguaFranca.g:370:3: ( (lv_connections_3_0= ruleConnection ) )*
            loop5:
            do {
                int alt5=2;
                int LA5_0 = input.LA(1);

                if ( (LA5_0==RULE_ID) ) {
                    alt5=1;
                }


                switch (alt5) {
            	case 1 :
            	    // InternalLinguaFranca.g:371:4: (lv_connections_3_0= ruleConnection )
            	    {
            	    // InternalLinguaFranca.g:371:4: (lv_connections_3_0= ruleConnection )
            	    // InternalLinguaFranca.g:372:5: lv_connections_3_0= ruleConnection
            	    {

            	    					newCompositeNode(grammarAccess.getCompositeAccess().getConnectionsConnectionParserRuleCall_3_0());
            	    				
            	    pushFollow(FOLLOW_8);
            	    lv_connections_3_0=ruleConnection();

            	    state._fsp--;


            	    					if (current==null) {
            	    						current = createModelElementForParent(grammarAccess.getCompositeRule());
            	    					}
            	    					add(
            	    						current,
            	    						"connections",
            	    						lv_connections_3_0,
            	    						"org.icyphy.LinguaFranca.Connection");
            	    					afterParserOrEnumRuleCall();
            	    				

            	    }


            	    }
            	    break;

            	default :
            	    break loop5;
                }
            } while (true);

            otherlv_4=(Token)match(input,17,FOLLOW_2); 

            			newLeafNode(otherlv_4, grammarAccess.getCompositeAccess().getRightCurlyBracketKeyword_4());
            		

            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleComposite"


    // $ANTLR start "entryRuleComponentBody"
    // InternalLinguaFranca.g:397:1: entryRuleComponentBody returns [EObject current=null] : iv_ruleComponentBody= ruleComponentBody EOF ;
    public final EObject entryRuleComponentBody() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleComponentBody = null;


        try {
            // InternalLinguaFranca.g:397:54: (iv_ruleComponentBody= ruleComponentBody EOF )
            // InternalLinguaFranca.g:398:2: iv_ruleComponentBody= ruleComponentBody EOF
            {
             newCompositeNode(grammarAccess.getComponentBodyRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleComponentBody=ruleComponentBody();

            state._fsp--;

             current =iv_ruleComponentBody; 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleComponentBody"


    // $ANTLR start "ruleComponentBody"
    // InternalLinguaFranca.g:404:1: ruleComponentBody returns [EObject current=null] : ( ( (lv_name_0_0= RULE_ID ) ) ( (lv_parameters_1_0= ruleParams ) )? otherlv_2= '{' ( (lv_inputs_3_0= ruleInput ) )* ( (lv_outputs_4_0= ruleOutput ) )* ( ( (lv_timers_5_0= ruleTimer ) ) | ( (lv_actions_6_0= ruleAction ) ) )* ( (lv_preamble_7_0= rulePreamble ) )? ( (lv_reactions_8_0= ruleReaction ) )* ) ;
    public final EObject ruleComponentBody() throws RecognitionException {
        EObject current = null;

        Token lv_name_0_0=null;
        Token otherlv_2=null;
        EObject lv_parameters_1_0 = null;

        EObject lv_inputs_3_0 = null;

        EObject lv_outputs_4_0 = null;

        EObject lv_timers_5_0 = null;

        EObject lv_actions_6_0 = null;

        EObject lv_preamble_7_0 = null;

        EObject lv_reactions_8_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:410:2: ( ( ( (lv_name_0_0= RULE_ID ) ) ( (lv_parameters_1_0= ruleParams ) )? otherlv_2= '{' ( (lv_inputs_3_0= ruleInput ) )* ( (lv_outputs_4_0= ruleOutput ) )* ( ( (lv_timers_5_0= ruleTimer ) ) | ( (lv_actions_6_0= ruleAction ) ) )* ( (lv_preamble_7_0= rulePreamble ) )? ( (lv_reactions_8_0= ruleReaction ) )* ) )
            // InternalLinguaFranca.g:411:2: ( ( (lv_name_0_0= RULE_ID ) ) ( (lv_parameters_1_0= ruleParams ) )? otherlv_2= '{' ( (lv_inputs_3_0= ruleInput ) )* ( (lv_outputs_4_0= ruleOutput ) )* ( ( (lv_timers_5_0= ruleTimer ) ) | ( (lv_actions_6_0= ruleAction ) ) )* ( (lv_preamble_7_0= rulePreamble ) )? ( (lv_reactions_8_0= ruleReaction ) )* )
            {
            // InternalLinguaFranca.g:411:2: ( ( (lv_name_0_0= RULE_ID ) ) ( (lv_parameters_1_0= ruleParams ) )? otherlv_2= '{' ( (lv_inputs_3_0= ruleInput ) )* ( (lv_outputs_4_0= ruleOutput ) )* ( ( (lv_timers_5_0= ruleTimer ) ) | ( (lv_actions_6_0= ruleAction ) ) )* ( (lv_preamble_7_0= rulePreamble ) )? ( (lv_reactions_8_0= ruleReaction ) )* )
            // InternalLinguaFranca.g:412:3: ( (lv_name_0_0= RULE_ID ) ) ( (lv_parameters_1_0= ruleParams ) )? otherlv_2= '{' ( (lv_inputs_3_0= ruleInput ) )* ( (lv_outputs_4_0= ruleOutput ) )* ( ( (lv_timers_5_0= ruleTimer ) ) | ( (lv_actions_6_0= ruleAction ) ) )* ( (lv_preamble_7_0= rulePreamble ) )? ( (lv_reactions_8_0= ruleReaction ) )*
            {
            // InternalLinguaFranca.g:412:3: ( (lv_name_0_0= RULE_ID ) )
            // InternalLinguaFranca.g:413:4: (lv_name_0_0= RULE_ID )
            {
            // InternalLinguaFranca.g:413:4: (lv_name_0_0= RULE_ID )
            // InternalLinguaFranca.g:414:5: lv_name_0_0= RULE_ID
            {
            lv_name_0_0=(Token)match(input,RULE_ID,FOLLOW_9); 

            					newLeafNode(lv_name_0_0, grammarAccess.getComponentBodyAccess().getNameIDTerminalRuleCall_0_0());
            				

            					if (current==null) {
            						current = createModelElement(grammarAccess.getComponentBodyRule());
            					}
            					setWithLastConsumed(
            						current,
            						"name",
            						lv_name_0_0,
            						"org.eclipse.xtext.common.Terminals.ID");
            				

            }


            }

            // InternalLinguaFranca.g:430:3: ( (lv_parameters_1_0= ruleParams ) )?
            int alt6=2;
            int LA6_0 = input.LA(1);

            if ( (LA6_0==26) ) {
                alt6=1;
            }
            switch (alt6) {
                case 1 :
                    // InternalLinguaFranca.g:431:4: (lv_parameters_1_0= ruleParams )
                    {
                    // InternalLinguaFranca.g:431:4: (lv_parameters_1_0= ruleParams )
                    // InternalLinguaFranca.g:432:5: lv_parameters_1_0= ruleParams
                    {

                    					newCompositeNode(grammarAccess.getComponentBodyAccess().getParametersParamsParserRuleCall_1_0());
                    				
                    pushFollow(FOLLOW_10);
                    lv_parameters_1_0=ruleParams();

                    state._fsp--;


                    					if (current==null) {
                    						current = createModelElementForParent(grammarAccess.getComponentBodyRule());
                    					}
                    					set(
                    						current,
                    						"parameters",
                    						lv_parameters_1_0,
                    						"org.icyphy.LinguaFranca.Params");
                    					afterParserOrEnumRuleCall();
                    				

                    }


                    }
                    break;

            }

            otherlv_2=(Token)match(input,19,FOLLOW_11); 

            			newLeafNode(otherlv_2, grammarAccess.getComponentBodyAccess().getLeftCurlyBracketKeyword_2());
            		
            // InternalLinguaFranca.g:453:3: ( (lv_inputs_3_0= ruleInput ) )*
            loop7:
            do {
                int alt7=2;
                int LA7_0 = input.LA(1);

                if ( (LA7_0==20) ) {
                    alt7=1;
                }


                switch (alt7) {
            	case 1 :
            	    // InternalLinguaFranca.g:454:4: (lv_inputs_3_0= ruleInput )
            	    {
            	    // InternalLinguaFranca.g:454:4: (lv_inputs_3_0= ruleInput )
            	    // InternalLinguaFranca.g:455:5: lv_inputs_3_0= ruleInput
            	    {

            	    					newCompositeNode(grammarAccess.getComponentBodyAccess().getInputsInputParserRuleCall_3_0());
            	    				
            	    pushFollow(FOLLOW_11);
            	    lv_inputs_3_0=ruleInput();

            	    state._fsp--;


            	    					if (current==null) {
            	    						current = createModelElementForParent(grammarAccess.getComponentBodyRule());
            	    					}
            	    					add(
            	    						current,
            	    						"inputs",
            	    						lv_inputs_3_0,
            	    						"org.icyphy.LinguaFranca.Input");
            	    					afterParserOrEnumRuleCall();
            	    				

            	    }


            	    }
            	    break;

            	default :
            	    break loop7;
                }
            } while (true);

            // InternalLinguaFranca.g:472:3: ( (lv_outputs_4_0= ruleOutput ) )*
            loop8:
            do {
                int alt8=2;
                int LA8_0 = input.LA(1);

                if ( (LA8_0==22) ) {
                    alt8=1;
                }


                switch (alt8) {
            	case 1 :
            	    // InternalLinguaFranca.g:473:4: (lv_outputs_4_0= ruleOutput )
            	    {
            	    // InternalLinguaFranca.g:473:4: (lv_outputs_4_0= ruleOutput )
            	    // InternalLinguaFranca.g:474:5: lv_outputs_4_0= ruleOutput
            	    {

            	    					newCompositeNode(grammarAccess.getComponentBodyAccess().getOutputsOutputParserRuleCall_4_0());
            	    				
            	    pushFollow(FOLLOW_12);
            	    lv_outputs_4_0=ruleOutput();

            	    state._fsp--;


            	    					if (current==null) {
            	    						current = createModelElementForParent(grammarAccess.getComponentBodyRule());
            	    					}
            	    					add(
            	    						current,
            	    						"outputs",
            	    						lv_outputs_4_0,
            	    						"org.icyphy.LinguaFranca.Output");
            	    					afterParserOrEnumRuleCall();
            	    				

            	    }


            	    }
            	    break;

            	default :
            	    break loop8;
                }
            } while (true);

            // InternalLinguaFranca.g:491:3: ( ( (lv_timers_5_0= ruleTimer ) ) | ( (lv_actions_6_0= ruleAction ) ) )*
            loop9:
            do {
                int alt9=3;
                int LA9_0 = input.LA(1);

                if ( (LA9_0==23) ) {
                    alt9=1;
                }
                else if ( (LA9_0==24) ) {
                    alt9=2;
                }


                switch (alt9) {
            	case 1 :
            	    // InternalLinguaFranca.g:492:4: ( (lv_timers_5_0= ruleTimer ) )
            	    {
            	    // InternalLinguaFranca.g:492:4: ( (lv_timers_5_0= ruleTimer ) )
            	    // InternalLinguaFranca.g:493:5: (lv_timers_5_0= ruleTimer )
            	    {
            	    // InternalLinguaFranca.g:493:5: (lv_timers_5_0= ruleTimer )
            	    // InternalLinguaFranca.g:494:6: lv_timers_5_0= ruleTimer
            	    {

            	    						newCompositeNode(grammarAccess.getComponentBodyAccess().getTimersTimerParserRuleCall_5_0_0());
            	    					
            	    pushFollow(FOLLOW_13);
            	    lv_timers_5_0=ruleTimer();

            	    state._fsp--;


            	    						if (current==null) {
            	    							current = createModelElementForParent(grammarAccess.getComponentBodyRule());
            	    						}
            	    						add(
            	    							current,
            	    							"timers",
            	    							lv_timers_5_0,
            	    							"org.icyphy.LinguaFranca.Timer");
            	    						afterParserOrEnumRuleCall();
            	    					

            	    }


            	    }


            	    }
            	    break;
            	case 2 :
            	    // InternalLinguaFranca.g:512:4: ( (lv_actions_6_0= ruleAction ) )
            	    {
            	    // InternalLinguaFranca.g:512:4: ( (lv_actions_6_0= ruleAction ) )
            	    // InternalLinguaFranca.g:513:5: (lv_actions_6_0= ruleAction )
            	    {
            	    // InternalLinguaFranca.g:513:5: (lv_actions_6_0= ruleAction )
            	    // InternalLinguaFranca.g:514:6: lv_actions_6_0= ruleAction
            	    {

            	    						newCompositeNode(grammarAccess.getComponentBodyAccess().getActionsActionParserRuleCall_5_1_0());
            	    					
            	    pushFollow(FOLLOW_13);
            	    lv_actions_6_0=ruleAction();

            	    state._fsp--;


            	    						if (current==null) {
            	    							current = createModelElementForParent(grammarAccess.getComponentBodyRule());
            	    						}
            	    						add(
            	    							current,
            	    							"actions",
            	    							lv_actions_6_0,
            	    							"org.icyphy.LinguaFranca.Action");
            	    						afterParserOrEnumRuleCall();
            	    					

            	    }


            	    }


            	    }
            	    break;

            	default :
            	    break loop9;
                }
            } while (true);

            // InternalLinguaFranca.g:532:3: ( (lv_preamble_7_0= rulePreamble ) )?
            int alt10=2;
            int LA10_0 = input.LA(1);

            if ( (LA10_0==29) ) {
                alt10=1;
            }
            switch (alt10) {
                case 1 :
                    // InternalLinguaFranca.g:533:4: (lv_preamble_7_0= rulePreamble )
                    {
                    // InternalLinguaFranca.g:533:4: (lv_preamble_7_0= rulePreamble )
                    // InternalLinguaFranca.g:534:5: lv_preamble_7_0= rulePreamble
                    {

                    					newCompositeNode(grammarAccess.getComponentBodyAccess().getPreamblePreambleParserRuleCall_6_0());
                    				
                    pushFollow(FOLLOW_14);
                    lv_preamble_7_0=rulePreamble();

                    state._fsp--;


                    					if (current==null) {
                    						current = createModelElementForParent(grammarAccess.getComponentBodyRule());
                    					}
                    					set(
                    						current,
                    						"preamble",
                    						lv_preamble_7_0,
                    						"org.icyphy.LinguaFranca.Preamble");
                    					afterParserOrEnumRuleCall();
                    				

                    }


                    }
                    break;

            }

            // InternalLinguaFranca.g:551:3: ( (lv_reactions_8_0= ruleReaction ) )*
            loop11:
            do {
                int alt11=2;
                int LA11_0 = input.LA(1);

                if ( (LA11_0==25) ) {
                    alt11=1;
                }


                switch (alt11) {
            	case 1 :
            	    // InternalLinguaFranca.g:552:4: (lv_reactions_8_0= ruleReaction )
            	    {
            	    // InternalLinguaFranca.g:552:4: (lv_reactions_8_0= ruleReaction )
            	    // InternalLinguaFranca.g:553:5: lv_reactions_8_0= ruleReaction
            	    {

            	    					newCompositeNode(grammarAccess.getComponentBodyAccess().getReactionsReactionParserRuleCall_7_0());
            	    				
            	    pushFollow(FOLLOW_14);
            	    lv_reactions_8_0=ruleReaction();

            	    state._fsp--;


            	    					if (current==null) {
            	    						current = createModelElementForParent(grammarAccess.getComponentBodyRule());
            	    					}
            	    					add(
            	    						current,
            	    						"reactions",
            	    						lv_reactions_8_0,
            	    						"org.icyphy.LinguaFranca.Reaction");
            	    					afterParserOrEnumRuleCall();
            	    				

            	    }


            	    }
            	    break;

            	default :
            	    break loop11;
                }
            } while (true);


            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleComponentBody"


    // $ANTLR start "entryRuleInput"
    // InternalLinguaFranca.g:574:1: entryRuleInput returns [EObject current=null] : iv_ruleInput= ruleInput EOF ;
    public final EObject entryRuleInput() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleInput = null;


        try {
            // InternalLinguaFranca.g:574:46: (iv_ruleInput= ruleInput EOF )
            // InternalLinguaFranca.g:575:2: iv_ruleInput= ruleInput EOF
            {
             newCompositeNode(grammarAccess.getInputRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleInput=ruleInput();

            state._fsp--;

             current =iv_ruleInput; 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleInput"


    // $ANTLR start "ruleInput"
    // InternalLinguaFranca.g:581:1: ruleInput returns [EObject current=null] : (otherlv_0= 'input' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'input' ) ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? otherlv_4= ';' ) ;
    public final EObject ruleInput() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token lv_name_1_1=null;
        Token lv_name_1_2=null;
        Token otherlv_2=null;
        Token otherlv_4=null;
        AntlrDatatypeRuleToken lv_type_3_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:587:2: ( (otherlv_0= 'input' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'input' ) ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? otherlv_4= ';' ) )
            // InternalLinguaFranca.g:588:2: (otherlv_0= 'input' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'input' ) ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? otherlv_4= ';' )
            {
            // InternalLinguaFranca.g:588:2: (otherlv_0= 'input' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'input' ) ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? otherlv_4= ';' )
            // InternalLinguaFranca.g:589:3: otherlv_0= 'input' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'input' ) ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? otherlv_4= ';'
            {
            otherlv_0=(Token)match(input,20,FOLLOW_15); 

            			newLeafNode(otherlv_0, grammarAccess.getInputAccess().getInputKeyword_0());
            		
            // InternalLinguaFranca.g:593:3: ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'input' ) ) )
            // InternalLinguaFranca.g:594:4: ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'input' ) )
            {
            // InternalLinguaFranca.g:594:4: ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'input' ) )
            // InternalLinguaFranca.g:595:5: (lv_name_1_1= RULE_ID | lv_name_1_2= 'input' )
            {
            // InternalLinguaFranca.g:595:5: (lv_name_1_1= RULE_ID | lv_name_1_2= 'input' )
            int alt12=2;
            int LA12_0 = input.LA(1);

            if ( (LA12_0==RULE_ID) ) {
                alt12=1;
            }
            else if ( (LA12_0==20) ) {
                alt12=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 12, 0, input);

                throw nvae;
            }
            switch (alt12) {
                case 1 :
                    // InternalLinguaFranca.g:596:6: lv_name_1_1= RULE_ID
                    {
                    lv_name_1_1=(Token)match(input,RULE_ID,FOLLOW_16); 

                    						newLeafNode(lv_name_1_1, grammarAccess.getInputAccess().getNameIDTerminalRuleCall_1_0_0());
                    					

                    						if (current==null) {
                    							current = createModelElement(grammarAccess.getInputRule());
                    						}
                    						setWithLastConsumed(
                    							current,
                    							"name",
                    							lv_name_1_1,
                    							"org.eclipse.xtext.common.Terminals.ID");
                    					

                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:611:6: lv_name_1_2= 'input'
                    {
                    lv_name_1_2=(Token)match(input,20,FOLLOW_16); 

                    						newLeafNode(lv_name_1_2, grammarAccess.getInputAccess().getNameInputKeyword_1_0_1());
                    					

                    						if (current==null) {
                    							current = createModelElement(grammarAccess.getInputRule());
                    						}
                    						setWithLastConsumed(current, "name", lv_name_1_2, null);
                    					

                    }
                    break;

            }


            }


            }

            // InternalLinguaFranca.g:624:3: (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )?
            int alt13=2;
            int LA13_0 = input.LA(1);

            if ( (LA13_0==21) ) {
                alt13=1;
            }
            switch (alt13) {
                case 1 :
                    // InternalLinguaFranca.g:625:4: otherlv_2= ':' ( (lv_type_3_0= ruleType ) )
                    {
                    otherlv_2=(Token)match(input,21,FOLLOW_17); 

                    				newLeafNode(otherlv_2, grammarAccess.getInputAccess().getColonKeyword_2_0());
                    			
                    // InternalLinguaFranca.g:629:4: ( (lv_type_3_0= ruleType ) )
                    // InternalLinguaFranca.g:630:5: (lv_type_3_0= ruleType )
                    {
                    // InternalLinguaFranca.g:630:5: (lv_type_3_0= ruleType )
                    // InternalLinguaFranca.g:631:6: lv_type_3_0= ruleType
                    {

                    						newCompositeNode(grammarAccess.getInputAccess().getTypeTypeParserRuleCall_2_1_0());
                    					
                    pushFollow(FOLLOW_6);
                    lv_type_3_0=ruleType();

                    state._fsp--;


                    						if (current==null) {
                    							current = createModelElementForParent(grammarAccess.getInputRule());
                    						}
                    						set(
                    							current,
                    							"type",
                    							lv_type_3_0,
                    							"org.icyphy.LinguaFranca.Type");
                    						afterParserOrEnumRuleCall();
                    					

                    }


                    }


                    }
                    break;

            }

            otherlv_4=(Token)match(input,14,FOLLOW_2); 

            			newLeafNode(otherlv_4, grammarAccess.getInputAccess().getSemicolonKeyword_3());
            		

            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleInput"


    // $ANTLR start "entryRuleOutput"
    // InternalLinguaFranca.g:657:1: entryRuleOutput returns [EObject current=null] : iv_ruleOutput= ruleOutput EOF ;
    public final EObject entryRuleOutput() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleOutput = null;


        try {
            // InternalLinguaFranca.g:657:47: (iv_ruleOutput= ruleOutput EOF )
            // InternalLinguaFranca.g:658:2: iv_ruleOutput= ruleOutput EOF
            {
             newCompositeNode(grammarAccess.getOutputRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleOutput=ruleOutput();

            state._fsp--;

             current =iv_ruleOutput; 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleOutput"


    // $ANTLR start "ruleOutput"
    // InternalLinguaFranca.g:664:1: ruleOutput returns [EObject current=null] : (otherlv_0= 'output' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'output' ) ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? otherlv_4= ';' ) ;
    public final EObject ruleOutput() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token lv_name_1_1=null;
        Token lv_name_1_2=null;
        Token otherlv_2=null;
        Token otherlv_4=null;
        AntlrDatatypeRuleToken lv_type_3_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:670:2: ( (otherlv_0= 'output' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'output' ) ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? otherlv_4= ';' ) )
            // InternalLinguaFranca.g:671:2: (otherlv_0= 'output' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'output' ) ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? otherlv_4= ';' )
            {
            // InternalLinguaFranca.g:671:2: (otherlv_0= 'output' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'output' ) ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? otherlv_4= ';' )
            // InternalLinguaFranca.g:672:3: otherlv_0= 'output' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'output' ) ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? otherlv_4= ';'
            {
            otherlv_0=(Token)match(input,22,FOLLOW_18); 

            			newLeafNode(otherlv_0, grammarAccess.getOutputAccess().getOutputKeyword_0());
            		
            // InternalLinguaFranca.g:676:3: ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'output' ) ) )
            // InternalLinguaFranca.g:677:4: ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'output' ) )
            {
            // InternalLinguaFranca.g:677:4: ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'output' ) )
            // InternalLinguaFranca.g:678:5: (lv_name_1_1= RULE_ID | lv_name_1_2= 'output' )
            {
            // InternalLinguaFranca.g:678:5: (lv_name_1_1= RULE_ID | lv_name_1_2= 'output' )
            int alt14=2;
            int LA14_0 = input.LA(1);

            if ( (LA14_0==RULE_ID) ) {
                alt14=1;
            }
            else if ( (LA14_0==22) ) {
                alt14=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 14, 0, input);

                throw nvae;
            }
            switch (alt14) {
                case 1 :
                    // InternalLinguaFranca.g:679:6: lv_name_1_1= RULE_ID
                    {
                    lv_name_1_1=(Token)match(input,RULE_ID,FOLLOW_16); 

                    						newLeafNode(lv_name_1_1, grammarAccess.getOutputAccess().getNameIDTerminalRuleCall_1_0_0());
                    					

                    						if (current==null) {
                    							current = createModelElement(grammarAccess.getOutputRule());
                    						}
                    						setWithLastConsumed(
                    							current,
                    							"name",
                    							lv_name_1_1,
                    							"org.eclipse.xtext.common.Terminals.ID");
                    					

                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:694:6: lv_name_1_2= 'output'
                    {
                    lv_name_1_2=(Token)match(input,22,FOLLOW_16); 

                    						newLeafNode(lv_name_1_2, grammarAccess.getOutputAccess().getNameOutputKeyword_1_0_1());
                    					

                    						if (current==null) {
                    							current = createModelElement(grammarAccess.getOutputRule());
                    						}
                    						setWithLastConsumed(current, "name", lv_name_1_2, null);
                    					

                    }
                    break;

            }


            }


            }

            // InternalLinguaFranca.g:707:3: (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )?
            int alt15=2;
            int LA15_0 = input.LA(1);

            if ( (LA15_0==21) ) {
                alt15=1;
            }
            switch (alt15) {
                case 1 :
                    // InternalLinguaFranca.g:708:4: otherlv_2= ':' ( (lv_type_3_0= ruleType ) )
                    {
                    otherlv_2=(Token)match(input,21,FOLLOW_17); 

                    				newLeafNode(otherlv_2, grammarAccess.getOutputAccess().getColonKeyword_2_0());
                    			
                    // InternalLinguaFranca.g:712:4: ( (lv_type_3_0= ruleType ) )
                    // InternalLinguaFranca.g:713:5: (lv_type_3_0= ruleType )
                    {
                    // InternalLinguaFranca.g:713:5: (lv_type_3_0= ruleType )
                    // InternalLinguaFranca.g:714:6: lv_type_3_0= ruleType
                    {

                    						newCompositeNode(grammarAccess.getOutputAccess().getTypeTypeParserRuleCall_2_1_0());
                    					
                    pushFollow(FOLLOW_6);
                    lv_type_3_0=ruleType();

                    state._fsp--;


                    						if (current==null) {
                    							current = createModelElementForParent(grammarAccess.getOutputRule());
                    						}
                    						set(
                    							current,
                    							"type",
                    							lv_type_3_0,
                    							"org.icyphy.LinguaFranca.Type");
                    						afterParserOrEnumRuleCall();
                    					

                    }


                    }


                    }
                    break;

            }

            otherlv_4=(Token)match(input,14,FOLLOW_2); 

            			newLeafNode(otherlv_4, grammarAccess.getOutputAccess().getSemicolonKeyword_3());
            		

            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleOutput"


    // $ANTLR start "entryRuleTimer"
    // InternalLinguaFranca.g:740:1: entryRuleTimer returns [EObject current=null] : iv_ruleTimer= ruleTimer EOF ;
    public final EObject entryRuleTimer() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleTimer = null;


        try {
            // InternalLinguaFranca.g:740:46: (iv_ruleTimer= ruleTimer EOF )
            // InternalLinguaFranca.g:741:2: iv_ruleTimer= ruleTimer EOF
            {
             newCompositeNode(grammarAccess.getTimerRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleTimer=ruleTimer();

            state._fsp--;

             current =iv_ruleTimer; 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleTimer"


    // $ANTLR start "ruleTimer"
    // InternalLinguaFranca.g:747:1: ruleTimer returns [EObject current=null] : (otherlv_0= 'timer' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'timer' ) ) ) ( (lv_timing_2_0= ruleTiming ) )? otherlv_3= ';' ) ;
    public final EObject ruleTimer() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token lv_name_1_1=null;
        Token lv_name_1_2=null;
        Token otherlv_3=null;
        EObject lv_timing_2_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:753:2: ( (otherlv_0= 'timer' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'timer' ) ) ) ( (lv_timing_2_0= ruleTiming ) )? otherlv_3= ';' ) )
            // InternalLinguaFranca.g:754:2: (otherlv_0= 'timer' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'timer' ) ) ) ( (lv_timing_2_0= ruleTiming ) )? otherlv_3= ';' )
            {
            // InternalLinguaFranca.g:754:2: (otherlv_0= 'timer' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'timer' ) ) ) ( (lv_timing_2_0= ruleTiming ) )? otherlv_3= ';' )
            // InternalLinguaFranca.g:755:3: otherlv_0= 'timer' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'timer' ) ) ) ( (lv_timing_2_0= ruleTiming ) )? otherlv_3= ';'
            {
            otherlv_0=(Token)match(input,23,FOLLOW_19); 

            			newLeafNode(otherlv_0, grammarAccess.getTimerAccess().getTimerKeyword_0());
            		
            // InternalLinguaFranca.g:759:3: ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'timer' ) ) )
            // InternalLinguaFranca.g:760:4: ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'timer' ) )
            {
            // InternalLinguaFranca.g:760:4: ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'timer' ) )
            // InternalLinguaFranca.g:761:5: (lv_name_1_1= RULE_ID | lv_name_1_2= 'timer' )
            {
            // InternalLinguaFranca.g:761:5: (lv_name_1_1= RULE_ID | lv_name_1_2= 'timer' )
            int alt16=2;
            int LA16_0 = input.LA(1);

            if ( (LA16_0==RULE_ID) ) {
                alt16=1;
            }
            else if ( (LA16_0==23) ) {
                alt16=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 16, 0, input);

                throw nvae;
            }
            switch (alt16) {
                case 1 :
                    // InternalLinguaFranca.g:762:6: lv_name_1_1= RULE_ID
                    {
                    lv_name_1_1=(Token)match(input,RULE_ID,FOLLOW_20); 

                    						newLeafNode(lv_name_1_1, grammarAccess.getTimerAccess().getNameIDTerminalRuleCall_1_0_0());
                    					

                    						if (current==null) {
                    							current = createModelElement(grammarAccess.getTimerRule());
                    						}
                    						setWithLastConsumed(
                    							current,
                    							"name",
                    							lv_name_1_1,
                    							"org.eclipse.xtext.common.Terminals.ID");
                    					

                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:777:6: lv_name_1_2= 'timer'
                    {
                    lv_name_1_2=(Token)match(input,23,FOLLOW_20); 

                    						newLeafNode(lv_name_1_2, grammarAccess.getTimerAccess().getNameTimerKeyword_1_0_1());
                    					

                    						if (current==null) {
                    							current = createModelElement(grammarAccess.getTimerRule());
                    						}
                    						setWithLastConsumed(current, "name", lv_name_1_2, null);
                    					

                    }
                    break;

            }


            }


            }

            // InternalLinguaFranca.g:790:3: ( (lv_timing_2_0= ruleTiming ) )?
            int alt17=2;
            int LA17_0 = input.LA(1);

            if ( (LA17_0==26) ) {
                alt17=1;
            }
            switch (alt17) {
                case 1 :
                    // InternalLinguaFranca.g:791:4: (lv_timing_2_0= ruleTiming )
                    {
                    // InternalLinguaFranca.g:791:4: (lv_timing_2_0= ruleTiming )
                    // InternalLinguaFranca.g:792:5: lv_timing_2_0= ruleTiming
                    {

                    					newCompositeNode(grammarAccess.getTimerAccess().getTimingTimingParserRuleCall_2_0());
                    				
                    pushFollow(FOLLOW_6);
                    lv_timing_2_0=ruleTiming();

                    state._fsp--;


                    					if (current==null) {
                    						current = createModelElementForParent(grammarAccess.getTimerRule());
                    					}
                    					set(
                    						current,
                    						"timing",
                    						lv_timing_2_0,
                    						"org.icyphy.LinguaFranca.Timing");
                    					afterParserOrEnumRuleCall();
                    				

                    }


                    }
                    break;

            }

            otherlv_3=(Token)match(input,14,FOLLOW_2); 

            			newLeafNode(otherlv_3, grammarAccess.getTimerAccess().getSemicolonKeyword_3());
            		

            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleTimer"


    // $ANTLR start "entryRuleAction"
    // InternalLinguaFranca.g:817:1: entryRuleAction returns [EObject current=null] : iv_ruleAction= ruleAction EOF ;
    public final EObject entryRuleAction() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleAction = null;


        try {
            // InternalLinguaFranca.g:817:47: (iv_ruleAction= ruleAction EOF )
            // InternalLinguaFranca.g:818:2: iv_ruleAction= ruleAction EOF
            {
             newCompositeNode(grammarAccess.getActionRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleAction=ruleAction();

            state._fsp--;

             current =iv_ruleAction; 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleAction"


    // $ANTLR start "ruleAction"
    // InternalLinguaFranca.g:824:1: ruleAction returns [EObject current=null] : (otherlv_0= 'action' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'action' ) ) ) ( (lv_timing_2_0= ruleTiming ) )? otherlv_3= ';' ) ;
    public final EObject ruleAction() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token lv_name_1_1=null;
        Token lv_name_1_2=null;
        Token otherlv_3=null;
        EObject lv_timing_2_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:830:2: ( (otherlv_0= 'action' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'action' ) ) ) ( (lv_timing_2_0= ruleTiming ) )? otherlv_3= ';' ) )
            // InternalLinguaFranca.g:831:2: (otherlv_0= 'action' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'action' ) ) ) ( (lv_timing_2_0= ruleTiming ) )? otherlv_3= ';' )
            {
            // InternalLinguaFranca.g:831:2: (otherlv_0= 'action' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'action' ) ) ) ( (lv_timing_2_0= ruleTiming ) )? otherlv_3= ';' )
            // InternalLinguaFranca.g:832:3: otherlv_0= 'action' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'action' ) ) ) ( (lv_timing_2_0= ruleTiming ) )? otherlv_3= ';'
            {
            otherlv_0=(Token)match(input,24,FOLLOW_21); 

            			newLeafNode(otherlv_0, grammarAccess.getActionAccess().getActionKeyword_0());
            		
            // InternalLinguaFranca.g:836:3: ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'action' ) ) )
            // InternalLinguaFranca.g:837:4: ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'action' ) )
            {
            // InternalLinguaFranca.g:837:4: ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'action' ) )
            // InternalLinguaFranca.g:838:5: (lv_name_1_1= RULE_ID | lv_name_1_2= 'action' )
            {
            // InternalLinguaFranca.g:838:5: (lv_name_1_1= RULE_ID | lv_name_1_2= 'action' )
            int alt18=2;
            int LA18_0 = input.LA(1);

            if ( (LA18_0==RULE_ID) ) {
                alt18=1;
            }
            else if ( (LA18_0==24) ) {
                alt18=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 18, 0, input);

                throw nvae;
            }
            switch (alt18) {
                case 1 :
                    // InternalLinguaFranca.g:839:6: lv_name_1_1= RULE_ID
                    {
                    lv_name_1_1=(Token)match(input,RULE_ID,FOLLOW_20); 

                    						newLeafNode(lv_name_1_1, grammarAccess.getActionAccess().getNameIDTerminalRuleCall_1_0_0());
                    					

                    						if (current==null) {
                    							current = createModelElement(grammarAccess.getActionRule());
                    						}
                    						setWithLastConsumed(
                    							current,
                    							"name",
                    							lv_name_1_1,
                    							"org.eclipse.xtext.common.Terminals.ID");
                    					

                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:854:6: lv_name_1_2= 'action'
                    {
                    lv_name_1_2=(Token)match(input,24,FOLLOW_20); 

                    						newLeafNode(lv_name_1_2, grammarAccess.getActionAccess().getNameActionKeyword_1_0_1());
                    					

                    						if (current==null) {
                    							current = createModelElement(grammarAccess.getActionRule());
                    						}
                    						setWithLastConsumed(current, "name", lv_name_1_2, null);
                    					

                    }
                    break;

            }


            }


            }

            // InternalLinguaFranca.g:867:3: ( (lv_timing_2_0= ruleTiming ) )?
            int alt19=2;
            int LA19_0 = input.LA(1);

            if ( (LA19_0==26) ) {
                alt19=1;
            }
            switch (alt19) {
                case 1 :
                    // InternalLinguaFranca.g:868:4: (lv_timing_2_0= ruleTiming )
                    {
                    // InternalLinguaFranca.g:868:4: (lv_timing_2_0= ruleTiming )
                    // InternalLinguaFranca.g:869:5: lv_timing_2_0= ruleTiming
                    {

                    					newCompositeNode(grammarAccess.getActionAccess().getTimingTimingParserRuleCall_2_0());
                    				
                    pushFollow(FOLLOW_6);
                    lv_timing_2_0=ruleTiming();

                    state._fsp--;


                    					if (current==null) {
                    						current = createModelElementForParent(grammarAccess.getActionRule());
                    					}
                    					set(
                    						current,
                    						"timing",
                    						lv_timing_2_0,
                    						"org.icyphy.LinguaFranca.Timing");
                    					afterParserOrEnumRuleCall();
                    				

                    }


                    }
                    break;

            }

            otherlv_3=(Token)match(input,14,FOLLOW_2); 

            			newLeafNode(otherlv_3, grammarAccess.getActionAccess().getSemicolonKeyword_3());
            		

            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleAction"


    // $ANTLR start "entryRuleReaction"
    // InternalLinguaFranca.g:894:1: entryRuleReaction returns [EObject current=null] : iv_ruleReaction= ruleReaction EOF ;
    public final EObject entryRuleReaction() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleReaction = null;


        try {
            // InternalLinguaFranca.g:894:49: (iv_ruleReaction= ruleReaction EOF )
            // InternalLinguaFranca.g:895:2: iv_ruleReaction= ruleReaction EOF
            {
             newCompositeNode(grammarAccess.getReactionRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleReaction=ruleReaction();

            state._fsp--;

             current =iv_ruleReaction; 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleReaction"


    // $ANTLR start "ruleReaction"
    // InternalLinguaFranca.g:901:1: ruleReaction returns [EObject current=null] : (otherlv_0= 'reaction' (otherlv_1= '(' ( ( (lv_triggers_2_0= RULE_ID ) ) (otherlv_3= ',' ( (lv_triggers_4_0= RULE_ID ) ) )* )? otherlv_5= ')' )? ( (lv_gets_6_0= ruleGets ) )? ( (lv_sets_7_0= ruleSets ) )? ( (lv_code_8_0= RULE_CODE ) ) ) ;
    public final EObject ruleReaction() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token otherlv_1=null;
        Token lv_triggers_2_0=null;
        Token otherlv_3=null;
        Token lv_triggers_4_0=null;
        Token otherlv_5=null;
        Token lv_code_8_0=null;
        EObject lv_gets_6_0 = null;

        EObject lv_sets_7_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:907:2: ( (otherlv_0= 'reaction' (otherlv_1= '(' ( ( (lv_triggers_2_0= RULE_ID ) ) (otherlv_3= ',' ( (lv_triggers_4_0= RULE_ID ) ) )* )? otherlv_5= ')' )? ( (lv_gets_6_0= ruleGets ) )? ( (lv_sets_7_0= ruleSets ) )? ( (lv_code_8_0= RULE_CODE ) ) ) )
            // InternalLinguaFranca.g:908:2: (otherlv_0= 'reaction' (otherlv_1= '(' ( ( (lv_triggers_2_0= RULE_ID ) ) (otherlv_3= ',' ( (lv_triggers_4_0= RULE_ID ) ) )* )? otherlv_5= ')' )? ( (lv_gets_6_0= ruleGets ) )? ( (lv_sets_7_0= ruleSets ) )? ( (lv_code_8_0= RULE_CODE ) ) )
            {
            // InternalLinguaFranca.g:908:2: (otherlv_0= 'reaction' (otherlv_1= '(' ( ( (lv_triggers_2_0= RULE_ID ) ) (otherlv_3= ',' ( (lv_triggers_4_0= RULE_ID ) ) )* )? otherlv_5= ')' )? ( (lv_gets_6_0= ruleGets ) )? ( (lv_sets_7_0= ruleSets ) )? ( (lv_code_8_0= RULE_CODE ) ) )
            // InternalLinguaFranca.g:909:3: otherlv_0= 'reaction' (otherlv_1= '(' ( ( (lv_triggers_2_0= RULE_ID ) ) (otherlv_3= ',' ( (lv_triggers_4_0= RULE_ID ) ) )* )? otherlv_5= ')' )? ( (lv_gets_6_0= ruleGets ) )? ( (lv_sets_7_0= ruleSets ) )? ( (lv_code_8_0= RULE_CODE ) )
            {
            otherlv_0=(Token)match(input,25,FOLLOW_22); 

            			newLeafNode(otherlv_0, grammarAccess.getReactionAccess().getReactionKeyword_0());
            		
            // InternalLinguaFranca.g:913:3: (otherlv_1= '(' ( ( (lv_triggers_2_0= RULE_ID ) ) (otherlv_3= ',' ( (lv_triggers_4_0= RULE_ID ) ) )* )? otherlv_5= ')' )?
            int alt22=2;
            int LA22_0 = input.LA(1);

            if ( (LA22_0==26) ) {
                alt22=1;
            }
            switch (alt22) {
                case 1 :
                    // InternalLinguaFranca.g:914:4: otherlv_1= '(' ( ( (lv_triggers_2_0= RULE_ID ) ) (otherlv_3= ',' ( (lv_triggers_4_0= RULE_ID ) ) )* )? otherlv_5= ')'
                    {
                    otherlv_1=(Token)match(input,26,FOLLOW_23); 

                    				newLeafNode(otherlv_1, grammarAccess.getReactionAccess().getLeftParenthesisKeyword_1_0());
                    			
                    // InternalLinguaFranca.g:918:4: ( ( (lv_triggers_2_0= RULE_ID ) ) (otherlv_3= ',' ( (lv_triggers_4_0= RULE_ID ) ) )* )?
                    int alt21=2;
                    int LA21_0 = input.LA(1);

                    if ( (LA21_0==RULE_ID) ) {
                        alt21=1;
                    }
                    switch (alt21) {
                        case 1 :
                            // InternalLinguaFranca.g:919:5: ( (lv_triggers_2_0= RULE_ID ) ) (otherlv_3= ',' ( (lv_triggers_4_0= RULE_ID ) ) )*
                            {
                            // InternalLinguaFranca.g:919:5: ( (lv_triggers_2_0= RULE_ID ) )
                            // InternalLinguaFranca.g:920:6: (lv_triggers_2_0= RULE_ID )
                            {
                            // InternalLinguaFranca.g:920:6: (lv_triggers_2_0= RULE_ID )
                            // InternalLinguaFranca.g:921:7: lv_triggers_2_0= RULE_ID
                            {
                            lv_triggers_2_0=(Token)match(input,RULE_ID,FOLLOW_24); 

                            							newLeafNode(lv_triggers_2_0, grammarAccess.getReactionAccess().getTriggersIDTerminalRuleCall_1_1_0_0());
                            						

                            							if (current==null) {
                            								current = createModelElement(grammarAccess.getReactionRule());
                            							}
                            							addWithLastConsumed(
                            								current,
                            								"triggers",
                            								lv_triggers_2_0,
                            								"org.eclipse.xtext.common.Terminals.ID");
                            						

                            }


                            }

                            // InternalLinguaFranca.g:937:5: (otherlv_3= ',' ( (lv_triggers_4_0= RULE_ID ) ) )*
                            loop20:
                            do {
                                int alt20=2;
                                int LA20_0 = input.LA(1);

                                if ( (LA20_0==27) ) {
                                    alt20=1;
                                }


                                switch (alt20) {
                            	case 1 :
                            	    // InternalLinguaFranca.g:938:6: otherlv_3= ',' ( (lv_triggers_4_0= RULE_ID ) )
                            	    {
                            	    otherlv_3=(Token)match(input,27,FOLLOW_5); 

                            	    						newLeafNode(otherlv_3, grammarAccess.getReactionAccess().getCommaKeyword_1_1_1_0());
                            	    					
                            	    // InternalLinguaFranca.g:942:6: ( (lv_triggers_4_0= RULE_ID ) )
                            	    // InternalLinguaFranca.g:943:7: (lv_triggers_4_0= RULE_ID )
                            	    {
                            	    // InternalLinguaFranca.g:943:7: (lv_triggers_4_0= RULE_ID )
                            	    // InternalLinguaFranca.g:944:8: lv_triggers_4_0= RULE_ID
                            	    {
                            	    lv_triggers_4_0=(Token)match(input,RULE_ID,FOLLOW_24); 

                            	    								newLeafNode(lv_triggers_4_0, grammarAccess.getReactionAccess().getTriggersIDTerminalRuleCall_1_1_1_1_0());
                            	    							

                            	    								if (current==null) {
                            	    									current = createModelElement(grammarAccess.getReactionRule());
                            	    								}
                            	    								addWithLastConsumed(
                            	    									current,
                            	    									"triggers",
                            	    									lv_triggers_4_0,
                            	    									"org.eclipse.xtext.common.Terminals.ID");
                            	    							

                            	    }


                            	    }


                            	    }
                            	    break;

                            	default :
                            	    break loop20;
                                }
                            } while (true);


                            }
                            break;

                    }

                    otherlv_5=(Token)match(input,28,FOLLOW_25); 

                    				newLeafNode(otherlv_5, grammarAccess.getReactionAccess().getRightParenthesisKeyword_1_2());
                    			

                    }
                    break;

            }

            // InternalLinguaFranca.g:967:3: ( (lv_gets_6_0= ruleGets ) )?
            int alt23=2;
            int LA23_0 = input.LA(1);

            if ( (LA23_0==RULE_ID) ) {
                alt23=1;
            }
            switch (alt23) {
                case 1 :
                    // InternalLinguaFranca.g:968:4: (lv_gets_6_0= ruleGets )
                    {
                    // InternalLinguaFranca.g:968:4: (lv_gets_6_0= ruleGets )
                    // InternalLinguaFranca.g:969:5: lv_gets_6_0= ruleGets
                    {

                    					newCompositeNode(grammarAccess.getReactionAccess().getGetsGetsParserRuleCall_2_0());
                    				
                    pushFollow(FOLLOW_26);
                    lv_gets_6_0=ruleGets();

                    state._fsp--;


                    					if (current==null) {
                    						current = createModelElementForParent(grammarAccess.getReactionRule());
                    					}
                    					set(
                    						current,
                    						"gets",
                    						lv_gets_6_0,
                    						"org.icyphy.LinguaFranca.Gets");
                    					afterParserOrEnumRuleCall();
                    				

                    }


                    }
                    break;

            }

            // InternalLinguaFranca.g:986:3: ( (lv_sets_7_0= ruleSets ) )?
            int alt24=2;
            int LA24_0 = input.LA(1);

            if ( (LA24_0==32) ) {
                alt24=1;
            }
            switch (alt24) {
                case 1 :
                    // InternalLinguaFranca.g:987:4: (lv_sets_7_0= ruleSets )
                    {
                    // InternalLinguaFranca.g:987:4: (lv_sets_7_0= ruleSets )
                    // InternalLinguaFranca.g:988:5: lv_sets_7_0= ruleSets
                    {

                    					newCompositeNode(grammarAccess.getReactionAccess().getSetsSetsParserRuleCall_3_0());
                    				
                    pushFollow(FOLLOW_27);
                    lv_sets_7_0=ruleSets();

                    state._fsp--;


                    					if (current==null) {
                    						current = createModelElementForParent(grammarAccess.getReactionRule());
                    					}
                    					set(
                    						current,
                    						"sets",
                    						lv_sets_7_0,
                    						"org.icyphy.LinguaFranca.Sets");
                    					afterParserOrEnumRuleCall();
                    				

                    }


                    }
                    break;

            }

            // InternalLinguaFranca.g:1005:3: ( (lv_code_8_0= RULE_CODE ) )
            // InternalLinguaFranca.g:1006:4: (lv_code_8_0= RULE_CODE )
            {
            // InternalLinguaFranca.g:1006:4: (lv_code_8_0= RULE_CODE )
            // InternalLinguaFranca.g:1007:5: lv_code_8_0= RULE_CODE
            {
            lv_code_8_0=(Token)match(input,RULE_CODE,FOLLOW_2); 

            					newLeafNode(lv_code_8_0, grammarAccess.getReactionAccess().getCodeCODETerminalRuleCall_4_0());
            				

            					if (current==null) {
            						current = createModelElement(grammarAccess.getReactionRule());
            					}
            					setWithLastConsumed(
            						current,
            						"code",
            						lv_code_8_0,
            						"org.icyphy.LinguaFranca.CODE");
            				

            }


            }


            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleReaction"


    // $ANTLR start "entryRulePreamble"
    // InternalLinguaFranca.g:1027:1: entryRulePreamble returns [EObject current=null] : iv_rulePreamble= rulePreamble EOF ;
    public final EObject entryRulePreamble() throws RecognitionException {
        EObject current = null;

        EObject iv_rulePreamble = null;


        try {
            // InternalLinguaFranca.g:1027:49: (iv_rulePreamble= rulePreamble EOF )
            // InternalLinguaFranca.g:1028:2: iv_rulePreamble= rulePreamble EOF
            {
             newCompositeNode(grammarAccess.getPreambleRule()); 
            pushFollow(FOLLOW_1);
            iv_rulePreamble=rulePreamble();

            state._fsp--;

             current =iv_rulePreamble; 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRulePreamble"


    // $ANTLR start "rulePreamble"
    // InternalLinguaFranca.g:1034:1: rulePreamble returns [EObject current=null] : (otherlv_0= 'preamble' ( (lv_code_1_0= RULE_CODE ) ) ) ;
    public final EObject rulePreamble() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token lv_code_1_0=null;


        	enterRule();

        try {
            // InternalLinguaFranca.g:1040:2: ( (otherlv_0= 'preamble' ( (lv_code_1_0= RULE_CODE ) ) ) )
            // InternalLinguaFranca.g:1041:2: (otherlv_0= 'preamble' ( (lv_code_1_0= RULE_CODE ) ) )
            {
            // InternalLinguaFranca.g:1041:2: (otherlv_0= 'preamble' ( (lv_code_1_0= RULE_CODE ) ) )
            // InternalLinguaFranca.g:1042:3: otherlv_0= 'preamble' ( (lv_code_1_0= RULE_CODE ) )
            {
            otherlv_0=(Token)match(input,29,FOLLOW_27); 

            			newLeafNode(otherlv_0, grammarAccess.getPreambleAccess().getPreambleKeyword_0());
            		
            // InternalLinguaFranca.g:1046:3: ( (lv_code_1_0= RULE_CODE ) )
            // InternalLinguaFranca.g:1047:4: (lv_code_1_0= RULE_CODE )
            {
            // InternalLinguaFranca.g:1047:4: (lv_code_1_0= RULE_CODE )
            // InternalLinguaFranca.g:1048:5: lv_code_1_0= RULE_CODE
            {
            lv_code_1_0=(Token)match(input,RULE_CODE,FOLLOW_2); 

            					newLeafNode(lv_code_1_0, grammarAccess.getPreambleAccess().getCodeCODETerminalRuleCall_1_0());
            				

            					if (current==null) {
            						current = createModelElement(grammarAccess.getPreambleRule());
            					}
            					setWithLastConsumed(
            						current,
            						"code",
            						lv_code_1_0,
            						"org.icyphy.LinguaFranca.CODE");
            				

            }


            }


            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "rulePreamble"


    // $ANTLR start "entryRuleInstance"
    // InternalLinguaFranca.g:1068:1: entryRuleInstance returns [EObject current=null] : iv_ruleInstance= ruleInstance EOF ;
    public final EObject entryRuleInstance() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleInstance = null;


        try {
            // InternalLinguaFranca.g:1068:49: (iv_ruleInstance= ruleInstance EOF )
            // InternalLinguaFranca.g:1069:2: iv_ruleInstance= ruleInstance EOF
            {
             newCompositeNode(grammarAccess.getInstanceRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleInstance=ruleInstance();

            state._fsp--;

             current =iv_ruleInstance; 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleInstance"


    // $ANTLR start "ruleInstance"
    // InternalLinguaFranca.g:1075:1: ruleInstance returns [EObject current=null] : ( ( (lv_name_0_0= RULE_ID ) ) otherlv_1= '=' otherlv_2= 'new' ( (lv_actorClass_3_0= RULE_ID ) ) (otherlv_4= '(' ( (lv_parameters_5_0= ruleAssignments ) )? otherlv_6= ')' )? otherlv_7= ';' ) ;
    public final EObject ruleInstance() throws RecognitionException {
        EObject current = null;

        Token lv_name_0_0=null;
        Token otherlv_1=null;
        Token otherlv_2=null;
        Token lv_actorClass_3_0=null;
        Token otherlv_4=null;
        Token otherlv_6=null;
        Token otherlv_7=null;
        EObject lv_parameters_5_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:1081:2: ( ( ( (lv_name_0_0= RULE_ID ) ) otherlv_1= '=' otherlv_2= 'new' ( (lv_actorClass_3_0= RULE_ID ) ) (otherlv_4= '(' ( (lv_parameters_5_0= ruleAssignments ) )? otherlv_6= ')' )? otherlv_7= ';' ) )
            // InternalLinguaFranca.g:1082:2: ( ( (lv_name_0_0= RULE_ID ) ) otherlv_1= '=' otherlv_2= 'new' ( (lv_actorClass_3_0= RULE_ID ) ) (otherlv_4= '(' ( (lv_parameters_5_0= ruleAssignments ) )? otherlv_6= ')' )? otherlv_7= ';' )
            {
            // InternalLinguaFranca.g:1082:2: ( ( (lv_name_0_0= RULE_ID ) ) otherlv_1= '=' otherlv_2= 'new' ( (lv_actorClass_3_0= RULE_ID ) ) (otherlv_4= '(' ( (lv_parameters_5_0= ruleAssignments ) )? otherlv_6= ')' )? otherlv_7= ';' )
            // InternalLinguaFranca.g:1083:3: ( (lv_name_0_0= RULE_ID ) ) otherlv_1= '=' otherlv_2= 'new' ( (lv_actorClass_3_0= RULE_ID ) ) (otherlv_4= '(' ( (lv_parameters_5_0= ruleAssignments ) )? otherlv_6= ')' )? otherlv_7= ';'
            {
            // InternalLinguaFranca.g:1083:3: ( (lv_name_0_0= RULE_ID ) )
            // InternalLinguaFranca.g:1084:4: (lv_name_0_0= RULE_ID )
            {
            // InternalLinguaFranca.g:1084:4: (lv_name_0_0= RULE_ID )
            // InternalLinguaFranca.g:1085:5: lv_name_0_0= RULE_ID
            {
            lv_name_0_0=(Token)match(input,RULE_ID,FOLLOW_28); 

            					newLeafNode(lv_name_0_0, grammarAccess.getInstanceAccess().getNameIDTerminalRuleCall_0_0());
            				

            					if (current==null) {
            						current = createModelElement(grammarAccess.getInstanceRule());
            					}
            					setWithLastConsumed(
            						current,
            						"name",
            						lv_name_0_0,
            						"org.eclipse.xtext.common.Terminals.ID");
            				

            }


            }

            otherlv_1=(Token)match(input,30,FOLLOW_29); 

            			newLeafNode(otherlv_1, grammarAccess.getInstanceAccess().getEqualsSignKeyword_1());
            		
            otherlv_2=(Token)match(input,31,FOLLOW_5); 

            			newLeafNode(otherlv_2, grammarAccess.getInstanceAccess().getNewKeyword_2());
            		
            // InternalLinguaFranca.g:1109:3: ( (lv_actorClass_3_0= RULE_ID ) )
            // InternalLinguaFranca.g:1110:4: (lv_actorClass_3_0= RULE_ID )
            {
            // InternalLinguaFranca.g:1110:4: (lv_actorClass_3_0= RULE_ID )
            // InternalLinguaFranca.g:1111:5: lv_actorClass_3_0= RULE_ID
            {
            lv_actorClass_3_0=(Token)match(input,RULE_ID,FOLLOW_20); 

            					newLeafNode(lv_actorClass_3_0, grammarAccess.getInstanceAccess().getActorClassIDTerminalRuleCall_3_0());
            				

            					if (current==null) {
            						current = createModelElement(grammarAccess.getInstanceRule());
            					}
            					setWithLastConsumed(
            						current,
            						"actorClass",
            						lv_actorClass_3_0,
            						"org.eclipse.xtext.common.Terminals.ID");
            				

            }


            }

            // InternalLinguaFranca.g:1127:3: (otherlv_4= '(' ( (lv_parameters_5_0= ruleAssignments ) )? otherlv_6= ')' )?
            int alt26=2;
            int LA26_0 = input.LA(1);

            if ( (LA26_0==26) ) {
                alt26=1;
            }
            switch (alt26) {
                case 1 :
                    // InternalLinguaFranca.g:1128:4: otherlv_4= '(' ( (lv_parameters_5_0= ruleAssignments ) )? otherlv_6= ')'
                    {
                    otherlv_4=(Token)match(input,26,FOLLOW_23); 

                    				newLeafNode(otherlv_4, grammarAccess.getInstanceAccess().getLeftParenthesisKeyword_4_0());
                    			
                    // InternalLinguaFranca.g:1132:4: ( (lv_parameters_5_0= ruleAssignments ) )?
                    int alt25=2;
                    int LA25_0 = input.LA(1);

                    if ( (LA25_0==RULE_ID) ) {
                        alt25=1;
                    }
                    switch (alt25) {
                        case 1 :
                            // InternalLinguaFranca.g:1133:5: (lv_parameters_5_0= ruleAssignments )
                            {
                            // InternalLinguaFranca.g:1133:5: (lv_parameters_5_0= ruleAssignments )
                            // InternalLinguaFranca.g:1134:6: lv_parameters_5_0= ruleAssignments
                            {

                            						newCompositeNode(grammarAccess.getInstanceAccess().getParametersAssignmentsParserRuleCall_4_1_0());
                            					
                            pushFollow(FOLLOW_30);
                            lv_parameters_5_0=ruleAssignments();

                            state._fsp--;


                            						if (current==null) {
                            							current = createModelElementForParent(grammarAccess.getInstanceRule());
                            						}
                            						set(
                            							current,
                            							"parameters",
                            							lv_parameters_5_0,
                            							"org.icyphy.LinguaFranca.Assignments");
                            						afterParserOrEnumRuleCall();
                            					

                            }


                            }
                            break;

                    }

                    otherlv_6=(Token)match(input,28,FOLLOW_6); 

                    				newLeafNode(otherlv_6, grammarAccess.getInstanceAccess().getRightParenthesisKeyword_4_2());
                    			

                    }
                    break;

            }

            otherlv_7=(Token)match(input,14,FOLLOW_2); 

            			newLeafNode(otherlv_7, grammarAccess.getInstanceAccess().getSemicolonKeyword_5());
            		

            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleInstance"


    // $ANTLR start "entryRuleConnection"
    // InternalLinguaFranca.g:1164:1: entryRuleConnection returns [EObject current=null] : iv_ruleConnection= ruleConnection EOF ;
    public final EObject entryRuleConnection() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleConnection = null;


        try {
            // InternalLinguaFranca.g:1164:51: (iv_ruleConnection= ruleConnection EOF )
            // InternalLinguaFranca.g:1165:2: iv_ruleConnection= ruleConnection EOF
            {
             newCompositeNode(grammarAccess.getConnectionRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleConnection=ruleConnection();

            state._fsp--;

             current =iv_ruleConnection; 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleConnection"


    // $ANTLR start "ruleConnection"
    // InternalLinguaFranca.g:1171:1: ruleConnection returns [EObject current=null] : ( ( (lv_leftPort_0_0= rulePort ) ) otherlv_1= '->' ( (lv_rightPort_2_0= rulePort ) ) otherlv_3= ';' ) ;
    public final EObject ruleConnection() throws RecognitionException {
        EObject current = null;

        Token otherlv_1=null;
        Token otherlv_3=null;
        AntlrDatatypeRuleToken lv_leftPort_0_0 = null;

        AntlrDatatypeRuleToken lv_rightPort_2_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:1177:2: ( ( ( (lv_leftPort_0_0= rulePort ) ) otherlv_1= '->' ( (lv_rightPort_2_0= rulePort ) ) otherlv_3= ';' ) )
            // InternalLinguaFranca.g:1178:2: ( ( (lv_leftPort_0_0= rulePort ) ) otherlv_1= '->' ( (lv_rightPort_2_0= rulePort ) ) otherlv_3= ';' )
            {
            // InternalLinguaFranca.g:1178:2: ( ( (lv_leftPort_0_0= rulePort ) ) otherlv_1= '->' ( (lv_rightPort_2_0= rulePort ) ) otherlv_3= ';' )
            // InternalLinguaFranca.g:1179:3: ( (lv_leftPort_0_0= rulePort ) ) otherlv_1= '->' ( (lv_rightPort_2_0= rulePort ) ) otherlv_3= ';'
            {
            // InternalLinguaFranca.g:1179:3: ( (lv_leftPort_0_0= rulePort ) )
            // InternalLinguaFranca.g:1180:4: (lv_leftPort_0_0= rulePort )
            {
            // InternalLinguaFranca.g:1180:4: (lv_leftPort_0_0= rulePort )
            // InternalLinguaFranca.g:1181:5: lv_leftPort_0_0= rulePort
            {

            					newCompositeNode(grammarAccess.getConnectionAccess().getLeftPortPortParserRuleCall_0_0());
            				
            pushFollow(FOLLOW_31);
            lv_leftPort_0_0=rulePort();

            state._fsp--;


            					if (current==null) {
            						current = createModelElementForParent(grammarAccess.getConnectionRule());
            					}
            					set(
            						current,
            						"leftPort",
            						lv_leftPort_0_0,
            						"org.icyphy.LinguaFranca.Port");
            					afterParserOrEnumRuleCall();
            				

            }


            }

            otherlv_1=(Token)match(input,32,FOLLOW_5); 

            			newLeafNode(otherlv_1, grammarAccess.getConnectionAccess().getHyphenMinusGreaterThanSignKeyword_1());
            		
            // InternalLinguaFranca.g:1202:3: ( (lv_rightPort_2_0= rulePort ) )
            // InternalLinguaFranca.g:1203:4: (lv_rightPort_2_0= rulePort )
            {
            // InternalLinguaFranca.g:1203:4: (lv_rightPort_2_0= rulePort )
            // InternalLinguaFranca.g:1204:5: lv_rightPort_2_0= rulePort
            {

            					newCompositeNode(grammarAccess.getConnectionAccess().getRightPortPortParserRuleCall_2_0());
            				
            pushFollow(FOLLOW_6);
            lv_rightPort_2_0=rulePort();

            state._fsp--;


            					if (current==null) {
            						current = createModelElementForParent(grammarAccess.getConnectionRule());
            					}
            					set(
            						current,
            						"rightPort",
            						lv_rightPort_2_0,
            						"org.icyphy.LinguaFranca.Port");
            					afterParserOrEnumRuleCall();
            				

            }


            }

            otherlv_3=(Token)match(input,14,FOLLOW_2); 

            			newLeafNode(otherlv_3, grammarAccess.getConnectionAccess().getSemicolonKeyword_3());
            		

            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleConnection"


    // $ANTLR start "entryRuleAssignments"
    // InternalLinguaFranca.g:1229:1: entryRuleAssignments returns [EObject current=null] : iv_ruleAssignments= ruleAssignments EOF ;
    public final EObject entryRuleAssignments() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleAssignments = null;


        try {
            // InternalLinguaFranca.g:1229:52: (iv_ruleAssignments= ruleAssignments EOF )
            // InternalLinguaFranca.g:1230:2: iv_ruleAssignments= ruleAssignments EOF
            {
             newCompositeNode(grammarAccess.getAssignmentsRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleAssignments=ruleAssignments();

            state._fsp--;

             current =iv_ruleAssignments; 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleAssignments"


    // $ANTLR start "ruleAssignments"
    // InternalLinguaFranca.g:1236:1: ruleAssignments returns [EObject current=null] : ( ( (lv_assignments_0_0= ruleAssignment ) ) (otherlv_1= ',' ( (lv_assignments_2_0= ruleAssignment ) ) )* ) ;
    public final EObject ruleAssignments() throws RecognitionException {
        EObject current = null;

        Token otherlv_1=null;
        EObject lv_assignments_0_0 = null;

        EObject lv_assignments_2_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:1242:2: ( ( ( (lv_assignments_0_0= ruleAssignment ) ) (otherlv_1= ',' ( (lv_assignments_2_0= ruleAssignment ) ) )* ) )
            // InternalLinguaFranca.g:1243:2: ( ( (lv_assignments_0_0= ruleAssignment ) ) (otherlv_1= ',' ( (lv_assignments_2_0= ruleAssignment ) ) )* )
            {
            // InternalLinguaFranca.g:1243:2: ( ( (lv_assignments_0_0= ruleAssignment ) ) (otherlv_1= ',' ( (lv_assignments_2_0= ruleAssignment ) ) )* )
            // InternalLinguaFranca.g:1244:3: ( (lv_assignments_0_0= ruleAssignment ) ) (otherlv_1= ',' ( (lv_assignments_2_0= ruleAssignment ) ) )*
            {
            // InternalLinguaFranca.g:1244:3: ( (lv_assignments_0_0= ruleAssignment ) )
            // InternalLinguaFranca.g:1245:4: (lv_assignments_0_0= ruleAssignment )
            {
            // InternalLinguaFranca.g:1245:4: (lv_assignments_0_0= ruleAssignment )
            // InternalLinguaFranca.g:1246:5: lv_assignments_0_0= ruleAssignment
            {

            					newCompositeNode(grammarAccess.getAssignmentsAccess().getAssignmentsAssignmentParserRuleCall_0_0());
            				
            pushFollow(FOLLOW_32);
            lv_assignments_0_0=ruleAssignment();

            state._fsp--;


            					if (current==null) {
            						current = createModelElementForParent(grammarAccess.getAssignmentsRule());
            					}
            					add(
            						current,
            						"assignments",
            						lv_assignments_0_0,
            						"org.icyphy.LinguaFranca.Assignment");
            					afterParserOrEnumRuleCall();
            				

            }


            }

            // InternalLinguaFranca.g:1263:3: (otherlv_1= ',' ( (lv_assignments_2_0= ruleAssignment ) ) )*
            loop27:
            do {
                int alt27=2;
                int LA27_0 = input.LA(1);

                if ( (LA27_0==27) ) {
                    alt27=1;
                }


                switch (alt27) {
            	case 1 :
            	    // InternalLinguaFranca.g:1264:4: otherlv_1= ',' ( (lv_assignments_2_0= ruleAssignment ) )
            	    {
            	    otherlv_1=(Token)match(input,27,FOLLOW_5); 

            	    				newLeafNode(otherlv_1, grammarAccess.getAssignmentsAccess().getCommaKeyword_1_0());
            	    			
            	    // InternalLinguaFranca.g:1268:4: ( (lv_assignments_2_0= ruleAssignment ) )
            	    // InternalLinguaFranca.g:1269:5: (lv_assignments_2_0= ruleAssignment )
            	    {
            	    // InternalLinguaFranca.g:1269:5: (lv_assignments_2_0= ruleAssignment )
            	    // InternalLinguaFranca.g:1270:6: lv_assignments_2_0= ruleAssignment
            	    {

            	    						newCompositeNode(grammarAccess.getAssignmentsAccess().getAssignmentsAssignmentParserRuleCall_1_1_0());
            	    					
            	    pushFollow(FOLLOW_32);
            	    lv_assignments_2_0=ruleAssignment();

            	    state._fsp--;


            	    						if (current==null) {
            	    							current = createModelElementForParent(grammarAccess.getAssignmentsRule());
            	    						}
            	    						add(
            	    							current,
            	    							"assignments",
            	    							lv_assignments_2_0,
            	    							"org.icyphy.LinguaFranca.Assignment");
            	    						afterParserOrEnumRuleCall();
            	    					

            	    }


            	    }


            	    }
            	    break;

            	default :
            	    break loop27;
                }
            } while (true);


            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleAssignments"


    // $ANTLR start "entryRuleAssignment"
    // InternalLinguaFranca.g:1292:1: entryRuleAssignment returns [EObject current=null] : iv_ruleAssignment= ruleAssignment EOF ;
    public final EObject entryRuleAssignment() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleAssignment = null;


        try {
            // InternalLinguaFranca.g:1292:51: (iv_ruleAssignment= ruleAssignment EOF )
            // InternalLinguaFranca.g:1293:2: iv_ruleAssignment= ruleAssignment EOF
            {
             newCompositeNode(grammarAccess.getAssignmentRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleAssignment=ruleAssignment();

            state._fsp--;

             current =iv_ruleAssignment; 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleAssignment"


    // $ANTLR start "ruleAssignment"
    // InternalLinguaFranca.g:1299:1: ruleAssignment returns [EObject current=null] : ( ( (lv_name_0_0= RULE_ID ) ) otherlv_1= '=' ( (lv_value_2_0= ruleValue ) ) ) ;
    public final EObject ruleAssignment() throws RecognitionException {
        EObject current = null;

        Token lv_name_0_0=null;
        Token otherlv_1=null;
        AntlrDatatypeRuleToken lv_value_2_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:1305:2: ( ( ( (lv_name_0_0= RULE_ID ) ) otherlv_1= '=' ( (lv_value_2_0= ruleValue ) ) ) )
            // InternalLinguaFranca.g:1306:2: ( ( (lv_name_0_0= RULE_ID ) ) otherlv_1= '=' ( (lv_value_2_0= ruleValue ) ) )
            {
            // InternalLinguaFranca.g:1306:2: ( ( (lv_name_0_0= RULE_ID ) ) otherlv_1= '=' ( (lv_value_2_0= ruleValue ) ) )
            // InternalLinguaFranca.g:1307:3: ( (lv_name_0_0= RULE_ID ) ) otherlv_1= '=' ( (lv_value_2_0= ruleValue ) )
            {
            // InternalLinguaFranca.g:1307:3: ( (lv_name_0_0= RULE_ID ) )
            // InternalLinguaFranca.g:1308:4: (lv_name_0_0= RULE_ID )
            {
            // InternalLinguaFranca.g:1308:4: (lv_name_0_0= RULE_ID )
            // InternalLinguaFranca.g:1309:5: lv_name_0_0= RULE_ID
            {
            lv_name_0_0=(Token)match(input,RULE_ID,FOLLOW_28); 

            					newLeafNode(lv_name_0_0, grammarAccess.getAssignmentAccess().getNameIDTerminalRuleCall_0_0());
            				

            					if (current==null) {
            						current = createModelElement(grammarAccess.getAssignmentRule());
            					}
            					setWithLastConsumed(
            						current,
            						"name",
            						lv_name_0_0,
            						"org.eclipse.xtext.common.Terminals.ID");
            				

            }


            }

            otherlv_1=(Token)match(input,30,FOLLOW_33); 

            			newLeafNode(otherlv_1, grammarAccess.getAssignmentAccess().getEqualsSignKeyword_1());
            		
            // InternalLinguaFranca.g:1329:3: ( (lv_value_2_0= ruleValue ) )
            // InternalLinguaFranca.g:1330:4: (lv_value_2_0= ruleValue )
            {
            // InternalLinguaFranca.g:1330:4: (lv_value_2_0= ruleValue )
            // InternalLinguaFranca.g:1331:5: lv_value_2_0= ruleValue
            {

            					newCompositeNode(grammarAccess.getAssignmentAccess().getValueValueParserRuleCall_2_0());
            				
            pushFollow(FOLLOW_2);
            lv_value_2_0=ruleValue();

            state._fsp--;


            					if (current==null) {
            						current = createModelElementForParent(grammarAccess.getAssignmentRule());
            					}
            					set(
            						current,
            						"value",
            						lv_value_2_0,
            						"org.icyphy.LinguaFranca.Value");
            					afterParserOrEnumRuleCall();
            				

            }


            }


            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleAssignment"


    // $ANTLR start "entryRuleGets"
    // InternalLinguaFranca.g:1352:1: entryRuleGets returns [EObject current=null] : iv_ruleGets= ruleGets EOF ;
    public final EObject entryRuleGets() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleGets = null;


        try {
            // InternalLinguaFranca.g:1352:45: (iv_ruleGets= ruleGets EOF )
            // InternalLinguaFranca.g:1353:2: iv_ruleGets= ruleGets EOF
            {
             newCompositeNode(grammarAccess.getGetsRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleGets=ruleGets();

            state._fsp--;

             current =iv_ruleGets; 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleGets"


    // $ANTLR start "ruleGets"
    // InternalLinguaFranca.g:1359:1: ruleGets returns [EObject current=null] : ( ( (lv_gets_0_0= RULE_ID ) ) (otherlv_1= ',' ( (lv_gets_2_0= RULE_ID ) ) )? ) ;
    public final EObject ruleGets() throws RecognitionException {
        EObject current = null;

        Token lv_gets_0_0=null;
        Token otherlv_1=null;
        Token lv_gets_2_0=null;


        	enterRule();

        try {
            // InternalLinguaFranca.g:1365:2: ( ( ( (lv_gets_0_0= RULE_ID ) ) (otherlv_1= ',' ( (lv_gets_2_0= RULE_ID ) ) )? ) )
            // InternalLinguaFranca.g:1366:2: ( ( (lv_gets_0_0= RULE_ID ) ) (otherlv_1= ',' ( (lv_gets_2_0= RULE_ID ) ) )? )
            {
            // InternalLinguaFranca.g:1366:2: ( ( (lv_gets_0_0= RULE_ID ) ) (otherlv_1= ',' ( (lv_gets_2_0= RULE_ID ) ) )? )
            // InternalLinguaFranca.g:1367:3: ( (lv_gets_0_0= RULE_ID ) ) (otherlv_1= ',' ( (lv_gets_2_0= RULE_ID ) ) )?
            {
            // InternalLinguaFranca.g:1367:3: ( (lv_gets_0_0= RULE_ID ) )
            // InternalLinguaFranca.g:1368:4: (lv_gets_0_0= RULE_ID )
            {
            // InternalLinguaFranca.g:1368:4: (lv_gets_0_0= RULE_ID )
            // InternalLinguaFranca.g:1369:5: lv_gets_0_0= RULE_ID
            {
            lv_gets_0_0=(Token)match(input,RULE_ID,FOLLOW_32); 

            					newLeafNode(lv_gets_0_0, grammarAccess.getGetsAccess().getGetsIDTerminalRuleCall_0_0());
            				

            					if (current==null) {
            						current = createModelElement(grammarAccess.getGetsRule());
            					}
            					addWithLastConsumed(
            						current,
            						"gets",
            						lv_gets_0_0,
            						"org.eclipse.xtext.common.Terminals.ID");
            				

            }


            }

            // InternalLinguaFranca.g:1385:3: (otherlv_1= ',' ( (lv_gets_2_0= RULE_ID ) ) )?
            int alt28=2;
            int LA28_0 = input.LA(1);

            if ( (LA28_0==27) ) {
                alt28=1;
            }
            switch (alt28) {
                case 1 :
                    // InternalLinguaFranca.g:1386:4: otherlv_1= ',' ( (lv_gets_2_0= RULE_ID ) )
                    {
                    otherlv_1=(Token)match(input,27,FOLLOW_5); 

                    				newLeafNode(otherlv_1, grammarAccess.getGetsAccess().getCommaKeyword_1_0());
                    			
                    // InternalLinguaFranca.g:1390:4: ( (lv_gets_2_0= RULE_ID ) )
                    // InternalLinguaFranca.g:1391:5: (lv_gets_2_0= RULE_ID )
                    {
                    // InternalLinguaFranca.g:1391:5: (lv_gets_2_0= RULE_ID )
                    // InternalLinguaFranca.g:1392:6: lv_gets_2_0= RULE_ID
                    {
                    lv_gets_2_0=(Token)match(input,RULE_ID,FOLLOW_2); 

                    						newLeafNode(lv_gets_2_0, grammarAccess.getGetsAccess().getGetsIDTerminalRuleCall_1_1_0());
                    					

                    						if (current==null) {
                    							current = createModelElement(grammarAccess.getGetsRule());
                    						}
                    						addWithLastConsumed(
                    							current,
                    							"gets",
                    							lv_gets_2_0,
                    							"org.eclipse.xtext.common.Terminals.ID");
                    					

                    }


                    }


                    }
                    break;

            }


            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleGets"


    // $ANTLR start "entryRuleParams"
    // InternalLinguaFranca.g:1413:1: entryRuleParams returns [EObject current=null] : iv_ruleParams= ruleParams EOF ;
    public final EObject entryRuleParams() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleParams = null;


        try {
            // InternalLinguaFranca.g:1413:47: (iv_ruleParams= ruleParams EOF )
            // InternalLinguaFranca.g:1414:2: iv_ruleParams= ruleParams EOF
            {
             newCompositeNode(grammarAccess.getParamsRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleParams=ruleParams();

            state._fsp--;

             current =iv_ruleParams; 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleParams"


    // $ANTLR start "ruleParams"
    // InternalLinguaFranca.g:1420:1: ruleParams returns [EObject current=null] : (otherlv_0= '(' ( (lv_params_1_0= ruleParam ) ) (otherlv_2= ',' ( (lv_params_3_0= ruleParam ) ) )* otherlv_4= ')' ) ;
    public final EObject ruleParams() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token otherlv_2=null;
        Token otherlv_4=null;
        EObject lv_params_1_0 = null;

        EObject lv_params_3_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:1426:2: ( (otherlv_0= '(' ( (lv_params_1_0= ruleParam ) ) (otherlv_2= ',' ( (lv_params_3_0= ruleParam ) ) )* otherlv_4= ')' ) )
            // InternalLinguaFranca.g:1427:2: (otherlv_0= '(' ( (lv_params_1_0= ruleParam ) ) (otherlv_2= ',' ( (lv_params_3_0= ruleParam ) ) )* otherlv_4= ')' )
            {
            // InternalLinguaFranca.g:1427:2: (otherlv_0= '(' ( (lv_params_1_0= ruleParam ) ) (otherlv_2= ',' ( (lv_params_3_0= ruleParam ) ) )* otherlv_4= ')' )
            // InternalLinguaFranca.g:1428:3: otherlv_0= '(' ( (lv_params_1_0= ruleParam ) ) (otherlv_2= ',' ( (lv_params_3_0= ruleParam ) ) )* otherlv_4= ')'
            {
            otherlv_0=(Token)match(input,26,FOLLOW_34); 

            			newLeafNode(otherlv_0, grammarAccess.getParamsAccess().getLeftParenthesisKeyword_0());
            		
            // InternalLinguaFranca.g:1432:3: ( (lv_params_1_0= ruleParam ) )
            // InternalLinguaFranca.g:1433:4: (lv_params_1_0= ruleParam )
            {
            // InternalLinguaFranca.g:1433:4: (lv_params_1_0= ruleParam )
            // InternalLinguaFranca.g:1434:5: lv_params_1_0= ruleParam
            {

            					newCompositeNode(grammarAccess.getParamsAccess().getParamsParamParserRuleCall_1_0());
            				
            pushFollow(FOLLOW_24);
            lv_params_1_0=ruleParam();

            state._fsp--;


            					if (current==null) {
            						current = createModelElementForParent(grammarAccess.getParamsRule());
            					}
            					add(
            						current,
            						"params",
            						lv_params_1_0,
            						"org.icyphy.LinguaFranca.Param");
            					afterParserOrEnumRuleCall();
            				

            }


            }

            // InternalLinguaFranca.g:1451:3: (otherlv_2= ',' ( (lv_params_3_0= ruleParam ) ) )*
            loop29:
            do {
                int alt29=2;
                int LA29_0 = input.LA(1);

                if ( (LA29_0==27) ) {
                    alt29=1;
                }


                switch (alt29) {
            	case 1 :
            	    // InternalLinguaFranca.g:1452:4: otherlv_2= ',' ( (lv_params_3_0= ruleParam ) )
            	    {
            	    otherlv_2=(Token)match(input,27,FOLLOW_34); 

            	    				newLeafNode(otherlv_2, grammarAccess.getParamsAccess().getCommaKeyword_2_0());
            	    			
            	    // InternalLinguaFranca.g:1456:4: ( (lv_params_3_0= ruleParam ) )
            	    // InternalLinguaFranca.g:1457:5: (lv_params_3_0= ruleParam )
            	    {
            	    // InternalLinguaFranca.g:1457:5: (lv_params_3_0= ruleParam )
            	    // InternalLinguaFranca.g:1458:6: lv_params_3_0= ruleParam
            	    {

            	    						newCompositeNode(grammarAccess.getParamsAccess().getParamsParamParserRuleCall_2_1_0());
            	    					
            	    pushFollow(FOLLOW_24);
            	    lv_params_3_0=ruleParam();

            	    state._fsp--;


            	    						if (current==null) {
            	    							current = createModelElementForParent(grammarAccess.getParamsRule());
            	    						}
            	    						add(
            	    							current,
            	    							"params",
            	    							lv_params_3_0,
            	    							"org.icyphy.LinguaFranca.Param");
            	    						afterParserOrEnumRuleCall();
            	    					

            	    }


            	    }


            	    }
            	    break;

            	default :
            	    break loop29;
                }
            } while (true);

            otherlv_4=(Token)match(input,28,FOLLOW_2); 

            			newLeafNode(otherlv_4, grammarAccess.getParamsAccess().getRightParenthesisKeyword_3());
            		

            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleParams"


    // $ANTLR start "entryRuleParam"
    // InternalLinguaFranca.g:1484:1: entryRuleParam returns [EObject current=null] : iv_ruleParam= ruleParam EOF ;
    public final EObject entryRuleParam() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleParam = null;


        try {
            // InternalLinguaFranca.g:1484:46: (iv_ruleParam= ruleParam EOF )
            // InternalLinguaFranca.g:1485:2: iv_ruleParam= ruleParam EOF
            {
             newCompositeNode(grammarAccess.getParamRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleParam=ruleParam();

            state._fsp--;

             current =iv_ruleParam; 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleParam"


    // $ANTLR start "ruleParam"
    // InternalLinguaFranca.g:1491:1: ruleParam returns [EObject current=null] : ( (otherlv_0= 'const' )? ( (lv_name_1_0= RULE_ID ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? (otherlv_4= '(' ( (lv_value_5_0= ruleValue ) ) otherlv_6= ')' )? ) ;
    public final EObject ruleParam() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token lv_name_1_0=null;
        Token otherlv_2=null;
        Token otherlv_4=null;
        Token otherlv_6=null;
        AntlrDatatypeRuleToken lv_type_3_0 = null;

        AntlrDatatypeRuleToken lv_value_5_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:1497:2: ( ( (otherlv_0= 'const' )? ( (lv_name_1_0= RULE_ID ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? (otherlv_4= '(' ( (lv_value_5_0= ruleValue ) ) otherlv_6= ')' )? ) )
            // InternalLinguaFranca.g:1498:2: ( (otherlv_0= 'const' )? ( (lv_name_1_0= RULE_ID ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? (otherlv_4= '(' ( (lv_value_5_0= ruleValue ) ) otherlv_6= ')' )? )
            {
            // InternalLinguaFranca.g:1498:2: ( (otherlv_0= 'const' )? ( (lv_name_1_0= RULE_ID ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? (otherlv_4= '(' ( (lv_value_5_0= ruleValue ) ) otherlv_6= ')' )? )
            // InternalLinguaFranca.g:1499:3: (otherlv_0= 'const' )? ( (lv_name_1_0= RULE_ID ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? (otherlv_4= '(' ( (lv_value_5_0= ruleValue ) ) otherlv_6= ')' )?
            {
            // InternalLinguaFranca.g:1499:3: (otherlv_0= 'const' )?
            int alt30=2;
            int LA30_0 = input.LA(1);

            if ( (LA30_0==33) ) {
                alt30=1;
            }
            switch (alt30) {
                case 1 :
                    // InternalLinguaFranca.g:1500:4: otherlv_0= 'const'
                    {
                    otherlv_0=(Token)match(input,33,FOLLOW_5); 

                    				newLeafNode(otherlv_0, grammarAccess.getParamAccess().getConstKeyword_0());
                    			

                    }
                    break;

            }

            // InternalLinguaFranca.g:1505:3: ( (lv_name_1_0= RULE_ID ) )
            // InternalLinguaFranca.g:1506:4: (lv_name_1_0= RULE_ID )
            {
            // InternalLinguaFranca.g:1506:4: (lv_name_1_0= RULE_ID )
            // InternalLinguaFranca.g:1507:5: lv_name_1_0= RULE_ID
            {
            lv_name_1_0=(Token)match(input,RULE_ID,FOLLOW_35); 

            					newLeafNode(lv_name_1_0, grammarAccess.getParamAccess().getNameIDTerminalRuleCall_1_0());
            				

            					if (current==null) {
            						current = createModelElement(grammarAccess.getParamRule());
            					}
            					setWithLastConsumed(
            						current,
            						"name",
            						lv_name_1_0,
            						"org.eclipse.xtext.common.Terminals.ID");
            				

            }


            }

            // InternalLinguaFranca.g:1523:3: (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )?
            int alt31=2;
            int LA31_0 = input.LA(1);

            if ( (LA31_0==21) ) {
                alt31=1;
            }
            switch (alt31) {
                case 1 :
                    // InternalLinguaFranca.g:1524:4: otherlv_2= ':' ( (lv_type_3_0= ruleType ) )
                    {
                    otherlv_2=(Token)match(input,21,FOLLOW_17); 

                    				newLeafNode(otherlv_2, grammarAccess.getParamAccess().getColonKeyword_2_0());
                    			
                    // InternalLinguaFranca.g:1528:4: ( (lv_type_3_0= ruleType ) )
                    // InternalLinguaFranca.g:1529:5: (lv_type_3_0= ruleType )
                    {
                    // InternalLinguaFranca.g:1529:5: (lv_type_3_0= ruleType )
                    // InternalLinguaFranca.g:1530:6: lv_type_3_0= ruleType
                    {

                    						newCompositeNode(grammarAccess.getParamAccess().getTypeTypeParserRuleCall_2_1_0());
                    					
                    pushFollow(FOLLOW_36);
                    lv_type_3_0=ruleType();

                    state._fsp--;


                    						if (current==null) {
                    							current = createModelElementForParent(grammarAccess.getParamRule());
                    						}
                    						set(
                    							current,
                    							"type",
                    							lv_type_3_0,
                    							"org.icyphy.LinguaFranca.Type");
                    						afterParserOrEnumRuleCall();
                    					

                    }


                    }


                    }
                    break;

            }

            // InternalLinguaFranca.g:1548:3: (otherlv_4= '(' ( (lv_value_5_0= ruleValue ) ) otherlv_6= ')' )?
            int alt32=2;
            int LA32_0 = input.LA(1);

            if ( (LA32_0==26) ) {
                alt32=1;
            }
            switch (alt32) {
                case 1 :
                    // InternalLinguaFranca.g:1549:4: otherlv_4= '(' ( (lv_value_5_0= ruleValue ) ) otherlv_6= ')'
                    {
                    otherlv_4=(Token)match(input,26,FOLLOW_33); 

                    				newLeafNode(otherlv_4, grammarAccess.getParamAccess().getLeftParenthesisKeyword_3_0());
                    			
                    // InternalLinguaFranca.g:1553:4: ( (lv_value_5_0= ruleValue ) )
                    // InternalLinguaFranca.g:1554:5: (lv_value_5_0= ruleValue )
                    {
                    // InternalLinguaFranca.g:1554:5: (lv_value_5_0= ruleValue )
                    // InternalLinguaFranca.g:1555:6: lv_value_5_0= ruleValue
                    {

                    						newCompositeNode(grammarAccess.getParamAccess().getValueValueParserRuleCall_3_1_0());
                    					
                    pushFollow(FOLLOW_30);
                    lv_value_5_0=ruleValue();

                    state._fsp--;


                    						if (current==null) {
                    							current = createModelElementForParent(grammarAccess.getParamRule());
                    						}
                    						set(
                    							current,
                    							"value",
                    							lv_value_5_0,
                    							"org.icyphy.LinguaFranca.Value");
                    						afterParserOrEnumRuleCall();
                    					

                    }


                    }

                    otherlv_6=(Token)match(input,28,FOLLOW_2); 

                    				newLeafNode(otherlv_6, grammarAccess.getParamAccess().getRightParenthesisKeyword_3_2());
                    			

                    }
                    break;

            }


            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleParam"


    // $ANTLR start "entryRuleTiming"
    // InternalLinguaFranca.g:1581:1: entryRuleTiming returns [EObject current=null] : iv_ruleTiming= ruleTiming EOF ;
    public final EObject entryRuleTiming() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleTiming = null;


        try {
            // InternalLinguaFranca.g:1581:47: (iv_ruleTiming= ruleTiming EOF )
            // InternalLinguaFranca.g:1582:2: iv_ruleTiming= ruleTiming EOF
            {
             newCompositeNode(grammarAccess.getTimingRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleTiming=ruleTiming();

            state._fsp--;

             current =iv_ruleTiming; 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleTiming"


    // $ANTLR start "ruleTiming"
    // InternalLinguaFranca.g:1588:1: ruleTiming returns [EObject current=null] : (otherlv_0= '(' ( ( (lv_offset_1_1= 'NOW' | lv_offset_1_2= RULE_ID | lv_offset_1_3= RULE_NUMBER ) ) ) (otherlv_2= ',' ( ( (lv_period_3_1= 'ONCE' | lv_period_3_2= 'STOP' | lv_period_3_3= RULE_ID | lv_period_3_4= RULE_NUMBER ) ) ) )? otherlv_4= ')' ) ;
    public final EObject ruleTiming() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token lv_offset_1_1=null;
        Token lv_offset_1_2=null;
        Token lv_offset_1_3=null;
        Token otherlv_2=null;
        Token lv_period_3_1=null;
        Token lv_period_3_2=null;
        Token lv_period_3_3=null;
        Token lv_period_3_4=null;
        Token otherlv_4=null;


        	enterRule();

        try {
            // InternalLinguaFranca.g:1594:2: ( (otherlv_0= '(' ( ( (lv_offset_1_1= 'NOW' | lv_offset_1_2= RULE_ID | lv_offset_1_3= RULE_NUMBER ) ) ) (otherlv_2= ',' ( ( (lv_period_3_1= 'ONCE' | lv_period_3_2= 'STOP' | lv_period_3_3= RULE_ID | lv_period_3_4= RULE_NUMBER ) ) ) )? otherlv_4= ')' ) )
            // InternalLinguaFranca.g:1595:2: (otherlv_0= '(' ( ( (lv_offset_1_1= 'NOW' | lv_offset_1_2= RULE_ID | lv_offset_1_3= RULE_NUMBER ) ) ) (otherlv_2= ',' ( ( (lv_period_3_1= 'ONCE' | lv_period_3_2= 'STOP' | lv_period_3_3= RULE_ID | lv_period_3_4= RULE_NUMBER ) ) ) )? otherlv_4= ')' )
            {
            // InternalLinguaFranca.g:1595:2: (otherlv_0= '(' ( ( (lv_offset_1_1= 'NOW' | lv_offset_1_2= RULE_ID | lv_offset_1_3= RULE_NUMBER ) ) ) (otherlv_2= ',' ( ( (lv_period_3_1= 'ONCE' | lv_period_3_2= 'STOP' | lv_period_3_3= RULE_ID | lv_period_3_4= RULE_NUMBER ) ) ) )? otherlv_4= ')' )
            // InternalLinguaFranca.g:1596:3: otherlv_0= '(' ( ( (lv_offset_1_1= 'NOW' | lv_offset_1_2= RULE_ID | lv_offset_1_3= RULE_NUMBER ) ) ) (otherlv_2= ',' ( ( (lv_period_3_1= 'ONCE' | lv_period_3_2= 'STOP' | lv_period_3_3= RULE_ID | lv_period_3_4= RULE_NUMBER ) ) ) )? otherlv_4= ')'
            {
            otherlv_0=(Token)match(input,26,FOLLOW_37); 

            			newLeafNode(otherlv_0, grammarAccess.getTimingAccess().getLeftParenthesisKeyword_0());
            		
            // InternalLinguaFranca.g:1600:3: ( ( (lv_offset_1_1= 'NOW' | lv_offset_1_2= RULE_ID | lv_offset_1_3= RULE_NUMBER ) ) )
            // InternalLinguaFranca.g:1601:4: ( (lv_offset_1_1= 'NOW' | lv_offset_1_2= RULE_ID | lv_offset_1_3= RULE_NUMBER ) )
            {
            // InternalLinguaFranca.g:1601:4: ( (lv_offset_1_1= 'NOW' | lv_offset_1_2= RULE_ID | lv_offset_1_3= RULE_NUMBER ) )
            // InternalLinguaFranca.g:1602:5: (lv_offset_1_1= 'NOW' | lv_offset_1_2= RULE_ID | lv_offset_1_3= RULE_NUMBER )
            {
            // InternalLinguaFranca.g:1602:5: (lv_offset_1_1= 'NOW' | lv_offset_1_2= RULE_ID | lv_offset_1_3= RULE_NUMBER )
            int alt33=3;
            switch ( input.LA(1) ) {
            case 34:
                {
                alt33=1;
                }
                break;
            case RULE_ID:
                {
                alt33=2;
                }
                break;
            case RULE_NUMBER:
                {
                alt33=3;
                }
                break;
            default:
                NoViableAltException nvae =
                    new NoViableAltException("", 33, 0, input);

                throw nvae;
            }

            switch (alt33) {
                case 1 :
                    // InternalLinguaFranca.g:1603:6: lv_offset_1_1= 'NOW'
                    {
                    lv_offset_1_1=(Token)match(input,34,FOLLOW_24); 

                    						newLeafNode(lv_offset_1_1, grammarAccess.getTimingAccess().getOffsetNOWKeyword_1_0_0());
                    					

                    						if (current==null) {
                    							current = createModelElement(grammarAccess.getTimingRule());
                    						}
                    						setWithLastConsumed(current, "offset", lv_offset_1_1, null);
                    					

                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:1614:6: lv_offset_1_2= RULE_ID
                    {
                    lv_offset_1_2=(Token)match(input,RULE_ID,FOLLOW_24); 

                    						newLeafNode(lv_offset_1_2, grammarAccess.getTimingAccess().getOffsetIDTerminalRuleCall_1_0_1());
                    					

                    						if (current==null) {
                    							current = createModelElement(grammarAccess.getTimingRule());
                    						}
                    						setWithLastConsumed(
                    							current,
                    							"offset",
                    							lv_offset_1_2,
                    							"org.eclipse.xtext.common.Terminals.ID");
                    					

                    }
                    break;
                case 3 :
                    // InternalLinguaFranca.g:1629:6: lv_offset_1_3= RULE_NUMBER
                    {
                    lv_offset_1_3=(Token)match(input,RULE_NUMBER,FOLLOW_24); 

                    						newLeafNode(lv_offset_1_3, grammarAccess.getTimingAccess().getOffsetNUMBERTerminalRuleCall_1_0_2());
                    					

                    						if (current==null) {
                    							current = createModelElement(grammarAccess.getTimingRule());
                    						}
                    						setWithLastConsumed(
                    							current,
                    							"offset",
                    							lv_offset_1_3,
                    							"org.icyphy.LinguaFranca.NUMBER");
                    					

                    }
                    break;

            }


            }


            }

            // InternalLinguaFranca.g:1646:3: (otherlv_2= ',' ( ( (lv_period_3_1= 'ONCE' | lv_period_3_2= 'STOP' | lv_period_3_3= RULE_ID | lv_period_3_4= RULE_NUMBER ) ) ) )?
            int alt35=2;
            int LA35_0 = input.LA(1);

            if ( (LA35_0==27) ) {
                alt35=1;
            }
            switch (alt35) {
                case 1 :
                    // InternalLinguaFranca.g:1647:4: otherlv_2= ',' ( ( (lv_period_3_1= 'ONCE' | lv_period_3_2= 'STOP' | lv_period_3_3= RULE_ID | lv_period_3_4= RULE_NUMBER ) ) )
                    {
                    otherlv_2=(Token)match(input,27,FOLLOW_38); 

                    				newLeafNode(otherlv_2, grammarAccess.getTimingAccess().getCommaKeyword_2_0());
                    			
                    // InternalLinguaFranca.g:1651:4: ( ( (lv_period_3_1= 'ONCE' | lv_period_3_2= 'STOP' | lv_period_3_3= RULE_ID | lv_period_3_4= RULE_NUMBER ) ) )
                    // InternalLinguaFranca.g:1652:5: ( (lv_period_3_1= 'ONCE' | lv_period_3_2= 'STOP' | lv_period_3_3= RULE_ID | lv_period_3_4= RULE_NUMBER ) )
                    {
                    // InternalLinguaFranca.g:1652:5: ( (lv_period_3_1= 'ONCE' | lv_period_3_2= 'STOP' | lv_period_3_3= RULE_ID | lv_period_3_4= RULE_NUMBER ) )
                    // InternalLinguaFranca.g:1653:6: (lv_period_3_1= 'ONCE' | lv_period_3_2= 'STOP' | lv_period_3_3= RULE_ID | lv_period_3_4= RULE_NUMBER )
                    {
                    // InternalLinguaFranca.g:1653:6: (lv_period_3_1= 'ONCE' | lv_period_3_2= 'STOP' | lv_period_3_3= RULE_ID | lv_period_3_4= RULE_NUMBER )
                    int alt34=4;
                    switch ( input.LA(1) ) {
                    case 35:
                        {
                        alt34=1;
                        }
                        break;
                    case 36:
                        {
                        alt34=2;
                        }
                        break;
                    case RULE_ID:
                        {
                        alt34=3;
                        }
                        break;
                    case RULE_NUMBER:
                        {
                        alt34=4;
                        }
                        break;
                    default:
                        NoViableAltException nvae =
                            new NoViableAltException("", 34, 0, input);

                        throw nvae;
                    }

                    switch (alt34) {
                        case 1 :
                            // InternalLinguaFranca.g:1654:7: lv_period_3_1= 'ONCE'
                            {
                            lv_period_3_1=(Token)match(input,35,FOLLOW_30); 

                            							newLeafNode(lv_period_3_1, grammarAccess.getTimingAccess().getPeriodONCEKeyword_2_1_0_0());
                            						

                            							if (current==null) {
                            								current = createModelElement(grammarAccess.getTimingRule());
                            							}
                            							setWithLastConsumed(current, "period", lv_period_3_1, null);
                            						

                            }
                            break;
                        case 2 :
                            // InternalLinguaFranca.g:1665:7: lv_period_3_2= 'STOP'
                            {
                            lv_period_3_2=(Token)match(input,36,FOLLOW_30); 

                            							newLeafNode(lv_period_3_2, grammarAccess.getTimingAccess().getPeriodSTOPKeyword_2_1_0_1());
                            						

                            							if (current==null) {
                            								current = createModelElement(grammarAccess.getTimingRule());
                            							}
                            							setWithLastConsumed(current, "period", lv_period_3_2, null);
                            						

                            }
                            break;
                        case 3 :
                            // InternalLinguaFranca.g:1676:7: lv_period_3_3= RULE_ID
                            {
                            lv_period_3_3=(Token)match(input,RULE_ID,FOLLOW_30); 

                            							newLeafNode(lv_period_3_3, grammarAccess.getTimingAccess().getPeriodIDTerminalRuleCall_2_1_0_2());
                            						

                            							if (current==null) {
                            								current = createModelElement(grammarAccess.getTimingRule());
                            							}
                            							setWithLastConsumed(
                            								current,
                            								"period",
                            								lv_period_3_3,
                            								"org.eclipse.xtext.common.Terminals.ID");
                            						

                            }
                            break;
                        case 4 :
                            // InternalLinguaFranca.g:1691:7: lv_period_3_4= RULE_NUMBER
                            {
                            lv_period_3_4=(Token)match(input,RULE_NUMBER,FOLLOW_30); 

                            							newLeafNode(lv_period_3_4, grammarAccess.getTimingAccess().getPeriodNUMBERTerminalRuleCall_2_1_0_3());
                            						

                            							if (current==null) {
                            								current = createModelElement(grammarAccess.getTimingRule());
                            							}
                            							setWithLastConsumed(
                            								current,
                            								"period",
                            								lv_period_3_4,
                            								"org.icyphy.LinguaFranca.NUMBER");
                            						

                            }
                            break;

                    }


                    }


                    }


                    }
                    break;

            }

            otherlv_4=(Token)match(input,28,FOLLOW_2); 

            			newLeafNode(otherlv_4, grammarAccess.getTimingAccess().getRightParenthesisKeyword_3());
            		

            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleTiming"


    // $ANTLR start "entryRulePort"
    // InternalLinguaFranca.g:1717:1: entryRulePort returns [String current=null] : iv_rulePort= rulePort EOF ;
    public final String entryRulePort() throws RecognitionException {
        String current = null;

        AntlrDatatypeRuleToken iv_rulePort = null;


        try {
            // InternalLinguaFranca.g:1717:44: (iv_rulePort= rulePort EOF )
            // InternalLinguaFranca.g:1718:2: iv_rulePort= rulePort EOF
            {
             newCompositeNode(grammarAccess.getPortRule()); 
            pushFollow(FOLLOW_1);
            iv_rulePort=rulePort();

            state._fsp--;

             current =iv_rulePort.getText(); 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRulePort"


    // $ANTLR start "rulePort"
    // InternalLinguaFranca.g:1724:1: rulePort returns [AntlrDatatypeRuleToken current=new AntlrDatatypeRuleToken()] : (this_ID_0= RULE_ID | (this_ID_1= RULE_ID kw= '.' (this_ID_3= RULE_ID | kw= 'input' | kw= 'output' ) ) ) ;
    public final AntlrDatatypeRuleToken rulePort() throws RecognitionException {
        AntlrDatatypeRuleToken current = new AntlrDatatypeRuleToken();

        Token this_ID_0=null;
        Token this_ID_1=null;
        Token kw=null;
        Token this_ID_3=null;


        	enterRule();

        try {
            // InternalLinguaFranca.g:1730:2: ( (this_ID_0= RULE_ID | (this_ID_1= RULE_ID kw= '.' (this_ID_3= RULE_ID | kw= 'input' | kw= 'output' ) ) ) )
            // InternalLinguaFranca.g:1731:2: (this_ID_0= RULE_ID | (this_ID_1= RULE_ID kw= '.' (this_ID_3= RULE_ID | kw= 'input' | kw= 'output' ) ) )
            {
            // InternalLinguaFranca.g:1731:2: (this_ID_0= RULE_ID | (this_ID_1= RULE_ID kw= '.' (this_ID_3= RULE_ID | kw= 'input' | kw= 'output' ) ) )
            int alt37=2;
            int LA37_0 = input.LA(1);

            if ( (LA37_0==RULE_ID) ) {
                int LA37_1 = input.LA(2);

                if ( (LA37_1==37) ) {
                    alt37=2;
                }
                else if ( (LA37_1==EOF||LA37_1==14||LA37_1==32) ) {
                    alt37=1;
                }
                else {
                    NoViableAltException nvae =
                        new NoViableAltException("", 37, 1, input);

                    throw nvae;
                }
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 37, 0, input);

                throw nvae;
            }
            switch (alt37) {
                case 1 :
                    // InternalLinguaFranca.g:1732:3: this_ID_0= RULE_ID
                    {
                    this_ID_0=(Token)match(input,RULE_ID,FOLLOW_2); 

                    			current.merge(this_ID_0);
                    		

                    			newLeafNode(this_ID_0, grammarAccess.getPortAccess().getIDTerminalRuleCall_0());
                    		

                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:1740:3: (this_ID_1= RULE_ID kw= '.' (this_ID_3= RULE_ID | kw= 'input' | kw= 'output' ) )
                    {
                    // InternalLinguaFranca.g:1740:3: (this_ID_1= RULE_ID kw= '.' (this_ID_3= RULE_ID | kw= 'input' | kw= 'output' ) )
                    // InternalLinguaFranca.g:1741:4: this_ID_1= RULE_ID kw= '.' (this_ID_3= RULE_ID | kw= 'input' | kw= 'output' )
                    {
                    this_ID_1=(Token)match(input,RULE_ID,FOLLOW_39); 

                    				current.merge(this_ID_1);
                    			

                    				newLeafNode(this_ID_1, grammarAccess.getPortAccess().getIDTerminalRuleCall_1_0());
                    			
                    kw=(Token)match(input,37,FOLLOW_40); 

                    				current.merge(kw);
                    				newLeafNode(kw, grammarAccess.getPortAccess().getFullStopKeyword_1_1());
                    			
                    // InternalLinguaFranca.g:1753:4: (this_ID_3= RULE_ID | kw= 'input' | kw= 'output' )
                    int alt36=3;
                    switch ( input.LA(1) ) {
                    case RULE_ID:
                        {
                        alt36=1;
                        }
                        break;
                    case 20:
                        {
                        alt36=2;
                        }
                        break;
                    case 22:
                        {
                        alt36=3;
                        }
                        break;
                    default:
                        NoViableAltException nvae =
                            new NoViableAltException("", 36, 0, input);

                        throw nvae;
                    }

                    switch (alt36) {
                        case 1 :
                            // InternalLinguaFranca.g:1754:5: this_ID_3= RULE_ID
                            {
                            this_ID_3=(Token)match(input,RULE_ID,FOLLOW_2); 

                            					current.merge(this_ID_3);
                            				

                            					newLeafNode(this_ID_3, grammarAccess.getPortAccess().getIDTerminalRuleCall_1_2_0());
                            				

                            }
                            break;
                        case 2 :
                            // InternalLinguaFranca.g:1762:5: kw= 'input'
                            {
                            kw=(Token)match(input,20,FOLLOW_2); 

                            					current.merge(kw);
                            					newLeafNode(kw, grammarAccess.getPortAccess().getInputKeyword_1_2_1());
                            				

                            }
                            break;
                        case 3 :
                            // InternalLinguaFranca.g:1768:5: kw= 'output'
                            {
                            kw=(Token)match(input,22,FOLLOW_2); 

                            					current.merge(kw);
                            					newLeafNode(kw, grammarAccess.getPortAccess().getOutputKeyword_1_2_2());
                            				

                            }
                            break;

                    }


                    }


                    }
                    break;

            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "rulePort"


    // $ANTLR start "entryRuleSets"
    // InternalLinguaFranca.g:1779:1: entryRuleSets returns [EObject current=null] : iv_ruleSets= ruleSets EOF ;
    public final EObject entryRuleSets() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleSets = null;


        try {
            // InternalLinguaFranca.g:1779:45: (iv_ruleSets= ruleSets EOF )
            // InternalLinguaFranca.g:1780:2: iv_ruleSets= ruleSets EOF
            {
             newCompositeNode(grammarAccess.getSetsRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleSets=ruleSets();

            state._fsp--;

             current =iv_ruleSets; 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleSets"


    // $ANTLR start "ruleSets"
    // InternalLinguaFranca.g:1786:1: ruleSets returns [EObject current=null] : (otherlv_0= '->' ( (lv_sets_1_0= RULE_ID ) ) (otherlv_2= ',' ( (lv_sets_3_0= RULE_ID ) ) )? ) ;
    public final EObject ruleSets() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token lv_sets_1_0=null;
        Token otherlv_2=null;
        Token lv_sets_3_0=null;


        	enterRule();

        try {
            // InternalLinguaFranca.g:1792:2: ( (otherlv_0= '->' ( (lv_sets_1_0= RULE_ID ) ) (otherlv_2= ',' ( (lv_sets_3_0= RULE_ID ) ) )? ) )
            // InternalLinguaFranca.g:1793:2: (otherlv_0= '->' ( (lv_sets_1_0= RULE_ID ) ) (otherlv_2= ',' ( (lv_sets_3_0= RULE_ID ) ) )? )
            {
            // InternalLinguaFranca.g:1793:2: (otherlv_0= '->' ( (lv_sets_1_0= RULE_ID ) ) (otherlv_2= ',' ( (lv_sets_3_0= RULE_ID ) ) )? )
            // InternalLinguaFranca.g:1794:3: otherlv_0= '->' ( (lv_sets_1_0= RULE_ID ) ) (otherlv_2= ',' ( (lv_sets_3_0= RULE_ID ) ) )?
            {
            otherlv_0=(Token)match(input,32,FOLLOW_5); 

            			newLeafNode(otherlv_0, grammarAccess.getSetsAccess().getHyphenMinusGreaterThanSignKeyword_0());
            		
            // InternalLinguaFranca.g:1798:3: ( (lv_sets_1_0= RULE_ID ) )
            // InternalLinguaFranca.g:1799:4: (lv_sets_1_0= RULE_ID )
            {
            // InternalLinguaFranca.g:1799:4: (lv_sets_1_0= RULE_ID )
            // InternalLinguaFranca.g:1800:5: lv_sets_1_0= RULE_ID
            {
            lv_sets_1_0=(Token)match(input,RULE_ID,FOLLOW_32); 

            					newLeafNode(lv_sets_1_0, grammarAccess.getSetsAccess().getSetsIDTerminalRuleCall_1_0());
            				

            					if (current==null) {
            						current = createModelElement(grammarAccess.getSetsRule());
            					}
            					addWithLastConsumed(
            						current,
            						"sets",
            						lv_sets_1_0,
            						"org.eclipse.xtext.common.Terminals.ID");
            				

            }


            }

            // InternalLinguaFranca.g:1816:3: (otherlv_2= ',' ( (lv_sets_3_0= RULE_ID ) ) )?
            int alt38=2;
            int LA38_0 = input.LA(1);

            if ( (LA38_0==27) ) {
                alt38=1;
            }
            switch (alt38) {
                case 1 :
                    // InternalLinguaFranca.g:1817:4: otherlv_2= ',' ( (lv_sets_3_0= RULE_ID ) )
                    {
                    otherlv_2=(Token)match(input,27,FOLLOW_5); 

                    				newLeafNode(otherlv_2, grammarAccess.getSetsAccess().getCommaKeyword_2_0());
                    			
                    // InternalLinguaFranca.g:1821:4: ( (lv_sets_3_0= RULE_ID ) )
                    // InternalLinguaFranca.g:1822:5: (lv_sets_3_0= RULE_ID )
                    {
                    // InternalLinguaFranca.g:1822:5: (lv_sets_3_0= RULE_ID )
                    // InternalLinguaFranca.g:1823:6: lv_sets_3_0= RULE_ID
                    {
                    lv_sets_3_0=(Token)match(input,RULE_ID,FOLLOW_2); 

                    						newLeafNode(lv_sets_3_0, grammarAccess.getSetsAccess().getSetsIDTerminalRuleCall_2_1_0());
                    					

                    						if (current==null) {
                    							current = createModelElement(grammarAccess.getSetsRule());
                    						}
                    						addWithLastConsumed(
                    							current,
                    							"sets",
                    							lv_sets_3_0,
                    							"org.eclipse.xtext.common.Terminals.ID");
                    					

                    }


                    }


                    }
                    break;

            }


            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleSets"


    // $ANTLR start "entryRuleType"
    // InternalLinguaFranca.g:1844:1: entryRuleType returns [String current=null] : iv_ruleType= ruleType EOF ;
    public final String entryRuleType() throws RecognitionException {
        String current = null;

        AntlrDatatypeRuleToken iv_ruleType = null;


        try {
            // InternalLinguaFranca.g:1844:44: (iv_ruleType= ruleType EOF )
            // InternalLinguaFranca.g:1845:2: iv_ruleType= ruleType EOF
            {
             newCompositeNode(grammarAccess.getTypeRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleType=ruleType();

            state._fsp--;

             current =iv_ruleType.getText(); 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleType"


    // $ANTLR start "ruleType"
    // InternalLinguaFranca.g:1851:1: ruleType returns [AntlrDatatypeRuleToken current=new AntlrDatatypeRuleToken()] : (this_ID_0= RULE_ID | this_CODE_1= RULE_CODE ) ;
    public final AntlrDatatypeRuleToken ruleType() throws RecognitionException {
        AntlrDatatypeRuleToken current = new AntlrDatatypeRuleToken();

        Token this_ID_0=null;
        Token this_CODE_1=null;


        	enterRule();

        try {
            // InternalLinguaFranca.g:1857:2: ( (this_ID_0= RULE_ID | this_CODE_1= RULE_CODE ) )
            // InternalLinguaFranca.g:1858:2: (this_ID_0= RULE_ID | this_CODE_1= RULE_CODE )
            {
            // InternalLinguaFranca.g:1858:2: (this_ID_0= RULE_ID | this_CODE_1= RULE_CODE )
            int alt39=2;
            int LA39_0 = input.LA(1);

            if ( (LA39_0==RULE_ID) ) {
                alt39=1;
            }
            else if ( (LA39_0==RULE_CODE) ) {
                alt39=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 39, 0, input);

                throw nvae;
            }
            switch (alt39) {
                case 1 :
                    // InternalLinguaFranca.g:1859:3: this_ID_0= RULE_ID
                    {
                    this_ID_0=(Token)match(input,RULE_ID,FOLLOW_2); 

                    			current.merge(this_ID_0);
                    		

                    			newLeafNode(this_ID_0, grammarAccess.getTypeAccess().getIDTerminalRuleCall_0());
                    		

                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:1867:3: this_CODE_1= RULE_CODE
                    {
                    this_CODE_1=(Token)match(input,RULE_CODE,FOLLOW_2); 

                    			current.merge(this_CODE_1);
                    		

                    			newLeafNode(this_CODE_1, grammarAccess.getTypeAccess().getCODETerminalRuleCall_1());
                    		

                    }
                    break;

            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleType"


    // $ANTLR start "entryRuleValue"
    // InternalLinguaFranca.g:1878:1: entryRuleValue returns [String current=null] : iv_ruleValue= ruleValue EOF ;
    public final String entryRuleValue() throws RecognitionException {
        String current = null;

        AntlrDatatypeRuleToken iv_ruleValue = null;


        try {
            // InternalLinguaFranca.g:1878:45: (iv_ruleValue= ruleValue EOF )
            // InternalLinguaFranca.g:1879:2: iv_ruleValue= ruleValue EOF
            {
             newCompositeNode(grammarAccess.getValueRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleValue=ruleValue();

            state._fsp--;

             current =iv_ruleValue.getText(); 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRuleValue"


    // $ANTLR start "ruleValue"
    // InternalLinguaFranca.g:1885:1: ruleValue returns [AntlrDatatypeRuleToken current=new AntlrDatatypeRuleToken()] : (this_ID_0= RULE_ID | this_NUMBER_1= RULE_NUMBER | this_STRING_2= RULE_STRING | this_CODE_3= RULE_CODE ) ;
    public final AntlrDatatypeRuleToken ruleValue() throws RecognitionException {
        AntlrDatatypeRuleToken current = new AntlrDatatypeRuleToken();

        Token this_ID_0=null;
        Token this_NUMBER_1=null;
        Token this_STRING_2=null;
        Token this_CODE_3=null;


        	enterRule();

        try {
            // InternalLinguaFranca.g:1891:2: ( (this_ID_0= RULE_ID | this_NUMBER_1= RULE_NUMBER | this_STRING_2= RULE_STRING | this_CODE_3= RULE_CODE ) )
            // InternalLinguaFranca.g:1892:2: (this_ID_0= RULE_ID | this_NUMBER_1= RULE_NUMBER | this_STRING_2= RULE_STRING | this_CODE_3= RULE_CODE )
            {
            // InternalLinguaFranca.g:1892:2: (this_ID_0= RULE_ID | this_NUMBER_1= RULE_NUMBER | this_STRING_2= RULE_STRING | this_CODE_3= RULE_CODE )
            int alt40=4;
            switch ( input.LA(1) ) {
            case RULE_ID:
                {
                alt40=1;
                }
                break;
            case RULE_NUMBER:
                {
                alt40=2;
                }
                break;
            case RULE_STRING:
                {
                alt40=3;
                }
                break;
            case RULE_CODE:
                {
                alt40=4;
                }
                break;
            default:
                NoViableAltException nvae =
                    new NoViableAltException("", 40, 0, input);

                throw nvae;
            }

            switch (alt40) {
                case 1 :
                    // InternalLinguaFranca.g:1893:3: this_ID_0= RULE_ID
                    {
                    this_ID_0=(Token)match(input,RULE_ID,FOLLOW_2); 

                    			current.merge(this_ID_0);
                    		

                    			newLeafNode(this_ID_0, grammarAccess.getValueAccess().getIDTerminalRuleCall_0());
                    		

                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:1901:3: this_NUMBER_1= RULE_NUMBER
                    {
                    this_NUMBER_1=(Token)match(input,RULE_NUMBER,FOLLOW_2); 

                    			current.merge(this_NUMBER_1);
                    		

                    			newLeafNode(this_NUMBER_1, grammarAccess.getValueAccess().getNUMBERTerminalRuleCall_1());
                    		

                    }
                    break;
                case 3 :
                    // InternalLinguaFranca.g:1909:3: this_STRING_2= RULE_STRING
                    {
                    this_STRING_2=(Token)match(input,RULE_STRING,FOLLOW_2); 

                    			current.merge(this_STRING_2);
                    		

                    			newLeafNode(this_STRING_2, grammarAccess.getValueAccess().getSTRINGTerminalRuleCall_2());
                    		

                    }
                    break;
                case 4 :
                    // InternalLinguaFranca.g:1917:3: this_CODE_3= RULE_CODE
                    {
                    this_CODE_3=(Token)match(input,RULE_CODE,FOLLOW_2); 

                    			current.merge(this_CODE_3);
                    		

                    			newLeafNode(this_CODE_3, grammarAccess.getValueAccess().getCODETerminalRuleCall_3());
                    		

                    }
                    break;

            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "ruleValue"


    // $ANTLR start "entryRulePath"
    // InternalLinguaFranca.g:1928:1: entryRulePath returns [String current=null] : iv_rulePath= rulePath EOF ;
    public final String entryRulePath() throws RecognitionException {
        String current = null;

        AntlrDatatypeRuleToken iv_rulePath = null;


        try {
            // InternalLinguaFranca.g:1928:44: (iv_rulePath= rulePath EOF )
            // InternalLinguaFranca.g:1929:2: iv_rulePath= rulePath EOF
            {
             newCompositeNode(grammarAccess.getPathRule()); 
            pushFollow(FOLLOW_1);
            iv_rulePath=rulePath();

            state._fsp--;

             current =iv_rulePath.getText(); 
            match(input,EOF,FOLLOW_2); 

            }

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "entryRulePath"


    // $ANTLR start "rulePath"
    // InternalLinguaFranca.g:1935:1: rulePath returns [AntlrDatatypeRuleToken current=new AntlrDatatypeRuleToken()] : (this_ID_0= RULE_ID (kw= '.' this_ID_2= RULE_ID )* ) ;
    public final AntlrDatatypeRuleToken rulePath() throws RecognitionException {
        AntlrDatatypeRuleToken current = new AntlrDatatypeRuleToken();

        Token this_ID_0=null;
        Token kw=null;
        Token this_ID_2=null;


        	enterRule();

        try {
            // InternalLinguaFranca.g:1941:2: ( (this_ID_0= RULE_ID (kw= '.' this_ID_2= RULE_ID )* ) )
            // InternalLinguaFranca.g:1942:2: (this_ID_0= RULE_ID (kw= '.' this_ID_2= RULE_ID )* )
            {
            // InternalLinguaFranca.g:1942:2: (this_ID_0= RULE_ID (kw= '.' this_ID_2= RULE_ID )* )
            // InternalLinguaFranca.g:1943:3: this_ID_0= RULE_ID (kw= '.' this_ID_2= RULE_ID )*
            {
            this_ID_0=(Token)match(input,RULE_ID,FOLLOW_41); 

            			current.merge(this_ID_0);
            		

            			newLeafNode(this_ID_0, grammarAccess.getPathAccess().getIDTerminalRuleCall_0());
            		
            // InternalLinguaFranca.g:1950:3: (kw= '.' this_ID_2= RULE_ID )*
            loop41:
            do {
                int alt41=2;
                int LA41_0 = input.LA(1);

                if ( (LA41_0==37) ) {
                    alt41=1;
                }


                switch (alt41) {
            	case 1 :
            	    // InternalLinguaFranca.g:1951:4: kw= '.' this_ID_2= RULE_ID
            	    {
            	    kw=(Token)match(input,37,FOLLOW_5); 

            	    				current.merge(kw);
            	    				newLeafNode(kw, grammarAccess.getPathAccess().getFullStopKeyword_1_0());
            	    			
            	    this_ID_2=(Token)match(input,RULE_ID,FOLLOW_41); 

            	    				current.merge(this_ID_2);
            	    			

            	    				newLeafNode(this_ID_2, grammarAccess.getPathAccess().getIDTerminalRuleCall_1_1());
            	    			

            	    }
            	    break;

            	default :
            	    break loop41;
                }
            } while (true);


            }


            }


            	leaveRule();

        }

            catch (RecognitionException re) {
                recover(input,re);
                appendSkippedTokens();
            }
        finally {
        }
        return current;
    }
    // $ANTLR end "rulePath"

    // Delegated rules


 

    public static final BitSet FOLLOW_1 = new BitSet(new long[]{0x0000000000000000L});
    public static final BitSet FOLLOW_2 = new BitSet(new long[]{0x0000000000000002L});
    public static final BitSet FOLLOW_3 = new BitSet(new long[]{0x0000000000058000L});
    public static final BitSet FOLLOW_4 = new BitSet(new long[]{0x0000000000058002L});
    public static final BitSet FOLLOW_5 = new BitSet(new long[]{0x0000000000000010L});
    public static final BitSet FOLLOW_6 = new BitSet(new long[]{0x0000000000004000L});
    public static final BitSet FOLLOW_7 = new BitSet(new long[]{0x0000000000020000L});
    public static final BitSet FOLLOW_8 = new BitSet(new long[]{0x0000000000020010L});
    public static final BitSet FOLLOW_9 = new BitSet(new long[]{0x0000000004080000L});
    public static final BitSet FOLLOW_10 = new BitSet(new long[]{0x0000000000080000L});
    public static final BitSet FOLLOW_11 = new BitSet(new long[]{0x0000000023D00002L});
    public static final BitSet FOLLOW_12 = new BitSet(new long[]{0x0000000023C00002L});
    public static final BitSet FOLLOW_13 = new BitSet(new long[]{0x0000000023800002L});
    public static final BitSet FOLLOW_14 = new BitSet(new long[]{0x0000000002000002L});
    public static final BitSet FOLLOW_15 = new BitSet(new long[]{0x0000000000100010L});
    public static final BitSet FOLLOW_16 = new BitSet(new long[]{0x0000000000204000L});
    public static final BitSet FOLLOW_17 = new BitSet(new long[]{0x0000000000000030L});
    public static final BitSet FOLLOW_18 = new BitSet(new long[]{0x0000000000400010L});
    public static final BitSet FOLLOW_19 = new BitSet(new long[]{0x0000000000800010L});
    public static final BitSet FOLLOW_20 = new BitSet(new long[]{0x0000000004004000L});
    public static final BitSet FOLLOW_21 = new BitSet(new long[]{0x0000000001000010L});
    public static final BitSet FOLLOW_22 = new BitSet(new long[]{0x0000000104000030L});
    public static final BitSet FOLLOW_23 = new BitSet(new long[]{0x0000000010000010L});
    public static final BitSet FOLLOW_24 = new BitSet(new long[]{0x0000000018000000L});
    public static final BitSet FOLLOW_25 = new BitSet(new long[]{0x0000000100000030L});
    public static final BitSet FOLLOW_26 = new BitSet(new long[]{0x0000000100000020L});
    public static final BitSet FOLLOW_27 = new BitSet(new long[]{0x0000000000000020L});
    public static final BitSet FOLLOW_28 = new BitSet(new long[]{0x0000000040000000L});
    public static final BitSet FOLLOW_29 = new BitSet(new long[]{0x0000000080000000L});
    public static final BitSet FOLLOW_30 = new BitSet(new long[]{0x0000000010000000L});
    public static final BitSet FOLLOW_31 = new BitSet(new long[]{0x0000000100000000L});
    public static final BitSet FOLLOW_32 = new BitSet(new long[]{0x0000000008000002L});
    public static final BitSet FOLLOW_33 = new BitSet(new long[]{0x00000000000000F0L});
    public static final BitSet FOLLOW_34 = new BitSet(new long[]{0x0000000200000010L});
    public static final BitSet FOLLOW_35 = new BitSet(new long[]{0x0000000004200002L});
    public static final BitSet FOLLOW_36 = new BitSet(new long[]{0x0000000004000002L});
    public static final BitSet FOLLOW_37 = new BitSet(new long[]{0x0000000400000050L});
    public static final BitSet FOLLOW_38 = new BitSet(new long[]{0x0000001800000050L});
    public static final BitSet FOLLOW_39 = new BitSet(new long[]{0x0000002000000000L});
    public static final BitSet FOLLOW_40 = new BitSet(new long[]{0x0000000000500010L});
    public static final BitSet FOLLOW_41 = new BitSet(new long[]{0x0000002000000002L});

}