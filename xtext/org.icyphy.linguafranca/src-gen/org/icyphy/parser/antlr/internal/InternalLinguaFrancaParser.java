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
        "<invalid>", "<EOR>", "<DOWN>", "<UP>", "RULE_ID", "RULE_CODE", "RULE_NUMBER", "RULE_STRING", "RULE_INT", "RULE_ML_COMMENT", "RULE_SL_COMMENT", "RULE_WS", "RULE_ANY_OTHER", "'target'", "';'", "'import'", "'reactor'", "'{'", "'}'", "'composite'", "'input'", "':'", "'output'", "'clock'", "'reaction'", "'('", "','", "')'", "'preamble'", "'constructor'", "'='", "'new'", "'->'", "'const'", "'.'"
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
    // InternalLinguaFranca.g:71:1: ruleModel returns [EObject current=null] : ( ( (lv_target_0_0= ruleTarget ) ) ( (lv_imports_1_0= ruleImport ) )* ( ( (lv_blocks_2_1= ruleReactor | lv_blocks_2_2= ruleComposite ) ) )+ ) ;
    public final EObject ruleModel() throws RecognitionException {
        EObject current = null;

        EObject lv_target_0_0 = null;

        EObject lv_imports_1_0 = null;

        EObject lv_blocks_2_1 = null;

        EObject lv_blocks_2_2 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:77:2: ( ( ( (lv_target_0_0= ruleTarget ) ) ( (lv_imports_1_0= ruleImport ) )* ( ( (lv_blocks_2_1= ruleReactor | lv_blocks_2_2= ruleComposite ) ) )+ ) )
            // InternalLinguaFranca.g:78:2: ( ( (lv_target_0_0= ruleTarget ) ) ( (lv_imports_1_0= ruleImport ) )* ( ( (lv_blocks_2_1= ruleReactor | lv_blocks_2_2= ruleComposite ) ) )+ )
            {
            // InternalLinguaFranca.g:78:2: ( ( (lv_target_0_0= ruleTarget ) ) ( (lv_imports_1_0= ruleImport ) )* ( ( (lv_blocks_2_1= ruleReactor | lv_blocks_2_2= ruleComposite ) ) )+ )
            // InternalLinguaFranca.g:79:3: ( (lv_target_0_0= ruleTarget ) ) ( (lv_imports_1_0= ruleImport ) )* ( ( (lv_blocks_2_1= ruleReactor | lv_blocks_2_2= ruleComposite ) ) )+
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

            // InternalLinguaFranca.g:117:3: ( ( (lv_blocks_2_1= ruleReactor | lv_blocks_2_2= ruleComposite ) ) )+
            int cnt3=0;
            loop3:
            do {
                int alt3=2;
                int LA3_0 = input.LA(1);

                if ( (LA3_0==16||LA3_0==19) ) {
                    alt3=1;
                }


                switch (alt3) {
            	case 1 :
            	    // InternalLinguaFranca.g:118:4: ( (lv_blocks_2_1= ruleReactor | lv_blocks_2_2= ruleComposite ) )
            	    {
            	    // InternalLinguaFranca.g:118:4: ( (lv_blocks_2_1= ruleReactor | lv_blocks_2_2= ruleComposite ) )
            	    // InternalLinguaFranca.g:119:5: (lv_blocks_2_1= ruleReactor | lv_blocks_2_2= ruleComposite )
            	    {
            	    // InternalLinguaFranca.g:119:5: (lv_blocks_2_1= ruleReactor | lv_blocks_2_2= ruleComposite )
            	    int alt2=2;
            	    int LA2_0 = input.LA(1);

            	    if ( (LA2_0==16) ) {
            	        alt2=1;
            	    }
            	    else if ( (LA2_0==19) ) {
            	        alt2=2;
            	    }
            	    else {
            	        NoViableAltException nvae =
            	            new NoViableAltException("", 2, 0, input);

            	        throw nvae;
            	    }
            	    switch (alt2) {
            	        case 1 :
            	            // InternalLinguaFranca.g:120:6: lv_blocks_2_1= ruleReactor
            	            {

            	            						newCompositeNode(grammarAccess.getModelAccess().getBlocksReactorParserRuleCall_2_0_0());
            	            					
            	            pushFollow(FOLLOW_4);
            	            lv_blocks_2_1=ruleReactor();

            	            state._fsp--;


            	            						if (current==null) {
            	            							current = createModelElementForParent(grammarAccess.getModelRule());
            	            						}
            	            						add(
            	            							current,
            	            							"blocks",
            	            							lv_blocks_2_1,
            	            							"org.icyphy.LinguaFranca.Reactor");
            	            						afterParserOrEnumRuleCall();
            	            					

            	            }
            	            break;
            	        case 2 :
            	            // InternalLinguaFranca.g:136:6: lv_blocks_2_2= ruleComposite
            	            {

            	            						newCompositeNode(grammarAccess.getModelAccess().getBlocksCompositeParserRuleCall_2_0_1());
            	            					
            	            pushFollow(FOLLOW_4);
            	            lv_blocks_2_2=ruleComposite();

            	            state._fsp--;


            	            						if (current==null) {
            	            							current = createModelElementForParent(grammarAccess.getModelRule());
            	            						}
            	            						add(
            	            							current,
            	            							"blocks",
            	            							lv_blocks_2_2,
            	            							"org.icyphy.LinguaFranca.Composite");
            	            						afterParserOrEnumRuleCall();
            	            					

            	            }
            	            break;

            	    }


            	    }


            	    }
            	    break;

            	default :
            	    if ( cnt3 >= 1 ) break loop3;
                        EarlyExitException eee =
                            new EarlyExitException(3, input);
                        throw eee;
                }
                cnt3++;
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
    // InternalLinguaFranca.g:158:1: entryRuleTarget returns [EObject current=null] : iv_ruleTarget= ruleTarget EOF ;
    public final EObject entryRuleTarget() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleTarget = null;


        try {
            // InternalLinguaFranca.g:158:47: (iv_ruleTarget= ruleTarget EOF )
            // InternalLinguaFranca.g:159:2: iv_ruleTarget= ruleTarget EOF
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
    // InternalLinguaFranca.g:165:1: ruleTarget returns [EObject current=null] : (otherlv_0= 'target' ( (lv_name_1_0= RULE_ID ) ) otherlv_2= ';' ) ;
    public final EObject ruleTarget() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token lv_name_1_0=null;
        Token otherlv_2=null;


        	enterRule();

        try {
            // InternalLinguaFranca.g:171:2: ( (otherlv_0= 'target' ( (lv_name_1_0= RULE_ID ) ) otherlv_2= ';' ) )
            // InternalLinguaFranca.g:172:2: (otherlv_0= 'target' ( (lv_name_1_0= RULE_ID ) ) otherlv_2= ';' )
            {
            // InternalLinguaFranca.g:172:2: (otherlv_0= 'target' ( (lv_name_1_0= RULE_ID ) ) otherlv_2= ';' )
            // InternalLinguaFranca.g:173:3: otherlv_0= 'target' ( (lv_name_1_0= RULE_ID ) ) otherlv_2= ';'
            {
            otherlv_0=(Token)match(input,13,FOLLOW_5); 

            			newLeafNode(otherlv_0, grammarAccess.getTargetAccess().getTargetKeyword_0());
            		
            // InternalLinguaFranca.g:177:3: ( (lv_name_1_0= RULE_ID ) )
            // InternalLinguaFranca.g:178:4: (lv_name_1_0= RULE_ID )
            {
            // InternalLinguaFranca.g:178:4: (lv_name_1_0= RULE_ID )
            // InternalLinguaFranca.g:179:5: lv_name_1_0= RULE_ID
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
    // InternalLinguaFranca.g:203:1: entryRuleImport returns [EObject current=null] : iv_ruleImport= ruleImport EOF ;
    public final EObject entryRuleImport() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleImport = null;


        try {
            // InternalLinguaFranca.g:203:47: (iv_ruleImport= ruleImport EOF )
            // InternalLinguaFranca.g:204:2: iv_ruleImport= ruleImport EOF
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
    // InternalLinguaFranca.g:210:1: ruleImport returns [EObject current=null] : (otherlv_0= 'import' ( (lv_name_1_0= rulePath ) ) otherlv_2= ';' ) ;
    public final EObject ruleImport() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token otherlv_2=null;
        AntlrDatatypeRuleToken lv_name_1_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:216:2: ( (otherlv_0= 'import' ( (lv_name_1_0= rulePath ) ) otherlv_2= ';' ) )
            // InternalLinguaFranca.g:217:2: (otherlv_0= 'import' ( (lv_name_1_0= rulePath ) ) otherlv_2= ';' )
            {
            // InternalLinguaFranca.g:217:2: (otherlv_0= 'import' ( (lv_name_1_0= rulePath ) ) otherlv_2= ';' )
            // InternalLinguaFranca.g:218:3: otherlv_0= 'import' ( (lv_name_1_0= rulePath ) ) otherlv_2= ';'
            {
            otherlv_0=(Token)match(input,15,FOLLOW_5); 

            			newLeafNode(otherlv_0, grammarAccess.getImportAccess().getImportKeyword_0());
            		
            // InternalLinguaFranca.g:222:3: ( (lv_name_1_0= rulePath ) )
            // InternalLinguaFranca.g:223:4: (lv_name_1_0= rulePath )
            {
            // InternalLinguaFranca.g:223:4: (lv_name_1_0= rulePath )
            // InternalLinguaFranca.g:224:5: lv_name_1_0= rulePath
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


    // $ANTLR start "entryRuleReactor"
    // InternalLinguaFranca.g:249:1: entryRuleReactor returns [EObject current=null] : iv_ruleReactor= ruleReactor EOF ;
    public final EObject entryRuleReactor() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleReactor = null;


        try {
            // InternalLinguaFranca.g:249:48: (iv_ruleReactor= ruleReactor EOF )
            // InternalLinguaFranca.g:250:2: iv_ruleReactor= ruleReactor EOF
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
    // InternalLinguaFranca.g:256:1: ruleReactor returns [EObject current=null] : (otherlv_0= 'reactor' ( (lv_name_1_0= RULE_ID ) ) ( (lv_parameters_2_0= ruleParams ) )? otherlv_3= '{' ( (lv_inputs_4_0= ruleInput ) )* ( (lv_outputs_5_0= ruleOutput ) )* ( (lv_clocks_6_0= ruleClock ) )* ( (lv_preamble_7_0= rulePreamble ) )? ( (lv_constructor_8_0= ruleConstructor ) )? ( (lv_reactions_9_0= ruleReaction ) )* otherlv_10= '}' ) ;
    public final EObject ruleReactor() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token lv_name_1_0=null;
        Token otherlv_3=null;
        Token otherlv_10=null;
        EObject lv_parameters_2_0 = null;

        EObject lv_inputs_4_0 = null;

        EObject lv_outputs_5_0 = null;

        EObject lv_clocks_6_0 = null;

        EObject lv_preamble_7_0 = null;

        EObject lv_constructor_8_0 = null;

        EObject lv_reactions_9_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:262:2: ( (otherlv_0= 'reactor' ( (lv_name_1_0= RULE_ID ) ) ( (lv_parameters_2_0= ruleParams ) )? otherlv_3= '{' ( (lv_inputs_4_0= ruleInput ) )* ( (lv_outputs_5_0= ruleOutput ) )* ( (lv_clocks_6_0= ruleClock ) )* ( (lv_preamble_7_0= rulePreamble ) )? ( (lv_constructor_8_0= ruleConstructor ) )? ( (lv_reactions_9_0= ruleReaction ) )* otherlv_10= '}' ) )
            // InternalLinguaFranca.g:263:2: (otherlv_0= 'reactor' ( (lv_name_1_0= RULE_ID ) ) ( (lv_parameters_2_0= ruleParams ) )? otherlv_3= '{' ( (lv_inputs_4_0= ruleInput ) )* ( (lv_outputs_5_0= ruleOutput ) )* ( (lv_clocks_6_0= ruleClock ) )* ( (lv_preamble_7_0= rulePreamble ) )? ( (lv_constructor_8_0= ruleConstructor ) )? ( (lv_reactions_9_0= ruleReaction ) )* otherlv_10= '}' )
            {
            // InternalLinguaFranca.g:263:2: (otherlv_0= 'reactor' ( (lv_name_1_0= RULE_ID ) ) ( (lv_parameters_2_0= ruleParams ) )? otherlv_3= '{' ( (lv_inputs_4_0= ruleInput ) )* ( (lv_outputs_5_0= ruleOutput ) )* ( (lv_clocks_6_0= ruleClock ) )* ( (lv_preamble_7_0= rulePreamble ) )? ( (lv_constructor_8_0= ruleConstructor ) )? ( (lv_reactions_9_0= ruleReaction ) )* otherlv_10= '}' )
            // InternalLinguaFranca.g:264:3: otherlv_0= 'reactor' ( (lv_name_1_0= RULE_ID ) ) ( (lv_parameters_2_0= ruleParams ) )? otherlv_3= '{' ( (lv_inputs_4_0= ruleInput ) )* ( (lv_outputs_5_0= ruleOutput ) )* ( (lv_clocks_6_0= ruleClock ) )* ( (lv_preamble_7_0= rulePreamble ) )? ( (lv_constructor_8_0= ruleConstructor ) )? ( (lv_reactions_9_0= ruleReaction ) )* otherlv_10= '}'
            {
            otherlv_0=(Token)match(input,16,FOLLOW_5); 

            			newLeafNode(otherlv_0, grammarAccess.getReactorAccess().getReactorKeyword_0());
            		
            // InternalLinguaFranca.g:268:3: ( (lv_name_1_0= RULE_ID ) )
            // InternalLinguaFranca.g:269:4: (lv_name_1_0= RULE_ID )
            {
            // InternalLinguaFranca.g:269:4: (lv_name_1_0= RULE_ID )
            // InternalLinguaFranca.g:270:5: lv_name_1_0= RULE_ID
            {
            lv_name_1_0=(Token)match(input,RULE_ID,FOLLOW_7); 

            					newLeafNode(lv_name_1_0, grammarAccess.getReactorAccess().getNameIDTerminalRuleCall_1_0());
            				

            					if (current==null) {
            						current = createModelElement(grammarAccess.getReactorRule());
            					}
            					setWithLastConsumed(
            						current,
            						"name",
            						lv_name_1_0,
            						"org.eclipse.xtext.common.Terminals.ID");
            				

            }


            }

            // InternalLinguaFranca.g:286:3: ( (lv_parameters_2_0= ruleParams ) )?
            int alt4=2;
            int LA4_0 = input.LA(1);

            if ( (LA4_0==25) ) {
                alt4=1;
            }
            switch (alt4) {
                case 1 :
                    // InternalLinguaFranca.g:287:4: (lv_parameters_2_0= ruleParams )
                    {
                    // InternalLinguaFranca.g:287:4: (lv_parameters_2_0= ruleParams )
                    // InternalLinguaFranca.g:288:5: lv_parameters_2_0= ruleParams
                    {

                    					newCompositeNode(grammarAccess.getReactorAccess().getParametersParamsParserRuleCall_2_0());
                    				
                    pushFollow(FOLLOW_8);
                    lv_parameters_2_0=ruleParams();

                    state._fsp--;


                    					if (current==null) {
                    						current = createModelElementForParent(grammarAccess.getReactorRule());
                    					}
                    					set(
                    						current,
                    						"parameters",
                    						lv_parameters_2_0,
                    						"org.icyphy.LinguaFranca.Params");
                    					afterParserOrEnumRuleCall();
                    				

                    }


                    }
                    break;

            }

            otherlv_3=(Token)match(input,17,FOLLOW_9); 

            			newLeafNode(otherlv_3, grammarAccess.getReactorAccess().getLeftCurlyBracketKeyword_3());
            		
            // InternalLinguaFranca.g:309:3: ( (lv_inputs_4_0= ruleInput ) )*
            loop5:
            do {
                int alt5=2;
                int LA5_0 = input.LA(1);

                if ( (LA5_0==20) ) {
                    alt5=1;
                }


                switch (alt5) {
            	case 1 :
            	    // InternalLinguaFranca.g:310:4: (lv_inputs_4_0= ruleInput )
            	    {
            	    // InternalLinguaFranca.g:310:4: (lv_inputs_4_0= ruleInput )
            	    // InternalLinguaFranca.g:311:5: lv_inputs_4_0= ruleInput
            	    {

            	    					newCompositeNode(grammarAccess.getReactorAccess().getInputsInputParserRuleCall_4_0());
            	    				
            	    pushFollow(FOLLOW_9);
            	    lv_inputs_4_0=ruleInput();

            	    state._fsp--;


            	    					if (current==null) {
            	    						current = createModelElementForParent(grammarAccess.getReactorRule());
            	    					}
            	    					add(
            	    						current,
            	    						"inputs",
            	    						lv_inputs_4_0,
            	    						"org.icyphy.LinguaFranca.Input");
            	    					afterParserOrEnumRuleCall();
            	    				

            	    }


            	    }
            	    break;

            	default :
            	    break loop5;
                }
            } while (true);

            // InternalLinguaFranca.g:328:3: ( (lv_outputs_5_0= ruleOutput ) )*
            loop6:
            do {
                int alt6=2;
                int LA6_0 = input.LA(1);

                if ( (LA6_0==22) ) {
                    alt6=1;
                }


                switch (alt6) {
            	case 1 :
            	    // InternalLinguaFranca.g:329:4: (lv_outputs_5_0= ruleOutput )
            	    {
            	    // InternalLinguaFranca.g:329:4: (lv_outputs_5_0= ruleOutput )
            	    // InternalLinguaFranca.g:330:5: lv_outputs_5_0= ruleOutput
            	    {

            	    					newCompositeNode(grammarAccess.getReactorAccess().getOutputsOutputParserRuleCall_5_0());
            	    				
            	    pushFollow(FOLLOW_10);
            	    lv_outputs_5_0=ruleOutput();

            	    state._fsp--;


            	    					if (current==null) {
            	    						current = createModelElementForParent(grammarAccess.getReactorRule());
            	    					}
            	    					add(
            	    						current,
            	    						"outputs",
            	    						lv_outputs_5_0,
            	    						"org.icyphy.LinguaFranca.Output");
            	    					afterParserOrEnumRuleCall();
            	    				

            	    }


            	    }
            	    break;

            	default :
            	    break loop6;
                }
            } while (true);

            // InternalLinguaFranca.g:347:3: ( (lv_clocks_6_0= ruleClock ) )*
            loop7:
            do {
                int alt7=2;
                int LA7_0 = input.LA(1);

                if ( (LA7_0==23) ) {
                    alt7=1;
                }


                switch (alt7) {
            	case 1 :
            	    // InternalLinguaFranca.g:348:4: (lv_clocks_6_0= ruleClock )
            	    {
            	    // InternalLinguaFranca.g:348:4: (lv_clocks_6_0= ruleClock )
            	    // InternalLinguaFranca.g:349:5: lv_clocks_6_0= ruleClock
            	    {

            	    					newCompositeNode(grammarAccess.getReactorAccess().getClocksClockParserRuleCall_6_0());
            	    				
            	    pushFollow(FOLLOW_11);
            	    lv_clocks_6_0=ruleClock();

            	    state._fsp--;


            	    					if (current==null) {
            	    						current = createModelElementForParent(grammarAccess.getReactorRule());
            	    					}
            	    					add(
            	    						current,
            	    						"clocks",
            	    						lv_clocks_6_0,
            	    						"org.icyphy.LinguaFranca.Clock");
            	    					afterParserOrEnumRuleCall();
            	    				

            	    }


            	    }
            	    break;

            	default :
            	    break loop7;
                }
            } while (true);

            // InternalLinguaFranca.g:366:3: ( (lv_preamble_7_0= rulePreamble ) )?
            int alt8=2;
            int LA8_0 = input.LA(1);

            if ( (LA8_0==28) ) {
                alt8=1;
            }
            switch (alt8) {
                case 1 :
                    // InternalLinguaFranca.g:367:4: (lv_preamble_7_0= rulePreamble )
                    {
                    // InternalLinguaFranca.g:367:4: (lv_preamble_7_0= rulePreamble )
                    // InternalLinguaFranca.g:368:5: lv_preamble_7_0= rulePreamble
                    {

                    					newCompositeNode(grammarAccess.getReactorAccess().getPreamblePreambleParserRuleCall_7_0());
                    				
                    pushFollow(FOLLOW_12);
                    lv_preamble_7_0=rulePreamble();

                    state._fsp--;


                    					if (current==null) {
                    						current = createModelElementForParent(grammarAccess.getReactorRule());
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

            // InternalLinguaFranca.g:385:3: ( (lv_constructor_8_0= ruleConstructor ) )?
            int alt9=2;
            int LA9_0 = input.LA(1);

            if ( (LA9_0==29) ) {
                alt9=1;
            }
            switch (alt9) {
                case 1 :
                    // InternalLinguaFranca.g:386:4: (lv_constructor_8_0= ruleConstructor )
                    {
                    // InternalLinguaFranca.g:386:4: (lv_constructor_8_0= ruleConstructor )
                    // InternalLinguaFranca.g:387:5: lv_constructor_8_0= ruleConstructor
                    {

                    					newCompositeNode(grammarAccess.getReactorAccess().getConstructorConstructorParserRuleCall_8_0());
                    				
                    pushFollow(FOLLOW_13);
                    lv_constructor_8_0=ruleConstructor();

                    state._fsp--;


                    					if (current==null) {
                    						current = createModelElementForParent(grammarAccess.getReactorRule());
                    					}
                    					set(
                    						current,
                    						"constructor",
                    						lv_constructor_8_0,
                    						"org.icyphy.LinguaFranca.Constructor");
                    					afterParserOrEnumRuleCall();
                    				

                    }


                    }
                    break;

            }

            // InternalLinguaFranca.g:404:3: ( (lv_reactions_9_0= ruleReaction ) )*
            loop10:
            do {
                int alt10=2;
                int LA10_0 = input.LA(1);

                if ( (LA10_0==24) ) {
                    alt10=1;
                }


                switch (alt10) {
            	case 1 :
            	    // InternalLinguaFranca.g:405:4: (lv_reactions_9_0= ruleReaction )
            	    {
            	    // InternalLinguaFranca.g:405:4: (lv_reactions_9_0= ruleReaction )
            	    // InternalLinguaFranca.g:406:5: lv_reactions_9_0= ruleReaction
            	    {

            	    					newCompositeNode(grammarAccess.getReactorAccess().getReactionsReactionParserRuleCall_9_0());
            	    				
            	    pushFollow(FOLLOW_13);
            	    lv_reactions_9_0=ruleReaction();

            	    state._fsp--;


            	    					if (current==null) {
            	    						current = createModelElementForParent(grammarAccess.getReactorRule());
            	    					}
            	    					add(
            	    						current,
            	    						"reactions",
            	    						lv_reactions_9_0,
            	    						"org.icyphy.LinguaFranca.Reaction");
            	    					afterParserOrEnumRuleCall();
            	    				

            	    }


            	    }
            	    break;

            	default :
            	    break loop10;
                }
            } while (true);

            otherlv_10=(Token)match(input,18,FOLLOW_2); 

            			newLeafNode(otherlv_10, grammarAccess.getReactorAccess().getRightCurlyBracketKeyword_10());
            		

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
    // InternalLinguaFranca.g:431:1: entryRuleComposite returns [EObject current=null] : iv_ruleComposite= ruleComposite EOF ;
    public final EObject entryRuleComposite() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleComposite = null;


        try {
            // InternalLinguaFranca.g:431:50: (iv_ruleComposite= ruleComposite EOF )
            // InternalLinguaFranca.g:432:2: iv_ruleComposite= ruleComposite EOF
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
    // InternalLinguaFranca.g:438:1: ruleComposite returns [EObject current=null] : (otherlv_0= 'composite' ( (lv_name_1_0= RULE_ID ) ) ( (lv_parameters_2_0= ruleParams ) )? otherlv_3= '{' ( (lv_inputs_4_0= ruleInput ) )* ( (lv_outputs_5_0= ruleOutput ) )* ( (lv_clocks_6_0= ruleClock ) )* ( (lv_preamble_7_0= rulePreamble ) )? ( (lv_constructor_8_0= ruleConstructor ) )? ( (lv_reactions_9_0= ruleReaction ) )* ( (lv_instances_10_0= ruleInstance ) )* ( (lv_connections_11_0= ruleConnection ) )* otherlv_12= '}' ) ;
    public final EObject ruleComposite() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token lv_name_1_0=null;
        Token otherlv_3=null;
        Token otherlv_12=null;
        EObject lv_parameters_2_0 = null;

        EObject lv_inputs_4_0 = null;

        EObject lv_outputs_5_0 = null;

        EObject lv_clocks_6_0 = null;

        EObject lv_preamble_7_0 = null;

        EObject lv_constructor_8_0 = null;

        EObject lv_reactions_9_0 = null;

        EObject lv_instances_10_0 = null;

        EObject lv_connections_11_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:444:2: ( (otherlv_0= 'composite' ( (lv_name_1_0= RULE_ID ) ) ( (lv_parameters_2_0= ruleParams ) )? otherlv_3= '{' ( (lv_inputs_4_0= ruleInput ) )* ( (lv_outputs_5_0= ruleOutput ) )* ( (lv_clocks_6_0= ruleClock ) )* ( (lv_preamble_7_0= rulePreamble ) )? ( (lv_constructor_8_0= ruleConstructor ) )? ( (lv_reactions_9_0= ruleReaction ) )* ( (lv_instances_10_0= ruleInstance ) )* ( (lv_connections_11_0= ruleConnection ) )* otherlv_12= '}' ) )
            // InternalLinguaFranca.g:445:2: (otherlv_0= 'composite' ( (lv_name_1_0= RULE_ID ) ) ( (lv_parameters_2_0= ruleParams ) )? otherlv_3= '{' ( (lv_inputs_4_0= ruleInput ) )* ( (lv_outputs_5_0= ruleOutput ) )* ( (lv_clocks_6_0= ruleClock ) )* ( (lv_preamble_7_0= rulePreamble ) )? ( (lv_constructor_8_0= ruleConstructor ) )? ( (lv_reactions_9_0= ruleReaction ) )* ( (lv_instances_10_0= ruleInstance ) )* ( (lv_connections_11_0= ruleConnection ) )* otherlv_12= '}' )
            {
            // InternalLinguaFranca.g:445:2: (otherlv_0= 'composite' ( (lv_name_1_0= RULE_ID ) ) ( (lv_parameters_2_0= ruleParams ) )? otherlv_3= '{' ( (lv_inputs_4_0= ruleInput ) )* ( (lv_outputs_5_0= ruleOutput ) )* ( (lv_clocks_6_0= ruleClock ) )* ( (lv_preamble_7_0= rulePreamble ) )? ( (lv_constructor_8_0= ruleConstructor ) )? ( (lv_reactions_9_0= ruleReaction ) )* ( (lv_instances_10_0= ruleInstance ) )* ( (lv_connections_11_0= ruleConnection ) )* otherlv_12= '}' )
            // InternalLinguaFranca.g:446:3: otherlv_0= 'composite' ( (lv_name_1_0= RULE_ID ) ) ( (lv_parameters_2_0= ruleParams ) )? otherlv_3= '{' ( (lv_inputs_4_0= ruleInput ) )* ( (lv_outputs_5_0= ruleOutput ) )* ( (lv_clocks_6_0= ruleClock ) )* ( (lv_preamble_7_0= rulePreamble ) )? ( (lv_constructor_8_0= ruleConstructor ) )? ( (lv_reactions_9_0= ruleReaction ) )* ( (lv_instances_10_0= ruleInstance ) )* ( (lv_connections_11_0= ruleConnection ) )* otherlv_12= '}'
            {
            otherlv_0=(Token)match(input,19,FOLLOW_5); 

            			newLeafNode(otherlv_0, grammarAccess.getCompositeAccess().getCompositeKeyword_0());
            		
            // InternalLinguaFranca.g:450:3: ( (lv_name_1_0= RULE_ID ) )
            // InternalLinguaFranca.g:451:4: (lv_name_1_0= RULE_ID )
            {
            // InternalLinguaFranca.g:451:4: (lv_name_1_0= RULE_ID )
            // InternalLinguaFranca.g:452:5: lv_name_1_0= RULE_ID
            {
            lv_name_1_0=(Token)match(input,RULE_ID,FOLLOW_7); 

            					newLeafNode(lv_name_1_0, grammarAccess.getCompositeAccess().getNameIDTerminalRuleCall_1_0());
            				

            					if (current==null) {
            						current = createModelElement(grammarAccess.getCompositeRule());
            					}
            					setWithLastConsumed(
            						current,
            						"name",
            						lv_name_1_0,
            						"org.eclipse.xtext.common.Terminals.ID");
            				

            }


            }

            // InternalLinguaFranca.g:468:3: ( (lv_parameters_2_0= ruleParams ) )?
            int alt11=2;
            int LA11_0 = input.LA(1);

            if ( (LA11_0==25) ) {
                alt11=1;
            }
            switch (alt11) {
                case 1 :
                    // InternalLinguaFranca.g:469:4: (lv_parameters_2_0= ruleParams )
                    {
                    // InternalLinguaFranca.g:469:4: (lv_parameters_2_0= ruleParams )
                    // InternalLinguaFranca.g:470:5: lv_parameters_2_0= ruleParams
                    {

                    					newCompositeNode(grammarAccess.getCompositeAccess().getParametersParamsParserRuleCall_2_0());
                    				
                    pushFollow(FOLLOW_8);
                    lv_parameters_2_0=ruleParams();

                    state._fsp--;


                    					if (current==null) {
                    						current = createModelElementForParent(grammarAccess.getCompositeRule());
                    					}
                    					set(
                    						current,
                    						"parameters",
                    						lv_parameters_2_0,
                    						"org.icyphy.LinguaFranca.Params");
                    					afterParserOrEnumRuleCall();
                    				

                    }


                    }
                    break;

            }

            otherlv_3=(Token)match(input,17,FOLLOW_14); 

            			newLeafNode(otherlv_3, grammarAccess.getCompositeAccess().getLeftCurlyBracketKeyword_3());
            		
            // InternalLinguaFranca.g:491:3: ( (lv_inputs_4_0= ruleInput ) )*
            loop12:
            do {
                int alt12=2;
                int LA12_0 = input.LA(1);

                if ( (LA12_0==20) ) {
                    alt12=1;
                }


                switch (alt12) {
            	case 1 :
            	    // InternalLinguaFranca.g:492:4: (lv_inputs_4_0= ruleInput )
            	    {
            	    // InternalLinguaFranca.g:492:4: (lv_inputs_4_0= ruleInput )
            	    // InternalLinguaFranca.g:493:5: lv_inputs_4_0= ruleInput
            	    {

            	    					newCompositeNode(grammarAccess.getCompositeAccess().getInputsInputParserRuleCall_4_0());
            	    				
            	    pushFollow(FOLLOW_14);
            	    lv_inputs_4_0=ruleInput();

            	    state._fsp--;


            	    					if (current==null) {
            	    						current = createModelElementForParent(grammarAccess.getCompositeRule());
            	    					}
            	    					add(
            	    						current,
            	    						"inputs",
            	    						lv_inputs_4_0,
            	    						"org.icyphy.LinguaFranca.Input");
            	    					afterParserOrEnumRuleCall();
            	    				

            	    }


            	    }
            	    break;

            	default :
            	    break loop12;
                }
            } while (true);

            // InternalLinguaFranca.g:510:3: ( (lv_outputs_5_0= ruleOutput ) )*
            loop13:
            do {
                int alt13=2;
                int LA13_0 = input.LA(1);

                if ( (LA13_0==22) ) {
                    alt13=1;
                }


                switch (alt13) {
            	case 1 :
            	    // InternalLinguaFranca.g:511:4: (lv_outputs_5_0= ruleOutput )
            	    {
            	    // InternalLinguaFranca.g:511:4: (lv_outputs_5_0= ruleOutput )
            	    // InternalLinguaFranca.g:512:5: lv_outputs_5_0= ruleOutput
            	    {

            	    					newCompositeNode(grammarAccess.getCompositeAccess().getOutputsOutputParserRuleCall_5_0());
            	    				
            	    pushFollow(FOLLOW_15);
            	    lv_outputs_5_0=ruleOutput();

            	    state._fsp--;


            	    					if (current==null) {
            	    						current = createModelElementForParent(grammarAccess.getCompositeRule());
            	    					}
            	    					add(
            	    						current,
            	    						"outputs",
            	    						lv_outputs_5_0,
            	    						"org.icyphy.LinguaFranca.Output");
            	    					afterParserOrEnumRuleCall();
            	    				

            	    }


            	    }
            	    break;

            	default :
            	    break loop13;
                }
            } while (true);

            // InternalLinguaFranca.g:529:3: ( (lv_clocks_6_0= ruleClock ) )*
            loop14:
            do {
                int alt14=2;
                int LA14_0 = input.LA(1);

                if ( (LA14_0==23) ) {
                    alt14=1;
                }


                switch (alt14) {
            	case 1 :
            	    // InternalLinguaFranca.g:530:4: (lv_clocks_6_0= ruleClock )
            	    {
            	    // InternalLinguaFranca.g:530:4: (lv_clocks_6_0= ruleClock )
            	    // InternalLinguaFranca.g:531:5: lv_clocks_6_0= ruleClock
            	    {

            	    					newCompositeNode(grammarAccess.getCompositeAccess().getClocksClockParserRuleCall_6_0());
            	    				
            	    pushFollow(FOLLOW_16);
            	    lv_clocks_6_0=ruleClock();

            	    state._fsp--;


            	    					if (current==null) {
            	    						current = createModelElementForParent(grammarAccess.getCompositeRule());
            	    					}
            	    					add(
            	    						current,
            	    						"clocks",
            	    						lv_clocks_6_0,
            	    						"org.icyphy.LinguaFranca.Clock");
            	    					afterParserOrEnumRuleCall();
            	    				

            	    }


            	    }
            	    break;

            	default :
            	    break loop14;
                }
            } while (true);

            // InternalLinguaFranca.g:548:3: ( (lv_preamble_7_0= rulePreamble ) )?
            int alt15=2;
            int LA15_0 = input.LA(1);

            if ( (LA15_0==28) ) {
                alt15=1;
            }
            switch (alt15) {
                case 1 :
                    // InternalLinguaFranca.g:549:4: (lv_preamble_7_0= rulePreamble )
                    {
                    // InternalLinguaFranca.g:549:4: (lv_preamble_7_0= rulePreamble )
                    // InternalLinguaFranca.g:550:5: lv_preamble_7_0= rulePreamble
                    {

                    					newCompositeNode(grammarAccess.getCompositeAccess().getPreamblePreambleParserRuleCall_7_0());
                    				
                    pushFollow(FOLLOW_17);
                    lv_preamble_7_0=rulePreamble();

                    state._fsp--;


                    					if (current==null) {
                    						current = createModelElementForParent(grammarAccess.getCompositeRule());
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

            // InternalLinguaFranca.g:567:3: ( (lv_constructor_8_0= ruleConstructor ) )?
            int alt16=2;
            int LA16_0 = input.LA(1);

            if ( (LA16_0==29) ) {
                alt16=1;
            }
            switch (alt16) {
                case 1 :
                    // InternalLinguaFranca.g:568:4: (lv_constructor_8_0= ruleConstructor )
                    {
                    // InternalLinguaFranca.g:568:4: (lv_constructor_8_0= ruleConstructor )
                    // InternalLinguaFranca.g:569:5: lv_constructor_8_0= ruleConstructor
                    {

                    					newCompositeNode(grammarAccess.getCompositeAccess().getConstructorConstructorParserRuleCall_8_0());
                    				
                    pushFollow(FOLLOW_18);
                    lv_constructor_8_0=ruleConstructor();

                    state._fsp--;


                    					if (current==null) {
                    						current = createModelElementForParent(grammarAccess.getCompositeRule());
                    					}
                    					set(
                    						current,
                    						"constructor",
                    						lv_constructor_8_0,
                    						"org.icyphy.LinguaFranca.Constructor");
                    					afterParserOrEnumRuleCall();
                    				

                    }


                    }
                    break;

            }

            // InternalLinguaFranca.g:586:3: ( (lv_reactions_9_0= ruleReaction ) )*
            loop17:
            do {
                int alt17=2;
                int LA17_0 = input.LA(1);

                if ( (LA17_0==24) ) {
                    alt17=1;
                }


                switch (alt17) {
            	case 1 :
            	    // InternalLinguaFranca.g:587:4: (lv_reactions_9_0= ruleReaction )
            	    {
            	    // InternalLinguaFranca.g:587:4: (lv_reactions_9_0= ruleReaction )
            	    // InternalLinguaFranca.g:588:5: lv_reactions_9_0= ruleReaction
            	    {

            	    					newCompositeNode(grammarAccess.getCompositeAccess().getReactionsReactionParserRuleCall_9_0());
            	    				
            	    pushFollow(FOLLOW_18);
            	    lv_reactions_9_0=ruleReaction();

            	    state._fsp--;


            	    					if (current==null) {
            	    						current = createModelElementForParent(grammarAccess.getCompositeRule());
            	    					}
            	    					add(
            	    						current,
            	    						"reactions",
            	    						lv_reactions_9_0,
            	    						"org.icyphy.LinguaFranca.Reaction");
            	    					afterParserOrEnumRuleCall();
            	    				

            	    }


            	    }
            	    break;

            	default :
            	    break loop17;
                }
            } while (true);

            // InternalLinguaFranca.g:605:3: ( (lv_instances_10_0= ruleInstance ) )*
            loop18:
            do {
                int alt18=2;
                int LA18_0 = input.LA(1);

                if ( (LA18_0==RULE_ID) ) {
                    int LA18_1 = input.LA(2);

                    if ( (LA18_1==30) ) {
                        alt18=1;
                    }


                }


                switch (alt18) {
            	case 1 :
            	    // InternalLinguaFranca.g:606:4: (lv_instances_10_0= ruleInstance )
            	    {
            	    // InternalLinguaFranca.g:606:4: (lv_instances_10_0= ruleInstance )
            	    // InternalLinguaFranca.g:607:5: lv_instances_10_0= ruleInstance
            	    {

            	    					newCompositeNode(grammarAccess.getCompositeAccess().getInstancesInstanceParserRuleCall_10_0());
            	    				
            	    pushFollow(FOLLOW_19);
            	    lv_instances_10_0=ruleInstance();

            	    state._fsp--;


            	    					if (current==null) {
            	    						current = createModelElementForParent(grammarAccess.getCompositeRule());
            	    					}
            	    					add(
            	    						current,
            	    						"instances",
            	    						lv_instances_10_0,
            	    						"org.icyphy.LinguaFranca.Instance");
            	    					afterParserOrEnumRuleCall();
            	    				

            	    }


            	    }
            	    break;

            	default :
            	    break loop18;
                }
            } while (true);

            // InternalLinguaFranca.g:624:3: ( (lv_connections_11_0= ruleConnection ) )*
            loop19:
            do {
                int alt19=2;
                int LA19_0 = input.LA(1);

                if ( (LA19_0==RULE_ID) ) {
                    alt19=1;
                }


                switch (alt19) {
            	case 1 :
            	    // InternalLinguaFranca.g:625:4: (lv_connections_11_0= ruleConnection )
            	    {
            	    // InternalLinguaFranca.g:625:4: (lv_connections_11_0= ruleConnection )
            	    // InternalLinguaFranca.g:626:5: lv_connections_11_0= ruleConnection
            	    {

            	    					newCompositeNode(grammarAccess.getCompositeAccess().getConnectionsConnectionParserRuleCall_11_0());
            	    				
            	    pushFollow(FOLLOW_19);
            	    lv_connections_11_0=ruleConnection();

            	    state._fsp--;


            	    					if (current==null) {
            	    						current = createModelElementForParent(grammarAccess.getCompositeRule());
            	    					}
            	    					add(
            	    						current,
            	    						"connections",
            	    						lv_connections_11_0,
            	    						"org.icyphy.LinguaFranca.Connection");
            	    					afterParserOrEnumRuleCall();
            	    				

            	    }


            	    }
            	    break;

            	default :
            	    break loop19;
                }
            } while (true);

            otherlv_12=(Token)match(input,18,FOLLOW_2); 

            			newLeafNode(otherlv_12, grammarAccess.getCompositeAccess().getRightCurlyBracketKeyword_12());
            		

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


    // $ANTLR start "entryRuleInput"
    // InternalLinguaFranca.g:651:1: entryRuleInput returns [EObject current=null] : iv_ruleInput= ruleInput EOF ;
    public final EObject entryRuleInput() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleInput = null;


        try {
            // InternalLinguaFranca.g:651:46: (iv_ruleInput= ruleInput EOF )
            // InternalLinguaFranca.g:652:2: iv_ruleInput= ruleInput EOF
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
    // InternalLinguaFranca.g:658:1: ruleInput returns [EObject current=null] : (otherlv_0= 'input' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'input' ) ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? otherlv_4= ';' ) ;
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
            // InternalLinguaFranca.g:664:2: ( (otherlv_0= 'input' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'input' ) ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? otherlv_4= ';' ) )
            // InternalLinguaFranca.g:665:2: (otherlv_0= 'input' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'input' ) ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? otherlv_4= ';' )
            {
            // InternalLinguaFranca.g:665:2: (otherlv_0= 'input' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'input' ) ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? otherlv_4= ';' )
            // InternalLinguaFranca.g:666:3: otherlv_0= 'input' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'input' ) ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? otherlv_4= ';'
            {
            otherlv_0=(Token)match(input,20,FOLLOW_20); 

            			newLeafNode(otherlv_0, grammarAccess.getInputAccess().getInputKeyword_0());
            		
            // InternalLinguaFranca.g:670:3: ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'input' ) ) )
            // InternalLinguaFranca.g:671:4: ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'input' ) )
            {
            // InternalLinguaFranca.g:671:4: ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'input' ) )
            // InternalLinguaFranca.g:672:5: (lv_name_1_1= RULE_ID | lv_name_1_2= 'input' )
            {
            // InternalLinguaFranca.g:672:5: (lv_name_1_1= RULE_ID | lv_name_1_2= 'input' )
            int alt20=2;
            int LA20_0 = input.LA(1);

            if ( (LA20_0==RULE_ID) ) {
                alt20=1;
            }
            else if ( (LA20_0==20) ) {
                alt20=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 20, 0, input);

                throw nvae;
            }
            switch (alt20) {
                case 1 :
                    // InternalLinguaFranca.g:673:6: lv_name_1_1= RULE_ID
                    {
                    lv_name_1_1=(Token)match(input,RULE_ID,FOLLOW_21); 

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
                    // InternalLinguaFranca.g:688:6: lv_name_1_2= 'input'
                    {
                    lv_name_1_2=(Token)match(input,20,FOLLOW_21); 

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

            // InternalLinguaFranca.g:701:3: (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )?
            int alt21=2;
            int LA21_0 = input.LA(1);

            if ( (LA21_0==21) ) {
                alt21=1;
            }
            switch (alt21) {
                case 1 :
                    // InternalLinguaFranca.g:702:4: otherlv_2= ':' ( (lv_type_3_0= ruleType ) )
                    {
                    otherlv_2=(Token)match(input,21,FOLLOW_22); 

                    				newLeafNode(otherlv_2, grammarAccess.getInputAccess().getColonKeyword_2_0());
                    			
                    // InternalLinguaFranca.g:706:4: ( (lv_type_3_0= ruleType ) )
                    // InternalLinguaFranca.g:707:5: (lv_type_3_0= ruleType )
                    {
                    // InternalLinguaFranca.g:707:5: (lv_type_3_0= ruleType )
                    // InternalLinguaFranca.g:708:6: lv_type_3_0= ruleType
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
    // InternalLinguaFranca.g:734:1: entryRuleOutput returns [EObject current=null] : iv_ruleOutput= ruleOutput EOF ;
    public final EObject entryRuleOutput() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleOutput = null;


        try {
            // InternalLinguaFranca.g:734:47: (iv_ruleOutput= ruleOutput EOF )
            // InternalLinguaFranca.g:735:2: iv_ruleOutput= ruleOutput EOF
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
    // InternalLinguaFranca.g:741:1: ruleOutput returns [EObject current=null] : (otherlv_0= 'output' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'output' ) ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? otherlv_4= ';' ) ;
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
            // InternalLinguaFranca.g:747:2: ( (otherlv_0= 'output' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'output' ) ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? otherlv_4= ';' ) )
            // InternalLinguaFranca.g:748:2: (otherlv_0= 'output' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'output' ) ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? otherlv_4= ';' )
            {
            // InternalLinguaFranca.g:748:2: (otherlv_0= 'output' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'output' ) ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? otherlv_4= ';' )
            // InternalLinguaFranca.g:749:3: otherlv_0= 'output' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'output' ) ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? otherlv_4= ';'
            {
            otherlv_0=(Token)match(input,22,FOLLOW_23); 

            			newLeafNode(otherlv_0, grammarAccess.getOutputAccess().getOutputKeyword_0());
            		
            // InternalLinguaFranca.g:753:3: ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'output' ) ) )
            // InternalLinguaFranca.g:754:4: ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'output' ) )
            {
            // InternalLinguaFranca.g:754:4: ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'output' ) )
            // InternalLinguaFranca.g:755:5: (lv_name_1_1= RULE_ID | lv_name_1_2= 'output' )
            {
            // InternalLinguaFranca.g:755:5: (lv_name_1_1= RULE_ID | lv_name_1_2= 'output' )
            int alt22=2;
            int LA22_0 = input.LA(1);

            if ( (LA22_0==RULE_ID) ) {
                alt22=1;
            }
            else if ( (LA22_0==22) ) {
                alt22=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 22, 0, input);

                throw nvae;
            }
            switch (alt22) {
                case 1 :
                    // InternalLinguaFranca.g:756:6: lv_name_1_1= RULE_ID
                    {
                    lv_name_1_1=(Token)match(input,RULE_ID,FOLLOW_21); 

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
                    // InternalLinguaFranca.g:771:6: lv_name_1_2= 'output'
                    {
                    lv_name_1_2=(Token)match(input,22,FOLLOW_21); 

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

            // InternalLinguaFranca.g:784:3: (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )?
            int alt23=2;
            int LA23_0 = input.LA(1);

            if ( (LA23_0==21) ) {
                alt23=1;
            }
            switch (alt23) {
                case 1 :
                    // InternalLinguaFranca.g:785:4: otherlv_2= ':' ( (lv_type_3_0= ruleType ) )
                    {
                    otherlv_2=(Token)match(input,21,FOLLOW_22); 

                    				newLeafNode(otherlv_2, grammarAccess.getOutputAccess().getColonKeyword_2_0());
                    			
                    // InternalLinguaFranca.g:789:4: ( (lv_type_3_0= ruleType ) )
                    // InternalLinguaFranca.g:790:5: (lv_type_3_0= ruleType )
                    {
                    // InternalLinguaFranca.g:790:5: (lv_type_3_0= ruleType )
                    // InternalLinguaFranca.g:791:6: lv_type_3_0= ruleType
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


    // $ANTLR start "entryRuleClock"
    // InternalLinguaFranca.g:817:1: entryRuleClock returns [EObject current=null] : iv_ruleClock= ruleClock EOF ;
    public final EObject entryRuleClock() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleClock = null;


        try {
            // InternalLinguaFranca.g:817:46: (iv_ruleClock= ruleClock EOF )
            // InternalLinguaFranca.g:818:2: iv_ruleClock= ruleClock EOF
            {
             newCompositeNode(grammarAccess.getClockRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleClock=ruleClock();

            state._fsp--;

             current =iv_ruleClock; 
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
    // $ANTLR end "entryRuleClock"


    // $ANTLR start "ruleClock"
    // InternalLinguaFranca.g:824:1: ruleClock returns [EObject current=null] : (otherlv_0= 'clock' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'clock' ) ) ) ( (lv_period_2_0= rulePeriod ) )? otherlv_3= ';' ) ;
    public final EObject ruleClock() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token lv_name_1_1=null;
        Token lv_name_1_2=null;
        Token otherlv_3=null;
        EObject lv_period_2_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:830:2: ( (otherlv_0= 'clock' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'clock' ) ) ) ( (lv_period_2_0= rulePeriod ) )? otherlv_3= ';' ) )
            // InternalLinguaFranca.g:831:2: (otherlv_0= 'clock' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'clock' ) ) ) ( (lv_period_2_0= rulePeriod ) )? otherlv_3= ';' )
            {
            // InternalLinguaFranca.g:831:2: (otherlv_0= 'clock' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'clock' ) ) ) ( (lv_period_2_0= rulePeriod ) )? otherlv_3= ';' )
            // InternalLinguaFranca.g:832:3: otherlv_0= 'clock' ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'clock' ) ) ) ( (lv_period_2_0= rulePeriod ) )? otherlv_3= ';'
            {
            otherlv_0=(Token)match(input,23,FOLLOW_24); 

            			newLeafNode(otherlv_0, grammarAccess.getClockAccess().getClockKeyword_0());
            		
            // InternalLinguaFranca.g:836:3: ( ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'clock' ) ) )
            // InternalLinguaFranca.g:837:4: ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'clock' ) )
            {
            // InternalLinguaFranca.g:837:4: ( (lv_name_1_1= RULE_ID | lv_name_1_2= 'clock' ) )
            // InternalLinguaFranca.g:838:5: (lv_name_1_1= RULE_ID | lv_name_1_2= 'clock' )
            {
            // InternalLinguaFranca.g:838:5: (lv_name_1_1= RULE_ID | lv_name_1_2= 'clock' )
            int alt24=2;
            int LA24_0 = input.LA(1);

            if ( (LA24_0==RULE_ID) ) {
                alt24=1;
            }
            else if ( (LA24_0==23) ) {
                alt24=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 24, 0, input);

                throw nvae;
            }
            switch (alt24) {
                case 1 :
                    // InternalLinguaFranca.g:839:6: lv_name_1_1= RULE_ID
                    {
                    lv_name_1_1=(Token)match(input,RULE_ID,FOLLOW_25); 

                    						newLeafNode(lv_name_1_1, grammarAccess.getClockAccess().getNameIDTerminalRuleCall_1_0_0());
                    					

                    						if (current==null) {
                    							current = createModelElement(grammarAccess.getClockRule());
                    						}
                    						setWithLastConsumed(
                    							current,
                    							"name",
                    							lv_name_1_1,
                    							"org.eclipse.xtext.common.Terminals.ID");
                    					

                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:854:6: lv_name_1_2= 'clock'
                    {
                    lv_name_1_2=(Token)match(input,23,FOLLOW_25); 

                    						newLeafNode(lv_name_1_2, grammarAccess.getClockAccess().getNameClockKeyword_1_0_1());
                    					

                    						if (current==null) {
                    							current = createModelElement(grammarAccess.getClockRule());
                    						}
                    						setWithLastConsumed(current, "name", lv_name_1_2, null);
                    					

                    }
                    break;

            }


            }


            }

            // InternalLinguaFranca.g:867:3: ( (lv_period_2_0= rulePeriod ) )?
            int alt25=2;
            int LA25_0 = input.LA(1);

            if ( (LA25_0==25) ) {
                alt25=1;
            }
            switch (alt25) {
                case 1 :
                    // InternalLinguaFranca.g:868:4: (lv_period_2_0= rulePeriod )
                    {
                    // InternalLinguaFranca.g:868:4: (lv_period_2_0= rulePeriod )
                    // InternalLinguaFranca.g:869:5: lv_period_2_0= rulePeriod
                    {

                    					newCompositeNode(grammarAccess.getClockAccess().getPeriodPeriodParserRuleCall_2_0());
                    				
                    pushFollow(FOLLOW_6);
                    lv_period_2_0=rulePeriod();

                    state._fsp--;


                    					if (current==null) {
                    						current = createModelElementForParent(grammarAccess.getClockRule());
                    					}
                    					set(
                    						current,
                    						"period",
                    						lv_period_2_0,
                    						"org.icyphy.LinguaFranca.Period");
                    					afterParserOrEnumRuleCall();
                    				

                    }


                    }
                    break;

            }

            otherlv_3=(Token)match(input,14,FOLLOW_2); 

            			newLeafNode(otherlv_3, grammarAccess.getClockAccess().getSemicolonKeyword_3());
            		

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
    // $ANTLR end "ruleClock"


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
            otherlv_0=(Token)match(input,24,FOLLOW_26); 

            			newLeafNode(otherlv_0, grammarAccess.getReactionAccess().getReactionKeyword_0());
            		
            // InternalLinguaFranca.g:913:3: (otherlv_1= '(' ( ( (lv_triggers_2_0= RULE_ID ) ) (otherlv_3= ',' ( (lv_triggers_4_0= RULE_ID ) ) )* )? otherlv_5= ')' )?
            int alt28=2;
            int LA28_0 = input.LA(1);

            if ( (LA28_0==25) ) {
                alt28=1;
            }
            switch (alt28) {
                case 1 :
                    // InternalLinguaFranca.g:914:4: otherlv_1= '(' ( ( (lv_triggers_2_0= RULE_ID ) ) (otherlv_3= ',' ( (lv_triggers_4_0= RULE_ID ) ) )* )? otherlv_5= ')'
                    {
                    otherlv_1=(Token)match(input,25,FOLLOW_27); 

                    				newLeafNode(otherlv_1, grammarAccess.getReactionAccess().getLeftParenthesisKeyword_1_0());
                    			
                    // InternalLinguaFranca.g:918:4: ( ( (lv_triggers_2_0= RULE_ID ) ) (otherlv_3= ',' ( (lv_triggers_4_0= RULE_ID ) ) )* )?
                    int alt27=2;
                    int LA27_0 = input.LA(1);

                    if ( (LA27_0==RULE_ID) ) {
                        alt27=1;
                    }
                    switch (alt27) {
                        case 1 :
                            // InternalLinguaFranca.g:919:5: ( (lv_triggers_2_0= RULE_ID ) ) (otherlv_3= ',' ( (lv_triggers_4_0= RULE_ID ) ) )*
                            {
                            // InternalLinguaFranca.g:919:5: ( (lv_triggers_2_0= RULE_ID ) )
                            // InternalLinguaFranca.g:920:6: (lv_triggers_2_0= RULE_ID )
                            {
                            // InternalLinguaFranca.g:920:6: (lv_triggers_2_0= RULE_ID )
                            // InternalLinguaFranca.g:921:7: lv_triggers_2_0= RULE_ID
                            {
                            lv_triggers_2_0=(Token)match(input,RULE_ID,FOLLOW_28); 

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
                            loop26:
                            do {
                                int alt26=2;
                                int LA26_0 = input.LA(1);

                                if ( (LA26_0==26) ) {
                                    alt26=1;
                                }


                                switch (alt26) {
                            	case 1 :
                            	    // InternalLinguaFranca.g:938:6: otherlv_3= ',' ( (lv_triggers_4_0= RULE_ID ) )
                            	    {
                            	    otherlv_3=(Token)match(input,26,FOLLOW_5); 

                            	    						newLeafNode(otherlv_3, grammarAccess.getReactionAccess().getCommaKeyword_1_1_1_0());
                            	    					
                            	    // InternalLinguaFranca.g:942:6: ( (lv_triggers_4_0= RULE_ID ) )
                            	    // InternalLinguaFranca.g:943:7: (lv_triggers_4_0= RULE_ID )
                            	    {
                            	    // InternalLinguaFranca.g:943:7: (lv_triggers_4_0= RULE_ID )
                            	    // InternalLinguaFranca.g:944:8: lv_triggers_4_0= RULE_ID
                            	    {
                            	    lv_triggers_4_0=(Token)match(input,RULE_ID,FOLLOW_28); 

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
                            	    break loop26;
                                }
                            } while (true);


                            }
                            break;

                    }

                    otherlv_5=(Token)match(input,27,FOLLOW_29); 

                    				newLeafNode(otherlv_5, grammarAccess.getReactionAccess().getRightParenthesisKeyword_1_2());
                    			

                    }
                    break;

            }

            // InternalLinguaFranca.g:967:3: ( (lv_gets_6_0= ruleGets ) )?
            int alt29=2;
            int LA29_0 = input.LA(1);

            if ( (LA29_0==RULE_ID) ) {
                alt29=1;
            }
            switch (alt29) {
                case 1 :
                    // InternalLinguaFranca.g:968:4: (lv_gets_6_0= ruleGets )
                    {
                    // InternalLinguaFranca.g:968:4: (lv_gets_6_0= ruleGets )
                    // InternalLinguaFranca.g:969:5: lv_gets_6_0= ruleGets
                    {

                    					newCompositeNode(grammarAccess.getReactionAccess().getGetsGetsParserRuleCall_2_0());
                    				
                    pushFollow(FOLLOW_30);
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
            int alt30=2;
            int LA30_0 = input.LA(1);

            if ( (LA30_0==32) ) {
                alt30=1;
            }
            switch (alt30) {
                case 1 :
                    // InternalLinguaFranca.g:987:4: (lv_sets_7_0= ruleSets )
                    {
                    // InternalLinguaFranca.g:987:4: (lv_sets_7_0= ruleSets )
                    // InternalLinguaFranca.g:988:5: lv_sets_7_0= ruleSets
                    {

                    					newCompositeNode(grammarAccess.getReactionAccess().getSetsSetsParserRuleCall_3_0());
                    				
                    pushFollow(FOLLOW_31);
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
            otherlv_0=(Token)match(input,28,FOLLOW_31); 

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


    // $ANTLR start "entryRuleConstructor"
    // InternalLinguaFranca.g:1068:1: entryRuleConstructor returns [EObject current=null] : iv_ruleConstructor= ruleConstructor EOF ;
    public final EObject entryRuleConstructor() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleConstructor = null;


        try {
            // InternalLinguaFranca.g:1068:52: (iv_ruleConstructor= ruleConstructor EOF )
            // InternalLinguaFranca.g:1069:2: iv_ruleConstructor= ruleConstructor EOF
            {
             newCompositeNode(grammarAccess.getConstructorRule()); 
            pushFollow(FOLLOW_1);
            iv_ruleConstructor=ruleConstructor();

            state._fsp--;

             current =iv_ruleConstructor; 
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
    // $ANTLR end "entryRuleConstructor"


    // $ANTLR start "ruleConstructor"
    // InternalLinguaFranca.g:1075:1: ruleConstructor returns [EObject current=null] : (otherlv_0= 'constructor' ( (lv_code_1_0= RULE_CODE ) ) ) ;
    public final EObject ruleConstructor() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token lv_code_1_0=null;


        	enterRule();

        try {
            // InternalLinguaFranca.g:1081:2: ( (otherlv_0= 'constructor' ( (lv_code_1_0= RULE_CODE ) ) ) )
            // InternalLinguaFranca.g:1082:2: (otherlv_0= 'constructor' ( (lv_code_1_0= RULE_CODE ) ) )
            {
            // InternalLinguaFranca.g:1082:2: (otherlv_0= 'constructor' ( (lv_code_1_0= RULE_CODE ) ) )
            // InternalLinguaFranca.g:1083:3: otherlv_0= 'constructor' ( (lv_code_1_0= RULE_CODE ) )
            {
            otherlv_0=(Token)match(input,29,FOLLOW_31); 

            			newLeafNode(otherlv_0, grammarAccess.getConstructorAccess().getConstructorKeyword_0());
            		
            // InternalLinguaFranca.g:1087:3: ( (lv_code_1_0= RULE_CODE ) )
            // InternalLinguaFranca.g:1088:4: (lv_code_1_0= RULE_CODE )
            {
            // InternalLinguaFranca.g:1088:4: (lv_code_1_0= RULE_CODE )
            // InternalLinguaFranca.g:1089:5: lv_code_1_0= RULE_CODE
            {
            lv_code_1_0=(Token)match(input,RULE_CODE,FOLLOW_2); 

            					newLeafNode(lv_code_1_0, grammarAccess.getConstructorAccess().getCodeCODETerminalRuleCall_1_0());
            				

            					if (current==null) {
            						current = createModelElement(grammarAccess.getConstructorRule());
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
    // $ANTLR end "ruleConstructor"


    // $ANTLR start "entryRuleInstance"
    // InternalLinguaFranca.g:1109:1: entryRuleInstance returns [EObject current=null] : iv_ruleInstance= ruleInstance EOF ;
    public final EObject entryRuleInstance() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleInstance = null;


        try {
            // InternalLinguaFranca.g:1109:49: (iv_ruleInstance= ruleInstance EOF )
            // InternalLinguaFranca.g:1110:2: iv_ruleInstance= ruleInstance EOF
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
    // InternalLinguaFranca.g:1116:1: ruleInstance returns [EObject current=null] : ( ( (lv_name_0_0= RULE_ID ) ) otherlv_1= '=' otherlv_2= 'new' ( (lv_actorClass_3_0= RULE_ID ) ) (otherlv_4= '(' ( (lv_parameters_5_0= ruleAssignments ) )? otherlv_6= ')' )? otherlv_7= ';' ) ;
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
            // InternalLinguaFranca.g:1122:2: ( ( ( (lv_name_0_0= RULE_ID ) ) otherlv_1= '=' otherlv_2= 'new' ( (lv_actorClass_3_0= RULE_ID ) ) (otherlv_4= '(' ( (lv_parameters_5_0= ruleAssignments ) )? otherlv_6= ')' )? otherlv_7= ';' ) )
            // InternalLinguaFranca.g:1123:2: ( ( (lv_name_0_0= RULE_ID ) ) otherlv_1= '=' otherlv_2= 'new' ( (lv_actorClass_3_0= RULE_ID ) ) (otherlv_4= '(' ( (lv_parameters_5_0= ruleAssignments ) )? otherlv_6= ')' )? otherlv_7= ';' )
            {
            // InternalLinguaFranca.g:1123:2: ( ( (lv_name_0_0= RULE_ID ) ) otherlv_1= '=' otherlv_2= 'new' ( (lv_actorClass_3_0= RULE_ID ) ) (otherlv_4= '(' ( (lv_parameters_5_0= ruleAssignments ) )? otherlv_6= ')' )? otherlv_7= ';' )
            // InternalLinguaFranca.g:1124:3: ( (lv_name_0_0= RULE_ID ) ) otherlv_1= '=' otherlv_2= 'new' ( (lv_actorClass_3_0= RULE_ID ) ) (otherlv_4= '(' ( (lv_parameters_5_0= ruleAssignments ) )? otherlv_6= ')' )? otherlv_7= ';'
            {
            // InternalLinguaFranca.g:1124:3: ( (lv_name_0_0= RULE_ID ) )
            // InternalLinguaFranca.g:1125:4: (lv_name_0_0= RULE_ID )
            {
            // InternalLinguaFranca.g:1125:4: (lv_name_0_0= RULE_ID )
            // InternalLinguaFranca.g:1126:5: lv_name_0_0= RULE_ID
            {
            lv_name_0_0=(Token)match(input,RULE_ID,FOLLOW_32); 

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

            otherlv_1=(Token)match(input,30,FOLLOW_33); 

            			newLeafNode(otherlv_1, grammarAccess.getInstanceAccess().getEqualsSignKeyword_1());
            		
            otherlv_2=(Token)match(input,31,FOLLOW_5); 

            			newLeafNode(otherlv_2, grammarAccess.getInstanceAccess().getNewKeyword_2());
            		
            // InternalLinguaFranca.g:1150:3: ( (lv_actorClass_3_0= RULE_ID ) )
            // InternalLinguaFranca.g:1151:4: (lv_actorClass_3_0= RULE_ID )
            {
            // InternalLinguaFranca.g:1151:4: (lv_actorClass_3_0= RULE_ID )
            // InternalLinguaFranca.g:1152:5: lv_actorClass_3_0= RULE_ID
            {
            lv_actorClass_3_0=(Token)match(input,RULE_ID,FOLLOW_25); 

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

            // InternalLinguaFranca.g:1168:3: (otherlv_4= '(' ( (lv_parameters_5_0= ruleAssignments ) )? otherlv_6= ')' )?
            int alt32=2;
            int LA32_0 = input.LA(1);

            if ( (LA32_0==25) ) {
                alt32=1;
            }
            switch (alt32) {
                case 1 :
                    // InternalLinguaFranca.g:1169:4: otherlv_4= '(' ( (lv_parameters_5_0= ruleAssignments ) )? otherlv_6= ')'
                    {
                    otherlv_4=(Token)match(input,25,FOLLOW_27); 

                    				newLeafNode(otherlv_4, grammarAccess.getInstanceAccess().getLeftParenthesisKeyword_4_0());
                    			
                    // InternalLinguaFranca.g:1173:4: ( (lv_parameters_5_0= ruleAssignments ) )?
                    int alt31=2;
                    int LA31_0 = input.LA(1);

                    if ( (LA31_0==RULE_ID) ) {
                        alt31=1;
                    }
                    switch (alt31) {
                        case 1 :
                            // InternalLinguaFranca.g:1174:5: (lv_parameters_5_0= ruleAssignments )
                            {
                            // InternalLinguaFranca.g:1174:5: (lv_parameters_5_0= ruleAssignments )
                            // InternalLinguaFranca.g:1175:6: lv_parameters_5_0= ruleAssignments
                            {

                            						newCompositeNode(grammarAccess.getInstanceAccess().getParametersAssignmentsParserRuleCall_4_1_0());
                            					
                            pushFollow(FOLLOW_34);
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

                    otherlv_6=(Token)match(input,27,FOLLOW_6); 

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
    // InternalLinguaFranca.g:1205:1: entryRuleConnection returns [EObject current=null] : iv_ruleConnection= ruleConnection EOF ;
    public final EObject entryRuleConnection() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleConnection = null;


        try {
            // InternalLinguaFranca.g:1205:51: (iv_ruleConnection= ruleConnection EOF )
            // InternalLinguaFranca.g:1206:2: iv_ruleConnection= ruleConnection EOF
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
    // InternalLinguaFranca.g:1212:1: ruleConnection returns [EObject current=null] : ( ( (lv_leftPort_0_0= rulePort ) ) otherlv_1= '->' ( (lv_rightPort_2_0= rulePort ) ) otherlv_3= ';' ) ;
    public final EObject ruleConnection() throws RecognitionException {
        EObject current = null;

        Token otherlv_1=null;
        Token otherlv_3=null;
        AntlrDatatypeRuleToken lv_leftPort_0_0 = null;

        AntlrDatatypeRuleToken lv_rightPort_2_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:1218:2: ( ( ( (lv_leftPort_0_0= rulePort ) ) otherlv_1= '->' ( (lv_rightPort_2_0= rulePort ) ) otherlv_3= ';' ) )
            // InternalLinguaFranca.g:1219:2: ( ( (lv_leftPort_0_0= rulePort ) ) otherlv_1= '->' ( (lv_rightPort_2_0= rulePort ) ) otherlv_3= ';' )
            {
            // InternalLinguaFranca.g:1219:2: ( ( (lv_leftPort_0_0= rulePort ) ) otherlv_1= '->' ( (lv_rightPort_2_0= rulePort ) ) otherlv_3= ';' )
            // InternalLinguaFranca.g:1220:3: ( (lv_leftPort_0_0= rulePort ) ) otherlv_1= '->' ( (lv_rightPort_2_0= rulePort ) ) otherlv_3= ';'
            {
            // InternalLinguaFranca.g:1220:3: ( (lv_leftPort_0_0= rulePort ) )
            // InternalLinguaFranca.g:1221:4: (lv_leftPort_0_0= rulePort )
            {
            // InternalLinguaFranca.g:1221:4: (lv_leftPort_0_0= rulePort )
            // InternalLinguaFranca.g:1222:5: lv_leftPort_0_0= rulePort
            {

            					newCompositeNode(grammarAccess.getConnectionAccess().getLeftPortPortParserRuleCall_0_0());
            				
            pushFollow(FOLLOW_35);
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
            		
            // InternalLinguaFranca.g:1243:3: ( (lv_rightPort_2_0= rulePort ) )
            // InternalLinguaFranca.g:1244:4: (lv_rightPort_2_0= rulePort )
            {
            // InternalLinguaFranca.g:1244:4: (lv_rightPort_2_0= rulePort )
            // InternalLinguaFranca.g:1245:5: lv_rightPort_2_0= rulePort
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
    // InternalLinguaFranca.g:1270:1: entryRuleAssignments returns [EObject current=null] : iv_ruleAssignments= ruleAssignments EOF ;
    public final EObject entryRuleAssignments() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleAssignments = null;


        try {
            // InternalLinguaFranca.g:1270:52: (iv_ruleAssignments= ruleAssignments EOF )
            // InternalLinguaFranca.g:1271:2: iv_ruleAssignments= ruleAssignments EOF
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
    // InternalLinguaFranca.g:1277:1: ruleAssignments returns [EObject current=null] : ( ( (lv_assignments_0_0= ruleAssignment ) ) (otherlv_1= ',' ( (lv_assignments_2_0= ruleAssignment ) ) )* ) ;
    public final EObject ruleAssignments() throws RecognitionException {
        EObject current = null;

        Token otherlv_1=null;
        EObject lv_assignments_0_0 = null;

        EObject lv_assignments_2_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:1283:2: ( ( ( (lv_assignments_0_0= ruleAssignment ) ) (otherlv_1= ',' ( (lv_assignments_2_0= ruleAssignment ) ) )* ) )
            // InternalLinguaFranca.g:1284:2: ( ( (lv_assignments_0_0= ruleAssignment ) ) (otherlv_1= ',' ( (lv_assignments_2_0= ruleAssignment ) ) )* )
            {
            // InternalLinguaFranca.g:1284:2: ( ( (lv_assignments_0_0= ruleAssignment ) ) (otherlv_1= ',' ( (lv_assignments_2_0= ruleAssignment ) ) )* )
            // InternalLinguaFranca.g:1285:3: ( (lv_assignments_0_0= ruleAssignment ) ) (otherlv_1= ',' ( (lv_assignments_2_0= ruleAssignment ) ) )*
            {
            // InternalLinguaFranca.g:1285:3: ( (lv_assignments_0_0= ruleAssignment ) )
            // InternalLinguaFranca.g:1286:4: (lv_assignments_0_0= ruleAssignment )
            {
            // InternalLinguaFranca.g:1286:4: (lv_assignments_0_0= ruleAssignment )
            // InternalLinguaFranca.g:1287:5: lv_assignments_0_0= ruleAssignment
            {

            					newCompositeNode(grammarAccess.getAssignmentsAccess().getAssignmentsAssignmentParserRuleCall_0_0());
            				
            pushFollow(FOLLOW_36);
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

            // InternalLinguaFranca.g:1304:3: (otherlv_1= ',' ( (lv_assignments_2_0= ruleAssignment ) ) )*
            loop33:
            do {
                int alt33=2;
                int LA33_0 = input.LA(1);

                if ( (LA33_0==26) ) {
                    alt33=1;
                }


                switch (alt33) {
            	case 1 :
            	    // InternalLinguaFranca.g:1305:4: otherlv_1= ',' ( (lv_assignments_2_0= ruleAssignment ) )
            	    {
            	    otherlv_1=(Token)match(input,26,FOLLOW_5); 

            	    				newLeafNode(otherlv_1, grammarAccess.getAssignmentsAccess().getCommaKeyword_1_0());
            	    			
            	    // InternalLinguaFranca.g:1309:4: ( (lv_assignments_2_0= ruleAssignment ) )
            	    // InternalLinguaFranca.g:1310:5: (lv_assignments_2_0= ruleAssignment )
            	    {
            	    // InternalLinguaFranca.g:1310:5: (lv_assignments_2_0= ruleAssignment )
            	    // InternalLinguaFranca.g:1311:6: lv_assignments_2_0= ruleAssignment
            	    {

            	    						newCompositeNode(grammarAccess.getAssignmentsAccess().getAssignmentsAssignmentParserRuleCall_1_1_0());
            	    					
            	    pushFollow(FOLLOW_36);
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
            	    break loop33;
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
    // InternalLinguaFranca.g:1333:1: entryRuleAssignment returns [EObject current=null] : iv_ruleAssignment= ruleAssignment EOF ;
    public final EObject entryRuleAssignment() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleAssignment = null;


        try {
            // InternalLinguaFranca.g:1333:51: (iv_ruleAssignment= ruleAssignment EOF )
            // InternalLinguaFranca.g:1334:2: iv_ruleAssignment= ruleAssignment EOF
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
    // InternalLinguaFranca.g:1340:1: ruleAssignment returns [EObject current=null] : ( ( (lv_name_0_0= RULE_ID ) ) otherlv_1= '=' ( (lv_value_2_0= ruleValue ) ) ) ;
    public final EObject ruleAssignment() throws RecognitionException {
        EObject current = null;

        Token lv_name_0_0=null;
        Token otherlv_1=null;
        AntlrDatatypeRuleToken lv_value_2_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:1346:2: ( ( ( (lv_name_0_0= RULE_ID ) ) otherlv_1= '=' ( (lv_value_2_0= ruleValue ) ) ) )
            // InternalLinguaFranca.g:1347:2: ( ( (lv_name_0_0= RULE_ID ) ) otherlv_1= '=' ( (lv_value_2_0= ruleValue ) ) )
            {
            // InternalLinguaFranca.g:1347:2: ( ( (lv_name_0_0= RULE_ID ) ) otherlv_1= '=' ( (lv_value_2_0= ruleValue ) ) )
            // InternalLinguaFranca.g:1348:3: ( (lv_name_0_0= RULE_ID ) ) otherlv_1= '=' ( (lv_value_2_0= ruleValue ) )
            {
            // InternalLinguaFranca.g:1348:3: ( (lv_name_0_0= RULE_ID ) )
            // InternalLinguaFranca.g:1349:4: (lv_name_0_0= RULE_ID )
            {
            // InternalLinguaFranca.g:1349:4: (lv_name_0_0= RULE_ID )
            // InternalLinguaFranca.g:1350:5: lv_name_0_0= RULE_ID
            {
            lv_name_0_0=(Token)match(input,RULE_ID,FOLLOW_32); 

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

            otherlv_1=(Token)match(input,30,FOLLOW_37); 

            			newLeafNode(otherlv_1, grammarAccess.getAssignmentAccess().getEqualsSignKeyword_1());
            		
            // InternalLinguaFranca.g:1370:3: ( (lv_value_2_0= ruleValue ) )
            // InternalLinguaFranca.g:1371:4: (lv_value_2_0= ruleValue )
            {
            // InternalLinguaFranca.g:1371:4: (lv_value_2_0= ruleValue )
            // InternalLinguaFranca.g:1372:5: lv_value_2_0= ruleValue
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
    // InternalLinguaFranca.g:1393:1: entryRuleGets returns [EObject current=null] : iv_ruleGets= ruleGets EOF ;
    public final EObject entryRuleGets() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleGets = null;


        try {
            // InternalLinguaFranca.g:1393:45: (iv_ruleGets= ruleGets EOF )
            // InternalLinguaFranca.g:1394:2: iv_ruleGets= ruleGets EOF
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
    // InternalLinguaFranca.g:1400:1: ruleGets returns [EObject current=null] : ( ( (lv_gets_0_0= RULE_ID ) ) (otherlv_1= ',' ( (lv_gets_2_0= RULE_ID ) ) )? ) ;
    public final EObject ruleGets() throws RecognitionException {
        EObject current = null;

        Token lv_gets_0_0=null;
        Token otherlv_1=null;
        Token lv_gets_2_0=null;


        	enterRule();

        try {
            // InternalLinguaFranca.g:1406:2: ( ( ( (lv_gets_0_0= RULE_ID ) ) (otherlv_1= ',' ( (lv_gets_2_0= RULE_ID ) ) )? ) )
            // InternalLinguaFranca.g:1407:2: ( ( (lv_gets_0_0= RULE_ID ) ) (otherlv_1= ',' ( (lv_gets_2_0= RULE_ID ) ) )? )
            {
            // InternalLinguaFranca.g:1407:2: ( ( (lv_gets_0_0= RULE_ID ) ) (otherlv_1= ',' ( (lv_gets_2_0= RULE_ID ) ) )? )
            // InternalLinguaFranca.g:1408:3: ( (lv_gets_0_0= RULE_ID ) ) (otherlv_1= ',' ( (lv_gets_2_0= RULE_ID ) ) )?
            {
            // InternalLinguaFranca.g:1408:3: ( (lv_gets_0_0= RULE_ID ) )
            // InternalLinguaFranca.g:1409:4: (lv_gets_0_0= RULE_ID )
            {
            // InternalLinguaFranca.g:1409:4: (lv_gets_0_0= RULE_ID )
            // InternalLinguaFranca.g:1410:5: lv_gets_0_0= RULE_ID
            {
            lv_gets_0_0=(Token)match(input,RULE_ID,FOLLOW_36); 

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

            // InternalLinguaFranca.g:1426:3: (otherlv_1= ',' ( (lv_gets_2_0= RULE_ID ) ) )?
            int alt34=2;
            int LA34_0 = input.LA(1);

            if ( (LA34_0==26) ) {
                alt34=1;
            }
            switch (alt34) {
                case 1 :
                    // InternalLinguaFranca.g:1427:4: otherlv_1= ',' ( (lv_gets_2_0= RULE_ID ) )
                    {
                    otherlv_1=(Token)match(input,26,FOLLOW_5); 

                    				newLeafNode(otherlv_1, grammarAccess.getGetsAccess().getCommaKeyword_1_0());
                    			
                    // InternalLinguaFranca.g:1431:4: ( (lv_gets_2_0= RULE_ID ) )
                    // InternalLinguaFranca.g:1432:5: (lv_gets_2_0= RULE_ID )
                    {
                    // InternalLinguaFranca.g:1432:5: (lv_gets_2_0= RULE_ID )
                    // InternalLinguaFranca.g:1433:6: lv_gets_2_0= RULE_ID
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
    // InternalLinguaFranca.g:1454:1: entryRuleParams returns [EObject current=null] : iv_ruleParams= ruleParams EOF ;
    public final EObject entryRuleParams() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleParams = null;


        try {
            // InternalLinguaFranca.g:1454:47: (iv_ruleParams= ruleParams EOF )
            // InternalLinguaFranca.g:1455:2: iv_ruleParams= ruleParams EOF
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
    // InternalLinguaFranca.g:1461:1: ruleParams returns [EObject current=null] : (otherlv_0= '(' ( (lv_params_1_0= ruleParam ) ) (otherlv_2= ',' ( (lv_params_3_0= ruleParam ) ) )* otherlv_4= ')' ) ;
    public final EObject ruleParams() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token otherlv_2=null;
        Token otherlv_4=null;
        EObject lv_params_1_0 = null;

        EObject lv_params_3_0 = null;



        	enterRule();

        try {
            // InternalLinguaFranca.g:1467:2: ( (otherlv_0= '(' ( (lv_params_1_0= ruleParam ) ) (otherlv_2= ',' ( (lv_params_3_0= ruleParam ) ) )* otherlv_4= ')' ) )
            // InternalLinguaFranca.g:1468:2: (otherlv_0= '(' ( (lv_params_1_0= ruleParam ) ) (otherlv_2= ',' ( (lv_params_3_0= ruleParam ) ) )* otherlv_4= ')' )
            {
            // InternalLinguaFranca.g:1468:2: (otherlv_0= '(' ( (lv_params_1_0= ruleParam ) ) (otherlv_2= ',' ( (lv_params_3_0= ruleParam ) ) )* otherlv_4= ')' )
            // InternalLinguaFranca.g:1469:3: otherlv_0= '(' ( (lv_params_1_0= ruleParam ) ) (otherlv_2= ',' ( (lv_params_3_0= ruleParam ) ) )* otherlv_4= ')'
            {
            otherlv_0=(Token)match(input,25,FOLLOW_38); 

            			newLeafNode(otherlv_0, grammarAccess.getParamsAccess().getLeftParenthesisKeyword_0());
            		
            // InternalLinguaFranca.g:1473:3: ( (lv_params_1_0= ruleParam ) )
            // InternalLinguaFranca.g:1474:4: (lv_params_1_0= ruleParam )
            {
            // InternalLinguaFranca.g:1474:4: (lv_params_1_0= ruleParam )
            // InternalLinguaFranca.g:1475:5: lv_params_1_0= ruleParam
            {

            					newCompositeNode(grammarAccess.getParamsAccess().getParamsParamParserRuleCall_1_0());
            				
            pushFollow(FOLLOW_28);
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

            // InternalLinguaFranca.g:1492:3: (otherlv_2= ',' ( (lv_params_3_0= ruleParam ) ) )*
            loop35:
            do {
                int alt35=2;
                int LA35_0 = input.LA(1);

                if ( (LA35_0==26) ) {
                    alt35=1;
                }


                switch (alt35) {
            	case 1 :
            	    // InternalLinguaFranca.g:1493:4: otherlv_2= ',' ( (lv_params_3_0= ruleParam ) )
            	    {
            	    otherlv_2=(Token)match(input,26,FOLLOW_38); 

            	    				newLeafNode(otherlv_2, grammarAccess.getParamsAccess().getCommaKeyword_2_0());
            	    			
            	    // InternalLinguaFranca.g:1497:4: ( (lv_params_3_0= ruleParam ) )
            	    // InternalLinguaFranca.g:1498:5: (lv_params_3_0= ruleParam )
            	    {
            	    // InternalLinguaFranca.g:1498:5: (lv_params_3_0= ruleParam )
            	    // InternalLinguaFranca.g:1499:6: lv_params_3_0= ruleParam
            	    {

            	    						newCompositeNode(grammarAccess.getParamsAccess().getParamsParamParserRuleCall_2_1_0());
            	    					
            	    pushFollow(FOLLOW_28);
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
            	    break loop35;
                }
            } while (true);

            otherlv_4=(Token)match(input,27,FOLLOW_2); 

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
    // InternalLinguaFranca.g:1525:1: entryRuleParam returns [EObject current=null] : iv_ruleParam= ruleParam EOF ;
    public final EObject entryRuleParam() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleParam = null;


        try {
            // InternalLinguaFranca.g:1525:46: (iv_ruleParam= ruleParam EOF )
            // InternalLinguaFranca.g:1526:2: iv_ruleParam= ruleParam EOF
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
    // InternalLinguaFranca.g:1532:1: ruleParam returns [EObject current=null] : ( (otherlv_0= 'const' )? ( (lv_name_1_0= RULE_ID ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? (otherlv_4= '(' ( (lv_value_5_0= ruleValue ) ) otherlv_6= ')' )? ) ;
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
            // InternalLinguaFranca.g:1538:2: ( ( (otherlv_0= 'const' )? ( (lv_name_1_0= RULE_ID ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? (otherlv_4= '(' ( (lv_value_5_0= ruleValue ) ) otherlv_6= ')' )? ) )
            // InternalLinguaFranca.g:1539:2: ( (otherlv_0= 'const' )? ( (lv_name_1_0= RULE_ID ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? (otherlv_4= '(' ( (lv_value_5_0= ruleValue ) ) otherlv_6= ')' )? )
            {
            // InternalLinguaFranca.g:1539:2: ( (otherlv_0= 'const' )? ( (lv_name_1_0= RULE_ID ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? (otherlv_4= '(' ( (lv_value_5_0= ruleValue ) ) otherlv_6= ')' )? )
            // InternalLinguaFranca.g:1540:3: (otherlv_0= 'const' )? ( (lv_name_1_0= RULE_ID ) ) (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )? (otherlv_4= '(' ( (lv_value_5_0= ruleValue ) ) otherlv_6= ')' )?
            {
            // InternalLinguaFranca.g:1540:3: (otherlv_0= 'const' )?
            int alt36=2;
            int LA36_0 = input.LA(1);

            if ( (LA36_0==33) ) {
                alt36=1;
            }
            switch (alt36) {
                case 1 :
                    // InternalLinguaFranca.g:1541:4: otherlv_0= 'const'
                    {
                    otherlv_0=(Token)match(input,33,FOLLOW_5); 

                    				newLeafNode(otherlv_0, grammarAccess.getParamAccess().getConstKeyword_0());
                    			

                    }
                    break;

            }

            // InternalLinguaFranca.g:1546:3: ( (lv_name_1_0= RULE_ID ) )
            // InternalLinguaFranca.g:1547:4: (lv_name_1_0= RULE_ID )
            {
            // InternalLinguaFranca.g:1547:4: (lv_name_1_0= RULE_ID )
            // InternalLinguaFranca.g:1548:5: lv_name_1_0= RULE_ID
            {
            lv_name_1_0=(Token)match(input,RULE_ID,FOLLOW_39); 

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

            // InternalLinguaFranca.g:1564:3: (otherlv_2= ':' ( (lv_type_3_0= ruleType ) ) )?
            int alt37=2;
            int LA37_0 = input.LA(1);

            if ( (LA37_0==21) ) {
                alt37=1;
            }
            switch (alt37) {
                case 1 :
                    // InternalLinguaFranca.g:1565:4: otherlv_2= ':' ( (lv_type_3_0= ruleType ) )
                    {
                    otherlv_2=(Token)match(input,21,FOLLOW_22); 

                    				newLeafNode(otherlv_2, grammarAccess.getParamAccess().getColonKeyword_2_0());
                    			
                    // InternalLinguaFranca.g:1569:4: ( (lv_type_3_0= ruleType ) )
                    // InternalLinguaFranca.g:1570:5: (lv_type_3_0= ruleType )
                    {
                    // InternalLinguaFranca.g:1570:5: (lv_type_3_0= ruleType )
                    // InternalLinguaFranca.g:1571:6: lv_type_3_0= ruleType
                    {

                    						newCompositeNode(grammarAccess.getParamAccess().getTypeTypeParserRuleCall_2_1_0());
                    					
                    pushFollow(FOLLOW_40);
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

            // InternalLinguaFranca.g:1589:3: (otherlv_4= '(' ( (lv_value_5_0= ruleValue ) ) otherlv_6= ')' )?
            int alt38=2;
            int LA38_0 = input.LA(1);

            if ( (LA38_0==25) ) {
                alt38=1;
            }
            switch (alt38) {
                case 1 :
                    // InternalLinguaFranca.g:1590:4: otherlv_4= '(' ( (lv_value_5_0= ruleValue ) ) otherlv_6= ')'
                    {
                    otherlv_4=(Token)match(input,25,FOLLOW_37); 

                    				newLeafNode(otherlv_4, grammarAccess.getParamAccess().getLeftParenthesisKeyword_3_0());
                    			
                    // InternalLinguaFranca.g:1594:4: ( (lv_value_5_0= ruleValue ) )
                    // InternalLinguaFranca.g:1595:5: (lv_value_5_0= ruleValue )
                    {
                    // InternalLinguaFranca.g:1595:5: (lv_value_5_0= ruleValue )
                    // InternalLinguaFranca.g:1596:6: lv_value_5_0= ruleValue
                    {

                    						newCompositeNode(grammarAccess.getParamAccess().getValueValueParserRuleCall_3_1_0());
                    					
                    pushFollow(FOLLOW_34);
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

                    otherlv_6=(Token)match(input,27,FOLLOW_2); 

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


    // $ANTLR start "entryRulePeriod"
    // InternalLinguaFranca.g:1622:1: entryRulePeriod returns [EObject current=null] : iv_rulePeriod= rulePeriod EOF ;
    public final EObject entryRulePeriod() throws RecognitionException {
        EObject current = null;

        EObject iv_rulePeriod = null;


        try {
            // InternalLinguaFranca.g:1622:47: (iv_rulePeriod= rulePeriod EOF )
            // InternalLinguaFranca.g:1623:2: iv_rulePeriod= rulePeriod EOF
            {
             newCompositeNode(grammarAccess.getPeriodRule()); 
            pushFollow(FOLLOW_1);
            iv_rulePeriod=rulePeriod();

            state._fsp--;

             current =iv_rulePeriod; 
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
    // $ANTLR end "entryRulePeriod"


    // $ANTLR start "rulePeriod"
    // InternalLinguaFranca.g:1629:1: rulePeriod returns [EObject current=null] : (otherlv_0= '(' ( ( (lv_period_1_1= RULE_ID | lv_period_1_2= RULE_NUMBER ) ) ) (otherlv_2= ',' ( ( (lv_offset_3_1= RULE_ID | lv_offset_3_2= RULE_NUMBER ) ) ) (otherlv_4= ',' ( ( (lv_count_5_1= RULE_ID | lv_count_5_2= RULE_NUMBER ) ) ) )? )? otherlv_6= ')' ) ;
    public final EObject rulePeriod() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token lv_period_1_1=null;
        Token lv_period_1_2=null;
        Token otherlv_2=null;
        Token lv_offset_3_1=null;
        Token lv_offset_3_2=null;
        Token otherlv_4=null;
        Token lv_count_5_1=null;
        Token lv_count_5_2=null;
        Token otherlv_6=null;


        	enterRule();

        try {
            // InternalLinguaFranca.g:1635:2: ( (otherlv_0= '(' ( ( (lv_period_1_1= RULE_ID | lv_period_1_2= RULE_NUMBER ) ) ) (otherlv_2= ',' ( ( (lv_offset_3_1= RULE_ID | lv_offset_3_2= RULE_NUMBER ) ) ) (otherlv_4= ',' ( ( (lv_count_5_1= RULE_ID | lv_count_5_2= RULE_NUMBER ) ) ) )? )? otherlv_6= ')' ) )
            // InternalLinguaFranca.g:1636:2: (otherlv_0= '(' ( ( (lv_period_1_1= RULE_ID | lv_period_1_2= RULE_NUMBER ) ) ) (otherlv_2= ',' ( ( (lv_offset_3_1= RULE_ID | lv_offset_3_2= RULE_NUMBER ) ) ) (otherlv_4= ',' ( ( (lv_count_5_1= RULE_ID | lv_count_5_2= RULE_NUMBER ) ) ) )? )? otherlv_6= ')' )
            {
            // InternalLinguaFranca.g:1636:2: (otherlv_0= '(' ( ( (lv_period_1_1= RULE_ID | lv_period_1_2= RULE_NUMBER ) ) ) (otherlv_2= ',' ( ( (lv_offset_3_1= RULE_ID | lv_offset_3_2= RULE_NUMBER ) ) ) (otherlv_4= ',' ( ( (lv_count_5_1= RULE_ID | lv_count_5_2= RULE_NUMBER ) ) ) )? )? otherlv_6= ')' )
            // InternalLinguaFranca.g:1637:3: otherlv_0= '(' ( ( (lv_period_1_1= RULE_ID | lv_period_1_2= RULE_NUMBER ) ) ) (otherlv_2= ',' ( ( (lv_offset_3_1= RULE_ID | lv_offset_3_2= RULE_NUMBER ) ) ) (otherlv_4= ',' ( ( (lv_count_5_1= RULE_ID | lv_count_5_2= RULE_NUMBER ) ) ) )? )? otherlv_6= ')'
            {
            otherlv_0=(Token)match(input,25,FOLLOW_41); 

            			newLeafNode(otherlv_0, grammarAccess.getPeriodAccess().getLeftParenthesisKeyword_0());
            		
            // InternalLinguaFranca.g:1641:3: ( ( (lv_period_1_1= RULE_ID | lv_period_1_2= RULE_NUMBER ) ) )
            // InternalLinguaFranca.g:1642:4: ( (lv_period_1_1= RULE_ID | lv_period_1_2= RULE_NUMBER ) )
            {
            // InternalLinguaFranca.g:1642:4: ( (lv_period_1_1= RULE_ID | lv_period_1_2= RULE_NUMBER ) )
            // InternalLinguaFranca.g:1643:5: (lv_period_1_1= RULE_ID | lv_period_1_2= RULE_NUMBER )
            {
            // InternalLinguaFranca.g:1643:5: (lv_period_1_1= RULE_ID | lv_period_1_2= RULE_NUMBER )
            int alt39=2;
            int LA39_0 = input.LA(1);

            if ( (LA39_0==RULE_ID) ) {
                alt39=1;
            }
            else if ( (LA39_0==RULE_NUMBER) ) {
                alt39=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 39, 0, input);

                throw nvae;
            }
            switch (alt39) {
                case 1 :
                    // InternalLinguaFranca.g:1644:6: lv_period_1_1= RULE_ID
                    {
                    lv_period_1_1=(Token)match(input,RULE_ID,FOLLOW_28); 

                    						newLeafNode(lv_period_1_1, grammarAccess.getPeriodAccess().getPeriodIDTerminalRuleCall_1_0_0());
                    					

                    						if (current==null) {
                    							current = createModelElement(grammarAccess.getPeriodRule());
                    						}
                    						setWithLastConsumed(
                    							current,
                    							"period",
                    							lv_period_1_1,
                    							"org.eclipse.xtext.common.Terminals.ID");
                    					

                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:1659:6: lv_period_1_2= RULE_NUMBER
                    {
                    lv_period_1_2=(Token)match(input,RULE_NUMBER,FOLLOW_28); 

                    						newLeafNode(lv_period_1_2, grammarAccess.getPeriodAccess().getPeriodNUMBERTerminalRuleCall_1_0_1());
                    					

                    						if (current==null) {
                    							current = createModelElement(grammarAccess.getPeriodRule());
                    						}
                    						setWithLastConsumed(
                    							current,
                    							"period",
                    							lv_period_1_2,
                    							"org.icyphy.LinguaFranca.NUMBER");
                    					

                    }
                    break;

            }


            }


            }

            // InternalLinguaFranca.g:1676:3: (otherlv_2= ',' ( ( (lv_offset_3_1= RULE_ID | lv_offset_3_2= RULE_NUMBER ) ) ) (otherlv_4= ',' ( ( (lv_count_5_1= RULE_ID | lv_count_5_2= RULE_NUMBER ) ) ) )? )?
            int alt43=2;
            int LA43_0 = input.LA(1);

            if ( (LA43_0==26) ) {
                alt43=1;
            }
            switch (alt43) {
                case 1 :
                    // InternalLinguaFranca.g:1677:4: otherlv_2= ',' ( ( (lv_offset_3_1= RULE_ID | lv_offset_3_2= RULE_NUMBER ) ) ) (otherlv_4= ',' ( ( (lv_count_5_1= RULE_ID | lv_count_5_2= RULE_NUMBER ) ) ) )?
                    {
                    otherlv_2=(Token)match(input,26,FOLLOW_41); 

                    				newLeafNode(otherlv_2, grammarAccess.getPeriodAccess().getCommaKeyword_2_0());
                    			
                    // InternalLinguaFranca.g:1681:4: ( ( (lv_offset_3_1= RULE_ID | lv_offset_3_2= RULE_NUMBER ) ) )
                    // InternalLinguaFranca.g:1682:5: ( (lv_offset_3_1= RULE_ID | lv_offset_3_2= RULE_NUMBER ) )
                    {
                    // InternalLinguaFranca.g:1682:5: ( (lv_offset_3_1= RULE_ID | lv_offset_3_2= RULE_NUMBER ) )
                    // InternalLinguaFranca.g:1683:6: (lv_offset_3_1= RULE_ID | lv_offset_3_2= RULE_NUMBER )
                    {
                    // InternalLinguaFranca.g:1683:6: (lv_offset_3_1= RULE_ID | lv_offset_3_2= RULE_NUMBER )
                    int alt40=2;
                    int LA40_0 = input.LA(1);

                    if ( (LA40_0==RULE_ID) ) {
                        alt40=1;
                    }
                    else if ( (LA40_0==RULE_NUMBER) ) {
                        alt40=2;
                    }
                    else {
                        NoViableAltException nvae =
                            new NoViableAltException("", 40, 0, input);

                        throw nvae;
                    }
                    switch (alt40) {
                        case 1 :
                            // InternalLinguaFranca.g:1684:7: lv_offset_3_1= RULE_ID
                            {
                            lv_offset_3_1=(Token)match(input,RULE_ID,FOLLOW_28); 

                            							newLeafNode(lv_offset_3_1, grammarAccess.getPeriodAccess().getOffsetIDTerminalRuleCall_2_1_0_0());
                            						

                            							if (current==null) {
                            								current = createModelElement(grammarAccess.getPeriodRule());
                            							}
                            							setWithLastConsumed(
                            								current,
                            								"offset",
                            								lv_offset_3_1,
                            								"org.eclipse.xtext.common.Terminals.ID");
                            						

                            }
                            break;
                        case 2 :
                            // InternalLinguaFranca.g:1699:7: lv_offset_3_2= RULE_NUMBER
                            {
                            lv_offset_3_2=(Token)match(input,RULE_NUMBER,FOLLOW_28); 

                            							newLeafNode(lv_offset_3_2, grammarAccess.getPeriodAccess().getOffsetNUMBERTerminalRuleCall_2_1_0_1());
                            						

                            							if (current==null) {
                            								current = createModelElement(grammarAccess.getPeriodRule());
                            							}
                            							setWithLastConsumed(
                            								current,
                            								"offset",
                            								lv_offset_3_2,
                            								"org.icyphy.LinguaFranca.NUMBER");
                            						

                            }
                            break;

                    }


                    }


                    }

                    // InternalLinguaFranca.g:1716:4: (otherlv_4= ',' ( ( (lv_count_5_1= RULE_ID | lv_count_5_2= RULE_NUMBER ) ) ) )?
                    int alt42=2;
                    int LA42_0 = input.LA(1);

                    if ( (LA42_0==26) ) {
                        alt42=1;
                    }
                    switch (alt42) {
                        case 1 :
                            // InternalLinguaFranca.g:1717:5: otherlv_4= ',' ( ( (lv_count_5_1= RULE_ID | lv_count_5_2= RULE_NUMBER ) ) )
                            {
                            otherlv_4=(Token)match(input,26,FOLLOW_41); 

                            					newLeafNode(otherlv_4, grammarAccess.getPeriodAccess().getCommaKeyword_2_2_0());
                            				
                            // InternalLinguaFranca.g:1721:5: ( ( (lv_count_5_1= RULE_ID | lv_count_5_2= RULE_NUMBER ) ) )
                            // InternalLinguaFranca.g:1722:6: ( (lv_count_5_1= RULE_ID | lv_count_5_2= RULE_NUMBER ) )
                            {
                            // InternalLinguaFranca.g:1722:6: ( (lv_count_5_1= RULE_ID | lv_count_5_2= RULE_NUMBER ) )
                            // InternalLinguaFranca.g:1723:7: (lv_count_5_1= RULE_ID | lv_count_5_2= RULE_NUMBER )
                            {
                            // InternalLinguaFranca.g:1723:7: (lv_count_5_1= RULE_ID | lv_count_5_2= RULE_NUMBER )
                            int alt41=2;
                            int LA41_0 = input.LA(1);

                            if ( (LA41_0==RULE_ID) ) {
                                alt41=1;
                            }
                            else if ( (LA41_0==RULE_NUMBER) ) {
                                alt41=2;
                            }
                            else {
                                NoViableAltException nvae =
                                    new NoViableAltException("", 41, 0, input);

                                throw nvae;
                            }
                            switch (alt41) {
                                case 1 :
                                    // InternalLinguaFranca.g:1724:8: lv_count_5_1= RULE_ID
                                    {
                                    lv_count_5_1=(Token)match(input,RULE_ID,FOLLOW_34); 

                                    								newLeafNode(lv_count_5_1, grammarAccess.getPeriodAccess().getCountIDTerminalRuleCall_2_2_1_0_0());
                                    							

                                    								if (current==null) {
                                    									current = createModelElement(grammarAccess.getPeriodRule());
                                    								}
                                    								setWithLastConsumed(
                                    									current,
                                    									"count",
                                    									lv_count_5_1,
                                    									"org.eclipse.xtext.common.Terminals.ID");
                                    							

                                    }
                                    break;
                                case 2 :
                                    // InternalLinguaFranca.g:1739:8: lv_count_5_2= RULE_NUMBER
                                    {
                                    lv_count_5_2=(Token)match(input,RULE_NUMBER,FOLLOW_34); 

                                    								newLeafNode(lv_count_5_2, grammarAccess.getPeriodAccess().getCountNUMBERTerminalRuleCall_2_2_1_0_1());
                                    							

                                    								if (current==null) {
                                    									current = createModelElement(grammarAccess.getPeriodRule());
                                    								}
                                    								setWithLastConsumed(
                                    									current,
                                    									"count",
                                    									lv_count_5_2,
                                    									"org.icyphy.LinguaFranca.NUMBER");
                                    							

                                    }
                                    break;

                            }


                            }


                            }


                            }
                            break;

                    }


                    }
                    break;

            }

            otherlv_6=(Token)match(input,27,FOLLOW_2); 

            			newLeafNode(otherlv_6, grammarAccess.getPeriodAccess().getRightParenthesisKeyword_3());
            		

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
    // $ANTLR end "rulePeriod"


    // $ANTLR start "entryRulePort"
    // InternalLinguaFranca.g:1766:1: entryRulePort returns [String current=null] : iv_rulePort= rulePort EOF ;
    public final String entryRulePort() throws RecognitionException {
        String current = null;

        AntlrDatatypeRuleToken iv_rulePort = null;


        try {
            // InternalLinguaFranca.g:1766:44: (iv_rulePort= rulePort EOF )
            // InternalLinguaFranca.g:1767:2: iv_rulePort= rulePort EOF
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
    // InternalLinguaFranca.g:1773:1: rulePort returns [AntlrDatatypeRuleToken current=new AntlrDatatypeRuleToken()] : (this_ID_0= RULE_ID | (this_ID_1= RULE_ID kw= '.' (this_ID_3= RULE_ID | kw= 'input' | kw= 'output' ) ) ) ;
    public final AntlrDatatypeRuleToken rulePort() throws RecognitionException {
        AntlrDatatypeRuleToken current = new AntlrDatatypeRuleToken();

        Token this_ID_0=null;
        Token this_ID_1=null;
        Token kw=null;
        Token this_ID_3=null;


        	enterRule();

        try {
            // InternalLinguaFranca.g:1779:2: ( (this_ID_0= RULE_ID | (this_ID_1= RULE_ID kw= '.' (this_ID_3= RULE_ID | kw= 'input' | kw= 'output' ) ) ) )
            // InternalLinguaFranca.g:1780:2: (this_ID_0= RULE_ID | (this_ID_1= RULE_ID kw= '.' (this_ID_3= RULE_ID | kw= 'input' | kw= 'output' ) ) )
            {
            // InternalLinguaFranca.g:1780:2: (this_ID_0= RULE_ID | (this_ID_1= RULE_ID kw= '.' (this_ID_3= RULE_ID | kw= 'input' | kw= 'output' ) ) )
            int alt45=2;
            int LA45_0 = input.LA(1);

            if ( (LA45_0==RULE_ID) ) {
                int LA45_1 = input.LA(2);

                if ( (LA45_1==34) ) {
                    alt45=2;
                }
                else if ( (LA45_1==EOF||LA45_1==14||LA45_1==32) ) {
                    alt45=1;
                }
                else {
                    NoViableAltException nvae =
                        new NoViableAltException("", 45, 1, input);

                    throw nvae;
                }
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 45, 0, input);

                throw nvae;
            }
            switch (alt45) {
                case 1 :
                    // InternalLinguaFranca.g:1781:3: this_ID_0= RULE_ID
                    {
                    this_ID_0=(Token)match(input,RULE_ID,FOLLOW_2); 

                    			current.merge(this_ID_0);
                    		

                    			newLeafNode(this_ID_0, grammarAccess.getPortAccess().getIDTerminalRuleCall_0());
                    		

                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:1789:3: (this_ID_1= RULE_ID kw= '.' (this_ID_3= RULE_ID | kw= 'input' | kw= 'output' ) )
                    {
                    // InternalLinguaFranca.g:1789:3: (this_ID_1= RULE_ID kw= '.' (this_ID_3= RULE_ID | kw= 'input' | kw= 'output' ) )
                    // InternalLinguaFranca.g:1790:4: this_ID_1= RULE_ID kw= '.' (this_ID_3= RULE_ID | kw= 'input' | kw= 'output' )
                    {
                    this_ID_1=(Token)match(input,RULE_ID,FOLLOW_42); 

                    				current.merge(this_ID_1);
                    			

                    				newLeafNode(this_ID_1, grammarAccess.getPortAccess().getIDTerminalRuleCall_1_0());
                    			
                    kw=(Token)match(input,34,FOLLOW_43); 

                    				current.merge(kw);
                    				newLeafNode(kw, grammarAccess.getPortAccess().getFullStopKeyword_1_1());
                    			
                    // InternalLinguaFranca.g:1802:4: (this_ID_3= RULE_ID | kw= 'input' | kw= 'output' )
                    int alt44=3;
                    switch ( input.LA(1) ) {
                    case RULE_ID:
                        {
                        alt44=1;
                        }
                        break;
                    case 20:
                        {
                        alt44=2;
                        }
                        break;
                    case 22:
                        {
                        alt44=3;
                        }
                        break;
                    default:
                        NoViableAltException nvae =
                            new NoViableAltException("", 44, 0, input);

                        throw nvae;
                    }

                    switch (alt44) {
                        case 1 :
                            // InternalLinguaFranca.g:1803:5: this_ID_3= RULE_ID
                            {
                            this_ID_3=(Token)match(input,RULE_ID,FOLLOW_2); 

                            					current.merge(this_ID_3);
                            				

                            					newLeafNode(this_ID_3, grammarAccess.getPortAccess().getIDTerminalRuleCall_1_2_0());
                            				

                            }
                            break;
                        case 2 :
                            // InternalLinguaFranca.g:1811:5: kw= 'input'
                            {
                            kw=(Token)match(input,20,FOLLOW_2); 

                            					current.merge(kw);
                            					newLeafNode(kw, grammarAccess.getPortAccess().getInputKeyword_1_2_1());
                            				

                            }
                            break;
                        case 3 :
                            // InternalLinguaFranca.g:1817:5: kw= 'output'
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
    // InternalLinguaFranca.g:1828:1: entryRuleSets returns [EObject current=null] : iv_ruleSets= ruleSets EOF ;
    public final EObject entryRuleSets() throws RecognitionException {
        EObject current = null;

        EObject iv_ruleSets = null;


        try {
            // InternalLinguaFranca.g:1828:45: (iv_ruleSets= ruleSets EOF )
            // InternalLinguaFranca.g:1829:2: iv_ruleSets= ruleSets EOF
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
    // InternalLinguaFranca.g:1835:1: ruleSets returns [EObject current=null] : (otherlv_0= '->' ( (lv_sets_1_0= RULE_ID ) ) (otherlv_2= ',' ( (lv_sets_3_0= RULE_ID ) ) )? ) ;
    public final EObject ruleSets() throws RecognitionException {
        EObject current = null;

        Token otherlv_0=null;
        Token lv_sets_1_0=null;
        Token otherlv_2=null;
        Token lv_sets_3_0=null;


        	enterRule();

        try {
            // InternalLinguaFranca.g:1841:2: ( (otherlv_0= '->' ( (lv_sets_1_0= RULE_ID ) ) (otherlv_2= ',' ( (lv_sets_3_0= RULE_ID ) ) )? ) )
            // InternalLinguaFranca.g:1842:2: (otherlv_0= '->' ( (lv_sets_1_0= RULE_ID ) ) (otherlv_2= ',' ( (lv_sets_3_0= RULE_ID ) ) )? )
            {
            // InternalLinguaFranca.g:1842:2: (otherlv_0= '->' ( (lv_sets_1_0= RULE_ID ) ) (otherlv_2= ',' ( (lv_sets_3_0= RULE_ID ) ) )? )
            // InternalLinguaFranca.g:1843:3: otherlv_0= '->' ( (lv_sets_1_0= RULE_ID ) ) (otherlv_2= ',' ( (lv_sets_3_0= RULE_ID ) ) )?
            {
            otherlv_0=(Token)match(input,32,FOLLOW_5); 

            			newLeafNode(otherlv_0, grammarAccess.getSetsAccess().getHyphenMinusGreaterThanSignKeyword_0());
            		
            // InternalLinguaFranca.g:1847:3: ( (lv_sets_1_0= RULE_ID ) )
            // InternalLinguaFranca.g:1848:4: (lv_sets_1_0= RULE_ID )
            {
            // InternalLinguaFranca.g:1848:4: (lv_sets_1_0= RULE_ID )
            // InternalLinguaFranca.g:1849:5: lv_sets_1_0= RULE_ID
            {
            lv_sets_1_0=(Token)match(input,RULE_ID,FOLLOW_36); 

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

            // InternalLinguaFranca.g:1865:3: (otherlv_2= ',' ( (lv_sets_3_0= RULE_ID ) ) )?
            int alt46=2;
            int LA46_0 = input.LA(1);

            if ( (LA46_0==26) ) {
                alt46=1;
            }
            switch (alt46) {
                case 1 :
                    // InternalLinguaFranca.g:1866:4: otherlv_2= ',' ( (lv_sets_3_0= RULE_ID ) )
                    {
                    otherlv_2=(Token)match(input,26,FOLLOW_5); 

                    				newLeafNode(otherlv_2, grammarAccess.getSetsAccess().getCommaKeyword_2_0());
                    			
                    // InternalLinguaFranca.g:1870:4: ( (lv_sets_3_0= RULE_ID ) )
                    // InternalLinguaFranca.g:1871:5: (lv_sets_3_0= RULE_ID )
                    {
                    // InternalLinguaFranca.g:1871:5: (lv_sets_3_0= RULE_ID )
                    // InternalLinguaFranca.g:1872:6: lv_sets_3_0= RULE_ID
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
    // InternalLinguaFranca.g:1893:1: entryRuleType returns [String current=null] : iv_ruleType= ruleType EOF ;
    public final String entryRuleType() throws RecognitionException {
        String current = null;

        AntlrDatatypeRuleToken iv_ruleType = null;


        try {
            // InternalLinguaFranca.g:1893:44: (iv_ruleType= ruleType EOF )
            // InternalLinguaFranca.g:1894:2: iv_ruleType= ruleType EOF
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
    // InternalLinguaFranca.g:1900:1: ruleType returns [AntlrDatatypeRuleToken current=new AntlrDatatypeRuleToken()] : (this_ID_0= RULE_ID | this_CODE_1= RULE_CODE ) ;
    public final AntlrDatatypeRuleToken ruleType() throws RecognitionException {
        AntlrDatatypeRuleToken current = new AntlrDatatypeRuleToken();

        Token this_ID_0=null;
        Token this_CODE_1=null;


        	enterRule();

        try {
            // InternalLinguaFranca.g:1906:2: ( (this_ID_0= RULE_ID | this_CODE_1= RULE_CODE ) )
            // InternalLinguaFranca.g:1907:2: (this_ID_0= RULE_ID | this_CODE_1= RULE_CODE )
            {
            // InternalLinguaFranca.g:1907:2: (this_ID_0= RULE_ID | this_CODE_1= RULE_CODE )
            int alt47=2;
            int LA47_0 = input.LA(1);

            if ( (LA47_0==RULE_ID) ) {
                alt47=1;
            }
            else if ( (LA47_0==RULE_CODE) ) {
                alt47=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 47, 0, input);

                throw nvae;
            }
            switch (alt47) {
                case 1 :
                    // InternalLinguaFranca.g:1908:3: this_ID_0= RULE_ID
                    {
                    this_ID_0=(Token)match(input,RULE_ID,FOLLOW_2); 

                    			current.merge(this_ID_0);
                    		

                    			newLeafNode(this_ID_0, grammarAccess.getTypeAccess().getIDTerminalRuleCall_0());
                    		

                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:1916:3: this_CODE_1= RULE_CODE
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
    // InternalLinguaFranca.g:1927:1: entryRuleValue returns [String current=null] : iv_ruleValue= ruleValue EOF ;
    public final String entryRuleValue() throws RecognitionException {
        String current = null;

        AntlrDatatypeRuleToken iv_ruleValue = null;


        try {
            // InternalLinguaFranca.g:1927:45: (iv_ruleValue= ruleValue EOF )
            // InternalLinguaFranca.g:1928:2: iv_ruleValue= ruleValue EOF
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
    // InternalLinguaFranca.g:1934:1: ruleValue returns [AntlrDatatypeRuleToken current=new AntlrDatatypeRuleToken()] : (this_ID_0= RULE_ID | this_NUMBER_1= RULE_NUMBER | this_STRING_2= RULE_STRING | this_CODE_3= RULE_CODE ) ;
    public final AntlrDatatypeRuleToken ruleValue() throws RecognitionException {
        AntlrDatatypeRuleToken current = new AntlrDatatypeRuleToken();

        Token this_ID_0=null;
        Token this_NUMBER_1=null;
        Token this_STRING_2=null;
        Token this_CODE_3=null;


        	enterRule();

        try {
            // InternalLinguaFranca.g:1940:2: ( (this_ID_0= RULE_ID | this_NUMBER_1= RULE_NUMBER | this_STRING_2= RULE_STRING | this_CODE_3= RULE_CODE ) )
            // InternalLinguaFranca.g:1941:2: (this_ID_0= RULE_ID | this_NUMBER_1= RULE_NUMBER | this_STRING_2= RULE_STRING | this_CODE_3= RULE_CODE )
            {
            // InternalLinguaFranca.g:1941:2: (this_ID_0= RULE_ID | this_NUMBER_1= RULE_NUMBER | this_STRING_2= RULE_STRING | this_CODE_3= RULE_CODE )
            int alt48=4;
            switch ( input.LA(1) ) {
            case RULE_ID:
                {
                alt48=1;
                }
                break;
            case RULE_NUMBER:
                {
                alt48=2;
                }
                break;
            case RULE_STRING:
                {
                alt48=3;
                }
                break;
            case RULE_CODE:
                {
                alt48=4;
                }
                break;
            default:
                NoViableAltException nvae =
                    new NoViableAltException("", 48, 0, input);

                throw nvae;
            }

            switch (alt48) {
                case 1 :
                    // InternalLinguaFranca.g:1942:3: this_ID_0= RULE_ID
                    {
                    this_ID_0=(Token)match(input,RULE_ID,FOLLOW_2); 

                    			current.merge(this_ID_0);
                    		

                    			newLeafNode(this_ID_0, grammarAccess.getValueAccess().getIDTerminalRuleCall_0());
                    		

                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:1950:3: this_NUMBER_1= RULE_NUMBER
                    {
                    this_NUMBER_1=(Token)match(input,RULE_NUMBER,FOLLOW_2); 

                    			current.merge(this_NUMBER_1);
                    		

                    			newLeafNode(this_NUMBER_1, grammarAccess.getValueAccess().getNUMBERTerminalRuleCall_1());
                    		

                    }
                    break;
                case 3 :
                    // InternalLinguaFranca.g:1958:3: this_STRING_2= RULE_STRING
                    {
                    this_STRING_2=(Token)match(input,RULE_STRING,FOLLOW_2); 

                    			current.merge(this_STRING_2);
                    		

                    			newLeafNode(this_STRING_2, grammarAccess.getValueAccess().getSTRINGTerminalRuleCall_2());
                    		

                    }
                    break;
                case 4 :
                    // InternalLinguaFranca.g:1966:3: this_CODE_3= RULE_CODE
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
    // InternalLinguaFranca.g:1977:1: entryRulePath returns [String current=null] : iv_rulePath= rulePath EOF ;
    public final String entryRulePath() throws RecognitionException {
        String current = null;

        AntlrDatatypeRuleToken iv_rulePath = null;


        try {
            // InternalLinguaFranca.g:1977:44: (iv_rulePath= rulePath EOF )
            // InternalLinguaFranca.g:1978:2: iv_rulePath= rulePath EOF
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
    // InternalLinguaFranca.g:1984:1: rulePath returns [AntlrDatatypeRuleToken current=new AntlrDatatypeRuleToken()] : (this_ID_0= RULE_ID (kw= '.' this_ID_2= RULE_ID )* ) ;
    public final AntlrDatatypeRuleToken rulePath() throws RecognitionException {
        AntlrDatatypeRuleToken current = new AntlrDatatypeRuleToken();

        Token this_ID_0=null;
        Token kw=null;
        Token this_ID_2=null;


        	enterRule();

        try {
            // InternalLinguaFranca.g:1990:2: ( (this_ID_0= RULE_ID (kw= '.' this_ID_2= RULE_ID )* ) )
            // InternalLinguaFranca.g:1991:2: (this_ID_0= RULE_ID (kw= '.' this_ID_2= RULE_ID )* )
            {
            // InternalLinguaFranca.g:1991:2: (this_ID_0= RULE_ID (kw= '.' this_ID_2= RULE_ID )* )
            // InternalLinguaFranca.g:1992:3: this_ID_0= RULE_ID (kw= '.' this_ID_2= RULE_ID )*
            {
            this_ID_0=(Token)match(input,RULE_ID,FOLLOW_44); 

            			current.merge(this_ID_0);
            		

            			newLeafNode(this_ID_0, grammarAccess.getPathAccess().getIDTerminalRuleCall_0());
            		
            // InternalLinguaFranca.g:1999:3: (kw= '.' this_ID_2= RULE_ID )*
            loop49:
            do {
                int alt49=2;
                int LA49_0 = input.LA(1);

                if ( (LA49_0==34) ) {
                    alt49=1;
                }


                switch (alt49) {
            	case 1 :
            	    // InternalLinguaFranca.g:2000:4: kw= '.' this_ID_2= RULE_ID
            	    {
            	    kw=(Token)match(input,34,FOLLOW_5); 

            	    				current.merge(kw);
            	    				newLeafNode(kw, grammarAccess.getPathAccess().getFullStopKeyword_1_0());
            	    			
            	    this_ID_2=(Token)match(input,RULE_ID,FOLLOW_44); 

            	    				current.merge(this_ID_2);
            	    			

            	    				newLeafNode(this_ID_2, grammarAccess.getPathAccess().getIDTerminalRuleCall_1_1());
            	    			

            	    }
            	    break;

            	default :
            	    break loop49;
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
    public static final BitSet FOLLOW_3 = new BitSet(new long[]{0x0000000000098000L});
    public static final BitSet FOLLOW_4 = new BitSet(new long[]{0x0000000000098002L});
    public static final BitSet FOLLOW_5 = new BitSet(new long[]{0x0000000000000010L});
    public static final BitSet FOLLOW_6 = new BitSet(new long[]{0x0000000000004000L});
    public static final BitSet FOLLOW_7 = new BitSet(new long[]{0x0000000002020000L});
    public static final BitSet FOLLOW_8 = new BitSet(new long[]{0x0000000000020000L});
    public static final BitSet FOLLOW_9 = new BitSet(new long[]{0x0000000031D40000L});
    public static final BitSet FOLLOW_10 = new BitSet(new long[]{0x0000000031C40000L});
    public static final BitSet FOLLOW_11 = new BitSet(new long[]{0x0000000031840000L});
    public static final BitSet FOLLOW_12 = new BitSet(new long[]{0x0000000021040000L});
    public static final BitSet FOLLOW_13 = new BitSet(new long[]{0x0000000001040000L});
    public static final BitSet FOLLOW_14 = new BitSet(new long[]{0x0000000031D40010L});
    public static final BitSet FOLLOW_15 = new BitSet(new long[]{0x0000000031C40010L});
    public static final BitSet FOLLOW_16 = new BitSet(new long[]{0x0000000031840010L});
    public static final BitSet FOLLOW_17 = new BitSet(new long[]{0x0000000021040010L});
    public static final BitSet FOLLOW_18 = new BitSet(new long[]{0x0000000001040010L});
    public static final BitSet FOLLOW_19 = new BitSet(new long[]{0x0000000000040010L});
    public static final BitSet FOLLOW_20 = new BitSet(new long[]{0x0000000000100010L});
    public static final BitSet FOLLOW_21 = new BitSet(new long[]{0x0000000000204000L});
    public static final BitSet FOLLOW_22 = new BitSet(new long[]{0x0000000000000030L});
    public static final BitSet FOLLOW_23 = new BitSet(new long[]{0x0000000000400010L});
    public static final BitSet FOLLOW_24 = new BitSet(new long[]{0x0000000000800010L});
    public static final BitSet FOLLOW_25 = new BitSet(new long[]{0x0000000002004000L});
    public static final BitSet FOLLOW_26 = new BitSet(new long[]{0x0000000102000030L});
    public static final BitSet FOLLOW_27 = new BitSet(new long[]{0x0000000008000010L});
    public static final BitSet FOLLOW_28 = new BitSet(new long[]{0x000000000C000000L});
    public static final BitSet FOLLOW_29 = new BitSet(new long[]{0x0000000100000030L});
    public static final BitSet FOLLOW_30 = new BitSet(new long[]{0x0000000100000020L});
    public static final BitSet FOLLOW_31 = new BitSet(new long[]{0x0000000000000020L});
    public static final BitSet FOLLOW_32 = new BitSet(new long[]{0x0000000040000000L});
    public static final BitSet FOLLOW_33 = new BitSet(new long[]{0x0000000080000000L});
    public static final BitSet FOLLOW_34 = new BitSet(new long[]{0x0000000008000000L});
    public static final BitSet FOLLOW_35 = new BitSet(new long[]{0x0000000100000000L});
    public static final BitSet FOLLOW_36 = new BitSet(new long[]{0x0000000004000002L});
    public static final BitSet FOLLOW_37 = new BitSet(new long[]{0x00000000000000F0L});
    public static final BitSet FOLLOW_38 = new BitSet(new long[]{0x0000000200000010L});
    public static final BitSet FOLLOW_39 = new BitSet(new long[]{0x0000000002200002L});
    public static final BitSet FOLLOW_40 = new BitSet(new long[]{0x0000000002000002L});
    public static final BitSet FOLLOW_41 = new BitSet(new long[]{0x0000000000000050L});
    public static final BitSet FOLLOW_42 = new BitSet(new long[]{0x0000000400000000L});
    public static final BitSet FOLLOW_43 = new BitSet(new long[]{0x0000000000500010L});
    public static final BitSet FOLLOW_44 = new BitSet(new long[]{0x0000000400000002L});

}