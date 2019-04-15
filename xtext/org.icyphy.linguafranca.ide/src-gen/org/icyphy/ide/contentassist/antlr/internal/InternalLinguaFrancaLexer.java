package org.icyphy.ide.contentassist.antlr.internal;

// Hack: Use our own Lexer superclass by means of import. 
// Currently there is no other way to specify the superclass for the lexer.
import org.eclipse.xtext.ide.editor.contentassist.antlr.internal.Lexer;


import org.antlr.runtime.*;
import java.util.Stack;
import java.util.List;
import java.util.ArrayList;

@SuppressWarnings("all")
public class InternalLinguaFrancaLexer extends Lexer {
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

    public InternalLinguaFrancaLexer() {;} 
    public InternalLinguaFrancaLexer(CharStream input) {
        this(input, new RecognizerSharedState());
    }
    public InternalLinguaFrancaLexer(CharStream input, RecognizerSharedState state) {
        super(input,state);

    }
    public String getGrammarFileName() { return "InternalLinguaFranca.g"; }

    // $ANTLR start "T__13"
    public final void mT__13() throws RecognitionException {
        try {
            int _type = T__13;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:11:7: ( 'input' )
            // InternalLinguaFranca.g:11:9: 'input'
            {
            match("input"); 


            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__13"

    // $ANTLR start "T__14"
    public final void mT__14() throws RecognitionException {
        try {
            int _type = T__14;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:12:7: ( 'output' )
            // InternalLinguaFranca.g:12:9: 'output'
            {
            match("output"); 


            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__14"

    // $ANTLR start "T__15"
    public final void mT__15() throws RecognitionException {
        try {
            int _type = T__15;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:13:7: ( 'clock' )
            // InternalLinguaFranca.g:13:9: 'clock'
            {
            match("clock"); 


            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__15"

    // $ANTLR start "T__16"
    public final void mT__16() throws RecognitionException {
        try {
            int _type = T__16;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:14:7: ( 'target' )
            // InternalLinguaFranca.g:14:9: 'target'
            {
            match("target"); 


            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__16"

    // $ANTLR start "T__17"
    public final void mT__17() throws RecognitionException {
        try {
            int _type = T__17;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:15:7: ( ';' )
            // InternalLinguaFranca.g:15:9: ';'
            {
            match(';'); 

            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__17"

    // $ANTLR start "T__18"
    public final void mT__18() throws RecognitionException {
        try {
            int _type = T__18;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:16:7: ( 'import' )
            // InternalLinguaFranca.g:16:9: 'import'
            {
            match("import"); 


            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__18"

    // $ANTLR start "T__19"
    public final void mT__19() throws RecognitionException {
        try {
            int _type = T__19;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:17:7: ( 'reactor' )
            // InternalLinguaFranca.g:17:9: 'reactor'
            {
            match("reactor"); 


            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__19"

    // $ANTLR start "T__20"
    public final void mT__20() throws RecognitionException {
        try {
            int _type = T__20;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:18:7: ( '{' )
            // InternalLinguaFranca.g:18:9: '{'
            {
            match('{'); 

            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__20"

    // $ANTLR start "T__21"
    public final void mT__21() throws RecognitionException {
        try {
            int _type = T__21;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:19:7: ( '}' )
            // InternalLinguaFranca.g:19:9: '}'
            {
            match('}'); 

            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__21"

    // $ANTLR start "T__22"
    public final void mT__22() throws RecognitionException {
        try {
            int _type = T__22;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:20:7: ( 'composite' )
            // InternalLinguaFranca.g:20:9: 'composite'
            {
            match("composite"); 


            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__22"

    // $ANTLR start "T__23"
    public final void mT__23() throws RecognitionException {
        try {
            int _type = T__23;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:21:7: ( ':' )
            // InternalLinguaFranca.g:21:9: ':'
            {
            match(':'); 

            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__23"

    // $ANTLR start "T__24"
    public final void mT__24() throws RecognitionException {
        try {
            int _type = T__24;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:22:7: ( 'reaction' )
            // InternalLinguaFranca.g:22:9: 'reaction'
            {
            match("reaction"); 


            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__24"

    // $ANTLR start "T__25"
    public final void mT__25() throws RecognitionException {
        try {
            int _type = T__25;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:23:7: ( '(' )
            // InternalLinguaFranca.g:23:9: '('
            {
            match('('); 

            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__25"

    // $ANTLR start "T__26"
    public final void mT__26() throws RecognitionException {
        try {
            int _type = T__26;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:24:7: ( ')' )
            // InternalLinguaFranca.g:24:9: ')'
            {
            match(')'); 

            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__26"

    // $ANTLR start "T__27"
    public final void mT__27() throws RecognitionException {
        try {
            int _type = T__27;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:25:7: ( ',' )
            // InternalLinguaFranca.g:25:9: ','
            {
            match(','); 

            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__27"

    // $ANTLR start "T__28"
    public final void mT__28() throws RecognitionException {
        try {
            int _type = T__28;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:26:7: ( 'preamble' )
            // InternalLinguaFranca.g:26:9: 'preamble'
            {
            match("preamble"); 


            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__28"

    // $ANTLR start "T__29"
    public final void mT__29() throws RecognitionException {
        try {
            int _type = T__29;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:27:7: ( 'constructor' )
            // InternalLinguaFranca.g:27:9: 'constructor'
            {
            match("constructor"); 


            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__29"

    // $ANTLR start "T__30"
    public final void mT__30() throws RecognitionException {
        try {
            int _type = T__30;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:28:7: ( 'instance' )
            // InternalLinguaFranca.g:28:9: 'instance'
            {
            match("instance"); 


            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__30"

    // $ANTLR start "T__31"
    public final void mT__31() throws RecognitionException {
        try {
            int _type = T__31;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:29:7: ( '=' )
            // InternalLinguaFranca.g:29:9: '='
            {
            match('='); 

            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__31"

    // $ANTLR start "T__32"
    public final void mT__32() throws RecognitionException {
        try {
            int _type = T__32;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:30:7: ( '->' )
            // InternalLinguaFranca.g:30:9: '->'
            {
            match("->"); 


            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__32"

    // $ANTLR start "T__33"
    public final void mT__33() throws RecognitionException {
        try {
            int _type = T__33;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:31:7: ( 'const' )
            // InternalLinguaFranca.g:31:9: 'const'
            {
            match("const"); 


            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__33"

    // $ANTLR start "T__34"
    public final void mT__34() throws RecognitionException {
        try {
            int _type = T__34;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:32:7: ( '.' )
            // InternalLinguaFranca.g:32:9: '.'
            {
            match('.'); 

            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__34"

    // $ANTLR start "T__35"
    public final void mT__35() throws RecognitionException {
        try {
            int _type = T__35;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:33:7: ( 'PERIODIC' )
            // InternalLinguaFranca.g:33:9: 'PERIODIC'
            {
            match("PERIODIC"); 


            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__35"

    // $ANTLR start "T__36"
    public final void mT__36() throws RecognitionException {
        try {
            int _type = T__36;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:34:7: ( 'ONCE' )
            // InternalLinguaFranca.g:34:9: 'ONCE'
            {
            match("ONCE"); 


            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "T__36"

    // $ANTLR start "RULE_NUMBER"
    public final void mRULE_NUMBER() throws RecognitionException {
        try {
            int _type = RULE_NUMBER;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:5044:13: ( ( '-' )? RULE_INT ( '.' RULE_INT )? )
            // InternalLinguaFranca.g:5044:15: ( '-' )? RULE_INT ( '.' RULE_INT )?
            {
            // InternalLinguaFranca.g:5044:15: ( '-' )?
            int alt1=2;
            int LA1_0 = input.LA(1);

            if ( (LA1_0=='-') ) {
                alt1=1;
            }
            switch (alt1) {
                case 1 :
                    // InternalLinguaFranca.g:5044:15: '-'
                    {
                    match('-'); 

                    }
                    break;

            }

            mRULE_INT(); 
            // InternalLinguaFranca.g:5044:29: ( '.' RULE_INT )?
            int alt2=2;
            int LA2_0 = input.LA(1);

            if ( (LA2_0=='.') ) {
                alt2=1;
            }
            switch (alt2) {
                case 1 :
                    // InternalLinguaFranca.g:5044:30: '.' RULE_INT
                    {
                    match('.'); 
                    mRULE_INT(); 

                    }
                    break;

            }


            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "RULE_NUMBER"

    // $ANTLR start "RULE_CODE"
    public final void mRULE_CODE() throws RecognitionException {
        try {
            int _type = RULE_CODE;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:5046:11: ( '{=' ( . )* '=}' )
            // InternalLinguaFranca.g:5046:13: '{=' ( . )* '=}'
            {
            match("{="); 

            // InternalLinguaFranca.g:5046:18: ( . )*
            loop3:
            do {
                int alt3=2;
                int LA3_0 = input.LA(1);

                if ( (LA3_0=='=') ) {
                    int LA3_1 = input.LA(2);

                    if ( (LA3_1=='}') ) {
                        alt3=2;
                    }
                    else if ( ((LA3_1>='\u0000' && LA3_1<='|')||(LA3_1>='~' && LA3_1<='\uFFFF')) ) {
                        alt3=1;
                    }


                }
                else if ( ((LA3_0>='\u0000' && LA3_0<='<')||(LA3_0>='>' && LA3_0<='\uFFFF')) ) {
                    alt3=1;
                }


                switch (alt3) {
            	case 1 :
            	    // InternalLinguaFranca.g:5046:18: .
            	    {
            	    matchAny(); 

            	    }
            	    break;

            	default :
            	    break loop3;
                }
            } while (true);

            match("=}"); 


            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "RULE_CODE"

    // $ANTLR start "RULE_ID"
    public final void mRULE_ID() throws RecognitionException {
        try {
            int _type = RULE_ID;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:5048:9: ( ( '^' )? ( 'a' .. 'z' | 'A' .. 'Z' | '_' ) ( 'a' .. 'z' | 'A' .. 'Z' | '_' | '0' .. '9' )* )
            // InternalLinguaFranca.g:5048:11: ( '^' )? ( 'a' .. 'z' | 'A' .. 'Z' | '_' ) ( 'a' .. 'z' | 'A' .. 'Z' | '_' | '0' .. '9' )*
            {
            // InternalLinguaFranca.g:5048:11: ( '^' )?
            int alt4=2;
            int LA4_0 = input.LA(1);

            if ( (LA4_0=='^') ) {
                alt4=1;
            }
            switch (alt4) {
                case 1 :
                    // InternalLinguaFranca.g:5048:11: '^'
                    {
                    match('^'); 

                    }
                    break;

            }

            if ( (input.LA(1)>='A' && input.LA(1)<='Z')||input.LA(1)=='_'||(input.LA(1)>='a' && input.LA(1)<='z') ) {
                input.consume();

            }
            else {
                MismatchedSetException mse = new MismatchedSetException(null,input);
                recover(mse);
                throw mse;}

            // InternalLinguaFranca.g:5048:40: ( 'a' .. 'z' | 'A' .. 'Z' | '_' | '0' .. '9' )*
            loop5:
            do {
                int alt5=2;
                int LA5_0 = input.LA(1);

                if ( ((LA5_0>='0' && LA5_0<='9')||(LA5_0>='A' && LA5_0<='Z')||LA5_0=='_'||(LA5_0>='a' && LA5_0<='z')) ) {
                    alt5=1;
                }


                switch (alt5) {
            	case 1 :
            	    // InternalLinguaFranca.g:
            	    {
            	    if ( (input.LA(1)>='0' && input.LA(1)<='9')||(input.LA(1)>='A' && input.LA(1)<='Z')||input.LA(1)=='_'||(input.LA(1)>='a' && input.LA(1)<='z') ) {
            	        input.consume();

            	    }
            	    else {
            	        MismatchedSetException mse = new MismatchedSetException(null,input);
            	        recover(mse);
            	        throw mse;}


            	    }
            	    break;

            	default :
            	    break loop5;
                }
            } while (true);


            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "RULE_ID"

    // $ANTLR start "RULE_INT"
    public final void mRULE_INT() throws RecognitionException {
        try {
            // InternalLinguaFranca.g:5050:19: ( ( '0' .. '9' )+ )
            // InternalLinguaFranca.g:5050:21: ( '0' .. '9' )+
            {
            // InternalLinguaFranca.g:5050:21: ( '0' .. '9' )+
            int cnt6=0;
            loop6:
            do {
                int alt6=2;
                int LA6_0 = input.LA(1);

                if ( ((LA6_0>='0' && LA6_0<='9')) ) {
                    alt6=1;
                }


                switch (alt6) {
            	case 1 :
            	    // InternalLinguaFranca.g:5050:22: '0' .. '9'
            	    {
            	    matchRange('0','9'); 

            	    }
            	    break;

            	default :
            	    if ( cnt6 >= 1 ) break loop6;
                        EarlyExitException eee =
                            new EarlyExitException(6, input);
                        throw eee;
                }
                cnt6++;
            } while (true);


            }

        }
        finally {
        }
    }
    // $ANTLR end "RULE_INT"

    // $ANTLR start "RULE_STRING"
    public final void mRULE_STRING() throws RecognitionException {
        try {
            int _type = RULE_STRING;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:5052:13: ( ( '\"' ( '\\\\' . | ~ ( ( '\\\\' | '\"' ) ) )* '\"' | '\\'' ( '\\\\' . | ~ ( ( '\\\\' | '\\'' ) ) )* '\\'' ) )
            // InternalLinguaFranca.g:5052:15: ( '\"' ( '\\\\' . | ~ ( ( '\\\\' | '\"' ) ) )* '\"' | '\\'' ( '\\\\' . | ~ ( ( '\\\\' | '\\'' ) ) )* '\\'' )
            {
            // InternalLinguaFranca.g:5052:15: ( '\"' ( '\\\\' . | ~ ( ( '\\\\' | '\"' ) ) )* '\"' | '\\'' ( '\\\\' . | ~ ( ( '\\\\' | '\\'' ) ) )* '\\'' )
            int alt9=2;
            int LA9_0 = input.LA(1);

            if ( (LA9_0=='\"') ) {
                alt9=1;
            }
            else if ( (LA9_0=='\'') ) {
                alt9=2;
            }
            else {
                NoViableAltException nvae =
                    new NoViableAltException("", 9, 0, input);

                throw nvae;
            }
            switch (alt9) {
                case 1 :
                    // InternalLinguaFranca.g:5052:16: '\"' ( '\\\\' . | ~ ( ( '\\\\' | '\"' ) ) )* '\"'
                    {
                    match('\"'); 
                    // InternalLinguaFranca.g:5052:20: ( '\\\\' . | ~ ( ( '\\\\' | '\"' ) ) )*
                    loop7:
                    do {
                        int alt7=3;
                        int LA7_0 = input.LA(1);

                        if ( (LA7_0=='\\') ) {
                            alt7=1;
                        }
                        else if ( ((LA7_0>='\u0000' && LA7_0<='!')||(LA7_0>='#' && LA7_0<='[')||(LA7_0>=']' && LA7_0<='\uFFFF')) ) {
                            alt7=2;
                        }


                        switch (alt7) {
                    	case 1 :
                    	    // InternalLinguaFranca.g:5052:21: '\\\\' .
                    	    {
                    	    match('\\'); 
                    	    matchAny(); 

                    	    }
                    	    break;
                    	case 2 :
                    	    // InternalLinguaFranca.g:5052:28: ~ ( ( '\\\\' | '\"' ) )
                    	    {
                    	    if ( (input.LA(1)>='\u0000' && input.LA(1)<='!')||(input.LA(1)>='#' && input.LA(1)<='[')||(input.LA(1)>=']' && input.LA(1)<='\uFFFF') ) {
                    	        input.consume();

                    	    }
                    	    else {
                    	        MismatchedSetException mse = new MismatchedSetException(null,input);
                    	        recover(mse);
                    	        throw mse;}


                    	    }
                    	    break;

                    	default :
                    	    break loop7;
                        }
                    } while (true);

                    match('\"'); 

                    }
                    break;
                case 2 :
                    // InternalLinguaFranca.g:5052:48: '\\'' ( '\\\\' . | ~ ( ( '\\\\' | '\\'' ) ) )* '\\''
                    {
                    match('\''); 
                    // InternalLinguaFranca.g:5052:53: ( '\\\\' . | ~ ( ( '\\\\' | '\\'' ) ) )*
                    loop8:
                    do {
                        int alt8=3;
                        int LA8_0 = input.LA(1);

                        if ( (LA8_0=='\\') ) {
                            alt8=1;
                        }
                        else if ( ((LA8_0>='\u0000' && LA8_0<='&')||(LA8_0>='(' && LA8_0<='[')||(LA8_0>=']' && LA8_0<='\uFFFF')) ) {
                            alt8=2;
                        }


                        switch (alt8) {
                    	case 1 :
                    	    // InternalLinguaFranca.g:5052:54: '\\\\' .
                    	    {
                    	    match('\\'); 
                    	    matchAny(); 

                    	    }
                    	    break;
                    	case 2 :
                    	    // InternalLinguaFranca.g:5052:61: ~ ( ( '\\\\' | '\\'' ) )
                    	    {
                    	    if ( (input.LA(1)>='\u0000' && input.LA(1)<='&')||(input.LA(1)>='(' && input.LA(1)<='[')||(input.LA(1)>=']' && input.LA(1)<='\uFFFF') ) {
                    	        input.consume();

                    	    }
                    	    else {
                    	        MismatchedSetException mse = new MismatchedSetException(null,input);
                    	        recover(mse);
                    	        throw mse;}


                    	    }
                    	    break;

                    	default :
                    	    break loop8;
                        }
                    } while (true);

                    match('\''); 

                    }
                    break;

            }


            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "RULE_STRING"

    // $ANTLR start "RULE_ML_COMMENT"
    public final void mRULE_ML_COMMENT() throws RecognitionException {
        try {
            int _type = RULE_ML_COMMENT;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:5054:17: ( '/*' ( options {greedy=false; } : . )* '*/' )
            // InternalLinguaFranca.g:5054:19: '/*' ( options {greedy=false; } : . )* '*/'
            {
            match("/*"); 

            // InternalLinguaFranca.g:5054:24: ( options {greedy=false; } : . )*
            loop10:
            do {
                int alt10=2;
                int LA10_0 = input.LA(1);

                if ( (LA10_0=='*') ) {
                    int LA10_1 = input.LA(2);

                    if ( (LA10_1=='/') ) {
                        alt10=2;
                    }
                    else if ( ((LA10_1>='\u0000' && LA10_1<='.')||(LA10_1>='0' && LA10_1<='\uFFFF')) ) {
                        alt10=1;
                    }


                }
                else if ( ((LA10_0>='\u0000' && LA10_0<=')')||(LA10_0>='+' && LA10_0<='\uFFFF')) ) {
                    alt10=1;
                }


                switch (alt10) {
            	case 1 :
            	    // InternalLinguaFranca.g:5054:52: .
            	    {
            	    matchAny(); 

            	    }
            	    break;

            	default :
            	    break loop10;
                }
            } while (true);

            match("*/"); 


            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "RULE_ML_COMMENT"

    // $ANTLR start "RULE_SL_COMMENT"
    public final void mRULE_SL_COMMENT() throws RecognitionException {
        try {
            int _type = RULE_SL_COMMENT;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:5056:17: ( '//' (~ ( ( '\\n' | '\\r' ) ) )* ( ( '\\r' )? '\\n' )? )
            // InternalLinguaFranca.g:5056:19: '//' (~ ( ( '\\n' | '\\r' ) ) )* ( ( '\\r' )? '\\n' )?
            {
            match("//"); 

            // InternalLinguaFranca.g:5056:24: (~ ( ( '\\n' | '\\r' ) ) )*
            loop11:
            do {
                int alt11=2;
                int LA11_0 = input.LA(1);

                if ( ((LA11_0>='\u0000' && LA11_0<='\t')||(LA11_0>='\u000B' && LA11_0<='\f')||(LA11_0>='\u000E' && LA11_0<='\uFFFF')) ) {
                    alt11=1;
                }


                switch (alt11) {
            	case 1 :
            	    // InternalLinguaFranca.g:5056:24: ~ ( ( '\\n' | '\\r' ) )
            	    {
            	    if ( (input.LA(1)>='\u0000' && input.LA(1)<='\t')||(input.LA(1)>='\u000B' && input.LA(1)<='\f')||(input.LA(1)>='\u000E' && input.LA(1)<='\uFFFF') ) {
            	        input.consume();

            	    }
            	    else {
            	        MismatchedSetException mse = new MismatchedSetException(null,input);
            	        recover(mse);
            	        throw mse;}


            	    }
            	    break;

            	default :
            	    break loop11;
                }
            } while (true);

            // InternalLinguaFranca.g:5056:40: ( ( '\\r' )? '\\n' )?
            int alt13=2;
            int LA13_0 = input.LA(1);

            if ( (LA13_0=='\n'||LA13_0=='\r') ) {
                alt13=1;
            }
            switch (alt13) {
                case 1 :
                    // InternalLinguaFranca.g:5056:41: ( '\\r' )? '\\n'
                    {
                    // InternalLinguaFranca.g:5056:41: ( '\\r' )?
                    int alt12=2;
                    int LA12_0 = input.LA(1);

                    if ( (LA12_0=='\r') ) {
                        alt12=1;
                    }
                    switch (alt12) {
                        case 1 :
                            // InternalLinguaFranca.g:5056:41: '\\r'
                            {
                            match('\r'); 

                            }
                            break;

                    }

                    match('\n'); 

                    }
                    break;

            }


            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "RULE_SL_COMMENT"

    // $ANTLR start "RULE_WS"
    public final void mRULE_WS() throws RecognitionException {
        try {
            int _type = RULE_WS;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:5058:9: ( ( ' ' | '\\t' | '\\r' | '\\n' )+ )
            // InternalLinguaFranca.g:5058:11: ( ' ' | '\\t' | '\\r' | '\\n' )+
            {
            // InternalLinguaFranca.g:5058:11: ( ' ' | '\\t' | '\\r' | '\\n' )+
            int cnt14=0;
            loop14:
            do {
                int alt14=2;
                int LA14_0 = input.LA(1);

                if ( ((LA14_0>='\t' && LA14_0<='\n')||LA14_0=='\r'||LA14_0==' ') ) {
                    alt14=1;
                }


                switch (alt14) {
            	case 1 :
            	    // InternalLinguaFranca.g:
            	    {
            	    if ( (input.LA(1)>='\t' && input.LA(1)<='\n')||input.LA(1)=='\r'||input.LA(1)==' ' ) {
            	        input.consume();

            	    }
            	    else {
            	        MismatchedSetException mse = new MismatchedSetException(null,input);
            	        recover(mse);
            	        throw mse;}


            	    }
            	    break;

            	default :
            	    if ( cnt14 >= 1 ) break loop14;
                        EarlyExitException eee =
                            new EarlyExitException(14, input);
                        throw eee;
                }
                cnt14++;
            } while (true);


            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "RULE_WS"

    // $ANTLR start "RULE_ANY_OTHER"
    public final void mRULE_ANY_OTHER() throws RecognitionException {
        try {
            int _type = RULE_ANY_OTHER;
            int _channel = DEFAULT_TOKEN_CHANNEL;
            // InternalLinguaFranca.g:5060:16: ( . )
            // InternalLinguaFranca.g:5060:18: .
            {
            matchAny(); 

            }

            state.type = _type;
            state.channel = _channel;
        }
        finally {
        }
    }
    // $ANTLR end "RULE_ANY_OTHER"

    public void mTokens() throws RecognitionException {
        // InternalLinguaFranca.g:1:8: ( T__13 | T__14 | T__15 | T__16 | T__17 | T__18 | T__19 | T__20 | T__21 | T__22 | T__23 | T__24 | T__25 | T__26 | T__27 | T__28 | T__29 | T__30 | T__31 | T__32 | T__33 | T__34 | T__35 | T__36 | RULE_NUMBER | RULE_CODE | RULE_ID | RULE_STRING | RULE_ML_COMMENT | RULE_SL_COMMENT | RULE_WS | RULE_ANY_OTHER )
        int alt15=32;
        alt15 = dfa15.predict(input);
        switch (alt15) {
            case 1 :
                // InternalLinguaFranca.g:1:10: T__13
                {
                mT__13(); 

                }
                break;
            case 2 :
                // InternalLinguaFranca.g:1:16: T__14
                {
                mT__14(); 

                }
                break;
            case 3 :
                // InternalLinguaFranca.g:1:22: T__15
                {
                mT__15(); 

                }
                break;
            case 4 :
                // InternalLinguaFranca.g:1:28: T__16
                {
                mT__16(); 

                }
                break;
            case 5 :
                // InternalLinguaFranca.g:1:34: T__17
                {
                mT__17(); 

                }
                break;
            case 6 :
                // InternalLinguaFranca.g:1:40: T__18
                {
                mT__18(); 

                }
                break;
            case 7 :
                // InternalLinguaFranca.g:1:46: T__19
                {
                mT__19(); 

                }
                break;
            case 8 :
                // InternalLinguaFranca.g:1:52: T__20
                {
                mT__20(); 

                }
                break;
            case 9 :
                // InternalLinguaFranca.g:1:58: T__21
                {
                mT__21(); 

                }
                break;
            case 10 :
                // InternalLinguaFranca.g:1:64: T__22
                {
                mT__22(); 

                }
                break;
            case 11 :
                // InternalLinguaFranca.g:1:70: T__23
                {
                mT__23(); 

                }
                break;
            case 12 :
                // InternalLinguaFranca.g:1:76: T__24
                {
                mT__24(); 

                }
                break;
            case 13 :
                // InternalLinguaFranca.g:1:82: T__25
                {
                mT__25(); 

                }
                break;
            case 14 :
                // InternalLinguaFranca.g:1:88: T__26
                {
                mT__26(); 

                }
                break;
            case 15 :
                // InternalLinguaFranca.g:1:94: T__27
                {
                mT__27(); 

                }
                break;
            case 16 :
                // InternalLinguaFranca.g:1:100: T__28
                {
                mT__28(); 

                }
                break;
            case 17 :
                // InternalLinguaFranca.g:1:106: T__29
                {
                mT__29(); 

                }
                break;
            case 18 :
                // InternalLinguaFranca.g:1:112: T__30
                {
                mT__30(); 

                }
                break;
            case 19 :
                // InternalLinguaFranca.g:1:118: T__31
                {
                mT__31(); 

                }
                break;
            case 20 :
                // InternalLinguaFranca.g:1:124: T__32
                {
                mT__32(); 

                }
                break;
            case 21 :
                // InternalLinguaFranca.g:1:130: T__33
                {
                mT__33(); 

                }
                break;
            case 22 :
                // InternalLinguaFranca.g:1:136: T__34
                {
                mT__34(); 

                }
                break;
            case 23 :
                // InternalLinguaFranca.g:1:142: T__35
                {
                mT__35(); 

                }
                break;
            case 24 :
                // InternalLinguaFranca.g:1:148: T__36
                {
                mT__36(); 

                }
                break;
            case 25 :
                // InternalLinguaFranca.g:1:154: RULE_NUMBER
                {
                mRULE_NUMBER(); 

                }
                break;
            case 26 :
                // InternalLinguaFranca.g:1:166: RULE_CODE
                {
                mRULE_CODE(); 

                }
                break;
            case 27 :
                // InternalLinguaFranca.g:1:176: RULE_ID
                {
                mRULE_ID(); 

                }
                break;
            case 28 :
                // InternalLinguaFranca.g:1:184: RULE_STRING
                {
                mRULE_STRING(); 

                }
                break;
            case 29 :
                // InternalLinguaFranca.g:1:196: RULE_ML_COMMENT
                {
                mRULE_ML_COMMENT(); 

                }
                break;
            case 30 :
                // InternalLinguaFranca.g:1:212: RULE_SL_COMMENT
                {
                mRULE_SL_COMMENT(); 

                }
                break;
            case 31 :
                // InternalLinguaFranca.g:1:228: RULE_WS
                {
                mRULE_WS(); 

                }
                break;
            case 32 :
                // InternalLinguaFranca.g:1:236: RULE_ANY_OTHER
                {
                mRULE_ANY_OTHER(); 

                }
                break;

        }

    }


    protected DFA15 dfa15 = new DFA15(this);
    static final String DFA15_eotS =
        "\1\uffff\4\35\1\uffff\1\35\1\45\5\uffff\1\35\1\uffff\1\32\1\uffff\2\35\1\uffff\1\32\1\uffff\3\32\2\uffff\2\35\1\uffff\4\35\1\uffff\1\35\7\uffff\1\35\4\uffff\2\35\4\uffff\27\35\1\131\1\132\3\35\1\136\1\35\1\141\4\35\2\uffff\1\35\1\150\1\151\1\uffff\2\35\1\uffff\1\154\5\35\2\uffff\2\35\1\uffff\1\164\3\35\1\170\2\35\1\uffff\1\173\1\174\1\175\1\uffff\1\176\1\35\4\uffff\1\35\1\u0081\1\uffff";
    static final String DFA15_eofS =
        "\u0082\uffff";
    static final String DFA15_minS =
        "\1\0\1\155\1\165\1\154\1\141\1\uffff\1\145\1\75\5\uffff\1\162\1\uffff\1\60\1\uffff\1\105\1\116\1\uffff\1\101\1\uffff\2\0\1\52\2\uffff\2\160\1\uffff\1\164\1\157\1\155\1\162\1\uffff\1\141\7\uffff\1\145\4\uffff\1\122\1\103\4\uffff\1\165\1\164\1\157\1\160\1\143\1\160\1\163\1\147\1\143\1\141\1\111\1\105\1\164\1\141\1\162\1\165\1\153\1\157\1\164\1\145\1\164\1\155\1\117\2\60\1\156\2\164\1\60\1\163\1\60\1\164\1\151\1\142\1\104\2\uffff\1\143\2\60\1\uffff\1\151\1\165\1\uffff\1\60\1\162\1\157\1\154\1\111\1\145\2\uffff\1\164\1\143\1\uffff\1\60\1\156\1\145\1\103\1\60\1\145\1\164\1\uffff\3\60\1\uffff\1\60\1\157\4\uffff\1\162\1\60\1\uffff";
    static final String DFA15_maxS =
        "\1\uffff\1\156\1\165\1\157\1\141\1\uffff\1\145\1\75\5\uffff\1\162\1\uffff\1\76\1\uffff\1\105\1\116\1\uffff\1\172\1\uffff\2\uffff\1\57\2\uffff\1\163\1\160\1\uffff\1\164\1\157\1\156\1\162\1\uffff\1\141\7\uffff\1\145\4\uffff\1\122\1\103\4\uffff\1\165\1\164\1\157\1\160\1\143\1\160\1\163\1\147\1\143\1\141\1\111\1\105\1\164\1\141\1\162\1\165\1\153\1\157\1\164\1\145\1\164\1\155\1\117\2\172\1\156\2\164\1\172\1\163\1\172\1\164\1\157\1\142\1\104\2\uffff\1\143\2\172\1\uffff\1\151\1\165\1\uffff\1\172\1\162\1\157\1\154\1\111\1\145\2\uffff\1\164\1\143\1\uffff\1\172\1\156\1\145\1\103\1\172\1\145\1\164\1\uffff\3\172\1\uffff\1\172\1\157\4\uffff\1\162\1\172\1\uffff";
    static final String DFA15_acceptS =
        "\5\uffff\1\5\2\uffff\1\11\1\13\1\15\1\16\1\17\1\uffff\1\23\1\uffff\1\26\2\uffff\1\31\1\uffff\1\33\3\uffff\1\37\1\40\2\uffff\1\33\4\uffff\1\5\1\uffff\1\32\1\10\1\11\1\13\1\15\1\16\1\17\1\uffff\1\23\1\24\1\31\1\26\2\uffff\1\34\1\35\1\36\1\37\43\uffff\1\30\1\1\3\uffff\1\3\2\uffff\1\25\6\uffff\1\6\1\2\2\uffff\1\4\7\uffff\1\7\3\uffff\1\22\2\uffff\1\14\1\20\1\27\1\12\2\uffff\1\21";
    static final String DFA15_specialS =
        "\1\1\25\uffff\1\2\1\0\152\uffff}>";
    static final String[] DFA15_transitionS = {
            "\11\32\2\31\2\32\1\31\22\32\1\31\1\32\1\26\4\32\1\27\1\12\1\13\2\32\1\14\1\17\1\20\1\30\12\23\1\11\1\5\1\32\1\16\3\32\16\25\1\22\1\21\12\25\3\32\1\24\1\25\1\32\2\25\1\3\5\25\1\1\5\25\1\2\1\15\1\25\1\6\1\25\1\4\6\25\1\7\1\32\1\10\uff82\32",
            "\1\34\1\33",
            "\1\36",
            "\1\37\2\uffff\1\40",
            "\1\41",
            "",
            "\1\43",
            "\1\44",
            "",
            "",
            "",
            "",
            "",
            "\1\53",
            "",
            "\12\56\4\uffff\1\55",
            "",
            "\1\60",
            "\1\61",
            "",
            "\32\35\4\uffff\1\35\1\uffff\32\35",
            "",
            "\0\62",
            "\0\62",
            "\1\63\4\uffff\1\64",
            "",
            "",
            "\1\66\2\uffff\1\67",
            "\1\70",
            "",
            "\1\71",
            "\1\72",
            "\1\73\1\74",
            "\1\75",
            "",
            "\1\76",
            "",
            "",
            "",
            "",
            "",
            "",
            "",
            "\1\77",
            "",
            "",
            "",
            "",
            "\1\100",
            "\1\101",
            "",
            "",
            "",
            "",
            "\1\102",
            "\1\103",
            "\1\104",
            "\1\105",
            "\1\106",
            "\1\107",
            "\1\110",
            "\1\111",
            "\1\112",
            "\1\113",
            "\1\114",
            "\1\115",
            "\1\116",
            "\1\117",
            "\1\120",
            "\1\121",
            "\1\122",
            "\1\123",
            "\1\124",
            "\1\125",
            "\1\126",
            "\1\127",
            "\1\130",
            "\12\35\7\uffff\32\35\4\uffff\1\35\1\uffff\32\35",
            "\12\35\7\uffff\32\35\4\uffff\1\35\1\uffff\32\35",
            "\1\133",
            "\1\134",
            "\1\135",
            "\12\35\7\uffff\32\35\4\uffff\1\35\1\uffff\32\35",
            "\1\137",
            "\12\35\7\uffff\32\35\4\uffff\1\35\1\uffff\21\35\1\140\10\35",
            "\1\142",
            "\1\144\5\uffff\1\143",
            "\1\145",
            "\1\146",
            "",
            "",
            "\1\147",
            "\12\35\7\uffff\32\35\4\uffff\1\35\1\uffff\32\35",
            "\12\35\7\uffff\32\35\4\uffff\1\35\1\uffff\32\35",
            "",
            "\1\152",
            "\1\153",
            "",
            "\12\35\7\uffff\32\35\4\uffff\1\35\1\uffff\32\35",
            "\1\155",
            "\1\156",
            "\1\157",
            "\1\160",
            "\1\161",
            "",
            "",
            "\1\162",
            "\1\163",
            "",
            "\12\35\7\uffff\32\35\4\uffff\1\35\1\uffff\32\35",
            "\1\165",
            "\1\166",
            "\1\167",
            "\12\35\7\uffff\32\35\4\uffff\1\35\1\uffff\32\35",
            "\1\171",
            "\1\172",
            "",
            "\12\35\7\uffff\32\35\4\uffff\1\35\1\uffff\32\35",
            "\12\35\7\uffff\32\35\4\uffff\1\35\1\uffff\32\35",
            "\12\35\7\uffff\32\35\4\uffff\1\35\1\uffff\32\35",
            "",
            "\12\35\7\uffff\32\35\4\uffff\1\35\1\uffff\32\35",
            "\1\177",
            "",
            "",
            "",
            "",
            "\1\u0080",
            "\12\35\7\uffff\32\35\4\uffff\1\35\1\uffff\32\35",
            ""
    };

    static final short[] DFA15_eot = DFA.unpackEncodedString(DFA15_eotS);
    static final short[] DFA15_eof = DFA.unpackEncodedString(DFA15_eofS);
    static final char[] DFA15_min = DFA.unpackEncodedStringToUnsignedChars(DFA15_minS);
    static final char[] DFA15_max = DFA.unpackEncodedStringToUnsignedChars(DFA15_maxS);
    static final short[] DFA15_accept = DFA.unpackEncodedString(DFA15_acceptS);
    static final short[] DFA15_special = DFA.unpackEncodedString(DFA15_specialS);
    static final short[][] DFA15_transition;

    static {
        int numStates = DFA15_transitionS.length;
        DFA15_transition = new short[numStates][];
        for (int i=0; i<numStates; i++) {
            DFA15_transition[i] = DFA.unpackEncodedString(DFA15_transitionS[i]);
        }
    }

    class DFA15 extends DFA {

        public DFA15(BaseRecognizer recognizer) {
            this.recognizer = recognizer;
            this.decisionNumber = 15;
            this.eot = DFA15_eot;
            this.eof = DFA15_eof;
            this.min = DFA15_min;
            this.max = DFA15_max;
            this.accept = DFA15_accept;
            this.special = DFA15_special;
            this.transition = DFA15_transition;
        }
        public String getDescription() {
            return "1:1: Tokens : ( T__13 | T__14 | T__15 | T__16 | T__17 | T__18 | T__19 | T__20 | T__21 | T__22 | T__23 | T__24 | T__25 | T__26 | T__27 | T__28 | T__29 | T__30 | T__31 | T__32 | T__33 | T__34 | T__35 | T__36 | RULE_NUMBER | RULE_CODE | RULE_ID | RULE_STRING | RULE_ML_COMMENT | RULE_SL_COMMENT | RULE_WS | RULE_ANY_OTHER );";
        }
        public int specialStateTransition(int s, IntStream _input) throws NoViableAltException {
            IntStream input = _input;
        	int _s = s;
            switch ( s ) {
                    case 0 : 
                        int LA15_23 = input.LA(1);

                        s = -1;
                        if ( ((LA15_23>='\u0000' && LA15_23<='\uFFFF')) ) {s = 50;}

                        else s = 26;

                        if ( s>=0 ) return s;
                        break;
                    case 1 : 
                        int LA15_0 = input.LA(1);

                        s = -1;
                        if ( (LA15_0=='i') ) {s = 1;}

                        else if ( (LA15_0=='o') ) {s = 2;}

                        else if ( (LA15_0=='c') ) {s = 3;}

                        else if ( (LA15_0=='t') ) {s = 4;}

                        else if ( (LA15_0==';') ) {s = 5;}

                        else if ( (LA15_0=='r') ) {s = 6;}

                        else if ( (LA15_0=='{') ) {s = 7;}

                        else if ( (LA15_0=='}') ) {s = 8;}

                        else if ( (LA15_0==':') ) {s = 9;}

                        else if ( (LA15_0=='(') ) {s = 10;}

                        else if ( (LA15_0==')') ) {s = 11;}

                        else if ( (LA15_0==',') ) {s = 12;}

                        else if ( (LA15_0=='p') ) {s = 13;}

                        else if ( (LA15_0=='=') ) {s = 14;}

                        else if ( (LA15_0=='-') ) {s = 15;}

                        else if ( (LA15_0=='.') ) {s = 16;}

                        else if ( (LA15_0=='P') ) {s = 17;}

                        else if ( (LA15_0=='O') ) {s = 18;}

                        else if ( ((LA15_0>='0' && LA15_0<='9')) ) {s = 19;}

                        else if ( (LA15_0=='^') ) {s = 20;}

                        else if ( ((LA15_0>='A' && LA15_0<='N')||(LA15_0>='Q' && LA15_0<='Z')||LA15_0=='_'||(LA15_0>='a' && LA15_0<='b')||(LA15_0>='d' && LA15_0<='h')||(LA15_0>='j' && LA15_0<='n')||LA15_0=='q'||LA15_0=='s'||(LA15_0>='u' && LA15_0<='z')) ) {s = 21;}

                        else if ( (LA15_0=='\"') ) {s = 22;}

                        else if ( (LA15_0=='\'') ) {s = 23;}

                        else if ( (LA15_0=='/') ) {s = 24;}

                        else if ( ((LA15_0>='\t' && LA15_0<='\n')||LA15_0=='\r'||LA15_0==' ') ) {s = 25;}

                        else if ( ((LA15_0>='\u0000' && LA15_0<='\b')||(LA15_0>='\u000B' && LA15_0<='\f')||(LA15_0>='\u000E' && LA15_0<='\u001F')||LA15_0=='!'||(LA15_0>='#' && LA15_0<='&')||(LA15_0>='*' && LA15_0<='+')||LA15_0=='<'||(LA15_0>='>' && LA15_0<='@')||(LA15_0>='[' && LA15_0<=']')||LA15_0=='`'||LA15_0=='|'||(LA15_0>='~' && LA15_0<='\uFFFF')) ) {s = 26;}

                        if ( s>=0 ) return s;
                        break;
                    case 2 : 
                        int LA15_22 = input.LA(1);

                        s = -1;
                        if ( ((LA15_22>='\u0000' && LA15_22<='\uFFFF')) ) {s = 50;}

                        else s = 26;

                        if ( s>=0 ) return s;
                        break;
            }
            NoViableAltException nvae =
                new NoViableAltException(getDescription(), 15, _s, input);
            error(nvae);
            throw nvae;
        }
    }
 

}