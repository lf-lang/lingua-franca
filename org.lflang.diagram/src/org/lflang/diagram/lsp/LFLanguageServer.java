package org.lflang.diagram.lsp;

import org.eclipse.lsp4j.WorkDoneProgressCancelParams;
import de.cau.cs.kieler.klighd.lsp.KGraphLanguageServerExtension;

import org.eclipse.lsp4j.Hover;
import org.eclipse.lsp4j.HoverParams;
import org.eclipse.xtext.ide.server.hover.IHoverService;
import org.eclipse.xtext.util.CancelIndicator;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.Path;
import org.lflang.generator.CodeMap;
import java.nio.charset.StandardCharsets;
import java.io.OutputStream;
import java.io.InputStream;
import com.fasterxml.jackson.databind.ObjectMapper;
// import com.fasterxml.jackson.databind.JsonFactory;
import com.fasterxml.jackson.databind.JsonNode;
import java.util.List;
import java.util.Collections;
import org.eclipse.lsp4j.jsonrpc.messages.Either;
import org.eclipse.lsp4j.MarkedString;
import java.util.Map;
import org.lflang.generator.Position;


import org.lflang.generator.GeneratorResult;

/**
 * The Lingua Franca language and diagram server.
 *
 * @author Peter Donovan <peterdonovan@berkeley.edu>
 */
public class LFLanguageServer extends KGraphLanguageServerExtension {
    @Override
    public void cancelProgress(WorkDoneProgressCancelParams params) {
        Progress.cancel(params.getToken().getRight().intValue());
    }

    @Override
    protected Hover hover(HoverParams params, CancelIndicator cancelIndicator) {
        // This override is just a hacky little patch that is being applied downstream of the original mistake and
        //  upstream of the ungraceful handling (IndexOutOfBoundsException) of said mistake. This patch is applied here
        //  simply because it is easy. This would be done differently were it not for the fact that we plan to rebuild
        //  this infrastructure from scratch anyway.
        try {
            System.out.println("got hover r");
            System.out.println(stdin);
            System.out.println(stdout);
            System.out.println(reader);
            System.out.println(writer);
            System.out.println(errorReader);
            System.out.println(initialized);
            System.out.println(pr);
            Map<Path, CodeMap> result =  LFLanguageServerExtension.getGeneratorResult().getCodeMaps();
            int currentBuildToken = LFLanguageServerExtension.getBuildToken();
            if (currentBuildToken != buildToken) {
                buildToken = currentBuildToken;
                initialized = false;
            }
            Path ccPath = Paths.get("/Users/daniil_11/lf-workspace/First/src-gen/HelloWorld/HelloWorld/HelloWorld.cc");
            CodeMap codeMap = result.get(ccPath);
            Path lfPath = Paths.get("/Users/daniil_11/lf-workspace/First/src/HelloWorld.lf");
            Position codePos = findPosition(params.getPosition().getLine(), params.getPosition().getCharacter(), lfPath, codeMap);
            System.out.println("request line is " + codePos.getOneBasedLine() + " char is " + codePos.getOneBasedColumn());
            return clangdHover(codePos);
            // return super.hover(params, cancelIndicator);
        } catch (IndexOutOfBoundsException e) {
            return IHoverService.EMPTY_HOVER;  // Fail silently
        }
    }
    OutputStream stdin = null;
    static Process pr;
    InputStream stdout = null;
    BufferedReader reader = null;
    BufferedWriter writer = null;
    BufferedReader errorReader = null;
    boolean initialized = false;
    int buildToken = -1;

    private Position findPosition(int line, int col, Path lfFile, CodeMap codeMap) {
        for (int brutLine = 1; brutLine <= 500; brutLine++) {
            for (int brutChar = 1; brutChar <= 100; brutChar++) {
                Position generatedPos = Position.fromOneBased(brutLine, brutChar);
                Position p = codeMap.adjusted(lfFile, generatedPos);
                // System.out.println(brutLine + " line and char " + brutChar + " from generated file maps to " + p.getOneBasedLine() + " and " + p.getOneBasedColumn());
                if (p.getOneBasedLine() == line && p.getOneBasedColumn() == col) return generatedPos;
            }
        }
        return Position.fromOneBased(1, 1);
    }

    private Hover clangdHover(Position params) {
        if (!initialized){
            process_init();
            // readErrorOutput();
            initialized = true;
            clangd_init();
            readOutput();
            did_open_clangd();
            readOutput();
        }
        
        send_hover_request(params.getOneBasedLine(), params.getOneBasedColumn());
        Hover resultHover = new Hover();
        try {
        String jsonResponse = readOutput();
        System.out.println("got response:");
        System.out.println(jsonResponse);
        jsonResponse = jsonResponse.substring(jsonResponse.indexOf(System.getProperty("line.separator"))+1);// remove first line since contains the header 
        jsonResponse = jsonResponse.substring(jsonResponse.indexOf(System.getProperty("line.separator"))+1);
        System.out.println("truncated is :");
        System.out.println(jsonResponse);
        ObjectMapper objectMapper = new ObjectMapper();
        JsonNode jsonNode = objectMapper.readTree(jsonResponse);
        System.out.println("whole:");
        System.out.println(jsonNode);
        String jsonValue = jsonNode.get("result").get("contents").get("value").textValue();
        List<Either<String, MarkedString>> content = Collections.singletonList(Either.forLeft(jsonValue));
        resultHover.setContents(content);
        int lvalue_c = jsonNode.get("result").get("range").get("start").get("character").intValue();
        int lvalue_l = jsonNode.get("result").get("range").get("start").get("line").intValue();
        int rvalue_c = jsonNode.get("result").get("range").get("end").get("character").intValue();
        int rvalue_l = jsonNode.get("result").get("range").get("end").get("line").intValue();
        System.out.println("parsed value: ");
        System.out.println(" " + lvalue_c + " " + lvalue_l + " " + rvalue_c + " " + rvalue_l);
        return resultHover;
        } catch (Exception e) {
            System.out.println("main failed");
        }
        return null;
    }

    //  private String readOutput() {
    //     System.out.println("reading output");
    //     try {
    //     int k = 0;
    //     char[] buf = new char[5000];
    //     int i = 0;
    //         while (k != -1) {
    //          buf[i++] = (char) reader.read();
    //          System.out.println("read: " + Character.toString(buf[i--]));
    //         if (!reader.ready()) break;
    //     }
    //     // System.out.println(buf);
    //     String returnStatus = "";
    //     // System.out.println("trying to return");
    //     for (int j= 0; j < i; j++) returnStatus += buf[j];
    //     System.out.println("read output: ");
    //     System.out.println(returnStatus);
    //     return returnStatus;
    //     } catch (Exception e) {
    //         System.out.println("failed reading clangd response");
    //     } 
    //     return "";
    // }

    // private String readOutput() {
    //     String ret = "";
    //     try {
    //     String str = null;
    //     System.out.println("reading output");
    //     while ((str = reader.readLine()) != null) {
    //         System.out.println("temp is: " + str);
    //         ret += str;
    //         Syste
    //     }
    //     } catch (Exception e) {
    //         System.out.println("failed reading output");
    //     } 
    //     System.out.println("read: " + ret);
    //     return ret;
    // }

    private String readOutput() {
        char[] chars = new char[8192];
        System.out.println("reading output");
        try {
        for(int len; (len = reader.read(chars)) > 0 && reader.ready();) {
            
        }
        System.out.println(chars);
        } catch (Exception e) {
            System.out.println("failed reading output");
        }
        return String.valueOf(chars);
    }

    // private String readOutput() {
    //     System.out.println("reading output");
    //     byte[] buffer = new byte[64];
    //     int len;
    //     String collected = "";
    //     do {
    //         try {
    //             // This depends on in.available() being
    //             //  greater than zero if data is available
    //             //  (so that all data is collected)
    //             //  and upper-bounded by maximum number of
    //             //  bytes that can be read without blocking.
    //             //  Only the latter of these two conditions
    //             //  is guaranteed by the spec.
    //             System.out.println(stdout.available());
    //             len = stdout.read(buffer, 0, Math.min(stdout.available(), buffer.length));
    //             if (len > 0) {
    //                 String temp = new String(buffer, 0, len, StandardCharsets.UTF_8);
    //                 collected += temp;
    //                 System.out.println("temp is " + temp);
    //             }
    //         } catch (Exception e) {
    //             break;
    //         }
    //     } while (len > 0); 
    //     System.out.println("output is " + collected);
    //     return collected;
    // }

    private void process_init() {
        System.out.println("initializing process");
        try {
        ProcessBuilder pb = new ProcessBuilder("clangd");
        pr = pb.start();  
        
        stdin = pr.getOutputStream();
        stdout = pr.getInputStream();
        reader = new BufferedReader (new InputStreamReader(stdout));
        writer = new BufferedWriter(new OutputStreamWriter(stdin));

        errorReader = new BufferedReader(new InputStreamReader(pr.getErrorStream()));
        } catch (Exception e) {
            System.out.println("failed process init");
        } 
    }
    
    private void readErrorOutput() {
        try {
        String str = null;
        System.out.println("reading error output");
        while ((str = errorReader.readLine()) != null && errorReader.ready()) {
            System.out.println(str);
        }
        } catch (Exception e) {
            System.out.println("failed reading error");
        } 
    }

    private void send_hover_request(int line, int character) {
        System.out.println("sending hover request");
        try {
        String body = "{\"method\": \"textDocument/hover\",\"jsonrpc\": \"2.0\",\"id\": 1,\"params\": {\"textDocument\": {\"uri\": \"file:///Users/daniil_11/lf-workspace/First/src-gen/HelloWorld/HelloWorld/HelloWorld.cc\"} ,\"position\": {\"line\": "+line+",\"character\": "+character+"}}}\r\n";
        String header = "Content-Length: " + body.length() +"\r\n\r\n";
        String cmd = header + body;
        System.out.println(cmd);
        writer.write(cmd);
        writer.flush();
        } catch (Exception e) {
            System.out.println("failed hover request");
        } 

    }

    private void clangd_init() {
        System.out.println("initializing clangd");
        try {
            String body = "{\"jsonrpc\": \"2.0\", \"method\": \"initialize\", \"id\": 1, \"params\": {\"rootUri\": \"file:///Users/daniil_11/lf-workspace/First/src-gen\", \"capabilities\": {\"hover\": {\"dynamicRegistration\": false, \"contentFormat\": []}}}}\r\n";
            String header = "Content-Length: " + body.length() +"\r\n\r\n";
            String cmd = header + body;
            System.out.println(cmd);
            
            writer.write(cmd);
            writer.flush();


        } catch (Exception e) {
            System.out.println("failed clangd init");
        } 
    }

    private void did_open_clangd() {
        System.out.println("sending did open request");

         try {
            String fileContent = Files.readString(Paths.get("/Users/daniil_11/lf-workspace/First/src-gen/HelloWorld/HelloWorld/HelloWorld.cc"), StandardCharsets.UTF_8);
            fileContent = fileContent.replaceAll("\"", "\\\\\"");
            fileContent = fileContent.replaceAll("\n", "\\\\n");
            String body = "{\"method\": \"textDocument/didOpen\", \"jsonrpc\": \"2.0\", \"params\": {\"textDocument\": {\"uri\": \"file:///Users/daniil_11/lf-workspace/First/src-gen/HelloWorld/HelloWorld/HelloWorld.cc\", \"languageId\": \"cpp\", \"version\": 1, \"text\": \""+fileContent+"\"}}}\r\n";
            String header = "Content-Length: " + (body.length()) +"\r\n\r\n";
            String cmd = header + body;
            System.out.println(cmd);
           
            writer.write(cmd);
            writer.flush();
        } catch (Exception e) {
            System.out.println("failed clangd did open");
        } 

    }
}
