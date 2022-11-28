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
import com.fasterxml.jackson.databind.JsonNode;
import java.util.List;
import java.util.Collections;
import org.eclipse.lsp4j.jsonrpc.messages.Either;
import org.eclipse.lsp4j.MarkedString;
import java.util.Map;
import org.lflang.generator.Position;
import org.lflang.generator.Range;
import java.util.NavigableMap;
import java.util.HashMap;
// import javafx.util.Pair;


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
            int currentBuildToken = LFLanguageServerExtension.getBuildToken();
            Map<Path, CodeMap> result =  LFLanguageServerExtension.getGeneratorResult().getCodeMaps();
            generatedDirectory = LFLanguageServerExtension.getGeneratorResult().getCommand().directory().toString() + "/src-gen";
            if (currentBuildToken != buildToken) {
                buildToken = currentBuildToken;
                initialized = false;
                // getCodeMaps();
            
            }
            Path lfPath = LFLanguageServerExtension.getBuildPath();
            System.out.println("lfpath is " + lfPath);
            Position codePos = findPosition(params.getPosition().getLine(), params.getPosition().getCharacter(), lfPath, result);
            // Position codePos = findPositionInvertedMap(lfPath);
            return clangdHover(codePos);
            // return super.hover(params, cancelIndicator);
        } catch (IndexOutOfBoundsException e) {
            return IHoverService.EMPTY_HOVER;  // Fail silently
        }
    }

    // void getCodeMaps() {
    //     Map<Path, CodeMap> codeMaps =  LFLanguageServerExtension.getGeneratorResult().getCodeMaps();
    //     for (Map.Entry<Path, CodeMap> entry : codeMaps.entrySet()) {
    //         Path generatedPath = entry.getKey();
    //         Map<Path, NavigableMap<Range, Range>> map = entry.getValue().getCodeMap();
    //         // Map<Path, NavigableMap<Range, Range>> constructedPathMap;
    //         for (Map.Entry<Path, NavigableMap<Range, Range>> pathEntry : map.entrySet()) {
    //             NavigableMap<Range, Range> pathMap = pathEntry.getValue();
    //             Path lfPath = pathEntry.getKey();
    //             // NavigableMap<Range, Range> constructedRangeMap;
    //             for (Map.Entry<Range, Range> rangeEntry : pathMap.entrySet()) {
    //                 // constructedRangeMap.put(rangeEntry.getValue(), rangeEntry.getKey());
    //                 Range generatedRange = rangeEntry.getKey();
    //                 Range lfRange = rangeEntry.getValue();
    //                 insertToRangeMap(lfPath, generatedPath, lfRange, generatedRange);
    //             }
    //         }             
    //     }
    // }

    // void insertToRangeMap(Path lfPath, Path generatedPath, Range lfRange, Range generatedRange) {
    //     if (codeMapping == null) codeMapping = new HashMap<Path, HashMap<Path, HashMap<Range, Range>>>();
    //     HashMap<Path, HashMap<Range, Range>> lfMap = codeMapping.get(lfPath);
    //     if (lfMap == null) {
    //         lfMap = new HashMap<Path, HashMap<Range, Range>>();
    //         codeMapping.put(lfPath, lfMap);
    //     }
    //     HashMap<Range, Range> pathMap = lfMap.get(generatedPath);
    //     if (pathMap == null) {
    //         pathMap = new HashMap<Range, Range>();
    //         lfMap.put(generatedPath, pathMap);
    //     }
    //     pathMap.put(lfRange, generatedRange);
    //     System.out.println("putting " + lfPath + " " + generatedPath + " " + lfRange + " " + generatedRange);
    // }

    // void insertToRangeMap(Path lfPath, Path generatedPath, Range lfRange, Range generatedRange) {
    //     if (codeMapping == null) codeMapping = new HashMap<Pair<Path, Range>, Pair<Path, Range>>();
    //     codeMapping.put(new Pair(lfPath, lfRange), new Pair(generatedPath, generatedRange));
    //     System.out.println("putting " + lfPath + " " + generatedPath + " " + lfRange + " " + generatedRange);
    // }

    OutputStream stdin = null;
    static Process pr;
    InputStream stdout = null;
    BufferedReader reader = null;
    BufferedWriter writer = null;
    BufferedReader errorReader = null;
    boolean initialized = false;
    int buildToken = -1;
    Path generatedPath;
    String generatedDirectory;
    // HashMap<Path, 
    //     HashMap<Path,
    //         HashMap<Range, Range>>> codeMapping = null;
    // HashMap<Pair<Path, Range>, 
    //     Pair<Path, Range>> codeMapping = null;

    private Position findPosition(int line, int col, Path lfFile, Map<Path, CodeMap> generatedResult) {
        for (Path sourcePath : generatedResult.keySet()) {
            CodeMap codeMap = generatedResult.get(sourcePath); 
            for (int brutLine = 1; brutLine <= 500; brutLine++) {
                for (int brutChar = 1; brutChar <= 100; brutChar++) {
                    Position generatedPos = Position.fromOneBased(brutLine, brutChar);
                    Position p = codeMap.adjusted(lfFile, generatedPos);
                    if (p.getOneBasedLine() == line && p.getOneBasedColumn() == col) {
                        generatedPath = sourcePath;
                        return generatedPos;
                    }
                }
            }
        }

        return Position.fromOneBased(1, 1);
    }

    private Hover clangdHover(Position params) {
        if (!initialized){
            process_init();
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
            jsonResponse = jsonResponse.substring(jsonResponse.indexOf(System.getProperty("line.separator"))+1);// remove first line since contains the header 
            jsonResponse = jsonResponse.substring(jsonResponse.indexOf(System.getProperty("line.separator"))+1);
            ObjectMapper objectMapper = new ObjectMapper();
            JsonNode jsonNode = objectMapper.readTree(jsonResponse);
            String jsonValue = jsonNode.get("result").get("contents").get("value").textValue();
            List<Either<String, MarkedString>> content = Collections.singletonList(Either.forLeft(jsonValue));
            resultHover.setContents(content);
            int lvalue_c = jsonNode.get("result").get("range").get("start").get("character").intValue();
            int lvalue_l = jsonNode.get("result").get("range").get("start").get("line").intValue();
            int rvalue_c = jsonNode.get("result").get("range").get("end").get("character").intValue();
            int rvalue_l = jsonNode.get("result").get("range").get("end").get("line").intValue();
            return resultHover;
        } catch (Exception e) {
            System.out.println("main failed");
        }
        return null;
    }

    private String readOutput() {
        char[] chars = new char[8192];
        try {
        for(int len; (len = reader.read(chars)) > 0 && reader.ready();) {
            
        }
        } catch (Exception e) {
            System.out.println("failed reading output");
        }
        return String.valueOf(chars);
    }

    private void process_init() {
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
        try {
        String body = "{\"method\": \"textDocument/hover\",\"jsonrpc\": \"2.0\",\"id\": 1,\"params\": {\"textDocument\": {\"uri\": \"file://" + generatedPath.toString() + "\"} ,\"position\": {\"line\": "+line+",\"character\": "+character+"}}}\r\n";
        String header = "Content-Length: " + body.length() +"\r\n\r\n";
        String cmd = header + body;
        writer.write(cmd);
        writer.flush();
        } catch (Exception e) {
            System.out.println("failed hover request");
        } 

    }

    private void clangd_init() {
        try {
            String body = "{\"jsonrpc\": \"2.0\", \"method\": \"initialize\", \"id\": 1, \"params\": {\"rootUri\": \"file://" + generatedDirectory + "\", \"capabilities\": {\"hover\": {\"dynamicRegistration\": false, \"contentFormat\": []}}}}\r\n";
            String header = "Content-Length: " + body.length() +"\r\n\r\n";
            String cmd = header + body;
            
            writer.write(cmd);
            writer.flush();


        } catch (Exception e) {
            System.out.println("failed clangd init");
        } 
    }

    private void did_open_clangd() {

         try {
            String fileContent = Files.readString(generatedPath, StandardCharsets.UTF_8);
            fileContent = fileContent.replaceAll("\"", "\\\\\"");
            fileContent = fileContent.replaceAll("\n", "\\\\n");
            String body = "{\"method\": \"textDocument/didOpen\", \"jsonrpc\": \"2.0\", \"params\": {\"textDocument\": {\"uri\": \"file://" + generatedPath + "\", \"languageId\": \"cpp\", \"version\": 1, \"text\": \""+fileContent+"\"}}}\r\n";
            String header = "Content-Length: " + (body.length()) +"\r\n\r\n";
            String cmd = header + body;           
            writer.write(cmd);
            writer.flush();
        } catch (Exception e) {
            System.out.println("failed clangd did open");
        } 

    }
}
