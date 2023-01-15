package org.lflang.diagram.lsp;

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
import java.util.HashSet;
import java.io.File;

import org.eclipse.lsp4j.Hover;
import org.eclipse.lsp4j.HoverParams;
import org.eclipse.xtext.ide.server.hover.IHoverService;
import org.eclipse.xtext.util.CancelIndicator;

import org.lflang.generator.GeneratorResult;

public class CppLanguageServer{
    static OutputStream stdin = null;
    static Process pr;
    static InputStream stdout = null;
    static BufferedReader reader = null;
    static BufferedWriter writer = null;
    static BufferedReader errorReader = null;
    static Path generatedPath;
    static String generatedDirectory;
    static Map<Path, GeneratorResult> storedGeneratedResults;
    static Map<Path, ProjectBuildResult> storedBuildResults;
    

    public static void init() {
        process_init();
        clangd_init();
        readOutput();
        storedBuildResults = new HashMap();
        System.out.println("Initialized language server");
    }

    public static Hover hoverRequest(HoverParams params) {
        Path lfPath = Paths.get(params.getTextDocument().getUri().substring(5));
        if (!storedBuildResults.containsKey(lfPath)) {
            Hover preHover = new Hover();
            List<Either<String, MarkedString>> content = Collections.singletonList(Either.forLeft("You first need to build a project"));
            preHover.setContents(content);
            return preHover;
        }
        try {
            Map<Path, CodeMap> result;
            GeneratorResult genResult;
            ProjectBuildResult buildResult = storedBuildResults.get(lfPath);
            result =  buildResult.getGeneratorResult().getCodeMaps();
            generatedDirectory = buildResult.getGeneratorResult().getCommand().directory().toString() + "/src-gen";
            check_and_add_cmake_file();
            Position codePos = findPosition(params.getPosition().getLine(), params.getPosition().getCharacter(), lfPath, result);
            if (codePos.getOneBasedLine() == 1 && codePos.getOneBasedColumn() == 1) {
                return IHoverService.EMPTY_HOVER; // hover to LF code
            }
            if (!buildResult.isOpen(generatedPath)) {
                buildResult.open(generatedPath);
            }
            return clangdHover(codePos);
        } catch (Exception e) {
            return IHoverService.EMPTY_HOVER;  // Fail silently
        }
    }

    private static Position findPosition(int line, int col, Path lfFile, Map<Path, CodeMap> generatedResult) {
        for (Path sourcePath : generatedResult.keySet()) {
            CodeMap codeMap = generatedResult.get(sourcePath); 
            for (int brutLine = 1; brutLine <= 500; brutLine++) { // TODO change brute force positions
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

    private static Hover clangdHover(Position params) {
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
            // int lvalue_c = jsonNode.get("result").get("range").get("start").get("character").intValue();
            // int lvalue_l = jsonNode.get("result").get("range").get("start").get("line").intValue();
            // int rvalue_c = jsonNode.get("result").get("range").get("end").get("character").intValue();
            // int rvalue_l = jsonNode.get("result").get("range").get("end").get("line").intValue();
            return resultHover;
        } catch (Exception e) {
            // most often gets here because there is no fields that jsonNode.get() is looking for (i.e. result is null)
            return IHoverService.EMPTY_HOVER;
        }
    }

    private static void check_and_add_cmake_file() {
        try {
        File cmake_file = new File(generatedDirectory + "compile_commands.json");
            if (!cmake_file.exists()) {
                ProcessBuilder cmake_command = new ProcessBuilder("cmake",  "-DCMAKE_EXPORT_COMPILE_COMMANDS=1");
                cmake_command.directory(new File(generatedDirectory));
                cmake_command.start();
            }
        } catch (Exception e) {
            System.out.println("failed creating cmake file");
        }
    }

    private static String readOutput() {
        char[] chars = new char[8192];
        try {
        for(int len; (len = reader.read(chars)) > 0 && reader.ready();) {
            
        }
        } catch (Exception e) {
            System.out.println("failed reading output");
        }
        return String.valueOf(chars);
    }

    private static void process_init() {
        try {
        ProcessBuilder clangd = new ProcessBuilder("clangd");
        pr = clangd.start();  
        
        stdin = pr.getOutputStream();
        stdout = pr.getInputStream();
        reader = new BufferedReader (new InputStreamReader(stdout));
        writer = new BufferedWriter(new OutputStreamWriter(stdin));

        errorReader = new BufferedReader(new InputStreamReader(pr.getErrorStream()));
        } catch (Exception e) {
            System.out.println("failed process init");
        } 
    }
    
    private static void readErrorOutput() {
        try {
        String str = null;
        while ((str = errorReader.readLine()) != null && errorReader.ready()) {
            System.out.println(str);
        }
        } catch (Exception e) {
            System.out.println("failed reading error output");
        } 
    }

    private static void send_hover_request(int line, int character) {
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

    private static void clangd_init() {
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

    private static void did_open_clangd(Path path) {

         try {
            String fileContent = Files.readString(path, StandardCharsets.UTF_8);
            fileContent = fileContent.replaceAll("\"", "\\\\\"");
            fileContent = fileContent.replaceAll("\n", "\\\\n");
            String body = "{\"method\": \"textDocument/didOpen\", \"jsonrpc\": \"2.0\", \"params\": {\"textDocument\": {\"uri\": \"file://" + path + "\", \"languageId\": \"cpp\", \"version\": 1, \"text\": \""+fileContent+"\"}}}\r\n";
            String header = "Content-Length: " + (body.length()) +"\r\n\r\n";
            String cmd = header + body;           
            writer.write(cmd);
            writer.flush();
        } catch (Exception e) {
            System.out.println("failed clangd did open");
        } 

    }

    private static void did_close_clangd(String filePath) {
        try {
            String body = "{\"method\": \"textDocument/didClose\", \"jsonrpc\": \"2.0\", \"params\": {\"textDocument\": {\"uri\": \"file://" + filePath + "\"}}}\r\n";
            String header = "Content-Length: " + (body.length()) +"\r\n\r\n";
            String cmd = header + body;         
            writer.write(cmd);
            writer.flush();
        } catch (Exception e) {
        System.out.println("failed clangd did close");
        } 
    }

    static class ProjectBuildResult {
        Path uri;
        GeneratorResult generatorResult;
        HashSet<Path> clangOpenFiles;

        public ProjectBuildResult(Path u, GeneratorResult gr) {
            uri = u;
            generatorResult = gr;
            clangOpenFiles = new HashSet();
        }
        public GeneratorResult getGeneratorResult() {
            return generatorResult;
        }
        public String path() {
            return uri.toString();
        }
        
        public boolean isOpen(Path p) {
            return clangOpenFiles.contains(p);
        }
        public void open(Path p) {
            did_open_clangd(p);
            readOutput();
            clangOpenFiles.add(p);
        }
        public void closeOpenFiles() {
            for (Path gen_path : clangOpenFiles) {
                did_close_clangd(gen_path.toString());
                readOutput();
            }
        }

    }

    public static void addBuild(Path uri, GeneratorResult generatorResult) {
        ProjectBuildResult buildResult = new ProjectBuildResult(uri, generatorResult);
        if (storedBuildResults.containsKey(uri)) {
            // update build if was re-built
            ProjectBuildResult result = storedBuildResults.get(uri);
            result.closeOpenFiles();
            storedBuildResults.remove(uri);
        }
        storedBuildResults.put(uri, buildResult);
    }
}