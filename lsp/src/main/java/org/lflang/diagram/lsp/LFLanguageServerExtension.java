package org.lflang.diagram.lsp;

import java.util.ArrayList;
import java.util.concurrent.CompletableFuture;
import org.eclipse.emf.common.util.URI;
import org.eclipse.lsp4j.jsonrpc.services.JsonNotification;
import org.eclipse.lsp4j.jsonrpc.services.JsonRequest;
import org.eclipse.lsp4j.services.LanguageClient;
import org.eclipse.xtext.ide.server.ILanguageServerAccess;
import org.eclipse.xtext.ide.server.ILanguageServerExtension;
import org.lflang.LFRuntimeModule;
import org.lflang.LFStandaloneSetup;
import org.lflang.ast.ToSExpr;
import org.lflang.generator.GeneratorResult;
import org.lflang.generator.GeneratorResult.Status;
import org.lflang.generator.IntegratedBuilder;
import org.lflang.util.LFCommand;

/**
 * Provide Lingua-Franca-specific extensions to the language server's behavior.
 *
 * @author Peter Donovan
 */
class LFLanguageServerExtension implements ILanguageServerExtension {

  /** The IntegratedBuilder instance that handles all build requests for the current session. */
  private static final IntegratedBuilder builder =
      new LFStandaloneSetup(new LFRuntimeModule())
          .createInjectorAndDoEMFRegistration()
          .getInstance(IntegratedBuilder.class);

  /** The access point for reading documents, communicating with the language client, etc. */
  private LanguageClient client;

  @Override
  public void initialize(ILanguageServerAccess access) {
    // This method is never invoked.
  }

  public void setClient(LanguageClient client) {
    this.client = client;
  }

  @JsonRequest("parser/ast")
  public CompletableFuture<String> getAst(String uri) {
    return CompletableFuture.supplyAsync(
        () -> {
          URI parsedUri;
          try {
            parsedUri = URI.createFileURI(new java.net.URI(uri).getPath());
          } catch (java.net.URISyntaxException e) {
            System.err.println(e);
            return "LF language server failed to get AST because the URI was invalid";
          }
          var model =
              builder
                  .getResource(parsedUri)
                  .getContents()
                  .get(0); // FIXME: if the resource has syntax errors this should fail
          var toSExpr = new ToSExpr();
          var sExpr = toSExpr.doSwitch(model);
          return sExpr.toString();
        });
  }

  /**
   * Handle a request for a complete build of the Lingua Franca file specified by {@code uri}.
   *
   * @param uriAndJson the URI of the LF file of interest
   * @return A message describing the outcome of the build process.
   */
  @JsonRequest("generator/build")
  public CompletableFuture<String> build(String[] uriAndJson) {
    if (client == null)
      return CompletableFuture.completedFuture(
          "Please wait for the Lingua Franca language server to be fully initialized.");
    return CompletableFuture.supplyAsync(
        () -> {
          try {
            return buildWithProgress(client, uriAndJson, true).getUserMessage();
          } catch (Exception e) {
            return "An internal error occurred:\n" + e;
          }
        });
  }

  /**
   * Handles a request for the most complete build of the specified Lingua Franca file that can be
   * done in a limited amount of time.
   *
   * @param uriAndJson the URI of the LF file of interest
   */
  @JsonNotification("generator/partialBuild")
  public void partialBuild(String[] uriAndJson) {
    if (client == null) return;
    buildWithProgress(client, uriAndJson, false);
  }

  /**
   * Completely build the specified LF program and provide information that is sufficient to run it.
   *
   * @param uriAndJson The URI of the LF program to be built.
   * @return An array consisting of the directory in which the execute command should be executed,
   *     the program of the execute command, and the arguments of the execute command.
   */
  @JsonNotification("generator/buildAndRun")
  public CompletableFuture<String[]> buildAndRun(BuildArgs uriAndJson) {
    return new CompletableFuture<String[]>()
        .completeAsync(
            () -> {
              var result = buildWithProgress(client, new String[]{uriAndJson.getUri(), uriAndJson.getJson()}, true);
              if (!result.getStatus().equals(Status.COMPILED)) return null;
              LFCommand cmd = result.getContext().getFileConfig().getCommand();
              ArrayList<String> ret = new ArrayList<>();
              ret.add(cmd.directory().toString());
              ret.addAll(cmd.command());
              return ret.toArray(new String[0]);
            });
  }

  /** Describes a build process that has a progress. */
  private GeneratorResult buildWithProgress(
      LanguageClient client, String[] uriAndJson, boolean mustComplete) {
    URI parsedUri;
    try {
      parsedUri = URI.createFileURI(new java.net.URI(uriAndJson[0]).getPath());
    } catch (java.net.URISyntaxException e) {
      // This error will appear as a silent failure to most users, but that is acceptable because
      // this error
      // should be impossible. The URI is not the result of user input -- the language client
      // provides it --
      // so it should be valid.
      System.err.println(e);
      return GeneratorResult.NOTHING;
    }
    Progress progress =
        new Progress(client, "Build \"" + parsedUri.lastSegment() + "\"", mustComplete);
    progress.begin();
    GeneratorResult result = null;
    try {
      result =
          builder.run(parsedUri, mustComplete, progress::report, progress.getCancelIndicator());
    } finally {
      progress.end(result == null ? "An internal error occurred." : result.getUserMessage());
    }
    return result;
  }

  private static class BuildArgs {

      private String uri;

      private String json;

      public String getUri() {
          return uri;
      }

      public void setUri(String uri) {
          this.uri = uri;
      }

      public String getJson() {
          return json;
      }

      public void setJson(String json) {
          this.json = json;
      }
  }
}
