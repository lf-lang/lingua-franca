package org.lflang.diagram.lsp;

import java.util.ArrayList;
import java.util.concurrent.CompletableFuture;
import java.util.stream.Stream;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
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
import org.lflang.lf.Model;
import org.lflang.lf.Reactor;
import org.lflang.lf.TargetDecl;
import org.lflang.util.LFCommand;

import org.eclipse.xtext.nodemodel.INode;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
import org.eclipse.xtext.resource.XtextResourceSet;

import com.google.inject.Inject;
import com.google.inject.Injector;

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
  private LFLanguageClient client;

  @Inject Injector injector;

  @Override
  public void initialize(ILanguageServerAccess access) {
    // This method is never invoked.
  }

  public void setClient(LFLanguageClient client) {
    this.client = client;
  }


  public XtextResourceSet getXtextResourceSet( final URI uri) {
      return injector.getInstance(XtextResourceSet.class);
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
   * @param args the URI of the LF file of interest
   * @return A message describing the outcome of the build process.
   */
  @JsonRequest("generator/build")
  public CompletableFuture<String> build(BuildArgs args) {
    if (client == null)
      return CompletableFuture.completedFuture(
          "Please wait for the Lingua Franca language server to be fully initialized.");
    return CompletableFuture.supplyAsync(
        () -> {
          try {
            return buildWithProgress(client, args, true).getUserMessage();
          } catch (Exception e) {
            return "An internal error occurred:\n" + e;
          }
        });
  }


  /**
   * Manage requests to retrieve a hierarchical structure of reactor libraries based on the provided {@code filePath}.
   *
   * @param filePath the URI of the LF file of interest
   * @return A message describing the outcome of the build process.
   */
  @JsonRequest("generator/getLibraryReactors")
  public CompletableFuture<Tree> getLibraryReactors(String filePath) {
    return CompletableFuture.supplyAsync(
        () -> {
          try {
              // LF program file parsing
              URI uri = URI.createURI(filePath);
              // Return a list of reactors within the file at the specific uri
              return parseLibraryReactors(uri);
          } catch (Exception e) {
            return null;
          }
        });
  }

    /**
     * Retrieves the target position specified in the LF program file at the given path.
     *
     * @param path The path to the LF program file.
     * @return A CompletableFuture containing the NodePosition object representing the position of the target,
     *         or null if an error occurs during parsing or if the target position is not found.
     */
    @JsonRequest("generator/getTargetPosition")
    public CompletableFuture<NodePosition> getTargetPosition(String path) {
        return CompletableFuture.supplyAsync(
            () -> {
                NodePosition targetPosition = null;
                try {
                    URI uri = URI.createURI(path);
                    // LF program file parsing
                    Resource resource = getXtextResourceSet(uri).getResource(uri, true);
                    Model m = (Model) resource.getContents().get(0);
                    TargetDecl target = m.getTarget();
                    INode node = NodeModelUtils.getNode(target);
                    targetPosition = new NodePosition(node.getStartLine(), node.getEndLine());
                    return targetPosition;
                } catch (Exception e) {
                    return null;
                }
            });
    }

  /**
   * Parses a library of reactors specified by the provided URI and constructs a hierarchical tree representation.
   *
   * @param uri The URI specifying the location of the library.
   * @return A Tree object representing the hierarchical structure of the reactor library,
   *         or null if an error occurs during parsing.
   */
  public Tree parseLibraryReactors(URI uri){
      Tree res = new Tree(uri.toString());
      try{
          Resource resource = getXtextResourceSet(uri).getResource(uri, true);
          Model m = (Model) resource.getContents().get(0);
          Stream<Reactor> reactors = m.getReactors().stream().filter( r -> r.getName() != null && !r.getName().isEmpty());
          reactors.forEach( r ->{
              INode node = NodeModelUtils.getNode(r);
              NodePosition nodePosition = new NodePosition(node.getStartLine(), node.getEndLine());
              res.addChild(new TreeNode(r.getName(), res.getUri(), nodePosition));
          });
      }catch(Exception e){
          return null;
      }
      return res;
  }

  /**
   * Handles a request for the most complete build of the specified Lingua Franca file that can be
   * done in a limited amount of time.
   *
   * @param args the URI of the LF file of interest
   */
  @JsonNotification("generator/partialBuild")
  public void partialBuild(BuildArgs args) {
    if (client == null) return;
    buildWithProgress(client, args, false);
  }

  /**
   * Completely build the specified LF program and provide information that is sufficient to run it.
   *
   * @param args The URI of the LF program to be built.
   * @return An array consisting of the directory in which the execute command should be executed,
   *     the program of the execute command, and the arguments of the execute command.
   */
  @JsonNotification("generator/buildAndRun")
  public CompletableFuture<String[]> buildAndRun(BuildArgs args) {
    return new CompletableFuture<String[]>()
        .completeAsync(
            () -> {
              var result = buildWithProgress(client, args, true);
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
      LanguageClient client, BuildArgs args, boolean mustComplete) {
    URI parsedUri;
    try {
      parsedUri = URI.createFileURI(new java.net.URI(args.getUri()).getPath());
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
          builder.run(
              parsedUri,
              args.getJson(),
              mustComplete,
              progress::report,
              progress.getCancelIndicator());
    } finally {
      progress.end(result == null ? "An internal error occurred." : result.getUserMessage());
    }
    return result;
  }
}
