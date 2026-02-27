package org.lflang;

import static java.nio.file.FileVisitResult.CONTINUE;

import com.google.common.collect.Iterables;
import java.io.IOException;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;
import org.lflang.lf.Reactor;
import org.lflang.util.FileUtil;

/**
 * Class that (upon instantiation) determines whether there are any conflicting main reactors in the
 * current package. Conflicts are reported in the instance's conflicts list.
 *
 * @author Marten Lohstroh
 */
public class MainConflictChecker {

  /** List of conflict encountered while traversing the package. */
  public final List<String> conflicts = new LinkedList<String>();

  /** The current file configuration. */
  protected FileConfig fileConfig;

  /** Resource set used to obtain resources from. */
  protected static final ResourceSet rs =
      new LFStandaloneSetup()
          .createInjectorAndDoEMFRegistration()
          .<LFResourceProvider>getInstance(LFResourceProvider.class)
          .getResourceSet();

  /**
   * Create a new instance that walks the file tree of the package to find conflicts.
   *
   * @param fileConfig The current file configuration.
   */
  public MainConflictChecker(FileConfig fileConfig) {
    // FIXME: See if there is a faster way to do this check that does not involve parsing all
    //  files in the current package (which happens when we get the resources)
    this.fileConfig = fileConfig;
    try {
      Files.walkFileTree(fileConfig.srcPath, new PackageVisitor());
    } catch (IOException e) {
      System.err.println("Error while checking for name conflicts in package.");
      e.printStackTrace();
    }
  }

  /**
   * Extension of a SimpleFileVisitor that adds entries to the conflicts list in the outer class.
   *
   * <p>Specifically, each visited file is checked against the name present in the current file
   * configuration. If the name matches the current file (but is not the file itself), then parse
   * that file and determine whether there is a main reactor declared. If there is one, report a
   * conflict.
   *
   * @author Marten Lohstroh
   */
  public class PackageVisitor extends SimpleFileVisitor<Path> {

    /** Report a conflicting main reactors in visited files. */
    @Override
    public FileVisitResult visitFile(Path path, BasicFileAttributes attr) {
      path = path.normalize();
      if (attr.isRegularFile() && path.toString().endsWith(".lf")) {
        // Parse the file.
        Resource r = rs.getResource(URI.createFileURI(path.toFile().getAbsolutePath()), true);
        if (r.getErrors().isEmpty()) {
          // No errors. Find the main reactor.
          Iterator<Reactor> reactors =
              Iterables.<Reactor>filter(
                      IteratorExtensions.<EObject>toIterable(r.getAllContents()), Reactor.class)
                  .iterator();
          // If this is not the same file, but it has a main reactor
          // and the name matches, then report the conflict.
          if (!fileConfig.srcFile.equals(path)
              && IteratorExtensions.exists(reactors, it -> it.isMain() || it.isFederated())
              && fileConfig.name.equals(FileUtil.nameWithoutExtension(path))) {
            conflicts.add(fileConfig.srcPath.relativize(path).toString());
          }
        }
      }
      return CONTINUE;
    }
  }
}
