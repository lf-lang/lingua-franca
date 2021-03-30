package org.icyphy;

import static java.nio.file.FileVisitResult.CONTINUE;

import java.io.File;
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
import org.icyphy.generator.Main;
import org.icyphy.linguaFranca.Reactor;

import com.google.common.collect.Iterables;

public class MainConflictChecker {

    public final List<String> conflicts = new LinkedList<String>();
    
    protected String name;
    
    protected FileConfig fileConfig;
    
    protected ResourceSet rs = new LinguaFrancaStandaloneSetup()
            .createInjectorAndDoEMFRegistration()
            .<Main>getInstance(Main.class).getResourceSet();
    
    public MainConflictChecker(String name, FileConfig fileConfig) {
        this.name = name;
        this.fileConfig = fileConfig;
        // FIXME: Report the conflicting differently depending on whether we are in the IDE or on the CLI.
        // We are now just reporting the file as it exists in the file system.
        try {
            Files.walkFileTree(fileConfig.srcPkgPath, new PackageVisitor());
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
    
    public class PackageVisitor extends SimpleFileVisitor<Path> {

        boolean inTestDir = false;
        
        /**
         * Add test files to the registry if they end with ".lf", but only if they have a main reactor.
         */
        @Override
        public FileVisitResult visitFile(Path path, BasicFileAttributes attr) {
            if (attr.isRegularFile() && path.toString().endsWith(".lf")) {
                // Parse the file.
                Resource r = rs.getResource(
                        URI.createFileURI(path.toFile().getAbsolutePath()),
                        true);
                if (r.getErrors().isEmpty()) {
                    // No errors. Find the main reactor.
                    Iterator<Reactor> reactors = Iterables
                            .<Reactor>filter(IteratorExtensions
                                    .<EObject>toIterable(r.getAllContents()),
                                    Reactor.class)
                            .iterator();
                    File file = path.toFile();
                    if (!fileConfig.srcFile.equals(file)
                            && IteratorExtensions.exists(reactors,
                                    it -> it.isMain() || it.isFederated())
                            && name.equals(
                                    FileConfig.nameWithoutExtension(file))) {
                        conflicts.add(path.toString());
                    }
                }
            }
            return CONTINUE;
        }
    }
}
