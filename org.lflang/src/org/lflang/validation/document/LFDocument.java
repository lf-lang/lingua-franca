package org.lflang.validation.document;

import org.lflang.Target;
import org.lflang.lf.Code;
import org.lflang.lf.TargetDecl;
import org.lflang.validation.DocumentRegistry;
import org.lflang.generator.Main;
import org.lflang.validation.document.generated.GeneratedDocument;
import org.lflang.validation.document.generated.GeneratedDocumentFactory;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.nodemodel.INode;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
import org.eclipse.xtext.validation.ValidationMessageAcceptor;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.List;
import java.nio.file.Files;
import java.io.File;
import java.io.PrintWriter;

import com.google.common.io.MoreFiles;

// FIXME: Substitute RuntimeExceptions out in favor of a more
//  appropriate type of error report.

/**
 * Represents a Lingua Franca document.
 */
public class LFDocument {

    // FIXME: Ensure that this list is exhaustive
    // This is a list rather than a set because it is small
    //  and structurally immutable (and so will stay small),
    //  even though it is used as a set.
    /**
     * Directories in src-gen that should not be searched
     * for generated files that relate directly to LF files.
     */
    private static final List<String> EXCLUDE_DIRECTORIES = List.of("core", "node_modules", "dist", "build");

    /**
     * Represents a unique identifier for an LFDocument
     * according to its absolute file system location and
     * target language.
     */
    public static class ID {

        final File file;
        final Target target;

        /**
         * Creates an ID for the document whose parse tree
         * root node is <code>parseRoot</code>.
         * @param parseRoot the root node of the parse tree
         * of an LF document
         */
        public ID(EObject parseRoot) {
            // FIXME: eResource() may return null. Make sure this never happens.
            this(parseRoot.eResource().getURI(), getTargetLanguage(parseRoot));
        }

        /**
         * Initializes an LFDocument.ID corresponding to the
         * file located at <code>uri</code> with the target
         * language given by <code>target</code>.
         * @param uri the URI of the file containing the LF
         *            code to be modeled
         * @param target the target language of the LF code
         *               to be modeled
         */
        public ID(URI uri, Target target) {
            this(new File(uri.toFileString()), target);
        }

        /**
         * Initializes an LFDocument.ID corresponding to the
         * file located at <code>uri</code> with the target
         * language given by <code>target</code>.
         * @param file the file containing the LF code to be
         *             modeled
         * @param target the target language of the LF code
         *               to be modeled
         */
        public ID(File file, Target target) {
            this.file = file;
            this.target = target;
        }

        /**
         * Returns whether this <code>LFDocument.ID</code> is
         * equal to <code>other</code>. This is true iff
         * <code>other</code> is an <code>LFDocument.ID</code>
         * corresponding to the same file system location
         * and the same target language.
         * @param other the object against which this ID is
         *              to be compared
         * @return whether this ID is equal to
         * <code>other</code>
         */
        @Override
        public boolean equals(Object other) {
            if (!(other instanceof LFDocument.ID)) {
                return false;
            }
            return ((LFDocument.ID) other).target == target
                && ((LFDocument.ID) other).file.equals(file);
        }

        /**
         * Returns the target language of the LF file
         * identified by this ID.
         */
        public Target getTarget() {
            return target;
        }

        @Override
        public String toString() {
            return "<LF File at " + file.toString() + " with target language " + target.toString() + ">";
        }

        /**
         * Gets the target language of the LF document whose
         * parse tree root is <code>parseRoot</code>, or
         * <code>null</code> if the target language is not
         * specified.
         * @param parseRoot the root node of the parse tree
         *                  of an LF document
         * @return the target language of the LF document
         * whose parse tree root is <code>parseRoot</code>
         */
        private static Target getTargetLanguage(EObject parseRoot) {
            Target targetLanguage = null;
            for (EObject level1Node : parseRoot.eContents()) {
                if (level1Node instanceof TargetDecl) {
                    // targetLanguage should be nullable.
                    targetLanguage = Target.match(((TargetDecl) level1Node).getName(), Target.values());
                    break;
                }
            }
            return targetLanguage;
        }
    }

    /** The root node of the parse tree of this LF document */
    private EObject parseRoot;
    /** The content of this document */
    private List<String> lines;
    /** Models of the documents generated by this document */
    private final Map<File, GeneratedDocument> generatedDocuments;
    /** The unique identifier of this document */
    private final ID id;

    /* ------------------------  CONSTRUCTORS  -------------------------- */

    /**
     * Creates a representation of the LF Document whose
     * parse tree root node is <code>parseRoot</code>.
     * @param parseRoot  the root node of the parse tree
     *                   of an LF document
     */
    public LFDocument(EObject parseRoot) {
        this.parseRoot = parseRoot;
        lines = new ArrayList<>();
        // FIXME: getNode may return null
        NodeModelUtils.getNode(parseRoot).getText().lines().forEach(lines::add);
        generatedDocuments = new HashMap<>();
        id = new ID(parseRoot);
    }

    /* -----------------------  PUBLIC METHODS  ------------------------- */

    /**
     * Updates the model of the document represented by this
     * <code>LFDocument</code> and reports diagnostics to
     * <code>acceptor</code> so that they can be sent to the
     * IDE.
     * @param parseRoot the root node of the parse tree
     *                  of this LF document
     * @param acceptor the <code>ValidationMessageAcceptor
     *                 </code> instance that is to relay
     *                 diagnostics to the IDE
     * @param fast whether the update must be performed
     *             quickly
     */
    public void validate(EObject parseRoot, ValidationMessageAcceptor acceptor, boolean fast) {
        this.parseRoot = parseRoot;
        if (fast) quickUpdateModel();
        else updateModel();
        reportDiagnostics(new DiagnosticAcceptor(this, acceptor));
    }

    /**
     * Returns the target language of this LFDocument.
     * @return the target language of this LFDocument
     */
    public Target getTarget() {
        return id.target;
    }

    /**
     * Returns the offset from the beginning of this
     * document to <code>p</code>, including line breaks
     * @param p a position in this document
     * @return the offset from the beginning of this
     * document to <code>p</code>
     */
    public int getOffset(Position p) {
        return lines.stream().limit(p.getZeroBasedLine()).mapToInt(String::length).sum()
            + p.getZeroBasedColumn() + p.getZeroBasedLine(); // Final term accounts for line breaks
    }

    /**
     * Returns the content of the requested line
     * @param index the zero-based index of a line in this
     *              <code>LFDocument</code>
     * @return the content of the requested line
     */
    public String getLine(int index) {
        return index < lines.size() ? lines.get(index) : "";
    }

    /**
     * Returns the root node of the parse tree of this LF
     * document.
     * @return the root node of the parse tree of this LF
     * document
     */
    public EObject getParseRoot() {
        return parseRoot;
    }

    /* -----------------------  PRIVATE METHODS  ------------------------ */

    /**
     * Updates document models to match the LF source code.
     * Returns true iff changes to the document
     * could be resolved without ambiguity. Otherwise, does
     * nothing and returns false.
     */
    private void quickUpdateModel() { // FIXME: This method is pathological: long with deeply nested logic.
        List<String> editSource = getLines(parseRoot);
        // Iterate in decreasing order by line number
        List<INode> codeBlocks = getCodeBlocks();
        for (DocumentEdit.Edit edit : DocumentEdit.minimalEdit(editSource, lines)) {
            boolean inCodeBlock = false;
            for (INode block : codeBlocks) {
                if (block.getStartLine() < edit.getLine() && edit.getLine() < block.getEndLine()) { // FIXME: Off-by-one error
                    inCodeBlock = true;
                    break;
                }
            }
            if (inCodeBlock) {
                if (edit instanceof DocumentEdit.Deletion) {
                    this.lines.remove(edit.getLine());
                    for (GeneratedDocument doc : generatedDocuments.values()) {
                        // FIXME: Must check whether this is the right document to edit
                        doc.deleteLine(edit.getLine());
                    }
                } else if (edit instanceof DocumentEdit.Mutation) {
                    this.lines.set(edit.getLine(), ((DocumentEdit.Mutation) edit).getNewText());
                    for (GeneratedDocument doc : generatedDocuments.values()) {
                        // FIXME: same as above
                        doc.mutateLine(edit.getLine(), ((DocumentEdit.Mutation) edit).getNewText());
                    }
                } else if (edit instanceof DocumentEdit.Insertion) {
                    this.lines.add(edit.getLine(), ((DocumentEdit.Insertion) edit).getNewText());
                    for (GeneratedDocument doc : generatedDocuments.values()) {
                        // FIXME: same as above
                        doc.insertLine(edit.getLine(), ((DocumentEdit.Insertion) edit).getNewText());
                    }
                }
            } else {
                // FIXME: Eliminate the state variable lines?
                // FIXME: a line that is only the empty string "" is assumed to have no effect on the code.
                //  Is this a good assumption?
                if (edit instanceof DocumentEdit.Deletion) {
                    this.lines.set(edit.getLine(), "");
                } else if (edit instanceof DocumentEdit.Mutation) {
                    this.lines.set(edit.getLine(), ((DocumentEdit.Mutation) edit).getNewText());
                } else if (edit instanceof DocumentEdit.Insertion) {
                    this.lines.add(edit.getLine(), ((DocumentEdit.Insertion) edit).getNewText());
                    for (GeneratedDocument doc : generatedDocuments.values()) {
                        // FIXME: same as above
                        doc.insertLine(edit.getLine(), "");
                    }
                }
            }
        }
    }

    /**
     * Returns a list of all parse nodes corresponding to
     * code blocks.
     * @return a list of all parse nodes corresponding to
     * code blocks
     */
    private List<INode> getCodeBlocks() {
        List<INode> ret = new ArrayList<>();
        Iterator<EObject> it = parseRoot.eAllContents();
        while (it.hasNext()) {
            EObject next = it.next();
            if (next instanceof Code) ret.add(NodeModelUtils.getNode(next));
        }
        return ret;
    }

    /**
     * Updates the current model of this LF document,
     * including the content of the document(s) that would
     * be generated from it.
     */
    private void updateModel() {
        lines = getLines(parseRoot);
        try {
            compile();
        } catch (IOException e) {
            throw new RuntimeException("Failed to compile LF document " + this + ".", e);
        }
        findGeneratedDocuments();
    }

    /**
     * Recovers diagnostic messages from the
     * <code>GeneratedDocument</code>s associated with this
     * LF document and passes them to <code>acceptor</code>.
     * @param acceptor the <code>DiagnosticAcceptor</code>
     *                 instance that is to relay diagnostics
     *                 to the IDE
     */
    private void reportDiagnostics(DiagnosticAcceptor acceptor) {
        for (Map.Entry<File, GeneratedDocument> entry : generatedDocuments.entrySet()) {
            try {
                entry.getValue().getDiagnostics(acceptor);
            } catch (IOException e) {
                throw new RuntimeException(
                    "Failed to compute diagnostics for " + this + " while analyzing " + entry.getValue() + ".",
                    e
                );
            }
        }
    }

    /**
     * Returns the Lingua Franca file location used for
     * compilation.
     * @return the Lingua Franca file location used for
     * compilation
     */
    private File getSrcFile() {
        final File f = new File(
            new File(getCompileDir(), "src"), id.file.getName()
        );
        f.getParentFile().mkdirs();
        return f;
    }

    /**
     * Returns the target language file produced by
     * compilation.
     * @return the target language file produced by
     * compilation
     */
    private File getOutDir() {
        return new File(getCompileDir(), "src-gen");
    }

    /**
     * Returns the working directory for any temporary files
     * needed for compilation.
     * @return the working directory for any temporary files
     * needed for compilation
     */
    private File getCompileDir() {
        // FIXME: I am not sure whether this ought to be cached rather than
        //  re-computed each time. Caching is more complex because it creates
        //  more state to keep track of, but it also feels silly to re-compute
        //  this.
        final File compileDir = DocumentRegistry.getInstance().getSaveLocation(
            new File(
                id.file.getParentFile(),
                id.file.getName().replace(".", "_dot_")
            )
        );
        compileDir.mkdirs();
        return compileDir;
    }

    /**
     * Compiles this to its target language and saves the
     * result in <code>lines</code>.
     */
    private void compile() throws IOException {
        // FIXME: It is surprising that it is necessary to delete the directory, but it is:
        //  If it is not deleted, the LF compiler will crash because lib files already exist.
        if (getCompileDir().exists()) {
            MoreFiles.deleteRecursively(getCompileDir().toPath()); // This function is in beta.
        }
        final PrintWriter writer = new PrintWriter(getSrcFile()); // FIXME: Do not create a new file?
        for (String lfLine : lines) {
            writer.println(lfLine);
        }
        writer.println("//DisableTargetLanguageValidation");
        writer.close();
        // FIXME: This is a hack used to work around the fact that AbstractDeclarativeValidator works like a singleton
        Thread t = new Thread(() -> Main.main(new String[] {
            "--no-compile",
            getSrcFile().getAbsolutePath()
        }));
        t.start();
        try {
            t.join(); // FIXME: Determine an acceptable waiting time
        } catch (InterruptedException e) {
            // FIXME: This is the wrong thing to do.
            throw new RuntimeException("Unable to compile.", e);
        }
    }

    /**
     * Returns the file extension that appears at the end of
     * <code>f</code>.
     * @param f a file
     * @return the file extension that appears at the end of
     * <code>f</code>
     */
    private static String getExtension(File f) {
        if (!f.isFile()) return "";
        String name = f.getName();
        int dotIdx = name.lastIndexOf('.');
        return dotIdx == -1 ? "" : name.substring(dotIdx + 1);
    }

    /**
     * Re-discovers and instantiates models of any generated
     * documents present in the file system.
     */
    private void findGeneratedDocuments() {
        generatedDocuments.clear();
        findGeneratedDocumentsRecursive(getOutDir());
    }

    /**
     * Finds any generated files that are not in excluded
     * directories. Intended only to be called by <code>
     * findGeneratedDocuments</code>.
     */
    private void findGeneratedDocumentsRecursive(File f) {
        if (f.isFile()) {
            try { // FIXME: Check if generated document has mappings to this LFDocument
                final GeneratedDocument g = GeneratedDocumentFactory.getGeneratedDocument(
                    Files.readAllLines(f.toPath()), f.getParentFile(), getExtension(f)
                );
                if (g != null) {
                    generatedDocuments.put(f, g);
                    g.refineSourceMap(this);
                }
            } catch (IOException e) {
                throw new RuntimeException("Failed to read " + f + ".", e);
            }
        } else {
            if (EXCLUDE_DIRECTORIES.contains(f.getName())) return;
            File[] children = f.listFiles();
            assert children != null;
            for (File c : children) {
                findGeneratedDocumentsRecursive(c);
            }
        }
    }

    /**
     * Returns a list of all lines of text associated with
     * the given parse node.
     * @param parseNode any EObject
     * @return the lines of code associated with
     * <code>parseNode</code>
     */
    private static List<String> getLines(EObject parseNode) {
        List<String> lines = new ArrayList<>();
        INode rootNode = NodeModelUtils.getNode(parseNode);
        if (rootNode != null) rootNode.getText().lines().forEach(lines::add);
        return lines;
    }

}
