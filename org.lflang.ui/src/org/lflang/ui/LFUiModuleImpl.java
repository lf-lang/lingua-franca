package org.lflang.ui;

import com.google.common.base.Objects;
import com.google.inject.Provider;
import java.io.PrintStream;
import java.util.LinkedList;
import java.util.List;
import org.eclipse.jface.text.BadLocationException;
import org.eclipse.jface.text.DocumentCommand;
import org.eclipse.jface.text.IDocument;
import org.eclipse.jface.text.IRegion;
import org.eclipse.jface.text.TextUtilities;
import org.eclipse.ui.console.ConsolePlugin;
import org.eclipse.ui.console.IConsole;
import org.eclipse.ui.console.MessageConsole;
import org.eclipse.ui.console.MessageConsoleStream;
import org.eclipse.ui.plugin.AbstractUIPlugin;
import org.eclipse.xtext.resource.containers.IAllContainersState;
import org.eclipse.xtext.ui.editor.autoedit.AbstractEditStrategyProvider;
import org.eclipse.xtext.ui.editor.autoedit.AbstractTerminalsEditStrategy;
import org.eclipse.xtext.ui.editor.autoedit.CommandInfo;
import org.eclipse.xtext.ui.editor.autoedit.DefaultAutoEditStrategyProvider;
import org.eclipse.xtext.ui.editor.autoedit.MultiLineTerminalsEditStrategy;
import org.eclipse.xtext.ui.editor.autoedit.SingleLineTerminalsStrategy;
import org.eclipse.xtext.ui.shared.Access;

/**
 * Use this class to register components to be used within the Eclipse IDE.
 *
 * This subclass provides an astonishingly opaque and complex override of
 * the default editor behavior to properly handle the code body delimiters
 * {= ... =} of Lingua Franca.  It is disheartening how difficult this was
 * to accomplish.  The design of xtext does not seem to lend itself to
 * subclassing, and the code has no comments in it at all.
 */
public class LFUiModuleImpl extends AbstractLFUiModule {
    public static class LinguaFrancaAutoEdit extends DefaultAutoEditStrategyProvider {
        public static class LFMultiLineTerminalsEditStrategy extends MultiLineTerminalsEditStrategy {
            public LFMultiLineTerminalsEditStrategy(final String leftTerminal,
                                                    final String rightTerminal,
                                                    final boolean nested) {
                super(leftTerminal, "", rightTerminal, nested);
            }

            @Override
            public CommandInfo handleCursorInFirstLine(final IDocument document,
                                                       final DocumentCommand command,
                                                       final IRegion startTerminal,
                                                       final IRegion stopTerminal) throws BadLocationException {
                // Create a modified command.
                final CommandInfo newC = new CommandInfo();
                // If this is handling delimiters { }, but the actual delimiters are {= =},
                // then do nothing.
                if (getLeftTerminal().equals("{")) {
                    final String start = document.get(startTerminal.getOffset(), 2);
                    if (start.equals("{=")) {
                        return newC;
                    }
                }
                newC.isChange = true;
                newC.offset = command.offset;
                // Insert the Return character into the new command.
                newC.text += "\n";
                newC.text += command.text.trim();
                newC.cursorOffset = command.offset + newC.text.length();
                if (stopTerminal == null && atEndOfLineInput(document, command.offset)) {
                    newC.text += command.text + getRightTerminal();
                }
                if (stopTerminal != null && stopTerminal.getOffset() >= command.offset) {
                    // If the right delimiter is on the same line as the left,
                    // collect the text between them and indent to the right place.
                    if (util.isSameLine(document, stopTerminal.getOffset(), command.offset)) {
                        // Get the code block between the delimiters, trimed of whitespace.
                        final String codeBlock = document.get(command.offset,
                                                           stopTerminal.getOffset() - command.offset).trim();
                        final int indentation = indentationAt(document, command.offset);

                        // Indent by at least 4 spaces.
                        int numSpaces = 4 * ((indentation / 4) + 1);
                        newC.text += " ".repeat(numSpaces);
                        newC.cursorOffset += numSpaces;

                        newC.text += codeBlock;
                        newC.text += command.text.trim();
                        newC.text += "\n";

                        int numNextLineSpaces = 4 * (indentation / 4);
                        newC.text += " ".repeat(numNextLineSpaces);
                        newC.length += codeBlock.length();
                    } else {
                        // Creating a new first line within a pre-existing block.
                        final int indentation = indentationAt(document, command.offset);
                        // Indent by at least 4 spaces.
                        int numSpaces = 4 * ((indentation / 4) + 1);
                        newC.text += " ".repeat(numSpaces);
                        newC.cursorOffset += numSpaces;
                        // The length field is, as usual for xtext, undocumented.
                        // It is not the length of the new command, but seems to be
                        // the number of characters of the original document that are
                        // to be replaced.
                        newC.length = 0;
                    }
                }
                return newC;
            }

            // Expose base class protected methods within this package.
            @Override
            public IRegion findStopTerminal(final IDocument document, final int offset) throws BadLocationException {
                return super.findStopTerminal(document, offset);
            }

            @Override
            public IRegion findStartTerminal(final IDocument document, final int offset) throws BadLocationException {
                return super.findStartTerminal(document, offset);
            }

            @Override
            public void internalCustomizeDocumentCommand(final IDocument document, final DocumentCommand command) throws BadLocationException {
                super.internalCustomizeDocumentCommand(document, command);
            }
        }

        /**
         * Strategy for handling combinations of nested braces of different types.
         * This is based on CompoundMultiLineTerminalsEditStrategy by Sebastian Zarnekow,
         * but unfortunately, that class is not written in a way that can be overridden,
         * so this is a reimplementation.
         */
        public static class CompoundLFMultiLineTerminalsEditStrategy extends AbstractTerminalsEditStrategy {
            public CompoundLFMultiLineTerminalsEditStrategy(final String leftTerminal, final String rightTerminal) {
                super(leftTerminal, rightTerminal);
            }

            @Override
            public void internalCustomizeDocumentCommand(final IDocument document, final DocumentCommand command) throws BadLocationException {
                if (command.length != 0) {
                    return;
                }
                final String[] lineDelimiters = document.getLegalLineDelimiters();
                final int delimiterIndex = TextUtilities.startsWith(lineDelimiters, command.text);
                if (delimiterIndex != -1) {
                    LFMultiLineTerminalsEditStrategy bestStrategy = null;
                    IRegion bestStart = null;
                    for (final LFMultiLineTerminalsEditStrategy strategy : strategies) {
                        IRegion candidate = strategy.findStartTerminal(document, command.offset);
                        if (candidate != null) {
                            if (bestStart == null || bestStart.getOffset() < candidate.getOffset()) {
                                bestStrategy = strategy;
                                bestStart = candidate;
                            }
                        }
                    }
                    if (bestStrategy != null) {
                        bestStrategy.internalCustomizeDocumentCommand(document, command);
                    }
                }
            }
        }

        /**
         * Strategies used to handle combinations of nested braces.
         */
        private final static List<LFMultiLineTerminalsEditStrategy> strategies =
            List.of(new LFMultiLineTerminalsEditStrategy("(", ")", true),
                    new LFMultiLineTerminalsEditStrategy("{", "}", true),
                    new LFMultiLineTerminalsEditStrategy("[", "]", true),
                    new LFMultiLineTerminalsEditStrategy("{=", "=}", true));

        /**
         * Handle combinations of nested braces.
         * The following from the base class completely messes up with codeblocks.
         * So we replace it below. Unfortunately, the base class CompoundMultiLineTerminalsEditStrategy
         * is not written in a way that can be overridden, so we have to completely
         * reimplement it.
         */
        @Override
        public void configureCompoundBracesBlocks(final AbstractEditStrategyProvider.IEditStrategyAcceptor acceptor) {
            acceptor.accept(new CompoundLFMultiLineTerminalsEditStrategy("{=", "=}"), IDocument.DEFAULT_CONTENT_TYPE);
        }

        /**
         * For the given document, return the indentation of the line at
         * the specified offset. If the indentation is accomplished with
         * tabs, count each tab as four spaces.
         * @param document The document.
         * @param offset The offset.
         */
        public static int indentationAt(final IDocument document, final int offset) throws BadLocationException {
            final int lineNumber = document.getLineOfOffset(offset); // Line number.
            final int lineStart = document.getLineOffset(lineNumber); // Offset of start of line.
            final int lineLength = document.getLineLength(lineNumber); // Length of the line.
            String line = document.get(lineStart, lineLength);
            // Replace all tabs with four spaces.
            line = line.replaceAll("\t", "    ");
            return line.indexOf(line.trim());
        }

        /**
         * When encountering {= append =}.
         */
        protected void configureCodeBlock(final AbstractEditStrategyProvider.IEditStrategyAcceptor acceptor) {
            acceptor.accept(
                new SingleLineTerminalsStrategy("{=", "=}", SingleLineTerminalsStrategy.DEFAULT) {
                    @Override
                    public void handleInsertLeftTerminal(final IDocument document,
                                                         final DocumentCommand command) throws BadLocationException {
                        if (command.text.length() > 0
                            && this.appliedText(document, command).endsWith(this.getLeftTerminal())
                            && this.isInsertClosingTerminal(document, (command.offset + command.length))) {
                            final String documentContent = this.getDocumentContent(document, command);
                            final int opening = this.count(this.getLeftTerminal(), documentContent);
                            final int closing = this.count(this.getRightTerminal(), documentContent);
                            final int occurences = (opening + closing);
                            if (occurences % 2 == 0) {
                                command.caretOffset = command.offset + command.text.length();
                                // Do not insert the right delimiter '=}' because there is already
                                // a '}' from the previous auto complete when the '{' was typed.
                                command.text = (command.text + "=");
                                command.shiftsCaret = false;
                            }
                        }
                    }

                    @Override
                    public boolean isInsertClosingTerminal(final IDocument doc, final int offset) {
                        try {
                            if (doc.getLength() <= offset) {
                                return true;
                            }
                            if (offset == 0) {
                                return false;
                            }
                            // xtend fails horribly with char literals, so we have to
                            // convert this to a string.
                            final String charAtOffset = Character.toString(doc.getChar(offset));
                            final String charBeforeOffset = Character.toString(doc.getChar(offset - 1));
                            final boolean result = (Objects.equal(charAtOffset, "}") && Objects.equal(charBeforeOffset, "{"));
                            return result;
                        } catch (BadLocationException e) {
                            throw new RuntimeException(e);
                        }
                    }
                },
                IDocument.DEFAULT_CONTENT_TYPE);
        }

        /**
         * When hitting Return with a code block, move the =} to a newline properly indented.
         */
        protected void configureMultilineCodeBlock(final AbstractEditStrategyProvider.IEditStrategyAcceptor acceptor) {
            for (LFMultiLineTerminalsEditStrategy strategy : strategies) {
                acceptor.accept(strategy, IDocument.DEFAULT_CONTENT_TYPE);
            }
        }

        /**
         * Ensure that all text printed via println() is shown in the Console of the LF IDE.
         */
        public boolean configureConsole() {
            if (!consoleInitialized) {
                final MessageConsole console = new MessageConsole("LF Output", null);
                ConsolePlugin.getDefault().getConsoleManager().addConsoles(new IConsole[]{console});
                ConsolePlugin.getDefault().getConsoleManager().showConsoleView(console);
                final MessageConsoleStream stream = console.newMessageStream();
                System.setOut(new PrintStream(stream));
                System.setErr(new PrintStream(stream));
                consoleInitialized = true;
            }
            return consoleInitialized;
        }

        /**
         * Specify these new acceptors.
         */
        @Override
        public void configure(final AbstractEditStrategyProvider.IEditStrategyAcceptor acceptor) {
            this.configureConsole();
            this.configureMultilineCodeBlock(acceptor);
            super.configure(acceptor);
            this.configureCodeBlock(acceptor);
        }
    }

    private static boolean consoleInitialized = false;

    /**
     *  Instead of classpath, use Properties -> Project Reference.
     */
    @Override
    public Provider<IAllContainersState> provideIAllContainersState() {
        return Access.getJavaProjectsState();
    }

    public Class<? extends DefaultAutoEditStrategyProvider> bindAutoEditStrategy() {
        return LinguaFrancaAutoEdit.class;
    }

    public LFUiModuleImpl(final AbstractUIPlugin arg0) {
        super(arg0);
    }
}
