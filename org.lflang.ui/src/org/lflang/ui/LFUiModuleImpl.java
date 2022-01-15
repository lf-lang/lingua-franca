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
import org.eclipse.xtend.lib.annotations.FinalFieldsConstructor;
import org.eclipse.xtext.resource.containers.IAllContainersState;
import org.eclipse.xtext.ui.editor.autoedit.AbstractEditStrategyProvider;
import org.eclipse.xtext.ui.editor.autoedit.AbstractTerminalsEditStrategy;
import org.eclipse.xtext.ui.editor.autoedit.CommandInfo;
import org.eclipse.xtext.ui.editor.autoedit.DefaultAutoEditStrategyProvider;
import org.eclipse.xtext.ui.editor.autoedit.MultiLineTerminalsEditStrategy;
import org.eclipse.xtext.ui.editor.autoedit.SingleLineTerminalsStrategy;
import org.eclipse.xtext.ui.shared.Access;
import org.eclipse.xtext.xbase.lib.CollectionLiterals;
import org.eclipse.xtext.xbase.lib.Conversions;
import org.eclipse.xtext.xbase.lib.Exceptions;

/**
 * Use this class to register components to be used within the Eclipse IDE.
 *
 * This subclass provides an astonishingly opaque and complex override of
 * the default editor behavior to properly handle the code body delimiters
 * {= ... =} of Lingua Franca.  It is disheartening how difficult this was
 * to accomplish.  The design of xtext does not seem to lend itself to
 * subclassing, and the code has no comments in it at all.
 */
@FinalFieldsConstructor
@SuppressWarnings("all")
public class LFUiModuleImpl extends AbstractLFUiModule {
    public static class LinguaFrancaAutoEdit extends DefaultAutoEditStrategyProvider {
        public static class LFMultiLineTerminalsEditStrategy extends MultiLineTerminalsEditStrategy {
            public LFMultiLineTerminalsEditStrategy(final String leftTerminal, final String rightTerminal, final boolean nested) {
                super(leftTerminal, "", rightTerminal, nested);
            }

            @Override
            public CommandInfo handleCursorInFirstLine(final IDocument document, final DocumentCommand command, final IRegion startTerminal, final IRegion stopTerminal) throws BadLocationException {
                final CommandInfo newC = new CommandInfo();
                String _leftTerminal = this.getLeftTerminal();
                boolean _equals = Objects.equal(_leftTerminal, "{");
                if (_equals) {
                    final String start = document.get(startTerminal.getOffset(), 2);
                    boolean _equals_1 = Objects.equal(start, "{=");
                    if (_equals_1) {
                        return newC;
                    }
                }
                newC.isChange = true;
                newC.offset = command.offset;
                String _text = newC.text;
                newC.text = (_text + "\n");
                String _text_1 = newC.text;
                String _trim = command.text.trim();
                newC.text = (_text_1 + _trim);
                int _length = newC.text.length();
                int _plus = (command.offset + _length);
                newC.cursorOffset = _plus;
                if (((stopTerminal == null) && this.atEndOfLineInput(document, command.offset))) {
                    String _text_2 = newC.text;
                    String _rightTerminal = this.getRightTerminal();
                    String _plus_1 = (command.text + _rightTerminal);
                    newC.text = (_text_2 + _plus_1);
                }
                if (((stopTerminal != null) && (stopTerminal.getOffset() >= command.offset))) {
                    boolean _isSameLine = this.util.isSameLine(document, stopTerminal.getOffset(), command.offset);
                    if (_isSameLine) {
                        int _offset = stopTerminal.getOffset();
                        int _minus = (_offset - command.offset);
                        final String string = document.get(
                            command.offset, _minus).trim();
                        final int indentation = LFUiModuleImpl.LinguaFrancaAutoEdit.indentationAt(document, command.offset);
                        for (int i = 0; (i < ((indentation / 4) + 1)); i++) {
                            {
                                String _text_3 = newC.text;
                                newC.text = (_text_3 + "    ");
                                int _cursorOffset = newC.cursorOffset;
                                newC.cursorOffset = (_cursorOffset + 4);
                            }
                        }
                        String _text_3 = newC.text;
                        newC.text = (_text_3 + string);
                        String _text_4 = newC.text;
                        String _trim_1 = command.text.trim();
                        newC.text = (_text_4 + _trim_1);
                        String _text_5 = newC.text;
                        newC.text = (_text_5 + "\n");
                        for (int i = 0; (i < (indentation / 4)); i++) {
                            String _text_6 = newC.text;
                            newC.text = (_text_6 + "    ");
                        }
                        int _length_1 = newC.length;
                        int _length_2 = string.length();
                        newC.length = (_length_1 + _length_2);
                    } else {
                        final int indentation_1 = LFUiModuleImpl.LinguaFrancaAutoEdit.indentationAt(document, command.offset);
                        int length = 0;
                        for (int i = 0; (i < ((indentation_1 / 4) + 1)); i++) {
                            {
                                String _text_6 = newC.text;
                                newC.text = (_text_6 + "    ");
                                int _cursorOffset = newC.cursorOffset;
                                newC.cursorOffset = (_cursorOffset + 4);
                                int _length_3 = length;
                                length = (_length_3 + 4);
                            }
                        }
                        newC.length = 0;
                    }
                }
                return newC;
            }

            @Override
            public IRegion findStopTerminal(final IDocument document, final int offset) throws BadLocationException {
                return super.findStopTerminal(document, offset);
            }

            @Override
            public IRegion findStartTerminal(final IDocument document, final int offset) throws BadLocationException {
                return super.findStartTerminal(document, offset);
            }

            @Override
            public void internalCustomizeDocumentCommand(final IDocument document, final DocumentCommand command) {
                try {
                    super.internalCustomizeDocumentCommand(document, command);
                } catch (Throwable _e) {
                    throw Exceptions.sneakyThrow(_e);
                }
            }
        }

        /**
         * Strategy for handling combinations of nested braces of different types.
         * This is based on CompoundMultiLineTerminalsEditStrategy by Sebastian Zarnekow,
         * but unfortunately, that class is not written in a way that can be overridden,
         * so this is a reimplementation.
         */
        public static class CompoundLFMultiLineTerminalsEditStrategy extends AbstractTerminalsEditStrategy {
            /**
             * Strategies used to handle combinations of nested braces.
             */
            private List<LFUiModuleImpl.LinguaFrancaAutoEdit.LFMultiLineTerminalsEditStrategy> strategies = new LinkedList<LFUiModuleImpl.LinguaFrancaAutoEdit.LFMultiLineTerminalsEditStrategy>();

            public CompoundLFMultiLineTerminalsEditStrategy(final String leftTerminal, final String rightTerminal) {
                super(leftTerminal, rightTerminal);
                LFUiModuleImpl.LinguaFrancaAutoEdit.LFMultiLineTerminalsEditStrategy _lFMultiLineTerminalsEditStrategy = new LFUiModuleImpl.LinguaFrancaAutoEdit.LFMultiLineTerminalsEditStrategy("(", ")", true);
                this.strategies.add(_lFMultiLineTerminalsEditStrategy);
                LFUiModuleImpl.LinguaFrancaAutoEdit.LFMultiLineTerminalsEditStrategy _lFMultiLineTerminalsEditStrategy_1 = new LFUiModuleImpl.LinguaFrancaAutoEdit.LFMultiLineTerminalsEditStrategy("{", "}", true);
                this.strategies.add(_lFMultiLineTerminalsEditStrategy_1);
                LFUiModuleImpl.LinguaFrancaAutoEdit.LFMultiLineTerminalsEditStrategy _lFMultiLineTerminalsEditStrategy_2 = new LFUiModuleImpl.LinguaFrancaAutoEdit.LFMultiLineTerminalsEditStrategy("[", "]", true);
                this.strategies.add(_lFMultiLineTerminalsEditStrategy_2);
                LFUiModuleImpl.LinguaFrancaAutoEdit.LFMultiLineTerminalsEditStrategy _lFMultiLineTerminalsEditStrategy_3 = new LFUiModuleImpl.LinguaFrancaAutoEdit.LFMultiLineTerminalsEditStrategy("{=", "=}", true);
                this.strategies.add(_lFMultiLineTerminalsEditStrategy_3);
            }

            @Override
            public void internalCustomizeDocumentCommand(final IDocument document, final DocumentCommand command) throws BadLocationException {
                if ((command.length != 0)) {
                    return;
                }
                final String[] lineDelimiters = document.getLegalLineDelimiters();
                final int delimiterIndex = TextUtilities.startsWith(lineDelimiters, command.text);
                if ((delimiterIndex != (-1))) {
                    LFUiModuleImpl.LinguaFrancaAutoEdit.LFMultiLineTerminalsEditStrategy bestStrategy = ((LFUiModuleImpl.LinguaFrancaAutoEdit.LFMultiLineTerminalsEditStrategy) null);
                    IRegion bestStart = ((IRegion) null);
                    for (final LFUiModuleImpl.LinguaFrancaAutoEdit.LFMultiLineTerminalsEditStrategy strategy : this.strategies) {
                        {
                            IRegion candidate = strategy.findStartTerminal(document, command.offset);
                            if ((candidate != null)) {
                                if (((bestStart == null) || (bestStart.getOffset() < candidate.getOffset()))) {
                                    bestStrategy = strategy;
                                    bestStart = candidate;
                                }
                            }
                        }
                    }
                    if ((bestStrategy != null)) {
                        bestStrategy.internalCustomizeDocumentCommand(document, command);
                    }
                }
            }
        }

        /**
         * Handle combinations of nested braces.
         * The following from the base class completely messes up with codeblocks.
         * So we replace it below. Unfortunately, the base class CompoundMultiLineTerminalsEditStrategy
         * is not written in a way that can be overridden, so we have to completely
         * reimplement it.
         */
        @Override
        public void configureCompoundBracesBlocks(final AbstractEditStrategyProvider.IEditStrategyAcceptor acceptor) {
            LFUiModuleImpl.LinguaFrancaAutoEdit.CompoundLFMultiLineTerminalsEditStrategy _compoundLFMultiLineTerminalsEditStrategy = new LFUiModuleImpl.LinguaFrancaAutoEdit.CompoundLFMultiLineTerminalsEditStrategy("{=", "=}");
            acceptor.accept(_compoundLFMultiLineTerminalsEditStrategy, IDocument.DEFAULT_CONTENT_TYPE);
        }

        /**
         * For the given document, return the indentation of the line at
         * the specified offset. If the indentation is accomplished with
         * tabs, count each tab as four spaces.
         * @param document The document.
         * @param offset The offset.
         */
        public static int indentationAt(final IDocument document, final int offset) {
            try {
                final int lineNumber = document.getLineOfOffset(offset);
                final int lineStart = document.getLineOffset(lineNumber);
                final int lineLength = document.getLineLength(lineNumber);
                String line = document.get(lineStart, lineLength);
                line = line.replaceAll("\t", "    ");
                return line.indexOf(line.trim());
            } catch (Throwable _e) {
                throw Exceptions.sneakyThrow(_e);
            }
        }

        /**
         * When encountering {= append =}.
         */
        protected void configureCodeBlock(final AbstractEditStrategyProvider.IEditStrategyAcceptor acceptor) {
            acceptor.accept(new SingleLineTerminalsStrategy("{=", "=}", SingleLineTerminalsStrategy.DEFAULT) {
                                @Override
                                public void handleInsertLeftTerminal(final IDocument document, final DocumentCommand command) throws BadLocationException {
                                    if ((((command.text.length() > 0) && this.appliedText(document, command).endsWith(this.getLeftTerminal())) && this.isInsertClosingTerminal(document, (command.offset + command.length)))) {
                                        final String documentContent = this.getDocumentContent(document, command);
                                        final int opening = this.count(this.getLeftTerminal(), documentContent);
                                        final int closing = this.count(this.getRightTerminal(), documentContent);
                                        final int occurences = (opening + closing);
                                        if (((occurences % 2) == 0)) {
                                            int _length = command.text.length();
                                            int _plus = (command.offset + _length);
                                            command.caretOffset = _plus;
                                            command.text = (command.text + "=");
                                            command.shiftsCaret = false;
                                        }
                                    }
                                }

                                @Override
                                public boolean isInsertClosingTerminal(final IDocument doc, final int offset) {
                                    try {
                                        int _length = doc.getLength();
                                        boolean _lessEqualsThan = (_length <= offset);
                                        if (_lessEqualsThan) {
                                            return true;
                                        }
                                        if ((offset == 0)) {
                                            return false;
                                        }
                                        final String charAtOffset = Character.toString(doc.getChar(offset));
                                        final String charBeforeOffset = Character.toString(doc.getChar((offset - 1)));
                                        final boolean result = (Objects.equal(charAtOffset, "}") && Objects.equal(charBeforeOffset, "{"));
                                        return result;
                                    } catch (Throwable _e) {
                                        throw Exceptions.sneakyThrow(_e);
                                    }
                                }
                            },
                            IDocument.DEFAULT_CONTENT_TYPE);
        }

        /**
         * When hitting Return with a code block, move the =} to a newline properly indented.
         */
        protected void configureMultilineCodeBlock(final AbstractEditStrategyProvider.IEditStrategyAcceptor acceptor) {
            LFUiModuleImpl.LinguaFrancaAutoEdit.LFMultiLineTerminalsEditStrategy _lFMultiLineTerminalsEditStrategy = new LFUiModuleImpl.LinguaFrancaAutoEdit.LFMultiLineTerminalsEditStrategy("(", ")", true);
            acceptor.accept(_lFMultiLineTerminalsEditStrategy, IDocument.DEFAULT_CONTENT_TYPE);
            LFUiModuleImpl.LinguaFrancaAutoEdit.LFMultiLineTerminalsEditStrategy _lFMultiLineTerminalsEditStrategy_1 = new LFUiModuleImpl.LinguaFrancaAutoEdit.LFMultiLineTerminalsEditStrategy("(", ")", true);
            acceptor.accept(_lFMultiLineTerminalsEditStrategy_1, IDocument.DEFAULT_CONTENT_TYPE);
            LFUiModuleImpl.LinguaFrancaAutoEdit.LFMultiLineTerminalsEditStrategy _lFMultiLineTerminalsEditStrategy_2 = new LFUiModuleImpl.LinguaFrancaAutoEdit.LFMultiLineTerminalsEditStrategy("[", "]", true);
            acceptor.accept(_lFMultiLineTerminalsEditStrategy_2, IDocument.DEFAULT_CONTENT_TYPE);
            LFUiModuleImpl.LinguaFrancaAutoEdit.LFMultiLineTerminalsEditStrategy _lFMultiLineTerminalsEditStrategy_3 = new LFUiModuleImpl.LinguaFrancaAutoEdit.LFMultiLineTerminalsEditStrategy("{=", "=}", true);
            acceptor.accept(_lFMultiLineTerminalsEditStrategy_3, IDocument.DEFAULT_CONTENT_TYPE);
        }

        /**
         * Ensure that all text printed via println() is shown in the Console of the LF IDE.
         */
        public boolean configureConsole() {
            boolean _xifexpression = false;
            if ((!LFUiModuleImpl.consoleInitialized)) {
                boolean _xblockexpression = false;
                {
                    final MessageConsole console = new MessageConsole("LF Output", null);
                    ConsolePlugin.getDefault().getConsoleManager().addConsoles(((IConsole[])Conversions.unwrapArray(CollectionLiterals.<IConsole>newArrayList(console), IConsole.class)));
                    ConsolePlugin.getDefault().getConsoleManager().showConsoleView(console);
                    final MessageConsoleStream stream = console.newMessageStream();
                    PrintStream _printStream = new PrintStream(stream);
                    System.setOut(_printStream);
                    PrintStream _printStream_1 = new PrintStream(stream);
                    System.setErr(_printStream_1);
                    _xblockexpression = LFUiModuleImpl.consoleInitialized = true;
                }
                _xifexpression = _xblockexpression;
            }
            return _xifexpression;
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

    @Override
    public Provider<IAllContainersState> provideIAllContainersState() {
        return Access.getJavaProjectsState();
    }

    public Class<? extends DefaultAutoEditStrategyProvider> bindAutoEditStrategy() {
        return LFUiModuleImpl.LinguaFrancaAutoEdit.class;
    }

    public LFUiModuleImpl(final AbstractUIPlugin arg0) {
        super(arg0);
    }
}
