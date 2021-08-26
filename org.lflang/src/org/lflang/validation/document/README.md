# Documentation: org.lflang.validation.document

## Overview

Here is one possible way for an interaction with the user to be handled by the software in this package:
1. The user opens an LF document and triggers a request for validation messages. This request is sent to the Language and Diagram Server.
1. A model of the document is instantiated and saved in the `DocumentRegistry` singleton. The document is compiled by the appropriate LF code generator, and a linter is run on the document. The results of the analysis are sent to the language client.
1. The user edits the document and (somehow) triggers a more lightweight request for validation.
1. This does not trigger a full compilation, but it does trigger an update of the generated code. The linter is run again, and the results are again sent to the language client.

This all depends on programmatic access to various command-line tools that resolve names, check for problems, perform style checks, automate formatting, etc.

It is worth noting that this plan is just a sketch in the sense that it may not be how our final product actually works. Software that is published as part of a language extension cannot make an unreasonable use of computing resources without being explicitly requested to do so by the user.

However, this plan is not just a sketch in the sense that a working prototype already exists. It is still possible that at least some aspects of this package will survive into the language extension -- possibly presented as a computationally intensive option that users can opt into once they know the consequences.

The remainder of this document describes the design of the prototype.

## Classes

### `DocumentRegistry`
This singleton stores all documents in a map. Keys correspond to (URI, target language) pairs. That means that if the user edits the target language declaration at the top of an LF file, then the document will be treated as an entirely new LF file.

#### Fields
* `Map<LFDocument.ID, LFDocument> registry`: The Lingua Franca documents tracked in the current session
* `File workingDirectory`: The location where any hidden files created by `LFDocument`s should be placed

#### Requirements/Assumptions
* The user cannot open too many Lingua Franca documents in one session. If they do, then the documents (and all associated metadata, diagnostics, mappings to `lfc` output, etc.) will accumulate in memory.

### `LFDocument`
Represents a Lingua Franca document.

#### Fields
* `private EObject parseRoot`
* `private List<String> lines`
* `private final Map<File, GeneratedDocument> generatedDocuments`
* `private final ID id`

### `LFDocument.ID`
Represents a unique identifier for an LFDocument according to its absolute file system location and target language.

#### Fields
* `final File file`
* `final Target target`

### `Position`
Represents a position in a document, including line and column.

#### Fields
* `private final int line`
* `private final int column`

### `DiagnosticAcceptor`
Accepts diagnostics (also called validation messages) and relays them on to the IDE. Essentially a wrapper for `ValidationMessageAcceptor`.

#### Fields
* `private final LFDocument document`
* `private final ValidationMessageAcceptor acceptor`

### `DocumentEdit`
Encapsulates logic related to finding document edits at the level of whole lines.

#### Fields
All members are static. This class is not meant to be instantiated, and it does not preserve state. It is used for namespacing purposes, to supply an implementation of the minimal edit distance algorithm along with the `Edit` class and its subclasses.

### `generated.GeneratedDocument`
Represents a document generated from a Lingua Franca file. Encapsulates any logic associated with the syntax of the document or the extraction of source mappings from it.

#### Fields
* `private final File directory`
* `private final List<String> lines`
* `private final NavigableMap<Position, Position> sourceMap`

### `generated.GeneratedDocumentFactory`
Produces the appropriate `GeneratedDocument` for a given generated file.

#### Fields
All members are static. This class is not meant to be instantiated, and it does not preserve state.

### `generated.CCppDocument extends GeneratedDocument`
Represents a generated C or C++ document.

#### Fields
* `private final String extension` (the file extension)

### `generated.TypeScriptDocument extends GeneratedDocument`
Represents a generated TypeScript document.

#### Fields
None.
