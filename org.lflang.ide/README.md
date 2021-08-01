# org.lflang.ide
This project contains customizations to the LSP infrastructure provided by XText.

## Overview

Here is one example of how an interaction with the user would be handled by the software in this project:
1. The user opens an LF document and triggers a request which is sent to the Language and Diagram Server, which depends on the XText LSP support, which depends on an interface that is implemented in this project.
1. The object in this project that implements that interface receives data about the newly opened document and passes that information to the `DocumentRegistry`.
1. The `DocumentRegistry` instantiates a model of that document. The document is compiled by `lfc`, and a static analyzer is run on the document. The results of the analysis are sent to the language client.
1. In response the the diagnostics, the client requests quick fixes. This request is handled by another object here that implements an XText interface. That object finds the document of interest in the `DocumentRegistry` and requests fixes, which are also provided by the static analyzer.

This all depends on programmatic access to various command-line tools that resolve names, check for problems, perform style checks, automate formatting, etc.

Classes in this project fall into the following categories:
* *Implementations of interfaces provided by XText.* These classes should be mostly concerned with interfacing with the larger framework, and they should not contain any logic that is specific to a target language. For example, the implementation of `org.eclipse.xtext.ide.server.hover.IHoverService` should not contain any logic that is specific to LF files that compile to Python.
* *General tools for modeling LF documents and projects.* This includes the document registry, the connection to the client, and the logic that documents of all target languages have in common.
* *Target language-specific classes.* These should be in separate packages, and they should be concerned with tasks such as running and interpreting the output of static analyzers, since linters and other static analyzers are generally language-specific.
    * Some target languages will be lumped together in one package if they are usually analyzed using the same set of tools. For example, C and C++ are both analyzed by the same linters and static analyzers.

## Classes

### `DocumentRegistry`
This singleton stores all documents in a map. Keys correspond to (URI, target language) pairs. That means that if the user edits the target language declaration at the top of an LF file, then the document will be treated as an entirely new LF file.

#### Fields
* `Map<LFDocument.ID, LFDocument> registry`: The Lingua Franca documents tracked in the current session
* `org.eclipse.lsp4j.services.LanguageClient client`: The LanguageClient instance through which to communicate with the language client (e.g., by sending diagnostics)
* `File workingDirectory`: The location where any hidden files created by `LFDocument`s should be placed

#### Requirements/Assumptions
* The user cannot open too many Lingua Franca documents in one session. If they do, then the documents (and all associated metadata, diagnostics, mappings to `lfc` output, etc.) will accumulate in memory. (FIXME?)

### `LFDocument` (abstract)
This class represents a Lingua Franca document and encapsulates the results of any analyses performed on it.

#### Fields
* `org.eclipse.xtext.resource.XtextResource resource`: Information about the document as it relates to the XText framework, as well as its URI
* `String[] lfLines`: The lines of the Lingua Franca document
* `String[] targetLines`: The lines of the target language document
* `NavigableMap<org.eclipse.lsp4j.Position, org.eclipse.lsp4j.Position> sourceMap`: Mappings from positions in `compiledDocument` to `document`
* `Map<org.eclipse.lsp4j.Diagnostic, org.eclipse.xtext.ide.editor.quickfix.DiagnosticResolution> diagnosticResolutions`: Proposed resolutions to problems found in the document.

### `CCppTargetDocument extends LFDocument`
Represents an `LFDocument` whose target language is C or C++.

#### Fields
* None

#### Requirements/Assumptions
* CppCheck must be installed on the user's system.

### `PythonTargetDocument extends LFDocument`
Represents an `LFDocument` whose target language is Python.

#### Fields
* None

#### Requirements/Assumptions
* ????

### `TSTargetDocument extends LFDocument`
Represents an `LFDocument` whose target language is TypeScript.

#### Fields
* None

#### Requirements/Assumptions
* ESLint must be installed on the user's system (either globally or in the current project)

### `QuickFixProvider`
Relays requests for quick fixes to the appropriate LFDocuments in the `LFDocumentRegistry` singleton.

## Notes
I made some configuration changes that I am not sure are right, so I am
writing down what they were for now and not committing them.
1. It was necessary to add the contents of `org.lflang.lds/target/repository` to the Eclipse target by creating a new target based on the default target and adding the entire directory. Go to Window -> Preferences -> Plug-in Development -> Target Platform -> Add... Current target to create a new target based on the default, and then Edit -> Add -> Directory to add the directory from `org.lflang.lds` that has the JARs.
1. It was necessary to add the following dependencies to the MANIFEST.mf of `org.lflang.diagram` to avoid a sneaky-throw error due to a ClassNotFoundException` from (I believe) `de.cau.cs.kieler.klighd.lsp`. This seems suboptimal since the real source of the dependency is the plugin (which does not have the dependency in its MANIFEST.mf):
  * org.eclipse.sprotty,
  * org.eclipse.sprotty.xtext,
  * org.eclipse.sprotty.layout
