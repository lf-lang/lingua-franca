'use strict';

import * as path from 'path';
import * as os from 'os';
import * as fs from 'fs';

import { Trace } from 'vscode-jsonrpc';
import { commands, window, workspace, ExtensionContext, languages, TextEditor, TextDocument } from 'vscode';
import { LanguageClient, LanguageClientOptions, ServerOptions } from 'vscode-languageclient';
import { legend, semanticTokensProvider } from './highlight';


let client: LanguageClient;

export async function activate(context: ExtensionContext) {
    let javaArgs: string[];
    const lsDir = 'ls'
    const ldsJar = context.asAbsolutePath(path.join(lsDir, 'lflang-lds.jar'));
    const hasDiagrams = fs.existsSync(ldsJar);

    if (hasDiagrams) {
        // Get os to select platform dependent SWT library
        let swt: string;
        let cpSep = ':';
        switch(os.platform()) {
            case 'win32': 
                swt = context.asAbsolutePath(path.join(lsDir, 'org.eclipse.swt.win32.win32.jar'));
                cpSep = ';';
                break;
            case 'darwin': 
                swt = context.asAbsolutePath(path.join(lsDir, 'org.eclipse.swt.cocoa.macosx.jar'));
                break;
            default: // maybe a bit too optimistic
                swt = context.asAbsolutePath(path.join(lsDir, 'org.eclipse.swt.gtk.linux.jar'));
                break;
        }
        javaArgs = ['-cp', ldsJar+cpSep+swt, 'org.lflang.diagram.lsp.LanguageDiagramServer'];
    } else {
        // This assumes the extention was build with the standart LS without diagrams only named lflang.jar (requires manual activation in the gradle script)
        javaArgs = ['-jar', context.asAbsolutePath(path.join(lsDir, 'lflang.jar'))];
    }
    
    // TODO check if correct java is available
    let serverOptions: ServerOptions = {
        run : { command: 'java', args: javaArgs },
        debug: { command: 'java', args: javaArgs, options: { env: createDebugEnv() } }
    };
    
    let clientOptions: LanguageClientOptions = {
        documentSelector: ['lflang'],
        synchronize: {
            fileEvents: workspace.createFileSystemWatcher('**/*.*')
        }
    };
    
    client = new LanguageClient('LF Language Server', serverOptions, clientOptions);
    // enable tracing (.Off, .Messages, Verbose)
    client.trace = Trace.Verbose;

    if (hasDiagrams) {
        // Register with Klighd Diagram extension
        const refId = await commands.executeCommand(
            "klighd-vscode.setLanguageClient",
            client,
            ["lf"]
        );
    }

    client.start();

    context.subscriptions.push(commands.registerTextEditorCommand(
        'linguafranca.build',
        (textEditor: TextEditor, _) => {
            const uri = getLfUri(textEditor.document)
            if (!uri) {
                window.showErrorMessage(
                    'The currently active file is not a Lingua Franca source '
                    + 'file.'
                );
                return;
            };
            workspace.saveAll().then(function(successful) {
                if (!successful) return;
                client.sendNotification('generator/build', uri);
            });
        }
    ));
    workspace.onDidSaveTextDocument(function(textDocument: TextDocument) {
        const uri = getLfUri(textDocument);
        if (!uri) return; // This is not an LF document, so do nothing.
        client.sendNotification('generator/partialBuild', uri);
    })

    context.subscriptions.push(languages.registerDocumentSemanticTokensProvider(
        { language: 'lflang', scheme: 'file' },
        semanticTokensProvider,
        legend
    ));
}

function getLfUri(textDocument: TextDocument): string | undefined {
    const uri: string = textDocument.uri.toString();
    if (!uri.endsWith('.lf')) return undefined;
    return uri;
}

function createDebugEnv() {
    return Object.assign({
        JAVA_OPTS:"-Xdebug -Xrunjdwp:server=y,transport=dt_socket,address=8000,suspend=n,quiet=y"
    }, process.env)
}

export function deactivate(): Thenable<void> | undefined {
    if (!client) {
        return undefined;
    }
    return client.stop();
}