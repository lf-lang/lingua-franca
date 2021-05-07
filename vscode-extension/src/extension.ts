'use strict';

import * as path from 'path';
import * as os from 'os';

import { Trace } from 'vscode-jsonrpc';
import { commands, window, workspace, ExtensionContext, Uri } from 'vscode';
import { LanguageClient, LanguageClientOptions, ServerOptions } from 'vscode-languageclient';

let client: LanguageClient;

export function activate(context: ExtensionContext) {
    let jar = context.asAbsolutePath(path.join('ls', 'lflang.jar'));
    // TODO check if correct java is available
    let serverOptions: ServerOptions = {
        run : { command: 'java', args: ['-jar', jar] },
        debug: { command: 'java', args: ['-jar', jar], options: { env: createDebugEnv() } }
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
    client.start();
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