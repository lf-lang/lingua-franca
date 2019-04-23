# LSP Example for Lingua Franca

This is based on https://code.visualstudio.com/api/language-extensions/language-server-extension-guide

## Installation

For running this you need `nodejs` and `npm` installed, get the latest version from: https://nodejs.org/en/download/

In the root of this subproject (`lsp` folder) run:
```
npm install
npm run compile
```

You might also need to globally install typescript, `npm install -g typescript`.
Depending on your installation you might need to run this as root: `sudo npm install -g typescript`

## Generating the Parser

To generate the parser you need pegjs
```
npm install pegjs
pegjs lf.pegjs
```

## Running the example

Start vscode (from the `lsp` folder). If you don't have vscode installed you can get it from: https://code.visualstudio.com/
If you're on a Mac, follow the instructions [here](https://code.visualstudio.com/docs/setup/mac) to add the `code` command to your `$PATH`.

```
code .
```

This should open vscode on the sample. Navigate to the lsp server file (`server/src/server.ts`) and open it in vscode and go to the debug view (i.e. with Ctrl/Cmd + D).

To run the server press `F5`. This should spawn a second instance of vscode running the LSP server behind it. Editing a file with the `lf` extension will automatically activate the language server and provide IDE support for syntax erros.


## Structure

```
.
├── client // Language Client
│   ├── src
│   │   ├── test // End to End tests for Language Client / Server
│   │   └── extension.ts // Language Client entry point
├── package.json // The extension manifest.
└── server // Language Server
    └── src
        └── server.ts // Language Server entry point
```
