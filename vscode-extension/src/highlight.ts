'use strict';

import { posix } from 'path';
import { nextTick } from 'process';
import {
    TextDocument,
    DocumentSemanticTokensProvider,
    ProviderResult,
    SemanticTokens,
    SemanticTokensBuilder,
    SemanticTokensLegend,
    Range,
    Position,
    CancellationToken
} from 'vscode';
import { workerData } from 'worker_threads';

const tokenTypes = [
    'typeParameter', 'parameter', 'type', 'variable', 'property', 'macro'
];
const tokenModifiers = [
    'declaration', 'readonly'
];

export const legend = new SemanticTokensLegend(tokenTypes, tokenModifiers);

/**
 * Returns a function that maps a position in a document to the position
 * that immediately follows it.
 * @param document - The document to which positions will correspond
 * @returns - A function that maps a position in a document to the position
 *     that immediately follows it
 */
function getNext(document: TextDocument): (pos: Position) => Position {
    return pos => {
        let lineLength = document.lineAt(pos.line).text.length;
        if (pos.character + 1 >= lineLength) {
            return new Position(pos.line + 1, 0);
        }
        return pos.translate(0, 1);
    };
}

/**
 * Returns a list of all ranges contained by `range` that correspond to
 * words.
 * @param document - The document from which to extract tokens
 * @param range - The range of the document in which to search
 */
function getWords(document: TextDocument, range?: Range): Range[] {
    let ret: Range[] = [];
    let next = getNext(document);
    for (let pos = range.start; range.contains(pos); ) {
        let word = document.getWordRangeAtPosition(pos);
        if (word) {
            ret.push(word);
            pos = word.end;
        }
        pos = next(pos);
    }
    return ret;
}

/**
 * Returns a Range containing the entirety of a given TextDocument.
 * @param document - A TextDocument
 * @returns a Range containing the entirety of document
 */
function wholeDocument(document: TextDocument): Range {
    return new Range(
        0, 0,
        document.lineCount - 1,
        document.lineAt(document.lineCount - 1).text.length - 1
    );
}

/**
 * Returns the ranges that appear strictly later than a match to begin
 * but strictly before a corresponding match to end. For example, if
 * begin is '(' and end is ')', then this function would return a range
 * including the entirety of '(a(b(c)d)e)'. The nesting depth of a token
 * is never negative.
 * 
 * @param document - The document to be examined for contained ranges
 * @param range - The range within the document in which to search
 * @param begin - The token that causes an increment of nesting depth
 * @param end - The token that causes a decrement of nesting depth. Must
 *     not be equal to begin.
 * @param minDepth - The desired minimum nesting depth of all returned
 *     ranges, relative to the start of the range in which to search
 * @param maxDepth - The desired maximum nesting depth of all returned
 *     ranges, relarive to the start of the range in which to search
 */
function getContainedRanges(
    document: TextDocument,
    range: Range,
    begin: string,
    end: string,
    minDepth: number = 1,
    maxDepth: number = Infinity
): Range[] {
    const ret = [];
    let depth = 0;
    let rangeStart: Position = minDepth <= 0 ? range.start : undefined;
    let next = getNext(document);
    for (let pos = range.start; range.contains(pos); pos = next(pos)) {
        let line = document.lineAt(pos.line).text;
        if (line.slice(pos.character - begin.length + 1, pos.character + 1)
        == begin) {
            if (depth == maxDepth && rangeStart) {
                ret.push(new Range(rangeStart, pos));
                rangeStart = undefined;
            }
            depth++;
            if (depth == minDepth) {
                rangeStart = next(pos);
            }
        } else if (
            line.slice(pos.character, pos.character + end.length)
        === end) {
            if (depth > 0) {
                if (depth == minDepth && rangeStart) {
                    ret.push(new Range(rangeStart, pos));
                    rangeStart = undefined;
                }
                depth--;
                if (depth == maxDepth) {
                    rangeStart = next(pos);
                }
            }
        }
    }
    if (rangeStart) {
        ret.push(new Range(rangeStart, range.end));
    }
    return ret;
}

/**
 * Side effect: Pushes semantic labels associated with type parameters
 * to tokensBuilder.
 * @param document - The document to be analyzed for type parameters
 * @param tokensBuilder  - The object that accumulates semantic labels
 */
 function provideTypeParameters(
    document: TextDocument,
    tokensBuilder: SemanticTokensBuilder
) {
    for (const reactorDeclaration of getContainedRanges(
        document, wholeDocument(document), 'reactor', '{'
    )) {
        // TODO: Highlight the type parameter throughout
        for (const typeParameterGroup of getContainedRanges(
            document, reactorDeclaration, '<', '>'
        )) {
            // Code blocks ({= =}) are permitted in type parameter groups,
            // and I wish to exclude them.
            const nonCodeBlocks = getContainedRanges(
                document, typeParameterGroup, '{=', '=}', 0, 0);
            for (const nonCodeBlock of nonCodeBlocks) {
                for (const word of getWords(document, nonCodeBlock)) {
                    tokensBuilder.push(word, 'typeParameter');
                }
            }
        }
    }
}

/**
 * Side effect: Pushes semantic labels associated with parameters
 * and their associated types and default values to `tokensBuilder`.
 * @param document - The document to be analyzed for parameters
 * @param tokensBuilder  - The object that accumulates semantic labels
 */
 function provideParameters(
    document: TextDocument,
    tokensBuilder: SemanticTokensBuilder
): void {
    for (const reactorDeclaration of getContainedRanges(
        document, wholeDocument(document), 'reactor', '{')
    ) {
        // TODO: Highlight the parameter throughout
        let parameters: string[] = [];
        for (const parameterList of getContainedRanges(
            document, reactorDeclaration, '(', ')'
        )) {
            for (const nonValue of getContainedRanges(
                document, parameterList, '(', ')', 0, 0
            )) {
                for (const nonTypeValuePair of getContainedRanges(
                    document, nonValue, ':', ',', 0, 0
                )) {
                    for (const word of getWords(document, nonTypeValuePair)) {
                        parameters.push(document.getText(word));
                        tokensBuilder.push(word, 'parameter', ['readonly']);
                    }
                }
            }
            for (const typeValuePair of getContainedRanges(
                document, parameterList, ':', ','
            )) {
                for (const nonValue of getContainedRanges(
                    document, typeValuePair, '(', ')', 0, 0
                )) {
                    for (const word of getWords(document, nonValue)) {
                        tokensBuilder.push(word, 'type');
                    } // TODO: make this storage.type.lflang, not entity.type
                }
            }
        }
    }
}

/**
 * Side effect: Pushes semantic labels associated with variables
 * to tokensBuilder.
 * @param document - The document to be analyzed for variables
 * @param tokensBuilder  - The object that accumulates semantic labels
 */
 function provideVariables(
    document: TextDocument,
    tokensBuilder: SemanticTokensBuilder
): void {

}

/**
 * Side effect: Pushes semantic labels associated with properties
 * to tokensBuilder.
 * @param document - The document to be analyzed for properties
 * @param tokensBuilder  - The object that accumulates semantic labels
 */
 function provideProperties(
    document: TextDocument,
    tokensBuilder: SemanticTokensBuilder
): void {

}

/**
 * Side effect: Pushes semantic labels associated with type parameters
 * to tokensBuilder.
 * @param document - The document to be analyzed for type parameters
 * @param tokensBuilder  - The object that accumulates semantic labels
 */
 function provideMacros(
    document: TextDocument,
    tokensBuilder: SemanticTokensBuilder
): void {

}

/**
 * Returns semantic labels associated with a subset of the tokens in
 * a document.
 * 
 * @param document - The document to be semantically analyzed
 * @returns - Semantic labels associated with some subset of the
 * document's tokens
 */
export const semanticTokensProvider: DocumentSemanticTokensProvider = {
    provideDocumentSemanticTokens(
        document: TextDocument,
        token: CancellationToken
    ): ProviderResult<SemanticTokens> {
        const tokensBuilder = new SemanticTokensBuilder(legend);
        provideTypeParameters(document, tokensBuilder);
        provideParameters(document, tokensBuilder);
        provideVariables(document, tokensBuilder);
        provideProperties(document, tokensBuilder);
        provideMacros(document, tokensBuilder);
        return tokensBuilder.build();
    }
}