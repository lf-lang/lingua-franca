'use strict';

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

const tokenTypes = [
    'typeParameter', 'parameter', 'type', 'variable', 'property', 'class'
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
        if (pos.character + 1 > lineLength) {
            return new Position(pos.line + 1, 0);
        }
        return pos.translate(0, 1);
    };
}
/**
 * Returns a function that maps a position in a document to the position
 * that immediately precedes it.
 * @param document - The document to which positions will correspond
 * @returns - A function that maps a position in a document to the position
 *     that immediately precedes it
 */
function getPrev(document: TextDocument): (pos: Position) => Position {
    return pos => {
        if (pos.character === 0) {
            return new Position(pos.line - 1,
                document.lineAt(pos.line - 1).text.length);
        }
        return pos.translate(0, -1);
    }
}

/**
 * Returns a list of all ranges contained by `range` that correspond to
 * words.
 * @param document - The document from which to extract tokens
 * @param range - The range of the document in which to search
 */
function getWords(document: TextDocument, range?: Range): Range[] {
    const ret: Range[] = [];
    const next = getNext(document);
    const prev = getPrev(document);
    for (let pos = range.start; range.contains(pos); pos = next(pos)) {
        let word = document.getWordRangeAtPosition(pos);
        // The "prev(word.end)" subexpression that follows is another
        // manifestation of the same weird inconsistency in the API that
        // haunts this entire document.
        if (word && range.contains(new Range(word.start, prev(word.end)))) {
            ret.push(word);
            pos = word.end;
        }
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
        new Position(0, 0),
        document.positionAt(document.getText().length - 1)
    );
}

/**
 * Returns the ranges in an LF document that are not parsed normally,
 * such as comments, strings, and code blocks.
 * @param document - A Lingua Franca source code file
 * @returns the ranges in an LF document that are not parsed normally,
 *     such as comments, strings, and code blocks
 */
function standardShadow(document: TextDocument): Range[] {
    const ret: Range[] = [];
    for (const [_, rangeList] of Object.entries(getNonLFBlocks(document))) {
        for (const range of rangeList) {
            ret.push(range);
        }
    }
    return ret;
}

/**
 * Returns an object with the list of ranges corresponding to each type
 * of block that is not written in Lingua Franca.
 * @param document - The document from which to extract ranges
 * @returns an object with the list of ranges corresponding to each type
 *     of block that is not LF
 */
function getNonLFBlocks(document: TextDocument): {
    comments: Range[], strings: Range[], code: Range[]
} {
    // TODO implement caching. It is likely that this same function will
    // be called on the same arguments several times consecutively.
    const ret = {comments: [], strings: [], code: []};
    // FIXME: This is not DRY. These patterns are already in the TextMate
    // grammar. In fact, this problem underscores the fact that the entire
    // function is redundant -- a hack used to circumvent the fact that
    // the VS Code API does not permit access to the labels assigned by
    // the TextMate grammar.
    const patterns = {
        comments: [['/*', '*/'], ['//', '\n'], ['#', '\n']],
        // Order matters in the following array! ''' must precede '
        strings: [['"""', '"""'], ["'''", "'''"], ["'", "'"], ['"', '"']],
        code: [['{=', '=}']]
    };
    const text = document.getText();
    let currentOffset = 0;
    while (true) {
        let firstBlockType: string;
        let firstPair: string[];
        let firstOffset = Infinity;
        for (const [blockType, pairs] of Object.entries(patterns)) {
            for (const pair of pairs) {
                let offset = text.indexOf(pair[0], currentOffset);
                if (offset != -1 && offset < firstOffset) {
                    firstOffset = offset;
                    firstPair = pair;
                    firstBlockType = blockType;
                }
            }
        }
        if (firstOffset === Infinity) {
            return ret;
        }
        let endOffset = text.indexOf(
            firstPair[1], firstOffset + firstPair[0].length);
        endOffset = endOffset === -1 ? text.length - 1
            : endOffset + firstPair[1].length - 1
        ret[firstBlockType].push(new Range(
            document.positionAt(firstOffset),
            document.positionAt(endOffset)
        ));
        currentOffset = endOffset + 1;
    }
}

/**
 * Returns a representation of the set difference of the Positions
 * contained by `range` and `shadowRanges`, respectively.
 * @param range - A Range that may intersect with some elements of
 *     shadowRanges
 * @param shadowRanges - An array of ranges
 * @returns a list of disjoint ranges whose union contains a Position
 *     iff (`range` contains that position, and no element of
 *     `shadowRanges` contains that position)
 */
function setDiff(
    document: TextDocument, range: Range, shadowRanges: Range[]
): Range[] {
    // This implementation is quadratic in the length of shadowRanges, I
    // believe, and can possibly be improved upon by sorting `shadowRanges`
    // by both start and end. That would permit a binary search...
    const ret: Range[] = [];
    const next = getNext(document);
    const prev = getPrev(document);
    let pos = range.start;
    while (range.contains(pos)) {
        do {
            var passed = true;
            for (let shadow of shadowRanges) {
                if (shadow.contains(pos)) {
                    pos = next(shadow.end);
                    passed = false;
                }
            }
        } while (!passed);
        let newRange = new Range(pos, range.end);
        do {
            passed = true;
            for (let shadow of shadowRanges) {
                if (newRange.contains(shadow.start)) {
                    // The following line depends on the invariant (which is
                    // maintained!) that in this context, shadow.start cannot
                    /// be the first character in the document.
                    newRange = new Range(newRange.start, prev(shadow.start));
                    passed = false;
                }
            }
        } while (!passed);
        if (range.contains(newRange)) {
            ret.push(newRange);
        }
        pos = next(newRange.end);
    }
    return ret;
}

/**
 * Returns the ranges that appear strictly later than a match to begin
 * but strictly before a corresponding match to end. For example, if
 * begin is '(' and end is ')', then this function would return a range
 * including the entirety of '(a(b(c)d)e)', except for the first and last
 * character. The nesting depth of a token is never negative.
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
 * @param shadowRanges - The ranges which detected matches to `begin`
 *     and `end` may not intersect
 */
function getContainedRanges(
    document: TextDocument,
    range: Range,
    begin: string,
    end: string,
    shadowRanges: Range[] = []
): Range[] {
    const ret = [];
    let depth = 0;
    let rangeStart: Position = undefined;
    const next = getNext(document);
    const prev = getPrev(document);
    for (let pos = next(range.start); range.contains(pos); pos = next(pos)) {
        let candidateBegin: Range;
        let candidateEnd: Range;
        try {
            candidateBegin = new Range( // FIXME: Eliminate + 1 and next(pos)?
                pos.line, pos.character - begin.length,
                pos.line, pos.character
            );
        } catch (e) { /* Do nothing */ }
        try {
            candidateEnd = new Range(
                pos.line, pos.character,
                pos.line, pos.character + end.length
            );
        } catch (e) { /* Do nothing */ }
        if (candidateBegin && document.getText(candidateBegin) === begin
            && shadowRanges.every(
                shadow => shadow.intersection(
                    // Here, I must create a new range because the behavior of
                    // `getText` does not agree with the behavior of
                    // `intersection` and `contains`. It's an ugly hack, but
                    // the real, underlying pathology is in the API itself.
                    new Range(candidateBegin.start, prev(candidateBegin.end))
                ) === undefined)
            && (begin !== end || depth == 0)
        ) {
            depth++;
            if (depth === 1) {
                rangeStart = pos;
            }
        }
        if (candidateEnd && document.getText(candidateEnd) === end
            && shadowRanges.every(
                shadow => shadow.intersection(
                    new Range(candidateEnd.start, prev(candidateEnd.end))
                ) === undefined
        )) {
            if (depth > 0) {
                if (depth === 1 && rangeStart) {
                    ret.push(new Range(rangeStart, prev(pos)));
                    rangeStart = undefined;
                }
                depth--;
            }
        }
    }
    if (rangeStart) {
        ret.push(new Range(rangeStart, range.end));
    }
    return ret;
}

/**
 * Returns an array of objects representing the locations of reactors in
 * `document`.
 * @param document - A document in which to search for reactors
 * @param range - The range in which to search for reactors
 * @return an array of objects representing the locations of reactors in
 *     `document`
 */
function getReactors(
    document: TextDocument, range: Range
): {declaration: Range, body: Range}[] {
    const stdShadow = standardShadow(document);
    const ret: {declaration: Range, body: Range}[] = [];
    for (const reactorDeclaration of getContainedRanges(
        document, range, 'reactor', '{', stdShadow
    )) {
        const reactorBody = getContainedRanges(
            document, new Range(reactorDeclaration.end, range.end), '{', '}',
            stdShadow
        )[0]; // FIXME: Unnecessary computations are performed here.
        ret.push({declaration: reactorDeclaration, body: reactorBody})
    }
    return ret;
}

/**
 * Associates `tokenType` and `tokenModifiers` with matches to elements
 * of `tokens` that appear in some element of `ranges`. All elements of
 * `tokens` must be words: Punctuation marks, for example, will not be
 * tagged.
 * @param document - The document in which to search for tokens
 * @param ranges - The ranges in `document` in which to search for tokens
 * @param tokens - The tokens for which to search
 * @param tokenType - The token type associated with the given tokens
 * @param tokenModifiers - The modifiers associated with the given tokens
 * @param tokensBuilder - The object that records the range-token type
 *     associations generated by this function
 */
function applyTokenType(
    document: TextDocument, ranges: Range[], tokens: string[],
    tokenType: string, tokenModifiers: string[],
    tokensBuilder: SemanticTokensBuilder
) {
    if (!tokens.length) return;
    const next = getNext(document);
    for (const range of ranges) {
        const rangeOffset = document.offsetAt(range.start);
        // New range is created as a hack to work with their API inconsistency
        const rangeText = document.getText(
            new Range(range.start, next(range.end)));
        const regex = new RegExp('\\b' + tokens.join('\\b|\\b') + '\\b', 'g');
        let groups: string[];
        while ((groups = regex.exec(rangeText)) != null) {
            let token = new Range(
                document.positionAt(
                    rangeOffset + regex.lastIndex - groups[0].length),
                document.positionAt(rangeOffset + regex.lastIndex)
            );
            tokensBuilder.push(token, tokenType, tokenModifiers);
        }
    }
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
    const stdShadow = standardShadow(document);
    const nonLFBlocks = getNonLFBlocks(document);
    const whole = wholeDocument(document);
    for (const reactor of getReactors(
        document, wholeDocument(document)
    )) {
        // TODO: Highlight the type parameter throughout
        const typeParameters: string[] = [];
        for (const typeParameterGroup of getContainedRanges(
            document, reactor.declaration, '<', '>', stdShadow
        )) {
            const lfRanges = setDiff(document, typeParameterGroup, stdShadow);
            for (const lfRange of lfRanges) {
                for (const word of getWords(document, lfRange)) {
                    tokensBuilder.push(word, 'typeParameter');
                    typeParameters.push(document.getText(word));
                }
            }
        }
        const affectedRanges = setDiff(
            document, reactor.body,
            nonLFBlocks.comments.concat(nonLFBlocks.strings)
        );
        applyTokenType(
            document, affectedRanges, typeParameters, 'typeParameter', [],
            tokensBuilder
        );
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
    const stdShadow = standardShadow(document);
    const nonLFBlocks = getNonLFBlocks(document);
    for (const reactor of getReactors(
        document, wholeDocument(document)
    )) {
        // TODO: Highlight the parameter throughout
        let parameters: string[] = [];
        for (const parameterList of getContainedRanges(
            document, reactor.declaration, '(', ')', stdShadow
        )) {
            const values = getContainedRanges(
                document, parameterList, '(', ')', stdShadow
            );
            const typeValuePairs = getContainedRanges(
                document, parameterList, ':', ',', stdShadow.concat(values)
            );
            for (const nonTypeValuePair of setDiff(
                document, parameterList, typeValuePairs
            )) {
                for (const word of getWords(document, nonTypeValuePair)) {
                    parameters.push(document.getText(word));
                    tokensBuilder.push(word, 'parameter', ['readonly']);
                }
            }
            for (const typeValuePair of typeValuePairs) {
                for (const nonValue of setDiff(
                    document, typeValuePair, values
                )) {
                    for (const word of getWords(document, nonValue)) {
                        let s: String = document.getText(word);
                        if (s.charAt(0).toUpperCase() === s.charAt(0)) {
                            tokensBuilder.push(word, 'class');
                        }
                        else {
                            tokensBuilder.push(word, 'type');
                        }
                    }
                }
            }
        }
        const affectedRanges = setDiff(
            document, reactor.body,
            nonLFBlocks.comments.concat(nonLFBlocks.strings)
        );
        applyTokenType(
            document, affectedRanges, parameters, 'parameter', ['readonly'],
            tokensBuilder
        );
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
    const whole: Range = wholeDocument(document);
    const stdShadow = standardShadow(document);
    const nonLFBlocks = getNonLFBlocks(document);
    for (const reactor of getReactors(
        document, wholeDocument(document)
    )) {
        // The following is inefficient because it searches for all ranges
        // instead of just the first one. In large documents with many
        // reactors, this will turn what should be a linearly growing time
        // cost into a quadratically growing time cost. TODO: fix.
        const reactorBody: Range = getContainedRanges(
            document, new Range(reactor.declaration.end, whole.end), '{', '}',
            stdShadow
        )[0];
        // TODO: Highlight the variable throughout
        const variables: string[] = [];
        for (const lfBlock of setDiff(document, reactorBody, stdShadow)) {
            const text = document.getText(lfBlock);
            let matches = text.match(
                /((?<=(action|timer|state|((input|output)(\[[^\]]*\])?))\s+)(\w+))|((?<=(;|^)\s*)\w+(?=\s*=))/g
            );
            if (matches) {
                for (const match of matches) {
                    variables.push(match);
                }
            }
        }
        const affectedRanges = setDiff(
            document, reactor.body,
            nonLFBlocks.comments.concat(nonLFBlocks.strings)
        );
        applyTokenType( // TODO: Possibly distinguish declaration?
            document, affectedRanges, variables, 'variable', [],
            tokensBuilder
        );
    }
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
        return tokensBuilder.build();
    }
}