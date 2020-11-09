/**
 * Library used for generating usage text for command line arguments.
 * @see {@link https://www.npmjs.com/package/command-line-usage}
 */

declare module 'command-line-usage' {
    export default function commandLineUsage(optionDefinitions: Object): string;
}
