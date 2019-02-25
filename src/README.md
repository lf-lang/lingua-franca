# Code samples for use in developing the language.

Files with the extension ".lf" are meant to be source files containing actor
definitions specified in a combination of the Lingua Franca interface language
and a host language.

## Actors
 * Source.lf: Actor with one state variable, one output, and periodic output events.
 * Source.js: Manually generated JavaScript output from LF compiler. Tested in CapeCode. NEEDS UPDATING TO USE set().
 * Counter.lf: Actor where the order of reactions determines whether there is direct feedthrough from input to output.
 * Asynchronous.lf: Actor that produces an asynchronous output in response to an input. WON'T WORK!
 * AsynchronousIndirect.lf: Variant using a level of indirection that could work.
 * Destination.lf: Sink actor that prints the input.
 
## Compositions
 * SimpleTest.lf: Two actors, one a test actor, and their composition.

## Testing using the node host

SimpleTest.lf, when compiled, should produce the files in the subdirectory SimpleTest.
Shell command:

node $PTII/org/terraswarm/accessor/accessors/web/hosts/node/nodeHostShell.js
  
