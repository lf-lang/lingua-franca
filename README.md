# A Model of Computation using Pret Machines (Precision Timed Computation)

Notes from an initial discussion.

## Objectives

 * Well defined simultaneity
 * Support parallel & distributed
 * Be able to construct deterministic
 * Explicitly allow nondeterminism

## Features

 * Islands of atomicity <=> actors
 * Coordination == DE

## Mutually Atomic

Private data
Time stamps on incoming messages

Handler for actor

 * A1 computation on private data
 * A2 send events to output ports
 * A3 request invocation of p at t + delta
 * A4 read any inputs & either get a value or null
 * A5 access current time == time stamp of input
 * A6 request an async, callback of proc (what is current time?)

## TODO

add rest of notes.


