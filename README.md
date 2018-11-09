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
 * A7 TR delay
 * A8 send to output port with delay delta/deadline t+delta (i.e., actually received at t+delta' where delta' >= delta)

## Prototype Implementation

We would like to provide a prototype implementation on FlexPRET and Patmos,
including worst-case execution time analysis of actors.
This calls for a system language that can be used to implement real-time systems.
Two options are considered: Rust and C.

### Rust

 * Safe system level language
 * No compiler for FlexPRET or Patmos
 * No WCET analysis

Rust is LLVM based. Can we use the Rust frontend and our LLVM backend?

### C

 * Old, unsafe language
 * Good compiler support (RISC-V, Patmos)
 * WCET analysis for Patmos

## TODO

add rest of notes.


