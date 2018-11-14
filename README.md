# A Model of Computation using Pret Machines (Precision Timed Computation)

Ongoing notes from discussions.

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

A DSL may be interesting to use for the definition of composite actors,
i.e hierarchical actors. A DSL within Rust may be nice.
However, just for configuration any DSL environment (e.g., Scala) would work.

Taking DSLs a step further, we can envision a DSL to also describe the
actor functions (like in Chisel) and then spill out low-level code, which
then can be C (like Verilog from Chisel), including #pragma annotation that
can help WCET analysis tools.

### Rust

 * Safe system level language
 * No compiler for FlexPRET or Patmos
   * There might be some work going on for RISC-V: https://abopen.com/news/rust-comes-risc-v/
 * No WCET analysis
 * Actor library: https://github.com/actix/actix
 * DSL in Rust: https://doc.rust-lang.org/rust-by-example/macros/dsl.html
 
Rust is LLVM based. Can we use the Rust frontend and our LLVM backend?
We briefly explored it with Patmos LLVM port, but Rust libraries are called.
This may be a several PM project (e.g., a master thesis).

### C

 * Old, unsafe language
 * Good compiler support (RISC-V, Patmos)
 * WCET analysis for Patmos

### Combinations with JavaScript Accessors

 * Node.js can run within Rust
 * Rust can also be compiled to WebAssembly (supported by JS interpreters)
 * This can be explored for mixed critical, more dynamic systems

## Further Links

 * A DSL: http://www.kframework.org/index.php/Main_Page

## TODO

add rest of notes.


