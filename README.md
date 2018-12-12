# A Model of Computation using Pret Machines (Precision Timed Computation)

## Proposed names for this project
  * Intermezzo
  
  
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

## Questions

 * Example applications?
   * Real-time audio, time sensitive peripherals, control loop
 * What concurrency model do we have? One or more threads per actor?
   * Single thread per actor solve the Rust ownership issue
 * What is the semantics of the delay operation?
   * Maybe like a yield in a coroutine?

## Prototype Implementation

We would like to provide a prototype implementation on
[FlexPRET](https://github.com/pretis/flexpret) and
[Patmos](https://github.com/t-crest/patmos),
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

### Shared State

Within one actor there is shared state, which may be muted by different handlers.
If those handlers are atomic with each other, we can simply pass a reference
to this shared state. The shared state is allocated at initialization time.

### Two Versions of Delay

We will explore two versions for the delay: (1) a busy wait and
(2) scheduling a handler for an invocation at a later time.


handler is `foo()`

#### Version 1:

```
foo() {
   do work A
   delay until time x
   do work B
}
```

This needs to be a blocking delay, if work A and B need to be atomic.

#### Version 2:

```
foo() {
   work
   call foo at time x // or any other function
}
```

This registers foo (or a different function) for a callback at time x.
This needs no blocking during the delay, as there is no atomic action during the delay.

### Rust

 * Safe system level language
 * No compiler for FlexPRET or Patmos
   * There might be some work going on for RISC-V: https://abopen.com/news/rust-comes-risc-v/
 * No WCET analysis
 * Actor library: https://github.com/actix/actix
 * DSL in Rust: https://doc.rust-lang.org/rust-by-example/macros/dsl.html
 
Rust is LLVM based. Can we use the Rust frontend and our LLVM backend?
We briefly explored it with the Patmos LLVM port, but Rust libraries are called.
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

## Rust/Tock discussion with Pat

 * To get a small kernel, exclude std library with a switch and manually import
   from core what is needed
 * Use nightly build from Rust to get latest support
 * Brad is working on Rust for RISC-V, which we should be able to use
 * Working on  getting https://www.tockos.org/ running on RISC-V
 * Tock OS (kernel) is single threaded with yield(), but applications
   are running in individual threads
 * Rust embedded group (IRC available)
 * Tock has a public Slack channel

## Uses cases

### Example 1
Given a time unit c,
 * H3 reacts sporadically >= 100c (e.g., 10, 120, 230, ...)
 * H4 reacts periodically with period 50c (e.g., 0, 50, 100, ...)
 * Delay adds 100c to the timestamp of each incoming event
 * Actuate shall start executing H5 _before_ r.t. clock exceeds time stamp of incoming events
 
```
+--------+
|        |          +--------+     +-------+     +---------+
|   H3   +----------> H1     |     |       |     |         |
|        |          |        +-----> Delay +-----> Actuate |
+--------+    +-----> H2     |     |  100  |     |   (H5)  |
              |     +--------+     +-------+     +---------+
              |
+--------+    |
|        |    |
|   H4   +----+
|        |
+--------+

```

We can construct a dependency graph:

```
H3 ---> h1 ---> H2 ---> H5
    |
H4 -+
```

A feasible schedule requires that:
 * WCET(H3) + WCET(H1) + WCET(H2) <= 100c
 * WCET(H4) + WCET(H1) + WCET(H2) <= 100c
