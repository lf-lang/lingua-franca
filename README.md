# Lingua Franca: A Programming Model for Precision-Timed Reactive Systems
  



## Host Programming Languages

### C

 * Old, unsafe language
 * Good compiler support (RISC-V, Patmos)
 * WCET analysis for Patmos

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


##### Rust/Tock discussion with Pat

 * To get a small kernel, exclude std library with a switch and manually import
   from core what is needed
 * Use nightly build from Rust to get latest support
 * Brad is working on Rust for RISC-V, which we should be able to use
 * Working on  getting https://www.tockos.org/ running on RISC-V
 * Tock OS (kernel) is single threaded with yield(), but applications
   are running in individual threads
 * Rust embedded group (IRC available)
 * Tock has a public Slack channel

#### Combinations with JavaScript Accessors

 * Node.js can run within Rust (but requires IPC)
 * Rust can also be compiled to WebAssembly (supported by JS interpreters)
 * This can be explored for mixed critical, more dynamic systems


## Uses cases

### Precision-Timed Actuation
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
H3 ---> H1 ---> H2 ---> H5
    |
H4 -+
```

A feasible schedule requires that:
 * WCET(H3) + WCET(H1) + WCET(H2) <= 100c
 * WCET(H4) + WCET(H1) + WCET(H2) <= 100c

## Links
 * A DSL: http://www.kframework.org/index.php/Main_Page

