# PretVM Bytecode Optimizers for LF Programs
Author: Shaokai Lin <shaokai@berkeley.edu>
Last Updated: April 29, 2024

## Table of Contents
- [Introduction](#introduction)
- [Peephole Optimizer](#peephole-optimizer)
- [DAG-based Optimizer](#dag-based-optimizer)
- [Current Progress](#current-progress)

## Introduction
Lingua Franca (LF) is a polyglot coordination language for building
deterministic, real-time, and concurrent systems.
Here is a simple example for illustrating the use of LF.
```C
target C
reactor Sensor1 {
    output out:int
    timer t(0, 100 msec)
    state count:int = 0
    reaction(t) -> out {= /* Application logic */ =}
}
reactor Sensor2 {
    output out:int
    timer t(0, 50 msec)
    state count:int = 0
    reaction(t) -> out {= /* Application logic */ =}
}
reactor Processing {
    input in1:int
    input in2:int
    output out:int
    reaction(in1, in2) -> out {= /* Application logic */ =}
}
reactor Actuator {
    input in:int
    reaction(in) {= /* Application logic */ =}
}
main reactor {
    s1 = new Sensor1()
    s2 = new Sensor2()
    p = new Processing()
    a = new Actuator()
    s1.out -> p.in1
    s2.out -> p.in2
    p.out -> a.in
}
```
The pipeline first starts with two sensors, `Sensor1` and `Sensor2`, each wrapped in a `reactor`
definition. Reactors are stateful containers that can communicate with the
outside world through ports and have reactive code blocks called "reactions." The reactor
definition of  `Sensor1` contains an
output port `out` with a type `int`. Since a sensor is typically triggered
periodically, a timer named `t` is defined with an initial offset of `0` and a period of
`100 msec`.
Due to the stateful nature of reactors, state variables can be defined within
a reactor definition. In this case, we define a state variable called `count`
with the type `int`.
Finally, a reaction, i.e., a reactive code block that gets triggered upon the
arrival of certain triggers, is defined inside `Sensor1` that is sensitive to
each firing of the timer `t`. Inside the `{=...=}` bracket, the user can write
the application logic of the reaction in a programming language of choice. In
this example, a C target is declared on top, so the program accepts C code
inside reaction bodies. At the time of writing, LF support C/C++, Python, Rust,
and TypeScript as target languages.

`Sensor2`, `Processing`, and `Actuator` can be defined in a similar manner. Once
all reactor definitions are provided, connection statements can be used to
connect the reactors' ports, which allow them to send messages to each other.

All events in the system are handled in the order of tags. Reactions are logically
instantaneous; logical time does not elapse during the execution of a
reaction (physical time on the other hand, does elapse). If a reaction
produces an output that triggers another reaction, then the two reactions execute
logically simultaneously (i.e., at the same tag).

Determinism is a central feature of LF, and LF achieves this by establishing
a logical order of events through logical time. In this example, `Sensor1`, with
a period of `100 msec` and `Sensor2`, with a period of `50 msec`, send messages to a common
downstream reactor `Processing`. The LF semantics ensure that every output of
`Sensor2` is aligned with every other output of `Sensor1`.

After an LF program is specified, we analyze
the state space of the program and find a DAG (directed acyclic graph) for each
phase of an LF program. The generated DAG is
then partitioned and mapped to available workers.

Once the partitioned DAGs are generated, the LF compiler generates PretVM
bytecode from them. The table bwlow shows the instruction set.

| Instruction | Description |
|:-------- |:--------:|
| ADD  op1, op2, op3 |  Add to an integer variable (op2) by an integer variable (op3) and store to a destination register (op1).   |
| ADDI op1, op2, op3 |   Add to an integer variable (op2) by an immediate (op3) and store to a destination register (op1).   |
| ADV  op1, op2, op3 | Advance the logical time of a reactor (op1) to a base time register (op2) + a time increment variable (op3). |
| ADVI op1, op2, op3 | Advance the logical time of a reactor (op1) to a base time register (op2) + an immediate value (op3). |
| BEQ  op1, op2, op3 | Take the branch (op3) if the op1 register value is equal to the op2 register value. |
| BGE  op1, op2, op3 | Take the branch (op3) if the op1 register value is greater than or equal to the op2 register value. |
| BLT  op1, op2, op3 | Take the branch (op3) if the op1 register value is less than the op2 register value. |
| BNE  op1, op2, op3 | Take the branch (op3) if the op1 register value is not equal to the op2 register value. |
| DU   op1, op2 | Delay until the physical clock reaches a base timepoint (op1) plus an offset (op2). |
| EXE  op1 | Execute a function (op1). |
| JAL  op1, op2 | Store the return address to op1 and jump to a label (op2). |
| JALR op1, op2, op3 | Store the return address to op1 and jump to a base address (op2) + an immediate offset (op3). |
| STP | Stop the execution. |
| WLT  op1, op2 | Wait until a register value (op1) to be less than a desired value (op2). |
| WU   op1, op2 | Wait until a register value (op1) to be greater than or equal to a desired value (op2). |

PretVM bytecode encodes a system's "coordination logic," which is
separated from its "application logic."
One major advantage of encoding the coordination logic in the form of bytecode
is that this format is amenable to optimization,
just like other types of bytecode, such
as JVM
bytecode, LLVM bitcode, and EVM (Ethereum virtual machine) bytecode, which likely
result in more efficient execution of the coordination logic.

This document aims to describe the high-level objectives of the bytecode
optimizer and serves as a technical documentation for the ongoing development.

## Peephole Optimizer

### Why do we need this?

1. (Done) Redundant `WU` instructions could be removed.

`WU` instructions are used to ensure that task dependencies are satisfied across
workers. For example, the code example above could generate the following
bytecode when the program is executed by two workers. Let's focus on `Sensor1`,
`Sensor2`, and `Processing` for now. Let's further assume that `Sensor1` and `Sensor2` are
mapped to worker 0, while `Processing` is mapped to worker 1.

The code generator currently generates code with the follwing form
```
Worker 0:
1. EXE Sensor1's reaction
2. ADDI worker 0's counter by 1
3. EXE Sensor2's reaction
4. ADDI worker 0's counter by 1

Worker 1:
5. WU worker 0's counter reaches 1
6. WU worker 0's counter reaches 2
7. EXE Processing's reaction
8. ADDI worker 1's counter by 1,
```
The code we want to optimize away is line 5 in worker 1. Clearly, we can safely
remove line 5 here because line 6 is waiting for a greater release value, i.e.,
2 > 1.

The number of `WU`s could grow linearly with the number of upstream
dependencies. If instead of two sensors, there are a thousand sensors, then there will
be a thousand `WU`s generated. On a Raspberry Pi 4, each instructon takes about
two microseconds. 1000 `WU`s could take around 2 milliseconds, which is a long
time in the software world.

2. (WIP) Time advancements, via the `ADV` and the `ADVI` instruction, could
   potentially be grouped, forming enclaves.

A similar (though subtly different) situation is time advancement. PretVM
currently uses the `ADV` and the `ADVI` instruction to advance reactor
timestamps. While a subset of reactors advance time in the middle of the
bytecode program, all reactors advance time to the next hyperperiod at the end
of the bytecode program. So at the end of the bytecode program, usually we see
the following pattern:
```
Worker 0:
1. ADV reactor 1's time to time T
2. ADV reactor 2's time to time T
3. ADV reactor 3's time to time T
...
```
It would be ideal to be able to collapse the sequence of `ADV`s into a single
one.
```
Worker 0:
1. ADV a shared time register to time T
```

However, this is more complicated in practice. The problem is the reactors that
advance time in the middle of the bytecode program. For example, in the above
example, since the minimal hyperperiod is 100 milliseconds, `Sensor2`, triggered
once every 50 milliseconds, needs to advance time once in the middle of the
hyperperiod and another at the end of the hyperpeiord, while `Sensor1`,
triggered every 100 milliseconds, only advances time at the end of the
hyperperiod.
If they share the
same time register, advancing `Sensor2`'s time in the middle of the hyperperiod
will inadvertantly advance `Sensor1`'s time. In more complicated programs, doing
so could result in certain reactors advancing time without finishing all the
work prior to the new tag.

A general solution to safely partition the reactors into regions that share time
registers is still work-in-progress.

### How is this optimizer implemented?

The above optimizations, especially the first one, are performed by the peephole
optimizer, which is a concrete strategy for implementing a term-rewriting system
(TRS).

The peephole optimizer works by focusing on a basic block and uses a sliding
window to find opportunities to apply rewrite rules.
For instructions within a current window, registered rewrite rules are checked to see
if they are applicable. If so, the rewrite rules are applied and transform the code.

Specifically, to eliminate redundant `WU`s, a pattern requiring a window of size two is
provided:
`WU counter, X ; WU counter, Y ==> WU counter, max(X, Y)`. The concrete
Java implementation can be found
[here](https://github.com/lf-lang/lingua-franca/blob/e2512debbba3726a85493a15885d10cc3f11c8d6/core/src/main/java/org/lflang/analyses/opt/PeepholeOptimizer.java#L55).

Given an infrastructure for peephole optimization has been set up, it will be
easy to add more optimizations, if applicable.

## DAG-based Optimizer

### Why do we need this?

Another optimizer currently under development is a DAG-based optimizer, which
focuses on the DAGs generated from an LF program and tries to perform procedure
extraction.

The basic idea is that each node, except the tail node, in the DAG represents a
reaction invocation and
could generate a short sequence of instructions; given that the same reaction
could be triggered multiple times, it should be possible to factor out the
instructions from a frequently invoked reaction into a procedure, and call
the procedure at multiple times during execution, instead of generating
duplicate instructions.

For example, `Sensor2`'s reaction could contribute the following instructions:
```
1. EXE Sensor2's reaction
2. ADDI worker_counter by 1
3. EXE connection management helper function
```
Since `Sensor2`'s reaction is invoked twice in the hyperperiod, instead of
generating the above sequence of instructions twice, we could first factor them
out into a procedure:
```
PROCEDURE_SENSOR_2:
1. EXE Sensor2's reaction
2. ADDI worker_counter by 1
3. EXE connection management helper function
4. JALR return_address
```
Then in the main procedure, jump to the procedure twice:
```
Worker 0:
1. JAL PROCEDURE_SENSOR_2
2. ADVI reactor's time
3. JAL PROCEDURE_SENSOR_2
```

### How is this optimizer implemented?

The DAG-based optimizer is implemented based on DAG traversal. It utilizes the
existing DAG structure in the tasks and treats the instructions generated by
each node as a basic block, which it aims to factor out if need be.

The DAG-based optimizer maintains two key data structures, a list of equivalence
classes for DAG nodes and a mapping from a node to an index in the equivalence
class list.

The optimizer traverses a DAG in the order of topological sort twice. In the
first pass, the optimizer populates the equivalence classes and the
node-to-index mapping. It considers two
nodes in the same equivalence class if they yield identical instructions. In the
second pass, the optimizer aims to generate an updated bytecode by first putting
procedure code in the bytecode, then uses `JAL` to jump to the procedures in the
main procedure.

A work-in-progress implementation can be found
[here](https://github.com/lf-lang/lingua-franca/blob/e2512debbba3726a85493a15885d10cc3f11c8d6/core/src/main/java/org/lflang/analyses/opt/DagBasedOptimizer.java#L24).
It is not fully working yet given the following challenges: 1. existing
labels may need to be collapsed and shared somehow, 2. the connection management
functions need to be parameterized, 3. it is unclear whether the existing
algorithm will work when multiple workers are involved.
Overall, it is still a work-in-progress.

## Current Progress

At the time of writing, the infrastructure of the peephole optimizer has been
set up, and the optimization that removes redundant `WU`s is fully operational.

A test case, [RemoveWUs.lf](https://github.com/lf-lang/lingua-franca/blob/e2512debbba3726a85493a15885d10cc3f11c8d6/test/C/src/static/RemoveWUs.lf), finishes in `881 msec` after the optimization, and in
`1010 msec` before the optimization, an `12.8%` improvement, measured on macOS with
2.3 GHz 8-core Intel Core i9.

The time advancement optimization and procedure extraction are both not finished
at the time of writing and are under active developement.
