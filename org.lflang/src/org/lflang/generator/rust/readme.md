# Status

Things that are supported:
- logical actions, incl. delays, actions with values, check whether the action is present
- timers
- composition of reactors (with a caveat)
- reactor parameters and state vars
- array types are poorly supported right now because of LF syntax (todo open issues)


Not supported yet:
- the dependency ordering in the runtime is a placeholder and doesn't work properly, this is maybe the biggest roadblock right now
- the runtime is not multithreaded yet
- deadlines and timely behavior
- reactor inheritance (there's no inheritance in Rust though, what would that look like)
- mutable inputs
- physical actions
