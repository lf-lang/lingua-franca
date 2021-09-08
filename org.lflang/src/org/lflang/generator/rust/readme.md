# Status

This is not exhaustive. Ideally each of those bullet points would have a test case.

## Language

- [x] reactor composition
- [ ] ports
    - [x] values
    - [ ] missing values at some tags
    - [x] port value is cleaned up at the end of a tag
    - [x] connections
      - [x] input of self to input of child
      - [x] input of self to output of self
      - [x] output of child to output of self
      - [x] output of child to input of child
    - [ ] mutable inputs
- [ ] reaction dependency handling
    - [ ] trigger dependencies
      - [x] on an input of this reactor
      - [ ] on an output of a child reactor
      - [x] on an action
    - [ ] uses-dependencies
      - [ ] on an input of this reactor
      - [ ] on an output of a child reactor
      - [ ] on an action
    - [ ] effects-dependency
      - [x] on a output of this reactor
      - [ ] on an input of a child reactor
      - [x] on a logical action
    - [ ] reaction priority is respected
      - [x] locally
      - [ ] between different child reactors fixme
- [x] logical actions
    - [x] delays
    - [x] actions with values
    - [x] action value is cleaned up at the end of a tag
    - [x] check whether the action is present
- [ ] physical actions
- [x] timers
  - [x] `TimerDefaults`: timer with all params defaulted (`timer t;`) is non-periodic and has offset zero
  - [ ] timer with delay
  - [ ] timer is not accessible from within reactions, cannot be scheduled manually
- [x] `shutdown` trigger & `request_stop`
  - [x] `Stop.lf`: `request_stop` schedules a shutdown at T+(1 microstep)
  - [x] `StopCleanup.lf`: ports are cleaned up before the shutdown wave executes
  - [x] `StopTopology.lf`: shutdown wave occurs in topological order like a normal wave
  - [x] `StopTimeout.lf`: `shutdown` is triggered even if the program exits because of timeout target property
  - [x] `StopIdempotence.lf`: `request_stop` may be called within the shutdown wave, but it should have no effect.
- [x] state variables
  - [x] support time type
  - [x] are accessible within reactions
  - [x] are *not* accessible within initializers
  - [x] are initialized to their proper value
- [x] reactor parameters
  - [x] accessible in initializers
  - [x] accessible in reactions
  - note: must be `Clone`
- [ ] array types
  - [x] support fixed-sized arrays
  - [x] support `Vec`
  - [x] support array initializer syntax
  - [ ] support array assignment syntax (fixme: doesn't exist in LF)
- [ ] deadlines
  - ...
- [ ] reactor inheritance
  - ...

### Runtime

- [ ] parallelize independent computation
