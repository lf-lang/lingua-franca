# Status

This is not exhaustive. Ideally each of those bullet points would have a test case.

## Language

- [x] reactor composition
  - [x] `CompositionInitializationOrder.lf`: startup reactions are called bottom-up
  - [x] `CompositionWithPorts.lf`: port bindings work
- [ ] ports
    - [x] values
    - [ ] missing values at some tags
    - [x] port value is cleaned up at the end of a tag
    - [ ] connections
      - [ ] `PortConnectionInSelfInChild.lf`: input of self to input of child
      - [ ] `PortConnectionInSelfOutSelf.lf`: input of self to output of self
      - [ ] `PortConnectionOutChildOutSelf.lf`: output of child to output of self
      - [x] `CompositionWithPorts.lf`: output of child to input of child
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
- [ ] imports
- [x] logical actions
    - [x] `ActionImplicitDelay.lf`: scheduling an action with no additional delay uses its implicit delay 
    - [x] `ActionDelay.lf`: 
    - [x] `ActionScheduleMicrostep.lf`: an action scheduled with a zero delay is only triggered on the next microstep
    - [x] `ActionValues.lf`: scheduling an action with a value at multiple different tags preserves each value
    - [x] `ActionValuesCleanup.lf`: action value is cleaned up at the end of a tag
    - [x] `ActionIsPresent.lf`: function `is_action_present` checks whether an action is present at the current tag
    - [x] `ActionIsPresentDouble.lf`: several actions may be present at the same tag
- [ ] physical actions
- [x] timers
  - [x] `TimerDefaults.lf`: timer with all params defaulted (`timer t;`) is non-periodic and has offset zero
  - [x] `TimerPeriodic.lf`: timer can be periodic
  - [ ] timer is not accessible from within reactions, cannot be scheduled manually
- [x] `shutdown` trigger & `request_stop`
  - [x] `Stop.lf`: `request_stop` schedules a shutdown at T+(1 microstep)
  - [x] `StopCleanup.lf`: ports are cleaned up before the shutdown wave executes
  - [x] `StopTopology.lf`: shutdown wave occurs in topological order like a normal wave
  - [x] `StopTimeout.lf`: `shutdown` is triggered even if the program exits because of timeout target property
  - [x] `StopNoEvent.lf`: `shutdown` is triggered even if the program exits because of an empty event queue
  - [x] `StopIdempotence.lf`: `request_stop` may be called within the shutdown wave, but it should have no effect.
- [x] state variables
  - [x] support time type
  - [x] are accessible within reactions
  - [x] are *not* accessible within initializers
  - [x] are initialized to their proper value
- [x] reactor parameters
  - [x] `CtorParamSimple.lf`: ctor parameters are accessible in initializers and reactions
  - [x] `CtorParamDefault.lf`: ctor arguments may be defaulted
  - [x] `CtorParamMixed.lf`: ctor arguments may be mentioned in any order, even with defaulted parameters
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
- [ ] multiports

### Runtime

- [ ] parallelize independent computation
