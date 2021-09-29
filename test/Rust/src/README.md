# Status

This is not exhaustive. Ideally each of those bullet points would have a test case.

## Language

- [x] reactor composition
  - [x] `CompositionInitializationOrder.lf`: startup reactions are called bottom-up
  - [x] `CompositionWithPorts.lf`: port bindings work
  - [x] `GenericReactor.lf`: generic reactors may compose, types are properly instantiated
- [ ] ports
    - [x] `PortValueCleanup.lf`: port value is cleaned up at the end of a tag
    - [x] `PortRefCleanup.lf`: port value is cleaned up at the end of a tag, when the upstream is a port reference
    - [x] connections...
      - [x] `PortConnectionInSelfInChild.lf`: input of self to input of child
      - [x] `PortConnectionInSelfOutSelf.lf`: input of self to output of self
      - [x] `PortConnectionOutChildOutSelf.lf`: output of child to output of self
      - [x] `CompositionWithPorts.lf`: output of child to input of child
    - [ ] mutable inputs
- [ ] reaction dependency handling
    - dependencies can be declared...
      - [ ] on ports of this reactor
      - [x] `DependencyOnChildPort.lf`: on ports of a child reactor
      - [ ] on an action
    - [ ] trigger dependencies
      - [ ] `todo.lf`: trigger dependencies trigger reactions
      - [ ] `todo.lf`: multiple trigger dependencies may be triggered independently
    - [ ] uses-dependencies
      - [x] `DependencyUseNonTrigger`: use dependencies do not trigger reactions 
      - [x] `DependencyUseAccessible`: use dependencies make the port accessible within the reaction, values are observable 
      - [ ] `todo.lf`: use dependencies may be declared on actions
    - [ ] effects-dependency
      - [ ] `todo.lf`: effects dependencies ...
      - [x] `todo.lf`: on a logical action
    - [ ] reaction priority is respected
      - [x] locally
      - [x] between different child reactors
- [ ] imports
- [x] preambles
  - [x] `Preamble.lf`: preamble within reactor
  - [ ] top-level preamble
- [x] logical actions
    - [x] `ActionImplicitDelay.lf`: scheduling an action with no additional delay uses its implicit delay 
    - [x] `ActionDelay.lf`: 
    - [x] `ActionScheduleMicrostep.lf`: an action scheduled with a zero delay is only triggered on the next microstep
    - [x] `ActionValues.lf`: scheduling an action with a value at multiple different tags preserves each value
    - [x] `ActionValuesCleanup.lf`: action value is cleaned up at the end of a tag
    - [x] `ActionIsPresent.lf`: function `is_present` checks whether an action is present at the current tag
    - [x] `ActionIsPresentDouble.lf`: several actions may be present at the same tag
- [ ] physical actions
- [x] timers
  - [x] `TimerDefaults.lf`: timer with all params defaulted (`timer t;`) is non-periodic and has offset zero
  - [x] `TimerPeriodic.lf`: timer can be periodic
  - [x] `TimerIsPresent.lf`: timer should be queryable with `is_present`
  - [x] timer cannot be scheduled manually
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
  - [x] `TypeVarLengthList.lf`: support variable length lists (`Vec`)
  - [x] support array initializer syntax
  - [ ] support array assignment syntax (fixme: doesn't exist in LF)
- [ ] deadlines
  - ...
- [ ] reactor inheritance
  - ...
- [ ] multiports

### Runtime

- [ ] parallelize independent computation
- [ ] keepalive option
- [ ] timeout option
