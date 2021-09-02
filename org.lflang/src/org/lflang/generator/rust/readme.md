# Status

## Language

- [x] reactor composition
- [ ] ports
    - [x] values
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
    - [x] check whether the action is present
- [ ] physical actions
- [x] timers
- [x] state variables
  - [x] time type
  - [x] not accessible in initializers
  - [x] accessible in reactions
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
- [ ] reactor inheritance

### Runtime

- [ ] parallelize independent computation
