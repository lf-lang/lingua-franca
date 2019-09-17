# Lingua Franca

See the main [wiki](https://github.com/icyphy/lingua-franca/wiki) for documentation.

Lingua Franca (LF) is a polyglot metalanguage for the definition and composition of reactors, which are stateful reactive components coordinated deterministically under a discrete-event semantics. Reactors have ports, and their functionality is defined in terms of reactions, which may be sensitive to events observable on input ports and may produce events on output ports. Reactors are composed by drawing connections between ports, and thus chaining reactions. In LF, the body of a reaction is written in pure target code that references a runtime library responsible for coordinates the ensemble.

## Publications
- [Invited: Actors Revisited for Time-Critical Systems](https://ptolemy.berkeley.edu/publications/papers/19/LohstrohEtAl_Reactors_DAC_2019.pdf) (DAC '19)
- [Deterministic Actors](https://ptolemy.berkeley.edu/publications/papers/19/Lohstroh_Lee_DeterministicActors_FDL_2019.pdf) (FDL '19)
