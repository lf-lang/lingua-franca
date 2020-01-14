[![Build Status](https://travis-ci.com/icyphy/lingua-franca.svg?branch=master)](https://travis-ci.com/icyphy/lingua-franca)

# Lingua Franca

See the main [wiki](https://github.com/icyphy/lingua-franca/wiki) for further information and detailed documentation.

Lingua Franca (LF) is a polyglot metalanguage for the definition and composition of reactors, which are stateful reactive components coordinated deterministically under a discrete-event semantics. Reactors have ports, and their functionality is defined in terms of reactions, which may be sensitive to events observable on input ports and may produce events on output ports. Reactors are composed by drawing connections between ports, and thus chaining reactions. In LF, the body of a reaction is written in pure target code that references a runtime library responsible for coordinating the ensemble.

## Publications
- [Reactors: A Deterministic Model for
Composable Reactive Systems](https://people.eecs.berkeley.edu/~marten/pdf/Lohstroh_etAl_CyPhy19.pdf) (CyPhy '19)
- [Work-in-Progress: Programs with Iron-clad Timing Guarantees](http://delivery.acm.org/10.1145/3360000/3351553/a1-lohstroh.pdf?ip=173.195.77.221&id=3351553&acc=OA&key=4D4702B0C3E38B35%2E4D4702B0C3E38B35%2E4D4702B0C3E38B35%2E921106E2838EC03D&__acm__=1575407821_5e0a3e321d04891b082e358b881d5bbf) (EMSOFT '19)
- [Deterministic Actors](https://ptolemy.berkeley.edu/publications/papers/19/Lohstroh_Lee_DeterministicActors_FDL_2019.pdf) (FDL '19)
- [Invited: Actors Revisited for Time-Critical Systems](https://ptolemy.berkeley.edu/publications/papers/19/LohstrohEtAl_Reactors_DAC_2019.pdf) (DAC '19)

