exports.setup = function () {
    //  This composite accessor was created by Cape Code.
    //  To run the code, run: 
    //  (cd /Users/eal/GITRepositories/lingua-franca/src/Composition; node /ptII/org/terraswarm/accessor/accessors/web/hosts/node/nodeHostInvoke.js Composition)
    //  To regenerate this composite accessor, run:
    //  $PTII/bin/ptinvoke ptolemy.cg.kernel.generic.accessor.AccessorCodeGenerator -language accessor file:/Users/eal/GITRepositories/lingua-franca/src/Composition/Composition.xml
    //  to edit the model, run:
    //  $PTII/bin/capecode file:/Users/eal/GITRepositories/lingua-franca/src/Composition/Composition.xml

    // Ports: Composition: ptolemy/cg/adapter/generic/accessor/adapters/ptolemy/actor/TypedCompositeActor.java

    // Start: TrainableTest: ptolemy/cg/adapter/generic/accessor/adapters/org/terraswarm/accessor/JSAccessor.java
    var TrainableTest = this.instantiate('TrainableTest', 'test/TrainableTest.js');
    TrainableTest.setParameter('correctValues', [0,1,2,3,4,5,6,7,8,9,10]);
    TrainableTest.setParameter('trainingMode', false);
    TrainableTest.setParameter('tolerance', 1.0E-9);

    // Start: Clock: ptolemy/cg/adapter/generic/accessor/adapters/org/terraswarm/accessor/JSAccessor.java
    var Clock = this.instantiate('Clock', 'utilities/Clock.js');
    Clock.setParameter('interval', 100.0);

    // Start: Stop: ptolemy/cg/adapter/generic/accessor/adapters/org/terraswarm/accessor/JSAccessor.java
    var Stop = this.instantiate('Stop', 'utilities/Stop.js');

    // Connections: Composition: ptolemy/cg/adapter/generic/accessor/adapters/ptolemy/actor/TypedCompositeActor.java
    this.connect(Clock, 'output', TrainableTest, 'input');
    this.connect(TrainableTest, 'output', Stop, 'stop');
};

// The stopTime parameter of the directory in the model was 0, so this.stopAt() is not being generated.

