// Boilerplate included for all actors.
var PERIODIC = true;
var ONCE = false;
function schedule(trigger, time, isPeriodic) {
    if (isPeriodic) {
        return trigger.actor.setInterval(trigger.reaction, time);
    } else {
        return trigger.actor.setTimeout(trigger.reaction, time);
    }
}
function setUnbound(port, value) {
    if (!port) {
        throw "Illegal reference to undeclared output.";
    }
    this.send(port, value);
}
var set = setUnbound.bind(this);
// Code generated for this particular actor.
// Trigger data structure:

exports.setup = function() {
    var b = this.instantiate('b', 'utilities/Clock.js');
    b.setParameter('interval', 100.0);
    var a = this.instantiate('a', 'test/TrainableTest.js');
    a.setParameter('correctValues',  [0,1,2,3,4,5,6,7,8,9,10] );
    var c = this.instantiate('c', 'utilities/Stop.js');
    this.connect(a, 'output', b, 'input');
    this.connect(b, 'output', c, 'stop');
}
exports.initialize = function() {

}
// Reaction 

