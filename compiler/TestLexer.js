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
var t = {'actor':this, 'triggerName':'t', 'reaction':reaction_t.bind(this)};

        var n = 0;
    
// Generated setup function:
exports.setup = function() {
    this.parameter('foo', {'type':'string', 'value':"hello"});
    this.parameter('period', {'type':'int', 'value':1000});
    this.parameter('bar', {'type':'foreignType', 'value':{- @#$% -}});
    this.output('y', {'type':'int'});
}
exports.initialize = function() {
    var foo = this.getParameter('foo');
    var period = this.getParameter('period');
    var bar = this.getParameter('bar');
    schedule(t, period, PERIODIC);

        n = 0;
    
}
// Reaction 
function reaction_t() {
    var foo = this.getParameter('foo');
    var period = this.getParameter('period');
    var bar = this.getParameter('bar');
    var y = 'y'; // FIXME in original .js code

        n = n + 1;
        set(y, n);
    
}

