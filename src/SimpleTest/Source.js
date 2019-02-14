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
// NOTE: bind() returns a new function.
// It does not alter the original function.
var set = setUnbound.bind(this);
// Code generated for this particular actor.
// Trigger data structure:
var t = {'actor':this, 'triggerName':'t', 'reaction':reaction_t.bind(this)};
// *********** From the preamble, verbatim:
        var n = 0;
// *********** End of preamble.

// Generated setup function:
exports.setup = function() {
    this.parameter('period', {'type':'int', 'value':1000});
    this.output('y', {'type':'int'});
}

// Generated initialize function:
exports.initialize = function() {
    var period = this.getParameter('period');
    schedule(t, period, PERIODIC);
    // *********** From initialize, verbatim:
        n = 0;
    // *********** End of initialize.
}
function reaction_t() {
	console.log("************************");
    console.log(util.inspect(this));
    console.log("************************");

    var period = this.getParameter('period');
    var y = 'y'; // FIXME: Too easy to cheat! User could just pass a string name
                 // of an output port to set().
    // *********** From reaction, verbatim:
        n = n + 1;
        set(y, n);
    // *********** End of reaction code.
}

