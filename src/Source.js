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
function set(port, value) {
    this.send(port, value);
}
set.bind(this);
// Code generated for this particular actor.
// Trigger data structure:
var t = {'actor':this, 'triggerName':'t', 'reaction':reaction_t};
var y = 'y';
// *********** From the preamble, verbatim:
        var n = 0;
// *********** End of preamble.

// Generated setup function:
exports.setup = function() {
    this.parameter('period', {'type':'int', 'value':1000});
    this.output('y', {'type':'string'});
}

// Generated initialize function:
exports.initialize = function() {
    var period = this.getParameter('period');
    // *********** From initialize, verbatim:
        schedule(t, period, PERIODIC);
        n = 0;
    // *********** End of initialize.
}
function reaction_t() {
    var period = this.getParameter('period');
    // *********** From reaction, verbatim:
        n = n + 1;
        set(y, n + ": Hello World!");
    // *********** End of reaction code.
}
// Bind the reaction function to this actor.
reaction_t.bind(this);
