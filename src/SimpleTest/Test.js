// Code generated for this particular actor.
// *********** From the preamble, verbatim:
        var i = 0;
// *********** End of preamble.

// Generated setup function:
exports.setup = function() {
    this.parameter('expected', {'type':'JSON', 'value':'[]'});
    this.input('x', {'type':'int'});
}

// Generated initialize function:
exports.initialize = function() {
    var expected = this.getParameter('expected');
    // *********** From initialize, verbatim:
        i = 0;
    // *********** End of initialize.
    this.addInputHandler('x', reaction_x);
}
function reaction_x() {
    var expected = this.getParameter('expected');
    var x = this.get('x');
    // *********** From reaction, verbatim:
        if (expected[i] != x) {
            throw "Expected: " + expected[i] + ", but got: " + x;
        }
        i += 1;
        if (i >= expected.length) {
            this.stop();
        }
    // *********** End of reaction code.
}
// Bind the reaction function to this actor.
reaction_x.bind(this);
