exports.setup = function() {
    var a = this.instantiate('a', 'Source');
    a.setParameter('period', 10);
    var b = this.instantiate('b', 'Test');
    b.setParameter('expected", '[1, 2, 3, 4]');
    this.connect(a, 'y', b, 'x');
}


