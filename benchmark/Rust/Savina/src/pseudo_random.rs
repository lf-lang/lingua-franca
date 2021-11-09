
struct PseudoRandomGenerator {
    m: i64,
}

impl PseudoRandomGenerator {
    fn new() -> Self {
        PseudoRandomGenerator {
            m: 74755,
        }
    }

    fn from(value: i64) {
        PseudoRandomGenerator {
            m: value,
        }
    }

    fn next(&mut self) -> i64 {
        self.m = ((self.m * 1309) + 13849) & 65535;
        self.m
    }
}
