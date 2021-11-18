
pub struct PseudoRandomGenerator {
    m: i64,
}

impl PseudoRandomGenerator {
    pub fn new() -> Self {
        PseudoRandomGenerator {
            m: 74755,
        }
    }

    pub fn from(value: i64) -> Self {
        PseudoRandomGenerator {
            m: value,
        }
    }

    pub fn next(&mut self) -> i64 {
        self.m = ((self.m * 1309) + 13849) & 65535;
        self.m
    }
}
