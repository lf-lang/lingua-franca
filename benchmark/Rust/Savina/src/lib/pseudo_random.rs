
pub struct PseudoRandomGenerator {
    m: u64,
}

impl PseudoRandomGenerator {
    pub fn from(value: u64) -> Self {
        PseudoRandomGenerator {
            m: value,
        }
    }

    pub fn next(&mut self) -> u64 {
        self.m = ((self.m * 1309) + 13849) & 65535;
        self.m
    }
}

impl Default for PseudoRandomGenerator {
    fn default() -> Self {
        PseudoRandomGenerator::from(74755)
    }
}
