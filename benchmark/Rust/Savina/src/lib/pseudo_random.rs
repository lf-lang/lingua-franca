/*
 * Copyright (c) 2021, TU Dresden.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Johannes Hayeß
 */


pub struct PseudoRandomGenerator {
    m: u64,
}

impl PseudoRandomGenerator {
    /// Reset internal state, as if this was just created with given seed.
    pub fn reseed(&mut self, seed: u64) {
        self.m = seed;
    }

    pub fn from(value: u64) -> Self { // this should be in a From impl
        PseudoRandomGenerator {
            m: value,
        }
    }

    pub fn next(&mut self) -> u64 {
        self.m = ((self.m * 1309) + 13849) & 65535;
        self.m
    }

    pub fn gen_u32(&mut self) -> u32 {
        let x = self.next();
        (x % (u32::MAX as u64)) as u32
    }

    pub fn gen_range(&mut self, range: std::ops::Range<u32>) -> u32 {
        let x = self.gen_u32();
        range.start + (x % (range.end - range.start))
    }
}

impl Default for PseudoRandomGenerator {
    fn default() -> Self {
        PseudoRandomGenerator::from(74755)
    }
}
