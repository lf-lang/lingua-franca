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

#[derive(Default)]
pub struct Matrix<T> {
    data: Vec<T>,
    size_y: usize,
}

#[derive(Default)]
pub struct TransposedMatrix<T> {
    data: Vec<T>,
    size_x: usize,
}

impl<T: Default + Clone + Copy> Matrix<T> {
    pub fn new(size_x: usize, size_y: usize) -> Self {
        Matrix::<T> {
            data: vec![T::default(); size_x * size_y],
            size_y,
        }
    }

    pub fn get(&self, x: usize, y: usize) -> &T {
        &self.data[x * self.size_y + y]
    }

    pub fn set(&mut self, x: usize, y: usize, value: T) {
        self.data[x * self.size_y + y] = value;
    }
}

impl<T: Default + Clone + Copy> TransposedMatrix<T> {
    pub fn new(size_x: usize, size_y: usize) -> Self {
        TransposedMatrix::<T> {
            data: vec![T::default(); size_x * size_y],
            size_x,
        }
    }

    pub fn get(&self, x: usize, y: usize) -> &T {
        &self.data[y * self.size_x + x]
    }

    pub fn set(&mut self, x: usize, y: usize, value: T) {
        self.data[y * self.size_x + x] = value;
    }
}
