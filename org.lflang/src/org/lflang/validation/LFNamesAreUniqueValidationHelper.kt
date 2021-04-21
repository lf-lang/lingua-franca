/*
 * Copyright (c) 2021, The University of California at Berkeley.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
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
 */

package org.lflang.validation

import org.eclipse.emf.ecore.EClass
import org.eclipse.xtext.validation.NamesAreUniqueValidationHelper
import org.lflang.lf.LfPackage

class LFNamesAreUniqueValidationHelper : NamesAreUniqueValidationHelper() {

    /**
     * Lump all inputs, outputs, timers, actions, parameters, and
     * instantiations in the same cluster type. This ensures that
     * names amongst them are checked for uniqueness.
     */
    override fun getAssociatedClusterType(eClass: EClass): EClass? {
        return when (eClass) {
            LfPackage.Literals.INPUT,
            LfPackage.Literals.OUTPUT,
            LfPackage.Literals.TIMER,
            LfPackage.Literals.ACTION,
            LfPackage.Literals.PARAMETER,
            LfPackage.Literals.INSTANTIATION,
                 -> LfPackage.Literals.VARIABLE
            else -> super.getAssociatedClusterType(eClass)
        }
    }
}
