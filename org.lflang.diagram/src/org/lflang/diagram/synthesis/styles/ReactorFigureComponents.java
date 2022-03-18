/**
 * Copyright (c) 2020, Kiel University.
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.lflang.diagram.synthesis.styles;

import de.cau.cs.kieler.klighd.krendering.KContainerRendering;
import de.cau.cs.kieler.klighd.krendering.KRendering;
import java.util.List;
import org.eclipse.xtext.xbase.lib.Pure;
import org.eclipse.xtext.xbase.lib.util.ToStringBuilder;

public class ReactorFigureComponents {
    private final KContainerRendering outer;

    private final KContainerRendering reactor;

    private final List<KRendering> figures;

    public ReactorFigureComponents(KContainerRendering outer, 
                                   KContainerRendering reactor,
                                   List<KRendering> figures) {
        super();
        this.outer = outer;
        this.reactor = reactor;
        this.figures = figures;
    }

    @Override
    @Pure
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + ((this.outer== null) ? 0 : this.outer.hashCode());
        result = prime * result + ((this.reactor== null) ? 0 : this.reactor.hashCode());
        return prime * result + ((this.figures== null) ? 0 : this.figures.hashCode());
    }

    @Override
    @Pure
    public boolean equals(final Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        ReactorFigureComponents other = (ReactorFigureComponents) obj;
        if (this.outer == null && other.outer != null) {
            return false;
        } else if (!this.outer.equals(other.outer)) {
            return false;
        }
        if (this.reactor == null && other.reactor != null) {
            return false;
        } else if (!this.reactor.equals(other.reactor)) {
            return false;
        }
        if (this.figures == null && other.figures != null) {
            return false;
        } else if (!this.figures.equals(other.figures)) {
            return false;
        }
        return true;
    }

    @Override
    @Pure
    public String toString() {
        ToStringBuilder b = new ToStringBuilder(this);
        b.add("outer", this.outer);
        b.add("reactor", this.reactor);
        b.add("figures", this.figures);
        return b.toString();
    }

    @Pure
    public KContainerRendering getOuter() {
        return this.outer;
    }

    @Pure
    public KContainerRendering getReactor() {
        return this.reactor;
    }

    @Pure
    public List<KRendering> getFigures() {
        return this.figures;
    }
}
