package org.lflang.diagram.synthesis.styles;

import de.cau.cs.kieler.klighd.krendering.KContainerRendering;
import de.cau.cs.kieler.klighd.krendering.KRendering;
import java.util.List;
import org.eclipse.xtext.xbase.lib.Pure;
import org.eclipse.xtext.xbase.lib.util.ToStringBuilder;

/**
 * Components of a reactor figure.
 *
 * @ingroup Diagram
 */
public class ReactorFigureComponents {
  private final KContainerRendering outer;

  private final KContainerRendering reactor;

  private final List<KRendering> figures;

  public ReactorFigureComponents(
      KContainerRendering outer, KContainerRendering reactor, List<KRendering> figures) {
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
    result = prime * result + ((this.outer == null) ? 0 : this.outer.hashCode());
    result = prime * result + ((this.reactor == null) ? 0 : this.reactor.hashCode());
    return prime * result + ((this.figures == null) ? 0 : this.figures.hashCode());
  }

  @Override
  @Pure
  public boolean equals(final Object obj) {
    if (this == obj) return true;
    if (obj == null) return false;
    if (getClass() != obj.getClass()) return false;
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
