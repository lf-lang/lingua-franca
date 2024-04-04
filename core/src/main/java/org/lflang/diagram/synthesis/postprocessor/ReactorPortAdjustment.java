/*************
 * Copyright (c) 2023, Kiel University.
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
 ***************/
package org.lflang.diagram.synthesis.postprocessor;

import com.google.inject.Binder;
import com.google.inject.Guice;
import com.google.inject.Scopes;
import com.google.inject.TypeLiteral;
import com.google.inject.Inject;
import de.cau.cs.kieler.klighd.IStyleModifier;
import de.cau.cs.kieler.klighd.IViewer;
import de.cau.cs.kieler.klighd.internal.ILayoutRecorder;
import de.cau.cs.kieler.klighd.kgraph.KGraphElement;
import de.cau.cs.kieler.klighd.kgraph.KGraphFactory;
import de.cau.cs.kieler.klighd.kgraph.KNode;
import de.cau.cs.kieler.klighd.krendering.KRendering;
import de.cau.cs.kieler.klighd.krendering.KRenderingFactory;
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared;
import de.cau.cs.kieler.klighd.syntheses.AbstractDiagramSynthesis;
import java.util.List;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.graph.properties.Property;
import org.eclipse.xtext.xbase.lib.Extension;
import org.lflang.diagram.synthesis.LinguaFrancaSynthesis;
import org.lflang.diagram.synthesis.styles.LinguaFrancaShapeExtensions;

/**
 * Adjusts the port figures of reactors when fixed side are off to keep the input output indication
 * correct.
 *
 * @author Alexander Schulz-Rosengarten
 */
public class ReactorPortAdjustment implements IStyleModifier {

  public static final String ID =
      "org.lflang.diagram.synthesis.postprocessor.ReactorPortAdjustment";

  /** INTERNAL property to mark node as flipped. */
  public static final Property<Boolean> FLIPPED =
      new Property<>("org.lflang.diagram.synthesis.postprocessor.reactor.ports.flipped", false);

  @Inject @Extension private LinguaFrancaShapeExtensions _linguaFrancaShapeExtensions;
  @Extension private KGraphFactory _kGraphFactory = KGraphFactory.eINSTANCE;
  private static KRenderingFactory _kRenderingFactory = KRenderingFactory.eINSTANCE;

  /** Register this modifier on a reaction rendering. */
  public static void apply(KNode node, List<KRendering> renderings) {
    var rendering = renderings.get(0); // Get first in bank
    // Add modifier that fixes port positions such that edges are properly attached to the shape
    var invisible = _kRenderingFactory.createKRotation();
    // make it ineffective (just for purpose of holding modifier)
    invisible.setRotation(0);
    // Add modifier to receive callback after layout
    invisible.setModifierId(ID);
    rendering.getStyles().add(invisible);
  }

  public ReactorPortAdjustment() {
    // Inject extension
    if (_linguaFrancaShapeExtensions == null) {
      var injector =
          Guice.createInjector(
              new com.google.inject.Module() {
                // This Module is created to satisfy ViewSynthesisShared scope of used
                // synthesis-extensions
                public void configure(Binder binder) {
                  binder.bindScope(ViewSynthesisShared.class, Scopes.SINGLETON);
                  binder
                      .bind(new TypeLiteral<AbstractDiagramSynthesis<?>>() {})
                      .toInstance(new LinguaFrancaSynthesis());
                }
              });
      _linguaFrancaShapeExtensions = injector.getInstance(LinguaFrancaShapeExtensions.class);
    }
  }

  @Override
  public boolean modify(IStyleModifier.StyleModificationContext context) {
    try {
      KGraphElement node = context.getGraphElement();
      if (node instanceof KNode) {
        KNode knode = (KNode) node;

        // Find root node
        KNode parent = knode;
        while (parent.eContainer() != null) {
          parent = (KNode) parent.eContainer();
        }

        // Get viewer (this is a bit brittle because it fetches the viewer from some internal
        // property)
        Object viewer =
            parent.getAllProperties().entrySet().stream()
                .filter(
                    entry ->
                        entry.getKey().getId().equals("de.cau.cs.kieler.klighd.viewer")
                            || entry.getKey().getId().equals("klighd.layout.viewer"))
                .findAny()
                .map(entry -> entry.getValue())
                .orElse(null);

        ILayoutRecorder recorder = null;
        if (viewer instanceof IViewer) {
          recorder = ((IViewer) viewer).getViewContext().getLayoutRecorder();
        }

        if (recorder != null) {
          recorder.startRecording();
        }
        for (var port : knode.getPorts()) {
          var isInput = port.getProperty(LinguaFrancaSynthesis.REACTOR_INPUT).booleanValue();
          if (!isInput && !port.getProperty(LinguaFrancaSynthesis.REACTOR_OUTPUT)) {
            continue; // skip
          }

          var xPos = port.getXpos();
          var isLeft = xPos < 0;
          var flip = isInput != isLeft;
          var isFlipped = port.getProperty(FLIPPED).booleanValue();
          var needsUpdate = flip != isFlipped;

          if (needsUpdate) {
            // Remove figure
            port.getData().removeIf(it -> it instanceof KRendering);

            // Get port type
            var isMultiport = port.getProperty(LinguaFrancaSynthesis.REACTOR_MULTIPORT);
            var isBank = port.getProperty(LinguaFrancaSynthesis.REACTOR_HAS_BANK_PORT_OFFSET);

            // Add new figure
            _linguaFrancaShapeExtensions.addTrianglePort(port, isMultiport, flip);
            port.setProperty(FLIPPED, flip);

            // Compute new offset
            var oldOffset = port.getProperty(CoreOptions.PORT_BORDER_OFFSET);
            var newOffset =
                LinguaFrancaSynthesis.getReactorPortOffset(!isLeft, isMultiport, isBank);

            // Apply offset directly
            port.setPos((float) (port.getXpos() + (oldOffset - newOffset)), port.getYpos());
          }
        }
        if (recorder != null) {
          recorder.stopRecording(0);
        }
      }
    } catch (Exception e) {
      e.printStackTrace();
      // do not disturb rendering process
    }
    return false;
  }
}
