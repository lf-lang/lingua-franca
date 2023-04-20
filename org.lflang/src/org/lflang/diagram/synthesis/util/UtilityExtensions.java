/*************
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
***************/
package org.lflang.diagram.synthesis.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.eclipse.elk.core.math.ElkMargin;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.util.IndividualSpacings;
import org.eclipse.xtext.nodemodel.ICompositeNode;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
import org.eclipse.xtext.xbase.lib.Extension;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.eclipse.xtext.xbase.lib.StringExtensions;
import org.lflang.diagram.synthesis.AbstractSynthesisExtensions;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.Code;
import org.lflang.lf.Expression;
import org.lflang.lf.Host;
import org.lflang.lf.Literal;
import org.lflang.lf.ParameterReference;
import org.lflang.lf.Reactor;
import org.lflang.lf.Time;
import org.lflang.util.StringUtil;

import de.cau.cs.kieler.klighd.internal.util.KlighdInternalProperties;
import de.cau.cs.kieler.klighd.kgraph.KGraphElement;
import de.cau.cs.kieler.klighd.kgraph.KGraphFactory;
import de.cau.cs.kieler.klighd.kgraph.KIdentifier;
import de.cau.cs.kieler.klighd.kgraph.KNode;
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared;

/**
 * Extension class that provides various utility methods for the synthesis.
 * 
 * @author Alexander Schulz-Rosengarten
 */
@ViewSynthesisShared
public class UtilityExtensions extends AbstractSynthesisExtensions {
    
    @Extension
    private KGraphFactory _kGraphFactory = KGraphFactory.eINSTANCE;

    
    /**
     * Returns true if the reactor is the primary reactor
     */
    public boolean isMainOrFederated(Reactor reactor) {
        return reactor.isMain() || reactor.isFederated();
    }
    
    /**
     * Returns true if the instance is a bank of reactors
     */
//    def boolean isBank(Instantiation ins) {
//        return ins?.widthSpec !== null ? ins.widthSpec.width !== 1 : false
//    }
    
    /**
     * Returns true if the reactor as has inner reactions or instances
     */
    public boolean hasContent(final ReactorInstance reactor) {
        return !reactor.reactions.isEmpty() || !reactor.instantiations().isEmpty();
    }
    
    /**
     * 
     */
    public boolean isRoot(final ReactorInstance ri) {
        return ri.getParent() == null;
    }
    
    /**
     * Trims the hostcode of reactions.
     */
    public String trimCode(final Code tokenizedCode) {
        if (tokenizedCode == null || StringExtensions.isNullOrEmpty(tokenizedCode.getBody())) {
            return "";
        }
        try {
            ICompositeNode node = NodeModelUtils.findActualNodeFor(tokenizedCode);
            String code = node != null ? node.getText() : null;
            int contentStart = 0;
            List<String> lines = new ArrayList<>();
            Arrays.stream(code.split("\n")).dropWhile(line -> !line.contains("{=")).forEachOrdered(lines::add);
            
            // Remove start pattern
            if (!lines.isEmpty()) {
                if (IterableExtensions.head(lines).trim().equals("{=")) {
                    lines.remove(0); // skip
                } else {
                    lines.set(0, IterableExtensions.head(lines).replace("{=", "").trim());
                    contentStart = 1;
                }
            }
            
            // Remove end pattern
            if (!lines.isEmpty()) {
                if (IterableExtensions.last(lines).trim().equals("=}")) {
                    lines.remove(lines.size() - 1); // skip
                } else {
                    lines.set(lines.size() - 1, IterableExtensions.last(lines).replace("=}", ""));
                }
            }
            
            // Find indentation
            String indentation = null;
            while (indentation == null && lines.size() > contentStart) {
                String firstLine = lines.get(contentStart);
                String trimmed = firstLine.trim();
                if (trimmed.isEmpty()) {
                    lines.set(contentStart, "");
                    contentStart++;
                } else {
                    int firstCharIdx = firstLine.indexOf(trimmed.charAt(0));
                    indentation = firstLine.substring(0, firstCharIdx);
                }
            }
            
            // Remove root indentation
            if (!lines.isEmpty()) {
                for (int i = 0; i < lines.size(); i++) {
                    if (lines.get(i).startsWith(indentation)) {
                        lines.set(i, lines.get(i).substring(indentation.length()));
                    }
                }
            }
            
            return String.join("\n", lines);
        } catch(Exception e) {
            e.printStackTrace();
            return tokenizedCode.getBody();
        }
    }
    
    /**
     * Sets KGE ID.
     */
    public boolean setID(KGraphElement kge, String id) {
        KIdentifier identifier  = _kGraphFactory.createKIdentifier();
        identifier.setId(id);
        return kge.getData().add(identifier);
    }
    
    /**
     * Retrieves the source element of the given diagram element
     */
    public Object sourceElement(KGraphElement elem) {
        return elem.getProperty(KlighdInternalProperties.MODEL_ELEMEMT);
    }
    
    /**
     * Checks if the source element of the given diagram element is a reactor
     */
    public boolean sourceIsReactor(KNode node) {
        return sourceElement(node) instanceof Reactor;
    }

    /**
     * Returns the port placement margins for the node.
     * If this spacing does not yet exist, the properties are initialized.
     */
    public ElkMargin getPortMarginsInitIfAbsent(KNode node) {
        IndividualSpacings spacing = node.getProperty(CoreOptions.SPACING_INDIVIDUAL);
        if (spacing == null) {
            spacing = new IndividualSpacings();
            node.setProperty(CoreOptions.SPACING_INDIVIDUAL, spacing);
        }
        ElkMargin margin = spacing.getProperty(CoreOptions.SPACING_PORTS_SURROUNDING);
        if (margin == null) {
            margin = new ElkMargin();
            node.setProperty(CoreOptions.SPACING_PORTS_SURROUNDING, margin);
        }
        return margin;
    }    

}
