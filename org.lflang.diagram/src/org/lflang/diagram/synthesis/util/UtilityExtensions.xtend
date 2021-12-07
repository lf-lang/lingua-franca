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
package org.lflang.diagram.synthesis.util

import de.cau.cs.kieler.klighd.internal.util.KlighdInternalProperties
import de.cau.cs.kieler.klighd.kgraph.KGraphElement
import de.cau.cs.kieler.klighd.kgraph.KGraphFactory
import de.cau.cs.kieler.klighd.kgraph.KNode
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared
import org.eclipse.elk.core.math.ElkMargin
import org.eclipse.elk.core.options.CoreOptions
import org.eclipse.elk.core.util.IndividualSpacings
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.lflang.ASTUtils
import org.lflang.diagram.synthesis.AbstractSynthesisExtensions
import org.lflang.generator.ReactorInstance
import org.lflang.lf.Code
import org.lflang.lf.Host
import org.lflang.lf.Reactor
import org.lflang.lf.Value

/**
 * Extension class that provides various utility methods for the synthesis.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
@ViewSynthesisShared
class UtilityExtensions extends AbstractSynthesisExtensions {
	
	extension KGraphFactory = KGraphFactory.eINSTANCE

	/**
	 * Converts a host value into readable text
	 */
	def String toText(Host host) {
		val sb = new StringBuilder
		if (host !== null) {
			if (!host.user.nullOrEmpty) {
				sb.append(host.user).append("@")
			}
			if (!host.addr.nullOrEmpty) {
				sb.append(host.addr)
			}
			if (host.port !== 0) {
				sb.append(":").append(host.port)
			}
		}
		return sb.toString
	}
	
	/**
	 * Returns true if the reactor is the primary reactor
	 */
	def isMainOrFederated(Reactor reactor) {
		return reactor.main || reactor.federated
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
	def hasContent(ReactorInstance reactor) {
		return !reactor.reactions.empty || !reactor.instantiations.empty
	}
	
    /**
     * 
     */
    def isRoot(ReactorInstance ri) {
        return ri.parent === null
    }
	
	/**
	 * Trims the hostcode of reactions.
	 */
	def trimCode(Code tokenizedCode) {
		if (tokenizedCode === null || tokenizedCode.body.nullOrEmpty) {
			return ""
		}
		try {
			val code = NodeModelUtils.findActualNodeFor(tokenizedCode)?.text
			var contentStart = 0
			val lines = newArrayList()
			lines += code.split("\n").dropWhile[!it.contains("{=")]
			
			// Remove start pattern
			if (!lines.empty) {
				if (lines.head.trim().equals("{=")) {
					lines.remove(0) // skip
				} else {
					lines.set(0, lines.head.replace("{=", "").trim())
					contentStart = 1
				}
			}
			
			// Remove end pattern
			if (!lines.empty) {
				if (lines.last.trim.equals("=}")) {
					lines.remove(lines.size - 1) // skip
				} else {
					lines.set(lines.size - 1, lines.last.replace("=}", ""))
				}
			}
			
			// Find indentation
			var String indentation = null
			while (indentation === null && lines.size > contentStart) {
				val firstLine = lines.get(contentStart)
				val trimmed = firstLine.trim()
				if (trimmed.empty) {
					lines.set(contentStart, "")
					contentStart++
				} else {
					val firstCharIdx = firstLine.indexOf(trimmed.charAt(0))
					indentation = firstLine.substring(0, firstCharIdx)
				}
			}
			
			// Remove root indentation
			if (!lines.empty) {
				for (i : 0..lines.size-1) {
					if (lines.get(i).startsWith(indentation)) {
						lines.set(i, lines.get(i).substring(indentation.length))
					}
				}
			}
			
			return lines.join("\n")
		} catch(Exception e) {
			e.printStackTrace
			return tokenizedCode.body
		}
	}
	
	/**
	 * Sets KGE ID.
	 */
	def setID(KGraphElement kge, String id) {
		kge.data.add(createKIdentifier => [it.setId(id)])
	}
	
	/**
	 * Retrieves the source element of the given diagram element
	 */
	def Object sourceElement(KGraphElement elem) {
		return elem.getProperty(KlighdInternalProperties.MODEL_ELEMEMT)
	}
	
    /**
     * Checks if the source element of the given diagram element is a reactor
     */
	def boolean sourceIsReactor(KNode node) {
		return node.sourceElement() instanceof Reactor
	}

    /**
     * Returns the port placement margins for the node.
     * If this spacing does not yet exist, the properties are initialized.
     */
    def getPortMarginsInitIfAbsent(KNode node) {
        var spacing = node.getProperty(CoreOptions.SPACING_INDIVIDUAL)
        if (spacing === null) {
            spacing = new IndividualSpacings()
            node.setProperty(CoreOptions.SPACING_INDIVIDUAL, spacing)
        }
        var margin = spacing.getProperty(CoreOptions.SPACING_PORTS_SURROUNDING)
        if (margin === null) {
            margin = new ElkMargin()
            node.setProperty(CoreOptions.SPACING_PORTS_SURROUNDING, margin)
        }
        return margin
    }	

}
