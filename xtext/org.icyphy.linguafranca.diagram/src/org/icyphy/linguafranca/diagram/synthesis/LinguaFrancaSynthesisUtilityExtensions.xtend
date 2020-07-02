package org.icyphy.linguafranca.diagram.synthesis

import de.cau.cs.kieler.klighd.internal.util.KlighdInternalProperties
import de.cau.cs.kieler.klighd.kgraph.KGraphElement
import de.cau.cs.kieler.klighd.kgraph.KGraphFactory
import de.cau.cs.kieler.klighd.kgraph.KNode
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared
import org.eclipse.emf.ecore.EObject
import org.eclipse.xtext.TerminalRule
import org.eclipse.xtext.nodemodel.impl.CompositeNode
import org.eclipse.xtext.nodemodel.impl.HiddenLeafNode
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.eclipse.xtext.resource.XtextResource
import org.icyphy.ASTUtils
import org.icyphy.linguaFranca.Code
import org.icyphy.linguaFranca.Host
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.Value

/**
 * Extension class that provides various utility methods for the synthesis.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
@ViewSynthesisShared
class LinguaFrancaSynthesisUtilityExtensions extends AbstractSynthesisExtensions {
	
	extension KGraphFactory = KGraphFactory.eINSTANCE
	
	/**
	 * Converts a timing value into readable text
	 */
	def String toText(Value value) {
		if (value !== null) {
			if (value.parameter !== null) {
                return value.parameter.name
            } else if (value.time !== null) {
                return value.time.interval +
                        value.time.unit.toString
            } else if (value.literal !== null) {
                return value.literal
            } else if (value.code !== null) {
                ASTUtils.toText(value.code)
            }
		}
		return ""
	}
	
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
	def isPrimary(Reactor reactor) {
		return reactor.main || reactor.federated
	}
	
	/**
	 * Returns true if the reactor as has inner reactions or instances
	 */
	def hasContent(Reactor reactor) {
		return !reactor.reactions.empty || !reactor.instantiations.empty
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
	 * Retrieves comments associated with model element form the AST.
	 */
	def String findComments(EObject object) {
		if (object.eResource instanceof XtextResource) {
			val compNode = NodeModelUtils.findActualNodeFor(object)
			if (compNode !== null) {
				val comments = newArrayList
				var node = compNode.firstChild
				while (node instanceof CompositeNode) {
					node = node.firstChild
				}
				while (node instanceof HiddenLeafNode) { // only comments preceding start of element
					val rule = node.grammarElement
					if (rule instanceof TerminalRule) {
						if ("SL_COMMENT".equals(rule.name)) {
							comments += node.text.substring(2).trim()
						} else if ("ML_COMMENT".equals(rule.name)) {
							var block = node.text
							block = block.substring(block.startsWith("/**") ? 3 : 2, block.length - 2).trim()
							val lines = block.split("\n").map[trim()].toList
							comments += lines.map[
								if (it.startsWith("* ")) {
									it.substring(2)
								} else if (it.startsWith("*")) {
									it.substring(1)
								} else {
									it
								}
							].join("\n")
						}
					}
					node = node.nextSibling
				}
				if (!comments.empty) {
					return comments.join("\n")
				}
			}
		}
		return null
	}
	
	/**
	 * Retrieves the annotations value from JavaDoc comments associated with the given model element in the AST.
	 */
	def String findAnnotationInComments(EObject object, String key) {
		if (object.eResource instanceof XtextResource) {
			val compNode = NodeModelUtils.findActualNodeFor(object)
			if (compNode !== null) {
				var node = compNode.firstChild
				while (node instanceof CompositeNode) {
					node = node.firstChild
				}
				while (node instanceof HiddenLeafNode) { // Only comments preceding start of element
					val rule = node.grammarElement
					if (rule instanceof TerminalRule) {
						if ("ML_COMMENT".equals(rule.name)) {
							if (node.text.trim().startsWith("/**")) { // Only JavaDoc
								var line = node.text.split("\n").filterNull.findFirst[contains(key)]
								if (line !== null) {
									var value = line.substring(line.indexOf(key) + key.length).trim()
									if (value.contains("*")) { // in case of single line JavaDoc (e.g. /** @anno 1503 */)
										value = value.substring(0, value.indexOf("*")).trim()
									}
									return value
								}
							}
						}
					}
					node = node.nextSibling
				}
			}
		}
		return null
	}
	
	def Object sourceElement(KGraphElement elem) {
		return elem.getProperty(KlighdInternalProperties.MODEL_ELEMEMT)
	}
	
	def boolean sourceIsReactor(KNode node) {
		return node.sourceElement() instanceof Reactor
	}

}
