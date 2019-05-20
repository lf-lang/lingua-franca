/* Class encoding properties of a reactor */
package org.icyphy.generator

import java.util.LinkedHashMap
import java.util.LinkedList
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Param
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Timer
import org.icyphy.linguaFranca.Timing

/** The properties of a reactor class.  
 */
 class ComponentProperties {
 	
 	/** Map from input name to Input object. */
	public var nameToInput = new LinkedHashMap<String,Input>()
	
	/** Map from output name to Output object. */
	public var nameToOutput = new LinkedHashMap<String,Output>()
	
	/** Map from parameter name to Parameter object. */		
	public var nameToParam = new LinkedHashMap<String,Param>()

	/** Map from action name to Action object. */
	public var nameToAction = new LinkedHashMap<String,Action>()

	/** Map from timer name to Timer object. */
	public var nameToTimer = new LinkedHashMap<String,Timer>()
	
	/** Map from timer name to Timing object. */
	public var nameToTiming = new LinkedHashMap<String,Timing>()
	
	/** For use by language-specific code generators, a generic map
	 *  for storing properties.
	 */
	public var targetProperties = new LinkedHashMap<Object,String>()
 	
	/** Map from trigger names to list of reactions triggered by it. */
	public var triggerNameToReactions = new LinkedHashMap<String,LinkedList<Reaction>>()
}