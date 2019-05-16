/* Class encoding properties of a reactor */
package org.icyphy.generator

import java.util.LinkedHashMap
import java.util.LinkedList
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Param
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Timer
import org.icyphy.linguaFranca.Timing

/** The properties of a reactor.  
 */
 class ComponentProperties {
 	/** The name of the struct type used to store
 	 *  parameter and state values.
 	 *  FIXME: THIS IS SPECIFIC TO THE C GENERATOR. DOES NOT BELONG HERE.
 	 */
 	public String structType
 	
 	// Map from input name to Input object.
	public var nameToInput = new LinkedHashMap<String,Input>()
	
	// Map from parameter name to Parameter object.		
	public var nameToParam = new LinkedHashMap<String,Param>()

	// Map from action name to Action object.
	public var nameToAction = new LinkedHashMap<String,Action>()

	// Map from timer name to Timer object.
	public var nameToTimer = new LinkedHashMap<String,Timer>()
	
	// Map from timer name to Timing object.
	public var nameToTiming = new LinkedHashMap<String,Timing>()
 	
	// Map from trigger names to list of reactions triggered by it.
	public var triggerNameToReactions = new LinkedHashMap<String,LinkedList<Reaction>>()
	
	// Map from reaction to a reaction function name.
	// FIXME: THIS IS SPECIFIC TO THE C GENERATOR. DOES NOT BELONG HERE.
	public var reactionToFunctionName = new LinkedHashMap<Reaction,String>()
 }