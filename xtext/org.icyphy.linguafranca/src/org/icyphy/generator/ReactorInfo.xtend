/* Class encoding properties of a reactor */
// The Lingua-Franca toolkit is is licensed under the BSD 2-Clause License.
// See LICENSE.md file in the top repository directory.
package org.icyphy.generator

import java.util.HashMap
import java.util.HashSet
import java.util.LinkedHashMap
import java.util.LinkedList
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.VarRef
import org.icyphy.linguaFranca.Variable

/** The properties of a reactor class.  
 */
 class ReactorInfo {
 	
 	static val reactorToInfo = new HashMap<Reactor, ReactorInfo>()
	
	static def get(Reactor reactor) {
		val info = reactorToInfo.get(reactor)
		if (info === null) { // No info available yet
			return new ReactorInfo(reactor)
		} else {
			return info
		}
	}
	
	private new(Reactor reactor) {
		this.reactor = reactor
		reactorToInfo.put(reactor, this)
	}
	
	public var Reactor reactor
 	
// 	/** Map from input name to Input object. */
//	public var nameToInput = new LinkedHashMap<String,Input>()
//	
//	/** Map from output name to Output object. */
//	public var nameToOutput = new LinkedHashMap<String,Output>()
//	
//	/** Map from parameter name to Parameter object. */		
//	public var nameToParam = new LinkedHashMap<String,Param>()
//
//	/** Map from action name to Action object. */
//	public var nameToAction = new LinkedHashMap<String,Action>()
//
//	/** Map from name to Instance object. */
//	public var nameToInstance = new LinkedHashMap<String,Instance>()
//
//	/** Map from timer name to Timer object. */
//	public var nameToTimer = new LinkedHashMap<String,Timer>()
//	
//	/** Map from timer name to Timing object. */
//	public var nameToTiming = new LinkedHashMap<String,Timing>()
	
	
	/** Map from output name to list of inputs names triggered
	 *  by this output. The names have form either "instanceName.portName"
	 *  (if the port belongs to a contained reactor) or "portName"
	 *  (if the port belongs to this component).
	 */
	public var outputToInputs = new LinkedHashMap<VarRef,HashSet<VarRef>>() // FIXME: could we use a HashSet<Input> instead?
	
	/** Map from output name for outputs of this reactor to output names
	 *  of contained reactors that send data via this output port.
	 */
	public var outputToContainedOutput = new LinkedHashMap<VarRef, VarRef>();
	
	/** For use by language-specific code generators, a generic map
	 *  for storing properties.
	 */
	public var targetProperties = new LinkedHashMap<Object,String>()
 	
	/** Map from trigger names to list of reactions triggered by it. */
	public var triggerToReactions = new LinkedHashMap<Variable,LinkedList<Reaction>>()
}