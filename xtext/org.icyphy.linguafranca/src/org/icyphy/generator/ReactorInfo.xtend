/* Class encoding properties of a reactor */
// The Lingua-Franca toolkit is is licensed under the BSD 2-Clause License.
// See LICENSE.md file in the top repository directory.
package org.icyphy.generator

import java.util.HashMap
import java.util.LinkedHashMap
import java.util.LinkedList
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.VarRef

/** The properties of a reactor class.  
 */
 class ReactorInfo {
 	// FIXME: We can probably get rid of this class.
 	// See ReactorInstance	
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
	
	/** Map from output name for outputs of this reactor to output names
	 *  of contained reactors that send data via this output port.
	 */
	public var outputToContainedOutput = new LinkedHashMap<Output, VarRef>();
	
	/** For use by language-specific code generators, a generic map
	 *  for storing properties.
	 */
	public var targetProperties = new LinkedHashMap<Object, String>()
 	
	/** Map from trigger names to list of reactions triggered by it. */
	public var triggerToReactions = new LinkedHashMap<VarRef, LinkedList<Reaction>>()

	
}