/** A data structure used by the C target compiler for Lingua Franca. */
// The Lingua-Franca toolkit is is licensed under the BSD 2-Clause License.
// See LICENSE.md file in the top repository directory.
package org.icyphy.generator

/** A data class used by CGenerator. */
class InitializeRemoteTriggersTable {
	new (
		ReactorInstance reactorInstance,
		String remoteTriggersArrayName,
		int arrayIndex,
		PortInstance input,
		ReactionInstance reaction
	) {
		this.reactor = reactorInstance
		this.remoteTriggersArrayName = remoteTriggersArrayName
		this.arrayIndex = arrayIndex
		this.input = input
		this.reaction = reaction
	}
	public ReactorInstance reactor
	public String remoteTriggersArrayName
	public int arrayIndex
	public PortInstance input
	// For the case where a reaction is triggered by the output
	// of a contained reactor, there is one more field:
	public ReactionInstance reaction
}