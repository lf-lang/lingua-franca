/** A data structure used by the C target compiler for Lingua Franca. */
// The Lingua-Franca toolkit is is licensed under the BSD 2-Clause License.
// See LICENSE.md file in the top repository directory.
package org.icyphy.generator

/** A data class used by CGenerator. */
class InitiatlizeRemoteTriggersTable {
	new (
		ReactorInstance reactor,
		String remoteTriggersArrayName,
		int arrayIndex,
		String inputName
	) {
		this.reactor = reactor
		this.remoteTriggersArrayName = remoteTriggersArrayName
		this.arrayIndex = arrayIndex
		this.inputName = inputName
	}
	public ReactorInstance reactor
	public String remoteTriggersArrayName
	public int arrayIndex
	public String inputName
}