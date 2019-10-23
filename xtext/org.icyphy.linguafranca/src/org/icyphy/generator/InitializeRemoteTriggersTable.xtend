/** A data structure used by the C target compiler for Lingua Franca. */
// The Lingua-Franca toolkit is is licensed under the BSD 2-Clause License.
// See LICENSE.md file in the top repository directory.
package org.icyphy.generator

import org.icyphy.linguaFranca.Instance
import org.icyphy.linguaFranca.VarRef

/** A data class used by CGenerator. */
class InitializeRemoteTriggersTable {
	new (
		Instance reactor,
		String remoteTriggersArrayName,
		int arrayIndex,
		VarRef input
	) {
		this.reactor = reactor
		this.remoteTriggersArrayName = remoteTriggersArrayName
		this.arrayIndex = arrayIndex
		this.input = input
	}
	public Instance reactor
	public String remoteTriggersArrayName
	public int arrayIndex
	public VarRef input
}