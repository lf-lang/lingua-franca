/** A data structure used by the C target compiler for Lingua Franca. */

/*************
Copyright (c) 2020, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

package org.icyphy.generator

/** A data class used by CGenerator.
 * 
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 */
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