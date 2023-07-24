/**
 * @author erling r. jellum (erling.r.jellum@ntnu.no)
 *
 * copyright (c) 2023, the norwegian university of science and technology.
 *
 * redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * this software is provided by the copyright holders and contributors "as is" and any
 * express or implied warranties, including, but not limited to, the implied warranties of
 * merchantability and fitness for a particular purpose are disclaimed. in no event shall
 * the copyright holder or contributors be liable for any direct, indirect, incidental,
 * special, exemplary, or consequential damages (including, but not limited to,
 * procurement of substitute goods or services; loss of use, data, or profits; or business
 * interruption) however caused and on any theory of liability, whether in contract,
 * strict liability, or tort (including negligence or otherwise) arising in any way out of
 * the use of this software, even if advised of the possibility of such damage.
 */
package org.lflang.generator.chisel

import org.lflang.*
import org.lflang.lf.*

val Port.getDataType: String
    get() {
        if (type.id == null) {
            return type.code.body
        } else if (type.id == "UInt") {
            return "UInt()"
        } else {
            return type.id
        }
    }

val Port.getTokenType: String
    get() = "new SingleToken($getDataType)"
val Port.getConnName: String
    get() = "conn_$name"
val Port.getConnFuncName: String
    get() = "conn_${name}_func"

val Input.getInwardConnName: String
    get() = "conn_pt_${name}"

val Input.getTriggeredReactions: List<Reaction>
    get() {
        val triggeredReactions = mutableListOf<Reaction>()
        val parent = this.eContainer() as Reactor
        for (r in parent.reactions) {
            for (dep in r.triggers + r.sources) {
                if ((dep is VarRef) && dep.variable == this) {
                    triggeredReactions += r
                }
            }
        }
        return triggeredReactions
    }

val Output.getWritingReactions: List<Reaction>
    get() {
        val writingReactions = mutableListOf<Reaction>()
        val parent = this.eContainer() as Reactor
        for (r in parent.reactions) {
            for (dep in r.effects) {
                if (dep.variable == this)
                    writingReactions += r
            }
        }
        return writingReactions
    }


// FIXME: Get right conn types
val Port.getConnType: String
    get() = "SingleToken"
val Port.getConnFunc: String
    get() ="(c: ConnectionConfig[$getDataType.type, $getTokenType.type]) => new ${getConnType}(c)"

val Timer.getDataType: String
    get() = "UInt(0.W)"

val Timer.getTokenType: String
    get() = "new PureToken()"


// FIXME: Actually get the correct data type
val StateVar.getDataType: String
    get() {
        if (type.id == null) {
            return type.code.body
        } else if (type.id == "UInt") {
            return "UInt()"
        } else {
            return type.id
        }
    }

val StateVar.getStateDecl: String
    get() {
        val nReactions = (this.eContainer() as Reactor).reactions.size
        return "new SingleValueState(StateConfig(${getDataType}, ${getTokenType}, ${nReactions}, ${getStateProtocol}))"
    }

val StateVar.getStateProtocol: String
    get() = "Immediate"

// FIXME: Actually get the right token type
val StateVar.getTokenType: String
    get() = "new SingleToken(${getDataType})"



val Reaction.getClassName: String
    get() = "Reaction_${indexInContainer}"
val Reaction.getInstanceName: String
    get() = "reaction_${indexInContainer}"
val Reaction.getIOClassName: String
    get() = "Reaction_${name}IO"

val Connection.getConnectionFactory: String
    get() = "new SingleValueConnectionFactory(${getDataType})"

// Consider this multi connections. Here we assu
val Connection.getDataType: String
    get() {
        require(this.leftPorts.size == 1 && this.rightPorts.size == 1)
        return (this.leftPorts.get(0).variable as Port).getDataType
    }

val Connection.getName: String
    get() {
        val lhsPort = this.leftPorts.get(0).variable as Port
        val lhsParentInst = this.leftPorts.get(0).container as Instantiation
        return "_conn_${lhsParentInst.name}_${lhsPort.name}"
    }

val VarRef.getConnectionName: String
    get() {
        val port = this.variable as Port
        val parentInst = this.container  as Instantiation
        return "_conn_${parentInst.name}_${port.name}"
    }

val Port.getConnectionFactory: String
    get() = "new SingleValueConnectionFactory(${getDataType})"

val Port.getInwardConnectionFactory: String
    get() = "new SingleValueInputPortInwardConnectionFactory(${getDataType})"
