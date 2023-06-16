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

import org.eclipse.emf.ecore.resource.Resource
import org.lflang.*
import org.lflang.lf.*

val Port.getDataType: String
    get() = "defData"

val Port.getTokenType: String
    get() = "defToken"
val Port.getConnName: String
    get() = "conn_$name"

val Port.getConnFuncName: String
    get() = "conn_${name}_func"

val Port.getConnType: String
    get() = "SingleValueToken"
val Port.getConnFunc: String
    get() ="(c: ConnectionConfig[${getDataType}.type, ${getTokenType}.type]) => new ${getConnType}(c)"

val Timer.getDataType: String
    get() = "UInt(0.W)"

val Timer.getTokenType: String
    get() = "PureToken"

val Timer.getConnName: String
    get() = "conn_${name}"

val StateVar.getDataType: String
    get() = "defData"

val StateVar.getTokenType: String
    get() = "defToken"

val Reaction.getClassName: String
    get() = "Reaction_${name}"
val Reaction.getInstanceName: String
    get() = "reaction_${indexInContainer}"
val Reaction.getIOClassName: String
    get() = "Reaction_${name}IO"
