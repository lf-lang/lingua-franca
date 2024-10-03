/*************
 * Copyright (c) 2019-2021, TU Dresden.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.generator.uc

import org.lflang.*
import org.lflang.generator.PrependOperator
import org.lflang.generator.uc.UcTimerGenerator.Companion.codeType
import org.lflang.lf.*


// This class is an abstraction over LF connections. Basically we take
// the LF connections and split them up into UcConnections, each of which has
// a single upstream port and possibly several downstream ports. We can then
// generate Connection objects as expected by reactor-uc
class UcConnection(val src: VarRef, val conn: Connection) {
    private val dests = mutableListOf<VarRef>();

    val codeType = "Conn_${if (src.container != null) src.container.name else ""}_${src.name}"
    val codeName= "conn_${if (src.container != null) src.container.name else ""}_${src.name}"
    val bufSize = 1 // FIXME: Set this somehow probably an annotation?

    fun addDst(port: VarRef) {
        dests.add(port)
    }

    fun getDests() : List<VarRef> {
        return dests
    }

    companion object {
        fun findConnectionFromPort(conns: List<UcConnection>, port: VarRef, connInfo: Connection): UcConnection? {
            return conns.find {c -> c.src == port && c.conn.isPhysical== connInfo.isPhysical && c.conn.delay == connInfo.delay}
        }
    }
}

class UcConnectionGenerator(private val reactor: Reactor) {

    fun getUcConnections(): List<UcConnection> {
        val res = mutableListOf<UcConnection>()
        for (conn: Connection in reactor.connections) {
            for ((index, rhs) in conn.rightPorts.withIndex()) {
                var lhs: VarRef;
                if (conn.isIterated) {
                    lhs = conn.leftPorts.get(0);
                    assert(conn.leftPorts.size == 1)
                } else {
                    assert(conn.leftPorts.size == conn.rightPorts.size)
                    lhs = conn.leftPorts.get(index)
                }

                var ucConn = UcConnection.findConnectionFromPort(res, lhs, conn)
                if (ucConn == null) {
                    ucConn = UcConnection(lhs, conn)
                    res.add(ucConn)
                }
                ucConn.addDst(rhs)

            }
        }
        return res
    }

    fun getPort(p: VarRef): Port {
        var r: Reactor

        if (p.container != null) {
            r = p.container.reactor
        } else {
            r = reactor
        }
        val resPort = r.inputs.plus(r.outputs).filter{ it.name == p.name}
        assert(resPort.size == 1)
        return resPort.get(0)
    }

    fun getPortCodeName(p: VarRef): String {
        var res = ""
        if (p.container != null) {
            res = "${p.container.name}.";
        }
        res += p.name
        return res;
    }

        fun generateLogicalSelfStruct(conn: UcConnection) = with(PrependOperator) {
            """
            |typedef struct {
            |   LogicalConnection super;
            |   Port *_downstreams[${conn.getDests().size}];
            |} ${conn.codeType};
        """.trimMargin()
        }

    fun generateDelayedSelfStruct(conn: UcConnection) = with(PrependOperator) {
        """
            |typedef struct {
            |   DelayedConnection super;
            |   ${getPort(conn.src).type.toText()} buffer[${conn.bufSize}];
            |   Port *_downstreams[${conn.getDests().size}];
            |} ${conn.codeType};
        """.trimMargin()
    }


        fun generateSelfStructs() = getUcConnections().joinToString(prefix = "// Connection structs\n", separator = "\n", postfix = "\n") {
            if (it.conn.isPhysical) {
                unreachable()
            } else if (it.conn.delay != null) {
                generateDelayedSelfStruct(it)
            } else {
                generateLogicalSelfStruct(it)
            }
        }
    fun generateReactorStructFields() =
        getUcConnections().joinToString(prefix = "// Connections \n", separator = "\n", postfix = "\n") { "${it.codeType} ${it.codeName};" }

    fun generateReactorCtorCode(conn: UcConnection)  =  with(PrependOperator) {
        """
            |${conn.codeType}_ctor(&self->${conn.codeName}, &self->super, (Port *) &self->${getPortCodeName(conn.src)});
        ${" |   "..generateConnectionStatements(conn)}
            |
            """.trimMargin()
    };
    fun generateConnectionStatements(conn: UcConnection) = conn.getDests().joinToString(separator = "\n") {
        "CONN_REGISTER_DOWNSTREAM(self->${conn.codeName}, self->${getPortCodeName(it)});"
    }
    fun generateReactorCtorCodes() = getUcConnections().joinToString(prefix = "// Initialize connections\n", separator = "\n", postfix = "\n") { generateReactorCtorCode(it)}

    fun generateLogicalCtor(conn: UcConnection) = with(PrependOperator) {
        """
            |static void ${conn.codeType}_ctor(${conn.codeType} *self, Reactor *parent, Port *upstream) {
            |   LogicalConnection_ctor(&self->super, parent, upstream, self->_downstreams, ${conn.getDests().size});
            |}
        """.trimMargin()
    }
    fun generateDelayedCtor(conn: UcConnection) = with(PrependOperator) {
        """
            |static void ${conn.codeType}_ctor(${conn.codeType} *self, Reactor *parent, Port *upstream) {
            |   DelayedConnection_ctor(&self->super, parent, upstream, self->_downstreams, ${conn.getDests().size}, ${conn.conn.delay.toCCode()}, self->buffer, sizeof(self->buffer[0]), ${conn.bufSize});
            |}
        """.trimMargin()
    }

    fun generateCtors() = getUcConnections().joinToString(prefix = "// Connection constructors\n", separator = "\n", postfix = "\n"){
        if(it.conn.isPhysical) unreachable()
        else if(it.conn.delay != null) generateDelayedCtor(it)
        else generateLogicalCtor(it)
    }
    }

