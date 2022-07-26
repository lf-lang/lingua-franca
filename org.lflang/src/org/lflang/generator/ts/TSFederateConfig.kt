package org.lflang.generator.ts

class TSFederateConfig (
    private val federateId: Int,
    private val rtiHost: String,
    private val rtiPort: Int
 ) {
    fun getFederateId() = federateId
    fun getRtiHost() = rtiHost
    fun getRtiPort() = rtiPort
}