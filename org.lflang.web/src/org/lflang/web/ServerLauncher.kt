@file:JvmName("ServerLauncher")

package org.lflang.web

import java.net.InetSocketAddress
import org.eclipse.jetty.annotations.AnnotationConfiguration
import org.eclipse.jetty.server.Server
import org.eclipse.jetty.util.log.Slf4jLog
import org.eclipse.jetty.webapp.MetaInfConfiguration
import org.eclipse.jetty.webapp.WebAppContext
import org.eclipse.jetty.webapp.WebInfConfiguration
import org.eclipse.jetty.webapp.WebXmlConfiguration
import kotlin.concurrent.thread
import kotlin.system.exitProcess

/**
 * This program starts an HTTP server for testing the web integration of your DSL.
 * Just execute it and point a web browser to http://localhost:8080/
 */
fun main(args: Array<String>) {
	val server = Server(InetSocketAddress("localhost", 8080))
	server.handler = WebAppContext().apply {
		resourceBase = "WebRoot"
		welcomeFiles = arrayOf("index.html")
		contextPath = "/"
		configurations = arrayOf(
			AnnotationConfiguration(),
			WebXmlConfiguration(),
			WebInfConfiguration(),
			MetaInfConfiguration()
		)
		setAttribute(WebInfConfiguration.CONTAINER_JAR_PATTERN, ".*/org\\.icyphy\\.linguafranca\\.web/.*,.*\\.jar")
		setInitParameter("org.mortbay.jetty.servlet.Default.useFileMappedBuffer", "false")
	}

	val log = Slf4jLog("org.lflang.web.ServerLauncher")
	try {
		server.start()
		log.info("Server started ${server.uri}...")

		thread(start = true) {
			log.info("Press enter to stop the server...")
			val key = System.`in`.read()
			if (key != -1) {
				server.stop()
			} else {
				log.warn("Console input is not available. In order to stop the server, you need to cancel process manually.")
			}
		}
		server.join()
	} catch (exception: Exception) {
		log.warn(exception.message)
		exitProcess(1)
	}
}
