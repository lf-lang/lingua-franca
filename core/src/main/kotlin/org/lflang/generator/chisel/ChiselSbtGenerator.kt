package org.lflang.generator.chisel

import org.lflang.ErrorReporter
import org.lflang.lf.Reactor

class ChiselSbtGenerator(private val mainReactor: Reactor, fileConfig: ChiselFileConfig, errorReporter: ErrorReporter) {

    fun generateSource() =
        """
            | ThisBuild / scalaVersion     := "2.13.8"
            | ThisBuild / version          := "0.1.0"
            | ThisBuild / organization     := ""
            |
            | val chiselVersion = "3.5.3"
            | val chiselTestVersion = "0.5.1"
            |
            | lazy val root = (project in file("."))
            |   .settings(
            |     name := "lf-${mainReactor.name}",
            |     libraryDependencies ++= Seq(
            |       "edu.berkeley.cs" %% "chisel3" % chiselVersion,
            |       "edu.berkeley.cs" %% "chiseltest" % chiselTestVersion % "test"
            |       ),
            |     scalacOptions ++= Seq(
            |       "-language:reflectiveCalls",
            |       "-deprecation",
            |       "-feature",
            |       "-Xcheckinit",
            |       "-P:chiselplugin:genBundleElements",
            |       ),
            |     addCompilerPlugin("edu.berkeley.cs" % "chisel3-plugin" % chiselVersion cross CrossVersion.full),
            |     ).dependsOn(reactorchisel)
            |
            | lazy val reactorchisel = (project in file("./reactor-chisel"))
            |
            | 
        """.trimMargin()
}
