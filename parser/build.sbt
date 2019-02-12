// Project name (artifact name in Maven)
name := "Lingua Franca"

// orgnization name (e.g., the package name of the project)
organization := "org.icyphy"

version := "1.0-SNAPSHOT"

// project description
description := "Parser for Lingua Franca"

// Enables publishing to maven repo
publishMavenStyle := true

// Do not append Scala versions to the generated artifacts
crossPaths := false

// This forbids including Scala related libraries into the dependency
autoScalaLibrary := false

// library dependencies. (orginization name) % (project name) % (version)
//libraryDependencies ++= Seq(
//   "org.apache.commons" % "commons-math3" % "3.1.1",
//   "org.fluentd" % "fluent-logger" % "0.2.10",
//   "org.mockito" % "mockito-core" % "1.9.5" % "test"  // Test-only dependency
//)

