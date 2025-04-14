package org.lflang.federated.launcher;

import java.nio.file.Path;
import org.lflang.FileConfig;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.AuthProperty;
import org.lflang.target.property.LoggingProperty;

/**
 * Class for storing configuration settings pertaining to the RTI.
 *
 * @author Marten Lohstroh
 */
public class RtiConfig {

  private Path directory;

  /** The host on which the RTI process is to be spawned. */
  private String host;

  /** The port on which to connect to the RTI process. */
  private int port;

  /** The username used to gain access to the host where the RTI is to be spawned. */
  private String user;

  /** Construct a new RTI configuration with all options set to their defaults. */
  public RtiConfig() {
    this.directory = Path.of("LinguaFrancaRemote");
    this.host = "localhost";
    this.port = 0;
  }

  /** Return the directory to create on the remote host. */
  public Path getDirectory() {
    return directory;
  }

  /** Return the host on which the RTI process is to be spawned. */
  public String getHost() {
    return host;
  }

  /** Return the port on which to connect to the RTI process. */
  public int getPort() {
    return port;
  }

  /** Return the username used to gain access to the host where the RTI is to be spawned. */
  public String getUser() {
    return user;
  }

  /** Return the path to the RTI binary on the remote host. */
  public String getRtiBinPath(FileConfig fileConfig) {
    return "~/" + directory.resolve(fileConfig.name).resolve("bin/RTI").toString();
  }

  /** Set the directory to create on the remote host. */
  public void setDirectory(Path directory) {
    this.directory = directory;
  }

  /** Set the host on which the RTI process is to be spawned. */
  public void setHost(String host) {
    this.host = host;
  }

  /** Set the port on which to connect to the RTI process. */
  public void setPort(int port) {
    this.port = port;
  }

  /** Set the username used to gain access to the host where the RTI is to be spawned. */
  public void setUser(String user) {
    this.user = user;
  }

  /** The CMakeLists.txt to be code-generated to build the RTI.  */
  public String getRtiCmake(TargetConfig targetConfig) {
    return String.join(
        "\n",
        "cmake_minimum_required(VERSION 3.12)",
        "project(RTI VERSION 1.0.0 LANGUAGES C)",
        "set(LOG_LEVEL " + targetConfig.get(LoggingProperty.INSTANCE).ordinal() + ")",
        "set(AUTH " + (targetConfig.get(AuthProperty.INSTANCE) ? "ON" : "OFF") + ")",
        "set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})",
        "add_subdirectory(${CMAKE_SOURCE_DIR}/core/federated/RTI ${CMAKE_BINARY_DIR}/build_RTI)");
  }
}
