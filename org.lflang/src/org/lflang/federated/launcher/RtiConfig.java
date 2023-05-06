package org.lflang.federated.launcher;

import java.nio.file.Path;

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
}
