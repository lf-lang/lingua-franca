package org.lflang.federated.launcher;

import java.nio.file.Path;

public class RtiConfig {

    private Path directory;

    private String host;

    private int port;

    private String user;

    public RtiConfig() {
        this.directory = Path.of("LinguaFrancaRemote");
        this.host = "localhost";
        this.port = 0;
    }

    public Path getDirectory() {
        return directory;
    }

    public String getHost() {
        return host;
    }

    public int getPort() {
        return port;
    }

    public String getUser() {
        return user;
    }

    public void setDirectory(Path directory) {
        this.directory = directory;
    }

    public void setHost(String host) {
        this.host = host;
    }

    public void setPort(int port) {
        this.port = port;
    }

    public void setUser(String user) {
        this.user = user;
    }

}
