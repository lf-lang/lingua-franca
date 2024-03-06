package org.lflang.diagram.lsp;

import java.util.Objects;

public class BuildArgs {

    private String uri;

    private String json;

    public String getUri() {
        return uri;
    }

    public void setUri(String uri) {
        this.uri = uri;
    }

    public String getJson() {
        return json;
    }

    public void setJson(String json) {
        this.json = json;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (o == null || getClass() != o.getClass()) {
            return false;
        }
        BuildArgs buildArgs = (BuildArgs) o;
        return Objects.equals(uri, buildArgs.uri)
            && Objects.equals(json, buildArgs.json);
    }

    @Override
    public int hashCode() {
        return Objects.hash(uri, json);
    }
}
