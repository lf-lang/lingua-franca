package org.icyphy.tests;

import java.io.File;
import java.nio.file.Path;

import org.icyphy.Target;

public class Test {
    public final Path path;
    public final String name;
    
    public Test(Target target, Path path) {
        this.path = path;
        this.name = normalize(target, path);
    }
    
    private static String normalize(Target target, Path path) {
        return path.toString().replaceFirst(TestRegistry.LF_TEST_PATH + target + File.separator, "");
    }
    
    @Override
    public boolean equals(Object o) {
        if (o instanceof Test && ((Test) o).name.equals(this.name)) {
            return true;
        }
        return false;
    }
    
    @Override
    public String toString() {
        return this.name;
    }
    
    @Override
    public int hashCode() {
        return this.name.hashCode();
    }
}
