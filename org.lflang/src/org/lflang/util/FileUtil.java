package org.lflang.util;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.IPath;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;

public class FileUtil {

    /**
     * Return the name of the file excluding its file extension.
     * @param file A Path object
     * @return The name of the file excluding its file extension.
     */
    public static String nameWithoutExtension(Path file) {
        String name = file.getFileName().toString();
        int idx = name.lastIndexOf('.');
        return idx < 0 ? name : name.substring(0, idx);
    }

    /**
     * Return the name of the file associated with the given resource,
     * excluding its file extension.
     * @param r Any {@code Resource}.
     * @return The name of the file associated with the given resource,
     * excluding its file extension.
     * @throws IOException If the resource has an invalid URI.
     */
    public static String nameWithoutExtension(Resource r) throws IOException {
        return nameWithoutExtension(toPath(r));
    }

    /**
     * Return a java.nio.Path object corresponding to the given URI.
     * @throws IOException If the given URI is invalid.
     */
    public static Path toPath(URI uri) throws IOException {
        return Paths.get(toIPath(uri).toFile().getAbsolutePath());
    }

    /**
     * Return a java.nio.Path object corresponding to the given Resource.
     * @throws IOException If the given resource has an invalid URI.
     */
    public static Path toPath(Resource resource) throws IOException {
        return toPath(resource.getURI());
    }

    /**
     * Return an org.eclipse.core.runtime.Path object corresponding to the
     * given URI.
     * @throws IOException If the given URI is invalid.
     */
    public static IPath toIPath(URI uri) throws IOException {
        if (uri.isPlatform()) {
            IPath path = new org.eclipse.core.runtime.Path(uri.toPlatformString(true));
            if (path.segmentCount() == 1) {
                return ResourcesPlugin.getWorkspace().getRoot().getProject(path.lastSegment()).getLocation();
            } else {
                return ResourcesPlugin.getWorkspace().getRoot().getFile(path).getLocation();
            }
        } else if (uri.isFile()) {
            return new org.eclipse.core.runtime.Path(uri.toFileString());
        } else {
            throw new IOException("Unrecognized file protocol in URI " + uri);
        }
    }

    /**
     * Convert a given path to a unix-style string.
     *
     * This ensures that '/' is used instead of '\' as file separator.
     */
    public static String toUnixString(Path path) {
        return path.toString().replace('\\', '/');
    }
}
