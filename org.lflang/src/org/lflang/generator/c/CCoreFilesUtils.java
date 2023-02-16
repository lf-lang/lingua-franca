package org.lflang.generator.c;
import java.util.List;

/**
 * Generates the list of files to be included in the
 * core library for each reactor given conditions listed
 * as arguments of each function.
 *
 * @author Hou Seng Wong
 */
public class CCoreFilesUtils {

    public static List<String> getCTargetSrc() {
        return List.of(
            "lib/schedule.c"
        );
    }

    public static List<String> getCTargetHeader() {
        return List.of(
            "include/api/api.h"
        );
    }

    public static String getCTargetSetHeader() {
        return "include/api/set.h";
    }

    public static String getCTargetSetUndefHeader() {
        return "include/api/set_undef.h";
    }
}
