package org.lflang.generator.c;
import java.util.List;

/**
 * Generates the list of files to be included in the
 * core library for each reactor given conditions listed
 * as arguments of each function.
 *
 * @author Hou Seng Wong <housengw@berkeley.edu>
 */
public class CCoreFilesUtils {

    public static List<String> getCTargetSrc() {
        return List.of(
            "lib/schedule.c"
        );
    }

    public static List<String> getCTargetHeader() {
        return List.of(
            "include/ctarget/ctarget.h"
        );
    }

    public static String getCTargetSetHeader() {
        return "include/ctarget/set.h";
    }

    public static String getCTargetSetUndefHeader() {
        return "include/ctarget/set_undef.h";
    }
}
