package org.lflang.generator.c;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.lflang.TargetProperty.SchedulerOption;

/**
 * Generates the list of files to be included in the
 * core library for each reactor given conditions listed
 * as arguments of each function.
 *
 * @author{Hou Seng Wong <housengw@berkeley.edu>}
 */
public class CCoreFilesUtils {
    public static List<String> getCoreFiles(
        boolean isFederated,
        boolean threading,
        SchedulerOption scheduler
    ) {
        List<String> coreFiles = new ArrayList<>();
        coreFiles.addAll(getBaseCoreFiles());
        coreFiles.addAll(getPlatformFiles());
        if (isFederated) {
            coreFiles.addAll(getFederatedFiles());
        }
        coreFiles.addAll(getThreadSupportFiles(threading, scheduler));
        return coreFiles;
    }

    public static List<String> getCTargetSrc() {
        return List.of(
            "lib/schedule.c",
            "lib/util.c",
            "lib/tag.c",
            "lib/time.c"
        );
    }

    public static List<String> getArduinoTargetHeaders() {
        return List.of(
            "Arduino.h"
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

    private static List<String> getBaseCoreFiles() {
        return List.of(
            "reactor_common.c",
            "reactor.h",
            "tag.h",
            "tag.c",
            "trace.h",
            "trace.c",
            "port.h",
            "port.c",
            "utils/pqueue.c",
            "utils/pqueue.h",
            "utils/pqueue_support.h",
            "utils/vector.c",
            "utils/vector.h",
            "utils/semaphore.h",
            "utils/semaphore.c",
            "utils/util.h",
            "utils/util.c",
            "platform.h",
            "platform/Platform.cmake",
            "mixed_radix.c",
            "mixed_radix.h",
            "modal_models/modes.h",
            "modal_models/modes.c"
        );
    }

    private static List<String> getPlatformFiles() {
        return List.of(
            "platform/lf_tag_64_32.h",
            "platform/lf_POSIX_threads_support.c",
            "platform/lf_C11_threads_support.c",
            "platform/lf_C11_threads_support.h",
            "platform/lf_POSIX_threads_support.h",
            "platform/lf_POSIX_threads_support.c",
            "platform/lf_unix_clock_support.c",
            "platform/lf_unix_syscall_support.c",
            "platform/lf_macos_support.c",
            "platform/lf_macos_support.h",
            "platform/lf_windows_support.c",
            "platform/lf_windows_support.h",
            "platform/lf_arduino_support.c",
            "platform/lf_arduino_support.h",
            "platform/lf_linux_support.c",
            "platform/lf_linux_support.h"
        );
    }

    private static List<String> getFederatedFiles() {
        return List.of(
            "federated/net_util.c",
            "federated/net_util.h",
            "federated/net_common.h",
            "federated/federate.c",
            "federated/federate.h",
            "federated/clock-sync.h",
            "federated/clock-sync.c"
        );
    }

    private static List<String> getThreadSupportFiles(
        boolean threading,
        SchedulerOption scheduler
    ) {
        return threading ?
            Stream.concat(
                Stream.of(
                    "threaded/scheduler.h",
                    "threaded/scheduler_instance.h",
                    "threaded/scheduler_sync_tag_advance.c",
                    "threaded/reactor_threaded.c"
                ),
                scheduler.getRelativePaths().stream().map(path -> "threaded/" + path.toString().replace("\\", "/"))
            ).collect(Collectors.toList()) :
            List.of("reactor.c");
    }
}
