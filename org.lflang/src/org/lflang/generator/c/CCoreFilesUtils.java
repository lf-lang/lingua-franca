package org.lflang.generator.c;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
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
            "ctarget/schedule.c",
            "ctarget/util.c"
        );
    }

    private static List<String> getBaseCoreFiles() {
        return List.of(
            "reactor_common.c",
            "reactor.h",
            "tag.h",
            "tag.c",
            "trace.h",
            "trace.c",
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
                List.of(
                    "threaded/scheduler.h",
                    "threaded/scheduler_instance.h",
                    "threaded/scheduler_sync_tag_advance.c",
                    "threaded/scheduler_" + scheduler + ".c",
                    "threaded/reactor_threaded.c"
                ) :
                List.of("reactor.c");
    }
}
