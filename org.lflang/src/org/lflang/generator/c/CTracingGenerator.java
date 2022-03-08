package org.lflang.generator.c;

import java.util.ArrayList;
import java.util.List;

import org.lflang.federated.FederateInstance;
import org.lflang.generator.ActionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.TimerInstance;

public class CTracingGenerator {
    /**
     * If tracing is turned on, then generate code that records
     * the full name of the specified reactor instance in the
     * trace table. If tracing is not turned on, do nothing.
     * 
     * If tracing is turned on, record the address of this reaction
     * in the _lf_trace_object_descriptions table that is used to generate
     * the header information in the trace file.
     * 
     * @param instance The reactor instance.
     * @param currentFederate The federate instance we are generating code for. 
     */
    public static String generateTraceTableEntries(
        ReactorInstance instance,
        FederateInstance currentFederate
    ) {
        List<String> code = new ArrayList<>();
        var description = CUtil.getShortenedName(instance);
        var selfStruct = CUtil.reactorRef(instance);
        code.add(registerTraceEvent(selfStruct, "NULL", "trace_reactor", description));
        for (ActionInstance action : instance.actions) {
            if (currentFederate.contains(action.getDefinition())) {
                code.add(registerTraceEvent(selfStruct, action.getName(), "trace_trigger", description));
            }
        }
        for (TimerInstance timer : instance.timers) {
            if (currentFederate.contains(timer.getDefinition())) {
                code.add(registerTraceEvent(selfStruct, timer.getName(), "trace_trigger", description));
            }
        }
        return String.join("\n", code);
    }

    public static String generateTracingHeader(String traceFileName) {
        if (traceFileName == null) {
            return "";
        }
        return "#define LINGUA_FRANCA_TRACE " + traceFileName;
    }
 
    private static String registerTraceEvent(String obj, String trigger, String type, String description) {
        return "_lf_register_trace_event("+obj+", &("+obj+"->_lf__"+trigger+"), "+type+", \""+description+"."+trigger+"\");";
    }
}
