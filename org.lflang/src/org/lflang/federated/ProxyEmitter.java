package org.lflang.federated;

import org.lflang.lf.Reactor;

public class ProxyEmitter {

    static String emitProxy(Reactor r) {

        String parameterList = emitParameterList(r);
        String.join(
            System.getProperty("line.separator"),
            "target Cpp;",
            "reactor {",

            "}"
        );

        return """
            reactor <name> (<parameters>) {
                        
                        
                        
            <ports>
                        
                        
            @interface
            <instance of r w/parameters>
                        
            <instances for senders and receivers>
                        
            <connections>
                        
                        
                        
            }""";
    }

    static String emitParameterList(Reactor r) {
        //                         int message_type,
        //                        unsigned short port,
        //                        unsigned short federate,
        return "";
    }

}
