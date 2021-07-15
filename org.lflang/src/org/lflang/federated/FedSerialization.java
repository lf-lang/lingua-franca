package org.lflang.federated;

import org.lflang.generator.GeneratorBase;

public interface FedSerialization {
    
    final static String serializedVarName = "serialized_message";
    final static String deserializedVarName = "deserialized_message";
    
    /**
     * Check whether the current generator is compatible with the given
     * serialization technique or not.
     * 
     * @param generator The current generator.
     * @return true if compatible, false if not.
     */
    public boolean isCompatible(GeneratorBase generator);

    public String serializedVarLength();
    public String seializedVarBuffer();
    
    public StringBuilder generateNetworkSerialzerCode(String portName, String portType);
    
    public StringBuilder generateNetworkDeserializerCode(String portName, String portType);
}
