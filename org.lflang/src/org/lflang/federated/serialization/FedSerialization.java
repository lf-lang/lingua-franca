package org.lflang.federated.serialization;

import org.lflang.generator.GeneratorBase;

/**
 * Interface to enable support for automatic data serialization 
 * in target code.
 * 
 * @author Soroush Bateni
 *
 */
public interface FedSerialization {
    
    /**
     * Variable name in the target language for the serialized data.
     */
    final static String serializedVarName = "serialized_message";
    
    /**
     * Variable name in the target language for the deserialized data.
     */
    final static String deserializedVarName = "deserialized_message";
    
    /**
     * Check whether the current generator is compatible with the given
     * serialization technique or not.
     * 
     * @param generator The current generator.
     * @return true if compatible, false if not.
     */
    public boolean isCompatible(GeneratorBase generator);

    /** 
     * @return Expression in target language that corresponds to the length
     *  of the serialized buffer.
     */
    public String serializedBufferLength();
    
    /**
     * @return Expression in target language that is the buffer variable 
     *  itself.
     */
    public String seializedBufferVar();
    
    /**
     * Generate code in target language that serializes 'varName'. This code
     * will convert the data in 'varName' from its 'originalType' into an 
     * unsigned byte array. The serialized data will be put in a variable 
     * with the name defined by @see serializedVarName.
     * 
     * @param varName The variable to be serialized.
     * @param originalType The original type of the variable.
     * @return Target code that serializes the 'varName' from 'type'
     *  to an unsigned byte array.
     */
    public StringBuilder generateNetworkSerializerCode(String varName, String originalType);
    
    /**
     * Generate code in target language that deserializes 'varName'. This code will
     * convert the data in 'varName' from an unsigned byte array into the 'targetType'.
     * The deserialized data will be put in a variable with the name defined by 
     * @see deserializedVarName.
     *  
     * @param varName The variable to deserialize.
     * @param targetType The type to deserialize into.
     * @return Target code that deserializes 'varName' from an unsigned byte array
     *  to 'type'.
     */
    public StringBuilder generateNetworkDeserializerCode(String varName, String targetType);
    
    /**
     * @return Code in target language that includes all the necessary preamble to enable
     *  support for the current serializer.
     */
    public StringBuilder generatePreambleForSupport();
    
    /** 
     * @return Code that should be appended to the compiler instructions (e.g., flags to gcc or
     *  additional lines to a CMakeLists.txt) to enable support for the current serializer.
     */
    public StringBuilder generateCompilerExtensionForSupport();
}
