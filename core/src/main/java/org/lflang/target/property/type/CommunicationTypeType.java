package org.lflang.target.property.type;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import org.lflang.target.property.type.CommunicationTypeType.CommunicationType;

/** Enumeration of communication types */
public class CommunicationTypeType extends OptionsType<CommunicationType> {

  @Override
  protected Class<CommunicationType> enumClass() {
    return CommunicationType.class;
  }

  /**
   * Enumeration of communication types.
   *
   * <ul>
   *   <li>TCP: Communications occur through TCP servers/clients.
   *   <li>SST: Communications occur through SST modules.
   *   <li>MQTT: Communications occur through a broker and pub/sub methods.
   * </ul>
   */
  public enum CommunicationType {
    TCP("TCP"),
    SST("SST"),
    MQTT("MQTT");

    /** Alias used in toString method. */
    private final String alias;

    /** Private constructor for Cmake build types. */
    CommunicationType(String alias) {
      this.alias = alias;
    }

    /** Return the alias. */
    @Override
    public String toString() {
      return this.alias;
    }

    public static List<CommunicationType> optionsList() {
      return Arrays.stream(CommunicationType.values()).collect(Collectors.toList());
    }

    public static CommunicationType getDefault() {
      return CommunicationType.TCP;
    }
  }
}