package org.lflang.target.property.type;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import org.lflang.target.property.type.CommunicationModeType.CommunicationMode;

/** Enumeration of communication types */
public class CommunicationModeType extends OptionsType<CommunicationMode> {

  @Override
  protected Class<CommunicationMode> enumClass() {
    return CommunicationMode.class;
  }

  /** Enumeration of communication types. */
  public enum CommunicationMode {
    TCP("TCP"),
    SST("SST");

    /** Alias used in toString method. */
    private final String alias;

    /** Private constructor for Cmake build types. */
    CommunicationMode(String alias) {
      this.alias = alias;
    }

    /** Return the alias. */
    @Override
    public String toString() {
      return this.alias;
    }

    public static List<CommunicationMode> optionsList() {
      return Arrays.stream(CommunicationMode.values()).collect(Collectors.toList());
    }

    public static CommunicationMode getDefault() {
      return CommunicationMode.TCP;
    }
  }
}
