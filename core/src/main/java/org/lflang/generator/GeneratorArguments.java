package org.lflang.generator;

import com.google.gson.JsonObject;
import java.net.URI;
import org.lflang.target.property.type.BuildTypeType.BuildType;
import org.lflang.target.property.type.LoggingType.LogLevel;
import org.lflang.target.property.type.SchedulerType.Scheduler;

/** */
public class GeneratorArguments {

  /**
   * @see org.lflang.target.property.BuildTypeProperty
   */
  public BuildType buildType;

  /** Whether to clean before building. */
  public boolean clean;

  /**
   * @see org.lflang.target.property.ExternalRuntimePathProperty
   */
  public URI externalRuntimeUri;

  /**
   * @see org.lflang.target.property.HierarchicalBinProperty
   */
  public Boolean hierarchicalBin;

  /** Generator arguments and target properties, if they were passed to the application. */
  public JsonObject jsonObject;

  /** For enabling or disabling the linting of generated code */
  public boolean lint;

  /**
   * @see org.lflang.target.property.LoggingProperty
   */
  public LogLevel logging;

  /**
   * @see org.lflang.target.property.PrintStatisticsProperty
   */
  public Boolean printStatistics;

  /**
   * @see org.lflang.target.property.NoCompileProperty
   */
  public Boolean noCompile;

  /**
   * @see org.lflang.target.property.VerifyProperty
   */
  public Boolean verify;

  /**
   * @see org.lflang.target.property.CompilerProperty
   */
  public String compiler;

  /** Whether to suppress output of the target compiler and other commands. */
  public boolean quiet;

  /** The location of the rti. */
  public URI rti;

  /**
   * @see org.lflang.target.property.RuntimeVersionProperty
   */
  public String runtimeVersion;

  /**
   * @see org.lflang.target.property.SchedulerProperty
   */
  public Scheduler scheduler;

  /**
   * @see org.lflang.target.property.SchedulerProperty
   */
  public Boolean threading;

  /**
   * @see org.lflang.target.property.SchedulerProperty
   */
  public Boolean tracing;

  /**
   * @see org.lflang.target.property.WorkersProperty
   */
  public Integer workers;
}
