/*
 * Copyright (c) 2021, TU Dresden.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.lflang.generator.rust;

import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.stream.Collectors;
import org.lflang.ASTUtils;
import org.lflang.TargetProperty;
import org.lflang.TargetProperty.TargetPropertyType;
import org.lflang.generator.InvalidLfSourceException;
import org.lflang.lf.Array;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfFactory;
import org.lflang.util.StringUtil;
import org.lflang.validation.LFValidator;

/**
 * Info about a cargo dependency. See {@link TargetProperty#CARGO_DEPENDENCIES}.
 *
 * @author Clément Fournier - TU Dresden, INSA Rennes
 */
public class CargoDependencySpec {

  private final String version;
  private String gitRepo;
  private String rev;
  private String gitTag;
  private String localPath;
  private final Set<String> features;

  CargoDependencySpec(
      String version,
      String gitRepo,
      String rev,
      String gitTag,
      String localPath,
      List<String> features) {
    this.version = StringUtil.removeQuotes(version);
    this.gitRepo = gitRepo;
    this.rev = rev;
    this.gitTag = gitTag;
    this.localPath = StringUtil.removeQuotes(localPath);
    this.features = new HashSet<>();
    if (features != null) {
      this.features.addAll(features);
    }
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) {
      return true;
    }
    if (o == null || getClass() != o.getClass()) {
      return false;
    }
    CargoDependencySpec that = (CargoDependencySpec) o;
    return Objects.equals(version, that.version)
        && Objects.equals(gitRepo, that.gitRepo)
        && Objects.equals(rev, that.rev)
        && Objects.equals(gitTag, that.gitTag)
        && Objects.equals(localPath, that.localPath)
        && Objects.equals(features, that.features);
  }

  /** The version. May be null. */
  public String getVersion() {
    return version;
  }

  /** Local path to the crate. May be null. */
  public String getLocalPath() {
    return localPath;
  }

  /** Returns the git path. */
  public String getGitRepo() {
    return gitRepo;
  }

  /** Returns the revision number to use with git/localPath. */
  public String getRev() {
    return rev;
  }

  public String getTag() {
    return gitTag;
  }

  public void setGitRepo(String gitRepo) {
    this.gitRepo = gitRepo;
    if (gitRepo != null) {
      this.localPath = null;
    }
  }

  public void setRev(String rev) {
    this.rev = rev;
  }

  public void setLocalPath(String localPath) {
    this.localPath = localPath;
    if (localPath != null) {
      this.gitRepo = null;
      this.rev = null;
      this.gitTag = null;
    }
  }

  /** Returns the list of features that are enabled on the crate. May be null. */
  public Set<String> getFeatures() {
    return features;
  }

  /**
   * Parse the given element. It must be a JSON map, whose keys are dependency names, and values are
   * {@link CargoDependencySpec}s.
   *
   * @throws InvalidLfSourceException If the element is somehow invalid
   */
  public static Map<String, CargoDependencySpec> parseAll(Element element) {
    var result = new LinkedHashMap<String, CargoDependencySpec>();
    if (element.getKeyvalue() == null) {
      throw new InvalidLfSourceException(element, "Expected key-value pairs");
    }
    for (KeyValuePair pair : element.getKeyvalue().getPairs()) {
      result.put(pair.getName(), parseValue(pair));
    }
    return result;
  }

  private static CargoDependencySpec parseValue(KeyValuePair pair) {
    // note that we hardcode the value because RustEmitterBase is a
    // kotlin class and we can't depend on it to use a constant.
    boolean isRuntimeCrate = pair.getName().equals("reactor_rt");
    return parseValue(pair.getValue(), isRuntimeCrate);
  }

  /**
   * Parse an element into a CargoDependencySpec. This is used for values of the {@link
   * TargetProperty#CARGO_DEPENDENCIES} map.
   *
   * @throws InvalidLfSourceException If the element is somehow invalid
   */
  private static CargoDependencySpec parseValue(Element element, boolean isRuntimeCrate) {
    if (element.getLiteral() != null) {
      return new CargoDependencySpec(element.getLiteral(), null, null, null, null, null);
    } else if (element.getKeyvalue() != null) {
      String version = null;
      String localPath = null;
      String gitRepo = null;
      String rev = null;
      String tag = null;
      List<String> features = null;
      for (KeyValuePair pair : element.getKeyvalue().getPairs()) {
        String name = pair.getName();
        if ("features".equals(name)) {
          Array array = pair.getValue().getArray();
          if (array == null) {
            throw new InvalidLfSourceException(
                pair.getValue(), "Expected an array of strings for key '" + name + "'");
          }
          features =
              array.getElements().stream()
                  .map(ASTUtils::elementToSingleString)
                  .map(StringUtil::removeQuotes)
                  .collect(Collectors.toList());
          continue;
        }
        String literal = pair.getValue().getLiteral();
        if (literal == null) {
          throw new InvalidLfSourceException(
              pair.getValue(), "Expected string literal for key '" + name + "'");
        }
        switch (name) {
        case "version" -> version = literal;
        case "git" -> gitRepo = literal;
        case "rev" -> rev = literal;
        case "tag" -> tag = literal;
        case "path" -> localPath = literal;
        default -> throw new InvalidLfSourceException(pair,
                                                      "Unknown key: '" + name
                                                          + "'");
        }
      }
      if (isRuntimeCrate || version != null || localPath != null || gitRepo != null) {
        return new CargoDependencySpec(version, gitRepo, rev, tag, localPath, features);
      } else {
        throw new InvalidLfSourceException(
            element.getKeyvalue(), "Must specify one of 'version', 'path', or 'git'");
      }
    }
    throw new InvalidLfSourceException(element, "Expected string or dictionary");
  }

  /** Extracts an AST representation of a CargoDependencySpec. */
  public static Element extractSpec(CargoDependencySpec spec) {
    if (spec.gitRepo == null
        && spec.rev == null
        && spec.gitTag == null
        && spec.localPath == null
        && spec.features == null) {
      return ASTUtils.toElement(spec.version);
    } else {
      Element e = LfFactory.eINSTANCE.createElement();
      KeyValuePairs kvp = LfFactory.eINSTANCE.createKeyValuePairs();
      addKvp(kvp, "version", spec.version);
      addKvp(kvp, "git", spec.gitRepo);
      addKvp(kvp, "rev", spec.rev);
      addKvp(kvp, "tag", spec.gitTag);
      addKvp(kvp, "path", spec.localPath);
      if (spec.features != null && !spec.features.isEmpty()) {
        Element subE = LfFactory.eINSTANCE.createElement();
        Array arr = LfFactory.eINSTANCE.createArray();
        for (String f : spec.features) {
          arr.getElements().add(ASTUtils.toElement(f));
        }
        subE.setArray(arr);
        KeyValuePair pair = LfFactory.eINSTANCE.createKeyValuePair();
        pair.setName("features");
        pair.setValue(subE);
        kvp.getPairs().add(pair);
      }
      e.setKeyvalue(kvp);
      return e;
    }
  }

  private static void addKvp(KeyValuePairs pairs, String name, String value) {
    if (value == null) {
      return;
    }
    KeyValuePair kvp = LfFactory.eINSTANCE.createKeyValuePair();
    kvp.setName(name);
    kvp.setValue(ASTUtils.toElement(value));
    pairs.getPairs().add(kvp);
  }

  /** The property type for the */
  public static final class CargoDependenciesPropertyType implements TargetPropertyType {

    public static final TargetPropertyType INSTANCE = new CargoDependenciesPropertyType();

    private CargoDependenciesPropertyType() {}

    @Override
    public boolean validate(Element e) {
      return e.getKeyvalue() != null;
    }

    @Override
    public void check(Element element, String name, LFValidator v) {
      for (KeyValuePair pair : element.getKeyvalue().getPairs()) {
        try {
          parseValue(pair);
        } catch (InvalidLfSourceException e) {
          v.getErrorReporter().reportError(e.getNode(), e.getProblem());
        }
      }
    }

    @Override
    public String toString() {
      return "<cargo dependency spec>";
    }
  }
}
