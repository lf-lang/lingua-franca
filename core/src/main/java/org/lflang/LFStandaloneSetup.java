package org.lflang;

import com.google.inject.Guice;
import com.google.inject.Injector;

/**
 * Initialization support for running Xtext languages without Equinox extension registry.
 *
 * See {@link LFRuntimeModule}, the base Guice module for LF services.
 *
 * @ingroup Infrastructure
 */
public class LFStandaloneSetup extends LFStandaloneSetupGenerated {

  protected static Injector injector;

  public static Injector doSetup() {
    if (injector == null) {
      injector = new LFStandaloneSetup().createInjectorAndDoEMFRegistration();
    }
    return injector;
  }

  @Override
  public Injector createInjector() {
    return Guice.createInjector(new LFRuntimeModule());
  }
}
