package org.lflang.cli;

import com.google.inject.Binder;
import com.google.inject.Module;
import java.util.Objects;
import org.eclipse.emf.ecore.EValidator;
import org.eclipse.emf.ecore.impl.EValidatorRegistryImpl;
import org.eclipse.xtext.validation.ValidationMessageAcceptor;
import org.lflang.LFRuntimeModule;
import org.lflang.MessageReporter;

/**
 * Module that is only available when running LFC as a standalone program.
 *
 * @see LFRuntimeModule
 * @ingroup CLI
 */
public class LFStandaloneModule implements Module {
  // Note that xtext's base module classes has broken support
  // for @Provides, which would allow us to bind this field.
  // So we directly implement Module, instead of extending eg LFRuntimeModule.
  private final ReportingBackend helper;
  private final Io io;

  public LFStandaloneModule(ReportingBackend helper, Io io) {
    this.helper = Objects.requireNonNull(helper);
    this.io = Objects.requireNonNull(io);
  }

  @Override
  public void configure(Binder binder) {
    binder.bind(MessageReporter.class).to(StandaloneMessageReporter.class);
    binder.bind(ReportingBackend.class).toInstance(helper);
    binder.bind(Io.class).toInstance(io);
    binder.bind(ValidationMessageAcceptor.class).to(StandaloneIssueAcceptor.class);
    // This is required to force the ResourceValidator to
    // use a new registry instance (which is reused by the injector as a singleton).
    // Otherwise, it uses the static EValidator.Registry.INSTANCE which is bad
    // as the first validator to be created would persist in that static instance.
    // New injectors would reuse the existing instance, but
    // its fields would have been injected by an older injector
    // and be out of sync with the rest of the application.
    binder.bind(EValidator.Registry.class).to(EValidatorRegistryImpl.class).asEagerSingleton();
  }
}
