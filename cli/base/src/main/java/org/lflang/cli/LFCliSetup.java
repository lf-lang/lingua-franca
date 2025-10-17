package org.lflang.cli;

import com.google.inject.Guice;
import com.google.inject.Injector;
import com.google.inject.Key;
import org.eclipse.emf.ecore.EPackage;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.xmi.impl.EcoreResourceFactoryImpl;
import org.eclipse.emf.ecore.xmi.impl.XMIResourceFactoryImpl;
import org.eclipse.xtext.XtextPackage;
import org.eclipse.xtext.resource.impl.BinaryGrammarResourceFactoryImpl;
import org.eclipse.xtext.util.Modules2;
import org.lflang.LFRuntimeModule;
import org.lflang.LFStandaloneSetup;
import org.lflang.ide.LFIdeModule;
import org.lflang.ide.LFIdeSetup;

/**
 * Setup for Cli module registration.
 *
 * @author Soeren Domroes
 * @ingroup CLI
 */
public class LFCliSetup extends LFStandaloneSetup {

  protected ReportingBackend reporter;
  protected Io io;

  public LFCliSetup(ReportingBackend reporter, Io io) {
    this.reporter = reporter;
    this.io = io;
  }

  public static Injector doSetup() {
    // Check whether the current injector is already an injector created by this class.
    // If the injector was already created by the LFStandaloneSetup, it might not have the
    // LFIdeModule in it. If this is the case, reinitialize the injector and the EMF registration.
    if (injector == null) {
      injector = new LFIdeSetup().createInjectorAndDoEMFRegistration();
    } else if (injector.getExistingBinding(Key.get(LFIdeModule.class)) == null) {
      injector = new LFIdeSetup().createInjectorAndDoEMFRegistration();
    }
    return injector;
  }

  public Injector createInjector() {
    return Guice.createInjector(
        Modules2.mixin(new LFRuntimeModule(), new LFStandaloneModule(reporter, io)));
  }

  @Override
  public Injector createInjectorAndDoEMFRegistration() {
    // register default ePackages
    if (!Resource.Factory.Registry.INSTANCE.getExtensionToFactoryMap().containsKey("ecore"))
      Resource.Factory.Registry.INSTANCE
          .getExtensionToFactoryMap()
          .put("ecore", new EcoreResourceFactoryImpl());
    if (!Resource.Factory.Registry.INSTANCE.getExtensionToFactoryMap().containsKey("xmi"))
      Resource.Factory.Registry.INSTANCE
          .getExtensionToFactoryMap()
          .put("xmi", new XMIResourceFactoryImpl());
    if (!Resource.Factory.Registry.INSTANCE.getExtensionToFactoryMap().containsKey("xtextbin"))
      Resource.Factory.Registry.INSTANCE
          .getExtensionToFactoryMap()
          .put("xtextbin", new BinaryGrammarResourceFactoryImpl());
    if (!EPackage.Registry.INSTANCE.containsKey(XtextPackage.eNS_URI))
      EPackage.Registry.INSTANCE.put(XtextPackage.eNS_URI, XtextPackage.eINSTANCE);

    Injector injector = createInjector();
    register(injector);
    return injector;
  }
}
