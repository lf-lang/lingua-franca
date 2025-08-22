package org.lflang;

import com.google.inject.Inject;
import java.util.Map;
import java.util.stream.Collectors;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.naming.QualifiedName;
import org.eclipse.xtext.resource.EObjectDescription;
import org.eclipse.xtext.resource.IEObjectDescription;
import org.eclipse.xtext.resource.impl.DefaultResourceDescriptionStrategy;
import org.eclipse.xtext.scoping.impl.ImportUriResolver;
import org.eclipse.xtext.util.IAcceptor;
import org.lflang.lf.Model;
import org.lflang.util.ImportUtil;

/**
 * Resource description strategy designed to limit global scope to only those files that were
 * explicitly imported.
 *
 * Adapted from example provided by Itemis.
 *
 * @author Marten Lohstroh
 * @see "https://blogs.itemis.com/en/in-five-minutes-to-transitive-imports-within-a-dsl-with-xtext"
 * @ingroup Infrastructure
 */
public class LFResourceDescriptionStrategy extends DefaultResourceDescriptionStrategy {

  /** Key used in user data attached to description of a Model. */
  public static final String INCLUDES = "includes";

  /**
   * Delimiter used in the values associated with INCLUDES keys in the user-data descriptions of
   * Models.
   */
  public static final String DELIMITER = ",";

  @Inject private ImportUriResolver uriResolver;

  @Override
  public boolean createEObjectDescriptions(
      EObject eObject, IAcceptor<IEObjectDescription> acceptor) {
    if (eObject instanceof Model) {
      this.createEObjectDescriptionForModel((Model) eObject, acceptor);
      return true;
    } else {
      return super.createEObjectDescriptions(eObject, acceptor);
    }
  }

  /**
   * Build an index containing the strings of the URIs imported resources.
   *
   * <p>All the URIs are added to comma-separated string and stored under the key "includes" in the
   * userData map of the object description.
   */
  private void createEObjectDescriptionForModel(
      Model model, IAcceptor<IEObjectDescription> acceptor) {
    var uris =
        model.getImports().stream()
            .map(
                importObj -> {
                  return (importObj.getImportURI() != null)
                      ? importObj.getImportURI()
                      : ImportUtil.buildPackageURI(
                          importObj.getImportPackage(),
                          model.eResource()); // Use the resolved import string
                })
            .collect(Collectors.joining(DELIMITER));
    var userData = Map.of(INCLUDES, uris);
    QualifiedName qname = QualifiedName.create(model.eResource().getURI().toString());
    acceptor.accept(EObjectDescription.create(qname, model, userData));
  }
}
