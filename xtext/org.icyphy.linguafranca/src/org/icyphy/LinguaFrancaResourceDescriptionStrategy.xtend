/* Custom resource description strategy for Lingua Franca. */

package org.icyphy

import com.google.inject.Inject
import java.util.HashMap
import org.eclipse.xtext.naming.QualifiedName
import org.eclipse.xtext.resource.EObjectDescription
import org.eclipse.xtext.resource.IEObjectDescription
import org.eclipse.xtext.resource.impl.DefaultResourceDescriptionStrategy
import org.eclipse.xtext.scoping.impl.ImportUriResolver
import org.eclipse.xtext.util.IAcceptor
import org.eclipse.emf.ecore.EObject
import org.icyphy.linguaFranca.Model

/** Resource description strategy designed limit global scope
 *  to only those files that were explicitly imported.
 */
class LinguaFrancaResourceDescriptionStrategy extends DefaultResourceDescriptionStrategy {
    // NOTE: Adapted from example provided by Itemis.
    // https://blogs.itemis.com/en/in-five-minutes-to-transitive-imports-within-a-dsl-with-xtext 
    
    public static final String INCLUDES = "includes"
    public static final String DELIMITER = ','

    @Inject
    ImportUriResolver uriResolver

    /** Handle eObject instances of type "Model" separately. */
    override createEObjectDescriptions(EObject eObject,
        IAcceptor<IEObjectDescription> acceptor) {
        if (eObject instanceof Model) {
            this.createEObjectDescriptionForModel(eObject, acceptor)
            return true
        } else {
            super.createEObjectDescriptions(eObject, acceptor)
        }
    }

    /** Build an index containing the strings of the URIs imported resources.
     *  All the URIs are added to comma-separated string and stored under the 
     *  key "includes" in the userData map of the object description.
     **/
    def void createEObjectDescriptionForModel(Model model,
        IAcceptor<IEObjectDescription> acceptor) {
        val uris = newArrayList()
        model.imports.forEach[uris.add(uriResolver.apply(it))]
        val userData = new HashMap<String, String>
        userData.put(INCLUDES, uris.join(DELIMITER))
        acceptor.accept(
            EObjectDescription.create(
                QualifiedName.create(model.eResource.URI.toString), model,
                userData))
    }
}
