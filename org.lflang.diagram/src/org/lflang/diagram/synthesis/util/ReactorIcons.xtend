/*************
* Copyright (c) 2020, Kiel University.
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
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/
package org.lflang.diagram.synthesis.util

import com.google.inject.Inject
import de.cau.cs.kieler.klighd.krendering.KContainerRendering
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared
import de.cau.cs.kieler.klighd.krendering.extensions.KContainerRenderingExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KRenderingExtensions
import java.io.File
import java.io.InputStream
import java.lang.ref.SoftReference
import java.net.URI
import java.net.URL
import org.eclipse.core.resources.ResourcesPlugin
import org.eclipse.emf.ecore.EObject
import org.eclipse.swt.graphics.ImageData
import org.eclipse.swt.graphics.ImageLoader
import org.lflang.ASTUtils
import org.lflang.diagram.synthesis.AbstractSynthesisExtensions
import org.lflang.lf.ReactorDecl

/**
 * Utility class to handle icons for reactors in Lingua Franca diagrams.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
@ViewSynthesisShared
class ReactorIcons extends AbstractSynthesisExtensions {

    @Inject extension KRenderingExtensions
    @Inject extension KContainerRenderingExtensions

    static val LOADER = new ImageLoader();
    static val CACHE = <URL, SoftReference<ImageData>>newHashMap // memory-sensitive cache

    def void handleIcon(KContainerRendering rendering, ReactorDecl reactor, boolean collapsed) {
        if (!collapsed) {
            return
        }
        val iconLocation = reactor.locateIcon()
        if (iconLocation !== null) {
            val data = iconLocation.loadImage()
            if (data !== null) {
                rendering.addRectangle() => [
                    invisible = true
                    setGridPlacementData(data.width, data.height) => [
                        from(LEFT, 3, 0, TOP, 0, 0).to(RIGHT, 3, 0, BOTTOM, 3, 0)
                    ]
                    addRectangle() => [
                        invisible = true
                        addImage(data)
                        setPointPlacementData(createKPosition(LEFT, 0, 0.5f, TOP, 0, 0.5f), H_CENTRAL, V_CENTRAL, 0, 0, data.width, data.height);
                    ]
                ]
            }
        }
    }

    private def URL locateIcon(EObject eobj) {
        var URL location = null
        val iconPath = ASTUtils.findAnnotationInComments(eobj, "@icon")
        if (!iconPath.nullOrEmpty) {
            // Check if path is URL
            try {
                return new URL(iconPath)
            } catch (Exception e) {
                // nothing
            }
            // Check if path exists as is
            val path = new File(iconPath)
            if (path.exists) {
                return path.toURI.toURL
            }
            // Check if path is relative to LF file
            val eURI = eobj.eResource?.URI
            if (eURI !== null) {
                var URI sourceURI = null
                try {
                    if (eURI.isFile) {
                        sourceURI = new URI(eURI.toString)
                        sourceURI = new URI(sourceURI.scheme, null,
                            sourceURI.path.substring(0, sourceURI.path.lastIndexOf('/')), null)
                    } else if (eURI.platformResource) {
                        val iFile = ResourcesPlugin.workspace.root.findMember(eURI.toPlatformString(true))
                        sourceURI = iFile?.rawLocation?.toFile.parentFile.toURI
                    } else if (eURI.platformPlugin) {
                        // TODO support loading from plugin bundles?
                    }
                } catch (Exception e) {
                    // nothing
                }
                if (sourceURI !== null) {
                    location = sourceURI.resolve(path.toString).toURL
                }
            }
            // TODO more variants based on package and library system in LF
        }
        return location
    }

    private def ImageData loadImage(URL url) {
        synchronized (CACHE) {
            if (CACHE.containsKey(url)) {
                val img = CACHE.get(url).get()
                if (img !== null) {
                    return img
                } else {
                    CACHE.remove(url)
                }
            }
        }
        synchronized (LOADER) {
            var InputStream inStream = null
            try {
                inStream = url.openStream
                // TODO check for memory leak !!!
                val data = LOADER.load(inStream)
                if (data !== null && data.length > 0) {
                    val img = data.get(0)
                    synchronized (CACHE) {
                        CACHE.put(url, new SoftReference(img))
                    }
                    return img
                }
                return null
            } finally {
                inStream?.close
            }
        }
    }
    
}
