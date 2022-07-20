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
package org.lflang.diagram.synthesis.util;

import com.google.inject.Inject;

import de.cau.cs.kieler.klighd.krendering.Colors;
import de.cau.cs.kieler.klighd.krendering.KContainerRendering;
import de.cau.cs.kieler.klighd.krendering.KGridPlacementData;
import de.cau.cs.kieler.klighd.krendering.KRectangle;
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared;
import de.cau.cs.kieler.klighd.krendering.extensions.KContainerRenderingExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KRenderingExtensions;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.lang.ref.SoftReference;
import java.net.URL;
import java.util.HashMap;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.swt.graphics.ImageData;
import org.eclipse.swt.graphics.ImageLoader;
import org.eclipse.xtext.xbase.lib.Exceptions;
import org.eclipse.xtext.xbase.lib.Extension;
import org.eclipse.xtext.xbase.lib.StringExtensions;
import org.lflang.ASTUtils;
import org.lflang.AttributeUtils;
import org.lflang.diagram.synthesis.AbstractSynthesisExtensions;
import org.lflang.lf.ReactorDecl;

/**
 * Utility class to handle icons for reactors in Lingua Franca diagrams.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
@ViewSynthesisShared
public class ReactorIcons extends AbstractSynthesisExtensions {

    @Inject @Extension private KRenderingExtensions _kRenderingExtensions;
    @Inject @Extension private KContainerRenderingExtensions _kContainerRenderingExtensions;
    
    private static final ImageLoader LOADER = new ImageLoader();
    
    // memory-sensitive cache
    private static final HashMap<URL, SoftReference<ImageData>> CACHE = new HashMap<>();
    

    public void handleIcon(KContainerRendering rendering, ReactorDecl reactor, boolean collapsed) {
        if (!collapsed) {
            return;
        }
        URL iconLocation = locateIcon(reactor);
        if (iconLocation != null) {
            ImageData data = loadImage(iconLocation);
            if (data != null) {
                KRectangle figure = _kContainerRenderingExtensions.addRectangle(rendering);
                _kRenderingExtensions.setInvisible(figure, true);
                KGridPlacementData figurePlacement = _kRenderingExtensions.setGridPlacementData(figure, data.width, data.height);
                _kRenderingExtensions.to(
                      _kRenderingExtensions.from(
                              figurePlacement, 
                              _kRenderingExtensions.LEFT, 3, 0,
                              _kRenderingExtensions.TOP, 0, 0), 
                      _kRenderingExtensions.RIGHT, 3, 0, 
                      _kRenderingExtensions.BOTTOM, 3, 0);
                
                KRectangle icon = _kContainerRenderingExtensions.addRectangle(figure);
                _kRenderingExtensions.setInvisible(icon, true);
                _kContainerRenderingExtensions.addImage(icon, data);
                _kRenderingExtensions.setPointPlacementData(icon, 
                        _kRenderingExtensions.createKPosition(
                                _kRenderingExtensions.LEFT, 0, 0.5f, 
                                _kRenderingExtensions.TOP, 0, 0.5f), 
                        _kRenderingExtensions.H_CENTRAL, _kRenderingExtensions.V_CENTRAL, 0,
                        0, data.width, data.height);
            } else {
                var errorText = _kContainerRenderingExtensions.addText(rendering, "Icon not found!");
                _kRenderingExtensions.setForeground(errorText, Colors.RED);
                _kRenderingExtensions.setFontBold(errorText, true);
                _kRenderingExtensions.setSurroundingSpaceGrid(errorText, 8, 0);
            }
        }
    }

    private URL locateIcon(EObject eobj) {
        URL location = null;
        String iconPath = AttributeUtils.findAttributeByName(eobj, "icon");
        if (iconPath == null) { // Fallback to old syntax (in comment)
            iconPath = ASTUtils.findAnnotationInComments(eobj, "@icon");
        }
        if (!StringExtensions.isNullOrEmpty(iconPath)) {
            // Check if path is URL
            try {
                return new URL(iconPath);
            } catch (Exception e) {
                // nothing
            }
            // Check if path exists as is
            File path = new File(iconPath);
            if (path.exists()) {
                try {
                    return path.toURI().toURL();
                } catch (Exception e) {
                    // nothing
                }
            }
            // Check if path is relative to LF file
            URI eURI = eobj.eResource() != null ? eobj.eResource().getURI() : null;
            if (eURI != null) {
                java.net.URI sourceURI = null;
                try {
                    if (eURI.isFile()) {
                        sourceURI = new java.net.URI(eURI.toString());
                        sourceURI = new java.net.URI(sourceURI.getScheme(), null,
                            sourceURI.getPath().substring(0, sourceURI.getPath().lastIndexOf("/")), null);
                    } else if (eURI.isPlatformResource()) {
                        IResource iFile = ResourcesPlugin.getWorkspace().getRoot().findMember(eURI.toPlatformString(true));
                        sourceURI = iFile != null ? iFile.getRawLocation().toFile().getParentFile().toURI() : null; 
                    } else if (eURI.isPlatformPlugin()) {
                        // TODO support loading from plugin bundles?
                    }
                } catch (Exception e) {
                    // nothing
                }
                if (sourceURI != null) {
                    try {
                        location = sourceURI.resolve(path.toString()).toURL();
                    } catch (Exception e) {
                        // nothing
                    }
                    
                }
            }
            // TODO more variants based on package and library system in LF
        }
        return location;
    }

    private ImageData loadImage(final URL url) {
        try {
            synchronized (CACHE) {
                if (CACHE.containsKey(url)) {
                    ImageData img = CACHE.get(url).get();
                    if (img != null) {
                        return img;
                    } else {
                        CACHE.remove(url);
                    }
                }
            }
            synchronized (LOADER) {
                InputStream inStream = null;
                try {
                    inStream = url.openStream();
                    // TODO check for memory leak !!!
                    ImageData[] data = LOADER.load(inStream);
                    if (data != null && data.length > 0) {
                        ImageData img = data[0];
                        synchronized (CACHE) {
                            CACHE.put(url, new SoftReference<ImageData>(img));
                        }
                        return img;
                    }
                    return null;
                } finally {
                    if (inStream != null) {
                        inStream.close();
                    }
                }
            }
        } catch (IOException ex) {
            ex.printStackTrace();
            return null;
        }
    }
    
}
