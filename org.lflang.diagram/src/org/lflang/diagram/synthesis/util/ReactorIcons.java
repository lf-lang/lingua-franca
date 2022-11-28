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

//import org.eclipse.swt.graphics.ImageData;
//import org.eclipse.swt.graphics.ImageLoader;
import org.eclipse.xtext.xbase.lib.Extension;

import org.lflang.AttributeUtils;
import org.lflang.diagram.synthesis.AbstractSynthesisExtensions;
import org.lflang.lf.ReactorDecl;
import org.lflang.util.FileUtil;

import com.google.inject.Inject;

import de.cau.cs.kieler.klighd.krendering.KContainerRendering;
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared;
import de.cau.cs.kieler.klighd.krendering.extensions.KContainerRenderingExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KRenderingExtensions;

/**
 * Utility class to handle icons for reactors in Lingua Franca diagrams.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
@ViewSynthesisShared
public class ReactorIcons extends AbstractSynthesisExtensions {

    @Inject @Extension private KRenderingExtensions _kRenderingExtensions;
    @Inject @Extension private KContainerRenderingExtensions _kContainerRenderingExtensions;
    
//    private static final ImageLoader LOADER = new ImageLoader();
    
    // Image cache during synthesis
//    private final HashMap<java.net.URI, ImageData> cache = new HashMap<>();
    
    // Error message
    private String error = null;

    public void handleIcon(KContainerRendering rendering, ReactorDecl reactor, boolean collapsed) {
        if (!collapsed) {
            return;
        }
        
        // Reset error
        error = null;
        
        // Get annotation
        var iconPath = AttributeUtils.getIconPath(reactor);
        if (iconPath != null && !iconPath.isEmpty()) {
            var iconLocation = FileUtil.locateFile(iconPath, reactor.eResource());
            if (iconLocation == null) {
                error = "Cannot find given icon file.";
            } else {
                /*
                 * This code was disabled because it cannot be compiled for the language server with Gradle.
                 * As soon as the Klighd API is extended to support URI-based images in both Eclipse and VSCode,
                 * this code should be reactivated and adapted.
                 * See: https://github.com/kieler/KLighD/issues/146
                 */
//                ImageData data = loadImage(iconLocation);
//                if (data != null) {
//                    KRectangle figure = _kContainerRenderingExtensions.addRectangle(rendering);
//                    _kRenderingExtensions.setInvisible(figure, true);
//                    KGridPlacementData figurePlacement = _kRenderingExtensions.setGridPlacementData(figure, data.width, data.height);
//                    _kRenderingExtensions.to(
//                            _kRenderingExtensions.from(
//                                    figurePlacement, 
//                                    _kRenderingExtensions.LEFT, 3, 0,
//                                    _kRenderingExtensions.TOP, 0, 0), 
//                            _kRenderingExtensions.RIGHT, 3, 0, 
//                            _kRenderingExtensions.BOTTOM, 3, 0);
//                    
//                    KRectangle icon = _kContainerRenderingExtensions.addRectangle(figure);
//                    _kRenderingExtensions.setInvisible(icon, true);
//                    _kContainerRenderingExtensions.addImage(icon, data);
//                    _kRenderingExtensions.setPointPlacementData(icon, 
//                            _kRenderingExtensions.createKPosition(
//                                    _kRenderingExtensions.LEFT, 0, 0.5f, 
//                                    _kRenderingExtensions.TOP, 0, 0.5f), 
//                            _kRenderingExtensions.H_CENTRAL, _kRenderingExtensions.V_CENTRAL, 0,
//                            0, data.width, data.height);
//                }
//                if (error != null) {
//                    var errorText = _kContainerRenderingExtensions.addText(rendering, "Icon not found!\n"+error);
//                    _kRenderingExtensions.setForeground(errorText, Colors.RED);
//                    _kRenderingExtensions.setFontBold(errorText, true);
//                    _kRenderingExtensions.setSurroundingSpaceGrid(errorText, 8, 0);
//                }
            }
        }
    }

//    private ImageData loadImage(final java.net.URI uri) {
//        try {
//            if (cache.containsKey(uri)) {
//                return cache.get(uri);
//            }
//            synchronized (LOADER) {
//                InputStream inStream = null;
//                try {
//                    inStream = uri.toURL().openStream();
//                    ImageData[] data = LOADER.load(inStream);
//                    if (data != null && data.length > 0) {
//                        ImageData img = data[0];
//                        cache.put(uri, img);
//                        return img;
//                    } else {
//                        error = "Could not load icon image.";
//                        return null;
//                    }
//                } finally {
//                    if (inStream != null) {
//                        inStream.close();
//                    }
//                }
//            }
//        } catch (Exception ex) {
//            ex.printStackTrace();
//            error = "Could not load icon image.";
//            return null;
//        }
//    }
    
}
