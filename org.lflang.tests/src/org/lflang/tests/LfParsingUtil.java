package org.lflang.tests;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.xtext.resource.XtextResource;
import org.eclipse.xtext.resource.XtextResourceSet;
import org.junit.jupiter.api.Assertions;

import org.lflang.LFStandaloneSetup;
import org.lflang.lf.Model;

import com.google.inject.Injector;

/**
 * @author Clément Fournier
 */
public class LfParsingUtil {

    /**
     * Parse the given file, asserts that there are no parsing errors.
     */
    public static Model parseValidModel(
        String fileName,
        String reformattedTestCase
    ) {
        Model resultingModel = parse(reformattedTestCase);
        checkValid(fileName, resultingModel);
        return resultingModel;
    }

    private static void checkValid(String fileName, Model resultingModel) {
        Assertions.assertNotNull(resultingModel);
        if (!resultingModel.eResource().getErrors().isEmpty()) {
            resultingModel.eResource().getErrors().forEach(System.err::println);
            Assertions.assertTrue(resultingModel.eResource().getErrors().isEmpty(),
                "Parsing errors in " + fileName);
        }
    }

    public static Model parseSourceAsIfInDirectory(
        Path directory,
        String sourceText
    ) {
        int num = 0;
        while (Files.exists(directory.resolve("file" + num + ".lf"))) {
            num++;
        }
        Path file = directory.resolve("file" + num + ".lf");
        try {
            Files.writeString(file, sourceText);
            Model resultingModel = parse(file);
            checkValid("file in " + directory, resultingModel);
            return resultingModel;
        } catch (IOException e) {
            throw new RuntimeException(e);
        } finally {
            try {
                Files.deleteIfExists(file);
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }

    }

    public static Model parse(String fileContents) {
        Path file = null;
        try {
            file = Files.createTempFile("lftests", ".lf");
            Files.writeString(file, fileContents);
            return parse(file);
        } catch (IOException e) {
            throw new RuntimeException(e);
        } finally {
            if (file != null) {
                try {
                    Files.deleteIfExists(file);
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }

    public static Model parse(Path file) {
        // Source: https://wiki.eclipse.org/Xtext/FAQ#How_do_I_load_my_model_in_a_standalone_Java_application_.3F
        Injector injector = new LFStandaloneSetup().createInjectorAndDoEMFRegistration();
        XtextResourceSet resourceSet = injector.getInstance(XtextResourceSet.class);
        resourceSet.addLoadOption(XtextResource.OPTION_RESOLVE_ALL, Boolean.TRUE);
        Resource resource = resourceSet.getResource(
            URI.createFileURI(file.toFile().getAbsolutePath()),
            true
        );
        return (Model) resource.getContents().get(0);
    }
}
