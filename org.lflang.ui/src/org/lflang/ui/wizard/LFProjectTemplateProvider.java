package org.lflang.ui.wizard;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.util.List;
import java.util.stream.Collectors;

import org.eclipse.xtext.ui.XtextProjectHelper;
import org.eclipse.xtext.ui.util.PluginProjectFactory;
import org.eclipse.xtext.ui.util.ProjectFactory;
import org.eclipse.xtext.ui.wizard.template.AbstractProjectTemplate;
import org.eclipse.xtext.ui.wizard.template.GroupTemplateVariable;
import org.eclipse.xtext.ui.wizard.template.IProjectGenerator;
import org.eclipse.xtext.ui.wizard.template.IProjectTemplateProvider;
import org.eclipse.xtext.ui.wizard.template.ProjectTemplate;
import org.eclipse.xtext.ui.wizard.template.StringSelectionTemplateVariable;

/**
 * Create a list with all project templates to be shown in the template new project wizard.
 * 
 * Each template is able to generate one or more projects. Each project can be configured such that any number of files are included.
 */
@SuppressWarnings("restriction")
class LFProjectTemplateProvider implements IProjectTemplateProvider {
	@Override
	   public AbstractProjectTemplate[] getProjectTemplates() {
           return new AbstractProjectTemplate[] { new FederatedProject(),
                   new HelloWorldProject(), new InteractiveProject(),
                   new ParallelProject(), new PipelineProject(),
                   new ReflexGameProject(), new WebServerProject()};
       }
}

@SuppressWarnings("restriction")
abstract class LFProjectTemplate extends AbstractProjectTemplate {
    
    ProjectFactory setup(List<String> folders) {
        var proj = new PluginProjectFactory();
        proj.setProjectName(this.getProjectInfo().getProjectName());
        proj.setLocation(this.getProjectInfo().getLocationPath());
        proj.addProjectNatures(XtextProjectHelper.NATURE_ID);
        proj.addBuilderIds(XtextProjectHelper.BUILDER_ID);
        proj.addFolders(folders);
        return proj;
    }
    
    String readFromFile(String target, String fileName) {
        var stream = this.getClass().getResourceAsStream("templates/" + target + "/" + fileName);
        var str = "";
        if (stream != null) {
            str = new BufferedReader(
                new InputStreamReader(stream)
            ).lines().collect(Collectors.joining("\n"));
        } else {
            throw new RuntimeException("Unable to open template for '" + fileName + "'");
        }
        return str;
    }
}

@SuppressWarnings("restriction")
@ProjectTemplate(label="Parallel", icon="project_template.png", description="<p><b>Parallel</b></p><p>A simple" + 
        " fork-join pattern that exploits parallelism.</p>")
final class PipelineProject extends LFProjectTemplate {

   @Override
   public void generateProjects(IProjectGenerator generator) {
        var proj = setup(List.of("src"));
        var fileName = "src/Pipeline.lf";
        this.addFile(proj, fileName, readFromFile("c", fileName));
        generator.generate(proj);
    }
}

@ProjectTemplate(label="Federated", icon="project_template.png", description="<p><b>Federated</b></p>" +
        "<p>A federated \"Hello World\" program.</p>")
@SuppressWarnings("restriction")
final class FederatedProject extends LFProjectTemplate {
    
    @Override
    public void generateProjects(IProjectGenerator generator) {
        var proj = setup(List.of("src"));
        var fileName = "src/FederatedHelloWorld.lf";
        this.addFile(proj, fileName, readFromFile("c", fileName));
        generator.generate(proj);            
    }
}

@ProjectTemplate(label="Parallel", icon="project_template.png", description="<p><b>Parallel</b></p>" +
"<p>A simple fork-join pattern that exploits parallelism.</p>")
@SuppressWarnings("restriction")
final class ParallelProject extends LFProjectTemplate {
    
    @Override
    public void generateProjects(IProjectGenerator generator) {
        var proj = setup(List.of("src"));
        var fileName = "src/Parallel.lf";
        this.addFile(proj, fileName, readFromFile("c", fileName));
        generator.generate(proj);
    }
}

@ProjectTemplate(label="Hello World", icon="project_template.png", description="<p><b>Hello World</b></p>" +
"<p>Print \"Hello world!\" in a target language of choice.</p>")
@SuppressWarnings("restriction")
final class HelloWorldProject extends LFProjectTemplate {
    GroupTemplateVariable config = group("Configuration");
    // FIXME: draw from Target enum instead
    StringSelectionTemplateVariable target = combo("Target:", new String[] {"C", "C++", "Python", "TypeScript"}, "The target language to compile down to", config);
    
    @Override
    public void generateProjects(IProjectGenerator generator) {
        var proj = setup(List.of("src"));
        var fileName = "src/HelloWorld.lf";
        switch (target.getValue()) {
            case "C++":
                addFile(proj, fileName, readFromFile("cpp", fileName));
                break;
            case "C":
                addFile(proj, fileName, readFromFile("c", fileName));
                break;
            case "Python":
                addFile(proj, fileName, readFromFile("py", fileName));
                break;
            case "TypeScript":
                addFile(proj, fileName, readFromFile("ts", fileName));
                break;
        }
        generator.generate(proj);
    }
}

@ProjectTemplate(label="Interactive", icon="project_template.png", description="<p><b>Interactive</b></p>" +
"<p>Simulate sensor input through key strokes.</p>")
@SuppressWarnings("restriction")
final class InteractiveProject extends LFProjectTemplate {
    
    @Override
    public void generateProjects(IProjectGenerator generator) {
        var proj = setup(List.of("src", "src/include"));
        var fileName = "src/Interactive.lf";        
        this.addFile(proj, fileName, readFromFile("c", fileName));
        var cmakeFile = "src/include/ncurses-cmake-extension.txt";
        this.addFile(proj, cmakeFile, readFromFile("c", cmakeFile));
        generator.generate(proj);
    }
}

@ProjectTemplate(label="WebServer", icon="project_template.png", description="<p><b>Web Server</b></p>" +
"<p>A simple web server implemented using TypeScript.</p>")
@SuppressWarnings("restriction")
final class WebServerProject extends LFProjectTemplate {
    
    @Override
    public void generateProjects(IProjectGenerator generator) {
        var proj = setup(List.of("src"));
        var fileName = "src/WebServer.lf";        
        this.addFile(proj, fileName, readFromFile("ts", fileName));
        generator.generate(proj);
    }
}

@ProjectTemplate(label="ReflexGame", icon="project_template.png", description="<p><b>ReflexGame</b></p>" +
"<p>A simple reflex game.</p>")
@SuppressWarnings("restriction")
final class ReflexGameProject extends LFProjectTemplate {
    GroupTemplateVariable config = group("Configuration");
    // FIXME: draw from Target enum instead
    StringSelectionTemplateVariable target = combo("Target:",
            new String[] { "C" },
            "The target language to compile down to", config);

    @Override
    public void generateProjects(IProjectGenerator generator) {
        var proj = setup(List.of("src"));
        var fileName = "src/ReflexGame.lf";
        if (target.getValue().equals("C")) {
            this.addFile(proj, fileName, readFromFile("c", fileName));
        }
        generator.generate(proj);
    }
}
