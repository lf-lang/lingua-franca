# Lingua-Franca Maven
Maven is a build system that is build Lingua-Franca from the command line

# Resources
* [Building Xtext Languages with Maven and Gradle (2014)](https://www.eclipse.org/community/eclipse_newsletter/2014/august/article3.php) is a good overview.

* [Implementing Domain-Specific Languages with Xtext and Xtend (2nd Ed. 2016)](https://github.com/varmaprr/books/blob/master/Implementing%20Domain%20Specific%20Languages%20with%20Xtext%20and%20Xtend%20-%20Second%20Edition.pdf) - Chapter 11 "Continuous Integration" describes how to select to use the Xtext advanced project wizard to create a Maven pom.xml file.

* [Continuous Integration (with Maven)](https://www.eclipse.org/Xtext/documentation/350_continuous_integration.html) is a web page that is probably more up to date than the book above.
** https://github.com/xtext/maven-xtext-example - example code for the above.

* [Eclipse Maven Tycho XText Archetype](https://github.com/fuinorg/emt-xtext-archetype) Use Maven to generate a Xtext project that includes support for Maven and an Eclipse p2 directory.  This could be used to generate the stub files for the preexisting Lingua-Franca structure.  However, the generated files could be out of date because the archetype was started 6 years ago and last modified 10 on Aug 9, 2018.

# TLDR


The pom.xml files were created from  https://github.com/xtext/maven-xtext-example

Under macOS, install Maven with:
    https://github.com/xtext/maven-xtext-example

Below are various Maven commands.

## Cleaning
   mvn clean

## Running the tests
To run the tests, run

    mvn verify

To run just the tests in one directory with debugging (-X) and a stack trace (-e):

    mvn -X -e verify

If the problem occurs in the verify goal and not before, then to just run that goal, use:

    mvn -X -e verify -rf :org.icyphy.linguafranca.tests

To run one test:

    mvn -Dtest=TestCircle test

See [Running a Single Test](https://maven.apache.org/surefire/maven-surefire-plugin/examples/single-test.html) for how to use wild cards

Oddly, this does not work:

       mvn -Dtest=LinguaFrancaGeneratorTest surefire:test


## p2 Site
Eclipse uses a p2 site to provide features.  For example, there are Eclipse OSGI bundles created as part of Triquetrum at https://ptolemy.berkeley.edu/projects/chess/triq/p2/

To create the p2 site:
    mvn install

See org.icyphy.linguafranca.updatesite/target/repository/ for what would be uploaded to a website.


## How to run maven-xtext-example code

The maven-xtext-example code is what was used to create the pom.xml
files that are checked in to the lingua-franca repo.  Below are
instructions about how to run the maven-xtext-example code.

Under macOS, install Maven with:
    https://github.com/xtext/maven-xtext-example

Check out the repo:
    git clone https://github.com/xtext/maven-xtext-example

cd to the directory and run mvn:
    cd maven-text-example
    mvn

Sit back and wait while the world is downloaded.  The command will end with an error because we did not specify a goal.

Below are various problems and their solutions.

### mvn site
Try
    mvn site

#### java.lang.ClassNotFoundException: org.apache.maven.doxia.siterenderer.DocumentContent

``mvn site`` fails with:

    Caused by: java.lang.ClassNotFoundException: org.apache.maven.doxia.siterenderer.DocumentContent
        at org.codehaus.plexus.classworlds.strategy.SelfFirstStrategy.loadClass (SelfFirstStrategy.java:50)

Solution: [maven-site plugins 3.3 java.lang.ClassNotFoundException: org.apache.maven.doxia.siterenderer.DocumentContent](https://stackoverflow.com/questions/51091539/maven-site-plugins-3-3-java-lang-classnotfoundexception-org-apache-maven-doxia):

"Or we can specify maven-site-plugin to the latest 3.7.1 like:

    <plugin>
        <groupId>org.apache.maven.plugins</groupId>
        <artifactId>maven-site-plugin</artifactId>
        <version>3.7.1</version>
    </plugin>

in build part of pom"

#### Caused by: org.eclipse.aether.transfer.ArtifactNotFoundException: Could not find artifact org.eclipse.m2e:lifecycle-mapping

``mvn site`` fails with:
    Caused by: org.eclipse.aether.transfer.ArtifactNotFoundException: Could not find artifact org.eclipse.m2e:lifecycle-mapping


This is a known WONTFIX bug: [Bug 367870 - Superfluous warning due to no POM for for org.eclipse.m2e:lifecycle-mapping:jar:1.0.0 is missing, no dependency information available ](https://bugs.eclipse.org/bugs/show_bug.cgi?id=367870)

The [workaround](https://stackoverflow.com/questions/7905501/get-rid-of-pom-not-found-warning-for-org-eclipse-m2elifecycle-mapping) is to do
    cd ..
    mvn archetype:generate -DgroupId=org.eclipse.m2e -DartifactId=lifecycle-mapping -Dversion=1.0.0 -DarchetypeArtifactId=maven-archetype-mojo
    cd lifecycle-mapping
    mv install
    cd ../maven-xtext-example
    mvn site

#### [ERROR] Failed to execute goal on project my.mavenized.herolanguage.ide: Could not resolve dependencies for project my.mavenized.herolanguage:my.mavenized.herolanguage.ide:eclipse-plugin:1.0.0-SNAPSHOT: Could not find artifact my.mavenized.herolanguage:my.mavenized.herolanguage:jar:1.0.0-SNAPSHOT

While running ```mvn site```:

    [ERROR] Failed to execute goal on project my.mavenized.herolanguage.ide: Could not resolve dependencies for project my.mavenized.herolanguage:my.mavenized.herolanguage.ide:eclipse-plugin:1.0.0-SNAPSHOT: Could not find artifact my.mavenized.herolanguage:my.mavenized.herolanguage:jar:1.0.0-SNAPSHOT -> [Help 1]

Try running
    mvn install
    mvn site


After the above changes, it seems like the Maven build works in the example.


==Adding pom.xml files to linguafranca==

Created a branch:
    git clone https://github.com/icyphy/lingua-franca.git
    cd linguafranca
    git checkout -b cxb-maven-1


Copied the updatesite directory
    cp -r ../../maven-xtext-example/my.mavenized.herolanguage.updatesite .

Copied pom.xml files to lingua-franca/xtext and made substitutions
* "my.mavenized.herolanguage" -> "org.icyphy.linguafranca"
* "My Hero Language" -> "Lingua-Franca"

#### java.io.FileNotFoundException: /Users/cxh/src/lf/lingua-franca/xtext/org.icyphy.linguafranca/src/my/mavenized/GenerateHeroLanguage.mwe2 (No such file or directory)
After ``mvn install``:

   Caused by: java.io.FileNotFoundException: /Users/cxh/src/lf/lingua-franca/xtext/org.icyphy.linguafranca/src/my/mavenized/GenerateHeroLanguage.mwe2 (No such file or directory)

org.icyphy.linguafranca/pom.xml contains:

	    <argument>/${project.basedir}/src/my/mavenized/GenerateHeroLanguage.mwe2</argument>

The solution is to update the path to the mwe2 file:
	    <argument>/${project.basedir}/src/org/icyphy/GenerateLinguaFranca.mwe2</argument>


#### Access restriction: The type 'Test' is not API

``mvn install`` fails with:
    [ERROR] Failed to execute goal org.eclipse.tycho:tycho-compiler-plugin:1.4.0:compile (default-compile) on project org.icyphy.linguafranca.tests: Compilation failure: Compilation failure:
    [ERROR] /Users/cxh/src/lf/lingua-franca/xtext/org.icyphy.linguafranca.tests/xtend-gen/org/icyphy/tests/LinguaFrancaGeneratorTest.java:[34]
    [ERROR]         import org.junit.Test;
    [ERROR]                ^^^^^^^^^^^^^^
    [ERROR] Access restriction: The type 'Test' is not API (restriction on classpath entry '/Users/cxh/.m2/repository/.cache/tycho/org.junit-4.12.0.v201504281640.jar/junit.jar')

[Access restriction on class due to restriction on required library rt.jar?](https://stackoverflow.com/questions/860187/access-restriction-on-class-due-to-restriction-on-required-library-rt-jar) says:

"I have been getting this error too, but my project is built on the command line using Maven and the tycho compiler (it's a set of OSGi plugins). After masses of sifting through people having the same problem but fixing it in Eclipse rather than on the command line, I found a message on the Tycho developer forum that answered my question, using configuration in pom.xml to ignore the compiler warning about the access restriction:"

    <plugin>
        <groupId>org.eclipse.tycho</groupId>
        <artifactId>tycho-compiler-plugin</artifactId>
        <version>${tycho.version}</version>
        <configuration>
            <compilerArgument>-warn:+discouraged,forbidden</compilerArgument>
        </configuration>
     </plugin>

"More information can be found in the Tycho FAQ. This took me AGES to work out, so I figured I would assist anyone else trying to fix these access restriction errors from the command line by posting this answer."


The solution was to edit ``lingua-franca/xtext/org.icyphy.linguafranca.tests/pom.xml`` and ``lingua-franca/xtext/org.icyphy.linguafranca.ui.tests/pom.xml`` and add:

            <compilerArgument>-warn:+discouraged,forbidden</compilerArgument>

as above to the org.eclipse.tycho plugin section.  However, that does not work [Tycho build fails to compile ide project #272](https://github.com/eclipse/xtext-eclipse/issues/272) suggests:

            <compilerArgument>-err:-forbidden</compilerArgument>

However, that still fails.  Instead add
      <plugin>
        <!-- See https://github.com/eclipse/xtext-eclipse/issues/272 -->
	<groupId>org.eclipse.tycho</groupId>
	<artifactId>tycho-compiler-plugin</artifactId>
	<version>${tycho-version}</version>
	<configuration>
	  <compilerArgument>-err:-forbidden</compilerArgument>
	  <useProjectSettings>false</useProjectSettings>
	</configuration>
      </plugin>
to both files and then ``mvn verify`` will get further.

#### No tests found 
``mvn verify`` fails with:


    [ERROR] Failed to execute goal org.eclipse.tycho:tycho-surefire-plugin:1.4.0:test (default-test) on project org.icyphy.linguafranca.tests: No tests found. -> [Help 1]
    [ERROR] 
    [ERROR] To see the full stack trace of the errors, re-run Maven with the -e switch.

The first attempted fix was to edit ``lingua-franca/xtext/org.icyphy.linguafranca.tests/build.properties`` and remove the last line

    bin.excludes = **/*.xtend

However, that did not do it.

https://github.com/eclipse/Xpect/issues/165 suggests a naming problem and points to http://maven.apache.org/surefire/maven-surefire-plugin/examples/inclusion-exclusion.html, but the test are named

* src/org/icyphy/tests/LinguaFrancaGeneratorTest.xtend
* src/org/icyphy/tests/LinguaFrancaParsingTest.xtend

which matches

* src/my/mavenized/tests/HeroLanguageParsingTest.xtend

It seems that the solution is described in [junit: no tests found](https://stackoverflow.com/questions/22469480/junit-no-tests-found/22469564#22469564).



"I was getting this too:"

    junit.framework.AssertionFailedError: No tests found in ...

"Solved by renaming the test method from method1"

    @Test
    public void method1(){
        // Do some stuff...
    }

"to testMethod1"

    @Test
    public void testMethod1(){
        // Do some stuff...
    }

So I changed lingua-franca/xtext/org.icyphy.linguafranca.tests/src/org/icyphy/tests/LinguaFrancaParsingTest.xtend:

	def void CheckForTarget() {
to
	def void testTarget() {

and lingua-franca/xtext/org.icyphy.linguafranca.tests/src/org/icyphy/tests/LinguaFrancaGeneratorTest.xtend:

	def void checkCCompiler() {
do
	def void testCCompiler() {

That still did not do it.  The ``@Test`` annotation should mean that the test* name is not needed.


One thing is that it looks like the imports are different between what works and what does not.

    diff ~/src/lingua-franca/xtext/org.icyphy.linguafranca.tests/META-INF/MANIFEST.MF org.icyphy.linguafranca.tests/META-INF/
    10c10,15
    <  org.junit.jupiter.api;bundle-version="[5.0.0,6.0.0)",
    ---
    >  org.eclipse.core.runtime,
    >  org.eclipse.xtext.junit4,
    >  org.eclipse.xtext.xbase.lib;bundle-version="2.13.0",
    >  org.eclipse.xtend.lib,
    >  org.eclipse.ui.workbench;resolution:=optional,
    >  org.objectweb.asm;bundle-version="[7.0.0,7.1.0)";resolution:=optional,
    12,13c17,25
    <  org.eclipse.xtext.xbase.testing,
    <  org.eclipse.xtext.xbase.lib;bundle-version="2.14.0"
    ---
    >  org.eclipse.xtext.xbase.testing
    > Import-Package: org.apache.log4j,
    >  org.junit;version="4.5.0",
    >  org.junit.runner;version="4.5.0",
    >  org.junit.runner.manipulation;version="4.5.0",
    >  org.junit.runner.notification;version="4.5.0",
    >  org.junit.runners;version="4.5.0",
    >  org.junit.runners.model;version="4.5.0",
    >  org.hamcrest.core

This indicates that what works is for JUnit4 and what fails is for JUnit5.

Backing out various changes and then looking at [No tests found #165](https://github.com/eclipse/Xpect/issues/165) suggests updating o.i.t/pom.xml to use https://github.com/meysholdt/Xpect/blob/master/org.eclipse.xtext.example.domainmodel.xpect.tests/pom.xml

If I do this, then my copy of xtext/org.icyphy.linguafranca.tests/src/org/icyphy/tests/HeroLanguageParsingTest.xtend (below) will run, but the other tests (LinguaFrancaParsingTest.xtend and LinguaFrancaGeneratorTest.xtend) do not?

```
/*
 * generated by Xtext 2.11.0.RC2
 */
package org.icyphy.tests
//package my.mavenized.tests

import com.google.inject.Inject
// import my.mavenized.herolanguage.Heros
import org.eclipse.xtext.testing.InjectWith
import org.eclipse.xtext.testing.XtextRunner
import org.eclipse.xtext.testing.util.ParseHelper

import org.icyphy.linguaFranca.Model

import org.junit.Assert
import org.junit.Test
import org.junit.runner.RunWith
import org.icyphy.linguaFranca.Model

@RunWith(XtextRunner)
@InjectWith(LinguaFrancaInjectorProvider)
class HeroLanguageParsingTest {
	@Inject
	ParseHelper<Model> parseHelper
	
	@Test
	def void loadModel() {
                val result = parseHelper.parse('''
			hero superman can FLY
			hero iceman can ICE
		''')
          
		Assert.assertNull(result)
		//Assert.assertTrue(result.eResource.errors.isEmpty)
	}
}
```

Or do they?

##### Solution?
It looks like [What does the “default-test” stand for in the maven-surefire plugin](https://stackoverflow.com/questions/11935181/what-does-the-default-test-stand-for-in-the-maven-surefire-plugin) has the solution, which is to add

	<executions>
          <!-- https://stackoverflow.com/questions/11935181/what-does-the-default-test-stand-for-in-the-maven-surefire-plugin-->
        <execution>
                <id>default-test</id>
                <configuration>
                    <skip>true</skip>
                </configuration>
        </execution>
	</executions>

to the tycho-surefire-plugin section of org.icyphy.linguafranca.tests/pom.xml

