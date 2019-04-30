# Getting Started with Lingua-Franca Xtext Development

## Installation of Eclipse and Xtext

* Install Eclipse from https://www.eclipse.org/downloads/
* Start Eclipse in a new workspace, just to be safe to not mess up my other work in any pre-existing workspace.
* Install xtext in that workspace by following instructions at: https://www.eclipse.org/Xtext/download.html
* See xtext documentation: https://www.eclipse.org/Xtext/documentation/index.html

## Get the Latest Lingua-Franca Editor from Github

* Checkout or pull lingua-franca repo: https://github.com/icyphy/lingua-franca.git
* Start Eclipse in a new workspace.
* In Eclipse, select File->Import->Team->Team Project Set
* Browse to lingua-franca/xtext and select the file LinguaFrancaProjectSet.psf
* Close the Eclipse welcome page (which obscures the projects)
* You should have five projects. Open org.icyphy.linguafranca

## Build and Run the Lingua-Franca editor

* Browse to src->org.icyphy->LinguaFranca.xtext (double click to open)
* You should see the latest Lingua-Franca grammar specification.
* Right click in that grammar file editor, select Run As->Generate Xtext Artifacts
* Right click on the first project in the PackageExplorer, org.icyphy.linguafranca and select Run As->Eclipse Application
* This should start a new Eclipse.

## Create Lingua-Franca Project

* In the new Eclipse, select File->New->Java Project
* Give the project a name, like "test"
* Open the project, and on the src directory, select New->File
* Give the new a name like "test.lf" (with .lf extension).
* IMPORTANT: A dialog appears: Do you want to convert 'test' to an Xtext Project? Say YES.
* Start typing in Lingua-Franca!
* When you save, generated code goes into ~/runtime-EclipseXtext/test (if "test" is your project name).

## Working on the Lingua-Franca compiler (package org.icyphy.linguafranca)
* The grammar is in src->org.icyphy->LinguaFranca.xtext
* The code generator for the Accessor target is in src->org.icyphy.generator->AccessorGenerator.xtend
* To add a code generator for a new target, edit src->org.icyphy.generator->LinguaFrancaGenerator.xtend

## Trouble Shooting
* Importing the Team Project Set from `LinguaFrancaProjectSet.psf` will result in Eclipse cloning a fresh copy of the lingua-franca repository; the clone will be located in `~/git/lingua-franca` (it will not use the clone that you obtained the `LinguaFrancaProjectSet.psf` from!) Note that, if you already have a clone in `~/git/lingua-franca`, that will simply be reused (not overwritten).
