# Getting Started with Lingua-Franca Xtext Development

## Installation of Eclipse and Xtext

* Install Eclipse from FIXME.
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
* 
