/*
 * Generator base class for shared code between code generators.
 */
// The Lingua-Franca toolkit is licensed under the BSD 2-Clause License.
// See LICENSE.md file in the top repository directory.
package org.icyphy.generator

import java.io.BufferedReader
import java.io.IOException
import java.io.InputStreamReader
import java.util.HashMap
import java.util.Hashtable
import org.eclipse.emf.common.util.URI
import org.eclipse.emf.ecore.EObject
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.emf.ecore.resource.ResourceSet
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.Import
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.LinguaFrancaFactory
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.TimeOrValue
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.impl.ActionImpl
import org.icyphy.linguaFranca.impl.LinguaFrancaFactoryImpl
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Port
import java.util.List
import java.util.LinkedList
import org.icyphy.linguaFranca.TimeOrValue
import org.icyphy.linguaFranca.TimeUnit
import org.icyphy.linguaFranca.Timer

/**
 * Generator base class for shared code between code generators.
 * 
 * @author Edward A. Lee, Marten Lohstroh, Chris Gill
 */
class GeneratorBase {

    /** All code goes into this string buffer. */
    var code = new StringBuilder
    
    /** Map from builder to its current indentation. */
    var indentation = new HashMap<StringBuilder, String>()
    
    // Map from time units to an expression that can convert a number in
    // the specified time unit into nanoseconds. This expression may need
    // to have a suffix like 'LL' or 'L' appended to it, depending on the
    // target language, to ensure that the result is a 64-bit long.
    static public var timeUnitsToNs = #{
            TimeUnit.NSEC->1L,
            TimeUnit.NSECS->1L,
            TimeUnit.USEC->1000L,
            TimeUnit.USECS->1000L,
            TimeUnit.MSEC->1000000L,
            TimeUnit.MSECS->1000000L,
            TimeUnit.SEC->1000000000L,
            TimeUnit.SECS->1000000000L,
            TimeUnit.SECOND->1000000000L,
            TimeUnit.SECONDS->1000000000L,
            TimeUnit.MIN->60000000000L,
            TimeUnit.MINS->60000000000L,
            TimeUnit.MINUTE->60000000000L,
            TimeUnit.MINUTES->60000000000L,
            TimeUnit.HOUR->3600000000000L,
            TimeUnit.HOURS->3600000000000L,
            TimeUnit.DAY->86400000000000L,
            TimeUnit.DAYS->86400000000000L,
            TimeUnit.WEEK->604800000000000L, 
            TimeUnit.WEEKS->604800000000000L}
            
            
    def generateDelayReactor(Reaction reaction) {
    	
    	val lffi = LinguaFrancaFactory.eINSTANCE
    	val delayReactor = lffi.createReactor
    	delayReactor.name = "generatedDelay" // TODO: check unique name
    	
    	
		for (effect : reaction.effects) {
			println("effect variable: " + effect.variable)
			if (effect.variable instanceof Action) {
				throw new Error("No after on actions")
			}
			val v = (effect.variable as Port)
			
			val in = lffi.createInput
			in.name = v.name + "_in"
			in.type = v.type
			delayReactor.inputs.add(in)
			
			val out = lffi.createOutput
			out.name = v.name + "_out"
			out.type = v.type
			delayReactor.outputs.add(out)
			
			val state = lffi.createState
			state.name = v.name + "_state"
			state.type = v.type
			state.value = "0" // FIXME: default for types
			delayReactor.states.add(state)
			
		// original outputs into additional state variables
		// in the C code generator set state, schedule timer action
		}
		println("after: " + reaction.delay.time)
		// TODO: for after add a logical action and reaction with delay
		val action = lffi.createAction
		// FIXME: how to handle time now
		// This was before some the change to TimeOrValue
		// action.delay = timeInTargetLanguage(reaction.delay.time)
		// This is how deadlines are handled, but the compiler
		// complains about time being a String.
		
		// action.delay = resolveTime(reaction.delay.time, delayReactor)
		action.name = "act"
		delayReactor.actions.add(action)

		// println("time: " + action.delay)
		
		val r1 = lffi.createReaction
		for (i : delayReactor.inputs) {
			val vr = lffi.createVarRef
			vr.variable = i
			r1.triggers.add(vr)
		}
		val a = lffi.createVarRef
		a.variable = action
		r1.effects.add(a)
		r1.code = "{= printf(\"generatedDelay\"); =}"
		delayReactor.reactions.add(r1)
		
		
		// quite some to do as this already generates a null pointer exception					
		// create a reactor first
		// reactor.actions.add(action)
		val r2 = lffi.createReaction
		val vr = lffi.createVarRef
		vr.variable = action
		r2.triggers.add(vr)
		r2.code = "{= printf(\"reaction 2\"); =}"
		// TODO: effects
		
		delayReactor
	}
    

    ////////////////////////////////////////////
    //// Code generation functions to override for a concrete code generator.
    
    /** Generate code from the Lingua Franca model contained by the
     *  specified resource. This is the main entry point for code
     *  generation. This base class invokes generateReactor()
     *  for each contained reactor. If errors occur during generation,
     *  then a subsequent call to errorsOccurred() will return true.
     *  @param resource The resource containing the source code.
     *  @param fsa The file system access (used to write the result).
     *  @param context FIXME: What is this?
     *  @param importTable The mapping given by import statements.
     */
    def void doGenerate(
            Resource resource, 
            IFileSystemAccess2 fsa,
            IGeneratorContext context,
            Hashtable<String,String> importTable) {

        generatorErrorsOccurred = false
        
        this.resource = resource

        // Figure out the file name for the target code from the source file name.
        filename = extractFilename(resource.getURI.toString)


		// have an abstract function to generate the C code for the delay,
		// implemented in the C generator
		
		val genDelayReactor = new LinkedList<Reactor>
		val lffi = LinguaFrancaFactory.eINSTANCE
		
		// Iterate over reactors
		for (reactor : resource.allContents.toIterable.filter(Reactor)) {
			/*
			println("reactor: "+reactor)
			for (action : reactor.actions) {
				println("action: " + action)
			}
			*/
			// Iterate over reactions to find some with a delay ("after time")
			for (reaction : reactor.reactions) {
				println("reaction: " + reaction)
				if (reaction.delay !== null) {
					val dr = generateDelayReactor(reaction)
					genDelayReactor.add(dr)
					val createInst = lffi.createInstantiation
					createInst.name = "uniqueTODO" // TODO:
					createInst.reactorClass = dr
					reactor.instantiations.add(createInst)
					// remove the output of the reaction, add it in the timer reaction
				}
//				if (reaction.localDeadline !== null) {
//					println("deadline: " + reaction.localDeadline.time)
//				}
			}
	
		}
		for (reactor : genDelayReactor) {
			generateReactor(reactor, importTable)
		}

        var Instantiation mainDef = null

        
        // Recursively instantiate reactors from their definitions
        for (reactor : resource.allContents.toIterable.filter(Reactor)) {
            generateReactor(reactor, importTable)
            if (reactor.isMain) {
                // Creating an definition for the main reactor because there isn't one.
                mainDef = LinguaFrancaFactory.eINSTANCE.createInstantiation()
                mainDef.setName(reactor.name)
                mainDef.setReactorClass(reactor)
                this.main = new ReactorInstance(mainDef, null, this) // Recursively builds instances.
            }
        }

                
        
    }
    
    /** Return true if errors occurred in the last call to doGenerate().
     *  This will return true if any of the reportError methods was called.
     *  @return True if errors occurred.
     */
    def errorsOccurred() {
        return generatorErrorsOccurred;
    }
    
    /** Collect data in a reactor or composite definition.
     *  Subclasses should override this and be sure to call
     *  super.generateReactor(reactor, importTable).
     *  @param reactor The parsed reactor AST data structure.
     *  @param importTable Substitution table for class names (from import statements).
     */    
    def void generateReactor(Reactor reactor, Hashtable<String,String> importTable) {
                
        // Reset indentation, in case it has gotten messed up.
        indentation.put(code, "")
        
        // Print a comment identifying the generated code.
        prComment("Code generated by the Lingua Franca compiler for reactor " 
            + reactor.name
            + " in "
            + filename
        )
        
        // Special Timer and Action for startup and shutdown, if they occur.
        // Only one of each of these should be created even if multiple
        // reactions are triggered by them.
        var Timer timer = null
        var Action action = null
        
        if (reactor.reactions !== null) {
            for (Reaction reaction : reactor.reactions) {
                // If the reaction triggers include 'startup' or 'shutdown',
                // then create Timer and TimerInstance objects named 'startup'
                // or Action and ActionInstance objects named 'shutdown'.
                // Using a Timer for startup means that the target-specific
                // code generator doesn't have to do anything special to support this.
                // However, for 'shutdown', the target-specific code generator
                // needs to check all reaction instances for a shutdownActionInstance
                // and schedule that action before shutting down the program.
                // These get inserted into both the ECore model and the
                // instance model.
                var hasStartupTrigger = false;
                var hasShutdownTrigger = false;
                for (trigger : reaction.triggers) {
                    if (trigger.startup !== null) {
                        hasStartupTrigger = true
                        if (timer === null) {
                            // FIXME: Creating here a pretty incomplete EObject. Is that OK?
                            timer = LinguaFrancaFactory.eINSTANCE.createTimer()
                            timer.setName('startup')
                            reactor.timers.add(timer)
                        }
                    } else if (trigger.shutdown !== null) {
                        hasShutdownTrigger = true
                        if (action === null) {
                            // FIXME: Creating here a pretty incomplete EObject. Is that OK?
                            action = LinguaFrancaFactory.eINSTANCE.createAction()
                            action.setName('shutdown')
                            reactor.actions.add(action)
                        }
                    }
                }
                // If appropriate, add a VarRef to the triggers list of this
                // reaction for the startup timer or shutdown action.
                if (hasStartupTrigger) {
                    var variableReference = LinguaFrancaFactory.eINSTANCE.createVarRef()
                    variableReference.setVariable(timer)
                    reaction.triggers.add(variableReference)
                }
                if (hasShutdownTrigger) {
                    var variableReference = LinguaFrancaFactory.eINSTANCE.createVarRef()
                    variableReference.setVariable(action)
                    reaction.triggers.add(variableReference)
                }
            }
        }
    }

    /** If the argument starts with '{=', then remove it and the last two characters.
     *  @return The body without the code delimiter or the unmodified argument if it
     *   is not delimited.
     */
    static def String removeCodeDelimiter(String code) {
        if (code === null) {
            ""
        } else if (code.startsWith("{=")) {
            code.substring(2, code.length - 2).trim();
        } else {
            code
        }
    }

	// FIXME: comments
	def resolveTime(TimeOrValue timeOrValue, ReactorInstance instance) {
		var timeLiteral = '0'
		var unit = TimeUnit.NONE
		if (timeOrValue !== null) {
			if (timeOrValue.parameter !== null) {
				var resolved = instance.resolveParameter(timeOrValue.parameter)
				if (resolved === null) {
					throw new InternalError("Incorrect reference to parameter :" + timeOrValue.parameter.name);
				} else {
					timeLiteral = resolved.literalValue
					unit = TimeUnit.NONE
				}
			} else {
				timeLiteral = timeOrValue.time.toString
				unit = timeOrValue.unit
			}
		}
		return timeInTargetLanguage(timeLiteral, unit)
	}

    /** Given a representation of time that may possibly include units,
     *  return a string that the target language can recognize as a value.
     *  In this base class, if units are given, e.g. "msec", then
     *  we convert the units to upper case and return an expression
     *  of the form "MSEC(value)". Particular target generators will need
     *  to either define functions or macros for each possible time unit
     *  or override this method to return something acceptable to the
     *  target language.
     *  @param time Literal that represents a time value.
     *  @param unit Enum that denotes units
     *  @return A string, such as "MSEC(100)" for 100 milliseconds.
     */
    def timeInTargetLanguage(String timeLiteral, TimeUnit unit) { // FIXME: make this static?
    	if (unit != TimeUnit.NONE) {
    		unit.name() + '(' + timeLiteral + ')'
    	} else {
    		timeLiteral
    	}       
    }
    
    /** Return a string that the target language can recognize as a type
     *  for a time value. This base class returns "instant_t".
     *  Particular target generators will likely need to override
     *  this method to return something acceptable to the target language.
     *  @return The string "instant_t"
     */
    def timeTypeInTargetLanguage() {
        "instant_t"
    }

    ////////////////////////////////////////////
    //// Protected fields.

    /** The main (top-level) reactor instance. */
    protected ReactorInstance main 
    
    // The root filename for the main file containing the source code, without the .lf.
    protected var String filename
    
    // The file containing the main source code.
    protected var Resource resource
    
    // Indicator of whether generator errors occurred.
    protected var generatorErrorsOccurred = false

    ////////////////////////////////////////////
    //// Protected methods.
    
    /** Clear the buffer of generated code.
     */
    protected def clearCode() {
        code = new StringBuilder
    }
    
    /** Get the code produced so far.
     *  @return The code produced so far as a String.
     */
    protected def getCode() {
        code.toString()
    }
        
    /** Increase the indentation of the output code produced.
     */
    protected def indent() {
        indent(code)
    }
    
    /** Increase the indentation of the output code produced
     *  on the specified builder.
     *  @param The builder to indent.
     */
    protected def indent(StringBuilder builder) {
        var prefix = indentation.get(builder)
        if (prefix === null) {
            prefix = ""
        }
        val buffer = new StringBuffer(prefix)
        for (var i = 0; i < 4; i++) {
            buffer.append(' ');
        }
        indentation.put(builder, buffer.toString)
    }
    
    /** Open and parse an import given a URI relative to currentResource.
     *  @param currentResource The current resource.
     *  @param importedURIAsString The URI to import as a string.
     *  @return The resource specified by the URI or null if either
     *   the resource cannot be found or it is the same as the currentResource.
     */
    protected def openImport(Resource currentResource, Import importSpec) {
        val importedURIAsString = importSpec.name;
        val URI currentURI = currentResource?.getURI;
        val URI importedURI = URI?.createFileURI(importedURIAsString);
        val URI resolvedURI = importedURI?.resolve(currentURI);
        if (resolvedURI.equals(currentURI)) {
            reportError(importSpec, "Recursive imports are not permitted: " + importSpec.name)
            return currentResource
        } else {
            val ResourceSet currentResourceSet = currentResource?.resourceSet;
            try {
                return currentResourceSet?.getResource(resolvedURI, true);
            } catch (Exception ex) {
                reportError(importSpec, "Import not found: " + importSpec.name 
                    + ". Exception message: " + ex.message
                )
                return null
            }
        }
    }

    /** Append the specified text plus a final newline to the current
     *  code buffer.
     *  @param text The text to append.
     */
    protected def pr(String format, Object... args) {
        pr(code, if (args !== null && args.length > 0) String.format(format, args) else format)
    }
    
    /** Append the specified text plus a final newline to the specified
     *  code buffer.
     *  @param builder The code buffer.
     *  @param text The text to append.
     */
    protected def pr(StringBuilder builder, Object text) {
        // Handle multi-line text.
        var string = text.toString
        var indent = indentation.get(builder)
        if (indent === null) {
            indent = ""
        }
        if (string.contains("\n")) {
            // Replace all tabs with four spaces.
            string = string.replaceAll("\t", "    ")
            // Use two passes, first to find the minimum leading white space
            // in each line of the source text.
            var split = string.split("\n")
            var offset = Integer.MAX_VALUE
            var firstLine = true
            for (line : split) {
                // Skip the first line, which has white space stripped.
                if (firstLine) {
                    firstLine = false
                } else {
                    var numLeadingSpaces = line.indexOf(line.trim());
                    if (numLeadingSpaces < offset) {
                        offset = numLeadingSpaces
                    }
                }
            }
            // Now make a pass for each line, replacing the offset leading
            // spaces with the current indentation.
            firstLine = true
            for (line : split) {
                builder.append(indent)
                // Do not trim the first line
                if (firstLine) {
                    builder.append(line)
                    firstLine = false
                } else {
                    builder.append(line.substring(offset))
                }
                builder.append("\n")
            }
        } else {
            builder.append(indent)
            builder.append(text)
            builder.append("\n")
        }
    }
    
    /** Prints an indented block of text with the given begin and end markers,
     *  but only if the actions print any text at all.
     *  This is helpful to avoid the production of empty blocks.
     *  @param begin The prolog of the block.
     *  @param end The epilog of the block.
     *  @param actions Actions that print the interior of the block. 
     */
    protected def prBlock(String begin, String end, Runnable... actions) {
        val i = code.length
        indent()
        for (action : actions) {
            action.run()
        }
        unindent()
        if (i < code.length) {
            val inserted = code.substring(i, code.length)
            code.delete(i, code.length)
            pr(begin)
            code.append(inserted)
            pr(end)
        }
    }

    /** Print a comment to the generated file.
     *  Particular targets will need to override this if comments
     *  start with something other than '//'.
     *  @param comment The comment.
     */
    protected def prComment(String comment) {
        pr(code, '// ' + comment);
    }

    /** Read a text file in the classpath and return its contents as a string.
     *  @param filename The file name as a path relative to the classpath.
     *  @return The contents of the file as a String or null if the file cannot be opened.
     */
    protected def readFileInClasspath(String filename) throws IOException {
        var inputStream = this.class.getResourceAsStream(filename)
        
        if (inputStream === null) {
            return null
        }
        try {
            var resultStringBuilder = new StringBuilder()
            // The following reads a file relative to the classpath.
            // The file needs to be in the src directory.
            var reader = new BufferedReader(new InputStreamReader(inputStream))
            var line = ""
            while ((line = reader.readLine()) !== null) {
                resultStringBuilder.append(line).append("\n");
            }
            return resultStringBuilder.toString();
        } finally {
            inputStream.close
        }
    }

    /** Report an error.
     *  @param message The error message.
     */
    protected def reportError(String message) {
        System.err.println("ERROR: " + message)
    }
    
    /** Report an error on the specified parse tree object.
     *  @param object The parse tree object.
     *  @param message The error message.
     */
    protected def reportError(EObject object, String message) {
        generatorErrorsOccurred = true;
        // FIXME: All calls to this should also be checked by the validator (See LinguaFrancaValidator.xtend).
        // In case we are using a command-line tool, we report the line number.
        // The caller should not throw an exception so compilation can continue.
        var node = NodeModelUtils.getNode(object)
        System.err.println("ERROR: Line "
                    + node.getStartLine()
                       + ": "
                    + message)
        // Return a string that can be inserted into the generated code.
        "[[ERROR: " + message + "]]"
    }

    /** Report a warning on the specified parse tree object.
     *  @param object The parse tree object.
     *  @param message The error message.
     */
    protected def reportWarning(EObject object, String message) {
        // FIXME: All calls to this should also be checked by the validator (See LinguaFrancaValidator.xtend).
        // In case we are using a command-line tool, we report the line number.
        // The caller should not throw an exception so compilation can continue.
        var node = NodeModelUtils.getNode(object)
        System.err.println("WARNING: Line "
                    + node.getStartLine()
                       + ": "
                    + message)
        // Return an empty string that can be inserted into the generated code.
        ""
    }

    /** Reduce the indentation by one level for generated code
     *  in the default code buffer.
     */
    protected def unindent() {
        unindent(code)
    }
    
    /** Reduce the indentation by one level for generated code
     *  in the specified code buffer.
     */
    protected def unindent(StringBuilder builder) {
        var indent = indentation.get(builder)
        if (indent !== null) {
            val end = indent.length - 4;
            if (end < 0) {
                indent = ""
            } else {
                indent = indent.substring(0, end)
            }
            indentation.put(builder, indent)
        }
    }
    
    /** Given a representation of time that may possibly include units,
     *  return a string for the same amount of time
     *  in terms of the specified baseUnit. If the two units are the
     *  same, or if no time unit is given, return the number unmodified.
     *  @param time The source time.
     *  @param baseUnit The target unit.
     */
    protected def unitAdjustment(TimeOrValue timeOrValue, TimeUnit baseUnit) { // FIXME: likelt needs revision
        if (timeOrValue === null) {
            return '0'
        }
        var timeValue = timeOrValue.time
        var timeUnit = timeOrValue.unit
        
        if (timeOrValue.parameter !== null) {
        	timeUnit = timeOrValue.parameter.unit
        	if (timeOrValue.parameter.unit != TimeUnit.NONE) {
        		timeValue = timeOrValue.parameter.time
        	} else {
        		try {
        			timeValue = Integer.parseInt(timeOrValue.parameter.value)
        		} catch (NumberFormatException e) {
        			reportError(timeOrValue, "Invalid time value: " + timeOrValue)
        		}
        	}
        }
        
        if (timeUnit === TimeUnit.NONE || baseUnit.equals(timeUnit)) {
        	return timeValue
       	}
        // Convert time to nanoseconds, then divide by base scale.
        return ((timeValue * timeUnitsToNs.get(timeUnit)) / timeUnitsToNs.get(baseUnit)).toString
        
    }
    
    ////////////////////////////////////////////////////
    //// Private functions
    
    /** Extract a filename from a path. */
    private def extractFilename(String path) {
        var result = path
        if (path.startsWith('platform:')) {
            result = result.substring(9)
        }
        var lastSlash = result.lastIndexOf('/')
        if (lastSlash >= 0) {
            result = result.substring(lastSlash + 1)
        }
        if (result.endsWith('.lf')) {
            result = result.substring(0, result.length - 3)
        }
        return result
    }
    
    enum Mode {
          STANDALONE,
          INTEGRATED,
          UNDEFINED
    }
}


