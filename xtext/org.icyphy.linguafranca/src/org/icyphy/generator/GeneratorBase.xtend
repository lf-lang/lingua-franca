/*
 * Generator base class for shared code between code generators.
 */
// The Lingua-Franca toolkit is is licensed under the BSD 2-Clause License.
// See LICENSE.md file in the top repository directory.
package org.icyphy.generator

import java.io.BufferedReader
import java.io.IOException
import java.io.InputStreamReader
import java.text.NumberFormat
import java.text.ParseException
import java.util.HashMap
import java.util.HashSet
import java.util.Hashtable
import java.util.LinkedList
import org.eclipse.emf.common.util.URI
import org.eclipse.emf.ecore.EObject
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.emf.ecore.resource.ResourceSet
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.icyphy.linguaFranca.Import
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.LinguaFrancaFactory
import org.icyphy.linguaFranca.Port
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.Time
import org.icyphy.linguaFranca.VarRef

/**
 * Generator base class for shared code between code generators.
 * 
 * @author Edward A. Lee, Marten Lohstroh, Chris Gill
 */
class GeneratorBase {
	
	public static var HashMap<Pair<String, String>, Integer> nameRegistry = new HashMap();	
	
	/** Precedence graph of reaction instances. */
	var precedenceGraph = new ReactionGraph(this)
	
	/** Map from a reactor AST spec to properties of the reactor. */
	//protected var reactorToProperties = new HashMap<Reactor, ReactorInfo>()
	
	/** Map from reactor class name to the AST reactor spec defining that class. */
	//var classToReactor = new LinkedHashMap<String, Reactor>()

	/** All code goes into this string buffer. */
	var code = new StringBuilder
	
	/** Map from builder to its current indentation. */
	var indentation = new HashMap<StringBuilder, String>()
	
	/** The main (top-level) reactor instance. */
	protected ReactorInstance main 
	
	// The root filename for the main file containing the source code, without the .lf.
	protected var String _filename
	
	// The file containing the main source code.
	protected var Resource _resource
	
	// Indicator of whether generator errors occurred.
	protected var generatorErrorsOccurred = false
	
	// Map from time units to an expression that can convert a number in
	// the specified time unit into nanoseconds. This expression may need
	// to have a suffix like 'LL' or 'L' appended to it, depending on the
	// target language, to ensure that the result is a 64-bit long.
	static public var timeUnitsToNs = #{
			'nsec' -> 1L,
		 	'usec' -> 1000L,
			'msec'->1000000L,
			'sec'->1000000000L,
			'secs'->1000000000L,
			'minute'->60000000000L,
			'minutes'->60000000000L,
			'hour'->3600000000000L,
			'hours'->3600000000000L,
			'day'->86400000000000L,
			'days'->86400000000000L,
			'week'->604800000000000L, 
			'weeks'->604800000000000L}
	
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
        
		_resource = resource

		// Figure out the file name for the target code from the source file name.
		_filename = extractFilename(_resource.getURI.toString)

		var mainDef = null as Instantiation
		
		precedenceGraph.nodes.clear()
		
		// Recursively instantiate reactors from their definitions
		for (reactor : resource.allContents.toIterable.filter(Reactor)) {
			generateReactor(reactor, importTable)
			if (reactor.isMain) {
				// Creating an definition for the main reactor because there isn't one.
				mainDef = LinguaFrancaFactory.eINSTANCE.createInstantiation()
				mainDef.setName(reactor.name)
				mainDef.setReactorClass(reactor)
				this.main = new ReactorInstance(mainDef, null) // this call recursively builds instances
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
		    + _filename
		)
		
		//classToReactor.put(reactor.name, reactor)
		var info = ReactorInfo.get(reactor);
		// Create the object for storing reactor properties.
		//var properties = new ReactorInfo()
		//reactorToProperties.put(reactor, properties)

		// Record parameters.
//		if (reactor.parameters !== null) {
//			for (param : reactor.parameters) {
//				properties.nameToParam.put(param.name, param)
//			}
//		}
//		if (reactor.parameters !== null) {
//			for (param : reactor.parameters) {
//				properties.nameToParam.put(param.name, param)
//			}
//		}
//		
//		// Record inputs.
//		for (input: reactor.inputs) {
//			properties.nameToInput.put(input.name, input)
//		}
//		
//		// Record outputs.
//		for (output: reactor.outputs) {
//			properties.nameToOutput.put(output.name, output)
//		}
//		
//		// Record actions.
//		for (action: reactor.actions) {
//			if (action.getDelay() === null) {
//				action.setDelay("0")
//			}
//			properties.nameToAction.put(action.name, action)
//		}
//		
//		// Record timers.
//		for (timer: reactor.timers) {
//			properties.nameToTimer.put(timer.name, timer)
//			var timing = timer.timing
//			// Make sure every timing object has both an offset
//			// and a period by inserting default of 0.
//			var zeroTime = LinguaFrancaFactory.eINSTANCE.createTime()
//			zeroTime.setTime("0")
//			if (timing === null) {
//				timing = LinguaFrancaFactory.eINSTANCE.createTiming()
//				timing.setOffset(zeroTime)
//				timing.setPeriod(zeroTime)
//			} else if (timing.getPeriod === null) {
//				timing.setPeriod(zeroTime)
//			}
//			
//			properties.nameToTiming.put(timer.name, timing)
//		}
		
		// Record the reactions triggered by each trigger.
		for (reaction: reactor.reactions) {
			// Iterate over the reaction's triggers
			if (reaction.triggers !== null && reaction.triggers.length > 0) {
				for (trigger: reaction.triggers) {
//					// Check validity of the trigger.
//					if (properties.nameToInput.get(trigger) === null
//							&& getTiming(reactor, trigger) === null
//							&& getAction(reactor, trigger) === null) {
//                        reportError(reaction,
//                        		"Trigger '" + trigger + "' is neither an input, a timer, nor an action.")
//                    }
                    var reactionList = info.triggerToReactions.get(trigger)
                    if (reactionList === null) {
                    	reactionList = new LinkedList<Reaction>()
						info.triggerToReactions.put(trigger, reactionList)
                    }
                    reactionList.add(reaction)
				}	
			}
		}
//		// Record contained instances.
//		for (instance: reactor.instances) {
//			properties.nameToInstance.put(instance.name, instance)
//		}
		// Record (and check) connections.
		for (connection: reactor.connections) {
            // Record the source-destination pair.
            var destinations = info.sourceToDestinations.get(connection.leftPort)
            if (destinations === null) {
                destinations = new HashSet<VarRef>()
                info.sourceToDestinations.put(connection.leftPort.variable as Port, destinations)	
            }
            destinations.add(connection.rightPort)

			// Record inside connections to output ports  // to excluded direct feed through
            if (connection.rightPort.container === null) { // && connection.leftPort.instance !== null) {
                info.outputToContainedOutput.put(
                    connection.rightPort, connection.leftPort
                )
            }
            
            // Next, check the connection and report any errors.			
//			var split = connection.leftPort.split('\\.')
//			if (split.length === 1) {
//				// It is a local input port.
//				if (getInput(reactor, connection.leftPort) === null) {
//					reportError(connection,
//							"Left side is not an input port of this composite: " + connection.leftPort)
//				}
//			} else if (split.length === 2) {
//				// Form is reactorName.portName.
//				var instance = properties.nameToInstance.get(split.get(0))
//				if(instance === null) {
//					reportError(connection,
//							"No such instance: " + split.get(0))
//				} else {
//					var contained = getReactor(instance.reactorClass.name)
//					// Contained object may be imported, i.e. not a Lingua Franca object.
//					// Cannot check here.
//					if (contained !== null) {
//						var props = reactorToProperties.get(contained)
//						if(props !== null && props.nameToOutput.get(split.get(1)) === null) {
//							reportError(connection,
//									"No such output port: " + connection.leftPort)
//						}
//					}
//				}
//			} else {
//				reportError(connection, "Invalid port specification: " + connection.leftPort)
//			}
//
//
//			// Check the right port.
//			if (connection.rightPort.instance !== null) {
//				// FIXME: Looks like this will only work on level deep; should this not be recursive?
//				// FIXME: Also, we should synthesize reactions for data transfer across levels of hierarchy
//				
//				// If the destination is the input port of a reactor that itself contains other
//				// reactors, we need to add any input ports inside the destination that it is
//				// connected to. These will have the form actorInstanceName.containedActorInstanceName.portName.
//				var insideDestinations = ReactorInfo.get(connection.rightPort.instance.reactorClass).sourceToDestinations.get(connection.rightPort)
//				if (insideDestinations !== null) {
//					// There are inside connections. Record them.
//					for (insideDestination : insideDestinations) {
//						destinations.add(insideDestination)
//					}
//				}
//			}
//			split = connection.rightPort.split('\\.')
//			if (split.length === 1) {
//				// It is a local input port.
//				if (getOutput(reactor, connection.rightPort) === null) {
//					reportError(connection,
//							"Right side is not an output port of this reactor: " + connection.rightPort)
//				}
//			} else if (split.length === 2) {
//				// Form is reactorName.portName.
//				var instance = properties.nameToInstance.get(split.get(0))
//				if(instance === null) {
//					reportError(connection,
//							"No such instance: " + split.get(0))
//				} else {
//					var contained = getReactor(instance.reactorClass.name)
//					// Check that the input port in a contained reactor exists.
//					// Contained object may be imported, i.e. not a Lingua Franca object.
//					// Cannot check here.
//					if (contained !== null) {
//						var props = reactorToProperties.get(contained)
//						if(props !== null && props.nameToInput.get(split.get(1)) === null) {
//							reportError(connection,
//									"No such input port: " + connection.rightPort)
//						}
						// FIXME: Looks like this will only work on level deep; should this not be recursive?
                        // If the destination is the input port of a reactor that itself contains other
                        // reactors, we need to add any input ports inside the destination that it is
                        // connected to. These will have the form actorInstanceName.containedActorInstanceName.portName.
//					}
//				}
//			} else {
//				reportError(connection, "Invalid port specification: " + connection.rightPort)
//			}
		}
	}
	
	////////////////////////////////////////////
	//// Utility functions for generating code.
	
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

	/** If the argument starts with '{=', then remove it and the last two characters.
	 *  @return The body without the code delimiter or the unmodified argument if it
	 *   is not delimited.
	 */
	protected def String removeCodeDelimiter(String code) {
		if (code === null) {
			""
		} else if (code.startsWith("{=")) {
            code.substring(2, code.length - 2).trim();
        } else {
        	code
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
	
	/** Prints an indented block of text with the given begin and end markers,
	 * but only if the actions print any text at all.
	 * This is helpful to avoid the production of empty blocks.
	 * @param begin The prolog of the block.
	 * @param end The epilog of the block.
	 * @param actions Actions that print the interior of the block. 
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

	/** Given a representation of time that may possibly include units,
	 *  return a string for the same amount of time
	 *  in terms of the specified baseUnit. If the two units are the
	 *  same, or if no time unit is given, return the number unmodified.
	 *  @param time The source time.
	 *  @param baseUnit The target unit.
	 */
	protected def unitAdjustment(Time time, String baseUnit) {
		if (time === null || time.time === null) {
			return '0'
		}
		if (time.unit === null || baseUnit.equals(time.unit)) {
			return time.time
		}
		try {
			var nf = NumberFormat.getInstance();
			// The following will try to return a Long, and if that fails, will return a Double.
			var parsed = nf.parse(time.time)
			var unitScale = timeUnitsToNs.get(time.unit)
			if (unitScale === null) {
				// Invalid unit specification.
				return reportError(time, "Invalid unit '" + time.unit + "'. Should be one of: " + timeUnitsToNs.keySet)
			}
			var baseScale = timeUnitsToNs.get(baseUnit)
			if (baseScale === null) {
				// This is an error in the target code generator, not in the source code.
				throw new Exception("Invalid target base unit: " + baseUnit + ". Should be one of: " + timeUnitsToNs.keySet)				
			}
			// Handle Double and Long separately.
			if (parsed instanceof Long) {
				// First convert the number to units of nanoseconds.
				var numberInNs = parsed.longValue() * unitScale
				// Then convert to baseUnits.
				var result = numberInNs / baseScale
				return result.toString()
			} else {
				// Assume its is a Double.
				// First convert the number to units of nanoseconds.
				var numberInNs = parsed.doubleValue() * unitScale
				// Then convert to baseUnits.
				var result = numberInNs / baseScale
				return result.toString()
			}
		} catch (ParseException ex) {
			return reportError(time, "Failed to parse number '" + time.time + "'. " + ex)
		}
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


