/*
 * Generator base class for shared code between code generators.
 */
package org.icyphy.generator

import java.text.NumberFormat
import java.text.ParseException
import java.util.HashMap
import java.util.Hashtable
import java.util.LinkedHashMap
import java.util.LinkedList
import org.eclipse.emf.ecore.EObject
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.Component
import org.icyphy.linguaFranca.LinguaFrancaFactory
import org.icyphy.linguaFranca.Param
import org.icyphy.linguaFranca.Time
import org.icyphy.linguaFranca.Timing

/**
 * Generator base class for shared code between code generators.
 * 
 * @author Edward A. Lee, Marten Lohstroh, Chris Gill
 */
class GeneratorBase {	
	// All code goes into this string buffer.
	var code = new StringBuilder
	
	// Current indentation.
	var indentation = ""
	
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
	
	// Map from action name to Action object.
	var actions = new HashMap<String,Action>()
	
	// List of parameters.		
	var parameters = new LinkedList<Param>()

	// Map from timer name to Timing object.
	var timers = new LinkedHashMap<String,Timing>()
	
	////////////////////////////////////////////
	//// Code generation functions to override for a concrete code generator.
	
	/** Collect data in a reactor or composite definition.
	 *  Subclasses should override this and be sure to call
	 *  super.generateComponent(component, importTable).
	 *  @param component The parsed component data structure.
	 *  @param importTable Substitution table for class names (from import statements).
	 */	
	def void generateComponent(Component component, Hashtable<String,String> importTable) {
		actions.clear()		// Reset map from action name to action object.
		parameters.clear()  // Reset set of parameters.
		timers.clear()      // Reset map of timer names to timer properties.

		// Record parameters.
		if (component.componentBody.parameters !== null) {
			for (param : component.componentBody.parameters.params) {
				parameters.add(param)
			}
		}
		
		// Record actions.
		for (action: component.componentBody.actions) {
			if (action.getDelay() === null) {
				action.setDelay("0")
			}
			actions.put(action.name, action)
		}
		
		// Record timers.
		for (timer: component.componentBody.timers) {
			var timing = timer.timing
			// Make sure every timing object has both an offset
			// and a period by inserting default of 0.
			var zeroTime = LinguaFrancaFactory.eINSTANCE.createTime()
			zeroTime.setTime("0")
			if (timing === null) {
				timing = LinguaFrancaFactory.eINSTANCE.createTiming()
				timing.setOffset(zeroTime)
				timing.setPeriod(zeroTime)
			} else if (timing.getPeriod === null) {
				timing.setPeriod(zeroTime)
			}
			
			timers.put(timer.name, timing)
		}
	}
	
	////////////////////////////////////////////
	//// Utility functions for generating code.
	
	/** Clear the buffer of generated code.
	 */
	protected def clearCode() {
		code = new StringBuilder
	}
	
	/** Return the map of action names to Actions.
	 *  @return The actions.
	 */
	protected def getActions() {
		actions;
	}
	
	
	/** Get the code produced so far.
	 *  @return The code produced so far as a String.
	 */
	protected def getCode() {
		code.toString()
	}
	
	/** Return the list of parameters.
	 *  @return The list of parameters.
	 */
	protected def getParameters() {
		parameters;
	}

	/** Return a set of timer names.
	 */
	protected def getTimerNames() {
		timers.keySet()
	}
	
	/** Get the timing of the timer with the specified name.
	 *  Return a Timing object or null if there is no timer with the specified name.
	 */
	protected def getTiming(String name) {
		timers.get(name)
	}
	
	/** Increase the indentation of the output code produced.
	 */
	protected def indent() {
		val buffer = new StringBuffer(indentation)
		for (var i = 0; i < 4; i++) {
			buffer.append(' ');
		}
		indentation = buffer.toString
	}
	
	/** Append the specified text plus a final newline to the current
	 *  code buffer.
	 *  @param text The text to append.
	 */
	protected def pr(Object text) {
		// Handle multi-line text.
		var string = text.toString
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
				code.append(indentation)
				// Do not trim the first line
				if (firstLine) {
					code.append(line)
					firstLine = false
				} else {
					code.append(line.substring(offset))
				}
				code.append("\n")
			}
		} else {
			code.append(indentation)
			code.append(text)
			code.append("\n")
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
	
	/** Report an error on the specified parse tree object.
	 *  @param object The parse tree object.
	 *  @param message The error message.
	 */
	protected def reportError(EObject object, String message) {
		// FIXME: All calls to this should also be checked by the validator (See LinguaFrancaValidator.xtend).
        // In case we are using a command-line tool, we report the line number.
        // The caller should not throw an exception so compilation can continue.
        var node = NodeModelUtils.getNode(object)
        System.err.println("Line "
            		+ node.getStartLine()
               		+ ": "
            		+ message)
        // Return a string that can be inserted into the generated code.
        "[[ERROR: " + message + "]]"
	}

	/** Reduce the indentation by one level for generated code.
	 */
	protected def unindent() {
		val end = indentation.length - 4;
		if (end < 0) {
			indentation = ""
		} else {
			indentation = indentation.substring(0, end)
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
}
