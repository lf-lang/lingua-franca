/*
 * Generator base class for shared code between code generators.
 */
package org.icyphy.generator

import java.util.Collection
import java.util.HashMap
import java.util.HashSet
import java.util.Hashtable
import org.icyphy.linguaFranca.Component
import org.icyphy.linguaFranca.LinguaFrancaFactory
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
	
	// Time units supported by all code generators, which includes
	// milliseconds and coarser.
	static var baseTimeUnits = #{'ms', 'msec', 's', 'sec', 'minute', 'minutes', 'hour', 'hours', 'day', 'days', 'week', 'weeks'}
	
	// Time units supported by an instance of the code generator.
	// This includes at least the base time units above, but it can
	// be augmented by subclasses using addTimeUnits().
	var timeUnits = new HashSet<String>(baseTimeUnits)
	
	// Map from timer name to Timing object.
	var timers = new HashMap<String,Timing>()
	
	////////////////////////////////////////////
	//// Code generation functions to override for a concrete code generator.

	/** Add the specified collection of time units to the time
	 *  units supported by this code generator.
	 *  @param units A collection of time units.
	 */
	def addTimeUnits(Collection<String> units) {
		for (unit: units) {
			timeUnits.add(unit)
		}
	}
	/** Collect data in a reactor or composite definition.
	 *  Subclasses should override this and be sure to call
	 *  super.generateComponent(component, importTable).
	 *  @param component The parsed component data structure.
	 *  @param importTable Substitution table for class names (from import statements).
	 */	
	def void generateComponent(Component component, Hashtable<String,String> importTable) {
		timers.clear()      // Reset map of timer names to timer properties.

		// Record timers.
		var count = 0;
		
		for (timer: component.componentBody.timers) {
			count++
			var timing = timer.timing
			// Make sure every timing object has both an offset
			// and a period by inserting default of 0.
			if (timing === null) {
				timing = LinguaFrancaFactory.eINSTANCE.createTiming()
				timing.setOffset("0")
				timing.setPeriod("0")
			} else if (timing.getPeriod === null) {
				timing.setPeriod("0")
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
	
	/** Get the code produced so far.
	 *  @return The code produced so far as a String.
	 */
	protected def getCode() {
		code.toString()
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
		if (code.startsWith("{=")) {
            code.substring(2, code.length - 2).trim();
        } else {
        	code
        }
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
}
