package org.icyphy.generator

import java.util.Hashtable
import java.util.LinkedHashMap
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.icyphy.linguaFranca.Instance
import org.icyphy.linguaFranca.Reactor

class SCLGenerator extends GeneratorBase {
	override void doGenerate(Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context, Hashtable<String,String> importTable) {
		super.doGenerate(resource, fsa, context, importTable)
		for (reactor : resource.allContents.toIterable.filter(Reactor)) {
			clearCode()
			generateReactor(reactor, importTable)
			var filename = reactor.name
			if (filename.equalsIgnoreCase('main')) {
				filename = _filename
			}
			fsa.generateFile(filename + ".scl", code)
		}
		var graph = new ReactionGraph(this)
		// Calculate levels for the graph.		
		graph.calculateLevels(main)
		for (node : graph.nodes) {
			// TODO: Use levels to propagate reactions
			
		}
	}

	override generateReactor(Reactor reactor, Hashtable<String,String> importTable) {
		super.generateReactor(reactor, importTable)
		if (reactor.preamble !== null) {
			pr(removeCodeDelimiter(reactor.preamble.code))
		}
		val inputs = new LinkedHashMap<String, String>()
		for (input: reactor.inputs) {
			inputs.put(input.name, input.type)
		}
		val outputs = new LinkedHashMap<String, String>()
		for (output: reactor.outputs) {
			outputs.put(output.name, output.type)
		}
		for (param: getParameters(reactor)) {
		}
		val compositional = reactor.instances.length + reactor.states.length > 0;
		if (compositional) {
			pr('TYPE ' + reactor.name)
			pr('STRUCT')
			indent();
			for (instance: reactor.instances) {
				pr(instance.name + ': ' + instance.reactorClass + ';')
			}
			for (state: reactor.states) {
				pr(state.name + ': ' + state.type + ';')
			}
			unindent();
			pr('END_STRUCT')
			pr('END_TYPE')
		}
		for (connection: reactor.connections) {
			// TODO: Generate connections
		}
		if (reactor.name.equalsIgnoreCase('main')) {
			val content = readFileInClasspath("/lib/SCL/Runtime.scl")
			if (content !== null) {
				pr(content)
			}
		}
		val reactions = reactor.reactions
		var ireaction = 0;
		for (reaction: reactions) {
			val reaction_name = reactor.name + '_' + (ireaction + 1)
			pr('FUNCTION ' + reaction_name)
			indent()
			if (compositional) {
				pr('VAR_IN_OUT')
				indent()
				pr('this' + ': ' + reactor.name + ';')
				unindent()
				pr('END_VAR')
			}
			val ninputs = (if (reaction.uses !== null && reaction.uses.uses !== null) reaction.uses.uses.length) + (if (reaction.triggers !== null) reaction.triggers.length else 0);
			if (ninputs > 0) {
				pr('VAR_INPUT')
				indent()
				if (reaction.triggers !== null) {
					for (v: reaction.triggers) {
						pr(v + ': ' + inputs.get(v) + ';')
					}
				}
				if (reaction.uses !== null && reaction.uses.uses !== null) {
					for (v: reaction.uses.uses) {
						pr(v + ': ' + inputs.get(v) + ';')
					}
				}
				unindent()
				pr('END_VAR')
			}
			if (reaction.produces !== null && reaction.produces.produces !== null && reaction.produces.produces.length > 0) {
				pr('VAR_OUTPUT')
				indent()
				for (v: reaction.produces.produces) {
					pr(v + ': ' + inputs.get(v) + ';')
				}
				unindent()
				pr('END_VAR')
			}
			for(parameter: getParameters(reactor)) {
				// TODO: Handle parameters
				print('# ERROR: PARAMETERS NOT IMPLEMENTED')
			}
			pr(removeCodeDelimiter(reaction.code))
			unindent()
			pr('END_FUNCTION')
			ireaction++
		}
	}
	
	override instantiate(
		Instance instance,
		ReactorInstance container,
		Hashtable<String,String> importTable
	) {
		var reactor = getReactor(instance.reactorClass)
		if (reactor === null) {
			reportError(instance, "No such reactor: " + instance.reactorClass)
			return null
		}
		val reactorInstance = super.instantiate(instance, container, importTable)
		reactorInstance
	}
}
