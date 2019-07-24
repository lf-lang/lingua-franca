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
			fsa.generateFile(filename + ".scl", getCode())
		}
		clearCode()
		pr('FUNCTION setup')
		indent()
		var graph = new ReactionGraph(this)
		// Calculate levels for the graph.
		graph.calculateLevels(main)
		var nlevels = 0
		for (node : graph.nodes) {
			nlevels = Math.max(nlevels, node.level + 1)
		}
		var nodesByLevel = newArrayOfSize(nlevels)
		for (var i = 0; i < nodesByLevel.size; i++) {
			nodesByLevel.set(i, newArrayList())
		}
		for (node : graph.nodes) {
			nodesByLevel.get(node.level).add(node)
		}
		for (var i = 0; i < nodesByLevel.size; i++) {
			for (node : nodesByLevel.get(i)) {
				pr("// Check if reaction needs to fire, then fire if so: " + node)
			}
		}
		if (false) {
			val nodeIndices = newHashMap()
			{
				var i = 0
				for (node : graph.nodes) {
					nodeIndices.put(node, i++)
				}
			}
			for (node : graph.nodes) {
				for (other : node.dependentReactions) {
					pr('graph_connect(#graph := #graph' + ', ' + 'parent := ' + nodeIndices.get(node) + ', ' + 'child := ' + nodeIndices.get(other) + ')')
				}
			}
		}
		unindent()
		pr('END_FUNCTION')
		fsa.generateFile("_topology.scl", getCode())
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
		val isMain = reactor.name.equalsIgnoreCase('main');
		if (isMain) {
			val content = readFileInClasspath("/lib/SCL/Runtime.scl")
			if (content !== null) {
				pr(content)
			}
			pr('')
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
			pr('# TODO: Generate port connections and handlers for reactor composition? Should be similar to those for main')
		}
		for (connection: reactor.connections) {
			// TODO: Generate connections
		}
		val reactions = reactor.reactions
		{
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
				val ninputs = (if (reaction.uses !== null && reaction.uses.uses !== null) reaction.uses.uses.length else 0) + (if (reaction.triggers !== null) reaction.triggers.length else 0);
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
				}
				pr(removeCodeDelimiter(reaction.code))
				unindent()
				pr('END_FUNCTION')
				ireaction++
			}
		}
		if (isMain) {
			// TODO: Combine 'main' reaction generation with the normal reaction generation code below somehow?
			pr('FUNCTION _' + reactor.name)  // TODO: Name might conflict
			indent()
			pr('VAR_IN_OUT')
			indent()
			pr('this' + ': ' + reactor.name + ';')
			unindent()
			pr('END_VAR')
			pr('VAR_TEMP')
			indent()
			for (instance: reactor.instances) {
				for (port : instance.reactorClass.reactor.outputs) {
					val varname = instance.name + '_' + port.name
					pr(varname + ': ' + port.type + ';')
				}
				for (port : instance.reactorClass.reactor.inputs) {
					val varname = instance.name + '_' + port.name
					pr(varname + ': ' + port.type + ';')
				}
			}
			unindent()
			pr('END_VAR')
			pr('# TODO: Go through the reactions in the correct order based on the port connections!!')
			pr('# TODO: Check which parameters are present')
			pr('# TODO: Figure out how to timestamp outputs')
			for (instance: reactor.instances) {
				var ireaction = 0
				for (reaction : instance.reactorClass.reactor.reactions) {
					val reaction_name = instance.reactorClass.reactor.name + '_' + (ireaction + 1)
					val args = newArrayList()
					args.add('this' + ' := ' + 'this' + '.' + instance.name)
					for (port : instance.reactorClass.reactor.outputs) {
						val varname = instance.name + '_' + port.name
						args.add(port.name + ' := ' + varname)
					}
					for (port : instance.reactorClass.reactor.inputs) {
						val varname = instance.name + '_' + port.name
						args.add(port.name + ' := ' + varname)
					}
					pr(reaction_name + '(' + args.join(', ') + ')')
					for (connection : reactor.connections) {
						// Propagate any outputs from this actor to the local variables
						if (connection.leftPort.startsWith(instance.name + '.')) {
							// TODO: Don't replace '.' with '_'; figure out a more robust solution to avoid naming clash
							val left = connection.leftPort.replace('.', '_')
							val right = connection.rightPort.replace('.', '_')
							pr(right + ' := ' + left + ';')
						}
					}
					ireaction++
				}
			}
			unindent()
			pr('END_FUNCTION')
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
