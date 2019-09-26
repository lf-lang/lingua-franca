package org.icyphy.generator

import java.util.Hashtable
import java.util.LinkedHashMap
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.icyphy.linguaFranca.Instance
import org.icyphy.linguaFranca.Reactor
import java.util.Collections

class SCLGenerator extends GeneratorBase {
	override void doGenerate(Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context, Hashtable<String,String> importTable) {
		super.doGenerate(resource, fsa, context, importTable)
		for (reactor : resource.allContents.toIterable.filter(Reactor)) {
			clearCode()
			generateReactor(reactor, importTable)
			var filename = reactor.name
			if (filename.equalsIgnoreCase("main")) {
				filename = _filename
			}
			fsa.generateFile(filename + ".scl", getCode())
		}
		val graph = new ReactionGraph(this)
		// Calculate levels for the graph.
		graph.calculateLevels(main)
		val instanceName = main.name
		clearCode()
		if (false) {
			val content = readFileInClasspath("/lib/SCL/Runtime.scl")
			if (content !== null) {
				pr(content)
			}
			pr("")
		}
		val allReactors = graph.nodes.map[it.reactorInstance.reactor].toSet.sortBy[it.name].toArray(<Reactor>newArrayOfSize(0))
		val reactorIndices = allReactors.indexed.toMap([it.value], [it.key])
		val reactionIndices = newHashMap(allReactors.toInvertedMap[r | r.reactions.indexed.toMap([it.value], [it.key])].values.flatMap[it.entrySet].map[it.key -> it.value])
		val reactorName = _filename
		prBlock("FUNCTION run", "END_FUNCTION")[
			prBlock("VAR_IN_OUT", "END_VAR")[pr("%s: %s;", instanceName, reactorName)]
			// TODO: Figure out how to timestamp outputs
			for (reactionInstance : graph.nodes.toList.sortBy[it.level]) {
				val reactionName = String.format("%s_%s", reactionInstance.reactorInstance.reactor.name, reactionIndices.get(reactionInstance.reactionSpec) + 1)
				val args = newArrayList()
				args.add(String.format("this := %s.%s", instanceName, reactionInstance.reactorInstance.name))
				for (input : reactionInstance.dependsOnPorts) {
					input.dependsOnPorts.forEach[args.add(String.format("%s := %s.%s", input.portName, it.reactorInstance.fullName, it.portName))]
				}
				reactionInstance.reactionSpec.effects.forEach[args.add(String.format("%s := %s.%s", it.variable.name, reactionInstance.reactorInstance.fullName, it))]
				pr("%s(%s)", reactionName, args.join(", "))
			}
		]
		fsa.generateFile("run.scl", getCode())
	}

	override generateReactor(Reactor reactor, Hashtable<String,String> importTable) {
		super.generateReactor(reactor, importTable)
		if (reactor.preamble !== null) {
			pr(removeCodeDelimiter(reactor.preamble.code))
		}
		val inputs = newLinkedHashMap(reactor.inputs.map[it.name -> it.type])
		val outputs = newLinkedHashMap(reactor.outputs.map[it.name -> it.type])
		for (param : getParameters(reactor)) {
		}
		val reactorName = if (reactor.name.equalsIgnoreCase("main")) _filename else reactor.name
		prBlock(String.format("TYPE %s", reactorName), "END_UDT")[
			prBlock("STRUCT", "END_STRUCT")[
				reactor.instances.forEach[pr("%s: %s;", it.name, it.reactorClass)]
				reactor.states.forEach[pr("%s: %s;", it.name, it.type)]
				if (false) {
					reactor.inputs.forEach[pr("%s: %s;  // FOR INTERNAL USE ONLY", it.name, it.type)]
				}
				reactor.outputs.forEach[pr("%s: %s;  // FOR INTERNAL USE ONLY", it.name, it.type)]
			]
		]
		reactor.reactions.forEach[reaction, r |
			val reaction_name = String.format("%s_%s", reactorName, r + 1)
			prBlock(String.format("FUNCTION %s", reaction_name), "END_FUNCTION")[
				prBlock("VAR_IN_OUT", "END_VAR")[
					pr("this: %s;", reactorName)
				]
				prBlock("VAR_INPUT", "END_VAR")[
					reaction.triggers?.forEach[pr("%s: %s;", it.variable.name, inputs.get(it.variable.name))]
					reaction.sources?.forEach[pr("%s: %s;", it.port.name, inputs.get(it.port.name))]
				]
				prBlock("VAR_OUTPUT", "END_VAR")[
					reaction.effects?.forEach[pr("%s: %s;", it.variable.name, inputs.get(it.variable.name))]
				]
				for (parameter: getParameters(reactor)) {
					// TODO: Handle parameters
				}
				pr(removeCodeDelimiter(reaction.code))
			]
		]
	}

	override instantiate(
		Instance instance,
		ReactorInstance container,
		Hashtable<String,String> importTable
	) {
		var reactor = getReactor(instance.reactorClass.name)
		if (reactor === null) {
			reportError(instance, String.format("No such reactor: %s", instance.reactorClass))
			return null
		}
		val reactorInstance = super.instantiate(instance, container, importTable)
		reactorInstance
	}
}
