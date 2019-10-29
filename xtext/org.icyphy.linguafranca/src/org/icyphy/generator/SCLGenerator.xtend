package org.icyphy.generator

import java.util.Hashtable
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.icyphy.linguaFranca.Reactor

class SCLGenerator extends GeneratorBase {
	val MAIN = "main"
	// http://plc4good.org.ua/files/03_downloads/SCL_table/SCL-cheat-sheet.pdf
	override void doGenerate(Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context, Hashtable<String,String> importTable) {
		super.doGenerate(resource, fsa, context, importTable)
		for (reactor : resource.allContents.toIterable.filter(Reactor)) {
			clearCode()
			generateReactor(reactor, importTable)
			var filename = reactor.name
			if (filename.equalsIgnoreCase(MAIN)) {
				filename = _filename
			}
			fsa.generateFile(filename + ".scl", getCode())
		}
		/* FIXME: Removed to get it to compile.
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

		val allReactors = graph.nodes.map[it.reactorInstance.getReactorClass()].toSet.sortBy[it.name].toArray(<Reactor>newArrayOfSize(0))  // TODO: This doesn't seem to include all reactors??
		val reactorIndices = allReactors.indexed.toMap([it.value], [it.key])
		val reactionIndices = newHashMap(allReactors.toInvertedMap[r | r.reactions.indexed.toMap([it.value], [it.key])].values.flatMap[it.entrySet].map[it.key -> it.value])
		val reactorName = _filename
		prBlock("FUNCTION run", "END_FUNCTION")[
			prBlock("VAR_IN_OUT", "END_VAR")[pr("%s: %s;", instanceName, reactorName)]
			// TODO: Figure out how to timestamp outputs
			// TODO: Add 't' and 't_present' field in structs!
			for (reactionInstance : graph.nodes.toList.sortBy[it.level]) {
				val reaction = reactionInstance.reactionSpec
				val reactorInstance = reactionInstance.reactorInstance
				val containerReactorInstance = org.icyphy.generator.ReactorInstance.get(reactorInstance).parent
				val containerReactor = containerReactorInstance.getReactorClass()
				val containerReactorIndex = reactorIndices.get(containerReactor)
				val reactor = reactorInstance.getReactorClass()
				val reactionIndex = reactionIndices.get(reaction) + 1;
				val reactionName = String.format("%s_%s", reactor.name, reactionIndex)
				val args = newArrayList(String.format("\"this\" := #%s.%s", instanceName, reactorInstance.name))
				val instanceTriggerNames = reaction.triggers.map[reactorInstance.name + "." + it];
				val containerConnectionMap = containerReactor.connections.toMap([it.rightPort], [it.leftPort])
				val incomings = instanceTriggerNames.map[containerConnectionMap.get(it)]
				incomings.indexed.forEach[if (it.value !== null) {
					args.add(String.format("%s := #%s.%s", instanceTriggerNames.get(it.key).split("\\.").last, org.icyphy.generator.ReactorInstance.get(containerReactorInstance).fullName, it.value))
				}]
				reaction.effects?.forEach[
					args.add(String.format("#%s := #%s.%s", it.variable.name, org.icyphy.generator.ReactorInstance.get(reactorInstance).fullName, it.variable.name))
					args.add(String.format("#%s_present := #%s.%s_present", it.variable.name, org.icyphy.generator.ReactorInstance.get(reactorInstance).fullName, it.variable.name))
				]
				val conditions = incomings.indexed.map[String.format("#%s.%s_present", org.icyphy.generator.ReactorInstance.get(containerReactorInstance).fullName, it.value ?: instanceTriggerNames.get(it.key))].filterNull.toList
				if (conditions.size == 0) {
					conditions.add("TRUE")
				}
				prBlock(String.format("IF (%s) THEN", conditions.join(" || ")), "END_IF;")[pr("\"%s\"(%s);", reactionName, args.join(", "))]
			}
		]
		fsa.generateFile("run.scl", getCode())
		* 
		*/
	}

	override generateReactor(Reactor reactor, Hashtable<String,String> importTable) {
		super.generateReactor(reactor, importTable)
		if (reactor.preamble !== null) {
			pr(removeCodeDelimiter(reactor.preamble.code))
		}
		val inputs = newLinkedHashMap(reactor.inputs.map[it.name -> it.type])
		val outputs = newLinkedHashMap(reactor.outputs.map[it.name -> it.type])
		for (param : reactor.parameters) {
		}
		val reactorName = if (reactor.name.equalsIgnoreCase(MAIN)) _filename else reactor.name
		prBlock(String.format("TYPE %s", reactorName), "END_UDT")[
			prBlock("STRUCT", "END_STRUCT")[
				pr("next_firing_time: DINT;  // FOR INTERNAL USE ONLY")
				// FIXME: Removed to get it to compile.
				// reactor.instances.forEach[pr("%s: %s;", it.name, it.reactorClass)]
				reactor.states.forEach[pr("%s: %s;", it.name, it.type)]
				reactor.outputs.forEach[
					pr("%s: %s;  // FOR INTERNAL USE ONLY", it.name, it.type)
					pr("%s_present: BOOL;  // FOR INTERNAL USE ONLY", it.name, it.type)
				]
			]
		]
		reactor.reactions.forEach[reaction, r |
			val reaction_name = String.format("%s_%s", reactorName, r + 1)
			prBlock(String.format("FUNCTION %s", reaction_name), "END_FUNCTION")[
				prBlock("VAR_IN_OUT", "END_VAR")[
					pr("this: %s;", reactorName)
				]
				prBlock("VAR_INPUT", "END_VAR")[
					reaction.triggers?.forEach[
						pr("%s: %s;", it.variable.name, inputs.get(it.variable.name))
						pr("%s_present: %s;", it.variable.name, "BOOL")
					]
					reaction.sources?.forEach[
						pr("%s: %s;", it.variable.name, inputs.get(it.variable.name))
						pr("%s_present: %s;", it.variable.name, "BOOL")
					]
				]
				prBlock("VAR_OUTPUT", "END_VAR")[
					reaction.effects?.forEach[
						pr("%s: %s;", it.variable.name, outputs.get(it.variable.name))
						pr("%s_present: %s;", it.variable.name, "BOOL")
					]
				]
				for (parameter: reactor.parameters) {
					// TODO: Handle parameters
				}
				reaction.effects?.forEach[
					pr("%s_present := %s;", it.variable.name, "TRUE")
				]
				pr(removeCodeDelimiter(reaction.code))
			]
		]
	}

    /* FIXME: Removed to get it to compile.
	override instantiate(
		Instance instance,
		Instance container,
		Hashtable<String,String> importTable
	) {
		var reactor = instance.reactorClass
//		if (reactor === null) {
//			reportError(instance, String.format("No such reactor: %s", instance.reactorClass))
//			return null
//		}
		super.instantiate(instance, container, importTable)
		//reactorInstance
	}
	*
	*/
}
