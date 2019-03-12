package org.icyphy.lf

class WalkLF extends LinguaFrancaBaseListener {

  var current: Actor = null
  var composite: Composite = null // TODO: shouldn't composite be the current Actor?

  object CodeState extends Enumeration {
    type CodeState = Value
    val Unknown, Pre, Init, React = Value
  }

  var codeState = CodeState.Unknown
  var currentReact: Reaction = null
  var currentInstance: Instance = null
  
  // Assignment statements to set parameters within instance constructors.
  override def enterAssignment(ctx: LinguaFrancaParser.AssignmentContext): Unit = {
    if (currentInstance != null) {
        val parameterName = ctx.ID().getText
        var parameterValue = ctx.lfValue.getText
        // FIXME: The following should be handled by the parser somehow!
        if (parameterValue.startsWith("{=")) {
            parameterValue = parameterValue.substring(2, parameterValue.length - 2);
        }
        currentInstance.instanceParameters += (parameterName -> parameterValue)
    }
  }

  override def enterCompositeHead(ctx: LinguaFrancaParser.CompositeHeadContext): Unit = {
    val name = ctx.ID().getText
    composite = new Composite(name)
    current = composite
    System.actors += (name -> current)
  }
  
  // Handle connections such as a.out -> b.in.
  override def enterConnection(ctx: LinguaFrancaParser.ConnectionContext): Unit = {
    val leftPort = ctx.`lport`.getText
    val rightPort = ctx.`rport`.getText
    // FIXME: Check for composite.
    composite.connections += new Connection(leftPort, rightPort)
  }

  override def enterHead(ctx: LinguaFrancaParser.HeadContext): Unit = {
    val name = ctx.ID().getText
    current = new Actor(name)
    System.actors += (name -> current)
  }
  
  // import statements are assumed to specify where to find accessors to be
  // instantiated in a composite.
  override def enterLfImport(ctx: LinguaFrancaParser.LfImportContext): Unit = {
    val path = ctx.path().getText
    val pieces = path.split('.')
    val root = pieces.last
    val filename = pieces.mkString("/") + ".js" // FIXME: JS specific
    System.imports += (root -> filename)
  }

  // instance statement
  override def enterInstance(ctx: LinguaFrancaParser.InstanceContext): Unit = {
    // FIXME: Generate a friendly error if composite is null (actor is not a Composite).
    val instanceName = ctx.ID.getText
    currentInstance = Instance(instanceName, ctx.lfActorClass.getText)
    composite.instances += (instanceName -> currentInstance)
  }

  override def enterParameter(ctx: LinguaFrancaParser.ParameterContext): Unit = {
    current.parameter += (ctx.ID.getText -> Parameter(ctx.ID.getText, ctx.lfType.getText, ctx.lfValue.getText))
  }

  override def enterOutput(ctx: LinguaFrancaParser.OutputContext): Unit = {
    current.outPorts += (ctx.ID.getText -> Port(ctx.ID.getText, ctx.lfType.getText))
  }

  override def enterPreamble(ctx: LinguaFrancaParser.PreambleContext): Unit = {
    codeState = CodeState.Pre
  }

  override def enterInitialize(ctx: LinguaFrancaParser.InitializeContext): Unit = {
    codeState = CodeState.Init
  }

  override def enterReaction(ctx: LinguaFrancaParser.ReactionContext): Unit = {
    codeState = CodeState.React
    val t = current.triggers(ctx.ID.getText)
    // sets is really a list
    val s = ctx.sets(0).ID.getText
    val r = Reaction(t, s, "")
    currentReact = r
    current.reactions += r
  }

  override def enterCode(ctx: LinguaFrancaParser.CodeContext): Unit = {
    var s: String = ctx.CODE.getText
    s = s.substring(2, s.length - 2)
    codeState match {
      case CodeState.Pre => current.preCode = s
      case CodeState.Init => current.initCode = s
      case CodeState.React => currentReact.code = s
      case _ => "???"
    }
  }

  override def enterTrigger(ctx: LinguaFrancaParser.TriggerContext): Unit = {
    val t = Trigger(ctx.ID.getText, ctx.triggerParameter.ID.getText, ctx.triggerType.getText)
    current.triggers += (ctx.ID.getText -> t)
  }

}
