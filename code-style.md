# Code style guidelines

Precise guidelines for code formatting are not yet available.
The following guidelines are concerned with more high-level practices.

We use the [Google Java style guide](https://google.github.io/styleguide/javaguide.html).
We refine some of the guidelines here. Unless specified otherwise,
these guidelines should be applied to Xtend, Java and Kotlin files.

### Ordering of source files

https://google.github.io/styleguide/javaguide.html#s3.4.2-ordering-class-contents

> The order you choose for the members and initializers of your class can have a great effect on learnability. However, there's no single correct recipe for how to do it; different classes may order their contents in different ways.

> What is important is that each class uses some logical order, which its maintainer could explain if asked. For example, new methods are not just habitually added to the end of the class, as that would yield "chronological by date added" ordering, which is not a logical ordering.

Currently some of classes are sorted by visibility in very boldly delimited "sections".

This type of sorting is bad practice, as a change in visibility requires the member to be moved around in the class, generally far away. Related members that don't have the same visibility may also be far away.

A more usual practice in java is to
- keep all fields at the very top (and sort them internally by visibility)
- then declare constructors
- then put all methods, in "some logical order". That may mean, a public method, then its three private implementation helpers, then another public method, etc.
- put inner classes at the very end

Keeping fields and constructors next to each other helps understanding the initialization logic of the object. Grouping fields at the top helps understanding at a glance what the object contains, and so, what is its lifetime in the program (is it a data class or identity based? does it contain references to big data structures (eg the AST)? can I copy this efficiently?).


### Visibility

- avoid public non-final fields
- avoid public collection fields (even if final, they may be mutable)


### Collections

- use interfaces like List, Set, Map instead of publishing the implementation

https://www.cs.cornell.edu/courses/JavaAndDS/JavaStyle.html#codeToInterface

> In most cases, the client code doesn't care about the implementation of the list ---it just wants to know what operations are supported. Specifying a linked list is needlessly providing a detail that makes future changes harder.

    List<String> users= new LinkedList<String>(); // Good
    LinkedList<String> users= new LinkedList<String>(); // Bad

#### Collection mutability

In Java, instances of collection classes may be read-only. They will throw an exception when attempting a modification. Read-only views on collections are used to prevent external code from modifying an internal collection by accident, which may break class invariants. Encapsulating modifications to the collection into extra mutator methods is good practice, because it allows maintaining class invariants if the class evolves.

- **Assume that any list returned from a method of an object you don't control is unmodifiable** unless otherwise documented. That means, make a copy if you need a local modification.

- If you return a collection which is a field of the current object, **make it unmodifiable** (`return Collections.unmodifiableList(internalList);`). If the API *must* return a modifiable list, document that it is modifiable. (It's usually better to provide mutator methods to encapsulate the modification to the collection.)


   List<String> contents = container.getListOfContents(); // assume this is unmodifiable
   // you can iterate on the list, get an item, but not set/add/remove
   // If you must modify it locally, make a copy:
   contents = new ArrayList<>(contents); // now it's modifiable, but changes won't affect the `container` object
   contents.add("extra");

   // If you need to modify the internal list of the `container`, add a method to the container that encapsulates that logic
   container.add("extra");


### Javadoc

The google style guide is pretty terse about Javadocs.

https://google.github.io/styleguide/javaguide.html#s7-javadoc

and in particular [Where javadoc is required](https://google.github.io/styleguide/javaguide.html#s7.3-javadoc-where-required).

#### Javadoc style guide

https://www.oracle.com/technical-resources/articles/java/javadoc-tool.html#styleguide

- Write description in English.
- without contractions like "aren't" or "isn't".

> OK to use phrases instead of complete sentences, in the interests of brevity.
> This holds especially in the initial summary and in @param tag descriptions.

> Use 3rd person (descriptive) not 2nd person (prescriptive).
> The description is in 3rd person declarative rather than 2nd person imperative.
>     Gets the label. (preferred)
>     Get the label. (avoid)

#### Compact Javadoc

Omit `@param` and `@return` tag from Javadoc if the summary line
already describes everything there is to know about them.
For instance, prefer:

>    /** Returns the sum of a and b. */
>    public static int getSum(int a, int b) { ... }

to

>    /**
>     * Returns the sum of two integers.
>     * @param a an integer
>     * @param b an integer
>     * @return the sum of both parameters
>     */
>    public static void printSum(int a, int b) { ... }

This of course does not hold where the parameters have richer
contracts and preconditions.

Note: this goes in the sense of the "compact style" described [here](https://www.cs.cornell.edu/courses/JavaAndDS/JavaStyle.html#Comments). It's also how Kotlin docs
should be written.
