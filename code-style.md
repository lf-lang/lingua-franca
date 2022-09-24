# Code style guidelines

## General Guidelines

No copypasta!

  - method length advisory; rule of thumb, it must fit on your screen without scrolling (no hard limit, but anything above 40 should be considered for breaking up)

- classes should be reasonably concise and well focused; anything above 1000 lines probably needs refactoring
  - a note on fixme's: they should not pass code review unless a github issue is created for addressing it
  TODO: example

   - column limit 100 for code, 80 for JavaDoc?
  - 4 spaces instead of 2 (double indentation for continuations on new line)



## Java

We use the [Google Java style guide](https://google.github.io/styleguide/javaguide.html), with a small number of exceptions.

### Ordering of class contents ([S3.4.2](https://google.github.io/styleguide/javaguide.html#s3.4.2-ordering-class-contents))

> The order you choose for the members and initializers of your class can have a great effect on learnability. However, there's no single correct recipe for how to do it; different classes may order their contents in different ways.

> What is important is that each class uses some logical order, which its maintainer could explain if asked. For example, new methods are not just habitually added to the end of the class, as that would yield "chronological by date added" ordering, which is not a logical ordering.

When in doubt, the following suggestions might help:
- keep all fields at the very top (and sort them internally by visibility)
- then declare constructors
- then put all methods, in "some logical order". That may mean, a public method, then its three private implementation helpers, then another public method, etc.
- put inner classes at the very end

### Visibility
- avoid public non-final fields
- avoid public collection fields (even if final, they may be mutable)

### Design for inheritance or prohibit it
Make new classes final by default, and don't use protected members. The class should only be unsealed if a use case for inheritance shows up, which is in many cases never. Often, composition can offer a better solution than inheritance (also see a [blog post](https://matthiasnoback.nl/2018/09/final-classes-by-default-why/) on this).

### Collections
Use interfaces like `List`, `Set`, `Map` to type variables, constructor parameters, and method signatures, not concrete implementations like `LinkedList`, `HashSet`, or `LinkedHashMap`.

#### Collection mutability
In Java, instances of collection classes may be read-only. They will throw an exception when modification is attempted. Read-only views on collections are used to prevent external code from modifying an internal collection by accident, which may break class invariants. Encapsulating modifications to the collection into extra mutator methods is good practice, because it allows maintaining class invariants if the class evolves.

- **Assume that any list returned from a method of an object you don't control is unmodifiable** unless otherwise documented. That means, make a copy if you need a local modification.

- If you return a collection which is a field of the current object, **make it unmodifiable** (`return Collections.unmodifiableList(internalList);`). If the API *must* return a modifiable list, then document that it is modifiable. (It's usually better to provide mutator methods to encapsulate the modification to the collection.)

```
List<String> contents = container.getListOfContents(); // assume this is unmodifiable
// you can iterate on the list, get an item, but not set/add/remove
// If you must modify it locally, make a copy:
contents = new ArrayList<>(contents); // now it's modifiable, but changes won't affect the `container` object
contents.add("extra");

// If you need to modify the internal list of the `container`, add a method to the container that encapsulates that logic
container.add("extra");
```
### Comments

FIXME: what to put in them, where to put them, etc.

#### Javadoc

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


## Kotlin
