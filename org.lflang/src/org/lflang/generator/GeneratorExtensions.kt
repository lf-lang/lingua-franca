/*************
 * Copyright (c) 2021, TU Dresden.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.generator

/** An object that defines the ,, (rangeTo) operator to prepend each line of the rhs multiline string with the lhs prefix.
 *
 * This is a neat little trick that allows for convenient insertion of multiline strings in string templates
 * while correctly managing the indentation. Consider this multiline string:
 * ```
 * val foo = """
 *    void foo() {
 *        // do something useful
 *    }""".trimIndent()
 * ```
 * Inserting this multiline string into another one like this:
 * ```
 * val bar = """
 *     class Bar {
 *         $foo
 *     };""".trimIndent()
 * ```
 * will unfortunately not produce  correctly indented result. Instead, the result will look like this:
 * * ```
 *     class Bar {
 *         void foo()
 *     // do something useful
 * }
 *     };""".trimIndent()
 * ```
 * This is because kotlin will insert the plain multiline string `foo` into the higher level string
 * without preserving the indentation of the first line. Using the .. operator as defined below,
 * we can properly indent `foo` while keeping a nice looking syntax that visualizes the expected
 * indentation in the resulting string.
 *
 *
 *  With the ,, operator, `foo` could be inserted into a higher level multiline string like this:
 *
 * ```
 * with (prependOperator) {
 *      val bar = """
 *          |class Bar {
 *      ${" |    "..foo}
 *          |};
 *      """.trimMargin()
 *      }
 * ```
 *
 * This will produce the expected output:
 * ```
 * class Bar {
 *     void foo() {
 *         // do something useful
 * };
 * ```
 *
 * Note that we define the .. operator inside of an object in order to restrict its visiblity. Since
 * the meaning of .. might not be obvious in other contexts, it needs to be explicilty enabled with
 * `with(prependOperator)`.
 */
object PrependOperator {
    operator fun String.rangeTo(str: String) = str.replaceIndent(this)
}