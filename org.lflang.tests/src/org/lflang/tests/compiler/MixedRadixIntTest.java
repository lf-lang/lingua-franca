package org.lflang.tests.compiler;

import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.lflang.generator.MixedRadixInt;

public class MixedRadixIntTest {

    // Constants used many times below.
    List<Integer> radixes = Arrays.asList(2, 3, 4, 5);
    List<Integer> digits = Arrays.asList(1, 2, 3, 4);

    @Test
    public void create() throws Exception {
        MixedRadixInt num = new MixedRadixInt(digits, radixes, null);
        Assertions.assertEquals("1%2, 2%3, 3%4, 4%5", num.toString());
        Assertions.assertEquals(1 + 2 * 2 + 3 * 6 + 4 * 24, num.get());

        List<Integer> altDigits = List.of(1, 2, 1);
        MixedRadixInt altNum = new MixedRadixInt(altDigits, radixes, null);
        Assertions.assertEquals(11, altNum.get());
    }

    @Test
    public void createWithInfinity() throws Exception {
        List<Integer> radixes = List.of(2, 3, 4, 10000);
        MixedRadixInt num = new MixedRadixInt(digits, radixes, null);
        Assertions.assertEquals(1 + 2 * 2 + 3 * 6 + 4 * 24, num.get());
    }

    @Test
    public void createWithError() throws Exception {
        List<Integer> radixes = List.of(2, 3);
        try {
            new MixedRadixInt(digits, radixes, null);
        } catch (IllegalArgumentException ex) {
            Assertions.assertTrue(ex.getMessage().startsWith("Invalid"));
            return;
        }
        Assertions.assertTrue(false, "Expected exception did not occur.");
    }

    @Test
    public void createWithNullAndSet() throws Exception {
        MixedRadixInt num = new MixedRadixInt(null, radixes, null);
        Assertions.assertEquals(0, num.get());
        Assertions.assertEquals("0%2", num.toString());
        num.set(1 + 2 * 2 + 3 * 6 + 4 * 24);
        Assertions.assertEquals(1 + 2 * 2 + 3 * 6 + 4 * 24, num.get());
        int mag = num.magnitude();
        Assertions.assertEquals(1 + 2 * 2 + 3 * 6 + 4 * 24, mag);
        num.increment(); // wrap to zero.
        Assertions.assertEquals(0, num.get());
        Assertions.assertEquals(0, num.magnitude());

        num = new MixedRadixInt(null, radixes, null);
        num.setMagnitude(mag);
        Assertions.assertEquals(mag, num.magnitude());
    }

    @Test
    public void testPermutation() throws Exception {
        List<Integer> radixes = Arrays.asList(2, 5);
        List<Integer> digits = Arrays.asList(1, 2);
        List<Integer> permutation = Arrays.asList(1, 0);
        MixedRadixInt num = new MixedRadixInt(digits, radixes, permutation);
        Assertions.assertEquals(5, num.get());
        Assertions.assertEquals(2, num.get(1));
        Assertions.assertEquals(7, num.magnitude());
        num.increment();
        Assertions.assertEquals(7, num.get());
        Assertions.assertEquals(8, num.magnitude());

        num = new MixedRadixInt(null, radixes, permutation);
        num.setMagnitude(8);
        Assertions.assertEquals(8, num.magnitude());
        Assertions.assertEquals(7, num.get());

        // Test radix infinity digit (effectively).
        digits = Arrays.asList(1, 2, 1);
        radixes = Arrays.asList(2, 5, 1000000);
        num = new MixedRadixInt(digits, radixes, null);
        Assertions.assertEquals(15, num.get());
        Assertions.assertEquals(7, num.get(1));
        Assertions.assertEquals(15, num.magnitude());

        permutation = Arrays.asList(1, 0, 2);
        num = new MixedRadixInt(digits, radixes, permutation);
        num.increment();
        Assertions.assertEquals(17, num.get());
        Assertions.assertEquals(18, num.magnitude());

        num = new MixedRadixInt(null, radixes, permutation);
        num.setMagnitude(18);
        Assertions.assertEquals(18, num.magnitude());
        Assertions.assertEquals(17, num.get());
    }

    @Test
    public void testIncrement() throws Exception {
        List<Integer> radixes = Arrays.asList(2, 3);
        List<Integer> digits = Arrays.asList(0, 2);
        MixedRadixInt num = new MixedRadixInt(digits, radixes, null);
        num.increment();
        Assertions.assertEquals(5, num.get());
        num.increment(); // Wrap around to zero.
        Assertions.assertEquals(0, num.get());
    }
}
