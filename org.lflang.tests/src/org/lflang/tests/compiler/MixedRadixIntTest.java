package org.lflang.tests.compiler;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import org.lflang.generator.MixedRadixInt;

public class MixedRadixIntTest {
    
    // Constants used many times below.
    List<Integer> radixes = new ArrayList<Integer>(List.of(2, 3, 4, 5));
    List<Integer> digits = new ArrayList<Integer>(List.of(1, 2, 3, 4));

    @Test
    public void create() throws Exception {
        MixedRadixInt num = new MixedRadixInt(digits, radixes);
        Assertions.assertEquals("1%2, 2%3, 3%4, 4%5", num.toString());
        Assertions.assertEquals(1 + 2*2 + 3*6 + 4*24, num.get());
        
        List<Integer> altDigits = new ArrayList<Integer>(List.of(1, 2, 1));
        MixedRadixInt altNum = new MixedRadixInt(altDigits, radixes);
        Assertions.assertEquals(11, altNum.get());
    }

    @Test
    public void createWithInfinity() throws Exception {
        List<Integer> radixes = new ArrayList<Integer>(List.of(2, 3, 4));
        MixedRadixInt num = new MixedRadixInt(digits, radixes);
        Assertions.assertEquals(1 + 2*2 + 3*6 + 4*24, num.get());
    }

    @Test
    public void createWithError() throws Exception {
        List<Integer> radixes = new ArrayList<Integer>(List.of(2, 3));
        try {
            new MixedRadixInt(digits, radixes);
        } catch (IllegalArgumentException ex) {
            Assertions.assertTrue(ex.getMessage().startsWith("Invalid"));
            return;
        }
        Assertions.assertTrue(false, "Expected exception did not occur.");
    }
    
    @Test
    public void createWithNullAndSet() throws Exception {
        MixedRadixInt num = new MixedRadixInt(null, radixes);
        Assertions.assertEquals(0, num.get());
        Assertions.assertEquals("0%2", num.toString());
        num.set(1 + 2*2 + 3*6 + 4*24);
        Assertions.assertEquals(1 + 2*2 + 3*6 + 4*24, num.get());
    }
    
    @Test
    public void testPermutation() throws Exception {
        List<Integer> radixes = new ArrayList<Integer>(List.of(2, 5));
        List<Integer> digits = new ArrayList<Integer>(List.of(1, 2));
        MixedRadixInt num = new MixedRadixInt(digits, radixes);
        Assertions.assertEquals(5, num.get());
        
        List<Integer> permutation = new ArrayList<Integer>(List.of(1, 0));
        MixedRadixInt pnum = num.permute(permutation);
        Assertions.assertEquals(7, pnum.get());
        
        digits = new ArrayList<Integer>(List.of(1, 2, 1));
        num = new MixedRadixInt(digits, radixes);
        Assertions.assertEquals(15, num.get());

        pnum = num.permute(permutation);
        Assertions.assertEquals(17, pnum.get());
    }
    
    @Test
    public void testIncrement() throws Exception {
        List<Integer> radixes = new ArrayList<Integer>(List.of(2, 3));
        List<Integer> digits = new ArrayList<Integer>(List.of(0, 2));
        MixedRadixInt num = new MixedRadixInt(digits, radixes);
        num.increment();
        Assertions.assertEquals(5, num.get());
        Assertions.assertEquals(2, digits.size());
        num.increment();
        Assertions.assertEquals(6, num.get());
        Assertions.assertEquals(3, digits.size());
    }
}
