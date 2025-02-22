/*
 * Copyright (c) 2025 Murad Nayal
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name Murad Nayal nor the names of contributors to this material may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.base.math;

import static java.lang.Math.abs;
import static java.lang.Math.floor;
import static java.lang.Math.ceil;
import static java.lang.Math.log10;
import static java.lang.Math.round;
import static java.lang.Math.sqrt;
import static java.lang.Math.pow;


import java.util.ArrayList;
import java.util.Locale;
import java.util.function.BiPredicate;

public class Math {
    public static double NUMERICAL_TOLERANCE_RATIO = 1E-3;

    /**
     * Solve a quadratic equation aX^2 + bX + c = 0
     * @param a: a coefficient
     * @param b: b coefficiennt
     * @param c: c coefficient
     * @return The two complex roots solving the quadratic equation
     */
    public static     ComplexNumberPair solveQuadraticEquation(double a, double b, double c) {
        double discriminant = b * b - 4 * a * c;
        if (discriminant > 0) {
            double root1 = (-b + sqrt(discriminant)) / (2 * a);
            double root2 = (-b - sqrt(discriminant)) / (2 * a);
            /// Roots are real and different
            return new ComplexNumberPair(
                    new ComplexNumber(root1, 0.0),
                    new ComplexNumber(root2, 0.0)
            );
        } else if (discriminant == 0) {
            double root = -b / (2 * a);
            /// Roots are real and equal
            return new ComplexNumberPair(
                    new ComplexNumber(root, 0.0),
                    new ComplexNumber(root, 0.0)
            );
        } else {
            /// Roots are complex and different
            double realPart = -b / (2 * a);
            double imaginaryPart = sqrt(-discriminant) / (2 * a);
            return new ComplexNumberPair(
                    new ComplexNumber(realPart,  imaginaryPart),
                    new ComplexNumber(realPart, -imaginaryPart)
            );
        }
    }
    /**
     * Determine whether two numbers are approximately equal
     * @param n1: First number
     * @param n2: Second number
     * @param tolerance: the relative tolerance of differences to establish approximate equality
     * @return True if n1 approximately equals n2
     */
    public static     boolean           approxEquals(double n1, double n2, double tolerance) {
        if(n1 == 0.0 && n2 == 0.0)
            return true;
        double errorRatio = abs(n1-n2)/(abs(n1)+abs(n2));
        return errorRatio < tolerance;
    }
    /**
     * Determine whether two numbers are approximately equal
     * @param n1: First number
     * @param n2: Second number
     * @return True if n1 approximately equals n2
     */
    public static     boolean           approxEquals(double n1, double n2) {
        return approxEquals(n1, n2, NUMERICAL_TOLERANCE_RATIO);
    }
    /**
     * Determine whether the time series is at steady state at a certain index
     * @param data: Time series
     * @param eIdx: Index at which the time series might be in steady state
     * @param lookback: Number of indices looking back to determine steady state
     * @return True if the time series is at steady state at eIdx
     */
    public static     boolean           isSteadyState(double[] data, int eIdx, int lookback, double tolerance) {
        return isSteadyStatePredicate(
                data,
                eIdx,
                lookback,
                (Double n1, Double n2) -> approxEquals(n1, n2, tolerance));
    }
    /**
     * Determine whether the time series is at steady state at a certain index
     * @param data: Time series
     * @param eIdx: Index at which the time series might be in steady state
     * @param lookback: Number of indices looking back to determine steady state
     * @param predicate: The predicate determining the similarly between data points in the time
     *                 series
     * @return True if the time series is at steady state at eIdx
     */
    public static     boolean           isSteadyStatePredicate(double[]                    data,
                                                               int                         eIdx,
                                                               int                         lookback,
                                                               BiPredicate<Double, Double> predicate) {
        int    sIdx        = eIdx-lookback+1;
        /// need at least lookback data points to determine steady state
        if(sIdx<0)
            return false;

        double currentData = data[eIdx];
        for(int idx=sIdx; idx<=eIdx; idx++) {
            if(!predicate.test(currentData, data[idx]))
                return false;
        }
        return true;
    }
    /**
     * Determine whether the time series is at steady state at a certain index
     * @param data: Time series
     * @param eIdx: Index at which the time series might be in steady state
     * @param lookback: Number of indices looking back to determine steady state
     * @param predicate: The predicate determining the similarly between data points in the time
     *                 series
     * @return True if the time series is at steady state at eIdx
     * @param <T> The type of time series data
     */
    public static <T> boolean           isSteadyStatePredicate(ArrayList<T>      data,
                                                               int               eIdx,
                                                               int               lookback,
                                                               BiPredicate<T, T> predicate) {
        int    sIdx        = eIdx-lookback+1;
        /// need at least lookback data points to determine steady state
        if(sIdx<0)
            return false;

        T currentData = data.get(eIdx);
        for(int idx=sIdx; idx<=eIdx; idx++) {
            if(!predicate.test(currentData, data.get(idx)))
                return false;
        }
        return true;
    }
    /**
     * Get the index of the start of steady state in the time series
     * @param data: Time series
     * @param lookback: Number of indices looking back to determine steady state
     * @param predicate: The predicate determining the similarity between data points in the time
     *                 series
     * @return The index of the start of steady state data in the time series
     */
    public static     Integer           getSteadyStateStartPredicate(double[]                    data,
                                                                     int                         lookback,
                                                                     BiPredicate<Double, Double> predicate) {
        for(int idx=lookback-1; idx<data.length; idx++)
            if(isSteadyStatePredicate(data, idx, lookback, predicate))
                return idx;

        return null;
    }
    /**
     * Get the index of the start of steady state in the time series
     * @param data: Time series
     * @param lookback: Number of indices looking back to determine steady state
     * @param predicate: The predicate determining similarity between data points in the time series
     * @return The index of the start of steady state data in the time series
     * @param <T> The type of time series data
     */
    public static <T> Integer           getSteadyStateStartPredicate(ArrayList<T>       data,
                                                                      int               lookback,
                                                                      BiPredicate<T, T> predicate) {
        for(int idx=lookback-1; idx<data.size(); idx++)
            if(isSteadyStatePredicate(data, idx, lookback, predicate))
                return idx;

        return null;
    }
    /**
     * Returns the index at which target can be inserted while preserving the order of the
     * array values
     *
     * @param target: The new value
     * @param values: A sorted array of values
     * @return insertion index. The index of the first element greater than target in values
     */
    public static     int               findInsertionIndex(double target, double[] values) {
        int low  = 0;
        int high = values.length - 1;

        while (low <= high) {
            int mid = low + (high - low) / 2;

            if (values[mid] == target) {
                return mid; // Target found at index 'mid'
            } else if (values[mid] < target) {
                low = mid + 1; // Search in the right half
            } else {
                high = mid - 1; // Search in the left half
            }
        }

        return low; // Target not found, return the insertion index
    }
    /**
     * Restrict the number of significant figures in a number to the level of 10^order specified
     * and return the floor
     * @param n: The number
     * @param order: 10^order. Magnitude of the units to retain as powers of 10
     * @return The number of with number of significant figures 'order' retained then floored
     */
    public static     double            retainSignificantDown(double n, int order) {
        if(n == 0.0 || order <= 0.0)
            return n;

        double nSign     = n > 0 ? 1 : -1;
        int    power10   = (int) log10(abs(n)) - order + 1;

        // magnitude to retain
        double magnitude = pow(10, power10);
        double nFloor    = nSign * floor(abs(n) / magnitude);

        /*
        System.out.printf("n=%1$8.3f order=%2$d power10=%3$3d magnitude=%4$8.2f nFloor=%5$8.3f nReg=%6$8.3f%n",
                n, order, power10, magnitude, nFloor, nReg);
        */

        return nFloor * magnitude;
    }
    /**
     * Restrict the number of significant figures in a number to the level of 10^order specified
     * and return the number rounded to nearest digit
     * @param n: The number
     * @param order: 10^order. Magnitude of the units to retain as powers of 10
     * @return The number of with number of significant figures 'order' retained rounded
     */
    public static     double            retainSignificantRound(double n, int order) {
        if(n == 0.0 || order <= 0.0)
            return n;

        double nSign     = n > 0 ? 1 : -1;
        int    power10   = (int) log10(abs(n)) - order + 1;

        // magnitude to retain
        double magnitude = pow(10, power10);
        double nRound    = nSign * round(abs(n) / magnitude);

        /*
        System.out.printf("n=%1$8.3f order=%2$d power10=%3$3d magnitude=%4$8.2f nFloor=%5$8.3f nReg=%6$8.3f%n",
                n, order, power10, magnitude, nFloor, nReg);
        */

        return nRound * magnitude;
    }
    /**
     * Restrict the number of significant figures in a number to the level of 10^order specified
     * and return the ceiling
     * @param n: The number
     * @param order: 10^order. Magnitude of the units to retain as powers of 10
     * @return The number of with number of significant figures 'order' retained then the ceiling
     * is taken
     */
    public static     double            retainSignificantUp(double n, int order) {
        if(n == 0.0 || order <= 0.0)
            return n;

        double nSign     = n > 0 ? 1 : -1;
        int    power10   = (int) log10(abs(n)) - order + 1;

        // magnitude to retain
        double magnitude = pow(10, power10);
        double nCeil     = nSign * ceil(abs(n) / magnitude);

        /*
        System.out.printf("n=%1$8.3f order=%2$d power10=%3$3d magnitude=%4$8.2f nFloor=%5$8.3f nReg=%6$8.3f%n",
                n, order, power10, magnitude, nFloor, nReg);
        */

        return nCeil * magnitude;
    }

    /**
     * Testing main
     * @param args: Not used
     */
    public static void                  main(String[] args) {
        // findInsertionPoint
        System.out.println("\nfindInsertionPoint tests");
        double[]   values  = new double[] {1.0, 2.0, 4.0, 5.0, 7.0};
        String     format  = "insertion index for %1$2d: expected %2$2d returned %3$2d passed: %4$b%n";
        int[][]    casesI   = new int[][] {
                { 1, 0, findInsertionIndex(  1.0, values)},
                {-4, 0, findInsertionIndex( -4.0, values)},
                { 3, 2, findInsertionIndex(  3.0, values)},
                { 6, 4, findInsertionIndex(  6.0, values)},
                {10, 5, findInsertionIndex( 10.0, values)},
        };
        for(int[] c: casesI)
            System.out.printf(Locale.US, format, c[0], c[1], c[2], c[1]==c[2]);

        // retainSignificantDown
        System.out.println("\nretainSignficant[Down/Round/Up] testing");
        format              = "Regularize Down  n=%1$8.2f order=%2$2.0f result=%3$8.3f match=%4$b%n";
        double[][] casesD   = new double[][]{

                {   0.0,   0, retainSignificantDown(   0.0,    0),    0.0  },
                {   0.0,   1, retainSignificantDown(   0.0,    1),    0.0  },
                {  -2.321, 0, retainSignificantDown(  -2.321,  0),   -2.321},
                {  -2.321, 1, retainSignificantDown(  -2.321,  1),   -2.0  },
                {  -2.321, 2, retainSignificantDown(  -2.321,  2),   -2.3  },
                {  -2.321, 3, retainSignificantDown(  -2.321,  3),   -2.32 },
                {  -2.321, 4, retainSignificantDown(  -2.321,  4),   -2.321},
                {   2.321, 0, retainSignificantDown(   2.321,  0),    2.321},
                {   2.321, 1, retainSignificantDown(   2.321,  1),    2.0  },
                {   2.321, 2, retainSignificantDown(   2.321,  2),    2.3  },
                {   2.321, 3, retainSignificantDown(   2.321,  3),    2.32 },
                {   2.321, 4, retainSignificantDown(   2.321,  4),    2.321},
                {   2.321, 5, retainSignificantDown(   2.321,  5),    2.321},
                {   2.321, 6, retainSignificantDown(   2.321,  6),    2.321},
                { 124.3,   0, retainSignificantDown( 124.3,    0),  124.3  },
                { 124.3,   1, retainSignificantDown( 124.3,    1),  100.0  },
                { 124.3,   2, retainSignificantDown( 124.3,    2),  120.0  },
                { 124.3,   3, retainSignificantDown( 124.3,    3),  124.0  },
                {1247.3,   1, retainSignificantDown(1247.3,    1), 1000.0  },
                {1247.3,   2, retainSignificantDown(1247.3,    2), 1200.0  },
                {1247.3,   3, retainSignificantDown(1247.3,    3), 1240.0  },
        };
        for(var c: casesD) {
            System.out.printf(Locale.US, format, c[0], c[1], c[2], approxEquals(c[2],c[3]));
        }

        format              = "Regularize Round n=%1$8.2f order=%2$2.0f result=%3$8.3f match=%4$b%n";
        double[][] casesR   = new double[][]{
                {   0.0,   0, retainSignificantRound(   0.0,    0),    0.0  },
                {   0.0,   1, retainSignificantRound(   0.0,    1),    0.0  },
                {  -2.321, 0, retainSignificantRound(  -2.321,  0),   -2.321},
                {  -2.321, 1, retainSignificantRound(  -2.321,  1),   -2.0  },
                {  -2.321, 2, retainSignificantRound(  -2.321,  2),   -2.3  },
                {  -2.321, 3, retainSignificantRound(  -2.321,  3),   -2.32 },
                {  -2.321, 4, retainSignificantRound(  -2.321,  4),   -2.321},
                {   2.321, 0, retainSignificantRound(   2.321,  0),    2.321},
                {   2.321, 1, retainSignificantRound(   2.321,  1),    2.0  },
                {   2.321, 2, retainSignificantRound(   2.321,  2),    2.3  },
                {   2.321, 3, retainSignificantRound(   2.321,  3),    2.32 },
                {   2.321, 4, retainSignificantRound(   2.321,  4),    2.321},
                {   2.321, 5, retainSignificantRound(   2.321,  5),    2.321},
                {   2.321, 6, retainSignificantRound(   2.321,  6),    2.321},
                { 124.3,   0, retainSignificantRound( 124.3,    0),  124.3  },
                { 124.3,   1, retainSignificantRound( 124.3,    1),  100.0  },
                { 124.3,   2, retainSignificantRound( 124.3,    2),  120.0  },
                { 124.3,   3, retainSignificantRound( 124.3,    3),  124.0  },
                {1247.3,   1, retainSignificantRound(1247.3,    1), 1000.0  },
                {1247.3,   2, retainSignificantRound(1247.3,    2), 1200.0  },
                {1247.3,   3, retainSignificantRound(1247.3,    3), 1250.0  },
                {1247.3,   4, retainSignificantRound(1247.3,    4), 1247.0  },

        };
        for(var c: casesR) {
            System.out.printf(Locale.US, format, c[0], c[1], c[2], approxEquals(c[2],c[3]));
        }

        format              = "Regularize Up    n=%1$8.2f order=%2$2.0f result=%3$8.3f match=%4$b%n";
        double[][] casesU   = new double[][]{
                {   0.0,   0, retainSignificantUp(   0.0,    0),    0.0  },
                {   0.0,   1, retainSignificantUp(   0.0,    1),    0.0  },
                {  -2.321, 0, retainSignificantUp(  -2.321,  0),   -2.321},
                {  -2.321, 1, retainSignificantUp(  -2.321,  1),   -3.0  },
                {  -2.321, 2, retainSignificantUp(  -2.321,  2),   -2.4  },
                {  -2.321, 3, retainSignificantUp(  -2.321,  3),   -2.33 },
                {  -2.321, 4, retainSignificantUp(  -2.321,  4),   -2.321},
                {   2.321, 0, retainSignificantUp(   2.321,  0),    2.321},
                {   2.321, 1, retainSignificantUp(   2.321,  1),    3.0  },
                {   2.321, 2, retainSignificantUp(   2.321,  2),    2.4  },
                {   2.321, 3, retainSignificantUp(   2.321,  3),    2.33 },
                {   2.321, 4, retainSignificantUp(   2.321,  4),    2.321},
                {   2.321, 5, retainSignificantUp(   2.321,  5),    2.321},
                {   2.321, 6, retainSignificantUp(   2.321,  6),    2.321},
                { 124.3,   0, retainSignificantUp( 124.3,    0),  124.3  },
                { 124.3,   1, retainSignificantUp( 124.3,    1),  200.0  },
                { 124.3,   2, retainSignificantUp( 124.3,    2),  130.0  },
                { 124.3,   3, retainSignificantUp( 124.3,    3),  125.0  },
                {1247.3,   1, retainSignificantUp(1247.3,    1), 2000.0  },
                {1247.3,   2, retainSignificantUp(1247.3,    2), 1300.0  },
                {1247.3,   3, retainSignificantUp(1247.3,    3), 1250.0  },

        };
        for(var c: casesU) {
            System.out.printf(Locale.US, format, c[0], c[1], c[2], approxEquals(c[2],c[3]));
        }
    }
}
