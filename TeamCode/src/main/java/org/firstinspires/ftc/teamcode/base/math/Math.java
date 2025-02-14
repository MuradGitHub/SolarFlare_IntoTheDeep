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
import static java.lang.Math.max;
import static java.lang.Math.sqrt;
import static java.lang.Math.pow;


import java.util.ArrayList;
import java.util.Locale;
import java.util.function.BiPredicate;

public class Math {
    public static double NUMERICAL_TOLERANCE_RATIO = 1E-3;

    public static ComplexNumberPair solveQuadraticEquation(double a, double b, double c) {
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

    public static boolean approxEquals(double n1, double n2, double tolerance) {
        if(n1 == 0.0 && n2 == 0.0)
            return true;
        double errorRatio = abs(n1-n2)/(abs(n1)+abs(n2));
        return errorRatio < tolerance;
    }

    public static boolean approxEquals(double n1, double n2) {
        return approxEquals(n1, n2, NUMERICAL_TOLERANCE_RATIO);
    }

    @SuppressWarnings("SpellCheckingInspection")
    public static boolean isSteadyState(double[] data, int eIdx, int lookback, double tolerance) {
        return isSteadyStatePredicate(
                data,
                eIdx,
                lookback,
                (Double n1, Double n2) -> approxEquals(n1, n2, tolerance));
    }

    @SuppressWarnings("SpellCheckingInspection")
    public static boolean isSteadyStatePredicate(double[] data,
                                                 int      eIdx,
                                                 int      lookback,
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

    public static <T> boolean isSteadyStatePredicate(ArrayList<T>      data,
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

    @SuppressWarnings("SpellCheckingInspection")
    public static Integer getSteadyStateStartPredicate(double[]                    data,
                                                       int                         lookback,
                                                       BiPredicate<Double, Double> predicate) {
        for(int idx=lookback-1; idx<data.length; idx++)
            if(isSteadyStatePredicate(data, idx, lookback, predicate))
                return idx;

        return null;
    }

    @SuppressWarnings("SpellCheckingInspection")
    public static <T> Integer getSteadyStateStartPredicate(ArrayList<T>      data,
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
    public static int findInsertionIndex(double target, double[] values) {
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

    public static double regularizeDown(double n, int order) {
        return floor(n / pow(10,order)) * pow(10,order);
    }

    public static double regularizeUp(double n, int order) {
        return ceil(n / pow(10,order)) * pow(10,order);
    }

    public static void main(String[] args) {
        /// findInsertionPoint
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

        /// regularizeDown
        format              = "Regularize n=%1$.2f order=%2$.0f result=%3$.3f match=%4$b%n";
        double[][] casesD   = new double[][] {
                {1247.3, 1, regularizeDown(1247.3, 1), 1240},
                {1247.3, 2, regularizeDown(1247.3, 2), 1200},
                {1247.3, 3, regularizeDown(1247.3, 3), 1000},
                {1247.3, 1, regularizeUp  (1247.3, 1), 1250},
                {1247.3, 2, regularizeUp  (1247.3, 2), 1300},
                {1247.3, 3, regularizeUp  (1247.3, 3), 2000}
        };
        for(var c: casesD) {
            System.out.printf(Locale.US, format, c[0], c[1], c[2], c[2]==c[3]);
        }
    }
}
