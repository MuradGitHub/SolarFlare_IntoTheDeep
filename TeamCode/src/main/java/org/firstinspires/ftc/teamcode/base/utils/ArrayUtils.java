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
package org.firstinspires.ftc.teamcode.base.utils;

import static org.firstinspires.ftc.teamcode.base.utils.StringUtils.repeatAndJoin;

import java.util.Arrays;
import java.util.Locale;

public class ArrayUtils {
    public static double[] concatenate(double[]... arrays) {
        int      numberOfElements = 0;
        for(double[] a: arrays)
            numberOfElements += a.length;
        double[] all = new double[numberOfElements];
        int allIdx = 0;
        for(double[] a: arrays)
            for(int idx=0; idx<a.length; idx++)
                all[allIdx++] = a[idx];

        return all;
    }

    public static void main(String[] args) {
        String     format      = "concatenate(%1$-15s) = %2$-15s matched:%3$b%n";
        String[][] casesString = new String[][] {
                {       "[1,2,3],[4,5]",
                        Arrays.toString(concatenate(new double[]{1,2,3}, new double[]{4,5})),
                        "[1.0, 2.0, 3.0, 4.0, 5.0]"
                },
                {       "[1,2],[3],[4,5]",
                        Arrays.toString(concatenate(new double[]{1,2}, new double[]{3}, new double[]{4,5})),
                        "[1.0, 2.0, 3.0, 4.0, 5.0]"
                }
        };
        for(String[] c: casesString) {
            System.out.printf(Locale.US, format, c[0],c[1],c[1].equals(c[2]));
        }
    }
}
