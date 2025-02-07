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
package org.firstinspires.ftc.teamcode.base.calibration;

import androidx.annotation.NonNull;

import java.lang.Math;
import java.util.Locale;

public class Range {
    public double min;
    public double max;

    public Range() {
        min = Double.POSITIVE_INFINITY;
        max = Double.NEGATIVE_INFINITY;
    }

    public Range(double min_in, double max_in) {
        min = min_in;
        max = max_in;
    }

    public Range(double[] a) {
        super();
        for(double n: a) {
            update(n);
        }
    }

    public Range update(double x) {
        if(x<min)
            min = x;
        if(x>max)
            max = x;

        return this;
    }

    public double getSpan() {
        return max - min;
    }

    public double constrain(double x) {
        return Math.min(Math.max(x,min),max);
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.US, "Range(%1$.5f, %2$.5f)", min, max);
    }

    public static void main(String[] args) {
        // getSpan()
        String     format      = "%1$-12s.getSpan() = %2$-6s match:%3$b%n";
        String[][] casesString = new String[][] {
                {"Range(1,10)", Double.toString(new Range( 1,10).getSpan()),"9.0"},
                {"Range(-1,10)",Double.toString(new Range(-1,10).getSpan()),"11.0"}
        };
        for(String[] c: casesString) {
            System.out.printf(Locale.US,format,c[0],c[1],c[1].equals(c[2]));
        }

        // update()
        format                 = "%1$-16s.update() = %2$-30s match:%3$b%n";
        casesString            = new String[][] {
                {"Range(1,10)", new Range(1 ,10).update(20).toString(), "Range(1.00000, 20.00000)"},
                {"Range(-1,10)",new Range(-1,10).update(-50).toString(),"Range(-50.00000, 10.00000)"}
        };
        for(String[] c: casesString) {
            System.out.printf(Locale.US,format,c[0],c[1],c[1].equals(c[2]));
        }

        // constrain()
        format                 = "%1$-16s.constrain(%2$-3s) = %3$-4s match:%4$b%n";
        casesString            = new String[][] {
                {"Range(1,10)","20", Double.toString(new Range(1,10).constrain(20)), "10.0"},
                {"Range(1,10)","-50",Double.toString(new Range(1,10).constrain(-50)),"1.0"}
        };
        for(String[] c: casesString) {
            System.out.printf(Locale.US,format,c[0],c[1],c[2],c[2].equals(c[3]));
        }
    }
}
