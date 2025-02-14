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

import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.function.Function;

public class LookupTable1D implements Function<Double, Double> {
    private final ArrayList<NumberPair> f = new ArrayList<>();

    public LookupTable1D addDataPoint(double x, double y) {
        f.add(new NumberPair(x, y));
        f.sort((a, b) -> Double.compare(a.n1, b.n1));
        return this;
    }

    public Double apply(Double x) {
        NumberPair lastNP = null;
        for(NumberPair np: f) {
            if(x == np.n1) {
                return np.n2;
            } else if(x > np.n1) {
                lastNP = np;
            } else {
                if(lastNP == null) {
                    return np.n2;
                }
                double w1 = (x-lastNP.n1) / (np.n1-lastNP.n1);
                return w1 * np.n2 + (1-w1) * lastNP.n2;
            }
        }
        return lastNP.n2;
    }

    @NonNull
    @Override
    public String toString() {
        return f.toString();
    }

    public static void main(String[] args) {
        LookupTable1D f = new LookupTable1D();
        f.addDataPoint(10.0, 1.0);
        f.addDataPoint( 9.0, 0.9);
        f.addDataPoint( 8.0, 0.8);
        f.addDataPoint( 5.0, 0.5);
        f.addDataPoint( 1.0, 0.1);

        System.out.println("f=" + f);

        System.out.println("f(10)=" + f.apply(10.0));
        System.out.println("f(9)="  + f.apply(9.0));
        System.out.println("f(8)="  + f.apply(8.0));
        System.out.println("f(7)="  + f.apply(7.0));
        System.out.println("f(6)="  + f.apply(6.0));
        System.out.println("f(5)="  + f.apply(5.0));
        System.out.println("f(4)="  + f.apply(4.0));
        System.out.println("f(3)="  + f.apply(3.0));
        System.out.println("f(2)="  + f.apply(2.0));
        System.out.println("f(1)="  + f.apply(1.0));

        LookupTable1D r = new LookupTable1D();
        r.addDataPoint(-10.0, -1.0);
        r.addDataPoint( -9.0, -0.9);
        r.addDataPoint( -8.0, -0.8);
        r.addDataPoint( -5.0, -0.5);
        r.addDataPoint( -1.0, -0.1);

        System.out.println("r=" + r);

        System.out.println("r(-10)=" + r.apply(-10.0));
        System.out.println("r(-9)="  + r.apply(-9.0));
        System.out.println("r(-8)="  + r.apply(-8.0));
        System.out.println("r(-7)="  + r.apply(-7.0));
        System.out.println("r(-6)="  + r.apply(-6.0));
        System.out.println("r(-5)="  + r.apply(-5.0));
        System.out.println("r(-4)="  + r.apply(-4.0));
        System.out.println("r(-3)="  + r.apply(-3.0));
        System.out.println("r(-2)="  + r.apply(-2.0));
        System.out.println("r(-1)="  + r.apply(-1.0));

    }
}
