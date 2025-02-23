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

import static java.lang.Math.sqrt;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.base.error.BadInputException;
import org.firstinspires.ftc.teamcode.base.utils.ArrayUtils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Locale;
import java.util.function.Function;

public class LookupTableInfD {
    public class DataPoint {
        public        double[] x;
        public        double   y;

        public           DataPoint(double... values) {
            if(values.length != dim+1) {
                String fmt = "LookupTableInfD.DataPoint() dim=%1$d input.dim=%2$d";
                throw new BadInputException(String.format(Locale.US, fmt, dim, values.length));
            }
            x           = Arrays.copyOf(values, dim);
            y           = values[values.length-1];
        }
        public           DataPoint() {
            x           = new double[dim];
        }
        public double    sumX() {
            double sum = 0;
            for(int i=0; i<dim; i++)
                sum   += x[i];
            return sum;
        }
        public DataPoint add(             DataPoint p) {
            DataPoint a = new DataPoint();
            for(int i=0; i<dim; i++)
                a.x[i] = x[i] + p.x[i];
            a.y        = y    + p.y;
            return a;
        }
        public DataPoint subtract(        DataPoint p) {
            DataPoint s = new DataPoint();
            for(int i=0; i<dim; i++)
                s.x[i] = x[i] - p.x[i];
            s.y        = y    - p.y;
            return s;
        }
        public DataPoint addInPlace(      DataPoint p) {
            for(int i=0; i<dim; i++)
                x[i] += p.x[i];
            y        += p.y;

            return this;
        }
        public DataPoint add2InPlace(     DataPoint p) {
            for(int i=0; i<dim; i++)
                x[i] += p.x[i]*p.x[i];
            y        += p.y*p.y;

            return this;
        }
        public DataPoint divide2InPlace(  DataPoint p) {
            for(int i=0; i<dim; i++)
                x[i] /= p.x[i];
            y        /= p.y;

            return this;
        }
        public DataPoint squareInPlace() {
            for(int i=0; i<dim; i++)
                x[i] *= x[i];
            y        *= y;

            return this;
        }
        public DataPoint divide(          double n) {
            DataPoint p = new DataPoint();
            for(int i=0; i<dim; i++)
                p.x[i]  = x[i] / n;
            p.y         = y    / n;

            return p;
        }
        public DataPoint divide(          DataPoint p) {
            DataPoint d = new DataPoint();
            for(int i=0; i<dim; i++)
                d.x[i]  = x[i] / p.x[i];
            d.y         = y    / p.y;

            return d;
        }
        public DataPoint multiply(        DataPoint p) {
            DataPoint m = new DataPoint();
            for(int i=0; i<dim; i++)
                m.x[i] = x[i] * p.x[i];
            m.y        = y    * p.y;
            return m;
        }
        public DataPoint applyTo (        Function<Double,Double> f) {
            for(int i=0; i<dim; i++)
                x[i]  = f.apply(x[i]);
            y         = f.apply(y);

            return this;
        }
        public double    getXDist2(       DataPoint p) {
            return subtract(p).squareInPlace().divide(mo2).sumX() / dim;
        }
        public double    getXDist(        DataPoint p) {
            return sqrt(getXDist2(p));
        }
        @NonNull
        @Override
        public String    toString() {
            return String.format(
                    Locale.US,
                    "DataPoint(x=%1$s, y=%2$.3f)",ArrayUtils.toString(x,"%1$.3f"),y);
        }
    }

    public        int                  dim;
    public        boolean              validStats = false;
    public        ArrayList<DataPoint> data       = new ArrayList<>();
    public        DataPoint            sum;
    public        DataPoint            sum2;
    public        DataPoint            mo1;
    public        DataPoint            mo2;
    public        DataPoint            std;

    public           LookupTableInfD(int dim_in) {
        dim            = dim_in;
        // only now we can create the accumulators
        sum            = new DataPoint();
        sum2           = new DataPoint();
    }
    public DataPoint makeDataPointX(double... xValues) {
        DataPoint p = new DataPoint();
        p.x         = Arrays.copyOf(xValues, dim);
        p.y         = 0.0;
        return p;
    }
    public void      addDataPoint(double... values) {
        DataPoint p   = new DataPoint(values);
        validStats    = false;
        data.add(p);
        sum.addInPlace(p);
        sum2.add2InPlace(p);
    }
    public void      updateStats() {
        mo1            = sum .divide(data.size());
        mo2            = sum2.divide(data.size());
        std            = mo2.subtract(mo1.multiply(mo1)).applyTo((Double x) -> sqrt(x));
        validStats     = true;
    }
    public void      interpolate(double... xValues) {

    }

    @NonNull
    @Override
    public String    toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("LookupTableInfD\n");
        sb.append("  dim=")       .append(dim)       .append("\n");
        sb.append("  validStats=").append(validStats).append("\n");
        sb.append("  data=")      .append(data)      .append("\n");
        sb.append("  sum=")       .append(sum)       .append("\n");
        sb.append("  sum2=")      .append(sum2)      .append("\n");
        sb.append("  mo1=")       .append(mo1)       .append("\n");
        sb.append("  mo2=")       .append(mo2)       .append("\n");
        sb.append("  std=")       .append(std)       .append("\n");

        return sb.toString();
    }

    public static void main(String[] args) {
        LookupTableInfD lut = new LookupTableInfD(3);
        lut.addDataPoint(1, 1, 1, 10);
        lut.addDataPoint(2, 2, 2, 20);
        lut.addDataPoint(3, 3, 3, 30);
        lut.updateStats();
        System.out.println(lut);

        DataPoint p   = lut.makeDataPointX(2, 2, 2);
        for(var pp: lut.data) {
            DataPoint p_pp    = p.subtract(pp);
            DataPoint p_pp2   = p.subtract(pp).squareInPlace();
            DataPoint p_pp2_n = p.subtract(pp).squareInPlace().divide(lut.mo2);
            System.out.printf(
                    Locale.US,
                    "p=%1$s pp=%2$s p-pp=%3$s p_pp2=%4$s p_pp2_n=%5$s xDist2=%6$.3f xDist=%7$.3f%n",
                    p, pp, p_pp, p_pp2, p_pp2_n, p.getXDist2(pp), p.getXDist(pp));
        }
    }
}
