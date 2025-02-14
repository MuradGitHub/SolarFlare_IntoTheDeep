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

import static java.lang.Math.min;
import static java.lang.Math.max;

import java.util.Arrays;
import java.util.Locale;
import java.util.PriorityQueue;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.firstinspires.ftc.teamcode.base.math.Math.approxEquals;
import static org.firstinspires.ftc.teamcode.base.math.Math.findInsertionIndex;
import static org.firstinspires.ftc.teamcode.base.utils.StringUtils.join;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.base.logging.MetricsFile;
import org.firstinspires.ftc.teamcode.base.logging.MultiMetricsWriter;
import org.firstinspires.ftc.teamcode.base.logging.RobotLogger;
import org.firstinspires.ftc.teamcode.base.logging.RobotMetrics;

public class LookupTable2D extends MultiMetricsWriter {
    private transient final Logger     logger  = RobotLogger.getInstance().getConfigLogger();
    private final           int        xResolution;
    private final           int        yResolution;
    private final           int        xIdxMax;
    private final           int        yIdxMax;
    public                  Range      xRange;
    public                  Range      yRange;
    private                 double[][] rawData = null;
    private                 double[][] data    = null;
    private                 double[][] weights = null;
    public                  double[]   xValues = null;
    public                  double[]   yValues = null;

    public static class EmptyPoint implements Comparable<EmptyPoint> {
        public int    xIdx;
        public int    yIdx;
        public double emptyValencesRatio = 0;
        public EmptyPoint(int xIdx_in, int yIdx_in) {
            xIdx = xIdx_in;
            yIdx = yIdx_in;
        }

        @Override
        public int compareTo(EmptyPoint other) {
            return Double.compare(emptyValencesRatio, other.emptyValencesRatio);
        }

        @NonNull
        @Override
        public String toString() {
            return String.format(Locale.US,
                    "EmptyPoint(xIdx=%1$d yIdx=%2$d, emptyValencesRatio=%3$.3f)",
                    xIdx, yIdx, emptyValencesRatio);

        }
    }

    public static class Point {
        int xIdx;
        int yIdx;
        public Point(int xIdx_in, int yIdx_in) {
            xIdx = xIdx_in;
            yIdx = yIdx_in;
        }
        @NonNull
        @Override
        public String toString() {
            return String.format(Locale.US, "Point(%1$d, %2$d)", xIdx, yIdx);
        }
    }

    public static class NeighborIterator {
        int     xIdx;
        int     yIdx;
        int     xIdxMax;
        int     yIdxMax;
        Point[] neighbors  = new Point[8];
        int     currentIdx = 0;
        public NeighborIterator(int xIdx_in, int yIx_in, int xIdxMax_in, int yIdxMax_in) {
            xIdx        = xIdx_in;
            yIdx        = yIx_in;
            xIdxMax     = xIdxMax_in;
            yIdxMax     = yIdxMax_in;
            for(int xItr=max(xIdx-1, 0); xItr<=min(xIdx+1, xIdxMax); xItr++) {
                for(int yItr=max(yIdx-1, 0); yItr<=min(yIdx+1, yIdxMax); yItr++) {
                    if(xItr==xIdx && yItr==yIdx)
                        continue;
                    neighbors[currentIdx++] = new Point(xItr, yItr);
                }
            }
            currentIdx = 0;
        }
        public boolean hasMorePoints() {
            return (currentIdx < neighbors.length) && neighbors[currentIdx] != null;
        }

        public Point getNextPoint() {
            return neighbors[currentIdx++];
        }

        @NonNull
        @Override
        public String toString() {
            StringBuilder sb = new StringBuilder();
            sb.append("NeighborIterator\n")                                   .append("\n");
            sb.append("  xIdx=")       .append(xIdx)                      .append("\n");
            sb.append("  yIdx=")       .append(yIdx)                      .append("\n");
            sb.append("  xIdxMax=")    .append(xIdxMax)                   .append("\n");
            sb.append("  yIdxMax=")    .append(yIdxMax)                   .append("\n");
            sb.append("  neighbors=")  .append(Arrays.toString(neighbors)).append("\n");
            sb.append("  currentIdx=") .append(currentIdx)                .append("\n");

            return sb.toString();
        }
    }

    // Constructor to initialize the table with data, x, and y values
    public LookupTable2D(double[][] data_in, double[] xValues_in, double[] yValues_in) {
        data         = data_in;
        rawData      = data_in;
        xValues      = xValues_in;
        yValues      = yValues_in;

        xResolution  = xValues.length;
        yResolution  = yValues.length;

        xIdxMax      = xResolution-1;
        yIdxMax      = yResolution-1;

        weights      = new double[xResolution][yResolution];
        for(double[] r: weights)
            Arrays.fill(r, 1.0);

        xRange       = new Range(xValues);
        yRange       = new Range(yValues);
    }

    public LookupTable2D(int xResolution_in, double xMin_in, double xMax_in,
                         int yResolution_in, double yMin_in, double yMax_in) {
        xResolution = xResolution_in;
        yResolution = yResolution_in;
        xIdxMax     = xResolution-1;
        yIdxMax     = yResolution-1;
        xRange      = new Range(xMin_in, xMax_in);
        yRange      = new Range(yMin_in, yMax_in);

        initArrays();
        initMetricsSpecs();
    }

    private void initArrays() {
        // xValues
        if(xValues == null) {
            xValues           = new double[xResolution];
            double dx         = xRange.getSpan() / xIdxMax;
            for(int xIdx=0; xIdx<xResolution; xIdx++)
                xValues[xIdx] = xRange.min + xIdx*dx;
        }

        // yValues
        if(yValues == null) {
            yValues           = new double[yResolution];
            double dy         = yRange.getSpan() / yIdxMax;
            for(int yIdx=0; yIdx<yResolution; yIdx++)
                yValues[yIdx] = yRange.min + yIdx*dy;
        }

        // data
        if(data == null) {
            data              = new double[xResolution][yResolution];
            for(double[] r: data)
                Arrays.fill(r, 0.0);
        }

        // rawData
        if(rawData == null) {
            rawData           = new double[xResolution][yResolution];
            for (double[] r: rawData)
                Arrays.fill(r, 0.0);
        }

        // weights
        if(weights == null) {
            weights           = new double[xResolution][yResolution];
            for(double[] r: weights)
                Arrays.fill(r, 0.0);
        }
    }

    protected void initMetricsSpecs() {
        // Header labeled columns. the elements of each row. These are velocities
        //   first column is the power labels
        String header      = "," + join(yValues, "%1$.4f", ",");
        // String format   = repeatAndJoinFormat("%1$.3f",",",yResolution+1);
        String itemFormat  = "%1$.3f";

        // Weights - prefilled
        addMetricsSpec(
                "LookupTable2D-Weights-Prefill",
                "LookupTable2D-Weights-Prefill",
                itemFormat,
                yResolution+1,
                header,
                getMetricsFileId());
        // Weights - prefilled
        addMetricsSpec(
                "LookupTable2D-Weights",
                "LookupTable2D-Weights",
                itemFormat,
                yResolution+1,
                header,
                getMetricsFileId());
        // rawData - prefilled
        addMetricsSpec(
                "LookupTable2D-RawData-Prefill",
                "LookupTable2D-RawData-Prefill",
                itemFormat,
                yResolution+1,
                header,
                getMetricsFileId());
        // rawData
        addMetricsSpec(
                "LookupTable2D-RawData",
                "LookupTable2D-RawData",
                itemFormat,
                yResolution+1,
                header,
                getMetricsFileId());
        // data
        addMetricsSpec(
                "LookupTable2D-Data",
                "LookupTable2D-Data",
                itemFormat,
                yResolution+1,
                header,
                getMetricsFileId());
    }

    private void fillEmptyCells() {
        PriorityQueue<EmptyPoint> ePoints          = new PriorityQueue<>();
        for(int xIdx=0; xIdx<xResolution; xIdx++) {
            for(int yIdx=0; yIdx<yResolution; yIdx++) {
                if(weights[xIdx][yIdx] == 0.0) {
                    EmptyPoint   ePoint        = new EmptyPoint(xIdx, yIdx);
                    NeighborIterator itr       = new NeighborIterator(xIdx, yIdx, xIdxMax, yIdxMax);
                    int numberOfNeighbors      = 0;
                    // System.out.println("neighbor itr\n" + itr);
                    while(itr.hasMorePoints()) {
                        numberOfNeighbors++;
                        Point nPoint           = itr.getNextPoint();
                        if(weights[nPoint.xIdx][nPoint.yIdx] == 0.0)
                            ePoint.emptyValencesRatio++;
                    }
                    ePoint.emptyValencesRatio /= numberOfNeighbors;
                    ePoints.add(ePoint);
                }
            }
        }
        int     maxIIdx                        = max(xResolution, yResolution);
        int     iIdx                           = 0;
        int     pIdx                           = 0;
        int     pXIdx;
        int     pYIdx;
        boolean hasEmptyPoints                 = false;
        do {
            for(EmptyPoint ePoint: ePoints) {
                pXIdx                          = ePoint.xIdx;
                pYIdx                          = ePoint.yIdx;
                NeighborIterator itr           = new NeighborIterator(pXIdx,pYIdx,xIdxMax,yIdxMax);
                double z                       = 0;
                double weight                  = 0;
                int numberOfNeighbors          = 0;
                while (itr.hasMorePoints()) {
                    Point point                = itr.getNextPoint();
                    int   nPXIdx               = point.xIdx;
                    int   nPYIdx               = point.yIdx;
                    if (weights[nPXIdx][nPYIdx] == 0)
                        continue;
                    numberOfNeighbors++;
                    z                         += rawData[nPXIdx][nPYIdx];
                    weight                    += weights[nPXIdx][nPYIdx];
                }
                if (numberOfNeighbors == 0.0) {
                    logger.logp(
                            Level.INFO,
                            "LookupTable2D",
                            "fillEmptyCells",
                            "Iteration=" + iIdx + " pIdx=" + pIdx++ + " " + ePoint + " has no neighbors"
                    );
                    hasEmptyPoints             = true;
                    continue;
                }

                // System.out.printf(Locale.US, "%1$s z=%2$.3f weight=%3$.3f%n",ePoint,z,weight);
                rawData[pXIdx][pYIdx]          = z      / numberOfNeighbors;
                weights[pXIdx][pYIdx]          = weight / numberOfNeighbors;
            }
            pIdx                               = 0;
        } while(hasEmptyPoints && iIdx++<maxIIdx);
    }

    public void update() {
        fillEmptyCells();

        for (int xIdx = 0; xIdx < xResolution; xIdx++)
            for (int yIdx = 0; yIdx < yResolution; yIdx++) {
                double weight = weights[xIdx][yIdx];
                data[xIdx][yIdx] = weight != 0.0 ? rawData[xIdx][yIdx] / weight : 0.0;
            }
    }

    public void addDataPoint(double x, double y, double z) {
        // First find the xIdx-yIdx square where the new data point falls
        int xIdx2              = findInsertionIndex(x, xValues);
        int yIdx2              = findInsertionIndex(y, yValues);
        int xIdx1              = max(xIdx2-1, 0);
        int yIdx1              = max(yIdx2-1, 0);

        // System.out.printf(Locale.US, "addDataPoint x= %1$.3f y= %2$.3f z= %3$.3f%n",x,y,z);
        // System.out.printf(Locale.US, "addDataPoint square: xIdx1= %1$d yIdx1= %2$d xIdx2= %3$d yIdx2= %4$d%n",xIdx1,yIdx1,xIdx2,yIdx2);

        double x1              = xValues[xIdx1];
        double x2              = xValues[xIdx2];
        double y1              = yValues[yIdx1];
        double y2              = yValues[yIdx2];

        // System.out.printf(Locale.US,"addDataPoint x1= %1$.3f x2= %2$.3f y1= %3$.3f y2= %4$.3f%n",x1,x2,y1,y2);

        double w11             = getWeight11(x, y, x1, y1, x2, y2);
        double w12             = getWeight12(x, y, x1, y1, x2, y2);
        double w21             = getWeight21(x, y, x1, y1, x2, y2);
        double w22             = getWeight22(x, y, x1, y1, x2, y2);

        // System.out.printf(Locale.US,"addDataPoint w11 =%1$.3f w12 =%2$.3f w21 =%3$.3f w22 =%4$.3f%n%n",w11,w12,w21,w22);

        rawData[xIdx1][yIdx1] += z * w11;
        rawData[xIdx1][yIdx2] += z * w12;
        rawData[xIdx2][yIdx1] += z * w21;
        rawData[xIdx2][yIdx2] += z * w22;

        weights[xIdx1][yIdx1] += w11;
        weights[xIdx1][yIdx2] += w12;
        weights[xIdx2][yIdx1] += w21;
        weights[xIdx2][yIdx2] += w22;
    }

    /**
     * Method to perform bilinear interpolation. get z using interpolation on theLUT
     * @param x: x value of the point to interpolate
     * @param y: y value of the point to interpolate
     * @return interpolated z value
     */
    public double interpolate(double x, double y) {

        // System.out.printf(Locale.US, "%ninterpolate%nx=%1$.3f y=%2$.3f%n",x,y);

        // Find the indices of the surrounding data points
        int xIdx1 = min(max(findInsertionIndex(x, xValues), 0), xIdxMax);
        int yIdx1 = min(max(findInsertionIndex(y, yValues), 0), yIdxMax);
        int xIdx2 = min(xIdx1+1, xIdxMax);
        int yIdx2 = min(yIdx1+1, yIdxMax);

        // System.out.printf(Locale.US, "xIdx1= %1$d xIdx2= %2$d yIdx1= %3$d yIdx2= %4$d%n",xIdx1,xIdx2,yIdx1,yIdx2);

        // Perform bilinear interpolation
        double x1  = xValues[xIdx1];
        double x2  = xValues[xIdx2];
        double y1  = yValues[yIdx1];
        double y2  = yValues[yIdx2];

        // System.out.printf(Locale.US, "x1= %1$.3f x2= %2$.3f y1= %3$.3f y2= %4$.3f%n",x1,x2,y1,y2);

        double q11 = data[xIdx1][yIdx1];
        double q12 = data[xIdx1][yIdx2];
        double q21 = data[xIdx2][yIdx1];
        double q22 = data[xIdx2][yIdx2];

        // System.out.printf(Locale.US, "q11= %1$.3f q21= %2$.3f q12= %3$.3f q22= %4$.3f%n",q11,q21,q12,q22);

        double w11 = getWeight11(x, y, x1, y1, x2, y2);
        double w21 = getWeight21(x, y, x1, y1, x2, y2);
        double w12 = getWeight12(x, y, x1, y1, x2, y2);
        double w22 = getWeight22(x, y, x1, y1, x2, y2);

        // System.out.printf(Locale.US, "w11= %1$.3f w21= %2$.3f w12= %3$.3f w22= %4$.3f%n",w11,w21,w12,w22);

        return q11 * w11 + q21 * w21 + q12 * w12 + q22 * w22;

        /*
        return  q11 * getWeight11(x, y, x1, y1, x2, y2) +
                q21 * getWeight21(x, y, x1, y1, x2, y2) +
                q12 * getWeight12(x, y, x1, y1, x2, y2) +
                q22 * getWeight22(x, y, x1, y1, x2, y2);
         */
    }

    public double getWeight11(double x, double y, double x1, double y1, double x2, double y2) {
        x     = min(max(x,x1),x2);
        y     = min(max(y,y1),y2);
        if(x1 == x2 || y1 == y2) {
            if (x1 != x2)
                /// y-side of the square is collapsed. i.e. y1 == y2. Uses x-side for weight
                return (x2 - x) / (x2 - x1);
            else if(y1 != y2)
                /// x-side of the square has collapsed but not the y-side. Use y-size for weight
                return (y2 - y) / (y2 - y1);
            /// both the x-side and the y-side of the inference square have collapsed. Point11,
            /// which is actually coincidental to all 4 square points, gets all the weight
            return 1.0;
        } else {
            return (x2 - x) * (y2 - y) / ((x2 - x1) * (y2 - y1));
        }
    }

    public double getWeight21(double x, double y, double x1, double y1, double x2, double y2) {
        x     = min(max(x,x1),x2);
        y     = min(max(y,y1),y2);
        if(x1 == x2 || y1 == y2) {
            if (x1 != x2)
                return (x - x1) / (x2 - x1);
            else if(y1 != y2)
                /// The x-side of the inference square collapsed. The point21, which is now
                /// equal to point11, should have zero weight.
                return 0.0; // (y2 - y) / (y2 - y1);
            return 0.0;
        } else {
            return (x - x1) * (y2 - y) / ((x2 - x1) * (y2 - y1));
        }
    }

    public double getWeight12(double x, double y, double x1, double y1, double x2, double y2) {
        x     = min(max(x,x1),x2);
        y     = min(max(y,y1),y2);
        if(x1 == x2 || y1 == y2) {
            if (x1 != x2)
                /// y-side collapsed. There should be no weight assigned to points in the second
                /// segment of the inference square
                return 0.0; // (x2 - x) / (x2 - x1);
            else if(y1 != y2)
                return (y - y1) / (y2 - y1);
            return 0.0;
        } else {
            return (x2 - x) * (y - y1) / ((x2 - x1) * (y2 - y1));
        }
    }

    public double getWeight22(double x, double y, double x1, double y1, double x2, double y2) {
        x     = min(max(x,x1),x2);
        y     = min(max(y,y1),y2);
        if(x1 == x2 || y1 == y2) {
            if (x1 != x2)
                /// y-side collapsed. There should be no weight assigned to the second
                /// segment of the inference square
                return 0.0; // (x - x1) / (x2 - x1);
            else if(y1 != y2)
                return (y - y1) / (y2 - y1);
            return 0.0;
        } else {
            return (x - x1) * (y - y1) / ((x2 - x1) * (y2 - y1));
        }
    }

    public void writeMetricsPrefill() {
        RobotMetrics robotMetrics = RobotMetrics.getInstance();

        // weights - pre-fill
        MetricsFile fileWeightsPrefill = robotMetrics
                .getMetricsFile(getMetricsSpec("LookupTable2D-Weights-Prefill"));
        for(int xIdx=0; xIdx<xResolution; xIdx++) {
            fileWeightsPrefill.addDataItem("%1$.3f,", xValues[xIdx]);
            fileWeightsPrefill.addData(weights[xIdx]);
        }
        fileWeightsPrefill.close();

        // raw data - pre-fill
        MetricsFile fileRawDataPrefill = robotMetrics
                .getMetricsFile(getMetricsSpec("LookupTable2D-RawData-Prefill"));
        for(int xIdx=0; xIdx<xResolution; xIdx++) {
            fileRawDataPrefill.addDataItem("%1$.3f,", xValues[xIdx]);
            fileRawDataPrefill.addData(rawData[xIdx]);
        }
        fileRawDataPrefill.close();
    }

    public void writeMetrics() {
        RobotMetrics robotMetrics = RobotMetrics.getInstance();

        // weights
        MetricsFile fileWeights = robotMetrics
                .getMetricsFile(getMetricsSpec("LookupTable2D-Weights"));
        for(int xIdx=0; xIdx<xResolution; xIdx++) {
            fileWeights.addDataItem("%1$.3f,", xValues[xIdx]);
            fileWeights.addData(weights[xIdx]);
        }
        fileWeights.close();

        // rawData
        MetricsFile fileRawData = robotMetrics
                .getMetricsFile(getMetricsSpec("LookupTable2D-RawData"));
        for(int xIdx=0; xIdx<xResolution; xIdx++) {
            fileRawData.addDataItem("%1$.3f,", xValues[xIdx]);
            fileRawData.addData(rawData[xIdx]);
        }
        fileRawData.close();

        // data
        MetricsFile fileData = robotMetrics
                .getMetricsFile(getMetricsSpec("LookupTable2D-Data"));
        for(int xIdx=0; xIdx<xResolution; xIdx++) {
            fileData.addDataItem("%1$.3f,", xValues[xIdx]);
            fileData.addData(data[xIdx]);
        }
        fileData.close();
    }

    @NonNull
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("LookupTable2D\n");
        sb.append("  xResolution=").append(xResolution)             .append("\n");
        sb.append("  yResolution=").append(yResolution)             .append("\n");
        sb.append("  xIdxMax=")    .append(xIdxMax)                 .append("\n");
        sb.append("  yIdxMax=")    .append(yIdxMax)                 .append("\n");
        sb.append("  xRange=")     .append(xRange)                  .append("\n");
        sb.append("  yRange=")     .append(yRange)                  .append("\n");
        sb.append("  xValues=")    .append(Arrays.toString(xValues)).append("\n");
        sb.append("  yValues=")    .append(Arrays.toString(yValues)).append("\n");

        sb.append("  rawData=\n");
        for(double[] r: rawData)
            sb.append("  ").append(Arrays.toString(r)).append("\n");
        sb.append("  data=\n");
        for(double[] r: data)
            sb.append("  ").append(Arrays.toString(r)).append("\n");
        sb.append("  weights=\n");
        for(double[] r: weights)
            sb.append("  ").append(Arrays.toString(r)).append("\n");

        return sb.toString();
    }

    public static void main(String[] args) {
        LookupTable2D lut = new LookupTable2D(5, 0, 4, 5, 0, 4);
        for(int xIdx=0; xIdx<5; xIdx++) {
            for(int yIdx=0; yIdx<5; yIdx++) {
                /// skip certain data points to assess the robustness of the LUT
                if(     (xIdx==0 && yIdx==1) || (xIdx==0 && yIdx==2) ||
                        (xIdx==1 && yIdx==1) || (xIdx==1 && yIdx==2) ||
                        (xIdx==2 && yIdx==1) || (xIdx==2 && yIdx==2) ||
                        (xIdx==3 && yIdx==1) || (xIdx==3 && yIdx==2)
                )
                    continue;
                lut.addDataPoint(xIdx, yIdx, xIdx+yIdx);
            }
        }

        lut.update();

        System.out.println(lut);

        String knownCase;
        for(int x=-2; x<8; x+=1) {
            double xTrim = lut.xRange.constrain(x);
            for (int y = -3; y < 9; y += 1) {
                double yTrim = lut.yRange.constrain(y);
                double r     = xTrim + yTrim;
                double rInter = lut.interpolate(x, y);
                if ((x == -2 && (y == 1 || y == 2)) ||
                        (x == -1 && (y == 1 || y == 2)) ||
                        (x == 0 && (y == 1 || y == 2))
                )
                    knownCase = "Known Case";
                else
                    knownCase = "";
                System.out.printf(Locale.US,
                        "x=%1$3d y=%2$3d expected=%3$5.3f returned=%4$5.3f match: %5$5b %6$s%n",
                        x, y, r, rInter, approxEquals(r, rInter, 0.10), knownCase);
            }
        }
    }
}
