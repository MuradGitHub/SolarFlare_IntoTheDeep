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

public class LookupTable2D {
    private double[][] data;
    private double[] xValues;
    private double[] yValues;

    // Constructor to initialize the table with data, x, and y values
    public LookupTable2D(double[][] data, double[] xValues, double[] yValues) {
        this.data = data;
        this.xValues = xValues;
        this.yValues = yValues;
    }

    // Method to perform bilinear interpolation
    public double interpolate(double x, double y) {
        // Find the indices of the surrounding data points
        int xIndex = findIndex(x, xValues);
        int yIndex = findIndex(y, yValues);

        // Check for out-of-bounds values
        if (xIndex < 0 || xIndex >= xValues.length - 1 || yIndex < 0 || yIndex >= yValues.length - 1) {
            throw new IllegalArgumentException("Interpolation point out of bounds.");
        }

        // Perform bilinear interpolation
        double x1 = xValues[xIndex];
        double x2 = xValues[xIndex + 1];
        double y1 = yValues[yIndex];
        double y2 = yValues[yIndex + 1];

        double q11 = data[xIndex][yIndex];
        double q12 = data[xIndex][yIndex + 1];
        double q21 = data[xIndex + 1][yIndex];
        double q22 = data[xIndex + 1][yIndex + 1];

        double result = q11 * (x2 - x) * (y2 - y) / ((x2 - x1) * (y2 - y1)) +
                q21 * (x - x1) * (y2 - y) / ((x2 - x1) * (y2 - y1)) +
                q12 * (x2 - x) * (y - y1) / ((x2 - x1) * (y2 - y1)) +
                q22 * (x - x1) * (y - y1) / ((x2 - x1) * (y2 - y1));

        return result;
    }

    // Helper method to find the index of a value in an array
    private int findIndex(double value, double[] array) {
        for (int i = 0; i < array.length - 1; i++) {
            if (value >= array[i] && value < array[i + 1]) {
                return i;
            }
        }
        return -1;
    }
}
