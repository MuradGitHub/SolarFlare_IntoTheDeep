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
package org.firstinspires.ftc.teamcode.base.logging;

import androidx.annotation.NonNull;

import java.io.IOException;
import java.util.Formatter;
import java.util.logging.Level;
import java.util.logging.Logger;

public class MetricsFile {
    private final MetricsSpec robotMetricsSpec;
    private       Formatter        formatter;
    private final String           fileId;
    private final String           fullFileName;

    public MetricsFile(MetricsSpec robotMetricsSpec_in, String fileId_in) {
        robotMetricsSpec = robotMetricsSpec_in;
        fileId           = fileId_in;
        fullFileName     = robotMetricsSpec.getFullFileName(fileId);
        open();
    }

    public void open() {
        if(isActive())
            close();

        try {
            formatter = new Formatter(fullFileName);
        } catch (IOException e) {
            Logger.getGlobal().logp(
                    Level.SEVERE,
                    "MetricsFile",
                    "open",
                    "Failed to open file:" + fullFileName,
                    e);
            return;
        }

        formatter.format("%1$s", robotMetricsSpec.getHeader() + "\n");
    }

    public <T> void addDataItem(String format, T dataItem) {
        if(!isActive()) {
            Logger.getGlobal().severe("MetricsFile " + robotMetricsSpec.tableType + " not open. Skipping...");
            return;
        }
        formatter.format(format, dataItem);
    }

    public void addData(Object... data) {
        if(!isActive()) {
            Logger.getGlobal().severe("MetricsFile " + robotMetricsSpec.tableType + " not open. Skipping...");
            return;
        }
        formatter.format(robotMetricsSpec.format, data);
    }

    public void addData(double[] data) {
        if(!isActive()) {
            Logger.getGlobal().severe("MetricsFile " + robotMetricsSpec.tableType + " not open. Skipping...");
            return;
        }

        String formatWithSep = robotMetricsSpec.itemFormat + ",";
        int    dIdx          = 0;
        for(dIdx=0; dIdx<data.length-1; dIdx++)
            formatter.format(formatWithSep, data[dIdx]);
        formatter.format(robotMetricsSpec.itemFormat, data[dIdx]).format("%n");
    }

    public void addData(MetricsDataPoint dataPoint) {
        if(!isActive()) {
            Logger.getGlobal().severe("MetricsFile " + robotMetricsSpec.tableType + " not open. Skipping...");
            return;
        }
        dataPoint.writeMetrics(formatter);
    }

    public boolean isActive() {
        return formatter != null;
    }

    public void close() {
        if(isActive()) {
            formatter.flush();
            formatter.close();
            formatter = null;
        }
    }

    @NonNull
    @Override
    public String toString() {
        var sb = new StringBuilder();
        sb.append("MetricsFile\n");
        sb.append("  robotMetricsSpec=\n").append(robotMetricsSpec);
        sb.append("  formatter=")         .append(formatter)   .append("\n");
        sb.append("  fileId=")            .append(fileId)      .append("\n");
        sb.append("  fullFileName=")      .append(fullFileName).append("\n");

        return sb.toString();
    }
}
