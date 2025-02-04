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

import org.firstinspires.ftc.teamcode.base.config.Application;

import java.util.Arrays;
import java.util.Locale;

public class RobotMetricsSpec implements Comparable<RobotMetricsSpec> {
    public String   tableType;
    public String   format;
    public String[] fieldNames;

    public RobotMetricsSpec(String tableType_in, String format_in, String[] fieldNames_in) {
        this.tableType  = tableType_in;
        this.format     = format_in;
        this.fieldNames = fieldNames_in.clone();
    }

    public int compareTo(RobotMetricsSpec other) {
        return tableType.compareTo(other.tableType);
    }

    public String getHeader() {
        return String.join(",", fieldNames);
    }

    public String getFileName(String id) {
        return String.format(Locale.US, "%1$s-%2$s.csv", tableType, id);
    }

    public String getFullFileName(String id) {
        return Application.getMetricsDirName() + "/" + getFileName(id);
    }

    @NonNull
    @Override
    public String toString() {
        var sb = new StringBuilder();
        sb.append("RobotMetricsSpec\n");
        sb.append("  tableType=").append(tableType)                  .append("\n");
        sb.append("  format=")   .append(format)                     .append("\n");
        sb.append("  fields")    .append(Arrays.toString(fieldNames)).append("\n");

        return sb.toString();
    }
}
