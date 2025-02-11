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

import java.util.SortedMap;
import java.util.TreeMap;

import org.firstinspires.ftc.teamcode.base.config.MissingDataException;

public class RobotMetrics {
    private static RobotMetrics instance;

    private final SortedMap<String, MetricsSpec> tablesSpecs = new TreeMap<>();

    public static RobotMetrics getInstance() {
        if(instance == null)
        {
            instance = new RobotMetrics();
            instance.initialize();
        }

        return instance;
    }

    private void initialize() {
        initializeTablesFormats();
    }

    @SuppressWarnings("SpellCheckingInspection")
    private void initializeTablesFormats() {
        /// MotionProfile files
        tablesSpecs.put("MotionProfile",
                new MetricsSpec(
                        "MotionProfile",
                        "%1$d,%2$d,%3$d,%4$.5f,%5$.5f,%6$.5f%n",
                new String[] {"iteration",
                        "targetMotorPosition",
                        "motorPosition",
                        "targetMotorPower",
                        "motorPower",
                        "motorVelocity"}));
        /// MotorProfileConstP
        tablesSpecs.put("MotorProfileConstP",
                new MetricsSpec(
                        "MotorProfileConstP",
                        "%1$.5f,%2$.5f,%3$.5f,%4$.5f,%5$.5f,%6$d,%7$.5f,%8$.5f,%9$.5f,%10$.5f,%11$.5f,%12$.5f%n",
                        new String[] {
                                "Time",
                                "tPextract",
                                "tVextract",
                                "tCextract",
                                "tCycle",
                                "Position",
                                "Velocity",
                                "Vavg",
                                "Acceleration",
                                "Aavg",
                                "Current",
                                "Power"}));

        tablesSpecs.put("MotionControl",
                new MetricsSpec(
                        "MotorControl",
                "%1$d,%2$d,%3$.5f,%4$.5f,%5$.5f,%6$.5f%n",
                new String[] {"iteration",
                        "targetMotorPosition",
                        "motorPosition",
                        "targetMotorPower",
                        "motorPower",
                        "motorVelocity"}));
    }

    public MetricsFile getMetricsFile(String tableType, String fileId) {
        MetricsSpec metricsSpec = tablesSpecs.get(tableType);
        if(metricsSpec == null)
            throw new MissingDataException("No MetricsSpec for TableType: " + tableType);

        return new MetricsFile(metricsSpec, fileId);
    }

    public MetricsFile getMetricsFile(MetricsFileSpec metricsSpec) {
        tablesSpecs.put(metricsSpec.getMetricsTableType(), metricsSpec);
        return new MetricsFile(metricsSpec, metricsSpec.getMetricsFileId());
    }

    public MetricsFile getMetricsFile(MetricsWritable obj) {
        return getMetricsFile(obj.getMetricsTableType(), obj.getMetricsFileId());
    }
}
