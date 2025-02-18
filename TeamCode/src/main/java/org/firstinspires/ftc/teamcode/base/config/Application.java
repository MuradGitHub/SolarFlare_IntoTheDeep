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
package org.firstinspires.ftc.teamcode.base.config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Objects;

@SuppressWarnings("SpellCheckingInspection")
public class Application {
    public static Telemetry   telemetry;
    public static HardwareMap hardwareMap;

    public static void        init(LinearOpMode opMode) {
        telemetry     = opMode.telemetry;;
        hardwareMap   = opMode.hardwareMap;
    }
    /**
     * sleep is a utility function that catches the InterruptedExcption and converts this
     * into a RunTimeException hence avoiding having to do that in the application code
     */
    public static void        sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch(Exception e) {
            throw new RuntimeException(e);
        }
    }
    /**
     * Returns Metrics directory name. Select based on context
     * This is the directory on the Robot Controller where metrics files are going to be saved
     * This should switch to a testing directory in a robot-less development environment
     * Output directory from persistentDataPath on Android
     * Application.persistentDataPath points to /storage/emulated/<userid>/Android/data/<packagename>/files
     * @return Metrics directory name.
     */
    public static String      getMetricsDirName() {
        if(Objects.requireNonNull(System.getProperty("os.name")).contains("Windows"))
            return "C:/Temp";
        else
            return "/storage/emulated/0/Android/data/com.qualcomm.ftcrobotcontroller/files/";
    }
    public static InputStream getResourceAsStream(String resourceName) {
        InputStream resource = Application.class.getResourceAsStream(resourceName);
        if(resource == null) {
            String resourcePath = getApplicationResourcesPath(resourceName);
            try {
                resource        = new FileInputStream(resourcePath);
            } catch(IOException e) {
                return null;
            }
        }
        return resource;
    }
    public static String      getApplicationResourcesPath() {
        String        packageName  = Objects.requireNonNull(Application.class.getPackage()).getName();
        String[]      packageDirs  = packageName.split("\\.");
        StringBuilder resourcesDir = new StringBuilder();
        resourcesDir.append(System.getProperty("user.dir"));
        resourcesDir.append("/TeamCode/src/main/resources/");
        for(String packageDir: packageDirs) {
            resourcesDir.append(packageDir).append("/");
        }
        return resourcesDir.toString();
    }
    public static String      getApplicationResourcesPath(String fileName) {
        return getApplicationResourcesPath() + fileName;
    }
    public static void        main(String[] args) {
        String   packageName = Objects.requireNonNull(Application.class.getPackage()).getName();
        String[] robotNames  = new String[] {"IntoTheDeep-V2", "Rig1Motor"};
        System.out.println("Package is:            " + packageName);
        System.out.println("Resources Path:        " + getApplicationResourcesPath());
        System.out.println("user.dir:              " + System.getProperty("user.dir"));
        System.out.println("os.name:               " + System.getProperty("os.name"));
        System.out.println("metricsDirName:        " + getMetricsDirName());
        System.out.println("Checking RobotConfig files");
        for(String robotName: robotNames) {
            String      robotConfigFileName = robotName + ".json";
            try(InputStream robotConfigFile = getResourceAsStream(robotConfigFileName)) {
                System.out.println("Obtained RobotConfigFile: " + robotConfigFile + " from " + robotConfigFileName);
            } catch(Exception e) {
                throw new RuntimeException(e);
            }

        }
    }
}
