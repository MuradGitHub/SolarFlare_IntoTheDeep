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

import androidx.annotation.NonNull;

import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Reader;
import java.io.Writer;
import java.util.Map;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import org.firstinspires.ftc.teamcode.base.config.Application;
import org.firstinspires.ftc.teamcode.base.config.JSONWritable;

/*
Code snippets for gson

- Serializing an object into JSON
Gson gson = new GsonBuilder().setPrettyPrinting().create();
String jsonString = gson.toJson(object);

 */

public class JSONUtils {
    public static <T> T parseJSON(Reader input, Class<T> contentsClass) {
        Gson gson              = new GsonBuilder()
                .serializeSpecialFloatingPointValues()
                .create();
        return gson.fromJson(input, contentsClass);
    }
    public static <T extends JSONWritable> void writeJSONThreaded(T obj) {
        Thread      thread     = Thread.currentThread();
        ThreadGroup group      = thread.getThreadGroup();
        Thread      jsonThread = new Thread(
                group,
                () -> {
                        String fileName = obj.getClass().getSimpleName() + "-" + obj.getJSONFileId() + ".json";
                        String fullFileName = Application.getMetricsDirName() + "/" + fileName;
                        try (Writer writer = new FileWriter(fullFileName)) {
                            Gson gson = new GsonBuilder()
                                    .setPrettyPrinting()
                                    .serializeSpecialFloatingPointValues()
                                    .create();
                            gson.toJson(obj, writer);
                        } catch (IOException e) {
                            throw new RuntimeException(e);
                        }
                    },
                "JSONThread",
                2*1024*1024);

        jsonThread.start();

        try {
            thread.join();
        } catch(InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public static <T extends JSONWritable> void writeJSON(T obj) {
        String fileName     = obj.getClass().getSimpleName() + "-" + obj.getJSONFileId() + ".json";
        String fullFileName = Application.getMetricsDirName() + "/" + fileName;
        try (Writer writer = new FileWriter(fullFileName)) {
            Gson gson = new GsonBuilder()
                    .setPrettyPrinting()
                    .serializeSpecialFloatingPointValues()
                    .create();
            gson.toJson(obj, writer);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    /***
     * Selected testing cases
     */
    public static class JsonTest {
        public enum MotorEnumTest {M1, M2}
        public enum ServoEnumTest {S1, S2}
        public static class MotorTest {
            public int           id = 1;
            public Object        objId;
            public MotorEnumTest motorEnum;
            public String        k1;
            public int           k2;
            @NonNull
            public String toString() {
                var sb = new StringBuilder();
                sb.append("MotorTest\n");
                sb.append("  id=       ").append(id).append("\n");
                sb.append("  objId=    ").append(objId).append("\n");
                sb.append("  motorEnum=").append(motorEnum).append("\n");
                sb.append("  k1=       ").append(k1).append("\n");
                sb.append("  k2=       ").append(k2).append("\n");
                return sb.toString();
            }
        }
        public static class ServoTest {
            public int           id = 10;
            public Object        objId;
            public ServoEnumTest servoEnum;
            public String        k1;
            public int           k2;
            @NonNull
            public String toString() {
                var sb = new StringBuilder();
                sb.append("ServoTest\n");
                sb.append("  id=       ").append(id).append("\n");
                sb.append("  objId=    ").append(objId).append("\n");
                sb.append("  servoEnum=").append(servoEnum).append("\n");
                sb.append("  k1=       ").append(k1).append("\n");
                sb.append("  k2=       ").append(k2).append("\n");
                return sb.toString();
            }
        }

        public String                        title;
        public int                           version;
        public Map<MotorEnumTest, MotorTest> motors;
        public Map<ServoEnumTest, ServoTest> servos;

        @NonNull
        public String toString() {
            var sb = new StringBuilder();
            sb.append("title=").append(title).append("\n");
            sb.append("version=").append(version).append("\n");
            sb.append("motors").append("\n");
            for(var entry: motors.entrySet())
                sb.append(entry.getKey()).append("=").append(entry.getValue().toString());
            sb.append("servos");
            for(var entry: servos.entrySet())
                sb.append(entry.getKey()).append("=").append(entry.getValue().toString());
            return sb.toString();
        }
    }
    public static void main(String[] args) {
        InputStream input    = Application.getResourceAsStream("JSONTest.json");
        Reader      reader   = new InputStreamReader(input);
        JsonTest    contents = parseJSON(reader, JsonTest.class);
        System.out.println(contents.toString());
    }
}
