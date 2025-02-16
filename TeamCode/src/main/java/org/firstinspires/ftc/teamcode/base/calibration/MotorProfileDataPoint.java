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

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import static java.lang.Math.abs;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.base.logging.MetricsDataPoint;
import org.firstinspires.ftc.teamcode.base.logging.MetricsFileSpec;

import java.util.Formatter;

public class MotorProfileDataPoint extends MetricsDataPoint {

    static {
        MetricsDataPoint.tableType  = "MotorProfileData";
        MetricsDataPoint.format     =
                "%1$s,%2$s,%3$d,%4$b,%5$b,%6$b,%7$.3f,%8$.3f,%9$.3f,%10$.3f,%11$.3f,%12$.3f,%13$.3f," +
                        "%14$.3f,%15$.3f,%16$.3f,%17$.3f,%18$.3f,%19$.3f,%20$.3f,%21$d,%22$d," +
                        "%23$.5f,%24$.5f,%25$.5f,%26$.5f,%27$.5f,%28$.5f,%29$.5f,%30$.5f%n";

        MetricsDataPoint.fieldNames = new String[] {
                "ProfileId",       "Direction",         "PosTol",       "isBusy",
                "isTargetReached", "isStrategyStopped",
                "RUE.Kp",          "RUE.Ki",            "RUE.Kd",       "RUE.Kf",
                "RTP.Kp",          "RTP.Ki",            "RTP.Kd",       "RTP.Kf",
                "Time",
                "TimePExtract",    "TimeVextract",      "TimeCExtract", "TimeOtherExtract", "TimeCycle",
                "Position",        "Target",            "Power",        "Velocity",         "Vavg",
                "A",               "Aavg",              "ApredFun",     "ApredLut",
                "C"
        };
    }

    public        String           profileId;
    public        Direction        direction;
    public        int              posTol;
    public        boolean          isBusy;
    public        boolean          isTargetReached;
    public        boolean          isStrategyStopped;
    public        double           t;
    public        double           tPextract;
    public        double           tVextract;
    public        double           tCextract;
    public        double           tOtherExtract;
    public        double           tCycle;
    public        int              P;
    public        int              target;
    public        double           power;
    public        double           V;
    public        double           Vavg;
    public        double           A;
    public        double           Aavg;
    public        double           ApredFun;
    public        double           ApredLut;
    public        double           C;

    public        PIDFCoefficients pidfRUE;
    public        PIDFCoefficients pidfRTP;

    public MotorProfileDataPoint(
            String    profileId_in,
            Direction direction_in,
            double    t_in,
            double    tPextract_in,
            double    tVextract_in,
            double    tCextract_in,
            double    tOtherExtract_in,
            double    tCycle_in,
            int       P_in,
            int       target_in,
            double    V_in,
            double    power_in,
            double    C_in,
            boolean   isBusy_in,
            int       posTol_in) {
        profileId         = profileId_in;
        direction         = direction_in;
        t                 = t_in;
        tPextract         = tPextract_in;
        tVextract         = tVextract_in;
        tCextract         = tCextract_in;
        tOtherExtract     = tOtherExtract_in;
        tCycle            = tCycle_in;
        P                 = P_in;
        target            = target_in;
        V                 = V_in;
        power             = power_in;
        C                 = C_in;
        isBusy            = isBusy_in;
        posTol            = posTol_in;
    }

    public         MotorProfileDataPoint(DcMotorEx   motor,
                                         String      profileId_in,
                                         Direction   direction_in,
                                         int         target_in,
                                         ElapsedTime timer,
                                         boolean     extractOther) {
        // Because of the time it takes to pull data from the motor, there measurements
        // unfortunately are not exactly synchronous
        profileId     = profileId_in;
        direction     = direction_in;
        t             = timer.milliseconds();
        P             = motor.getCurrentPosition();
        target        = target_in;
        tPextract     = timer.milliseconds() - t;
        // Pull velocity info
        // With no arguments getVelocity() returns Ticks Per Second
        V             = motor.getVelocity() / 1000.0;
        tVextract     = timer.milliseconds() - tPextract - t;
        C             = motor.getCurrent(CurrentUnit.AMPS);
        tCextract     = timer.milliseconds() - tVextract - tPextract - t;
        if(extractOther) {
            power     = motor.getPower();
            pidfRTP   = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            pidfRUE   = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            isBusy    = motor.isBusy();
        }
        tOtherExtract = timer.milliseconds() - tCextract - tVextract - tPextract - t;
        tCycle        = timer.milliseconds() - t;
    }

    public void    setProfileId(String profileId_in) {
        profileId = profileId_in;
    }

    public boolean isTargetReached() {
        return isTargetReached;
    }

    public boolean isTargetReached(int Ptarget) {
        return direction == Direction.FORWARD ? P>=Ptarget : P<=Ptarget;
    }

    public void    setIsTargetReached(boolean isTargetReached_in) {
        isTargetReached = isTargetReached_in;
    }

    public boolean isStrategyStopped() {
        return isStrategyStopped;
    }

    public void    setIsStrategyStopped(boolean isStrategyStopped_in) {
        isStrategyStopped = isStrategyStopped_in;
    }

    public boolean isSeeking(int Ptarget, int posTol, double velTol) {
        return isBusy || !isAtTarget(Ptarget, posTol) || isMoving(velTol);
    }

    public boolean isAtTarget(int Ptarget, int posTol) {
        return abs(P - Ptarget) <= posTol;
    }

    public boolean isMoving(double velTol) {
        return abs(V) > velTol;
    }

    public static MetricsFileSpec makeMetricsSpec(String fileId) {
        return MetricsDataPoint.makeMetricsSpec(fileId);
    }

    public void    writeMetrics(Formatter formatter) {
        formatter.format(format,
                profileId,       direction,         posTol,    isBusy,
                isTargetReached, isStrategyStopped,
                pidfRUE.p,       pidfRUE.i,         pidfRUE.d, pidfRUE.f,
                pidfRTP.p,       pidfRTP.i,         pidfRTP.d, pidfRTP.f,
                t,               tPextract,         tVextract, tCextract, tOtherExtract, tCycle,
                P,               target,            power,     V,         Vavg,          A,
                Aavg,            ApredFun,          ApredLut,  C
        );
    }

    @NonNull
    @Override
    public String  toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("MotorProfileDataPoint\n");

        sb.append("  profileId=")        .append(profileId)        .append("\n");
        sb.append("  direction=")        .append(direction)        .append("\n");
        sb.append("  posTol=")           .append(posTol)           .append("\n");
        sb.append("  isBusy=")           .append(isBusy)           .append("\n");
        sb.append("  isTargetReached=")  .append(isTargetReached)  .append("\n");
        sb.append("  isStrategyStopped=").append(isStrategyStopped).append("\n");
        sb.append("  t=")                .append(t)                .append("\n");
        sb.append("  tPextract=")        .append(tPextract)        .append("\n");
        sb.append("  tVextract=")        .append(tVextract)        .append("\n");
        sb.append("  tCextract=")        .append(tCextract)        .append("\n");
        sb.append("  tOtherExtract=")    .append(tOtherExtract)    .append("\n");
        sb.append("  tCycle=")           .append(tCycle)           .append("\n");
        sb.append("  P=")                .append(P)                .append("\n");
        sb.append("  target=")           .append(target)           .append("\n");
        sb.append("  power=")            .append(power)            .append("\n");
        sb.append("  V=")                .append(V)                .append("\n");
        sb.append("  Vavg=")             .append(Vavg)             .append("\n");
        sb.append("  A=")                .append(A)                .append("\n");
        sb.append("  Aavg=")             .append(Aavg)             .append("\n");
        sb.append("  ApredFun=")         .append(ApredFun)         .append("\n");
        sb.append("  ApredLut=")         .append(ApredLut)         .append("\n");
        sb.append("  C=")                .append(C)                .append("\n");

        return sb.toString();
    }

    public static void main(String[] args) {
        MetricsFileSpec ms = MotorProfileDataPoint.makeMetricsSpec("This File");
        System.out.println(ms);
    }
}
