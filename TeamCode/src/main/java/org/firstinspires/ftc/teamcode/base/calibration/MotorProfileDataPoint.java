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
                "%1$s,%2$s,%3$s,%4$s,%5$s,%6$d,%7$.5f,%8$b,%9$b,%10$b,%11$.3f,%12$.3f,%13$.3f,"    +
                        "%14$.3f,%15$.3f,%16$.3f,%17$.3f,%18$.3f,%19$.3f,%20$.3f,%21$.3f,"  +
                        "%22$.3f,%23$.3f,%24$.3f,%25$.3f,%26$d,%27$d,%28$d,%29$.3f,%30$.3f," +
                        "%31$.5f,%32$.5f,%33$.5f,%34$.5f,%35$.5f,%36$.5f,%37$.5f,%38$.5f,"   +
                        "%39$.5f,%40$.5f%n";

        MetricsDataPoint.fieldNames = new String[] {
                "MotorProfileState",    "MotionProfileState",                                 // 1-2
                "PowerStrategyState",   "FBControllerState",                                  // 3-4
                "Direction",            "PosTol",             "VelTol",                       // 5-7
                "isBusy",               "isTargetReached",    "isStrategyStopped",            // 8-10
                "mpVmax",               "mpAmax",             "mpDmax",                       // 11-13
                "timeToUltimateTarget", "dError",                                             // 14-15
                "Kp",                   "Ki",                 "Kd",                           // 16-18
                "Time",                 "TimeEndMP",                                          // 19-20
                "TimePExtract",         "TimeVextract",       "TimeCExtract",                 // 21-23
                "TimeOtherExtract",     "TimeCycle",                                          // 24-25
                "Position",             "UltimateTarget",     "ImmediateTarget",              // 26-28
                "minMovePower",         "powerP",             "powerI",          "powerD",    // 29-32
                "Power",                "Velocity",           "Vavg",                         // 33-35
                "A",                    "Aavg",               "ApredFun",        "ApredLut",  // 36-39
                "C"                                                                           // 40
        };
    }

    public        String           motorProfileState  = "";
    public        String           motionProfileState = "";
    public        String           powerStrategyState = "";
    public        String           fbControllerState  = "";
    public        Direction        direction;
    public        int              posTol;
    public        double           velTol;
    public        boolean          isBusy;
    public        boolean          isTargetReached;
    public        boolean          isStrategyStopped;
    public        double           mpVmax;
    public        double           mpAmax;
    public        double           mpDmax;
    public        double           timeToUltimateTarget;
    public        double           dError;
    public        double           Kp;
    public        double           Ki;
    public        double           Kd;
    public        double           t;
    public        double           tEndMP;
    public        double           tPextract;
    public        double           tVextract;
    public        double           tCextract;
    public        double           tOtherExtract;
    public        double           tCycle;
    public        int              P;
    public        int              ultimateTarget;
    public        int              immediateTarget;
    public        double           minMovePower;
    public        double           powerP;
    public        double           powerI;
    public        double           powerD;
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

    public         MotorProfileDataPoint(String    motorProfileState_in,
                                         Direction direction_in,
                                         double    t_in,
                                         double    tPextract_in,
                                         double    tVextract_in,
                                         double    tCextract_in,
                                         double    tOtherExtract_in,
                                         double    tCycle_in,
                                         int       P_in,
                                         int       ultimateTarget_in,
                                         int       immediateTarget_in,
                                         double    V_in,
                                         double    power_in,
                                         double    C_in,
                                         boolean   isBusy_in,
                                         int       posTol_in,
                                         double    velTol_in) {
        motorProfileState = motorProfileState_in;
        direction         = direction_in;
        t                 = t_in;
        tPextract         = tPextract_in;
        tVextract         = tVextract_in;
        tCextract         = tCextract_in;
        tOtherExtract     = tOtherExtract_in;
        tCycle            = tCycle_in;
        P                 = P_in;
        ultimateTarget    = ultimateTarget_in;
        immediateTarget   = immediateTarget_in;
        V                 = V_in;
        power             = power_in;
        C                 = C_in;
        isBusy            = isBusy_in;
        posTol            = posTol_in;
        velTol            = velTol_in;
    }
    public         MotorProfileDataPoint(DcMotorEx   motor,
                                         String      motorProfileState_in,
                                         Direction   direction_in,
                                         ElapsedTime timer,
                                         Double      minMovPower_in,
                                         boolean     extractOther) {
        // Because of the time it takes to pull data from the motor, there measurements
        // unfortunately are not exactly synchronous
        motorProfileState = motorProfileState_in;
        direction         = direction_in;
        t                 = timer.milliseconds();
        P                 = motor.getCurrentPosition();
        tPextract         = timer.milliseconds() - t;
        // Pull velocity info
        // With no arguments getVelocity() returns Ticks Per Second
        V                 = motor.getVelocity() / 1000.0;
        tVextract         = timer.milliseconds() - tPextract - t;
        C                 = motor.getCurrent(CurrentUnit.AMPS);
        power             = motor.getPower();
        tCextract         = timer.milliseconds() - tVextract - tPextract - t;
        if(extractOther) {
            pidfRTP       = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            pidfRUE       = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            isBusy        = motor.isBusy();
        }
        tOtherExtract     = timer.milliseconds() - tCextract - tVextract - tPextract - t;
        tCycle            = timer.milliseconds() - t;
        minMovePower      = minMovPower_in != null ? minMovPower_in : 0.0;
    }
    public void    setMotorProfileState(String motorProfileState_in) {
        motorProfileState   = motorProfileState_in;
    }
    public void    appendMotorProfileState(String appendState) {
        motorProfileState  += "-" + appendState;
    }
    public void    setMotionProfileState(String motionProfileState_in) {
        motionProfileState  = motionProfileState_in;
    }
    public void    appendMotionProfileState(String appendState) {
        motionProfileState += "-" + appendState;
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
    public boolean isSeeking(int Ptarget) {
        return isBusy || !isAtTarget(Ptarget) || isMoving();
    }
    public boolean isAtTarget(int Ptarget) {
        return abs(P - Ptarget) <= posTol;
    }
    public boolean isMoving() {
        return abs(V) > velTol;
    }
    public void    writeMetrics(Formatter formatter) {
        formatter.format(format,
                motorProfileState,    motionProfileState,
                powerStrategyState,   fbControllerState,
                direction,            posTol,             velTol,
                isBusy,               isTargetReached,    isStrategyStopped,
                mpVmax,               mpAmax,             mpDmax,
                timeToUltimateTarget, dError,
                Kp,                   Ki,                 Kd,
                t,                    tEndMP,
                tPextract,            tVextract,          tCextract,         tOtherExtract, tCycle,
                P,                    ultimateTarget,     immediateTarget,
                minMovePower,         powerP,             powerI,            powerD,
                power,                V,                  Vavg,
                A,                    Aavg,               ApredFun,          ApredLut,
                C
        );
    }
    @NonNull
    @Override
    public String  toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("MotorProfileDataPoint\n");

        sb.append("  motorProfileState=")   .append(motorProfileState)   .append("\n");
        sb.append("  motionProfileState=")  .append(motionProfileState)  .append("\n");
        sb.append("  powerStrategyState=")  .append(powerStrategyState)  .append("\n");
        sb.append("  fbControllerState=")   .append(fbControllerState)   .append("\n");
        sb.append("  direction=")           .append(direction)           .append("\n");
        sb.append("  posTol=")              .append(posTol)              .append("\n");
        sb.append("  velTol=")              .append(velTol)              .append("\n");
        sb.append("  isBusy=")              .append(isBusy)              .append("\n");
        sb.append("  isTargetReached=")     .append(isTargetReached)     .append("\n");
        sb.append("  isStrategyStopped=")   .append(isStrategyStopped)   .append("\n");
        sb.append("  mpVmax=")              .append(mpVmax)              .append("\n");
        sb.append("  mpAmax=")              .append(mpAmax)              .append("\n");
        sb.append("  mpDmax=")              .append(mpDmax)              .append("\n");
        sb.append("  timeToUltimateTarget=").append(timeToUltimateTarget).append("\n");
        sb.append("  dError=")              .append(dError)              .append("\n");
        sb.append("  Kp=")                  .append(Kp)                  .append("\n");
        sb.append("  Ki=")                  .append(Ki)                  .append("\n");
        sb.append("  Kd=")                  .append(Kd)                  .append("\n");
        sb.append("  t=")                   .append(t)                   .append("\n");
        sb.append("  tEndMP=")              .append(tEndMP)              .append("\n");
        sb.append("  tPextract=")           .append(tPextract)           .append("\n");
        sb.append("  tVextract=")           .append(tVextract)           .append("\n");
        sb.append("  tCextract=")           .append(tCextract)           .append("\n");
        sb.append("  tOtherExtract=")       .append(tOtherExtract)       .append("\n");
        sb.append("  tCycle=")              .append(tCycle)              .append("\n");
        sb.append("  P=")                   .append(P)                   .append("\n");
        sb.append("  ultimateTarget=")      .append(ultimateTarget)      .append("\n");
        sb.append("  immediateTarget=")     .append(immediateTarget)     .append("\n");
        sb.append("  minMovePower=")        .append(minMovePower)        .append("\n");
        sb.append("  powerP=")              .append(powerP)              .append("\n");
        sb.append("  powerI=")              .append(powerI)              .append("\n");
        sb.append("  powerD=")              .append(powerD)              .append("\n");
        sb.append("  power=")               .append(power)               .append("\n");
        sb.append("  V=")                   .append(V)                   .append("\n");
        sb.append("  Vavg=")                .append(Vavg)                .append("\n");
        sb.append("  A=")                   .append(A)                   .append("\n");
        sb.append("  Aavg=")                .append(Aavg)                .append("\n");
        sb.append("  ApredFun=")            .append(ApredFun)            .append("\n");
        sb.append("  ApredLut=")            .append(ApredLut)            .append("\n");
        sb.append("  C=")                   .append(C)                   .append("\n");

        return sb.toString();
    }

    public static MetricsFileSpec makeMetricsSpec(String fileId) {
        return MetricsDataPoint.makeMetricsFileSpec(fileId);
    }
    public static void            main(String[] args) {
        MetricsFileSpec ms = MotorProfileDataPoint.makeMetricsSpec("This File");
        System.out.println(ms);
    }
}
