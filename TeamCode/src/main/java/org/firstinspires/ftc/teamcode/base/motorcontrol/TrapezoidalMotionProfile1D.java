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
package org.firstinspires.ftc.teamcode.base.motorcontrol;

import static java.lang.Math.abs;
import static java.lang.Math.signum;
import static java.lang.Math.round;

import java.util.Locale;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;

import static org.firstinspires.ftc.teamcode.base.math.Math.solveQuadraticEquation;
import static org.firstinspires.ftc.teamcode.base.math.Math.approxEquals;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.base.error.CalculationException;
import org.firstinspires.ftc.teamcode.base.math.ComplexNumberPair;
import org.firstinspires.ftc.teamcode.base.logging.MetricsWritable;
import org.firstinspires.ftc.teamcode.base.regtest.RegTest;

@SuppressWarnings({"SpellCheckingInspection"})
public class TrapezoidalMotionProfile1D implements MotionProfile, MetricsWritable {
    /**
     * initialized: Has this profile been initialized?
     */
    private boolean   initialized = false;
    /**
     * FTC Dashboard Telemetry
     */
    private Telemetry telemetryDash;
    /**
     * Lead-in Profile. To be used if:
     *   1- signum(Vi) == -signum(dist) or
     *   2- Vi will overshoot distance even if maximum deceleration was applied immediately
     */
    private TrapezoidalMotionProfile1D leadInProfile = null;
    /**
     * Distance to travel
     */
    private double    dist;
    /**
     * Initial Position
     */
    private double    Pi;
    /**
     * Initial Velocity
     */
    private double    Vi;
    /**
     * Is Vi so high that the object will overshoot the target despite
     * the application of immediate max deceleration
     */
    private boolean   overshoot;
    /**
     * Maximum Velocity
     */
    private double    Vmax;
    /**
     * Initial brake phase - Tb time to brake speed down to Vmax
     */
    private double    Tb;
    /**
     * Initial brake phase - Sb distance to brake speed down to Vmax
     */
    private double    Sb;
    /**
     * Maximum Acceleration
     */
    private double    Amax;
    /**
     * Acceleration Time
     */
    private double    Ta;
    /**
     * Span (distance) while accelerating
     */
    private double    Sa;
    /**
     * Top Velocity for the triangular case
     */
    private double    Vt;
    /**
     * Cruise Velocity
     */
    private double    Vc;
    /**
     * Cruise Time
     */
    private double    Tc;
    /**
     * Span (distance) cruising
     */
    private double    Sc;
    /**
     * Max Deceleration
     */
    private double    Dmax;
    /**
     * Deceleration Time
     */
    private double    Td;
    /**
     * Span (distance) decelerating
     */
    private double    Sd;
    /**
     * Total Time
     */
    private double    Tt;

    public TrapezoidalMotionProfile1D getLeadInProfile() {
        if(leadInProfile == null)
            leadInProfile = new TrapezoidalMotionProfile1D();
        return leadInProfile;
    }

    public boolean isInitialized() {
        return initialized;
    }
    public void setInitialized() { initialized = true; }

    public TrapezoidalMotionProfile1D reset() {
        initialized             = false;
        telemetryDash           = null;
        leadInProfile           = new TrapezoidalMotionProfile1D();
        dist                    = 0.0;
        Pi                      = 0.0;
        Vi                      = 0.0;
        overshoot               = false;
        Vmax                    = 0.0;
        Tb                      = 0.0;
        Sb                      = 0.0;
        Amax                    = 0.0;
        Ta                      = 0.0;
        Sa                      = 0.0;
        Vt                      = 0.0;
        Vc                      = 0.0;
        Tc                      = 0.0;
        Sc                      = 0.0;
        Dmax                    = 0.0;
        Td                      = 0.0;
        Sd                      = 0.0;
        Tt                      = 0.0;

        return this;
    }

    protected Telemetry getTelemetryDash() {
        if(telemetryDash == null)
            telemetryDash = FtcDashboard.getInstance().getTelemetry();
        return telemetryDash;
    }

    /**
     * All parameters are positive,
     *  except dist, Vinit can be either negative or positive
     *  We adjust the signs of Vmax, Amax, and Dmax to direct motion towards dist
     *  Consider the possibility of Vi <= Vmax
     * @param Amax_in  Max Acceleration
     * @param Vmax_in  Max Velocity
     * @param Vi_in Initial Velocity
     * @param dist_in  Distance to travel
     * @param Pi_in Initial Position
     */
    public void calcProfile(double dist_in,
                            double Pi_in,
                            double Vi_in,
                            double Vmax_in,
                            double Amax_in,
                            double Dmax_in)
    {
        initialized             = true;
        dist                    = dist_in;
        Pi                      = Pi_in;
        Vi                      = Vi_in;
        Vmax                    = signum(dist)*abs(Vmax_in);
        Amax                    = signum(dist)*abs(Amax_in);
        Dmax                    = -1*signum(dist)*abs(Dmax_in);

        // System.out.println("dist="+dist+" Pi="+Pi+" Vi="+Vi+" Vmax="+Vmax+" Amax="+Amax+" Dmax="+Dmax);

        if(leadInProfile == null)
            leadInProfile       = getLeadInProfile();

        if((Vmax == 0) || (Amax == 0) || (Dmax == 0)) {
            String errorMsg = "TrapezoidalMotionProfile1D.calcProfile()";
            errorMsg += Vmax == 0 ? " - Vmax == 0" : "";
            errorMsg += Amax == 0 ? " - Amax == 0" : "";
            errorMsg += Dmax == 0 ? " - Dmax == 0" : "";
            throw new CalculationException(errorMsg);
        }

        /// Overshoot test: is Vi so high that it would overshoot the target even
        /// with max deceleration applied
        if(abs(Vi) > 0 && signum(Vi) == signum(dist)) {
            /// Time to zero velocity
            double Tz           = abs(Vi / Dmax);
            /// Distance to zero velocity
            double distZ        = Vi * Tz + 0.5 * Dmax * Tz * Tz;
            overshoot           = abs(distZ) > abs(dist);
            if (overshoot) {
                leadInProfile.calcProfile(distZ, Pi, Vi, Vmax, Amax, Dmax);
                Pi             += distZ;
                dist           -= distZ;
                Vi              = 0;
                Vmax            = signum(dist)*abs(Vmax);
                Amax            = signum(dist)*abs(Amax);
                Dmax            = -signum(dist)*abs(Dmax);
            }
        }

        if((signum(Vi) == signum(Vmax)) && (abs(Vi) > abs(Vmax))) {
            /// decelerate first to get Vi down to Vmax then proceed with the normal calculation
            /// No need to adjust Pi
            Tb                  = abs((Vi-Vmax)/Dmax);
            Sb                  = Vi*Tb + 0.5*Dmax*Tb*Tb;
            dist               -= Sb;
            Vi                  = Vmax;
        } else {
            Tb                  = 0.0;
            Sb                  = 0.0;
        }

        // Solve for Triangular Motion Profile first
        // Vc will be assigned the direction (sign) of distance to travel
        // Vt is a solution to the following quadratic equation aX^2 + bX + c = 0
        double            a     = Dmax - Amax;
        double            b     = 0;
        double            c     = -Dmax*(Vi*Vi + 2*dist*Amax);
        ComplexNumberPair roots = solveQuadraticEquation(a, b, c);
        if(roots.n1.isComplex() || roots.n2.isComplex()) {
            System.out.println("Vi="+Vi+" dist="+dist+" Amax="+Amax+" Dmax="+Dmax);
            throw new CalculationException("Could not solve for cruise velocity");
        } else if(signum(roots.n1.a) != signum(roots.n2.a)) {
            Vt                  = signum(roots.n1.a) == signum(dist) ? roots.n1.a : roots.n2.a;
        } else if(roots.n1.compareTo(roots.n2) >= 0) {
            Vt                  = roots.n1.a;
        } else {
            Vt                  = roots.n2.a;
        }

        Vc                      = abs(Vt)<abs(Vmax) ? Vt : Vmax;

        Ta                      = abs((Vc-Vi)/Amax);
        Td                      = abs(Vc/Dmax);
        Sa                      = Vi*Ta + 0.5*Amax*Ta*Ta;
        Sd                      = 0.5*Vc*Td;
        Sc                      = dist - Sa - Sd;
        Tc                      = Sc/Vc;
        Tt                      = Tb + Ta + Tc + Td;

        /*
        telemetryDash = getTelemetryDash();
        telemetryDash.addData("Pi", Pi);
        telemetryDash.addData("Vi", Vi);
        telemetryDash.addData("distance", dist);
        telemetryDash.addData("Max Velocity", Vmax);
        telemetryDash.addData("Cruise Velocity", Vc);
        telemetryDash.addData("Accel Span", Sa);
        telemetryDash.addData("Cruise Span", Sc);
        telemetryDash.addData("Decel Span", Sd);
        telemetryDash.update();
         */
    }

    // Run this method in a loop
    public int runProfile(double t) {
        if(leadInProfile == null)
            throw new CalculationException("TrapezoidalMotionProfile1D not initialized");

        if(t < leadInProfile.Tt) {
            return leadInProfile.runProfile(t);
        } if(t < leadInProfile.Tt+Tb) {
            double tInB         = t-leadInProfile.Tt;
            return (int) round(Pi + Vi * tInB + 0.5 * Dmax*tInB*tInB);
        } else if (t < leadInProfile.Tt+Tb+Ta) {
            double tInA         = t-leadInProfile.Tt-Tb;
            return (int) round(Pi + Sb + Vi * tInA + 0.5 * Amax*tInA*tInA);
        } else if (t < leadInProfile.Tt+Tb+Ta+Tc) {
            double tInC         = t-leadInProfile.Tt-Tb-Ta;
            return (int) round(Pi + Sb + Sa + Vc * tInC);
        } else if (t < Tt){
            double tInD         = t-leadInProfile.Tt-Tb-Ta-Tc;
            return (int) round(Pi + Sb + Sa + Sc + Vc*tInD + 0.5*Dmax*tInD*tInD);
        }  else {
            return (int) round(Pi + dist);
        }
    }

    public String getMetricsFileId() {
        return String.format(Locale.US,"%1$.2f-%2$.2f-%3$.2f-%4$.2f-%5$.2f-",Pi,Vi,Vmax,Amax,Dmax);
    }

    public String getMetricsTableType() {
        return "TrapezoidalMotionProfile1D";
    }

    public boolean approxEqual(TrapezoidalMotionProfile1D other) {
        if(other == null)
            throw new CalculationException("TrapezoidalMotionProfile1D.approxEqual: other is null");

        boolean invalidLeadInProfileThis  =
                leadInProfile       == null || !leadInProfile.isInitialized();
        boolean invalidLeadInProfileOther =
                other.leadInProfile == null || !other.leadInProfile.isInitialized();

        boolean leadInTest   = true;
        if(!invalidLeadInProfileThis && !invalidLeadInProfileOther)
            leadInTest       = leadInProfile.approxEqual(other.leadInProfile);
        else if(invalidLeadInProfileThis ^ invalidLeadInProfileOther)
            leadInTest       = false;

        boolean overshootTest = overshoot == other.overshoot;
        boolean distTest      = approxEquals(dist,     other.dist);
        boolean PiTest        = approxEquals(Pi,       other.Pi);
        boolean ViTest        = approxEquals(Vi,       other.Vi);
        boolean VmaxTest      = approxEquals(Vmax,     other.Vmax);
        boolean TbTest        = approxEquals(Tb,       other.Tb);
        boolean SbTest        = approxEquals(Sb,       other.Sb);
        boolean AmaxTest      = approxEquals(Amax,     other.Amax);
        boolean TaTest        = approxEquals(Ta,       other.Ta);
        boolean SaTest        = approxEquals(Sa,       other.Sa);
        boolean VtTest        = approxEquals(Vt,       other.Vt);
        boolean VcTest        = approxEquals(Vc,       other.Vc);
        boolean TcTest        = approxEquals(Tc,       other.Tc);
        boolean ScTest        = approxEquals(Sc,       other.Sc);
        boolean DmaxTest      = approxEquals(Dmax,     other.Dmax);
        boolean TdTest        = approxEquals(Td,       other.Td);
        boolean SdTest        = approxEquals(Sd,       other.Sd);
        boolean TtTest        = approxEquals(Tt,       other.Tt);

        if(!leadInTest)
            System.out.println("Failed leadInTest");
        if(!distTest)
            System.out.println("Failed distTest");
        if(!PiTest)
            System.out.println("Failed PiTest");
        if(!ViTest)
            System.out.println("Failed ViTest");
        if(!overshootTest)
            System.out.println("Failed overshootTest");
        if(!VmaxTest)
            System.out.println("Failed VmaxTest");
        if(!TbTest)
            System.out.println("Failed TbTest");
        if(!SbTest)
            System.out.println("Failed SbTest");
        if(!AmaxTest)
            System.out.println("Failed AmaxTest");
        if(!TaTest)
            System.out.println("Failed TaTest");
        if(!SaTest)
            System.out.println("Failed SaTest");
        if(!VtTest)
            System.out.println("Failed VtTest");
        if(!VcTest)
            System.out.println("Failed VcTest");
        if(!TcTest)
            System.out.println("Failed TcTest");
        if(!ScTest)
            System.out.println("Failed ScTest");
        if(!DmaxTest)
            System.out.println("Failed DmaxTest");
        if(!TdTest)
            System.out.println("Failed TdTest");
        if(!SdTest)
            System.out.println("Failed SdTest");
        if(!TtTest)
            System.out.println("Failed TtTest");

        return  leadInTest && overshootTest && distTest && PiTest   && ViTest && VmaxTest &&
                TbTest     && SbTest        && AmaxTest && TaTest   && SaTest && VtTest   &&
                VcTest     && TcTest        && ScTest   && DmaxTest && TdTest && SdTest   &&
                TtTest;
    }

    @NonNull
    @Override
    public String toString() {
        String formatString =
                "    dist      = %1$.5f%n"  +
                "    Pi        = %2$.5f%n"  +
                "    Vi        = %3$.5f%n"  +
                "    overshoot = %4$b%n"    +
                "    Vmax      = %5$.5f%n"  +
                "    Tb        = %6$.5f%n"  +
                "    Sb        = %7$.5f%n"  +
                "    Amax      = %8$.5f%n"  +
                "    Ta        = %9$.5f%n"  +
                "    Sa        = %10$.5f%n" +
                "    Vt        = %11$.5f%n" +
                "    Tc        = %12$.5f%n" +
                "    Vc        = %13$.5f%n" +
                "    Sc        = %14$.5f%n" +
                "    Dmax      = %15$.5f%n" +
                "    Td        = %16$.5f%n" +
                "    Sd        = %17$.5f%n" +
                "    Tt        = %18$.5f%n";

        String str = "TrapezoidalMotionProfile1D\n";
        if(leadInProfile.isInitialized())
            str   += "leadInProfile:\n" + leadInProfile.toString();
        str       += "mainProfile:\n"   + String.format(Locale.US, formatString,
                dist, Pi,Vi,overshoot,Vmax,Tb,Sb,Amax,Ta,Sa,Vt,Tc,Vc,Sc,Dmax,Td,Sd,Tt);

        return str;
    }

    /**
     * use to test example motion profiles
     * @param args not used
     */
    public static void main(String[] args) {
        TrapezoidalMotionProfile1D profile = new TrapezoidalMotionProfile1D();
        TrapezoidalMotionProfile1D test    = new TrapezoidalMotionProfile1D();
        test.reset().setInitialized();

        String  testDesc;
        boolean testResult;
        //
        // Profile variables, initialized to
        testDesc               = "Profile 1 - Positive Distance - Trinagular Profile - No Cruise";
        test.dist              =  27.0;     // m
        test.Pi                =   0;       // m
        test.Vi                =   0.0;     // m/s
        test.overshoot         = false;     // boolean
        test.Vmax              =   6.0;     // m/s
        test.Tb                =   0.0;     // s
        test.Sb                =   0.0;     // m
        test.Amax              =   2.0;     // m/s^2
        test.Ta                =   3.0;     // a
        test.Sa                =   9.0;     // m (0.5*Amax*Ta*Ta)
        test.Vt                =   6.0;     // m/s
        test.Tc                =   0.0;     // s
        test.Vc                =   6.0;     // m/s
        test.Sc                =   0.0;     // m
        test.Dmax              =  -1.0;     // m/s^2
        test.Td                =   6.0;     // s (Vc/Td)
        test.Sd                =  18.0;     // -0.5*Dmax*Td*Td
        test.Tt                =   9.0;     // s

        profile.calcProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        testResult             = profile.approxEqual(test);
        RegTest.report(testDesc, testResult);
        if(!testResult)
            System.out.println(profile + "\n");

        testDesc               = "Profile 2 - Negative Distance + Triangular Profile - No Cruise";
        test.dist              = -27.0;     // m
        test.Pi                =   0;       // m
        test.Vi                =   0.0;     // m/s
        test.overshoot         = false;     // boolean
        test.Tb                =   0.0;     // s
        test.Sb                =   0.0;     // m
        test.Vmax              =  -6.0;     // m/s
        test.Amax              =  -2.0;     // m/s^2
        test.Ta                =   3.0;     // a
        test.Sa                =  -9.0;     // m (0.5*Amax*Ta*Ta)
        test.Vt                =  -6.0;     // m/s
        test.Tc                =   0.0;     // s
        test.Vc                =  -6.0;     // m/s
        test.Sc                =   0.0;     // m
        test.Dmax              =   1.0;     // m/s^2
        test.Td                =   6.0;     // s (Vc/Td)
        test.Sd                = -18.0;     // -0.5*Dmax*Td*Td
        test.Tt                =   9.0;     // s

        profile.calcProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        testResult             = profile.approxEqual(test);
        RegTest.report(testDesc, testResult);
        if(!testResult)
            System.out.println(profile + "\n");

        testDesc               = "Profile 3 - Negative Distance + Cruise";
        test.dist              = -39.0;     // m
        test.Pi                =   0;       // m
        test.Vi                =   0.0;     // m/s
        test.overshoot         = false;     // boolean
        test.Vmax              =  -6.0;     // m/s
        test.Tb                =   0.0;     // s
        test.Sb                =   0.0;     // m
        test.Amax              =  -2.0;     // m/s^2
        test.Ta                =   3.0;     // a
        test.Sa                =  -9.0;     // m (0.5*Amax*Ta*Ta)
        test.Vt                =  -7.21110; // m/s
        test.Tc                =   2.0;     // s
        test.Vc                =  -6.0;     // m/s
        test.Sc                = -12.0;     // m
        test.Dmax              =   1.0;     // m/s^2
        test.Td                =   6.0;     // s (Vc/Td)
        test.Sd                = -18.0;     // -0.5*Dmax*Td*Td
        test.Tt                =  11.0;     // s

        profile.calcProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        testResult             = profile.approxEqual(test);
        RegTest.report(testDesc, testResult);
        if(!testResult)
            System.out.println(profile + "\n");

        testDesc               = "Profile 4 - Vc < Vmax";
        test.dist              =   7.0;     // m
        test.Pi                =   0.0;     // m
        test.Vi                =   2.0;     // m/s
        test.overshoot         = false;     // boolean
        test.Vmax              =   6.0;     // m/s
        test.Tb                =   0.0;     // s
        test.Sb                =   0.0;     // m
        test.Amax              =   2.0;     // m/s^2
        test.Ta                =   1.0;     // s
        test.Sa                =   3.0;     // m (0.5*Amax*Ta*Ta)
        test.Vt                =   4.0;     // m/s
        test.Tc                =   0.0;     // s
        test.Vc                =   4.0;     // m/s
        test.Sc                =   0.0;     // m
        test.Dmax              =  -2.0;     // m/s^2
        test.Td                =   2.0;     // s
        test.Sd                =   4.0;     // -0.5*Dmax*Td*Td
        test.Tt                =   3.0;     // s

        profile.calcProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        testResult             = profile.approxEqual(test);
        RegTest.report(testDesc, testResult);
        if(!testResult)
            System.out.println(profile + "\n");

        testDesc               = "Profile 5 - No Acceleration";
        test.dist              =   4.0;     // m
        test.Pi                =   0.0;     // m
        test.Vi                =   4.0;     // m/s
        test.overshoot         = false;     // boolean
        test.Vmax              =   5.0;     // m/s
        test.Tb                =   0.0;     // s
        test.Sb                =   0.0;     // m
        test.Amax              =   2.0;     // m/s^2
        test.Ta                =   0.0;     // s
        test.Sa                =   0.0;     // m (0.5*Amax*Ta*Ta)
        test.Vt                =   4.0;     // m/s
        test.Tc                =   0.0;     // s
        test.Vc                =   4.0;     // m/s
        test.Sc                =   0.0;     // m
        test.Dmax              =  -2.0;     // m/s^2
        test.Td                =   2.0;     // s
        test.Sd                =   4.0;     // -0.5*Dmax*Td*Td
        test.Tt                =   2.0;     // s

        profile.calcProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        testResult             = profile.approxEqual(test);
        RegTest.report(testDesc, testResult);
        if(!testResult)
            System.out.println(profile + "\n");

        testDesc               = "Profile 6 - Vi = Vmax - No acceleration";
        test.dist              =  24.0;     // m
        test.Pi                =   0.0;     // m
        test.Vi                =   4.0;     // m/s
        test.overshoot         = false;     // boolean
        test.Vmax              =   4.0;     // m/s
        test.Tb                =   0.0;     // s
        test.Sb                =   0.0;     // m
        test.Amax              =   2.0;     // m/s^2
        test.Ta                =   0.0;     // s
        test.Sa                =   0.0;     // m (0.5*Amax*Ta*Ta)
        test.Vt                =   6.11010; // m/s
        test.Tc                =   4.0;     // s
        test.Vc                =   4.0;     // m/s
        test.Sc                =  16.0;     // m
        test.Dmax              =  -1.0;     // m/s^2
        test.Td                =   4.0;     // s
        test.Sd                =   8.0;     // -0.5*Dmax*Td*Td
        test.Tt                =   8.0;     // s

        profile.calcProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        testResult             = profile.approxEqual(test);
        RegTest.report(testDesc, testResult);
        if(!testResult)
            System.out.println(profile + "\n");

        testDesc               = "Profile 7 - Vi > Vmax";
        test.dist              =  30.0;     // m
        test.Pi                =   0.0;     // m
        test.Vi                =   6.0;     // m/s
        test.overshoot         = false;     // boolean
        test.Vmax              =   6.0;     // m/s
        test.Tb                =   2.0;     // s
        test.Sb                =  14.0;     // m
        test.Amax              =   2.0;     // m/s^2
        test.Ta                =   0.0;     // s
        test.Sa                =   0.0;     // m (0.5*Amax*Ta*Ta)
        test.Vt                =   7.21110; // m/s
        test.Tc                =   2.0;     // s
        test.Vc                =   6.0;     // m/s
        test.Sc                =  12.0;     // m
        test.Dmax              =  -1.0;     // m/s^2
        test.Td                =   6.0;     // s
        test.Sd                =  18.0;     // -0.5*Dmax*Td*Td
        test.Tt                =  10.0;     // s

        profile.calcProfile(44.0, test.Pi, 8.0, test.Vmax, test.Amax, test.Dmax);
        testResult             = profile.approxEqual(test);
        RegTest.report(testDesc, testResult);
        if(!testResult)
            System.out.println(profile + "\n");

        testDesc               = "Profile 8 - signum(Vi) == -signum(dist) + Cruise";

        test.dist              =  23.0;     // m
        test.Pi                =   0.0;     // m
        test.Vi                =  -8.0;     // m/s
        test.overshoot         = false;     // boolean
        test.Vmax              =   6.0;     // m/s
        test.Tb                =   0.0;     // s
        test.Sb                =   0.0;     // m
        test.Amax              =   2.0;     // m/s^2
        test.Ta                =   7.0;     // s
        test.Sa                =  -7.0;     // m (0.5*Amax*Ta*Ta)
        test.Vt                =   7.21110; // m/s
        test.Tc                =   2.0;     // s
        test.Vc                =   6.0;     // m/s
        test.Sc                =  12.0;     // m
        test.Dmax              =  -1.0;     // m/s^2
        test.Td                =   6.0;     // s
        test.Sd                =  18.0;     // -0.5*Dmax*Td*Td
        test.Tt                =  15.0;     // s

        profile.calcProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        testResult             = profile.approxEqual(test);
        RegTest.report(testDesc, testResult);
        if(!testResult)
            System.out.println(profile + "\n");

        testDesc               = "Profile 9 - signum(Vi) == -signum(dist) + No Cruise";

        test.dist              =  18.5;     // m
        test.Pi                =   0.0;     // m
        test.Vi                =  -1.0;     // m/s
        test.overshoot         = false;     // boolean
        test.Vmax              =   6.0;     // m/s
        test.Tb                =   0.0;     // s
        test.Sb                =   0.0;     // m
        test.Amax              =   2.0;     // m/s^2
        test.Ta                =   3.0;     // s
        test.Sa                =   6.0;     // m (0.5*Amax*Ta*Ta)
        test.Vt                =   5.0;     // m/s
        test.Tc                =   0.0;     // s
        test.Vc                =   5.0;     // m/s
        test.Sc                =   0.0;     // m
        test.Dmax              =  -1.0;     // m/s^2
        test.Td                =   5.0;     // s
        test.Sd                =  12.5;     // -0.5*Dmax*Td*Td
        test.Tt                =   8.0;     // s

        profile.calcProfile(test.dist, test.Pi, test.Vi, test.Vmax, test.Amax, test.Dmax);
        testResult             = profile.approxEqual(test);
        RegTest.report(testDesc, testResult);
        if(!testResult)
            System.out.println(profile + "\n");

        testDesc               = "Profile 10 - Overshoot";

        test.leadInProfile.dist      =  18.0;     // m
        test.leadInProfile.Pi        =   0.0;     // m
        test.leadInProfile.Vi        =   6.0;     // m/s
        test.leadInProfile.overshoot = false;     // boolean
        test.leadInProfile.Vmax      =   6.0;     // m/s
        test.leadInProfile.Tb        =   2.0;     // s
        test.leadInProfile.Sb        =  14.0;     // m
        test.leadInProfile.Amax      =   2.0;     // m/s^2
        test.leadInProfile.Ta        =   0.0;     // s
        test.leadInProfile.Sa        =   0.0;     // m (0.5*Amax*Ta*Ta)
        test.leadInProfile.Vt        =   6.0; // m/s
        test.leadInProfile.Tc        =   0.0;     // s
        test.leadInProfile.Vc        =   6.0;     // m/s
        test.leadInProfile.Sc        =   0.0;     // m
        test.leadInProfile.Dmax      =  -1.0;     // m/s^2
        test.leadInProfile.Td        =   6.0;     // s
        test.leadInProfile.Sd        =  18.0;     // -0.5*Dmax*Td*Td
        test.leadInProfile.Tt        =   8.0;     // s

        test.leadInProfile.setInitialized();

        test.dist              = -12.0;     // m
        test.Pi                =  32.0;     // m
        test.Vi                =   0.0;     // m/s
        test.overshoot         =  true;     // boolean
        test.Vmax              =  -6.0;     // m/s
        test.Tb                =   0.0;     // s
        test.Sb                =   0.0;     // m
        test.Amax              =  -2.0;     // m/s^2
        test.Ta                =   2.0;     // s
        test.Sa                =  -4.0;     // m (0.5*Amax*Ta*Ta)
        test.Vt                =  -4.0;     // m/s
        test.Tc                =   0.0;     // s
        test.Vc                =  -4.0;     // m/s
        test.Sc                =   0.0;     // m
        test.Dmax              =   1.0;     // m/s^2
        test.Td                =   4.0;     // s
        test.Sd                =  -8.0;     // -0.5*Dmax*Td*Td
        test.Tt                =   6.0;     // s

        profile.calcProfile(20.0, 0.0, 8.0, test.Vmax, test.Amax, test.Dmax);
        testResult             = profile.approxEqual(test);
        RegTest.report(testDesc, testResult);
        if(!testResult)
            System.out.println(profile + "\n");
    }
}
