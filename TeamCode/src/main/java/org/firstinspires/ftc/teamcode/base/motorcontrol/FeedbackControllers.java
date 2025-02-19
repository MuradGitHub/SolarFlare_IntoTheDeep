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

import static java.lang.Math.ceil;

import org.firstinspires.ftc.teamcode.base.config.MotorControlConfig;
import org.firstinspires.ftc.teamcode.base.error.BadInputException;
import org.firstinspires.ftc.teamcode.base.logging.RobotLogger;

import java.util.HashMap;
import java.util.logging.Level;
import java.util.logging.Logger;

public class FeedbackControllers {
    public static Logger logger  = RobotLogger.getInstance().getConfigLogger();
    
    public static FeedbackController makeFeedbackController(
            FeedbackControllerEnum FBCEnum,
            MotorControlConfig config) {
        switch(FBCEnum) {
            case PID:
                HashMap<String,Double> params = config.feedback.get(FBCEnum);
                logger.logp(
                        Level.INFO,
                        "FeedbackControllers",
                        "makeFeedbackController",
                        params != null ? params.toString() : "FBC Params=null"
                        );
                if(params == null)
                    throw new BadInputException("No Control Params for " + FBCEnum);
                Double Kp            = params.get("Kp");
                Double Ki            = params.get("Ki");
                Double Kd            = params.get("Kd");
                Double ErrLookback   = params.get("ErrLookback");
                Double DerLookback   = params.get("DerLookback");
                return new PIDController(
                        Kp          != null ? Kp : 0.0,
                        Ki          != null ? Ki : 0.0,
                        Kd          != null ? Kd : 0.0,
                        ErrLookback != null ? (int) ceil(ErrLookback) : 1,
                        DerLookback != null ? (int) ceil(DerLookback) : 1);
            default:
                throw new BadInputException("FeedbackControllerEnum: " + FBCEnum + " not supported");
            }
    }
}
