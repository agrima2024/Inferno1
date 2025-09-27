/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class X_arm {
    public static final double MAX_POS = 4.0;
    public static final double MIN_POS = 0.0;
    public static final double MAX_PWR = 1.0;
    public static final double MIN_PWR = -1.0;

    private Motor slideMotor;

    public X_arm(HardwareMap hardwareMap) {
        slideMotor = new Motor(hardwareMap, "extensionMotor");

        slideMotor.setRunMode(Motor.RunMode.RawPower);

        slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        slideMotor.resetEncoder();
    }
    public X_arm(Motor motor) {
        slideMotor = motor;
        slideMotor.setRunMode(Motor.RunMode.RawPower);
        slideMotor.resetEncoder();
    }

    public double limit(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    public class MoveX_armTask extends Task {

        public MoveX_armTask(RobotContext robotContext) {
            super(robotContext);
            this.targetPosition = targetPosition;
        }

        protected void initialize(RobotContext robotContext) {
            pidController.setSetPoint(targetPosition);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            return Math.abs(slideMotor.getCurrentPosition() - targetPosition) > 5; // Tolerance of 5 ticks
        }
    }
}