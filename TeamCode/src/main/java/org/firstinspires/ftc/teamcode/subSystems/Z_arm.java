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

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Z_arm {
    private Motor slideMotor;
    public static double max_pos;
    public static double min_pos;
    PIDFController pidController;

    public Z_arm(HardwareMap hardwareMap) {
        slideMotor = new Motor(hardwareMap, "extensionMotor");

        slideMotor.setRunMode(Motor.RunMode.RawPower);

        slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        slideMotor.resetEncoder();
    }
    public void setPos(double pos){
        slideMotor.set(Math.max(min_pos, Math.min(max_pos, pos)));
    }
    public void tickPID(){
        pidController = new PIDController(0.2,0.1, 0.1);
        slideMotor.set(pidController.calculate(slideMotor.encoder.getPosition()));
    }

    public class MoveZ_armTask extends Task {
        private final double targetPosition;

        public MoveZ_armTask(RobotContext robotContext, double targetPosition) {
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
