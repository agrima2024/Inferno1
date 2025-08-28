package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    private final Motor EXTENSION_MOTOR;
    private final Motor SHOULDER_MOTOR;

    private final PIDController EXTENSION_PID = new PIDController(0.01, 0, 0.005);
    private final PIDController SHOULDER_PID = new PIDController(0.015, 0, 0.003);

    public static double EXTENSION_INTAKING_POSITION = 200;
    public static double EXTENSION_OUTTAKING_POSITION = 1000;
    public static double SHOULDER_INTAKING_POSITION = 0.0;
    public static double SHOULDER_OUTTAKING_POSITION = 1.0;

    public Arm(HardwareMap hardwareMap) {
        EXTENSION_MOTOR = new Motor(hardwareMap, "extensionMotor");
        SHOULDER_MOTOR = new Motor(hardwareMap, "shoulderMotor");

        EXTENSION_MOTOR.setRunMode(Motor.RunMode.RawPower);
        SHOULDER_MOTOR.setRunMode(Motor.RunMode.RawPower);

        EXTENSION_MOTOR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        SHOULDER_MOTOR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        EXTENSION_MOTOR.resetEncoder();
        SHOULDER_MOTOR.resetEncoder();
    }

    // To be called periodically in the main OpMode loop
    public void calculatePID() {
        double extensionPower = EXTENSION_PID.calculate(EXTENSION_MOTOR.getCurrentPosition());
        double shoulderPower = SHOULDER_PID.calculate(SHOULDER_MOTOR.getCurrentPosition());

        EXTENSION_MOTOR.set(extensionPower);
        SHOULDER_MOTOR.set(shoulderPower);
    }

    public class MoveExtensionTask extends Task {
        private final double targetPosition;

        public MoveExtensionTask(RobotContext robotContext, double targetPosition) {
            super(robotContext);
            this.targetPosition = targetPosition;
        }

        @Override
        protected void initialize(com.jumpypants.murphy.RobotContext robotContext) {
            EXTENSION_PID.setSetPoint(targetPosition);
        }

        @Override
        protected boolean run(com.jumpypants.murphy.RobotContext robotContext) {
            return Math.abs(EXTENSION_MOTOR.getCurrentPosition() - targetPosition) > 10; // Tolerance of 10 ticks
        }
    }

    public class MoveShoulderTask extends Task {
        private final double targetPosition;

        public MoveShoulderTask(RobotContext robotContext, double targetPosition) {
            super(robotContext);
            this.targetPosition = targetPosition;
        }

        @Override
        protected void initialize(com.jumpypants.murphy.RobotContext robotContext) {
            SHOULDER_PID.setSetPoint(targetPosition);
        }

        @Override
        protected boolean run(com.jumpypants.murphy.RobotContext robotContext) {
            return Math.abs(SHOULDER_MOTOR.getCurrentPosition() - targetPosition) > 5; // Tolerance of 5 ticks
        }
    }
}
