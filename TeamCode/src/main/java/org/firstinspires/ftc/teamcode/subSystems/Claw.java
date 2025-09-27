package org.firstinspires.ftc.teamcode.subSystems;

import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MyRobot;

public class Claw{

    public static final double WRIST_MAX_POS = 0.8;
    public static final double WRIST_MIN_POS = -0.5;
    public static final  double CLAW_OPEN_POS = 0.5;
    public static final double CLAW_CLOSED_POS = 0.1;

    private  Servo wristServo;
    private  Servo clawServo;

    public Claw (Servo wristServo, Servo clawServo){
        this.wristServo = wristServo;
        this.clawServo = clawServo;
    }
    public double limit(double value, double min, double max){
        return Math.min(Math.max(value, min), max);
    }
    private void setWristPos(double pos){
        this.wristServo.setPosition(limit(pos, WRIST_MIN_POS, WRIST_MAX_POS));
    }

    private void setClawPos(double pos){
        this.clawServo.setPosition(limit(pos, CLAW_CLOSED_POS, CLAW_OPEN_POS));
    }

    public class MoveWristTask extends Task {
        private final double estimatedTimeTaken;
        private final double targetPosition;


        public MoveWristTask(RobotContext robotContext, double targetPosition) {
            super(robotContext);
            double currentPosition = wristServo.getPosition();

            // seconds per unit distance
            double TIME_COEFFICIENT = 0.5;

            estimatedTimeTaken = Math.abs(targetPosition - currentPosition) * TIME_COEFFICIENT;
            wristServo.setPosition(targetPosition);

            this.targetPosition = targetPosition;
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            wristServo.setPosition(targetPosition);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            return ELAPSED_TIME.seconds() < estimatedTimeTaken;
        }

    }

    public class ManualWristTask extends Task {
        public ManualWristTask(MyRobot robotContext) {
            super(robotContext);
        }

        @Override
        protected void initialize(RobotContext robotContext) {}

        @Override
        protected boolean run(RobotContext robotContext) {
            if (robotContext.gamepad2.dpad_up) {
                wristServo.setPosition(WRIST_MAX_POS);
            } else if (robotContext.gamepad2.dpad_down) {
                wristServo.setPosition(WRIST_MIN_POS);
            }
            return true;
        }
    }

    public class MoveClawTask extends Task {
        private final double estimatedTimeTaken;
        private final double targetPosition;


        public MoveClawTask(RobotContext robotContext, double targetPosition) {
            super(robotContext);
            double currentPosition = clawServo.getPosition();

            // seconds per unit distance
            double TIME_COEFFICIENT = 0.5;

            estimatedTimeTaken = Math.abs(targetPosition - currentPosition) * TIME_COEFFICIENT;
            clawServo.setPosition(targetPosition);

            this.targetPosition = targetPosition;
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            clawServo.setPosition(targetPosition);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            return ELAPSED_TIME.seconds() < estimatedTimeTaken;
        }
    }
}
