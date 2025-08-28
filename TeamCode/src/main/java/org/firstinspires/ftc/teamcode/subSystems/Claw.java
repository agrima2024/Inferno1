package org.firstinspires.ftc.teamcode.subSystems;

import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MyRobot;

public class Claw {

    // Mechanical components are private in their subsystem class
    private final Servo clawServo;
    private final Servo wristServo;

    // Positions are public constants to be used by other classes as a frame of reference
    public static final double CLAW_OPEN_POSITION = 0.0;
    public static final double CLAW_CLOSED_POSITION = 0.7;
    public static final double WRIST_UP_POSITION = 0.1;
    public static final double WRIST_DOWN_POSITION = 1.0;

    public Claw(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
    }

    // Tasks live inside the subsystem class
    // Notice how they have access to the private components of the subsystem

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
        private final RobotContext robotContext;

        public ManualWristTask(MyRobot robotContext) {
            super(robotContext);
            this.robotContext = robotContext;
        }

        @Override
        protected void initialize(RobotContext robotContext) {}

        @Override
        protected boolean run(RobotContext robotContext) {
            if (robotContext.gamepad2.dpad_up) {
                wristServo.setPosition(WRIST_UP_POSITION);
            } else if (robotContext.gamepad2.dpad_down) {
                wristServo.setPosition(WRIST_DOWN_POSITION);
            }
            return true;
        }
    }
}
