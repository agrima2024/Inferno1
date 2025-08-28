package org.firstinspires.ftc.teamcode.robotStates;

import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.states.State;
import com.jumpypants.murphy.tasks.ParallelTask;
import com.jumpypants.murphy.tasks.SequentialTask;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.subSystems.Arm;
import org.firstinspires.ftc.teamcode.subSystems.Claw;

public class IntakingState implements State {
    private final MyRobot robotContext;
    private final Task mainTask;

    public IntakingState(MyRobot robotContext) {
        this.robotContext = robotContext;

        mainTask = new SequentialTask(robotContext,
                new GrabTask(robotContext),
                new TransferTask(robotContext)
        );
    }

    @Override
    public State step() {
        // Drive the drivebase with gamepad1
        Gamepad gamepad1 = robotContext.gamepad1;
        robotContext.drive.driveRobotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        robotContext.arm.calculatePID();

        // Run the main task until finished, then transition to OuttakingState
        if (mainTask.step()) {
            return this;
        }

        return new OuttakingState(robotContext);
    }

    @Override
    public String getName() {
        return "Intaking";
    }

    private static class GrabTask extends ParallelTask {
        public GrabTask(MyRobot robotContext) {
            super(robotContext, true,
                    robotContext.claw.new ManualWristTask(robotContext),
                    new WaitForTransferInputTask(robotContext)
            );
        }
    }

    private static class WaitForTransferInputTask extends Task {
        public WaitForTransferInputTask(MyRobot robotContext) {
            super(robotContext);
        }

        @Override
        protected void initialize(RobotContext robotContext) {}

        @Override
        protected boolean run(RobotContext robotContext) {
            return !robotContext.gamepad1.a;
        }
    }

    private static class TransferTask extends SequentialTask {
        public TransferTask(MyRobot robotContext) {
            super(robotContext,
                    robotContext.claw.new MoveClawTask(robotContext, Claw.CLAW_CLOSED_POSITION),
                    new ParallelTask(robotContext, false,
                            robotContext.arm.new MoveExtensionTask(robotContext, Arm.EXTENSION_OUTTAKING_POSITION),
                            robotContext.arm.new MoveShoulderTask(robotContext, Arm.SHOULDER_OUTTAKING_POSITION),
                            robotContext.claw.new MoveWristTask(robotContext, Claw.WRIST_UP_POSITION)
                    )
            );
        }
    }
}
