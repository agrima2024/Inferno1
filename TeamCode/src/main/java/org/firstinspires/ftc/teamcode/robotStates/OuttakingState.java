package org.firstinspires.ftc.teamcode.robotStates;

import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.states.State;
import com.jumpypants.murphy.tasks.ParallelTask;
import com.jumpypants.murphy.tasks.SequentialTask;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MyRobot;
import org.firstinspires.ftc.teamcode.subSystems.Z_arm;
import org.firstinspires.ftc.teamcode.subSystems.Claw;

public class OuttakingState implements State {
    private final MyRobot robotContext;
    private final Task mainTask;

    public OuttakingState(MyRobot robotContext) {
        this.robotContext = robotContext;

        mainTask = new SequentialTask(robotContext,
                new WaitForDumpInputTask(robotContext),
                robotContext.claw.new MoveClawTask(robotContext, Claw.CLAW_CLOSED_POS),
                new ParallelTask(robotContext, false,
                        robotContext.claw.new MoveWristTask(robotContext, Claw.WRIST_MAX_POS),
                        robotContext.X_arm.new MoveExtensionTask(robotContext),
                        robotContext.Z_arm.new MoveShoulderTask(robotContext, Claw.WRIST_MAX_POS)
                )
        );
    }

    @Override
    public State step() {
        // Drive the drivebase with gamepad1
        Gamepad gamepad1 = robotContext.gamepad1;
        robotContext.drive.driveRobotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        robotContext.Z_arm.tickPID();

        // Run the main task until finished, then transition to IntakingState
        if (mainTask.step()) {
            return this;
        }

        return new IntakingState(robotContext);
    }

    private static class WaitForDumpInputTask extends Task {
        public WaitForDumpInputTask(MyRobot robotContext) {
            super(robotContext);
        }

        @Override
        protected void initialize(RobotContext robotContext) {}

        @Override
        protected boolean run(RobotContext robotContext) {
            return !robotContext.gamepad2.a;
        }
    }


    @Override
    public String getName() {
        return "Outtaking";
    }
}
