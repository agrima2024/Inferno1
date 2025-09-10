package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.jumpypants.murphy.RobotContext;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subSystems.Z_arm;
import org.firstinspires.ftc.teamcode.subSystems.X_arm;
import org.firstinspires.ftc.teamcode.subSystems.Claw;

/**
 * MyRobot class that extends RobotContext to include robot-specific subsystems.
 */
public class MyRobot extends RobotContext {
    public final MecanumDrive drive;
    public final X_arm X_arm;
    public final Claw claw;
    public final Z_arm Z_arm;

    /**
     * Creates a new RobotContext with the specified telemetry and gamepad references.
     * All parameters are required and cannot be null.
     *
     * @param telemetry the telemetry instance for driver station communication
     * @param gamepad1  the primary gamepad controller
     * @param gamepad2  the secondary gamepad controller
     */
    public MyRobot(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, MecanumDrive drive, Z_arm z_arm, X_arm x_arm, Claw claw) {
        super(telemetry, gamepad1, gamepad2);
        this.drive = drive;
        this.Z_arm = z_arm;
        this.X_arm = x_arm;
        this.claw = claw;
    }
}
