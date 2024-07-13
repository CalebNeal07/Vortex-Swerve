// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.drivetrain.Drivetrain;
import monologue.Logged;
import monologue.Monologue;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot implements Logged {
    // Subsystems
    Drivetrain drivetrain;

    FieldModel fieldModel;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        fieldModel = new FieldModel();

        // Instantiate Subsystems
        drivetrain = new Drivetrain(fieldModel);

        Monologue.setupMonologue(this, "/Robot", false, false);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // fieldModel.update();

        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        Monologue.updateAll();
    }

    @Override
    public void disabledPeriodic() {
        Monologue.setFileOnly(DriverStation.isFMSAttached());
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();

        CommandPS5Controller driverController = new CommandPS5Controller(0);

        // Setup Driver Controls
        drivetrain.setDefaultCommand(
                drivetrain.fieldOrientedDriveCommand(
                        () -> MathUtil.applyDeadband(driverController.getLeftY(), 0.1),
                        () -> MathUtil.applyDeadband(driverController.getLeftX(), 0.1),
                        () ->
                                driverController.getHID().getPOV() == -1
                                        ? MathUtil.applyDeadband(-driverController.getRightX(), 0.1)
                                        : Constants.ANGLE_ALIGNMENT_KP
                                                * (Rotation2d.fromDegrees(
                                                                driverController.getHID().getPOV())
                                                        .minus(
                                                                fieldModel
                                                                        .getThisRobot()
                                                                        .pose
                                                                        .getRotation())
                                                        .getRadians())));
        driverController.cross().whileTrue(drivetrain.setCrossCommand());
    }
}
