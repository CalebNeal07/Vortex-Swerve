// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.REVLibError;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Elastic;
import frc.robot.util.MotorUtil.IntiializationError;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    // Subsystems
    Drivetrain drivetrain;

    EnviornmentModel enviornment;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        System.out.println(REVLibError.kCANDisconnected.name());
        Alert failedInitializationAlert = new Alert("", AlertType.kInfo);

        // Instantiate Subsystems
        while (true) {
            try {
                drivetrain = new Drivetrain();
                break;
            } catch (IntiializationError error) {
                Elastic.sendAlert(error.generateNotification());
                failedInitializationAlert = error.generateAlert();
                failedInitializationAlert.set(true);
            }
        }

        failedInitializationAlert.set(false);

        enviornment = new EnviornmentModel();
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
        // Update enviornment before commands are run and after subssytem periodics are run.
        enviornment.update();

        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();

        CommandXboxController driverController = new CommandXboxController(0);

        drivetrain.setDefaultCommand(
                drivetrain.buildDriveCommand(
                        () -> {
                            return ChassisSpeeds.fromFieldRelativeSpeeds(
                                    driverController.getLeftY()
                                            * Constants.MAX_LINEAR_DRIVE_SPEED.in(MetersPerSecond)
                                            * kDefaultPeriod,
                                    driverController.getLeftX()
                                            * Constants.MAX_LINEAR_DRIVE_SPEED.in(MetersPerSecond)
                                            * kDefaultPeriod,
                                    driverController.getRightX()
                                            * Constants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond),
                                    enviornment.getRobotHeading());
                        }));
    }
}
