package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldModel;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;

public class Drivetrain extends SubsystemBase implements Logged {
    private final SwerveModuleIO frontLeftModule;
    private final SwerveModuleIO frontRightModule;
    private final SwerveModuleIO backLeftModule;
    private final SwerveModuleIO backRightModule;

    private final SwerveDriveOdometry odometry;
    private final FieldModel fieldModel;

    public Drivetrain(FieldModel fieldModel) {
        frontLeftModule =
                RobotBase.isReal()
                        ? new SwerveModuleIOSparkFlex(0, 0, "FL")
                        : new SwerveModuleIOSim("FL");
        frontRightModule =
                RobotBase.isReal()
                        ? new SwerveModuleIOSparkFlex(0, 0, "FR")
                        : new SwerveModuleIOSim("FR");
        backLeftModule =
                RobotBase.isReal()
                        ? new SwerveModuleIOSparkFlex(0, 0, "BL")
                        : new SwerveModuleIOSim("BL");
        backRightModule =
                RobotBase.isReal()
                        ? new SwerveModuleIOSparkFlex(0, 0, "BR")
                        : new SwerveModuleIOSim("BR");

        this.fieldModel = fieldModel;

        odometry =
                new SwerveDriveOdometry(
                        DrivetrainConstants.SWERVE_DRIVE_KINEMATICS,
                        fieldModel.getRobotRotation(),
                        new SwerveModulePosition[] {
                            frontLeftModule.getPosition(),
                            frontRightModule.getPosition(),
                            backLeftModule.getPosition(),
                            backRightModule.getPosition()
                        });
    }

    @Override
    public void periodic() {
        frontLeftModule.periodic();
        frontRightModule.periodic();
        backLeftModule.periodic();
        backRightModule.periodic();

        odometry.update(
                RobotBase.isReal()
                        ? fieldModel.getRobotRotation()
                        : fieldModel
                                .getRobotRotation()
                                .plus(
                                        Rotation2d.fromRadians(
                                                DrivetrainConstants.SWERVE_DRIVE_KINEMATICS
                                                                .toChassisSpeeds(
                                                                        getMeasuredModuleStates())
                                                                .omegaRadiansPerSecond
                                                        * 0.02)),
                new SwerveModulePosition[] {
                    frontLeftModule.getPosition(),
                    frontRightModule.getPosition(),
                    backLeftModule.getPosition(),
                    backRightModule.getPosition()
                });

        fieldModel.updateDrivetrainData(odometry.getPoseMeters());
    }

    @Log
    public SwerveModuleState[] getMeasuredModuleStates() {
        return new SwerveModuleState[] {
            frontLeftModule.getState(),
            frontRightModule.getState(),
            backLeftModule.getState(),
            backRightModule.getState()
        };
    }

    @Log
    public SwerveModuleState[] getModuleStateSetpoints() {
        return new SwerveModuleState[] {
            frontLeftModule.getSetpoint(),
            frontRightModule.getSetpoint(),
            backLeftModule.getSetpoint(),
            backRightModule.getSetpoint()
        };
    }

    /** Generates a command to arrange the swerve modules in an immobile position. */
    public Command setCrossCommand() {
        return new RunCommand(
                () -> {
                    frontLeftModule.setSetpoint(
                            new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
                    frontRightModule.setSetpoint(
                            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
                    backLeftModule.setSetpoint(
                            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
                    backRightModule.setSetpoint(
                            new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
                },
                this);
    }

    /**
     * The primary drive command. Field Oriented Drive turns joystick inputs into the motor inputs
     * that drive the robot. Field Oriented drives the robot such that pushing the driver
     * controller's left joystick forward will always move the robot downfield, regardless of the
     * robot's orientation.
     */
    public Command fieldOrientedDriveCommand(
            DoubleSupplier downfieldJoystickSupplier,
            DoubleSupplier sideTranslateJoystickSupplier,
            DoubleSupplier angularVelocitySupplier) {
        return new RunCommand(
                () -> {
                    var speeds =
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    DrivetrainConstants.MAX_LINEAR_VELOCITY.times(
                                            downfieldJoystickSupplier.getAsDouble()
                                                    * downfieldJoystickSupplier.getAsDouble()
                                                    * Math.signum(
                                                            downfieldJoystickSupplier
                                                                    .getAsDouble())),
                                    DrivetrainConstants.MAX_LINEAR_VELOCITY.times(
                                            sideTranslateJoystickSupplier.getAsDouble()
                                                    * sideTranslateJoystickSupplier.getAsDouble()
                                                    * Math.signum(
                                                            sideTranslateJoystickSupplier
                                                                    .getAsDouble())),
                                    RadiansPerSecond.of(
                                            angularVelocitySupplier.getAsDouble()
                                                    * DrivetrainConstants.MAX_ANGULAR_VELOCITY.in(
                                                            RadiansPerSecond)),
                                    fieldModel.getThisRobot().getPose().getRotation());

                    if (MathUtil.applyDeadband(speeds.vxMetersPerSecond, -10e-2, 10e2) == 0
                            && MathUtil.applyDeadband(speeds.vyMetersPerSecond, -10e-2, 10e2) == 0
                            && MathUtil.applyDeadband(speeds.omegaRadiansPerSecond, -10e-2, 10e2)
                                    == 0) {
                        frontLeftModule.setSetpoint(
                                new SwerveModuleState(0, frontLeftModule.getState().angle));
                        frontRightModule.setSetpoint(
                                new SwerveModuleState(0, frontRightModule.getState().angle));
                        backLeftModule.setSetpoint(
                                new SwerveModuleState(0, backLeftModule.getState().angle));
                        backRightModule.setSetpoint(
                                new SwerveModuleState(0, backRightModule.getState().angle));
                        return;
                    }

                    ChassisSpeeds.discretize(speeds, 0.02);

                    var moduleSpeeds =
                            DrivetrainConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                                    speeds);
                    SwerveDriveKinematics.desaturateWheelSpeeds(
                            moduleSpeeds, DrivetrainConstants.MAX_LINEAR_VELOCITY);

                    frontLeftModule.setSetpoint(moduleSpeeds[0]);
                    frontRightModule.setSetpoint(moduleSpeeds[1]);
                    backLeftModule.setSetpoint(moduleSpeeds[2]);
                    backRightModule.setSetpoint(moduleSpeeds[3]);
                },
                this);
    }
}
