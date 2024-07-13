package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public class DrivetrainConstants {
    protected static final double DRIVE_PINION_TEETH = 13;
    protected static final double DRIVE_GEARING = (45.0 * 22) / (DRIVE_PINION_TEETH * 15);
    protected static final double TURN_GEARING = (62.0 / 14) * 12;
    protected static final Measure<Distance> WHEEL_BASE_WIDTH =
            Inches.of(28); // TODO: Find real value

    /**
     * Object that handles the vector calculations to generate individual module speeds. NOTE:
     * Modules are ordered FL-FR-BL-BR
     */
    protected static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS =
            new SwerveDriveKinematics(
                    new Translation2d(
                            WHEEL_BASE_WIDTH.in(Meter) / 2, WHEEL_BASE_WIDTH.in(Meter) / 2),
                    new Translation2d(
                            WHEEL_BASE_WIDTH.in(Meter) / 2, -WHEEL_BASE_WIDTH.in(Meter) / 2),
                    new Translation2d(
                            -WHEEL_BASE_WIDTH.in(Meter) / 2, WHEEL_BASE_WIDTH.in(Meter) / 2),
                    new Translation2d(
                            -WHEEL_BASE_WIDTH.in(Meter) / 2, -WHEEL_BASE_WIDTH.in(Meter) / 2));

    protected static final Measure<Velocity<Distance>> MAX_LINEAR_VELOCITY =
            MetersPerSecond.of(4); // TODO: Find real value
    protected static final Measure<Velocity<Angle>> MAX_ANGULAR_VELOCITY =
            RadiansPerSecond.of(3 * Math.PI);
    protected static final Measure<Distance> WHEEL_DIAMETER = Inches.of(3);
}
