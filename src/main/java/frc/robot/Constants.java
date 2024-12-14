// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final Time ROBOT_LOOP_PERIOD = Milliseconds.of(20);

    /**
     * The width of the drivebase divided by two. The easist way to find the width of the drivebase
     * is to align two wheels forward, measure the distance between them, and subtract the width of
     * one wheel.
     */
    public static final Distance HALF_DRIVE_BASE_WIDTH = Centimeters.of(70).divide(2);

    /**
     * The translations of each module relative to the robot's center of rotation. On a square base
     * it should be this.
     */
    public static final Translation2d[] MODULE_TRANLATIONS = {
        new Translation2d(HALF_DRIVE_BASE_WIDTH, HALF_DRIVE_BASE_WIDTH),
        new Translation2d(HALF_DRIVE_BASE_WIDTH, HALF_DRIVE_BASE_WIDTH.unaryMinus()),
        new Translation2d(HALF_DRIVE_BASE_WIDTH.unaryMinus(), HALF_DRIVE_BASE_WIDTH),
        new Translation2d(HALF_DRIVE_BASE_WIDTH.unaryMinus(), HALF_DRIVE_BASE_WIDTH.unaryMinus())
    };

    /** Theoretical max free speed of a NEO Vortex MAXSwerve drivetrain on low gearing. */
    public static final LinearVelocity MAX_LINEAR_DRIVE_SPEED = MetersPerSecond.of(4.92);

    // Calculate in desmos: https://www.desmos.com/3d/txiz3pevkp
    public static final AngularVelocity MAX_ANGULAR_VELOCITY =
            RadiansPerSecond.of(
                    4.0 / Math.sqrt(2 * (Math.pow(HALF_DRIVE_BASE_WIDTH.in(Meters), 2))));
}
