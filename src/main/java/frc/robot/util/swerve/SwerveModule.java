package frc.robot.util.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    public static final int FRONT_LEFT = 0;
    public static final int FRONT_RIGHT = 1;
    public static final int BACK_LEFT = 2;
    public static final int BACK_RIGHT = 3;

    SparkFlex driveMotorController;
    SparkMax rotationMotorController;

    RelativeEncoder driveMotorEncoder;
    AbsoluteEncoder rotationAbsoluteEncoder;

    SparkClosedLoopController driveController;
    SparkClosedLoopController rotationController;

    public SwerveModule(SparkFlex driveMotorController, SparkMax rotationMotorController) {
        this.driveMotorController = driveMotorController;
        this.rotationMotorController = rotationMotorController;

        driveMotorEncoder = driveMotorController.getEncoder();
        rotationAbsoluteEncoder = rotationMotorController.getAbsoluteEncoder();

        driveMotorController.getClosedLoopController();
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
                MetersPerSecond.of(driveMotorEncoder.getVelocity()),
                Rotation2d.fromRadians(rotationAbsoluteEncoder.getPosition()));
    }

    public void commandModuleState(SwerveModuleState state) {
        driveController.setReference(
                state.speedMetersPerSecond, ControlType.kMAXMotionVelocityControl);
        rotationController.setReference(
                state.angle.getRadians(), ControlType.kMAXMotionPositionControl);
    }
}
