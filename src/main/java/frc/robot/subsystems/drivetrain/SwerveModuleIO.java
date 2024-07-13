package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    void periodic();

    void setSetpoint(SwerveModuleState state);

    SwerveModuleState getState();

    SwerveModuleState getSetpoint();

    SwerveModulePosition getPosition();
}
