package frc.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import lombok.Getter;
import lombok.Setter;
import monologue.Logged;

public class SwerveModuleIOSparkFlex implements SwerveModuleIO, Logged {
    CANSparkFlex driveMotor;
    CANSparkMax turnMotor;

    SparkRelativeEncoder driveEncoder;
    SparkAbsoluteEncoder turnEncoder;

    @Getter @Setter SwerveModuleState setpoint = new SwerveModuleState();

    protected SwerveModuleIOSparkFlex(int driveID, int turnID, String name) {}

    @Override
    public void periodic() {}

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveEncoder.getVelocity(), Rotation2d.fromRadians(turnEncoder.getPosition()));
    }

    @Override
    public SwerveModulePosition getPosition() {
        return null;
    }
}
