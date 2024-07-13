package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import lombok.Getter;
import lombok.Setter;
import monologue.Logged;

public class SwerveModuleIOSim implements SwerveModuleIO, Logged {
    DCMotorSim driveSim;
    DCMotorSim turnSim;

    private final SimpleMotorFeedforward driveFeedforwardController =
            new SimpleMotorFeedforward(0, 1.1, 0);

    private final PIDController turnFeedbackController = new PIDController(20, 0, 0);

    @Getter @Setter SwerveModuleState setpoint = new SwerveModuleState();

    protected SwerveModuleIOSim(String name) {
        driveSim =
                new DCMotorSim(DCMotor.getNeoVortex(1), DrivetrainConstants.DRIVE_GEARING, 0.025);
        turnSim = new DCMotorSim(DCMotor.getNeo550(1), DrivetrainConstants.TURN_GEARING, 0.004);
    }

    @Override
    public void periodic() {
        driveSim.setInputVoltage(
                driveFeedforwardController.calculate(setpoint.speedMetersPerSecond));

        turnSim.setInputVoltage(
                turnFeedbackController.calculate(
                        turnSim.getAngularPositionRad(), setpoint.angle.getRadians()));

        driveSim.update(0.02);
        turnSim.update(0.02);
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveSim.getAngularVelocityRadPerSec()
                        * DrivetrainConstants.WHEEL_DIAMETER.in(Meter),
                Rotation2d.fromRadians(turnSim.getAngularPositionRad()));
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveSim.getAngularPositionRad() * DrivetrainConstants.WHEEL_DIAMETER.in(Meter),
                Rotation2d.fromRadians(turnSim.getAngularPositionRad()));
    }
}
