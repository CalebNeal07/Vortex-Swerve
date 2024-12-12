package frc.robot.subsystems;

import static frc.robot.util.swerve.SwerveModule.BACK_LEFT;
import static frc.robot.util.swerve.SwerveModule.BACK_RIGHT;
import static frc.robot.util.swerve.SwerveModule.FRONT_LEFT;
import static frc.robot.util.swerve.SwerveModule.FRONT_RIGHT;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MotorUtil.IntiializationError;
import frc.robot.util.swerve.SwerveModule;

public class Drivetrain extends SubsystemBase implements AutoCloseable {
    /** List of the robot's swerve modules, ordered FL-FR-BL-BR */
    private final SwerveModule[] modules = new SwerveModule[4];

    public Drivetrain() throws IntiializationError {
        modules[FRONT_LEFT] = new SwerveModule("Front Left", 1, 0);
        modules[FRONT_RIGHT] = new SwerveModule("Front Right", 3, 2);
        modules[BACK_LEFT] = new SwerveModule("Back Left", 5, 4);
        modules[BACK_RIGHT] = new SwerveModule("Back Right", 7, 6);
    }

    @Override
    public void close() {
        for (var module : modules) {
            if (module != null) {
                module.close();
            }
        }
    }
}
