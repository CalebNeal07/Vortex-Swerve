package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.util.swerve.SwerveModule.BACK_LEFT;
import static frc.robot.util.swerve.SwerveModule.BACK_RIGHT;
import static frc.robot.util.swerve.SwerveModule.FRONT_LEFT;
import static frc.robot.util.swerve.SwerveModule.FRONT_RIGHT;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MotorUtil;
import frc.robot.util.MotorUtil.IntiializationError;
import frc.robot.util.swerve.SwerveModule;
import java.util.function.Supplier;

public class Drivetrain extends SubsystemBase {
    /** List of the robot's swerve modules, ordered FL-FR-BL-BR */
    private final SwerveModule[] modules = new SwerveModule[4];

    private final SwerveDriveKinematics DRIVETRAIN_KINEMATICS =
            new SwerveDriveKinematics(Constants.MODULE_TRANLATIONS);

    public Drivetrain() throws IntiializationError {
        modules[FRONT_LEFT] =
                new SwerveModule(
                        MotorUtil.initializeSparkFlex("Front Left Drive", 1, null),
                        MotorUtil.initializeSparkMax("Front Left Rotation", 0, null));
        modules[FRONT_RIGHT] =
                new SwerveModule(
                        MotorUtil.initializeSparkFlex("Front Right Drive", 1, null),
                        MotorUtil.initializeSparkMax("Front Right Rotation", 0, null));
        modules[BACK_LEFT] =
                new SwerveModule(
                        MotorUtil.initializeSparkFlex("Back Left Drive", 1, null),
                        MotorUtil.initializeSparkMax("Back Left Rotation", 0, null));
        modules[BACK_RIGHT] =
                new SwerveModule(
                        MotorUtil.initializeSparkFlex("Back Right Drive", 1, null),
                        MotorUtil.initializeSparkMax("Back Right Rotation", 0, null));
    }

    public SwerveModuleState[] getCurrentModuleStates() {
        return new SwerveModuleState[] {
            modules[FRONT_LEFT].getModuleState(),
            modules[FRONT_RIGHT].getModuleState(),
            modules[BACK_LEFT].getModuleState(),
            modules[BACK_RIGHT].getModuleState()
        };
    }

    /**
     * Constructs a command that drives a swerve drive from robot relative speeds
     *
     * @param inputSupplier
     * @param headingSupplier
     * @return
     */
    public Command buildDriveCommand(Supplier<ChassisSpeeds> inputSupplier) {
        return new Command() {
            @Override
            public void execute() {
                var input = inputSupplier.get();

                input = ChassisSpeeds.discretize(input, Constants.ROBOT_LOOP_PERIOD.in(Seconds));

                var desiredModuleStates = DRIVETRAIN_KINEMATICS.toSwerveModuleStates(input);

                var currentModuleStates = Drivetrain.this.getCurrentModuleStates();

                // Optimize module states and perform cosine compensation
                desiredModuleStates[FRONT_LEFT].optimize(currentModuleStates[FRONT_LEFT].angle);
                desiredModuleStates[FRONT_LEFT].speedMetersPerSecond *=
                        desiredModuleStates[FRONT_LEFT]
                                .angle
                                .minus(currentModuleStates[FRONT_LEFT].angle)
                                .getCos();

                desiredModuleStates[FRONT_RIGHT].optimize(currentModuleStates[FRONT_RIGHT].angle);
                desiredModuleStates[FRONT_RIGHT].speedMetersPerSecond *=
                        desiredModuleStates[FRONT_RIGHT]
                                .angle
                                .minus(currentModuleStates[FRONT_RIGHT].angle)
                                .getCos();

                desiredModuleStates[BACK_LEFT].optimize(currentModuleStates[BACK_LEFT].angle);
                desiredModuleStates[BACK_LEFT].speedMetersPerSecond *=
                        desiredModuleStates[BACK_LEFT]
                                .angle
                                .minus(currentModuleStates[BACK_LEFT].angle)
                                .getCos();

                desiredModuleStates[BACK_RIGHT].optimize(currentModuleStates[BACK_RIGHT].angle);
                desiredModuleStates[BACK_RIGHT].speedMetersPerSecond *=
                        desiredModuleStates[BACK_RIGHT]
                                .angle
                                .minus(currentModuleStates[BACK_RIGHT].angle)
                                .getCos();

                modules[FRONT_LEFT].commandModuleState(desiredModuleStates[FRONT_LEFT]);
                modules[FRONT_RIGHT].commandModuleState(desiredModuleStates[FRONT_RIGHT]);
                modules[BACK_LEFT].commandModuleState(desiredModuleStates[BACK_LEFT]);
                modules[BACK_RIGHT].commandModuleState(desiredModuleStates[BACK_RIGHT]);
            }
        };
    }
}
