package frc.robot.util.swerve;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.util.MotorUtil;
import frc.robot.util.MotorUtil.IntiializationError;

public class SwerveModule implements AutoCloseable {
    public static final int FRONT_LEFT = 0;
    public static final int FRONT_RIGHT = 0;
    public static final int BACK_LEFT = 0;
    public static final int BACK_RIGHT = 0;

    SparkFlex driveMotorController;
    SparkMax rotationMotorController;

    public SwerveModule(String module, int driveMotorID, int rotationMotorID)
            throws IntiializationError {

        driveMotorController =
                MotorUtil.initializeSparkFlex(
                        String.format(module + " Drive"), rotationMotorID, null);

        rotationMotorController = new SparkMax(rotationMotorID, MotorType.kBrushless);
    }

    @Override
    public void close() {
        if (this.driveMotorController != null) {
            driveMotorController.close();
        }
    }
}
