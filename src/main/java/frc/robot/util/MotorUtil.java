package frc.robot.util;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.util.Elastic.ElasticNotification;
import frc.robot.util.Elastic.ElasticNotification.NotificationLevel;
import lombok.AllArgsConstructor;

public class MotorUtil {    
    /**
     * Initialize and configures a Spark Flex motor, gracefully handling any errors
     */
    public static SparkFlex initializeSparkFlex(String name, int id, SparkFlexConfig config) throws IntiializationError { 
        var sparkFlex = new SparkFlex(id, MotorType.kBrushless);
        var error = sparkFlex.getLastError();

        if (error != REVLibError.kOk) {
            throw IntiializationError.fromREVLibError(error, name, id);
        }

        return sparkFlex;
    }

    @AllArgsConstructor
    public static class IntiializationError extends Exception {
        private String title;
        private String format;

        private static IntiializationError fromREVLibError(REVLibError error, String name, int id) {
            String title;
            String description;
            switch (error) {
                case kCANDisconnected:
                    title = "No CAN connection to the %1$s motor";
                    description = "Failed to establish a connection to the %1$s (ID: %2$d) motor.";
                    break;
                default:
                    title = "REV Initialization Error";
                    description = "The robot failed to initialize a REV motor and the error didn't match an implemented case.";
            }

            title.formatted(name, id);
            description.formatted(name, id);

            return new IntiializationError(title, description);
        }

        public Alert generateAlert() {
            return new Alert(title + ": " + format, AlertType.kError);
        }

        public ElasticNotification generateNotification() {
            return new ElasticNotification(NotificationLevel.ERROR, title, format);
        }
    }
}
