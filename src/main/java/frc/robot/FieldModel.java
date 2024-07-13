package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.Setter;
import monologue.Annotations.Log;
import monologue.Logged;

@Getter
public class FieldModel implements Logged {
    @Setter
    @Getter
    @AllArgsConstructor
    public static class Robot {
        Pose2d pose;
        //        int number;
        //        DriverStation.Alliance allianceColor;
    }

    private final Robot thisRobot;

    public FieldModel() {
        thisRobot = new Robot(new Pose2d());
        //                        8230,
        //                        DriverStation.getAlliance().isPresent()
        //                                ? DriverStation.getAlliance().get()
        //                                : null);
    }

    public void updateDrivetrainData(Pose2d odometryEstimate) {
        thisRobot.pose = odometryEstimate;
    }

    public Rotation2d getRobotRotation() {
        return thisRobot.pose.getRotation();
    }

    @Log
    public Pose2d getRobotPose() {
        return thisRobot.pose;
    }
}
