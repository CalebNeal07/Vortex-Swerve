package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.Setter;

public class EnviornmentModel implements Subsystem {
    @Setter
    @Getter
    @AllArgsConstructor
    public static class Robot {
        Pose2d pose;
    }

    private final Robot thisRobot;

    public EnviornmentModel() {
        thisRobot = new Robot(new Pose2d());
    }

    public void updateDrivetrainData(Pose2d odometryEstimate) {
        thisRobot.pose = odometryEstimate;
    }

    public void update() {}

    public Rotation2d getRobotRotation() {
        return thisRobot.pose.getRotation();
    }

    public Pose2d getRobotPose() {
        return thisRobot.pose;
    }
}
