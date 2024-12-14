package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Subsystem;
import lombok.Getter;

public class EnviornmentModel implements Subsystem, Sendable {
    @Getter Rotation2d robotHeading;

    Pigeon2 pigeon2;

    public EnviornmentModel() {
        pigeon2 = new Pigeon2(2, "rio");
    }

    public void update() {
        robotHeading = pigeon2.getRotation2d();
    }

    @Override
    public void initSendable(SendableBuilder builder) {}
}
