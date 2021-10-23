// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import com.google.common.math.Stats;

import com.chopshop166.chopshoplib.SampleBuffer;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

/* 
Vision Goals:
Find target(s)
Tell turret a bunch of stuff about the target(s), primarily where they are.
*/

public class Vision extends SmartSubsystemBase {
    private final PhotonCamera camera = new PhotonCamera("swerveCam");
    private final double CAMERA_HEIGHT_METERS = 2.24;
    private final double TARGET_HEIGHT_METERS = 10;
    private final double CAMERA_PITCH_RADIANS = 0.75;
    private SampleBuffer latency = new SampleBuffer(25);

    public Vision() {
        super();
    }

    // Driver Mode turns off the limelight.
    public CommandBase DriverMode(final boolean toggle) {
        return instant("DriverMode", () -> {
            camera.setDriverMode(toggle);
        });
    }

    double getDistance() {
        var result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        return PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS, target.getPitch());
    }

    double getRotation(double lastSpeed) {
        var result = camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
        double skew = target.getSkew();
        // Takes skew from target and subtracts latency and last rotational speed to
        // account for distance travelled inbetween frames.
        return skew - (Stats.of(latency).mean() * lastSpeed);
    }

    @Override
    public void reset() {
        // Nothing to reset here
    }

    @Override
    public void periodic() {
        latency.addSample(camera.getLatestResult().getLatencyMillis() / 1000.0);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation

    }
}