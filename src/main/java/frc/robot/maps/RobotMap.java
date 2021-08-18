package frc.robot.maps;

import com.chopshop166.chopshoplib.sensors.MockGyro;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;

public class RobotMap {
    public static class DriveMap {
        // Locations for the swerve drive modules relative to the robot center.
        // Distances are in Meter

        public Translation2d frontLeftLocation() {
            return new Translation2d(0.381, 0.381);
        }

        public Translation2d frontRightLocation() {
            return new Translation2d(0.381, -0.381);
        }

        public Translation2d rearLeftLocation() {
            return new Translation2d(-0.381, 0.381);
        }

        public Translation2d rearRightLocation() {
            return new Translation2d(-0.381, -0.381);
        }

        public double maxDriveSpeedMetersPerSecond() {
            return 2;
        }

        public double maxRotationRadianPerSecond() {
            return 2 * Math.PI;
        }

        public GyroBase gyro() {
            return new MockGyro();
        }

    }

    public DriveMap getDriveMap() {
        return new DriveMap();
    }

}