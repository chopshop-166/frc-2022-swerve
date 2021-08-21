package frc.robot.maps;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.outputs.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.MockGyro;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.outputs.SwerveModule;

// Need to get MAC address for roborio
@RobotMapFor("Default")
public class RobotMap {
    public static class DriveMap {
        // All Distances are in Meters

        private final SwerveModule frontLeft;
        private final SwerveModule frontRight;
        private final SwerveModule rearLeft;
        private final SwerveModule rearRight;
        private final double maxDriveSpeedMetersPerSecond;
        private final double maxRotationRadianPerSecond;
        private final GyroBase gyro;

        public DriveMap() {

            this.frontLeft = new SwerveModule(new Translation2d(0.381, 0.381), new CANCoder(0),
                    new SmartMotorController(), new SmartMotorController());

            this.frontRight = new SwerveModule(new Translation2d(0.381, -0.381), new CANCoder(1),
                    new SmartMotorController(), new SmartMotorController());

            this.rearLeft = new SwerveModule(new Translation2d(-0.381, 0.381), new CANCoder(2),
                    new SmartMotorController(), new SmartMotorController());

            this.rearRight = new SwerveModule(new Translation2d(-0.381, -0.381), new CANCoder(3),
                    new SmartMotorController(), new SmartMotorController());

            this.maxDriveSpeedMetersPerSecond = 2;

            this.maxRotationRadianPerSecond = Math.PI;

            this.gyro = new MockGyro();
        }

        public DriveMap(final SwerveModule frontLeft, final SwerveModule frontRight, final SwerveModule rearLeft,
                final SwerveModule rearRight, final double maxDriveSpeedMetersPerSecond,
                final double maxRotationRadianPerSecond, final GyroBase gyro) {

            this.frontLeft = frontLeft;

            this.frontRight = frontRight;

            this.rearLeft = rearLeft;

            this.rearRight = rearRight;

            this.maxDriveSpeedMetersPerSecond = maxDriveSpeedMetersPerSecond;

            this.maxRotationRadianPerSecond = maxRotationRadianPerSecond;

            this.gyro = gyro;
        }

        public SwerveModule getFrontLeft() {
            return frontLeft;
        }

        public SwerveModule getFrontRight() {
            return frontRight;
        }

        public SwerveModule getRearLeft() {
            return rearLeft;
        }

        public SwerveModule getRearRight() {
            return rearRight;
        }

        public double getMaxDriveSpeedMetersPerSecond() {
            return maxDriveSpeedMetersPerSecond;
        }

        public double getMaxRotationRadianPerSecond() {
            return maxRotationRadianPerSecond;
        }

        public GyroBase getGyro() {
            return gyro;
        }
    }

    public DriveMap getDriveMap() {
        return new DriveMap();
    }

}