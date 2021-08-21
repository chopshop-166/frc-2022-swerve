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
        public SwerveModule frontLeft() {
            return new SwerveModule(new Translation2d(0.381, 0.381), new CANCoder(0), new SmartMotorController(),
                    new SmartMotorController());
        }

        public SwerveModule frontRight() {
            return new SwerveModule(new Translation2d(0.381, -0.381), new CANCoder(1), new SmartMotorController(),
                    new SmartMotorController());
        }

        public SwerveModule rearLeft() {
            return new SwerveModule(new Translation2d(-0.381, 0.381), new CANCoder(2), new SmartMotorController(),
                    new SmartMotorController());
        }

        public SwerveModule rearRight() {
            return new SwerveModule(new Translation2d(-0.381, -0.381), new CANCoder(3), new SmartMotorController(),
                    new SmartMotorController());
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