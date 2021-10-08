package frc.robot.maps;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.outputs.IDSolenoid;
import com.chopshop166.chopshoplib.outputs.MockDSolenoid;
import com.chopshop166.chopshoplib.outputs.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.MockDigitalInput;
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

            this.frontLeft = new SwerveModule("Front Left", new Translation2d(0.381, 0.381), new CANCoder(0),
                    new SmartMotorController(), new SmartMotorController());

            this.frontRight = new SwerveModule("Front Right", new Translation2d(0.381, -0.381), new CANCoder(1),
                    new SmartMotorController(), new SmartMotorController());

            this.rearLeft = new SwerveModule("Rear Left", new Translation2d(-0.381, 0.381), new CANCoder(2),
                    new SmartMotorController(), new SmartMotorController());

            this.rearRight = new SwerveModule("Rear Right", new Translation2d(-0.381, -0.381), new CANCoder(3),
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

    public static class IntakeMap {
        private final IDSolenoid piston;
        private final SmartMotorController motor;

        public IntakeMap(final IDSolenoid piston, final SmartMotorController motor) {
            this.piston = piston;
            this.motor = motor;
        }

        public IntakeMap() {
            this(new MockDSolenoid(), new SmartMotorController());
        }

        // Forward should deploy intake
        public IDSolenoid getPiston() {
            return piston;
        }

        // Forward should intake ball
        public SmartMotorController getMotor() {
            return motor;
        }
    }

    public IntakeMap getIntakeMap() {
        return new IntakeMap();
    }

    public static class SpindexerMap {
        private final SmartMotorController motor;

        public SpindexerMap(final SmartMotorController motor) {
            this.motor = motor;
        }

        public SpindexerMap() {
            this(new SmartMotorController());
        }

        public SmartMotorController getMotor() {
            return motor;
        }
    }

    public SpindexerMap getSpindexerMap() {
        return new SpindexerMap();
    }
    public static class KickerMap {
        SmartMotorController motor;
        BooleanSupplier ballSensor;

        public KickerMap(SmartMotorController motor, BooleanSupplier ballSensor) {
            this.motor = motor;
            this.ballSensor = ballSensor;
        }

        public KickerMap() {
            this(new SmartMotorController(), new MockDigitalInput()::getAsBoolean);
        }

        public SmartMotorController getMotor() {
            return motor;
        }

        public BooleanSupplier getBallSensor() {
            return ballSensor;
        }
    }

    public KickerMap getKickerMap() {
        return new KickerMap();
    }

    public static class TurretMap {
        private final SmartMotorController motor;
        private final BooleanSupplier limitSwitch;

        public TurretMap(SmartMotorController motor, BooleanSupplier limitSwitch) {
            this.motor = motor;
            this.limitSwitch = limitSwitch;
        }

        public TurretMap() {
            this(new SmartMotorController(), new MockDigitalInput());
        }

        public SmartMotorController getMotor() {
            return motor;
        }

        public BooleanSupplier getLimitSwitch() {
            return limitSwitch;
        }
    }

    public TurretMap getTurretMap() {
        return new TurretMap();
    }
}