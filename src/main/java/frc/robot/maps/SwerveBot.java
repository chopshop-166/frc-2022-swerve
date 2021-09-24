package frc.robot.maps;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.outputs.PIDSparkMax;
import com.chopshop166.chopshoplib.sensors.MockGyro;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import frc.outputs.SwerveModule;

// Need to get MAC address for roborio
@RobotMapFor("00:80:2F:19:7B:A3")
public class SwerveBot extends RobotMap {

        @Override
        public DriveMap getDriveMap() {

                // Value taken from CAD as offset from center of module base pulley to center of
                // robot
                final double MODULE_OFFSET_XY = 0.298450;

                // All Distances are in Meters
                // Front Left Module
                final PIDSparkMax frontLeftController = new PIDSparkMax(new CANSparkMax(1, MotorType.kBrushless));
                final CANCoder encoderFL = new CANCoder(1);
                encoderFL.configMagnetOffset(-36.0078125);
                encoderFL.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
                SwerveModule.configureDriveMotor(frontLeftController);
                final SwerveModule frontLeft = new SwerveModule("Front Left",
                                new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY), encoderFL,
                                new PIDSparkMax(2, MotorType.kBrushless), frontLeftController);

                // Front Right Module
                final PIDSparkMax frontRightController = new PIDSparkMax(new CANSparkMax(3, MotorType.kBrushless));
                final CANCoder encoderFR = new CANCoder(2);
                encoderFR.configMagnetOffset(-293.02734375000006);
                encoderFR.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
                SwerveModule.configureDriveMotor(frontRightController);
                final SwerveModule frontRight = new SwerveModule("Front Right",
                                new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY), encoderFR,
                                new PIDSparkMax(4, MotorType.kBrushless), frontRightController);

                // Rear Left Module
                final PIDSparkMax rearLeftController = new PIDSparkMax(new CANSparkMax(5, MotorType.kBrushless));
                final CANCoder encoderRL = new CANCoder(3);
                encoderRL.configMagnetOffset(-102.6562);
                encoderRL.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
                SwerveModule.configureDriveMotor(rearLeftController);
                final SwerveModule rearLeft = new SwerveModule("Rear Left",
                                new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY), encoderRL,
                                new PIDSparkMax(6, MotorType.kBrushless), rearLeftController);

                // Rear Right Module
                final PIDSparkMax rearRightController = new PIDSparkMax(new CANSparkMax(7, MotorType.kBrushless));
                final CANCoder encoderRR = new CANCoder(4);
                encoderRR.configMagnetOffset(-269.121);
                encoderRR.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
                SwerveModule.configureDriveMotor(rearRightController);
                final SwerveModule rearRight = new SwerveModule("Rear Right",
                                new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY), encoderRR,
                                new PIDSparkMax(8, MotorType.kBrushless), rearRightController);

                final double maxDriveSpeedMetersPerSecond = Units.feetToMeters(10);

                final double maxRotationRadianPerSecond = Math.PI;

                final GyroBase gyro = new MockGyro();

                return new DriveMap(frontLeft, frontRight, rearLeft, rearRight, maxDriveSpeedMetersPerSecond,
                                maxRotationRadianPerSecond, gyro);
        }

}