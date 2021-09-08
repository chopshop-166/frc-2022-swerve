package frc.robot.maps;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.outputs.PIDSparkMax;
import com.chopshop166.chopshoplib.sensors.MockGyro;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
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
        final PIDSparkMax frontLeftSteering = new PIDSparkMax(2, MotorType.kBrushless);
        final CANCoder encoderFL = new CANCoder(1);
        encoderFL.configMagnetOffset(-36.0078125);
        SwerveModule.configureDriveMotor(frontLeftController);
        final SwerveModule frontLeft = new SwerveModule(new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderFL, frontLeftSteering, frontLeftController);

        // Front Right Module
        final PIDSparkMax frontRightController = new PIDSparkMax(new CANSparkMax(3, MotorType.kBrushless));
        final CANCoder encoderFR = new CANCoder(2);
        encoderFR.configMagnetOffset(-293.02734375000006);
        SwerveModule.configureDriveMotor(frontRightController);
        final SwerveModule frontRight = new SwerveModule(new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderFR, new PIDSparkMax(new CANSparkMax(4, MotorType.kBrushless)), frontRightController);

        // Rear Left Module
        final PIDSparkMax rearLeftController = new PIDSparkMax(new CANSparkMax(5, MotorType.kBrushless));
        final CANCoder encoderRL = new CANCoder(3);
        encoderRL.configMagnetOffset(-102.6562);
        SwerveModule.configureDriveMotor(rearLeftController);
        final SwerveModule rearLeft = new SwerveModule(new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY),
                encoderRL, new PIDSparkMax(new CANSparkMax(6, MotorType.kBrushless)), rearLeftController);

        // Rear Right Module
        final PIDSparkMax rearRightController = new PIDSparkMax(new CANSparkMax(7, MotorType.kBrushless));
        final CANCoder encoderRR = new CANCoder(4);
        encoderRR.configMagnetOffset(-269.121);
        SwerveModule.configureDriveMotor(rearRightController);
        final SwerveModule rearRight = new SwerveModule(new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY),
                encoderRR, new PIDSparkMax(new CANSparkMax(8, MotorType.kBrushless)), rearRightController);

        final double maxDriveSpeedMetersPerSecond = Units.feetToMeters(14.4);

        final double maxRotationRadianPerSecond = Math.PI;

        final GyroBase gyro = new MockGyro();

        return new DriveMap(frontLeft, frontRight, rearLeft, rearRight, maxDriveSpeedMetersPerSecond,
                maxRotationRadianPerSecond, gyro);
    }

}