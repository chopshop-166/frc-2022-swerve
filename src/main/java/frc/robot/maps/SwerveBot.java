package frc.robot.maps;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.outputs.PIDSparkMax;
import com.chopshop166.chopshoplib.sensors.PigeonGyro;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import frc.outputs.SwerveModule;

// Need to get MAC address for roborio
@RobotMapFor("SwerveBotMac")
public class SwerveBot extends RobotMap {

    @Override
    public DriveMap getDriveMap() {
        return new DriveMap() {
            // Value taken from CAD as offset from center of module base pulley to center of
            // robot
            private static final double MODULE_OFFSET_XY = 0.298450;

            // All Distances are in Meters
            @Override
            public SwerveModule frontLeft() {
                final PIDSparkMax frontLeftController = new PIDSparkMax(new CANSparkMax(1, MotorType.kBrushless));
                SwerveModule.configureDriveMotor(frontLeftController);

                return new SwerveModule(new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY), new CANCoder(1),
                        new PIDSparkMax(new CANSparkMax(2, MotorType.kBrushless)), frontLeftController);
            }

            @Override
            public SwerveModule frontRight() {
                final PIDSparkMax frontRightController = new PIDSparkMax(new CANSparkMax(3, MotorType.kBrushless));
                SwerveModule.configureDriveMotor(frontRightController);

                return new SwerveModule(new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY), new CANCoder(2),
                        new PIDSparkMax(new CANSparkMax(4, MotorType.kBrushless)), frontRightController);
            }

            @Override
            public SwerveModule rearLeft() {
                final PIDSparkMax rearLeftController = new PIDSparkMax(new CANSparkMax(5, MotorType.kBrushless));
                SwerveModule.configureDriveMotor(rearLeftController);

                return new SwerveModule(new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY), new CANCoder(3),
                        new PIDSparkMax(new CANSparkMax(6, MotorType.kBrushless)), rearLeftController);
            }

            @Override
            public SwerveModule rearRight() {
                final PIDSparkMax rearRightController = new PIDSparkMax(new CANSparkMax(7, MotorType.kBrushless));
                SwerveModule.configureDriveMotor(rearRightController);
                return new SwerveModule(new Translation2d(-MODULE_OFFSET_XY, -MODULE_OFFSET_XY), new CANCoder(4),
                        new PIDSparkMax(new CANSparkMax(8, MotorType.kBrushless)), rearRightController);
            }

            @Override
            public double maxDriveSpeedMetersPerSecond() {
                return Units.feetToMeters(14.4);
            }

            @Override
            public double maxRotationRadianPerSecond() {
                return Math.PI;
            }

            @Override
            public GyroBase gyro() {
                return new PigeonGyro(new TalonSRX(1));
            }
        };
    }

}