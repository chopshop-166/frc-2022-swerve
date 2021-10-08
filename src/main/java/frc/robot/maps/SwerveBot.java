package frc.robot.maps;

import com.chopshop166.chopshoplib.maps.RobotMapFor;
import com.chopshop166.chopshoplib.outputs.PIDSparkMax;
import com.chopshop166.chopshoplib.outputs.WDSolenoid;
import com.chopshop166.chopshoplib.sensors.MockGyro;
import com.chopshop166.chopshoplib.sensors.WDigitalInput;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogTrigger;
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
        final PIDSparkMax frontLeftController = new PIDSparkMax(1, MotorType.kBrushless);
        final CANCoder encoderFL = new CANCoder(1);
        encoderFL.configMagnetOffset(-36.0078125);
        encoderFL.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        SwerveModule.configureDriveMotor(frontLeftController);
        final SwerveModule frontLeft = new SwerveModule("Front Left",
                new Translation2d(MODULE_OFFSET_XY, MODULE_OFFSET_XY), encoderFL,
                new PIDSparkMax(2, MotorType.kBrushless), frontLeftController);

        // Front Right Module
        final PIDSparkMax frontRightController = new PIDSparkMax(3, MotorType.kBrushless);
        final CANCoder encoderFR = new CANCoder(2);
        encoderFR.configMagnetOffset(-293.02734375000006);
        encoderFR.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        SwerveModule.configureDriveMotor(frontRightController);
        final SwerveModule frontRight = new SwerveModule("Front Right",
                new Translation2d(MODULE_OFFSET_XY, -MODULE_OFFSET_XY), encoderFR,
                new PIDSparkMax(4, MotorType.kBrushless), frontRightController);

        // Rear Left Module
        final PIDSparkMax rearLeftController = new PIDSparkMax(5, MotorType.kBrushless);
        final CANCoder encoderRL = new CANCoder(3);
        encoderRL.configMagnetOffset(-102.6562);
        encoderRL.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        SwerveModule.configureDriveMotor(rearLeftController);
        final SwerveModule rearLeft = new SwerveModule("Rear Left",
                new Translation2d(-MODULE_OFFSET_XY, MODULE_OFFSET_XY), encoderRL,
                new PIDSparkMax(6, MotorType.kBrushless), rearLeftController);

        // Rear Right Module
        final PIDSparkMax rearRightController = new PIDSparkMax(7, MotorType.kBrushless);
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

    @Override
    public IntakeMap getIntakeMap() {
        // Create intake motor, and configure encoder
        final var motor = new PIDSparkMax(9, MotorType.kBrushless);
        final var rawMotor = motor.getMotorController();
        // Set current limit on the motor to avoid damaging anything if a ball gets
        // jammed
        rawMotor.setSmartCurrentLimit(20);
        // Set motor to brake mode so any balls that are partially in the intake don't
        // roll out
        rawMotor.setIdleMode(IdleMode.kBrake);
        // Configure Encoder scaling
        final var encoder = motor.getEncoder();
        // Intake has 2.5:1 gear reduction
        final double reduction = 1 / 2.5;
        // Velocity is in roller RPM
        encoder.setVelocityScaleFactor(reduction);
        // Position is in roller rotations
        encoder.setPositionScaleFactor(reduction);

        final WDSolenoid piston = new WDSolenoid(0, 1);
        return new IntakeMap(piston, motor);
    }

    @Override
    public SpindexerMap getSpindexerMap() {
        // Gear box introduces reduction
        final double GEAR_RATIO = 1 / 55.61;

        final PIDSparkMax motor = new PIDSparkMax(10, MotorType.kBrushless);
        final var rawMotor = motor.getMotorController();
        // Configure ramp rate to limit current draw
        rawMotor.setOpenLoopRampRate(0.5);
        // Configure Current limit to ensure we don't push too hard if something gets
        // jammed
        rawMotor.setSmartCurrentLimit(20);
        // We don't need the spindexer to hold it's position when disabled
        rawMotor.setIdleMode(IdleMode.kCoast);

        // Configure encoder distance scaling
        final var encoder = motor.getEncoder();
        encoder.setPositionScaleFactor(GEAR_RATIO);
        encoder.setVelocityScaleFactor(GEAR_RATIO);

        return new SpindexerMap(motor);
    }

    @Override
    public KickerMap getKickerMap() {
        final double GEAR_RATIO = 1 / 6;
        final PIDSparkMax motor = new PIDSparkMax(11, MotorType.kBrushless);
        final var rawMotor = motor.getMotorController();
        // Limit current to ensure we don't push too hard if we jam
        rawMotor.setSmartCurrentLimit(10);
        final var encoder = motor.getEncoder();
        encoder.setPositionScaleFactor(GEAR_RATIO);
        encoder.setVelocityScaleFactor(GEAR_RATIO);

        final AnalogTrigger ballSensor = new AnalogTrigger(0);
        // TODO Find the correct voltages for whatever sensor we use
        ballSensor.setLimitsVoltage(1.0, 1.5);

        return new KickerMap(motor, ballSensor::getTriggerState);
    }
    @Override
    public TurretMap getTurretMap() {
        final double GEAR_RATIO = 1 / 177.08;
        // Forward corresponds to clockwise rotation of the turret
        final var motor = new PIDSparkMax(12, MotorType.kBrushless);
        final var rawMotor = motor.getMotorController();
        // We want to be able to manually rotate the turret when disabled
        rawMotor.setIdleMode(IdleMode.kCoast);
        // Probably need to tune this, but we don't want to push to hard if the turret
        // is stuck. JVN calculator estimates 8A with highly conservative inputs
        rawMotor.setSmartCurrentLimit(8);
        // Limit how fast we change speed to ensure we don't accelerate suddenly
        rawMotor.setOpenLoopRampRate(1);
        // Configure Encoder Conversion factor so values are in degress rotation of the
        // turret
        final var encoder = motor.getEncoder();
        encoder.setPositionScaleFactor(GEAR_RATIO * 360);
        encoder.setVelocityScaleFactor(GEAR_RATIO * 360);

        // Limit switch represents a known angle on the turret
        // Limit switch returns true when turret hits it
        final var limitSwitch = new WDigitalInput(0);

        return new TurretMap(motor, limitSwitch);
    }
}