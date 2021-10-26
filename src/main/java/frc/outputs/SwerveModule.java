package frc.outputs;

import com.chopshop166.chopshoplib.outputs.PIDSparkMax;
import com.chopshop166.chopshoplib.outputs.SmartMotorController;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

public class SwerveModule {

    // Overall gear ratio for the swerve module drive motor
    private static final double GEAR_RATIO = 1 / 6.86;
    private static final double WHEEL_DIAMETER_M = Units.inchesToMeters(4);
    private static final double K_P = 0.0043;
    private static final double K_I = 0.00;
    private static final double K_D = 0.0001;
    private static final double K_I_STEADY_STATE = 0.0035;
    private static final double I_TRANSITION_POINT = 5;

    private final String name;
    private final Translation2d location;
    private final CANCoder steeringEncoder;
    private final SmartMotorController steeringController;
    private final PIDController steeringPID;
    private final SmartMotorController driveController;

    private double previousSpeed = 0;

    public SwerveModule(final String name, final Translation2d moduleLocation, final CANCoder steeringEncoder,
            final SmartMotorController steeringController, final SmartMotorController driveController) {
        this.name = name;
        this.location = moduleLocation;
        this.steeringEncoder = steeringEncoder;
        this.steeringController = steeringController;
        this.driveController = driveController;
        this.steeringPID = new PIDController(K_P, K_I, K_D);
        this.steeringPID.enableContinuousInput(-180, 180);
    }

    /**
     * Get the modules location in relation to the CoM of the robot.
     *
     * @return Location2d object representing the offset
     */
    public Translation2d getLocation() {
        return location;
    }

    /**
     * Process the desired state and set the output values for the motor controllers
     *
     *
     */
    public void setDesiredState(final SwerveModuleState desiredState) {
        final SwerveModuleState state = calculateSteeringAngle(desiredState);

        // Run Steering angle PID to calculate output since the Spark Max can't take
        // advantage of the Cancoder
        final double angleOutput = steeringPID.calculate(getAngle().getDegrees(), state.angle.getDegrees());
        steeringController.set(angleOutput);

        // Changes the K_I as the angle error gets small
        if (Math.abs(steeringPID.getPositionError()) <= I_TRANSITION_POINT) {
            steeringPID.setI(K_I_STEADY_STATE);
        } else {
            steeringPID.setI(K_I);
        }

        // Set the drive motor output speed
        if (state.speedMetersPerSecond == 0 && previousSpeed != 0) {
            ((PIDSparkMax) driveController).getPidController().setIAccum(0);
        }
        previousSpeed = state.speedMetersPerSecond;

        driveController.setSetpoint(state.speedMetersPerSecond);
    }

    /**
     * Configures a PIDSparkMax for use as the drive motor on a MK3 swerve module
     *
     * @param motor Drive motor controller to configure
     * @return Drive motor controller for chaining
     */
    public static PIDSparkMax configureDriveMotor(final PIDSparkMax motor) {
        // Get raw objects from the PIDSparkMax
        final CANSparkMax sparkMax = motor.getMotorController();
        final CANEncoder encoder = motor.getEncoder().getRaw();
        final CANPIDController pid = motor.getPidController();

        // Set Motor controller configuration
        motor.setControlType(ControlType.kVelocity);
        sparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
        // Set velocity conversion to convert RPM to M/s
        encoder.setVelocityConversionFactor((GEAR_RATIO * Math.PI * WHEEL_DIAMETER_M) / 60);
        // Set Position conversion to convert from Rotations to M
        encoder.setPositionConversionFactor(GEAR_RATIO * Math.PI * WHEEL_DIAMETER_M);

        // Configure PID
        // https: // docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control
        pid.setP(0.0);
        pid.setI(0.00015);
        pid.setD(0.0);
        pid.setFF(0.219);

        // Return the original object so this can be chained
        return motor;
    }

    /**
     * Returns the current angle of the swerve module
     *
     * @return The current angle of the module
     */
    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(steeringEncoder.getAbsolutePosition());
    }

    /**
     * Optimizes the desired module angle by taking into account the current module
     * angle
     *
     * @param desiredState The module state as calculated by a SwerveDriveKinematics
     *                     object
     * @return The optimized module state
     */
    private SwerveModuleState calculateSteeringAngle(final SwerveModuleState desiredState) {
        return SwerveModuleState.optimize(desiredState, getAngle());
    }
}
