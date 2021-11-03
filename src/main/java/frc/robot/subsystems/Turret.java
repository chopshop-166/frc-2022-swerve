// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.outputs.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.maps.RobotMap.TurretMap;
import frc.utils.SpinDirection;

public class Turret extends SmartSubsystemBase {

    // Speed constants
    private final static double ZERO_SPEED_ONE = 0.2;
    private final static double ZERO_SPEED_TWO = 0.1;
    private final static double AIMING_SPEED = 0.1;

    // Angle Constants
    private final static double FORWARD_POSITION = 90;

    // Angle Error
    private final static double POSITION_ERROR = 2;

    // PID Values
    private final static double K_P = 0;
    private final static double K_I = 0;
    private final static double K_D = 0;

    private final SmartMotorController motor;
    private final IEncoder encoder;
    private final BooleanSupplier limitSwitch;
    private final Vision vision;

    private final PIDController pid;

    public Turret(final TurretMap map, final Vision vision) {
        motor = map.getMotor();
        encoder = motor.getEncoder();
        limitSwitch = map.getLimitSwitch();
        this.vision = vision;
        pid = new PIDController(K_P, K_I, K_D);
    }

    public CommandBase zeroTurretPhaseOne() {
        return cmd("Zero Turret-1").onExec(() -> {
            // Turn clockwise direction towards the limit switch
            motor.set(ZERO_SPEED_ONE);
        }).onEnd((interrupted) -> {
            motor.set(0.0);
            // Reset the encoder once we've hit it
            // This may not be as precise
            encoder.reset();
        }).finishedWhen(limitSwitch::getAsBoolean).build();
    }

    public CommandBase zeroTurretPhaseTwo() {
        return cmd("Zero Turret-2").onExec(() -> {
            // Turn counter-clockwise direction away from the limit switch
            motor.set(-ZERO_SPEED_TWO);
        }).onEnd((interrupted) -> {
            motor.set(0.0);
        }).finishedWhen(() -> {
            return !limitSwitch.getAsBoolean();
        }).build();
    }

    public CommandBase zeroTurretPhaseThree() {
        return cmd("Zero Turret-3").onExec(() -> {
            // Turn counter-clockwise direction away from the limit switch
            motor.set(ZERO_SPEED_TWO);
        }).onEnd((interrupted) -> {
            motor.set(0.0);
            // Reset the encoder again now that we've approached it slowly
            encoder.reset();
        }).finishedWhen(limitSwitch::getAsBoolean).build();
    }

    // Turn clockwise relatively quickly until turret hits limit switch
    // Back off of limit switch slowly
    // Turn clockwise until turret hits limit switch at low spseed for increased
    // accuracy
    public CommandBase zeroTurret() {
        return new SequentialCommandGroup(zeroTurretPhaseOne(), zeroTurretPhaseTwo(), zeroTurretPhaseThree());
    }

    public CommandBase aimForward() {
        return moveToAngle(FORWARD_POSITION);
    }

    public CommandBase slowRotate(final SpinDirection direction) {
        return startEnd("Rotate Turret", () -> {
            motor.set(direction == SpinDirection.CLOCKWISE ? ZERO_SPEED_TWO : -ZERO_SPEED_TWO);
        }, () -> motor.set(0));
    }

    // Simple Bang-Bang position control
    // If this doesn't work well we can investigate using PID
    public CommandBase moveToAngle(final double angle) {
        return functional("Move to " + angle, () -> {
        }, () -> {
            final double angleToMove = angle - encoder.getDistance();
            motor.set(Math.signum(angleToMove) * AIMING_SPEED);
        }, (interrupted) -> {
            motor.set(0);
        }, () -> {
            return Math.abs(angle - encoder.getDistance()) <= POSITION_ERROR;
        });
    }

    private void checkZero() {
        // If the limit switch is pressed but we don't think we should be near the limit
        // switch than lets reset the encoder
        // We should also indicate to the driver that we should do a full zero procedure
        if (limitSwitch.getAsBoolean() && Math.abs(encoder.getDistance()) > POSITION_ERROR) {
            encoder.reset();
            // TODO add indicator for the driver
        }
    }

    public CommandBase aimTurret() {
        final PersistenceCheck check = new PersistenceCheck(5, () -> Math.abs(vision.getRotation(AIMING_SPEED)) <= 1.5);
        return functional("Auto Aim", () -> {
            check.reset();
        }, () -> {
            motor.set(Math.signum(vision.getRotation(AIMING_SPEED)) * AIMING_SPEED);
        }, (interrupted) -> {
            motor.set(0);
        }, () -> {
            return check.getAsBoolean();
        });
    }

    public CommandBase aimTurretPID() {
        final PersistenceCheck check = new PersistenceCheck(5, () -> Math.abs(vision.getRotation(AIMING_SPEED)) <= 1.5);
        return functional("Auto Aim", () -> {
            check.reset();
        }, () -> {
            pid.calculate(vision.getRotation(AIMING_SPEED));
            motor.set(Math.signum(vision.getRotation(AIMING_SPEED)) * AIMING_SPEED);
        }, (interrupted) -> {
            motor.set(0);
        }, () -> {
            return check.getAsBoolean();
        });
    }

    @Override
    public void reset() {
        motor.set(0);
        encoder.reset();
    }

    @Override
    public void periodic() {
        checkZero();
        SmartDashboard.putNumber("Turret Angle", encoder.getDistance());
    }
}
