// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.outputs.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.maps.RobotMap.TurretMap;

public class Turret extends SmartSubsystemBase {

    // Speed constants
    private final static double ZERO_SPEED_ONE = 0.5;
    private final static double ZERO_SPEED_TWO = 0.2;
    private final static double AIMING_SPEED = 0.3;

    // Angle Constants
    private final static double STOW_POSITION = 90;
    private final static double FORWARD_POSITION = 0;

    // Angle Error
    private final static double POSITION_ERROR = 2;

    private final SmartMotorController motor;
    private final IEncoder encoder;
    private final BooleanSupplier limitSwitch;

    public Turret(final TurretMap map) {
        motor = map.getMotor();
        encoder = motor.getEncoder();
        limitSwitch = map.getLimitSwitch();
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

    public CommandBase stowTurret() {
        return moveToAngle(STOW_POSITION);
    }

    public CommandBase aimForward() {
        return moveToAngle(FORWARD_POSITION);
    }

    public CommandBase slowRotate(final boolean clockwise) {
        return startEnd("Rotate Turret", () -> {
            motor.set(clockwise ? ZERO_SPEED_TWO : -ZERO_SPEED_TWO);
        }, () -> motor.set(0));
    }

    // Simple Bang-Bang position control
    // If this doesn't work well we can investigate using PID
    public CommandBase moveToAngle(double angle) {
        return functional("Move to " + angle, () -> {
        }, () -> {
            double angleToMove = angle - encoder.getAbsolutePosition();
            motor.set(Math.signum(angleToMove) * AIMING_SPEED);
        }, (interrupted) -> {
            motor.set(0);
        }, () -> {
            return Math.abs(angle - encoder.getAbsolutePosition()) <= POSITION_ERROR;
        });
    }

    private void checkZero() {
        // If the limit switch is pressed but we don't think we should be near the limit
        // switch than lets reset the encoder
        // We should also indicate to the driver that we should do a full zero procedure
        if (limitSwitch.getAsBoolean() && (Math.abs(encoder.getAbsolutePosition()) > POSITION_ERROR)) {
            encoder.reset();
            // TODO add indicator for the driver
        }
    }

    @Override
    public void reset() {
        motor.set(0);
    }

    @Override
    public void periodic() {
        checkZero();
    }
}
