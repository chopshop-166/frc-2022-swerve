// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.chopshop166.chopshoplib.PersistenceCheck;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.outputs.SmartMotorController;
import com.chopshop166.chopshoplib.sensors.IEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap.ShooterMap;

public class Shooter extends SmartSubsystemBase {

    private final static double MAX_SPEED = 8500;
    private final static double TRENCH_SHOT_SPEED = 4500;
    private final static double INITIATION_LINE_SPEED = 3500;
    private final static double SPIT_OUT_SPEED = 2000;

    private final static double ROLLER_SPEED = 3000;

    private final SmartMotorController shooter;
    private final IEncoder shooterEncoder;
    private final SmartMotorController roller;
    private final IEncoder rollerEncoder;
    private final SmartMotorController hood;

    public Shooter(ShooterMap map) {
        shooter = map.getMotor();
        shooterEncoder = shooter.getEncoder();
        roller = map.getRoller();
        rollerEncoder = roller.getEncoder();
        hood = map.getHood();
    }

    public CommandBase spinUpToSpeed(String name, DoubleSupplier speed) {
        final PersistenceCheck check = new PersistenceCheck(5,
                () -> Math.abs(shooterEncoder.getRate() - Math.min(speed.getAsDouble(), MAX_SPEED)) <= 50.0);
        // The roller check is less restrictive as it should have less impact
        final PersistenceCheck rollerCheck = new PersistenceCheck(5,
                () -> Math.abs(rollerEncoder.getRate() - ROLLER_SPEED) <= 200.0);
        return initAndWait(name, () -> {
            shooter.setSetpoint(Math.min(speed.getAsDouble(), MAX_SPEED));
            check.reset();
            rollerCheck.reset();
        }, () -> check.getAsBoolean() && rollerCheck.getAsBoolean());
    }

    public CommandBase speedUpForTrenchShot() {
        return spinUpToSpeed("Trench Shot", () -> TRENCH_SHOT_SPEED);
    }

    public CommandBase speedUpForInitiationLine() {
        return spinUpToSpeed("Initiation Line", () -> INITIATION_LINE_SPEED);
    }

    // Stop Motor instead of setting to 0 so that it spins down on its own
    public CommandBase spinDown() {
        return instant("Spin Down", () -> {
            shooter.stopMotor();
            roller.stopMotor();
        });
    }

    public CommandBase spitOutBall() {
        return spinUpToSpeed("Spit Out", () -> SPIT_OUT_SPEED);
    }

    // Get estimated distance to the target and calculate the shooter speed
    public CommandBase autoSpinUp() {
        // TODO collect data to develop model of speed vs distance
        return spinUpToSpeed("Trench Shot", () -> ROLLER_SPEED);
    }

    @Override
    public void reset() {
        shooter.stopMotor();
        roller.stopMotor();
        hood.stopMotor();
    }

}
