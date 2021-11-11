// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.commands.SmartSubsystemBase;
import com.chopshop166.chopshoplib.outputs.IDSolenoid;
import com.chopshop166.chopshoplib.outputs.SmartMotorController;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.maps.RobotMap.IntakeMap;

public class Intake extends SmartSubsystemBase {
    private final static double INTAKE_SPEED = 0.75;
    private final static double INTAKE_DEPLOY_SPEED = 0.2;
    private final static double INTAKE_DEPLOY_ROTATIONS = 1;

    private final IDSolenoid piston;
    private final SmartMotorController motor;

    public Intake(final IntakeMap map) {
        super();
        piston = map.getPiston();
        motor = map.getMotor();
    }

    public CommandBase runIntake(final boolean intake) {
        return startEnd("Intake", () -> {
            piston.set(Value.kForward);
            // If we're intaking than run forwards
            // If we're removing a ball run backwards
            motor.set(intake ? INTAKE_SPEED : -INTAKE_SPEED);
        }, () -> {
            piston.set(Value.kReverse);
            motor.set(0.0);
        });
    }

    public CommandBase extendIntake() {
        return instant("Extend", () -> piston.set(Value.kForward));
    }

    public CommandBase retractIntake() {
        return instant("Retract", () -> piston.set(Value.kReverse));
    }

    // We need to "Deploy" the intake at the start of the match.
    // To do this we just need to rotate the intake a bit
    // For safety sake we will also put a timelimit on this so we don't stall the
    // motor for 15 seconds if something gets stuck in the intake
    public CommandBase deployIntake() {
        return deadline("Deploy Intake Deadline", parallel("Deploy Intake", new StartEndCommand(() -> {
            motor.set(INTAKE_DEPLOY_SPEED);
        }, () -> {
            motor.set(0.0);
        }, this), new WaitUntilCommand(() -> motor.getEncoder().getDistance() >= INTAKE_DEPLOY_ROTATIONS)),
                new WaitCommand(2));
    }

    @Override
    public void reset() {
        motor.getEncoder().reset();
        motor.set(0);
    }

    @Override
    public void safeState() {
        motor.stopMotor();
    }
}
