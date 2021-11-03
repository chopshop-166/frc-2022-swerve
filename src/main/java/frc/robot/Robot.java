// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;
import com.chopshop166.chopshoplib.controls.ButtonXboxController.Direction;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter.Speed;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.utils.SpinDirection;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends CommandRobot {

    final private ButtonXboxController driveController = new ButtonXboxController(0);
    final private ButtonXboxController copilotController = new ButtonXboxController(1);

    // Robot map initalization
    final private RobotMap map = getRobotMap(RobotMap.class, "frc.robot.maps", new RobotMap());

    private final Drive drive = new Drive(map.getDriveMap());
    private final Intake intake = new Intake(map.getIntakeMap());

    private final Spindexer spindexer = new Spindexer(map.getSpindexerMap());

    private final Kicker kicker = new Kicker(map.getKickerMap());

    private final Vision vision = new Vision();

    private final Turret turret = new Turret(map.getTurretMap(), vision);

    private final Shooter shooter = new Shooter(map.getShooterMap());

    public CommandBase shootPowerCell(final Speed speed) {
        return sequence(speed.getName(), shooter.spinUpToSpeed(speed), kicker.start(),
                spindexer.spin(SpinDirection.CLOCKWISE));
    }

    /**
     * This function sets up each controller to have the appropriate button mappings
     */
    @Override
    public void configureButtonBindings() {
        driveController.getButton(Button.kStart).whenPressed(drive.resetGyro());
        driveController.getButton(Button.kA).whileHeld(intake.runIntake(true)).whileHeld(spindexer.washerMachine())
                .whenReleased(spindexer.stop());
        // Shooter mappings
        driveController.getButton(Button.kB).whileHeld(shootPowerCell(Speed.GOAL_BASE))
                .whenReleased(parallel("Stop", shooter.spinDown(), kicker.stop()));

        // Turret mappings
        driveController.getButton(Button.kBumperRight).whileHeld(turret.slowRotate(SpinDirection.CLOCKWISE));
        driveController.getButton(Button.kBumperLeft).whileHeld(turret.slowRotate(SpinDirection.COUNTERCLOCKWISE));

        // Secondary functions mapped to the dpad
        driveController.getPovButton(Direction.Left).whileHeld(intake.runIntake(false));
        driveController.getPovButton(Direction.Up).whileHeld(kicker.run(true));
        driveController.getPovButton(Direction.Right).whileHeld(spindexer.washerMachine())
                .whenReleased(spindexer.stop());
        driveController.getPovButton(Direction.Down).whileHeld(kicker.run(false));

        copilotController.getButton(Button.kA).whileHeld(intake.runIntake(true)).whileHeld(spindexer.washerMachine())
                .whenReleased(spindexer.stop());
        copilotController.getButton(Button.kY).whileHeld(intake.runIntake(false));

        copilotController.getButton(Button.kBumperRight).whileHeld(turret.slowRotate(SpinDirection.CLOCKWISE));
        copilotController.getButton(Button.kBumperLeft).whileHeld(turret.slowRotate(SpinDirection.COUNTERCLOCKWISE));
        copilotController.getButton(Button.kB).whileHeld(shootPowerCell(Speed.GOAL_BASE))
                .whenReleased(parallel("Stop", shooter.spinDown(), kicker.stop()));
        copilotController.getPovButton(Direction.Right).whileHeld(spindexer.spin(SpinDirection.CLOCKWISE))
                .whenReleased(spindexer.stop());
        copilotController.getPovButton(Direction.Left).whileHeld(spindexer.spin(SpinDirection.COUNTERCLOCKWISE))
                .whenReleased(spindexer.stop());
    }

    @Override
    public void populateDashboard() {
        // Add Intake commands
        SmartDashboard.putData("Extend Intake", intake.extendIntake());
        SmartDashboard.putData("Retract Intake", intake.retractIntake());
        SmartDashboard.putData("Deploy Intake", intake.deployIntake());
        SmartDashboard.putData("ShootClose", shooter.spinUpToSpeed(Speed.GOAL_BASE));
        SmartDashboard.putData("Rotate Forward", turret.aimForward());
    }

    /**
     *
     */
    @Override
    public void setDefaultCommands() {
        drive.setDefaultCommand(drive.fieldCentricDrive(() -> driveController.getX(Hand.kLeft),
                () -> driveController.getY(Hand.kLeft), () -> driveController.getX(Hand.kRight)));
    }

    @Override
    public void populateAutonomous() {
        addAutonomous("Driveoff", drive.driveDistanceY(1.2));
    }

}
