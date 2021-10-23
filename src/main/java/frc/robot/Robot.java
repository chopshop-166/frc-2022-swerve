// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.maps.RobotMap;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import io.github.oblarg.oblog.Logger;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends CommandRobot {

    private Command autonomousCommand;

    final private SendableChooser<Command> autoChooser = new SendableChooser<>();

    final private ButtonXboxController driveController = new ButtonXboxController(0);

    // Robot map initalization
    final private RobotMap map = getRobotMap(RobotMap.class, "frc.robot.maps", new RobotMap());

    private final Drive drive = new Drive(map.getDriveMap());
    private final Intake intake = new Intake(map.getIntakeMap());

    private final Spindexer spindexer = new Spindexer(map.getSpindexerMap());

    private final Kicker kicker = new Kicker(map.getKickerMap());

    private final Shooter shooter = new Shooter(map.getShooterMap());

    /**
     * This function sets up each controller to have the appropriate button mappings
     */
    @Override
    public void configureButtonBindings() {
        driveController.getButton(Button.kStart).whenPressed(drive.resetGyro());
        // No button bindings yet
    }

    @Override
    public void populateDashboard() {
        // TODO: Define dashboard
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
        // TODO: Define autonomous
    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        super.robotInit();
        Logger.configureLoggingAndConfig(this, false);
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();
        Logger.updateEntries();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        super.disabledInit();
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = autoChooser.getSelected();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }
}
