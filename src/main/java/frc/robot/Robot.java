// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.chopshop166.chopshoplib.Autonomous;
import com.chopshop166.chopshoplib.commands.CommandRobot;
import com.chopshop166.chopshoplib.controls.ButtonXboxController;
import com.chopshop166.chopshoplib.controls.ButtonXboxController.POVDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.maps.RobotMap;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.Speed;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
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

    @Autonomous(defaultAuto = true)
    public CommandBase driveOff = drive.driveDistanceY(1.2).withName("Driveoff");

    public CommandBase shootPowerCell(final Speed speed) {
        return sequence(speed.getName(), shooter.spinUpToSpeed(speed), kicker.start(),
                spindexer.spin(SpinDirection.COUNTERCLOCKWISE, Spindexer.Speeds.SHOOTING));
    }

    public CommandBase aimTurret() {
        return sequence("Aim Turret", vision.driverMode(false), turret.aimTurret());
    }

    /**
     * This function sets up each controller to have the appropriate button mappings
     */
    @Override
    public void configureButtonBindings() {
        driveController.start().whenPressed(drive.resetGyro());
        driveController.a().whileHeld(intake.runIntake(true))
                .whileHeld(spindexer.washerMachine())
                .whenReleased(spindexer.stop());
        // Shooter mappings
        driveController.b().whileHeld(shootPowerCell(Speed.GOAL_BASE))
                .whenReleased(parallel("Stop", shooter.spinDown(), kicker.stop()));

        // Turret mappings
        driveController.rbumper().whileHeld(turret.slowRotate(SpinDirection.CLOCKWISE));
        driveController.lbumper().whileHeld(turret.slowRotate(SpinDirection.COUNTERCLOCKWISE));

        // Secondary functions mapped to the dpad
        driveController.getPovButton(POVDirection.LEFT).whileHeld(intake.runIntake(false));
        driveController.getPovButton(POVDirection.UP).whileHeld(kicker.run(true));
        driveController.getPovButton(POVDirection.RIGHT).whileHeld(spindexer.washerMachine())
                .whenReleased(spindexer.stop());
        driveController.getPovButton(POVDirection.DOWN).whileHeld(kicker.run(false));

        copilotController.a().whileHeld(intake.runIntake(true))
                .whileHeld(spindexer.washerMachine())
                .whenReleased(spindexer.stop());
        copilotController.b().whileHeld(shootPowerCell(Speed.GOAL_BASE))
                .whenReleased(parallel("Stop", shooter.spinDown(), kicker.stop()));
        copilotController.y().whileHeld(intake.runIntake(false));
        copilotController.x().whileHeld(aimTurret()).whenReleased(vision.driverMode(true));

        copilotController.rbumper().whileHeld(turret.slowRotate(SpinDirection.CLOCKWISE));
        copilotController.lbumper()
                .whileHeld(turret.slowRotate(SpinDirection.COUNTERCLOCKWISE));
        copilotController.b().whileHeld(shootPowerCell(Speed.GOAL_BASE))
                .whenReleased(parallel("Stop", shooter.spinDown(), kicker.stop()));
        copilotController.getPovButton(POVDirection.RIGHT)
                .whileHeld(spindexer.spin(SpinDirection.CLOCKWISE, Spindexer.Speeds.CLEARING))
                .whenReleased(spindexer.stop());
        copilotController.getPovButton(POVDirection.LEFT)
                .whileHeld(spindexer.spin(SpinDirection.COUNTERCLOCKWISE, Spindexer.Speeds.CLEARING))
                .whenReleased(spindexer.stop());
        copilotController.getPovButton(POVDirection.UP).whileHeld(kicker.run(true));
        copilotController.getPovButton(POVDirection.DOWN).whileHeld(kicker.run(false));
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
        drive.setDefaultCommand(drive.fieldCentricDrive(driveController::getLeftX, driveController::getLeftY,
                driveController::getRightX));
    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        super.robotInit();
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();
    }
}
