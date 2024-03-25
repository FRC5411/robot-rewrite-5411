// In Java We Trust

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.DriverIntakeFeedback;
import frc.robot.commands.SmartFeed;
import frc.robot.commands.SwerveCommands;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Drive.GyroIO;
import frc.robot.subsystems.Drive.GyroIOPigeon2;
import frc.robot.subsystems.Drive.ModuleIO;
import frc.robot.subsystems.Drive.ModuleIOSim;
import frc.robot.subsystems.Drive.ModuleIOSparkMax;
import frc.robot.subsystems.IndexerIntake.IndexerIntake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class RobotContainer {
  private Drive robotDrive;
  private IndexerIntake indexerIntake;
  private Shooter shooter;
  public static Boolean isField = true;

  private CommandXboxController pilotController = new CommandXboxController(0);
  private CommandXboxController operator = new CommandXboxController(1);

  private LoggedDashboardChooser<Command> AutonChooser;

  private SmartFeed smartFeed;


  public RobotContainer() {

    

    initializeSubsystems();

    configureAutonomous();

    // AutoBuilder is configured when Drive is initialized, thus chooser must be instantiated after
    // initializeSubsystems()
    configureTriggers();

    // Use assisted control by default
    configureButtonBindings();

    try {
      AutonChooser =
          new LoggedDashboardChooser<>("Autonomous Selector", AutoBuilder.buildAutoChooser());
    } catch (Exception e) {
      AutonChooser = new LoggedDashboardChooser<>("Autonomous Selector");
      AutonChooser.addDefaultOption("New Auto", shooter.shooterPodium());
    }
  }

  /** Instantiate subsystems */
  private void initializeSubsystems() {
    switch (Constants.currentMode) {
      case REAL:
        robotDrive =
            new Drive(
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3),
                new GyroIOPigeon2());

        indexerIntake = 
            new IndexerIntake();
        smartFeed = new SmartFeed(indexerIntake);

    

        shooter = new Shooter();
        break;
      case SIM:
        robotDrive =
            new Drive(
                new ModuleIOSim(0),
                new ModuleIOSim(1),
                new ModuleIOSim(2),
                new ModuleIOSim(3),
                new GyroIO() {});
        break;
      default:
        robotDrive =
            new Drive(
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new GyroIO() {});
        break;
    }
  }

  /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */


    public CommandXboxController getDriveController(){
        return pilotController;
      }

  /** Register commands with PathPlanner and add default autos to chooser */
  private void configureAutonomous() {
    NamedCommands.registerCommand("Shoot at Podium", shooter.shooterPodium());

    NamedCommands.registerCommand("Shoot at Amp", shooter.shooterAmp());
    
    NamedCommands.registerCommand("Shoot at Wing", shooter.shooterWing());

    NamedCommands.registerCommand("Shoot at subwoofer", shooter.shooterSubwoofer());

    NamedCommands.registerCommand("Shoot at idle", shooter.shooterIdle());

    NamedCommands.registerCommand("Forward Note Shoot", indexerIntake.INTAKE(1));

    NamedCommands.registerCommand("Forward Note Amp", indexerIntake.ampIntake());

    NamedCommands.registerCommand("Intake Note", indexerIntake.smartFeedCommand());
  }


  private void configureTriggers() {
    // For LEDS
  }

  /** Configure controllers */
  private void configureButtonBindings() {

    /* Drive with joysticks */
    robotDrive.setDefaultCommand(SwerveCommands.swerveDrive(
              robotDrive,
              () -> -pilotController.getLeftY(),
              () -> -pilotController.getLeftX(),
              () -> pilotController.getRightX(),
              isField));

    pilotController.a().toggleOnTrue(new InstantCommand(() -> isField  = !isField));
    pilotController.a().toggleOnFalse(new InstantCommand(() -> isField = isField));

    
    operator.b().whileTrue(shooter.shooterSubwoofer());
    operator.b().whileFalse(shooter.shooterIdle());

    operator.a().whileTrue(shooter.shooterPodium());
    operator.a().whileFalse(shooter.shooterIdle());

    operator.x().whileTrue(shooter.shooterAmp());
    operator.x().whileFalse(shooter.shooterIdle());

    operator.y().whileTrue(shooter.shooterFeed());
    operator.y().whileFalse(shooter.shooterIdle());

    operator.povUp().whileTrue(shooter.shooterWing());
    operator.povUp().whileFalse(shooter.shooterIdle());

    operator.rightBumper().whileTrue(smartFeed);
    operator.rightBumper().onFalse(indexerIntake.INTAKE(0));

    operator.leftBumper().onTrue(indexerIntake.INTAKE(-1));
    operator.leftBumper().onFalse(indexerIntake.INTAKE(0));

    operator.rightTrigger().onTrue(indexerIntake.INTAKE(1));
    operator.rightTrigger().onFalse(indexerIntake.INTAKE(0));

    operator.leftTrigger().onTrue(indexerIntake.ampIntake());
    operator.leftTrigger().onFalse(indexerIntake.INTAKE(0));
    operator.povLeft().whileTrue(shooter.shooterLaser());
    operator.povLeft().onFalse(shooter.shooterIdle());
    operator.povRight().whileTrue(shooter.shooterLob());
    operator.povRight().onFalse(shooter.shooterIdle());

    pilotController.leftTrigger().onTrue(indexerIntake.smartFeedCommand());
    pilotController.leftTrigger().onFalse(indexerIntake.INTAKE(0));
    pilotController.x().onTrue(robotDrive.gyroReset());
  }

  public Command getAutonomousCommand() {
    return AutonChooser.get();
  }

  public void reset() {
    robotDrive.resetModules();
  }
}
