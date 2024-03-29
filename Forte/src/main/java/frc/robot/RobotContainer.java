// In Java We Trust

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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

public class RobotContainer {
  private Drive robotDrive;
  private IndexerIntake indexerIntake;
  private Shooter shooter;
  public static Boolean isField = true;

  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController operator = new CommandXboxController(1);

  private SmartFeed smartFeed;

  private SendableChooser<Command> autoChooser;


  
  public RobotContainer() {

    initializeSubsystems();

    configureAutonomous();

    try {
      autoChooser = AutoBuilder.buildAutoChooser();
    } catch (Exception e) {
      autoChooser = new SendableChooser<Command>(); 
      autoChooser.setDefaultOption("P3_C_N", shooter.shooterPodium());
    }


    SmartDashboard.putData("autoChooser", autoChooser);
    configureTriggers();

    // Use assisted control by default
    configureButtonBindings();

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

        smartFeed = 
        new SmartFeed(indexerIntake, operator);

        shooter = 
        new Shooter();

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

    public CommandXboxController getDriveController(){
        return driver;
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

    NamedCommands.registerCommand("Intake Note", smartFeed);
  }


  private void configureTriggers() {
    // For LEDS
  }

  /** Configure controllers */
  private void configureButtonBindings() {

    /* Drive with joysticks */
    robotDrive.setDefaultCommand(SwerveCommands.swerveDrive(
              robotDrive,
              () -> -driver.getLeftY(),
              () -> -driver.getLeftX(),
              () -> driver.getRightX(),
              true));
    
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
    operator.rightBumper().onFalse(indexerIntake.INTAKE((0)));

    operator.leftBumper().onTrue(indexerIntake.INTAKE((-1)));
    operator.leftBumper().onFalse(indexerIntake.INTAKE((0)));

    operator.rightTrigger().onTrue(indexerIntake.INTAKE((1)));
    operator.rightTrigger().onFalse(indexerIntake.INTAKE((0)));

    operator.leftTrigger().onTrue(indexerIntake.ampIntake());
    operator.leftTrigger().onFalse(indexerIntake.INTAKE((0)));

    operator.povLeft().whileTrue(shooter.shooterLaser());
    operator.povLeft().onFalse(shooter.shooterIdle());

    operator.povRight().whileTrue(shooter.shooterLob());
    operator.povRight().onFalse(shooter.shooterIdle());

    driver.x().onTrue(robotDrive.gyroReset());
  }

  public Command getAutonomousCommand() {
      return autoChooser.getSelected();
    }

  public void reset() {
    robotDrive.resetModules();
  }
}
