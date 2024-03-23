// In Java We Trust

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import frc.robot.commands.DriverIntakeFeedback;
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

  private CommandXboxController pilotController = new CommandXboxController(0);
  private CommandXboxController operator = new CommandXboxController(1);

  

  private LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    initializeSubsystems();

    configureAutonomous();

    // AutoBuilder is configured when Drive is initialized, thus chooser must be instantiated after
    // initializeSubsystems()
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
                new GyroIOPigeon2(false));

        indexerIntake = 
            new IndexerIntake();

    

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
  private void configureAutonomous() {}

  private void configureTriggers() {
    // For LEDS
  }

  /** Configure controllers */
  private void configureButtonBindings() {
      /* Drive with joysticks */
      robotDrive.setDefaultCommand(
          SwerveCommands.swerveDrive(
              robotDrive,
              () -> -pilotController.getLeftY(),
              () -> -pilotController.getLeftX(),
              () -> -pilotController.getRightX()));

      operator.leftTrigger().onTrue(new InstantCommand(() -> indexerIntake.eject()));
      operator.leftTrigger().onFalse(new InstantCommand(() -> indexerIntake.stopFeed()));
      operator.rightTrigger().onTrue(new InstantCommand(() -> indexerIntake.setIndexerSpeed(-1.0)));
      operator.rightTrigger().onTrue(new InstantCommand(() -> indexerIntake.setIntakeSpeed(-1.0)));
      operator.rightTrigger().onFalse(new InstantCommand(() -> indexerIntake.stopFeed()));
      operator.leftBumper().onTrue(new InstantCommand(() -> indexerIntake.setIndexerSpeed(0.5)));
      operator.leftBumper().onFalse(new InstantCommand(() -> indexerIntake.setIndexerSpeed(0)));
      operator.rightTrigger().whileTrue(new DriverIntakeFeedback(indexerIntake, pilotController, operator));

      pilotController.rightBumper().whileTrue(new InstantCommand(() -> shooter.movePivotSlowUp()));
      pilotController.rightBumper().whileFalse(new InstantCommand(() -> shooter.movePivotZero()));
      pilotController.leftBumper().whileTrue(new InstantCommand(() -> shooter.movePivotSlowDown()));
      pilotController.leftBumper().whileFalse(new InstantCommand(() -> shooter.movePivotZero()));

      operator.a().onTrue(new InstantCommand(() -> shooter.shootAmp()));
      operator.a().onFalse(new InstantCommand(() -> shooter.zeroShoot()));
      operator.b().onTrue(new InstantCommand(() -> shooter.shootSubwoofer()));
      operator.b().onFalse(new InstantCommand(() -> shooter.zeroShoot()));
      operator.x().onTrue(new InstantCommand(() -> shooter.shootFeed()));
      operator.x().onFalse(new InstantCommand(() -> shooter.zeroShoot()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void reset() {
    robotDrive.resetModules();
  }
}
