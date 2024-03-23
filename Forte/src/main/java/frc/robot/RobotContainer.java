// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.DriverIntakeFeedback;
import frc.robot.subsystems.IndexerIntake.IndexerIntake;

  
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final IndexerIntake IndexerIntake = new IndexerIntake();
    private SendableChooser<Command> autoChooser;


    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
   
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        //zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro(), s_Swerve));
        operator.leftTrigger().onTrue(new InstantCommand(() -> IndexerIntake.eject()));
        operator.leftTrigger().onFalse(new InstantCommand(() -> IndexerIntake.stopFeed()));
        operator.rightTrigger().onTrue(new InstantCommand(() -> IndexerIntake.smartFeed()));
        operator.rightTrigger().onFalse(new InstantCommand(() -> IndexerIntake.stopFeed()));
        operator.rightTrigger().whileTrue(new DriverIntakeFeedback(IndexerIntake, driver, operator));
        //opX.onTrue(s_Feeder.setShooterToSpeaker());
    }

    public CommandXboxController getDriveController(){
        return driver;
      }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
}