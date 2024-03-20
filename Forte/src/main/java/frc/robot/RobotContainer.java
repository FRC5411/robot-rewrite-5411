// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.subsystems.Winch;
import frc.robot.subsystems.Swerve.Swerve;
//import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.Autos;
import frc.robot.commands.TeleopSwerve;
import frc.robot.lib.math.LauncherInterpolation;
import frc.robot.lib.util.AxisButton;

import java.time.Instant;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

  
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final Swerve s_Swerve = new Swerve();
    private SendableChooser<Command> autoChooser;


    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final Joystick driver = new Joystick(0);
    //private final Joystick operator = new Joystick(1);
    // The robot's subsystems and commands are defined here...
    // private final JoystickButton rotateWithTag = new JoystickButton(driver, XboxController.Button.kRightStick.value);
    // //private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
    // private final JoystickButton driveA = new JoystickButton(driver, XboxController.Button.kA.value);
    // private final JoystickButton driveY = new JoystickButton(driver, XboxController.Button.kY.value);
    // private final JoystickButton driveB = new JoystickButton(driver, XboxController.Button.kB.value);
    // private final JoystickButton driveX = new JoystickButton(driver, XboxController.Button.kX.value);
    //  private final JoystickButton driveLeftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton driveRightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    // private final JoystickButton driveStart = new JoystickButton(driver, XboxController.Button.kStart.value);
    // private final JoystickButton driveSelect = new JoystickButton(driver, XboxController.Button.kBack.value);
    // private final AxisButton driveLeftTrigger = new AxisButton(driver, 2, 0.5);
    // private final AxisButton driveRightTrigger = new AxisButton(driver, 3, 0.5);

    // private final JoystickButton opLeftStick = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
    // private final JoystickButton opRightStick = new JoystickButton(operator, XboxController.Button.kRightStick.value);
    // private final JoystickButton opY = new JoystickButton(operator, XboxController.Button.kY.value);
    // private final JoystickButton opA = new JoystickButton(operator, XboxController.Button.kA.value);
    // private final JoystickButton opB = new JoystickButton(operator, XboxController.Button.kB.value);
    // private final JoystickButton opX = new JoystickButton(operator, XboxController.Button.kX.value);
    // private final POVButton povUp = new POVButton(operator, 0);
    // private final POVButton povDown = new POVButton(operator, 180);
    // private final POVButton povRight = new POVButton(operator, 90);
    // private final POVButton povLeft = new POVButton(operator, 270);
    // private final JoystickButton opStart = new JoystickButton(operator, XboxController.Button.kStart.value);
    // private final JoystickButton opSelect = new JoystickButton(operator, XboxController.Button.kBack.value);
    // private final JoystickButton opLeftBumper = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton opRightBumper = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    // private final AxisButton opLeftTrigger = new AxisButton(operator, 2, 0.5);
    // private final AxisButton opRightTrigger = new AxisButton(operator, 3, 0.5);




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false));

       
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
        
    }

    public Joystick getDriveController(){
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