// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.IndexerIntake.IndexerIntake;
import frc.robot.subsystems.Shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnePieceMobilitySides extends SequentialCommandGroup {
  /** Creates a new OnePieceMobilitySides. */
  public OnePieceMobilitySides(
    Drive drive,
    IndexerIntake indexerIntake,
    Shooter shooter
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      shooter.shooterSubwoofer(),
      new WaitCommand(1),
      indexerIntake.INTAKE(1.0),
      new WaitCommand(0.25),
      drive.runSwerveCommand(
        ChassisSpeeds.fromRobotRelativeSpeeds(
          0.3 * drive.getMaxLinearSpeedMPS(),
          0.3 * drive.getMaxLinearSpeedMPS(),
          0.0,
          drive.getRotation()
          )
      )
    );
  }
}
