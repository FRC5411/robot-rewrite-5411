// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerIntake.IndexerIntake;
import frc.robot.subsystems.IndexerIntake.IndexerIntakeConstants;
import frc.robot.subsystems.Shooter.Shooter;

public class SmartFeed extends Command {
  /** Creates a new SmartFeed. */
  private IndexerIntake indexerIntake;
  private Shooter shooter;

  private static DigitalInput indexerSensor;

  private boolean indexerHasNote;


  public SmartFeed(IndexerIntake indexerIntake) {
    this.indexerIntake = indexerIntake;

    indexerSensor = new DigitalInput(IndexerIntakeConstants.INDEXER_SENSOR_ID);
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(indexerIntake);
  }

  public static boolean IndexerSensorHasNote(){
    return indexerSensor.get();
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    indexerHasNote = IndexerSensorHasNote();


    if(!indexerSensor.get() ){
      shooter.shooterIdle();
      indexerIntake.setIntakeSpeed(0.1);
      indexerIntake.setIndexerSpeed(0);
      // driver.getHID().setRumble(RumbleType.kBothRumble, 1);
      // operator.getHID().setRumble(RumbleType.kBothRumble, 1);
    } else{
      shooter.shooterIntake();
      indexerIntake.setIntakeSpeed(1.0);
      indexerIntake.setIndexerSpeed(0.25);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
