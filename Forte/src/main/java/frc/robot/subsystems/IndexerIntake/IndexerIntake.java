// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IndexerIntake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerIntake extends SubsystemBase {
  /** Creates a new Feeder. */
  private CANSparkMax intakeMotor;
  private CANSparkMax indexerMotor;
  private DigitalInput intakeSensor;
  private DigitalInput indexerSensor;

  public IndexerIntake() {
    intakeMotor = new CANSparkMax(Constants.Feeder.INTAKE_ID, MotorType.kBrushless);
    indexerMotor = new CANSparkMax(Constants.Feeder.INDEXER_ID, MotorType.kBrushless);

    intakeSensor = new DigitalInput(Constants.Feeder.INTAKE_SENSOR_ID);
    indexerSensor = new DigitalInput(Constants.Feeder.INDEXER_SENSOR_ID);

    config();
  }

  private void config(){
    indexerMotor.clearFaults();
    indexerMotor.restoreFactoryDefaults();
    indexerMotor.setSmartCurrentLimit(Constants.Feeder.INDEXER_CURRENT_LIMIT);
    indexerMotor.setIdleMode(IdleMode.kBrake);
    indexerMotor.setInverted(false);
    indexerMotor.burnFlash();

    intakeMotor.clearFaults();
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setSmartCurrentLimit(Constants.Feeder.INTAKE_CURRENT_LIMIT);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setInverted(false);
    intakeMotor.burnFlash();
  }


  public void setIntakeSpeed(double speed){
    intakeMotor.set(speed);
  }

  public void setIndexerSpeed(double speed){
    indexerMotor.set(speed);
  }

  //TODO: Check with Armaan
  public void smartFeed(){
    if(indexerSensor.get() && intakeSensor.get()){
      setIntakeSpeed(0.5);
      setIndexerSpeed(0.25);
    }
    else if(indexerSensor.get() && !intakeSensor.get()){
      setIntakeSpeed(0.1);
      setIndexerSpeed(0);
    } else{
      setIntakeSpeed(1.0);
      setIndexerSpeed(0.25);
    }
  }

  public boolean IndexerSensorHasNote(){
    return indexerSensor.get();
  }

  public boolean IntakeSensorHasNote(){
    return intakeSensor.get();
  }

  public void intakeFeedback(Joystick driver, Joystick operator){
    driver.setRumble(RumbleType.kBothRumble, 0.5);
    operator.setRumble(RumbleType.kBothRumble, 0.5);
  }

  public void eject(){
    setIntakeSpeed(-1.0);
    setIndexerSpeed(-1.0);
  }

  public void stopFeed(){
    setIntakeSpeed(0);
    setIndexerSpeed(0);
  }

  public void speakerScore(){
    setIndexerSpeed(1.0);
  }

  public void ampScore(){
    setIndexerSpeed(0.2);
  }

  public double currentIndexerAmp(){
    return indexerMotor.getOutputCurrent();
  }
  
  @Override
   public void periodic() {


    SmartDashboard.putBoolean("Intake Sensor Note Present", IntakeSensorHasNote());
    SmartDashboard.putBoolean("Indexer  Note Present", IndexerSensorHasNote());
    SmartDashboard.putNumber("Indexer Applied Output", currentIndexerAmp());
    
   }
}
