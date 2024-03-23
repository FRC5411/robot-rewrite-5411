// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  /** Creates a new Feeder. */
  private CANSparkMax pivotMotor;
  private DutyCycleEncoder pivotEncoder;
  private TalonFX topMotor;
  private TalonFX bottomMotor;
  private double pivotGearRatio; 
  private ProfiledPIDController pivotController;
  
  public Shooter() {
    pivotMotor = new CANSparkMax(ShooterConstants.PIVOT_PORT, MotorType.kBrushless);
    topMotor = new TalonFX(ShooterConstants.UPPER_ROLLER_PORT, "drivetrain");
    bottomMotor = new TalonFX(ShooterConstants.LOWER_ROLLER_PORT, "drivetrain");
    pivotEncoder = new DutyCycleEncoder(ShooterConstants.PIVOT_ENCODER_PORT);

    pivotController = new ProfiledPIDController(
      ShooterConstants.PIVOT_P,
      ShooterConstants.PIVOT_I,
      ShooterConstants.PIVOT_D, 
      new TrapezoidProfile.Constraints(2, 2));

    pivotGearRatio =  1 / 144;

    config();
  }

  public double getPivotAngle(){
    return (pivotEncoder.getAbsolutePosition() * pivotGearRatio * 360 ) - ShooterConstants.PIVOT_OFFSET;
  }

  public void shootSubwoofer(){
    topMotor.set(0.6);
    bottomMotor.set(0.6);
  }

  public void shootFeed(){
    topMotor.set(0.4);
    bottomMotor.set(0.4);
  }

  public void shootAmp(){
    topMotor.set(0);
    bottomMotor.set(0.3);
  }  

  public void zeroShoot(){
    topMotor.set(0);
    bottomMotor.set(0);
  }

  public void reverseShoot(){
    topMotor.set(0.2);
    bottomMotor.set(0.2);
  }

  public void PIDMovePivot(double setpoint){
    pivotMotor.set(pivotController.calculate(getPivotAngle(), setpoint));
  }

  public void movePivotSlowUp(){
    pivotMotor.set(0.2);
  }

  public void movePivotSlowDown(){
    pivotMotor.set(-0.2);
  }

  public void movePivotZero(){
    pivotMotor.set(0);
  }


  private void config(){
    pivotMotor.clearFaults();
    pivotMotor.restoreFactoryDefaults();
    pivotMotor.burnFlash();
    pivotMotor.setSmartCurrentLimit((40));
    pivotMotor.setInverted(false);
    pivotMotor.setIdleMode(IdleMode.kBrake);

    topMotor.clearStickyFaults();
    topMotor.setNeutralMode(NeutralModeValue.Coast);
    topMotor.setInverted(false);
    topMotor.getConfigurator().apply(new TalonFXConfiguration().CurrentLimits.withSupplyCurrentLimit((25d)).withStatorCurrentLimit((20d)));

    bottomMotor.clearStickyFaults();
    bottomMotor.setNeutralMode(NeutralModeValue.Coast);
    bottomMotor.setInverted(false);
    bottomMotor.getConfigurator().apply(new TalonFXConfiguration().CurrentLimits.withSupplyCurrentLimit((25d)).withStatorCurrentLimit((20d)));
  }
  
  @Override
   public void periodic() {
    Logger.recordOutput("Shooter Angle Degrees", getPivotAngle());
   }
}
