// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
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
  
  public Shooter() {
    pivotMotor = new CANSparkMax(ShooterConstants.PIVOT_PORT, MotorType.kBrushless);
    topMotor = new TalonFX(ShooterConstants.UPPER_ROLLER_PORT, "drivetrain");
    bottomMotor = new TalonFX(ShooterConstants.LOWER_ROLLER_PORT, "drivetrain");
    pivotEncoder = new DutyCycleEncoder(ShooterConstants.PIVOT_ENCODER_PORT);

    pivotGearRatio =  1.0 / 144.0;
    config();
  }

  public double getPivotAngle(){
    return (-pivotEncoder.getAbsolutePosition() * 360) + ShooterConstants.PIVOT_OFFSET;
  }

  public double getPivotAngleRadians(){
    return (getPivotAngle() * Math.PI / 180);
  }

  public double getPivotVelocity(){
    return pivotMotor.getEncoder().getVelocity();
  }

  public void shootWing(){
    topMotor.set(0.7);
    bottomMotor.set(0.7);
  }


  public void shootSubwoofer(){
    topMotor.set(0.6);
    bottomMotor.set(0.6);
  }

  public void shootFeed(){
    topMotor.set(0.45);
    bottomMotor.set(0.45);
  }

  public void shootLaser(){
    topMotor.set(0.80);
    bottomMotor.set(0.80);  
  }

  public void shootLob(){
    topMotor.set(0.30);
    bottomMotor.set(0.30);  
  }

  public void shootAmp(){
    topMotor.set(0);
    bottomMotor.set(0.21);
  }
  
  public void motorZero(){
    topMotor.set(0);
    bottomMotor.set(0);
    pivotMotor.set(0);
  }

  public void reverse(){
    topMotor.set(0.2);
    bottomMotor.set(0.2);
  }

  public double getShooterVelocity(){
    return topMotor.getVelocity().getValueAsDouble();
  }

  public void setShooterVelocity(double demand){
    topMotor.setControl(new VelocityDutyCycle((demand / (60d))));
    bottomMotor.setControl(new VelocityDutyCycle((demand / (60d))));
  }


  public Command shooterPID(double setpoint) {
    ProfiledPIDController pivotController;
    ArmFeedforward pivotFeedforward;

    pivotController = new ProfiledPIDController(
      ShooterConstants.PIVOT_P,
      ShooterConstants.PIVOT_I,
      ShooterConstants.PIVOT_D, 
      new TrapezoidProfile.Constraints(600, 600));

    pivotFeedforward = new ArmFeedforward(0, 0.03, 0);

    return new FunctionalCommand(
      () -> {
        pivotController.reset(getPivotAngle());
      }, 
      () -> {
        pivotMotor.set(-pivotController.calculate(getPivotAngle(), setpoint));
        if(pivotController.atGoal()){
          pivotMotor.set(-pivotFeedforward.calculate(getPivotAngleRadians(), getPivotVelocity()));
        }
      }, 
      (interrupted) -> {

      }, 
      () -> pivotController.atGoal(), 
      this
      );
  }

//TODO: CHECK U SLAVE
  public Command shooterRPM(double setpoint){
    ProfiledPIDController RPMController;
    RPMController = new ProfiledPIDController(
      ShooterConstants.PIVOT_P,
      ShooterConstants.PIVOT_I,
      ShooterConstants.PIVOT_D, 
      new TrapezoidProfile.Constraints(900, 900));

    return new FunctionalCommand(
      () -> {
        RPMController.reset(getShooterVelocity());
      }, 
      () -> {
        setShooterVelocity((-RPMController.calculate(getPivotAngle(), setpoint)));
      }, 
      (interrupted) -> {

      }, 
      () -> RPMController.atGoal(), 
      this
      );
  } 

  public Command shooterIntake(){
    return shooterPID(ShooterConstants.PIVOT_INTAKE_ANGLE);
  }

  public Command shooterAmp(){
    return new ParallelCommandGroup(
      shooterPID(ShooterConstants.PIVOT_AMP_ANGLE),
      shootRPMAmp()
    );
  }

  public Command shooterWing(){
    return new ParallelCommandGroup(
      shooterPID(ShooterConstants.PIVOT_WING_ANGLE),
      shootRPMWing()
    );
  }

  public Command shooterIdle(){
    return new ParallelCommandGroup(
      shooterPID(ShooterConstants.PIVOT_IDLE_ANGLE),
      zeroShoot()
    );
  }

  public Command shooterSubwoofer(){
    return new ParallelCommandGroup(
      shooterPID(ShooterConstants.PIVOT_SUBWOOFER_ANGLE),
      shootRPMSubwoofer()
    );
  }

  public Command shooterFeed(){
    return new ParallelCommandGroup(
      shooterPID(ShooterConstants.PIVOT_SUBWOOFER_ANGLE),
      shootRPMFeed()
    );
  }

  public Command shooterLaser(){
    return new ParallelCommandGroup(
      shooterPID(ShooterConstants.PIVOT_IDLE_ANGLE),
      shootRPMLaser()
    );
  }

  public Command shooterLob(){
    return new ParallelCommandGroup(
      shooterPID(ShooterConstants.PIVOT_SUBWOOFER_ANGLE),
      shootRPMLob()
    );
  }

  public Command shooterPodium(){
    return new ParallelCommandGroup(
      shooterPID(ShooterConstants.PIVOT_PODIUM_ANGLE),
      shootRPMSubwoofer()
    );
  }

  public Command shootRPMAmp(){
    return new InstantCommand(() -> shootAmp());
  }

  public Command shootRPMFeed(){
    return new InstantCommand(() -> shootFeed());
  }

  public Command shootRPMSubwoofer(){
    return new InstantCommand(() -> shootSubwoofer());
  }

  public Command shootRPMLaser(){
    return new InstantCommand(() -> shootLaser());
  }

  public Command shootRPMWing(){
    return new InstantCommand(() -> shootWing());
  }

  public Command shootRPMLob(){
    return new InstantCommand(() -> shootLob());
  }

  public Command reverseShoot(){
    return new InstantCommand(() -> reverse());
  }

  public Command zeroShoot(){
    return new InstantCommand(() -> motorZero());
  }

  public Command movePivotSlowUp(){
    return new InstantCommand(() -> pivotMotor.set(0.3));
  }

  public Command movePivotSlowDown(){
    return new InstantCommand(() -> pivotMotor.set(-0.3));
  }

  public Command movePivotZero(){
    return new InstantCommand(() -> pivotMotor.set(0));
  }

  public Command setShooterRPM(double demand){
    return new InstantCommand(() -> setShooterVelocity(demand));
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
    SmartDashboard.putNumber("Shooter Angle Degrees", getPivotAngle());
    SmartDashboard.putNumber("Target Setpoint man", ShooterConstants.PIVOT_SUBWOOFER_ANGLE);
    SmartDashboard.putNumber("Shooter RPM", getShooterVelocity());
    SmartDashboard.putNumber("Shooter RPM Setpoint", 20);
   }
}
