package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve.Swerve;


import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier lockOnTag;
    private BooleanSupplier windhamAim;
    private PIDController thetaController;
    private double rotationVal, translationVal, strafeVal;
    private BooleanSupplier lockOnSpeaker;
    private PIDController translationController;

    Timer shotTimer;
    Boolean ranOnce;

    Pose2d currentRobotPose;
    Translation2d currentRobotTranslation;
    double currentAngleToSpeaker;

    Pose2d futureRobotPose2d;
    Translation2d futureRobotTranslation;
    Rotation2d futureAngleToSpeaker;

    ChassisSpeeds speeds;
    Translation2d moveDelta;

    /**The calculated the time until the note leaves based on the constant and time since button press */
    Double timeUntilShot; 
    DoubleSupplier m_trigger; 

    Double correctedDistance;
    Rotation2d correctedRotation;

    public TeleopSwerve(Swerve s_Swerve, 
        DoubleSupplier translationSup, 
        DoubleSupplier strafeSup, 
        DoubleSupplier rotationSup, 
        BooleanSupplier robotCentricSup) {
        
        this.s_Swerve = s_Swerve;
        shotTimer = new Timer();
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        thetaController = new PIDController(0.015, 0.001, 0.0);
        thetaController.enableContinuousInput(-180, 180);
        translationController = new PIDController(0.05, 0, 0);
    }

    @Override
    public void execute() {
            translationVal = Math.pow(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband), 3);
            strafeVal = Math.pow(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband), 3);
            rotationVal = Math.pow(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband),3);

            shotTimer.stop();
            shotTimer.reset();

        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );

    }
}