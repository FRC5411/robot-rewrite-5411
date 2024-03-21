package frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.MutableMeasure.mutable;

import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.math.kinematics.SwerveModulePosition; looking into this, where are we getting "getModulePositions" from?
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public Alliance alliance;
    public double voltage;
    public SwerveDrivePoseEstimator swervePoseEstimator;

    public static final Vector<N3> odometryStd = VecBuilder.fill(0.06, 0.06, 0.01);
    public static final Vector<N3> visionStd = VecBuilder.fill(0.35, 0.35, 0.4);

    private final Field2d m_field = new Field2d();
    private final Field2d m_poseEstimatorField = new Field2d();

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "drivetrain/shooter");
        gyro.clearStickyFaults();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getAngle(), getModulePositions());
        if (!DriverStation.isAutonomousEnabled()) {
            swervePoseEstimator = new SwerveDrivePoseEstimator(
                    Constants.Swerve.swerveKinematics,
                    getAngle(),
                    getModulePositions(),
                    new Pose2d(),
                    odometryStd,
                    visionStd);

            // Drive base radius needs to be configured
            AutoBuilder.configureHolonomic(
                    this::getPose,
                    this::resetOdometry,
                    this::getRobotRelativeSpeeds,
                    this::driveRobotRelative,
                    new HolonomicPathFollowerConfig(
                            new PIDConstants(10),
                            new PIDConstants(10),
                            4.5,
                            0.406,
                            new ReplanningConfig()),
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this);

            // SmartDashboard.putData("Raw Odometry Field", m_field);
            SmartDashboard.putData("Pose Estimator Field", m_poseEstimatorField);
        }

    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(), translation.getY(), rotation, getAngle())
                        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates());
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates()),
                getAngle());
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getAngle(), getModulePositions(), pose);

    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getAngle() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getAngle())
                : Rotation2d.fromDegrees(gyro.getAngle());
    }

    public double getAngleAsDouble() {
        return 360 - gyro.getAngle();
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    protected Pose2d getPosition() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    public void driveForVoltage(double volts) {
        for (SwerveModule mod : mSwerveMods) {
            // if (mod.moduleNumber == 0 || mod.moduleNumber == 1) {
            // mod.setDriveVoltage(volts);
            // } else {
            mod.setDriveVoltage(volts);
            // }

        }
    }

    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    // return sysIdRoutine.quasistatic(direction);
    // }

    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    // return sysIdRoutine.dynamic(direction);
    // }

    public void xPosition(boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(1, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(1, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(1, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(1, Rotation2d.fromDegrees(45))
        };

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }

        System.out.println("Set to X Position");
    }

    /**
     * Gets a field-relative position for the shot to the speaker the robot should
     * take.
     * 
     * @return A {@link Translation2d} representing a field relative position in
     *         meters.
     */
    public Translation2d getSpeakerPosition() {
        boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                .equals(DriverStation.Alliance.Blue);
        Translation2d goalPose = isBlue ? FieldConstants.BLUE_SPEAKER : FieldConstants.RED_SPEAKER;
        ChassisSpeeds robotVel = getFieldRelativeSpeeds();
        double distanceToSpeaker = getPosition().getTranslation().getDistance(goalPose);

        double directionFlip = isBlue ? 1.0 : -1.0;
        double x = goalPose.getX()
                - (directionFlip * robotVel.vxMetersPerSecond * (distanceToSpeaker / FieldConstants.NOTE_VELOCITY));
        double y = goalPose.getY()
                - (directionFlip * robotVel.vyMetersPerSecond * (distanceToSpeaker / FieldConstants.NOTE_VELOCITY));
        return new Translation2d(x, y);
    }

    public double getRobotToSpeakerDistance() {
        boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                .equals(DriverStation.Alliance.Blue);
        Translation2d goalPose = isBlue ? FieldConstants.BLUE_SPEAKER : FieldConstants.RED_SPEAKER;
        ChassisSpeeds robotVel = getFieldRelativeSpeeds();
        double distanceToSpeaker = getPosition().getTranslation().getDistance(goalPose);

        return distanceToSpeaker;
    }

    // from 3467
    public Rotation2d getAngleToSpeaker(Translation2d pose) {
        return getSpeakerPosition().minus(pose).getAngle();
    }

    public double getDistToSpeaker(Translation2d robotPose) {
        return getSpeakerPosition().getDistance(robotPose);
    }

    /**
     * Gets the angle for the robot to face to score in the speaker, in radians.
     */
    public double getSpeakerAngle() {
        Translation2d speakerPosition = getSpeakerPosition();
        double speakerDistance = getSpeakerDistance();
        Translation2d robotPoint = getPosition().getTranslation();
        Rotation2d shotRot = speakerPosition.minus(robotPoint).getAngle();

        // Translation2d targetPosition = new Translation2d(
        // speakerPosition.getX() + shotRot.getSin() * speakerDistance,
        // speakerPosition.getY() + shotRot.getCos() * speakerDistance);

        // Pose3d speakerRotTarget = new Pose3d(targetPosition.getX(),
        // targetPosition.getY(), DriveConstants.SPEAKER_HEIGHT, new Rotation3d());
        // SmartDashboard.putNumber("Angle To Speaker",
        // targetPosition.minus(robotPoint).getAngle().getRadians());
        // SmartDashboard.putNumber("Target Position X", targetPosition.getX());
        // SmartDashboard.putNumber("Target Position Y", targetPosition.getY());

        double angleToSpeaker = MathUtil.angleModulus(speakerPosition.minus(robotPoint).getAngle().getRadians());
        SmartDashboard.putNumber("Angle To Speaker", angleToSpeaker);

        return MathUtil.angleModulus(angleToSpeaker);
    }

    public double getSpeakerDistance() {
        return getPosition().getTranslation().getDistance(getSpeakerPosition());
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getAngle(), getModulePositions());
        m_field.setRobotPose(swerveOdometry.getPoseMeters());
        m_poseEstimatorField.setRobotPose(swervePoseEstimator.getEstimatedPosition());

        // Optional<EstimatedRobotPose> pose = photonPoseEstimator.update();
        // if(pose.isPresent()) {
        // swervePoseEstimator.addVisionMeasurement(pose.get().estimatedPose.toPose2d(),
        // pose.get().timestampSeconds);
        // }

        swervePoseEstimator.update(getAngle(), getModulePositions());

        // SmartDashboard.putData("Raw Odometry Field", m_field);
        SmartDashboard.putData("Pose Estimator Field", m_poseEstimatorField);
        // SmartDashboard.putBoolean("Pose Present", pose.isPresent());

        if (Constants.swerveTuningMode) {
            for (SwerveModule mod : mSwerveMods) {
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " absoluteEncoderPorts",
                        mod.getAbsoluteEncoder().getDegrees());
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());

                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Position", mod.getPosition().distanceMeters);
                SmartDashboard.putNumber("test velocity" + mod.moduleNumber, mod.getDriveVelocity());
                // SmartDashboard.putNumber("Mod" + mod.moduleNumber + " get encoder 1",
                // mod.getDriveEncoderPositon());

            }

        }

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
        SmartDashboard.putNumber("Gyro Angle", getAngle().getDegrees());
    }

}

// fix getModulePositions => exists within 6328, commented out function above,
// why no worky?