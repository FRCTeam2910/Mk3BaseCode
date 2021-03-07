package org.frcteam2910.mk3.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.google.errorprone.annotations.concurrent.GuardedBy;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.mk3.Constants;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.kinematics.ChassisVelocity;
import org.frcteam2910.common.kinematics.SwerveKinematics;
import org.frcteam2910.common.kinematics.SwerveOdometry;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.UpdateManager;
import org.frcteam2910.common.robot.drivers.Mk3SwerveModule;
import org.frcteam2910.common.robot.drivers.Pigeon;
import org.frcteam2910.common.util.HolonomicDriveSignal;


public class DrivetrainSubsystem implements Subsystem, UpdateManager.Updatable {
    public static final double TRACKWIDTH = 1.0;
    public static final double WHEELBASE = 1.0;
    public static final double STEER_GEAR_RATIO = 12.8;
    public static final double DRIVE_GEAR_RATIO = 6.86;

    private final Mk3SwerveModule[] modules;

    private final SwerveKinematics swerveKinematics = new SwerveKinematics(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0),         // Front left
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),        // Front right
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),        // Back left
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)        // Back right
    );

    private final Object sensorLock = new Object();
    @GuardedBy("sensorLock")
    private Gyroscope gyroscope = new Pigeon(Constants.PIGEON_PORT);

    private final Object kinematicsLock = new Object();
    @GuardedBy("kinematicsLock")
    private final SwerveOdometry swerveOdometry = new SwerveOdometry(swerveKinematics, RigidTransform2.ZERO);
    @GuardedBy("kinematicsLock")
    private RigidTransform2 pose = RigidTransform2.ZERO;

    private final Object stateLock = new Object();
    @GuardedBy("stateLock")
    private HolonomicDriveSignal driveSignal = null;

    // Logging
    private final NetworkTableEntry odometryXEntry;
    private final NetworkTableEntry odometryYEntry;
    private final NetworkTableEntry odometryAngleEntry;

    private final NetworkTableEntry[] moduleAngleEntries;

    public DrivetrainSubsystem() {
        synchronized (sensorLock) {
            gyroscope.setInverted(false);
        }

        TalonFX frontLeftSteeringMotor = new TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR);
        TalonFX backLeftSteeringMotor = new TalonFX(Constants.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR);

        TalonFX frontLeftDriveMotor = new TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR);
        TalonFX frontRightDriveMotor = new TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR);
        TalonFX backLeftDriveMotor = new TalonFX(Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR);
        TalonFX backRightDriveMotor = new TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR);
        frontRightDriveMotor.setInverted(true);
        backRightDriveMotor.setInverted(true);

        // Limit speed (testing only)
        configTalon(frontLeftDriveMotor);
        configTalon(frontRightDriveMotor);
        configTalon(backLeftDriveMotor);
        configTalon(backRightDriveMotor);

        Mk3SwerveModule frontLeftModule = new Mk3SwerveModule(new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET,
                STEER_GEAR_RATIO,
                DRIVE_GEAR_RATIO,
                frontLeftSteeringMotor,
                frontLeftDriveMotor,
                new CANCoder(Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_PORT));

        Mk3SwerveModule frontRightModule = new Mk3SwerveModule(new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET,
                STEER_GEAR_RATIO,
                DRIVE_GEAR_RATIO,
                new TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR),
                frontRightDriveMotor,
                new CANCoder(Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT));

        Mk3SwerveModule backLeftModule = new Mk3SwerveModule(new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
                Constants.DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET,
                STEER_GEAR_RATIO,
                DRIVE_GEAR_RATIO,
                backLeftSteeringMotor,
                backLeftDriveMotor,
                new CANCoder(Constants.DRIVETRAIN_BACK_LEFT_ENCODER_PORT));

        Mk3SwerveModule backRightModule = new Mk3SwerveModule(new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
                Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET,
                STEER_GEAR_RATIO,
                DRIVE_GEAR_RATIO,
                new TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR),
                backRightDriveMotor,
                new CANCoder(Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_PORT));

        modules = new Mk3SwerveModule[] {frontLeftModule, frontRightModule, backLeftModule, backRightModule};

        moduleAngleEntries = new NetworkTableEntry[modules.length];

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        odometryXEntry = tab.add("X", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
        odometryYEntry = tab.add("Y", 0.0)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();
        odometryAngleEntry = tab.add("Angle", 0.0)
                .withPosition(0, 2)
                .withSize(1, 1)
                .getEntry();

        ShuffleboardLayout[] moduleLayouts = {
                tab.getLayout("Front Left Module", BuiltInLayouts.kList),
                tab.getLayout("Front Right Module", BuiltInLayouts.kList),
                tab.getLayout("Back Left Module", BuiltInLayouts.kList),
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
        };
        for (int i = 0; i < modules.length; i++) {
            ShuffleboardLayout layout = moduleLayouts[i]
                    .withPosition(2 + i * 2, 0)
                    .withSize(2, 4);
            moduleAngleEntries[i] = layout.add("Angle", 0.0).getEntry();
        }
        tab.addNumber("Rotation Voltage", () -> {
            HolonomicDriveSignal signal;
            synchronized (stateLock) {
                signal = driveSignal;
            }

            if (signal == null) {
                return 0.0;
            }

            return signal.getRotation() * RobotController.getBatteryVoltage();
        });
    }

    public RigidTransform2 getPose() {
        synchronized (kinematicsLock) {
            return pose;
        }
    }

    public void drive(Vector2 translationalVelocity, double rotationalVelocity, boolean isFieldOriented) {
        synchronized (stateLock) {
            driveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, isFieldOriented);
        }
    }

    public void resetPose(RigidTransform2 pose) {
        synchronized (kinematicsLock) {
            this.pose = pose;
            swerveOdometry.resetPose(pose);
        }
    }

    public void resetGyroAngle(Rotation2 angle) {
        synchronized (sensorLock) {
            gyroscope.setAdjustmentAngle(
                    gyroscope.getUnadjustedAngle().rotateBy(angle.inverse())
            );
        }
    }

    public void resetWheelAngles() {
        for (Mk3SwerveModule module : modules) {
            module.resetAngleOffsetWithAbsoluteEncoder();
        }
    }

    private void configTalon(TalonFX talon) {
        talon.configPeakOutputForward(0.3, 30);
        talon.configPeakOutputReverse(-0.3, 30);
    }

    private void updateOdometry(double dt) {
        Vector2[] moduleVelocities = new Vector2[modules.length];
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.updateSensors();

            moduleVelocities[i] = Vector2.fromAngle(Rotation2.fromRadians(module.getCurrentAngle())).scale(module.getCurrentVelocity());
        }

        Rotation2 angle;
        synchronized (sensorLock) {
            angle = gyroscope.getAngle();
        }

        synchronized (kinematicsLock) {
            this.pose = swerveOdometry.update(angle, dt, moduleVelocities);
        }
    }

    private void updateModules(HolonomicDriveSignal driveSignal, double dt) {
        ChassisVelocity chassisVelocity;
        if (driveSignal == null) {
            chassisVelocity = new ChassisVelocity(Vector2.ZERO, 0.0);
        } else if (driveSignal.isFieldOriented()) {
            chassisVelocity = new ChassisVelocity(
                    driveSignal.getTranslation().rotateBy(getPose().rotation.inverse()),
                    driveSignal.getRotation()
            );
        } else {
            chassisVelocity = new ChassisVelocity(
                    driveSignal.getTranslation(),
                    driveSignal.getRotation()
            );
        }

        Vector2[] moduleOutputs = swerveKinematics.toModuleVelocities(chassisVelocity);
        SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1);
        for (int i = 0; i < moduleOutputs.length; i++) {
            var module = modules[i];
            module.setTargetVelocity(moduleOutputs[i]);
            module.updateState(dt);
        }
    }

    @Override
    public void update(double time, double dt) {
        updateOdometry(dt);

        HolonomicDriveSignal driveSignal;
        synchronized (stateLock) {
            driveSignal = this.driveSignal;
        }

        updateModules(driveSignal, dt);
    }

    @Override
    public void periodic() {
        RigidTransform2 pose = getPose();
        odometryXEntry.setDouble(pose.translation.x);
        odometryYEntry.setDouble(pose.translation.y);
        odometryAngleEntry.setDouble(getPose().rotation.toDegrees());

        for (int i = 0; i < modules.length; i++) {
            moduleAngleEntries[i].setDouble(Math.toDegrees(modules[i].getCurrentAngle()));
        }
    }
}
