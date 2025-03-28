package team1403.robot.swerve;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import team1403.robot.Constants;

public class Telemetry {
    private final double MaxSpeed;
    private final Field2d m_field = new Field2d();

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
        SignalLogger.start();

        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
            m_field.getObject("traj").setPoses(activePath);
        });
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

        SmartDashboard.putData("Field", m_field);
    }

    /* Mechanisms to represent the swerve module states */
    private final LoggedMechanism2d[] m_moduleMechanisms = new LoggedMechanism2d[] {
        new LoggedMechanism2d(1, 1),
        new LoggedMechanism2d(1, 1),
        new LoggedMechanism2d(1, 1),
        new LoggedMechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final LoggedMechanismLigament2d[] m_moduleSpeeds = new LoggedMechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new LoggedMechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new LoggedMechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new LoggedMechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new LoggedMechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final LoggedMechanismLigament2d[] m_moduleDirections = new LoggedMechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new LoggedMechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new LoggedMechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new LoggedMechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new LoggedMechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        /*
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Also write to log file
        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        for (int i = 0; i < 4; ++i) {
            m_moduleStatesArray[i*2 + 0] = state.ModuleStates[i].angle.getRadians();
            m_moduleStatesArray[i*2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
            m_moduleTargetsArray[i*2 + 0] = state.ModuleTargets[i].angle.getRadians();
            m_moduleTargetsArray[i*2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
        }*/

        //SignalLogger.writeDoubleArray("DriveState/Pose", m_poseArray);
        Logger.recordOutput("Swerve/RobotPose", state.Pose);
        //SignalLogger.writeDoubleArray("DriveState/ModuleStates", m_moduleStatesArray);
        Logger.recordOutput("Swerve/ModuleStates", state.ModuleStates);
        //SignalLogger.writeDoubleArray("DriveState/ModuleTargets", m_moduleTargetsArray);
        Logger.recordOutput("Swerve/ModuleTargets", state.ModuleTargets);
        //SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        Logger.recordOutput("Swerve/Speeds", state.Speeds);

        Logger.recordOutput("Swerve/Period", state.OdometryPeriod);

        m_field.setRobotPose(state.Pose);

        /* Telemeterize the module states to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));

            //SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
            if(Constants.DEBUG_MODE)
                Logger.recordOutput("Swerve/Module " + i, m_moduleMechanisms[i]);
        }
    }
}
