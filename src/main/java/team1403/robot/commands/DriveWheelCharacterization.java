package team1403.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.swerve.TunerConstants;

public class DriveWheelCharacterization extends Command {

    private final SwerveSubsystem m_swerve;
    private Rotation2d m_prevRot;
    private double angleIGyro = 0;
    private double[] angleIWheel = new double[4];
    private final Timer m_timer = new Timer();
    //may need to change on a non square swerve
    private static final double kDriveBaseRadius = TunerConstants.kModulePositions[0].getNorm();

    public DriveWheelCharacterization(SwerveSubsystem swerve) {
        m_swerve = swerve;
        m_prevRot = Rotation2d.kZero;

        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        m_prevRot = m_swerve.getPigeon2().getRotation2d();
        SwerveModulePosition[] p = m_swerve.getState().ModulePositions;
        for(int i = 0; i < p.length; i++) {
            angleIWheel[i] = p[i].distanceMeters;
        }
        m_timer.restart();
    }

    @Override
    public void execute() {
        Rotation2d cur = m_swerve.getPigeon2().getRotation2d();
        angleIGyro += cur.minus(m_prevRot).getRadians();
        m_prevRot = cur;

        m_swerve.drive(
            new ChassisSpeeds(
                0, 0,
                MathUtil.clamp(Math.PI / 8.0 * m_timer.get(), 0, Math.PI / 3.0)
            ));
    }

    @Override
    public void end(boolean interrupt) {
        SwerveModulePosition[] p = m_swerve.getState().ModulePositions;
        for(int i = 0; i < p.length; i++) {
            angleIWheel[i] = (p[i].distanceMeters - angleIWheel[i]) / TunerConstants.kWheelRadius.in(Meters); //radian conversion
            angleIWheel[i] = Math.abs(angleIGyro * kDriveBaseRadius / angleIWheel[i]); //store radius (c/theta = r)
            System.out.println("Wheel " + i + " Radius: " + angleIWheel[i] + " m");
            System.out.println("Wheel " + i + " Radius: " + Units.metersToInches(angleIWheel[i]) + " in");
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
