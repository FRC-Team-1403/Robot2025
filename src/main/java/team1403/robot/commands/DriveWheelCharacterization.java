package team1403.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.io.Console;
import java.util.Arrays;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.swerve.TunerConstants;

public class DriveWheelCharacterization extends Command {

    private final SwerveSubsystem m_swerve;
    private final StatusSignal<AngularVelocity> m_velo;
    private double angleIGyro = 0;
    private double[] angleIWheel = new double[4];

    public DriveWheelCharacterization(SwerveSubsystem swerve) {
        m_swerve = swerve;
        m_velo = swerve.getPigeon2().getAngularVelocityZWorld();

        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        angleIGyro = 0;
        SwerveModulePosition[] p = m_swerve.getState().ModulePositions;
        for(int i = 0; i < p.length; i++) {
            angleIWheel[i] = p[i].distanceMeters;
        }
    }

    @Override
    public void execute() {
        angleIGyro += m_velo.getValue().in(RadiansPerSecond) * Constants.kLoopTime;

        m_swerve.drive(
            new ChassisSpeeds(
                0, 0, Math.PI / 4.0
            ));
    }

    @Override
    public void end(boolean interrupt) {
        SwerveModulePosition[] p = m_swerve.getState().ModulePositions;
        for(int i = 0; i < p.length; i++) {
            angleIWheel[i] = p[i].distanceMeters - angleIWheel[i];
            angleIWheel[i] /= (TunerConstants.kWheelRadius.in(Meters)); // d = \Delta t \times r : \Delta t = \frac{d}{r}
            angleIWheel[i] = Math.abs(angleIGyro / angleIWheel[i]) / 2; //store radius
            System.out.println("Wheel Radius: " + angleIWheel[i] + " m");
        }

        

    }


    @Override
    public boolean isFinished() {
        return false;
    }
    
}
