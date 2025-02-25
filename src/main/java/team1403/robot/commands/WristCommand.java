package team1403.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.google.errorprone.annotations.OverridingMethodsMustInvokeSuper;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.WristSubsystem;

public class WristCommand extends Command {
    private BooleanSupplier m_zero;
    private WristSubsystem m_wrist;
    private double m_target;
    
    public WristCommand(WristSubsystem wrist, double target) {
        m_wrist = wrist;
        m_target = target;

        addRequirements(m_wrist);
    }

    public void initialize() {
        m_wrist.setWristAngle(m_target / 360.);
    }
    
    @Override
    public void execute() {
        Constants.Wrist.Setpoints.current = m_target;
        m_wrist.setWristAngle(m_target / 360.);
    }

    @Override
    public boolean isFinished() {
        return m_wrist.isAtSetpoint();
    }
}
