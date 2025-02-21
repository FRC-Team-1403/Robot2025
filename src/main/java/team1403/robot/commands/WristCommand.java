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
    private BooleanSupplier m_low;
    private BooleanSupplier m_mid;
    private BooleanSupplier m_high;
    private WristSubsystem m_wrist;
    
    public WristCommand(WristSubsystem wrist, BooleanSupplier zero, BooleanSupplier low, BooleanSupplier mid, BooleanSupplier high) {
        m_wrist = wrist;
        m_zero = zero;
        m_low = low;
        m_mid = mid;
        m_high = high;

        addRequirements(m_wrist);
    }

    public void initialize() {
        m_wrist.setWristAngle(0.23);
    }
    
    @Override
    public void execute() {
        if(m_zero.getAsBoolean()) {
            m_wrist.setWristAngle(.23);
        }
        else if(m_low.getAsBoolean()) {
            m_wrist.setWristAngle(Constants.Wrist.Setpoints.L2Setpoint / 360);
        }
        else if(m_mid.getAsBoolean()) {
            m_wrist.setWristAngle(Constants.Wrist.Setpoints.L3Setpoint / 360);
        }
        else if (m_high.getAsBoolean()) {
            m_wrist.setWristAngle(Constants.Wrist.Setpoints.L4Setpoint / 360);
        }
    }
}
