package team1403.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team1403.robot.subsystems.ArmWrist;
import team1403.robot.Constants;

public class ArmWristCommand extends Command {

    private ArmWrist m_armwrist;
    private BooleanSupplier m_up;
    private BooleanSupplier m_down; 

    public ArmWristCommand(ArmWrist armwrist, BooleanSupplier up, BooleanSupplier down) {
        m_armwrist = armwrist;
        m_up = up;
        m_down = down;

        addRequirements(m_armwrist);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (m_up.getAsBoolean()) {
            m_armwrist.setWristSetpoint(147);
            if (m_armwrist.isWristAtSetpoint()) {
                m_armwrist.setArmSetpoint(Constants.Arm.kIntakeSetpoint);
            }  
        }
        if (m_down.getAsBoolean()) {
            m_armwrist.setWristSetpoint(120);
            if (m_armwrist.isWristAtSetpoint()) {
                m_armwrist.setArmSetpoint(Constants.Arm.kDriveSetpoint);
            }
        }
        Logger.recordOutput("working", true);
        Logger.recordOutput("up button", m_up.getAsBoolean());
        Logger.recordOutput("down button", m_down.getAsBoolean());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
