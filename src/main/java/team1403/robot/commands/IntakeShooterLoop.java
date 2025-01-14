
package team1403.robot.commands;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.ArmWrist;
import team1403.robot.subsystems.Blackbox;
import team1403.robot.subsystems.IntakeAndShooter;

public class IntakeShooterLoop extends Command {
    private IntakeAndShooter m_intakeAndShooter;
    private ArmWrist m_armwrist;
    private BooleanSupplier m_trigger;
    private BooleanSupplier m_amp;
    private BooleanSupplier m_reset;

    private boolean amp;
    private boolean speaker;
    private boolean isShooting;

    public IntakeShooterLoop(IntakeAndShooter intakeAndShooter, ArmWrist armwrist,
     BooleanSupplier trigger, BooleanSupplier amp, BooleanSupplier reset) {
        m_intakeAndShooter = intakeAndShooter;
        m_armwrist = armwrist;
        m_trigger = trigger;
        m_amp = amp;
        m_reset = reset;

        addRequirements(m_armwrist, m_intakeAndShooter);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        Logger.recordOutput("Is Shooter Ready", isShooting);
        // set isShooting to false when the note is not in the robot
        if (!m_intakeAndShooter.isIntakePhotogateTriggered() && !m_intakeAndShooter.isShooterPhotogateTriggered()) {
            isShooting = false;
        }
        // when the note is not in the robot: set arm and wrist to intake setpoint, start intake, stop shooter
        if (!m_intakeAndShooter.isLoaded() && !isShooting){
            amp = false; 
            speaker = false;
            m_armwrist.setWristSetpoint(Constants.Wrist.kIntakeSetpoint);
            if (m_armwrist.isWristAtSetpoint()) {
                m_armwrist.setArmSetpoint(Constants.Arm.kIntakeSetpoint); 
            }
            if (m_armwrist.isArmAndWristAtSetpoint()) {
                m_intakeAndShooter.setIntakeSpeed(0.2);
            }
            if (!m_armwrist.isArmAndWristAtSetpoint()) {
                m_intakeAndShooter.intakeStop();
            }
            m_intakeAndShooter.shooterStop();
        }
        // roll back the note
        if ((m_intakeAndShooter.intakeSpeed() > 0) && m_intakeAndShooter.isShooterPhotogateTriggered() && !isShooting) {
            m_intakeAndShooter.setIntakeSpeed(-0.1);
        }
        // once the note is rolled back stop intake and set arm and wrist and start spinning shooter motors
        if ((m_intakeAndShooter.intakeSpeed() < 0) && m_intakeAndShooter.isLoaded() && !isShooting) {
            m_intakeAndShooter.intakeStop();
            m_armwrist.setArmSetpoint(Constants.Arm.kDriveSetpoint);
            m_armwrist.setWristSetpoint(Constants.Wrist.kDriveSetpoint);
            isShooting = true;
            speaker = true;
        }
        // if shooting for speaker have rpm at 2000
        if (isShooting && speaker && m_armwrist.isArmAndWristAtSetpoint()) {
            m_intakeAndShooter.setShooterRPM(2000);
        }
        // if shooting for amp have rpm at 1000
        if (isShooting && amp) {
            m_intakeAndShooter.setShooterRPM(1000);
        }
        // shoot if trigger is hit and the top and bottom shooter motors are close to the target rpm
        if (m_trigger.getAsBoolean() && m_armwrist.isArmAndWristAtSetpoint() && m_intakeAndShooter.isReady()) {
            m_intakeAndShooter.setIntakeSpeed(0.5);
        }
        // if amp button is hit set arm and wrist to amp mode
        if (m_amp.getAsBoolean()) {
            m_intakeAndShooter.intakeStop();
            //m_intakeAndShooter.shooterStop();
            m_armwrist.setArmSetpoint(Constants.Arm.kAmpSetpoint);
            m_armwrist.setWristSetpoint(Constants.Wrist.kAmpSetpoint);
            amp = true;
        }

        // if reset button is hit set arm and wrist to intake mode (not loaded)
        if (m_reset.getAsBoolean()) {
            speaker = false;
            amp = false;
            m_intakeAndShooter.intakeStop();
            m_intakeAndShooter.shooterStop();

            if(m_intakeAndShooter.isLoaded()) {
                m_armwrist.setWristSetpoint(Constants.Wrist.kDriveSetpoint);
                m_armwrist.setArmSetpoint(Constants.Arm.kDriveSetpoint);
                speaker = true;
            }
            else {
                m_armwrist.setWristSetpoint(Constants.Wrist.kIntakeSetpoint);
                m_armwrist.setArmSetpoint(Constants.Arm.kIntakeSetpoint);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}