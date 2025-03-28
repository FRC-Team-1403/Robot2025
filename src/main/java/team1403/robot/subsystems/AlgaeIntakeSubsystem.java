package team1403.robot.subsystems;

import java.nio.file.DirectoryStream.Filter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team1403.robot.Constants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
    private SparkMax m_algaeIntakeMotor;
    private DigitalInput m_algaeIntakePhotogate;
    private Debouncer m_debounce;

    public AlgaeIntakeSubsystem() {
        m_algaeIntakeMotor = new SparkMax(Constants.CanBus.algaeIntakeMotorID, MotorType.kBrushed);
        m_algaeIntakePhotogate = new DigitalInput(Constants.RioPorts.kAlgaeIntakePhotogateID);
        m_debounce = new Debouncer(Constants.AlgaeIntake.debounceTime, DebounceType.kBoth);
        configMotors();
    }

    private void configMotors() {
        SparkMaxConfig intakeconfig = new SparkMaxConfig();
        intakeconfig
            .idleMode(IdleMode.kBrake);
        m_algaeIntakeMotor.configure(intakeconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        SparkMaxConfig wristconfig = new SparkMaxConfig();
        wristconfig
            .idleMode(IdleMode.kBrake);
        m_algaeIntakeMotor.configure(wristconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void stop() {
        m_algaeIntakeMotor.set(0);
    }

    public void setIntakeSpeed(double speed) {
        m_algaeIntakeMotor.set(speed);
    }

    @AutoLogOutput (key = "AlgaeIntake/Intake Speed")
    public double getIntakeSpeed() {
        return m_algaeIntakeMotor.get();
    }

    @AutoLogOutput (key = "AlgaeIntake/photogate triggered")
    private boolean photoGateTriggered() {
        return !m_algaeIntakePhotogate.get();
    }

    @AutoLogOutput (key = "AlgaeIntake/debounced photogate triggered")
    public boolean algaeIntaked() {
        return m_debounce.calculate(photoGateTriggered());
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Algae Intake Motor Temp", m_algaeIntakeMotor.getMotorTemperature());
    }
}