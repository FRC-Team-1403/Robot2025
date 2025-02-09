package team1403.robot.swerve.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants.Swerve;

public class SwerveModuleFXSFX extends SubsystemBase implements ISwerveModule {

    private final SwerveModule<TalonFX, TalonFXS, CANcoder> m_module;
    private final ModuleRequest m_modRequest;
    private final SwerveModuleState m_targetState;

    public SwerveModuleFXSFX(String name, int driveMotorPort, int steerMotorPort,
    int canCoderPort, double offset, int index, Translation2d modulePose, boolean inverted) {

        SwerveModuleConstantsFactory<TalonFXConfiguration, 
                                    TalonFXSConfiguration, 
                                    CANcoderConfiguration> factory = 
                                    new SwerveModuleConstantsFactory<>();

        factory.WheelRadius = Swerve.kWheelDiameterMeters / 2.0;
        factory.DriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        factory.SteerMotorType = SteerMotorArrangement.TalonFXS_NEO_JST;
        factory.SlipCurrent = Swerve.kDriveCurrentLimit;
        factory.SpeedAt12Volts = Swerve.kMaxSpeed;

        factory.DriveMotorGearRatio = 1.0/Swerve.kDriveReduction;
        factory.SteerMotorGearRatio = 1.0/Swerve.kSteerReduction;
        factory.CouplingGearRatio = 1.0/Swerve.kFirstDriveStage;

        factory.DriveMotorGains.kA = Swerve.kADrive;
        factory.DriveMotorGains.kV = Swerve.kVDrive;
        factory.DriveMotorGains.kS = Swerve.kSDrive;
        factory.DriveMotorGains.kP = Swerve.kPDrive;
        factory.DriveMotorGains.kI = Swerve.kIDrive;
        factory.DriveMotorGains.kD = Swerve.kDDrive;

        factory.SteerMotorGains.kS = Swerve.kSTurning;
        factory.SteerMotorGains.kP = Swerve.kPTurning;
        factory.SteerMotorGains.kI = Swerve.kITurning;
        factory.SteerMotorGains.kD = Swerve.kDTurning;

        m_targetState = new SwerveModuleState();
        m_modRequest = new ModuleRequest();
        m_module = new SwerveModule<TalonFX, TalonFXS, CANcoder>(TalonFX::new, TalonFXS::new, CANcoder::new, 
            factory.createModuleConstants(steerMotorPort, driveMotorPort, canCoderPort, offset, 
            modulePose.getX(), modulePose.getY(), inverted, false, false), 
            "rio", 0, index);
        
    }

    public SwerveModuleFXSFX(String name, int driveMotorPort, int steerMotorPort,
    int canCoderPort, double offset, Translation2d modulePose, int index) {
        this(name, driveMotorPort, steerMotorPort, canCoderPort, offset, index, modulePose, true);
    }


    @Override
    public SwerveModuleState getState() {
        return m_module.getCurrentState();
    }

    @Override
    public synchronized SwerveModulePosition getModulePosition() {
        return m_module.getPosition(true);
    }

    @Override
    public void set(DriveControlType type, double driveValue, SteerControlType s_type, double steerValue, DriveFeedforwards ff, int index) {

        if(type == DriveControlType.Velocity) m_modRequest.DriveRequest = DriveRequestType.Velocity;
        if(s_type == SteerControlType.Angle) { 
            m_modRequest.SteerRequest = SteerRequestType.Position; /* todo: try motion magic expo */ 
            m_targetState.angle = Rotation2d.fromRadians(steerValue);
        }

        //enable FOC cuz it's goated :)
        m_modRequest.EnableFOC = true;
        m_modRequest.WheelForceFeedforwardX = 0;
        m_modRequest.WheelForceFeedforwardY = 0;
        if(ff != null && index > 0) {
            m_modRequest.WheelForceFeedforwardX = ff.robotRelativeForcesXNewtons()[index];
            m_modRequest.WheelForceFeedforwardY = ff.robotRelativeForcesYNewtons()[index];
        }

        m_targetState.speedMetersPerSecond = driveValue;

        //apply the request because we are done
        if(type == DriveControlType.Velocity && s_type == SteerControlType.Angle)
            m_module.apply(m_modRequest);
        else if(type == DriveControlType.Velocity && s_type == SteerControlType.Angle)
            m_module.apply(new PositionVoltage(Units.radiansToRotations(steerValue)), new VoltageOut(driveValue)); //use torque current FOC if needed for both
        else throw new UnsupportedOperationException(); //todo: implement other combinations of flags
    }

    
    
}
