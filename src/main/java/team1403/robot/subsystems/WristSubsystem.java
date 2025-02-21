// package team1403.robot.subsystems;

// <<<<<<< HEAD
// import javax.imageio.plugins.tiff.TIFFDirectory;

// =======
// import com.revrobotics.spark.SparkAbsoluteEncoder;
// import com.revrobotics.spark.SparkBase;
// >>>>>>> ed9f11c2a49f08d6947bc53b9e5d8b9820b0319d
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;

// import static edu.wpi.first.units.Units.Volts;

// import org.littletonrobotics.junction.AutoLogOutput;
// import org.littletonrobotics.junction.Logger;

// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.AbsoluteEncoderConfig;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// <<<<<<< HEAD
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// =======
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// >>>>>>> ed9f11c2a49f08d6947bc53b9e5d8b9820b0319d
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import team1403.robot.Constants;
// import team1403.robot.Constants.Wrist;

// public class WristSubsystem extends SubsystemBase {
//     private SparkMax m_wristMotor;
//     private SparkAbsoluteEncoder m_encoder;                    
//     private ArmFeedforward m_feedForward;
//     private PIDController m_wristPID;
// <<<<<<< HEAD
//     private TrapezoidProfile m_trapezoidPID;

//     public WristSubsystem() {
//         m_wristMotor = new SparkMax(Constants.CanBus.wristMotorID, MotorType.kBrushless);
//         m_feedForward = new ArmFeedforward(Constants.Wrist.WristKS, Constants.Wrist.WristKG, Constants.Wrist.WristKV);
//         m_wristPID = new PIDController(Constants.Wrist.WristKP, Constants.Wrist.WristKI, Constants.Wrist.WristKD);
//         m_trapezoidPID = new TrapezoidProfile(new TrapezoidProfile.Constraints(Constants.Wrist.trapezoidVel*5,Constants.Wrist.trapezoidAcc*5));

        
// =======
//     private ProfiledPIDController m_profiled;
//     private SysIdRoutine m_SysIDRoutine;

//     public WristSubsystem() {
//         m_wristMotor = new SparkMax(Constants.CanBus.wristMotorID, MotorType.kBrushless);

//         SparkMaxConfig config = new SparkMaxConfig();
//         //AbsoluteEncoderConfig config2 = new AbsoluteEncoderConfig();
//         //config2.zeroOffset(Constants.Wrist.WristEncoderOffset);
//         //config.absoluteEncoder.apply(config2);

//         m_wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//         m_encoder = m_wristMotor.getAbsoluteEncoder();
//         // m_feedForward = new ArmFeedforward(Constants.Wrist.WristKS, Constants.Wrist.WristKG, Constants.Wrist.WristKV);
//         m_wristPID = new PIDController(Constants.Wrist.WristKP, Constants.Wrist.WristKI, Constants.Wrist.WristKD);
//         m_profiled = new ProfiledPIDController(Wrist.WristKP, Constants.Wrist.WristKI, Constants.Wrist.WristKD, new TrapezoidProfile.Constraints(Constants.Wrist.maxVelo, Constants.Wrist.maxAccel));
//         m_feedForward = new ArmFeedforward(Constants.Wrist.WristKS, Constants.Wrist.WristKG, 0);

//         SmartDashboard.putData("wrist 0idp", m_wristPID);
//         SmartDashboard.putData("trapezdoi ipd", m_profiled);

//         m_SysIDRoutine = new SysIdRoutine(new SysIdRoutine.Config(null, null, null, 
//             (state) -> Logger.recordOutput("SYSID Wrist", state.toString())),
//             new SysIdRoutine.Mechanism((voltage) -> {m_wristMotor.setVoltage(voltage.in(Volts));
//             }, (log)  -> {
//             }, this));
//     }

//     //in rotations!!
//     @AutoLogOutput
//     public double getWristAngle(){
//         return MathUtil.inputModulus(m_encoder.getPosition() + Constants.Wrist.WristEncoderOffset, -0.5, 0.5);
//     }

//     @AutoLogOutput
//     public double getWristAngleDeg() {
//         return 360 * getWristAngle();
//     }

//     @AutoLogOutput
//     public double getWristVelocity() {
//         return m_encoder.getVelocity() * 60;
//     }

//     public void setWristAngle(double targetAngle) {
//         m_profiled.setGoal(targetAngle);
//     }

//     public void stop() {
//     }

//     public Command getSysIDQ(SysIdRoutine.Direction d){
//         return m_SysIDRoutine.quasistatic(d);
//     }

//     public Command getSysIDD(SysIdRoutine.Direction d){
//         return m_SysIDRoutine.dynamic(d);
//     }

//     public void periodic() {
//         getWristAngle();
//         getWristVelocity();
//         getWristAngleDeg();
//         m_wristMotor.set(m_profiled.calculate(getWristAngle()) + 
//             -m_feedForward.calculate(
//                 Units.rotationsToRadians(getWristAngle()), 
//                 Units.rotationsToRadians(getWristVelocity())));
// >>>>>>> ed9f11c2a49f08d6947bc53b9e5d8b9820b0319d
//     }
// }
