// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.robot;

import java.util.Set;

import org.ejml.dense.row.MatrixFeatures_CDRM;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import team1403.lib.util.AutoUtil;
import team1403.lib.util.CougarUtil;
import team1403.robot.commands.*;
import team1403.robot.subsystems.*;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.swerve.TunerConstants;
import team1403.robot.vision.AprilTagCamera;
import team1403.robot.Constants;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private SwerveSubsystem m_swerve;
  
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController;
  private final CommandXboxController m_operatorController;

  private final PowerDistribution m_powerDistribution;

  private SendableChooser<Command> autoChooser;

  private Command m_teleopCommand;

  private final double MaxSpeed;
  private final double MaxAngularRate;
  private final SwerveRequest.FieldCentric drive;
  private final SwerveRequest.SwerveDriveBrake brake;
  private final SwerveRequest.PointWheelsAt point;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 2.0; // kSpeedAt12Volts desired top speed
    MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) / 2.0; // 3/4 of a rotation per second max angular velocity
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    brake = new SwerveRequest.SwerveDriveBrake();
    point = new SwerveRequest.PointWheelsAt();
    
    // Configure the trigger bindings
    m_driverController = new CommandXboxController(Constants.Driver.pilotPort);
    m_operatorController = new CommandXboxController(Constants.Operator.pilotPort);
    Blackbox.init();
    m_swerve = TunerConstants.createDrivetrain();
    


    // Enables power distribution logging
    m_powerDistribution = new PowerDistribution(Constants.CanBus.powerDistributionID, ModuleType.kRev);
    //m_operatorController.b().whileTrue(() -> m_intakeSubsystem.setIntakeMotorSpeed(0));
   // m_operatorController.a().whileTrue().new InstantCommand(() -> m_elevator.)
    


    autoChooser = AutoBuilder.buildAutoChooser();
    
    // //avoid cluttering up auto chooser at competitions
    // if (Constants.ENABLE_SYSID) {
    //   autoChooser.addOption("Swerve SysID QF", m_swerve.getSysIDQ(Direction.kForward));
    //   autoChooser.addOption("Swerve SysID QR", m_swerve.getSysIDQ(Direction.kReverse));
    //   autoChooser.addOption("Swerve SysID DF", m_swerve.getSysIDD(Direction.kForward));
    //   autoChooser.addOption("Swerve SysID DR", m_swerve.getSysIDD(Direction.kReverse));
    //   autoChooser.addOption("Swerve SysID Steer QF", m_swerve.getSysIDSteerQ(Direction.kForward));
    //   autoChooser.addOption("Swerve SysID Steer QR", m_swerve.getSysIDSteerQ(Direction.kReverse));
    //   autoChooser.addOption("Swerve SysID Steer DF", m_swerve.getSysIDSteerD(Direction.kForward));
    //   autoChooser.addOption("Swerve SysID Steer DR", m_swerve.getSysIDSteerD(Direction.kReverse));
    // }

    // autoChooser.addOption("Choreo Auto", AutoUtil.loadChoreoAuto("test", m_swerve));
    // autoChooser.addOption("FivePieceCenter", AutoHelper.getFivePieceAuto(m_swerve));

    SmartDashboard.putData("Auto Chooser", autoChooser);
    if(Constants.DEBUG_MODE) {
      SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
      // SmartDashboard.putData("Swerve Drive", m_swerve);
      SmartDashboard.putData("Power Distribution", m_powerDistribution);
    }

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // Setting default command of swerve subPsystem
    // red
    
    m_swerve.setDefaultCommand(new DefaultSwerveCommand(
      m_swerve,
      () -> -m_driverController.getLeftX(),
      () -> -m_driverController.getLeftY(),
      () -> -m_driverController.getRightX(),
      () -> m_driverController.getHID().getBButton(),
      () -> m_driverController.getRightTriggerAxis(),
      () -> m_driverController.getLeftTriggerAxis()));

    // m_swerve.setDefaultCommand(new DefaultSwerveCommand(
    //     m_swerve,
    //     () -> -m_driverController.getLeftX(),
    //     () -> -m_driverController.getLeftY(),
    //     () -> -m_driverController.getRightX(),
    //     () -> m_driverController.getHID().getYButtonPressed(),
    //     () -> m_driverController.getHID().getBButtonPressed(),
    //     () -> m_driverController.getHID().getXButton(),
    //     () -> m_driverController.getRightTriggerAxis(),
    //     () -> m_driverController.getLeftTriggerAxis()));

    // Command driverVibrationCmd = new ControllerVibrationCommand(m_driverController.getHID(), 0.28, 1);

    // //m_driverController.povRight().onTrue(Blackbox.reefSelect(ReefSelect.RIGHT));
    // //m_driverController.povLeft().onTrue(Blackbox.reefSelect(ReefSelect.LEFT));

    // m_driverController.rightBumper().onTrue(Blackbox.setAligningCmd(true, ReefSelect.RIGHT));

    // m_driverController.leftBumper().whileTrue(Blackbox.setAligningCmd(true,ReefSelect.LEFT));

    // //m_driverController.a().onTrue(new ControllerVibrationCommand(m_driverController.getHID(), 0.28, 1));
    // //SmartDashboard.putNumber("vibration", 0);

    // m_driverController.b().onTrue(m_swerve.runOnce(() -> m_swerve.zeroHeading()));



    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public Command getTeleopCommand() {
    return m_teleopCommand;
  }
}
