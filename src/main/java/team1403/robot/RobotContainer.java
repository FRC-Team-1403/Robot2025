// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.robot;

import java.util.Set;

import org.ejml.dense.row.MatrixFeatures_CDRM;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;

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
import team1403.robot.commands.AlignCommand;
import team1403.robot.commands.ControllerVibrationCommand;
import team1403.robot.commands.IntakeShooterLoop;
import team1403.robot.subsystems.ArmWrist;
import team1403.robot.subsystems.Blackbox;
import team1403.robot.subsystems.Blackbox.ReefSelect;
import team1403.robot.subsystems.IntakeAndShooter;
import team1403.robot.swerve.DefaultSwerveCommand;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.vision.AprilTagCamera;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private SwerveSubsystem m_swerve;
  private ArmWrist m_armwrist = new ArmWrist();
  private IntakeAndShooter m_intakeShooter = new IntakeAndShooter();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController;
  private final CommandXboxController m_operatorController;

  private final PowerDistribution m_powerDistribution;

  private SendableChooser<Command> autoChooser;
  private Command m_teleopCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    m_swerve = new SwerveSubsystem();
    // initialize the blackbox subsystem so that data can be reference later
    Blackbox.getInstance();
    m_driverController = new CommandXboxController(Constants.Driver.pilotPort);
    m_operatorController = new CommandXboxController(Constants.Operator.pilotPort);
    // Enables power distribution logging
    m_powerDistribution = new PowerDistribution(Constants.CanBus.powerDistributionID, ModuleType.kRev);
    if(Constants.DEBUG_MODE) SmartDashboard.putData("Power Distribution", m_powerDistribution);
    // DogLog.setPdh(m_powerDistribution);

    // NamedCommands.registerCommand("stop", new InstantCommand(() -> m_swerve.stop()));
    // NamedCommands.registerCommand("First Piece", new AutoIntakeShooterLoop(m_endeff, m_arm, m_wrist, m_led, () -> false, () -> false, false, () -> false, false));
    // NamedCommands.registerCommand("Shoot Side", new AutoIntakeShooterLoop(m_endeff, m_arm, m_wrist, m_led, () -> true, () -> false, true, () -> false, false));
    // NamedCommands.registerCommand("Shoot", new AutoIntakeShooterLoop(m_endeff, m_arm, m_wrist, m_led, () -> true, () -> false, false, () -> false, false));
    // NamedCommands.registerCommand("Reset Shooter", new AutoIntakeShooterLoop(m_endeff, m_arm, m_wrist, m_led, () -> false, () -> false, false, () -> false, false));
    // NamedCommands.registerCommand("First Piece Side",  new AutoIntakeShooterLoop(m_endeff, m_arm, m_wrist, m_led, () -> false, () -> false, true, () -> true, false));
    // NamedCommands.registerCommand("Second Source Shoot", new AutoIntakeShooterLoop(m_endeff, m_arm, m_wrist, m_led, () -> true, () -> false, false, () -> false, true));
    // NamedCommands.registerCommand("IntakeClose", new IntakeCommand(m_endeff, m_arm, m_wrist,  Constants.Arm.kDriveSetpoint, Constants.Wrist.kDriveSetpoint, Constants.IntakeAndShooter.kCloseRPM));    
    // NamedCommands.registerCommand("ShootLoaded", new ShootCommand(m_endeff, m_arm, m_wrist));
    // NamedCommands.registerCommand("Trigger Shot", new TriggerShotCommand(m_endeff, m_wrist));

    // NamedCommands.registerCommand("Trigger Shot", new TriggerShotCommand());

    autoChooser = AutoBuilder.buildAutoChooser();
    
    //avoid cluttering up auto chooser at competitions
    if (Constants.ENABLE_SYSID) {
      autoChooser.addOption("Swerve SysID QF", m_swerve.getSysIDQ(Direction.kForward));
      autoChooser.addOption("Swerve SysID QR", m_swerve.getSysIDQ(Direction.kReverse));
      autoChooser.addOption("Swerve SysID DF", m_swerve.getSysIDD(Direction.kForward));
      autoChooser.addOption("Swerve SysID DR", m_swerve.getSysIDD(Direction.kReverse));
      autoChooser.addOption("Swerve SysID Steer QF", m_swerve.getSysIDSteerQ(Direction.kForward));
      autoChooser.addOption("Swerve SysID Steer QR", m_swerve.getSysIDSteerQ(Direction.kReverse));
      autoChooser.addOption("Swerve SysID Steer DF", m_swerve.getSysIDSteerD(Direction.kForward));
      autoChooser.addOption("Swerve SysID Steer DR", m_swerve.getSysIDSteerD(Direction.kReverse));

      autoChooser.addOption("Shooter SysID Dynamic Forward", m_intakeShooter.getSysIDD(Direction.kForward));
      autoChooser.addOption("Shooter SysID Dynamic Reverse", m_intakeShooter.getSysIDD(Direction.kReverse));
      autoChooser.addOption("Shooter SysID Quasistatic Forward", m_intakeShooter.getSysIDQ(Direction.kForward));
      autoChooser.addOption("Shooter SysID Quasistatic Reverse", m_intakeShooter.getSysIDQ(Direction.kReverse));
    }

    // autoChooser.addOption("Choreo Auto", AutoUtil.loadChoreoAuto("test", m_swerve));
    // autoChooser.addOption("FivePieceCenter", AutoHelper.getFivePieceAuto(m_swerve));

    SmartDashboard.putData("Auto Chooser", autoChooser);
    if(Constants.DEBUG_MODE) {
      SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
      SmartDashboard.putData("Swerve Drive", m_swerve);
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
        () -> m_driverController.getHID().getYButtonPressed(),
        () -> m_driverController.getHID().getBButtonPressed(),
        () -> m_driverController.getHID().getXButton(),
        () -> m_driverController.getRightTriggerAxis(),
        () -> m_driverController.getLeftTriggerAxis()));

    m_teleopCommand = new IntakeShooterLoop(m_intakeShooter, m_armwrist, () -> m_operatorController.getHID().getRightTriggerAxis() > 0.5, () -> m_operatorController.getHID().getBButton(), () -> m_operatorController.getHID().getAButton());

    m_armwrist.setDefaultCommand(m_teleopCommand);

    //m_driverController.povRight().onTrue(Blackbox.reefSelect(ReefSelect.RIGHT));
    //m_driverController.povLeft().onTrue(Blackbox.reefSelect(ReefSelect.LEFT));

    m_driverController.rightBumper().whileTrue(new DeferredCommand(() -> {
      Blackbox.reefSelect(ReefSelect.RIGHT);
      Pose2d currentPose = m_swerve.getPose();
      Pose2d target = Blackbox.getNearestAlignPositionReef(currentPose);
      if (target == null) return Commands.none();  
      return Commands.sequence(
        AutoUtil.pathFindToPose(target),
        new AlignCommand(m_swerve, target),
        new ControllerVibrationCommand(m_driverController.getHID(), 0.28, 1)
      );
     }, Set.of(m_swerve)));

     m_driverController.leftBumper().whileTrue(new DeferredCommand(() -> {
      Blackbox.reefSelect(ReefSelect.LEFT);
      Pose2d currentPose = m_swerve.getPose();
      Pose2d target = Blackbox.getNearestAlignPositionReef(currentPose);
      if (target == null) return Commands.none();
      return Commands.sequence(
        AutoUtil.pathFindToPose(target),
        new AlignCommand(m_swerve, target),
        new ControllerVibrationCommand(m_driverController.getHID(), 0.28, 1)
      );
     }, Set.of(m_swerve)));

    //m_driverController.a().onTrue(new ControllerVibrationCommand(m_driverController.getHID(), 0.28, 1));
    //SmartDashboard.putNumber("vibration", 0);

    m_driverController.b().onTrue(m_swerve.runOnce(() -> m_swerve.zeroHeading()));
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
