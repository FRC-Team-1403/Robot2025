// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.robot;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.GeometryUtil;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team1403.lib.auto.TreeAuto;
import team1403.lib.auto.TreeCommandNode;
import team1403.lib.auto.TreeCommandProxy;
import team1403.lib.util.AutoUtil;
import team1403.lib.util.CougarUtil;
import team1403.robot.Constants.Driver;
import team1403.robot.Constants.Setpoints;
import team1403.robot.autos.AutoHelper;
import team1403.robot.subsystems.Blackbox;
import team1403.robot.swerve.DefaultSwerveCommand;
import team1403.robot.swerve.SwerveSubsystem;

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
  private Command m_pathFinder = Commands.none();
  private Command m_teleopCommand = Commands.none();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    m_swerve = new SwerveSubsystem();
    m_driverController = new CommandXboxController(Constants.Driver.pilotPort);
    m_operatorController = new CommandXboxController(Constants.Operator.pilotPort);
    // Enables power distribution logging
    m_powerDistribution = new PowerDistribution(Constants.CanBus.powerDistributionID, ModuleType.kRev);
    if(Constants.DEBUG_MODE) Constants.kDebugTab.add("Power Distribution", m_powerDistribution);

    Constants.kDriverTab.addDouble("Battery Voltage", () -> m_powerDistribution.getVoltage());
    Constants.kDriverTab.addDouble("Match Time", () -> DriverStation.getMatchTime());
    DogLog.setPdh(m_powerDistribution);

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
    autoChooser.addOption("Choreo Auto", AutoUtil.loadChoreoAuto("test", m_swerve));
    autoChooser.addOption("FivePieceCenter", AutoHelper.getFivePieceAuto(m_swerve));

    Constants.kDriverTab.add("Auto Chooser", autoChooser);
    if(Constants.DEBUG_MODE) {
      Constants.kDebugTab.add("Command Scheduler", CommandScheduler.getInstance());
      Constants.kDebugTab.add("Swerve Drive", m_swerve);
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

    Translation2d pos_blue = new Translation2d(-0.038099999999999995,  5.547867999999999);
    Translation2d pos_red = FlippingUtil.flipFieldPosition(pos_blue);
    Pose2d pos_blue_shoot = new Pose2d(new Translation2d(1.5, 5.4), new Rotation2d());
    Pose2d pos_red_shoot = FlippingUtil.flipFieldPose(pos_blue_shoot);
    Pose2d pose_blue_amp = new Pose2d(new Translation2d(1.813, 7.715), new Rotation2d(-Math.PI/2));
    Pose2d pose_red_amp = new Pose2d(FlippingUtil.flipFieldPosition(pose_blue_amp.getTranslation()), new Rotation2d(-Math.PI/2));
    
    m_swerve.setDefaultCommand(new DefaultSwerveCommand(
        m_swerve,
        () -> -m_driverController.getLeftX(),
        () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getRightX(),
        () -> m_driverController.getHID().getYButtonPressed(),
        () -> m_driverController.getHID().getXButton(),
        () -> m_driverController.getHID().getAButton(),
        () -> m_driverController.getHID().getLeftBumperButton(),
        () -> m_driverController.getHID().getRightBumperButton(),
        () -> CougarUtil.getAlliance() == Alliance.Blue ? pos_blue : pos_red,
        () -> m_driverController.getRightTriggerAxis(),
        () -> m_driverController.getLeftTriggerAxis()));

    m_driverController.b().onTrue(m_swerve.runOnce(() -> m_swerve.zeroHeading()));

    m_driverController.rightBumper().whileTrue(Commands.runOnce(() -> {
      Pose2d tar = pos_red_shoot;
      if(CougarUtil.getAlliance() == Alliance.Blue) tar = pos_blue_shoot;
      Blackbox.targetPosition = tar;
      m_pathFinder = AutoUtil.pathFindToPose(tar);
    }).andThen(m_swerve.defer(() -> m_pathFinder)));

    m_driverController.leftBumper().whileTrue(Commands.runOnce(() -> {
      Pose2d tar = pose_red_amp;
      if(CougarUtil.getAlliance() == Alliance.Blue) tar = pose_blue_amp;
      Blackbox.targetPosition = tar;
      m_pathFinder = AutoUtil.pathFindToPose(tar);
    }).andThen(m_swerve.defer(() -> m_pathFinder)));

    //disable NT publish if FMS is attached at any point
    new Trigger(() -> DriverStation.isFMSAttached())
    .onTrue(new InstantCommand(
      () -> DogLog.setOptions(
        DogLog.getOptions().withNtPublish(false))));
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
}
