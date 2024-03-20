// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeFromShooter;
import frc.robot.commands.IntakeOff;
import frc.robot.commands.IntakeOn;
import frc.robot.commands.IntakeReverse;
import frc.robot.commands.PrepareToShoot;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootAuto;
import frc.robot.commands.ShooterDown;
import frc.robot.commands.ShooterUp;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAdjuster;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));
  private final Shooter m_Shooter = new Shooter();
  private final Intake m_Intake = new Intake();
  private final ShooterAdjuster m_ShooterAdjuster = new ShooterAdjuster();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandXboxController driverXbox = new CommandXboxController(0);
  CommandXboxController operatorXbox = new CommandXboxController(1);
  // CommandJoystick pretendJoystic = new CommandJoystick(2);
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    SmartDashboard.putNumber("Amp speed", .25);
    // pretendJoystic.button(1).onTrue(getAutonomousCommand());
    // registers Named Commands
    NamedCommands.registerCommand("IntakeOff", new IntakeOff(m_Intake));
    NamedCommands.registerCommand("IntakeOn", new IntakeOn(m_Intake, false));
    NamedCommands.registerCommand("PrepareToShoot", new PrepareToShoot(m_Shooter));
    NamedCommands.registerCommand("Shoot", new Shoot(m_Shooter, 12, 12));
    NamedCommands.registerCommand("Shoot Auto", new ShootAuto(m_Shooter));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the trigger bindings
    configureBindings();

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
            OperatorConstants.RIGHT_X_DEADBAND),
        driverXbox.y(),
        driverXbox.a(),
        driverXbox.x(),
        driverXbox.b());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(),
        () -> -driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX());

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
    // m_Shooter.setDefaultCommand(new IntakeFromShooter(m_Shooter));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    driverXbox.back().onTrue((new InstantCommand(drivebase::zeroGyro)));
    // driverXbox.back().onTrue(new
    // InstantComma''''''''''''''''''''''''''''''''''''''''''''''''''nd(drivebase::addFakeVisionReading));

    // Intake Commands
    operatorXbox.start().whileTrue(new IntakeFromShooter(m_Shooter))
        .onTrue(new InstantCommand(() -> {
          m_ShooterAdjuster.goToZero();
        }, m_ShooterAdjuster));

    driverXbox.rightBumper().whileTrue(new IntakeOn(m_Intake, true))
        .whileTrue(new InstantCommand(() -> {
          m_Shooter.setBottomMotorVolts(.05 * 12.0);
        }, m_Shooter))
        .onTrue(new InstantCommand(() -> {
          m_ShooterAdjuster.goToZero();
        }, m_ShooterAdjuster));

    operatorXbox.leftTrigger().whileTrue(new IntakeReverse(m_Intake));

    // operatorXbox.rightBumper().whileTrue(new InstantCommand(() ->
    // {m_ShooterAdjuster.setSpeed(operatorXbox.getRightY() * .5);},
    // m_ShooterAdjuster));

    // Shooter Commands

    DoubleSupplier mySupplier = new DoubleSupplier() {
      @Override
      public double getAsDouble() {
        // TODO Auto-generated method stub
        return SmartDashboard.getNumber("Amp speed", .25) * 12.0;
      }

    };

    driverXbox.rightTrigger().whileTrue(new Shoot(m_Shooter, 12, 12));
    driverXbox.leftTrigger().whileTrue(new InstantCommand(() -> {
      m_Shooter.ampShot();
    }, m_Shooter))
        .onFalse(new InstantCommand(() -> {
          m_Shooter.shooterOff();
        }, m_Shooter));

    operatorXbox.rightTrigger().onTrue(new PrepareToShoot(m_Shooter));

    operatorXbox.x().onTrue(new InstantCommand(() -> {
      m_ShooterAdjuster.setPosition(Constants.ShooterAngles.UNDER_STAGE);
    }, m_ShooterAdjuster));
    operatorXbox.y().onTrue(new InstantCommand(() -> {
      m_ShooterAdjuster.goToZero();
      ;
    }, m_ShooterAdjuster));
    operatorXbox.a().onTrue(new InstantCommand(() -> {
      m_ShooterAdjuster.setPosition(Constants.ShooterAngles.AMP);
    }, m_ShooterAdjuster));

    operatorXbox.povUp().onTrue((new
    InstantCommand(()->m_ShooterAdjuster.setPosition(Constants.ShooterAngles.PEDESTAL))));
    // operatorXbox.povLeft().onTrue((new
    // InstantCommand(()->m_ShooterAdjuster.setPosition(Constants.ShooterAngles.AMP_ZONE))));

   // operatorXbox.povUp().whileTrue(new ShooterUp(m_ShooterAdjuster));
   // operatorXbox.povDown().whileTrue(new ShooterDown(m_ShooterAdjuster));
    // operatorXbox.a().onTrue((new
    // InstantCommand(()->m_ShooterAdjuster.setPosition(.90)))); //Drive Commands

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      driverXbox.povUp().onTrue((new InstantCommand(() -> drivebase.setLastAngleScalar(Math.PI)))); //DEFAULT
      driverXbox.povDown().onTrue((new InstantCommand(() -> drivebase.setLastAngleScalar(Math.PI - Math.toRadians(47))))); // AMP ZONE
      driverXbox.povRight().onTrue((new InstantCommand(() -> drivebase.setLastAngleScalar(-Math.PI / 2)))); // AMP
      driverXbox.povLeft().onTrue((new InstantCommand(() -> drivebase.setLastAngleScalar(Math.PI + Math.toRadians(31))))); // PODIUM

    } else {
      driverXbox.povUp().onTrue((new InstantCommand(() -> drivebase.setLastAngleScalar(0)))); // DEFAULT
      driverXbox.povDown().onTrue((new InstantCommand(() -> drivebase.setLastAngleScalar(Math.toRadians(47))))); // AMP ZONE
      driverXbox.povLeft().onTrue((new InstantCommand(() -> drivebase.setLastAngleScalar(-Math.PI / 2)))); // AMP
      driverXbox.povRight().onTrue((new InstantCommand(() -> drivebase.setLastAngleScalar(-Math.toRadians(31))))); // PODIUM
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return drivebase.sysIdDriveMotorCommand();
    // An example command will be run in autonomousas
    // return new PathPlannerAuto("4-Auto");
    // return new PathPlannerAuto("Straight Auto");
    // return new ShootAuto(m_Shooter);
    return autoChooser.getSelected();
  }

  public void setDriveMode() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      drivebase.setLastAngleScalar(Math.PI); //DEFAULT
    } else {
      drivebase.setLastAngleScalar(0); // DEFAULT
     }
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
