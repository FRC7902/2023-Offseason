// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import frc.robot.util.SparkMaxEncoderWrapper;

import java.io.Serial;
import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveSubsystem extends SubsystemBase {

  // Declare motor controllers
  private final CANSparkMax m_leftleader = new CANSparkMax(13, MotorType.kBrushless);
  private final CANSparkMax m_leftfollower = new CANSparkMax(14, MotorType.kBrushless);
  private final CANSparkMax m_rightleader = new CANSparkMax(15, MotorType.kBrushless);
  private final CANSparkMax m_rightfollower = new CANSparkMax(16, MotorType.kBrushless);

  private final RelativeEncoder m_leftEncoder = m_leftleader.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightleader.getEncoder();

  private final AnalogGyro m_gyro = new AnalogGyro(1);
  private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

  private final MotorControllerGroup left = new MotorControllerGroup(m_leftleader, m_leftfollower);
  private final MotorControllerGroup right = new MotorControllerGroup(m_rightleader, m_rightfollower);
  private final DifferentialDrive drive = new DifferentialDrive(left, right);

  Encoder m_leftEncoderDummy = new Encoder(0, 1);
  Encoder m_rightEncoderDummy = new Encoder(2, 3);
  

  EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoderDummy);
  EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoderDummy);

  private final DifferentialDriveOdometry m_odometry;

  private Field2d m_field = new Field2d();

  DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
      DCMotor.getNEO(2),
      10.71,
      7.5,
      60,
      Units.inchesToMeters(6 / 2),
      0.7112,
      null);

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    left.setInverted(true);
    right.setInverted(false);


    m_leftEncoder.setPositionConversionFactor(Constants.DriveConstants.distanceInMetersPerMotorTick);
    m_rightEncoder.setPositionConversionFactor(Constants.DriveConstants.distanceInMetersPerMotorTick);
    m_leftEncoderDummy.setReverseDirection(true);

    // Dummy encoders driving (linear) distance per encoder tick
    m_leftEncoderDummy.setDistancePerPulse(2 * Math.PI * Units.inchesToMeters(3) / 42);
    m_leftEncoderDummy.setDistancePerPulse(2 * Math.PI * Units.inchesToMeters(3) / 42);

    if (Robot.isSimulation()) {
      m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d().times(-1),
          m_leftEncoderDummy.getDistance(), m_rightEncoderDummy.getDistance());
          resetPose(getPose());

    } else {
      m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(),
          m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    }

    AutoBuilder.configureRamsete(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getWheelSpeeds, // Current ChassisSpeeds supplier
        this::driveSpeeds, // Method that will drive the robot given ChassisSpeeds
        new ReplanningConfig(), // Default path replanning config. See the API for the options here
        this // Reference to this subsystem to set requirements
    );

    SmartDashboard.putData("Field", m_field);

  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    m_rightEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public ChassisSpeeds getWheelSpeeds() {

    if (Robot.isSimulation()) {
      return new ChassisSpeeds(m_leftEncoderDummy.getRate(), m_rightEncoderDummy.getRate(),
          m_gyro.getRate());
    } else {
      return new ChassisSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity(),
          Units.degreesToRadians(m_gyro.getRate()));
    }

  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public void driveSpeeds(ChassisSpeeds speeds) {

    this.driveArcade(speeds.vxMetersPerSecond / 15, speeds.omegaRadiansPerSecond / 15);

  }

  public void driveArcade(double xForward, double zRotation) {
    drive.arcadeDrive(xForward, zRotation);
  }

  public void driveRaw(double power) {
    left.set(power);
    right.set(power);
  }

  public void turnLeft(double amount) {
    left.set(-amount);
    right.set(amount);
  }

  public void turnRight(double amount) {
    left.set(amount);
    right.set(-amount);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoderDummy.getDistance(), m_rightEncoderDummy.getDistance());
  
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {

    // Update the inputs
    m_driveSim.setInputs(m_leftleader.get() * 12,
        m_rightleader.get() * -12);

    // Update all of our sensors.
    m_leftEncoderSim.setDistance(-m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(-m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(-m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(-m_driveSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(m_driveSim.getHeading().getDegrees());

    m_driveSim.update(0.02);

  }
}
