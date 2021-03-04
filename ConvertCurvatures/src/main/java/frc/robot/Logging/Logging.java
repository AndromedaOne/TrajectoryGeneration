package frc.robot.Logging;

import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.differentiation.DerivativeStructure;
import org.apache.commons.math3.analysis.differentiation.FiniteDifferencesDifferentiator;
import org.apache.commons.math3.analysis.differentiation.UnivariateDifferentiableFunction;
import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;

public class Logging {
    public static final double TIMESTEP = 0.02;

    public static void createCurvaturesCSV(String name, List<State> originalJsonPathPoints, List<State> newJsonPathPoints) {
        if(originalJsonPathPoints.size() != newJsonPathPoints.size()) {
            throw new RuntimeException("The two Lists of States are not the same length when plotting curavatures!");
        }
        try {
            FileWriter csvWriter = new FileWriter(name);
            csvWriter.append("Time, OriginalCurvature, NewCurvature, x, y\n");
            int count = 0;
            for (State point : originalJsonPathPoints) {
              csvWriter.append(point.timeSeconds + ", " + point.curvatureRadPerMeter + ", " + newJsonPathPoints.get(count).curvatureRadPerMeter + ", " + (point.poseMeters.getX())
                  + ", " + (point.poseMeters.getY()) + "\n");
              count++;
            }
            csvWriter.flush();
            csvWriter.close();
          } catch (IOException e) {
            e.printStackTrace();
          }
    }

    public static void createVelocitiesCSV(String name, Trajectory originalTrajectory, Trajectory newTrajectory) {
        try {
            FileWriter csvWriter = new FileWriter(name);
            csvWriter.append("Time, OriginalLeftVelocity, OriginalRightVelocity, NewLeftVelocity, NewRightVelocity\n");
            RamseteController follower = new RamseteController();
            DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.6);
            for (double t = 0.0; t < originalTrajectory.getTotalTimeSeconds(); t += TIMESTEP) {
              var targetWheelSpeeds = kinematics
                  .toWheelSpeeds(follower.calculate(originalTrajectory.sample(t).poseMeters, originalTrajectory.sample(t)));
      
              double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
              double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

              var newTargetWheelSpeeds = kinematics
                  .toWheelSpeeds(follower.calculate(newTrajectory.sample(t).poseMeters, newTrajectory.sample(t)));
              double newLeftSpeedSetpoint = newTargetWheelSpeeds.leftMetersPerSecond;
              double newRightSpeedSetpoint = newTargetWheelSpeeds.rightMetersPerSecond;
              csvWriter.append(t + ", " + leftSpeedSetpoint + ", " + rightSpeedSetpoint + ", " + newLeftSpeedSetpoint
                  + ", " + newRightSpeedSetpoint + "\n");
            }
            csvWriter.flush();
            csvWriter.close();
          } catch (IOException e) {
            e.printStackTrace();
          }
    }

    public static void createXYCSV(String name, Trajectory trajectory) {
        try {
            FileWriter csvWriter = new FileWriter(name);
            csvWriter.append("x, y\n");
            for (double t = 0.0; t < trajectory.getTotalTimeSeconds(); t += TIMESTEP) {
              State state = trajectory.sample(t);
              csvWriter.append(state.poseMeters.getX() + ", " + state.poseMeters.getY() + "\n");
            }
            csvWriter.flush();
            csvWriter.close();
          } catch (IOException e) {
            e.printStackTrace();
          }
    }

    public static void createTimeXYCSV(String name, Trajectory trajectory) {
      try {
          FileWriter csvWriter = new FileWriter(name);
          csvWriter.append("Time, x, y\n");
          for (double t = 0.0; t < trajectory.getTotalTimeSeconds(); t += TIMESTEP) {
            State state = trajectory.sample(t);
            csvWriter.append(state.timeSeconds + ", " + state.poseMeters.getX() + ", " + state.poseMeters.getY() + "\n");
          }
          csvWriter.flush();
          csvWriter.close();
      } catch (IOException e) {
          e.printStackTrace();
      }
    }
    public static void compareGeneratedPolynomialForY(String name, List<State> states, UnivariateFunction polynomial) {
      try {
        FileWriter csvWriter = new FileWriter(name);
        csvWriter.append("Time, OriginalValue, NewValue \n");
        for (State state : states) {
          csvWriter.append(state.timeSeconds + ", " + state.poseMeters.getY() + ", " + polynomial.value(state.timeSeconds) + "\n");
        }
        csvWriter.flush();
        csvWriter.close();
      } catch (IOException e) {
        e.printStackTrace();
      }
    }

    public static void compareGeneratedPolynomialForX(String name, List<State> states, UnivariateFunction polynomial) {
      try {
        FileWriter csvWriter = new FileWriter(name);
        csvWriter.append("Time, OriginalValue, NewValue \n");
        for (State state : states) {
          csvWriter.append(state.timeSeconds + ", " + state.poseMeters.getX() + ", " + polynomial.value(state.timeSeconds) + "\n");
        }
        csvWriter.flush();
        csvWriter.close();
      } catch (IOException e) {
        e.printStackTrace();
      }
    }

}
