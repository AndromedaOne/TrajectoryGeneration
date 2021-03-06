// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import frc.robot.BetterTrajectoryGenerators.BetterStatesGeneratorFactory;
import frc.robot.Logging.Logging;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public final class Main {
  private Main() {
  }
  public static final String path = "/Users/seandoyle/test/PathWeaver/output/BarrelLessPoints.wpilib.json";
  private static final String[] pathToOriginalFile = path.split("/");
  private static final String nameOfFile = pathToOriginalFile[pathToOriginalFile.length - 1];
  public static final String dest = "/Users/seandoyle/git/2020Code/src/main/deploy/paths/" + nameOfFile;
  private static final boolean regenerateTrajectory = true;


  public static void main(String... args) {

    Trajectory trajectory = createTrajectory();
    JSONArray jsonArray = getJsonArray();
    List<State> jsonPathPoints = getPathPoints(jsonArray);
    
    List<State> betterJsonPathPoints = getBetterPathPoints(jsonPathPoints);
    Trajectory betterTrajectory = BetterStatesGeneratorFactory.createTrajectory(betterJsonPathPoints);
    if(regenerateTrajectory){
      TrajectoryConfig trajectoryConfig = getTrajectoryConfig(jsonPathPoints);
      betterTrajectory = BetterStatesGeneratorFactory.createTrajectory(betterJsonPathPoints, trajectoryConfig);
    }
    
    System.out.println("New Logs!");
    Logging.createCurvaturesCSV("GeneratedCurvatures.csv", jsonPathPoints, betterJsonPathPoints);
    Logging.createVelocitiesCSV("GeneratedVelocities.csv", trajectory, betterTrajectory);
    Logging.createXYCSV("Trajectory.csv", trajectory);
    Logging.createXYCSV("NewTrajectory.csv", betterTrajectory);
    Logging.createTimeXYCSV("TimedTrajectory.csv", trajectory);
    Logging.plotRotation("Rotation.csv", jsonPathPoints);
    Logging.plotRotation("NewRotation.csv", betterJsonPathPoints);
    
    exportTrajectory(trajectory);
    
  }

  private static TrajectoryConfig getTrajectoryConfig(List<State> originalStates) {
    double totalVelocity = 0;
    for(State state: originalStates) {
      if(Math.abs(state.velocityMetersPerSecond) > maxVelocity) {
        maxVelocity = Math.abs(state.velocityMetersPerSecond);
      }
      if(Math.abs(state.accelerationMetersPerSecondSq) > maxAcceleration) {
        maxAcceleration = Math.abs(state.accelerationMetersPerSecondSq);
      }
      totalVelocity += state.velocityMetersPerSecond;
    }
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(maxVelocity, maxAcceleration);
    trajectoryConfig.setEndVelocity(0);
    trajectoryConfig.setStartVelocity(0);
    trajectoryConfig.setReversed(totalVelocity < 0);
    trajectoryConfig.addConstraint(new MyTrajectoryContraint());
    trajectoryConfig.setKinematics(new DifferentialDriveKinematics(0.6));
    return trajectoryConfig;
  }

  private static double maxVelocity = 0;
  private static double maxAcceleration = 0;

  public static class MyTrajectoryContraint implements TrajectoryConstraint {

    @Override
    public double getMaxVelocityMetersPerSecond(Pose2d poseMeters, double curvatureRadPerMeter,
        double velocityMetersPerSecond) {
      return maxVelocity;
    }

    @Override
    public MinMax getMinMaxAccelerationMetersPerSecondSq(Pose2d poseMeters, double curvatureRadPerMeter,
        double velocityMetersPerSecond) {
      return new MinMax(-maxAcceleration, maxAcceleration);
    }

  }

  private static List<State> getBetterPathPoints(List<State> jsonPathPoints) {
    return BetterStatesGeneratorFactory.create(jsonPathPoints).getBetterStates();
  }

  public static Trajectory createTrajectory() {
    Trajectory trajectory = null;
    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get(path));
    } catch (IOException e1) {
      e1.printStackTrace();
    }

    return trajectory;
  }

  public static JSONArray getJsonArray() {
    JSONParser parser = new JSONParser();
    JSONArray jsonArray = null;
    try {
      Object obj = parser.parse(new FileReader(path));
      jsonArray = (JSONArray) obj;

    } catch (Exception e) {
      e.printStackTrace();
    }
    return jsonArray;
  }

  public static List<State> getPathPoints(JSONArray jsonArray) {
    List<State> jsonPathPoints = new ArrayList<State>();
    for (Object point : jsonArray) {
      point = (JSONObject) point;
      String pointInString = point.toString();
      double acceleration = getDouble(pointInString, "acceleration\":", ",\"pose");
      double rotation = getDouble(pointInString, "radians\":", "},\"transl");
      double x = getDouble(pointInString, "{\"x\":", ",\"y\":");
      double y = getDouble(pointInString, ",\"y\":", "}},\"tim");
      double velocity = getDouble(pointInString, "velocity\":", ",\"curvat");
      double curvature = getDouble(pointInString, "curvature\":");
      double time = getDouble(pointInString, "time\":", ",\"veloc");
      jsonPathPoints
          .add(new State(time, velocity, acceleration, new Pose2d(x, y, new Rotation2d(rotation)), curvature));
    }

    return jsonPathPoints;
  }

  public static double getDouble(String fullString, String leftString) {
    int leftIndex = fullString.indexOf(leftString) + leftString.length();
    int rightIndex = fullString.length() - 1;
    String doubleInString = fullString.substring(leftIndex, rightIndex);
    double actualDouble = Double.parseDouble(doubleInString);
    return actualDouble;
  } 
  public static double getDouble(String fullString, String leftString, String rightString) {
    int leftIndex = fullString.indexOf(leftString) + leftString.length();
    int rightIndex = fullString.indexOf(rightString);
    String doubleInString = fullString.substring(leftIndex, rightIndex);
    double actualDouble = Double.parseDouble(doubleInString);
    return actualDouble;
  }

  public static void exportTrajectory(Trajectory trajectory) {
    try {
      TrajectoryUtil.toPathweaverJson(trajectory, Path.of(dest));
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
