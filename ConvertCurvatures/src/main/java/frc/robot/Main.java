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
  
  // CHECK THESE VARIABLES BEFORE RUNNING ON YOUR COMPUTER
  public static final double TRACKWIDTH = 0.142072613;
  public static final String INITIAL_DIR = "/Users/seandoyle/git/2020Code/pathweaverprojects/romi/Pathweaver/output/";
  public static final String DESTINATION_DIR = "/Users/seandoyle/git/2020Code/src/main/deploy/paths/";
  
  
  public static final List<String> paths = new ArrayList<String>();
  static {
    /*paths.add("/Users/seandoyle/test/PathWeaver/output/BarrelLessPoints.wpilib.json");
    paths.add("/Users/seandoyle/test/PathWeaver/output/Slalom.wpilib.json");
    paths.add("/Users/seandoyle/test/PathWeaver/output/BouncePart1.wpilib.json");
    paths.add("/Users/seandoyle/test/PathWeaver/output/BouncePart2.wpilib.json");
    paths.add("/Users/seandoyle/test/PathWeaver/output/BouncePart3.wpilib.json");
    paths.add("/Users/seandoyle/test/PathWeaver/output/BouncePart4.wpilib.json");*/
    /*paths.add("/Users/seandoyle/test/PathWeaver/output/GalacticSearchPathBRed.wpilib.json");
    paths.add("/Users/seandoyle/test/PathWeaver/output/GalacticSearchPathABlue.wpilib.json");
    paths.add("/Users/seandoyle/test/PathWeaver/output/GalacticSearchPathBBlue.wpilib.json");
    paths.add("/Users/seandoyle/test/PathWeaver/output/GalacticSearchPathARed.wpilib.json")*/;

    paths.add(INITIAL_DIR + "RomiBounceP1.wpilib.json");
    paths.add(INITIAL_DIR + "RomiBounceP2.wpilib.json");
    paths.add(INITIAL_DIR + "RomiBounceP4.wpilib.json");
    paths.add(INITIAL_DIR + "RomiBounceP3.wpilib.json");
    paths.add(INITIAL_DIR + "StraitLine.wpilib.json");
    
  }
  

  private static List<String> getNewDestinations() {
    List<String[]> originalFiles = new ArrayList<String[]>();
    for (String path : paths) {
      originalFiles.add(path.split("/"));
    }
    List<String> names = new ArrayList<String>();
    for(String[] path : originalFiles) {
      names.add(path[path.length - 1]);
    }
    List<String> destinations = new ArrayList<String>();
    for(String name : names) {
      destinations.add(DESTINATION_DIR + name);
    }
    return destinations;
  }
  
  public static final List<String> dests =  getNewDestinations();
  private static final boolean regenerateTrajectory = true;

  // java -cp build/libs/ConvertCurvatures.jar frc.robot.Main
  public static void main(String... args) {
    System.out.println("Starting!");
    int count = 0;

    for (String path : paths){
      Trajectory trajectory = createTrajectory(path);
      JSONArray jsonArray = getJsonArray(path);
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
      
      exportTrajectory(betterTrajectory, dests.get(count));
      count++;
    }
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
    trajectoryConfig.setEndVelocity(0.0);
    trajectoryConfig.setStartVelocity(0.0);
    trajectoryConfig.setReversed(totalVelocity < 0);
    trajectoryConfig.addConstraint(new MyTrajectoryContraint());  
    trajectoryConfig.setKinematics(new DifferentialDriveKinematics(TRACKWIDTH));
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

  public static Trajectory createTrajectory(String path) {
    Trajectory trajectory = null;
    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get(path));
    } catch (IOException e1) {
      e1.printStackTrace();
    }

    return trajectory;
  }

  public static JSONArray getJsonArray(String path) {
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

  public static void exportTrajectory(Trajectory trajectory, String dest) {
    try {
      TrajectoryUtil.toPathweaverJson(trajectory, Path.of(dest));
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
