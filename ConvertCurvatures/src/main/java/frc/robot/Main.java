// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;

import org.json.*;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public final class Main {
  private Main() {}
  private static Trajectory trajectory;
  private static String path = "/Users/seandoyle/test/PathWeaver/output/BarrelLessPoints.wpilib.json";
  
  public static void main(String... args) {
    System.out.println("Hello World");
    trajectory = null;
    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get(path));
    } catch (IOException e1) {
      e1.printStackTrace();
    }
    JSONParser parser = new JSONParser();
    JSONArray jsonArray = null;
    try {
      Object obj = parser.parse(new FileReader(path));
      jsonArray = (JSONArray )obj;
      
    } catch(Exception e) {
        e.printStackTrace();
    }
    List<JsonPathPoint> jsonPathPoints = new ArrayList<JsonPathPoint>();
    for (Object point : jsonArray){
      point = (JSONObject)point;
      String pointInString = point.toString();
      double acceleration = getDouble(pointInString, "acceleration\":", ",\"pose");
      double rotation = getDouble(pointInString, "radians\":", "},\"transl");
      double x = getDouble(pointInString, "{\"x\":", ",\"y\":");
      double y = getDouble(pointInString, ",\"y\":", "}},\"tim");
      double velocity = getDouble(pointInString, "velocity\":", ",\"curvat");
      double curvature = getDouble(pointInString, "curvature\":");
      double time = getDouble(pointInString, "time\":", ",\"veloc");
      jsonPathPoints.add(new JsonPathPoint(time, velocity, acceleration, curvature, x, y, rotation));
    }
    
    double cumulativeDifferenceInCurvature = 0;
    int count = 0;
    List<Double> newCurvatures = new ArrayList<Double>();
    List<JsonPathPoint> betterJsonPathPoints = new ArrayList<JsonPathPoint>();
    for(JsonPathPoint point : jsonPathPoints) {
      double newCurvature = 0;
      if(count == 0 || count == jsonPathPoints.size() - 1) {
        newCurvature = point.curvature;
      }else if(Math.abs(point.curvature) > 1.0e-6) {
        newCurvature = point.curvature;
      }else {
        JsonPathPoint previousPoint = jsonPathPoints.get(count - 1);
        JsonPathPoint nextPoint = jsonPathPoints.get(count + 1);
        double totalTime = nextPoint.time - previousPoint.time;
        double curvatureWeightedAverage = previousPoint.curvature * (point.time - previousPoint.time) / totalTime + nextPoint.curvature * (nextPoint.time - point.time) / totalTime;
        newCurvature = curvatureWeightedAverage;
      }
      JsonPathPoint betterPoint = point.copy();
      betterPoint.curvature = newCurvature;
      betterJsonPathPoints.add(betterPoint);
      newCurvatures.add(newCurvature);
      count++;
    }
    System.out.println("Average difference in curvatures: " + cumulativeDifferenceInCurvature / count);
    // Now output my curvatures to a csv file and plot that!!!!
    try{
      FileWriter csvWriter = new FileWriter("GeneratedCurvatures.csv");
      csvWriter.append("Time, OriginalCurvature, NewCurvature, x, y\n");
      int count2 = 0;
      for(JsonPathPoint point : jsonPathPoints) {
        csvWriter.append(point.time + ", " + point.curvature +  ", " + newCurvatures.get(count2) +  ", " + (point.x - 5) + ", " + (point.y - 1.572) + "\n");
        count2++;
      }
      csvWriter.flush();
      csvWriter.close();
    }catch(IOException e) {
      e.printStackTrace();
    }

    try{
      FileWriter csvWriter = new FileWriter("GeneratedVelocities.csv");
      csvWriter.append("Time, OriginalLeftVelocity, OriginalRightVelocity, NewLeftVelocity, NewRightVelocity\n");
      RamseteController follower = new RamseteController();
      DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.6);
      List<State> states = trajectory.getStates();
      List<State> betterStates = new ArrayList<State>();
      int count2 = 0;
      for (Trajectory.State state : states) {
        State betterState = new State(
          state.timeSeconds,
          state.velocityMetersPerSecond,
          state.accelerationMetersPerSecondSq,
          state.poseMeters,
          newCurvatures.get(count2)
        );
        count2++;
        betterStates.add(betterState);
      }
      Trajectory betterTrajectory = new Trajectory(betterStates);
      for(double t = 0.0; t < trajectory.getTotalTimeSeconds(); t += 0.02){
        var targetWheelSpeeds =
        kinematics.toWheelSpeeds(
          follower.calculate(trajectory.sample(t).poseMeters, trajectory.sample(t)));

        double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;
        var betterTargetWheelSpeeds = kinematics.toWheelSpeeds(
          follower.calculate(betterTrajectory.sample(t).poseMeters, betterTrajectory.sample(t)));
        double betterLeftSpeedSetpoint = betterTargetWheelSpeeds.leftMetersPerSecond;
        double betterRightSpeedSetpoint = betterTargetWheelSpeeds.rightMetersPerSecond;
        csvWriter.append(t + ", " + leftSpeedSetpoint + ", " + rightSpeedSetpoint + ", " + betterLeftSpeedSetpoint + ", " + betterRightSpeedSetpoint + "\n");
      }
      csvWriter.flush();
      csvWriter.close();
    }catch(IOException e) {
      e.printStackTrace();
    }
    
    

  }
  private static double epsilon = 1.0e-2;

  public static double getDerivativeForX(double time, Trajectory trajectory) {
    double deltaX = trajectory.sample(time + (epsilon / 2.0)).poseMeters.getX() - trajectory.sample(time - (epsilon / 2.0)).poseMeters.getX();
    return deltaX / epsilon;
  }

  public static double getDerivativeForY(double time, Trajectory trajectory) {
    double deltaY = trajectory.sample(time + (epsilon / 2.0)).poseMeters.getY() - trajectory.sample(time - (epsilon / 2.0)).poseMeters.getY();
    return deltaY / epsilon;
  }

  public static double getSecondDerivativeForX(double time, Trajectory trajectory) {
    double deltaDx = getDerivativeForX(time + (epsilon / 2.0), trajectory) - getDerivativeForX(time - (epsilon / 2.0), trajectory);
    return deltaDx / epsilon;
  }

  public static double getSecondDerivativeForY(double time, Trajectory trajectory) {
    double deltaDy = getDerivativeForY(time + (epsilon / 2.0), trajectory) - getDerivativeForY(time - (epsilon / 2.0), trajectory);
    return deltaDy / epsilon;
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
}
