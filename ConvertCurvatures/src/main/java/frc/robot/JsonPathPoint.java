package frc.robot;

public class JsonPathPoint {
    public double time;
    public double velocity;
    public double acceleration;
    public double curvature;
    public double x;
    public double y;
    public double rotation;
    
    public JsonPathPoint(double time, double velocity, double acceleration, double curvature, double x, double y, double rotation) {
        this.time = time;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.curvature = curvature;
        this.x = x;
        this.y = y;
        this.rotation = rotation;
    }

    @Override
    public String toString() {
    
        return "Time: " + time
            + "\nVelocity: " + velocity
            + "\nAcceleration: " + acceleration
            + "\nCurvature: " + curvature
            + "\nX: " + x
            + "\nY: " + y
            + "\nRotation: " + rotation;
    }

    public JsonPathPoint copy() {
        return new JsonPathPoint(time, velocity,  acceleration, curvature, x,  y, rotation);
    }

}
