import json
import numpy as np
import math

map = [{"time":x/10.0,"velocity":1.0,"acceleration":0.0,
        "pose":{"translation":{"x":2*np.cos(x/10.0),"y":2*np.sin(x/10.0)},
        "rotation":{"radians":0.5}},"curvature":2.0} for x in range(0, 130, 1)]

def generatePartialCircle (initialAngle, finalAngle, radius, velocity, initialX, initialY):
   totalTime = (finalAngle - initialAngle) * radius / velocity
   rotationSpeed = (finalAngle - initialAngle) / totalTime
   theta = initialAngle - math.pi / 2
   xTranslation = initialX - radius * math.cos(theta)
   yTranslation = initialY - radius * math.sin(theta)
   circle = [{"time":t,"velocity":velocity,"acceleration":0.0,
        "pose":{"translation":rotateThenTranslate(theta, xTranslation, yTranslation, radius*np.cos(rotationSpeed * t + initialAngle), radius*np.sin(rotationSpeed * t + initialAngle)),
        "rotation":{"radians":rotationSpeed * t + initialAngle}},"curvature":1.0/radius} for t in np.arange(0, totalTime, 0.1)]
   return circle

def rotateThenTranslate(theta, xTranslation, yTranslation, x, y):
   newX = x * math.cos(theta) - y * math.sin(theta)
   newY = x * math.sin(theta) + y * math.cos(theta)
   newX = newX + xTranslation
   newY = newY + yTranslation
   return {"x": newX, "y": newY}

def accelerateLinearTrajectory(initialVelocity, finalVelocity, acceleration, initialX, initialY):
   totalTime = (finalVelocity - initialVelocity) / acceleration
   
   return [{"time":t,"velocity":initialVelocity + t*acceleration,"acceleration":acceleration,
        "pose":{"translation":{"x":((initialVelocity + t*acceleration)**2 - initialVelocity**2) / (2 * acceleration) + initialX, "y":initialY},
        "rotation":{"radians":math.atan2(initialY, initialX)}},"curvature":0} for t in np.arange(0, totalTime, 0.1)]


firstMove = accelerateLinearTrajectory(0, 2.0, 0.5, 0,0)
lastPoint = firstMove[-1]["pose"]["translation"]

circleMove = generatePartialCircle(0, 2* math.pi, 1.5, 2.0, lastPoint["x"], lastPoint["y"])
lastPoint = circleMove[-1]["pose"]["translation"]

finalMove = accelerateLinearTrajectory(2.0, 0.0, -0.5, lastPoint["x"], lastPoint["y"])

fullMove = firstMove + circleMove + finalMove
if(False):
        print(firstMove)
        print()
        print()
        print()
        print(circleMove)
else:
        print(json.dumps(fullMove))
        print()