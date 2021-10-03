using System;
using BepuPhysics;

namespace Demos.Demos.Cars
{
    struct SimpleCarController
    {
        public SimpleCar Car;

        private float steeringAngle;

        public readonly float SteeringAngle { get { return steeringAngle; } }

        public float SteeringSpeed;
        public float MaximumSteeringAngle;

        public float ForwardSpeed;
        public float ForwardForce;
        public float ZoomMultiplier;
        public float BackwardSpeed;
        public float BackwardForce;
        public float IdleForce;
        public float BrakeForce;
        public float WheelBaseLength;
        public float WheelBaseWidth;
        /// <summary>
        /// Fraction of Ackerman steering angle to apply to wheels. Using 0 does not modify the the steering angle at all, leaving the wheels pointed exactly along the steering angle, while 1 uses the full Ackerman angle.
        /// </summary>
        public float AckermanSteering;

        //Track the previous state to force wakeups if the constraint targets have changed.
        private float previousTargetSpeed;
        private float previousTargetForce;

        public SimpleCarController(SimpleCar car,
            float forwardSpeed, float forwardForce, float zoomMultiplier, float backwardSpeed, float backwardForce, float idleForce, float brakeForce,
            float steeringSpeed, float maximumSteeringAngle, float wheelBaseLength, float wheelBaseWidth, float ackermanSteering)
        {
            Car = car;
            ForwardSpeed = forwardSpeed;
            ForwardForce = forwardForce;
            ZoomMultiplier = zoomMultiplier;
            BackwardSpeed = backwardSpeed;
            BackwardForce = backwardForce;
            IdleForce = idleForce;
            BrakeForce = brakeForce;
            SteeringSpeed = steeringSpeed;
            MaximumSteeringAngle = maximumSteeringAngle;
            WheelBaseLength = wheelBaseLength;
            WheelBaseWidth = wheelBaseWidth;
            AckermanSteering = ackermanSteering;

            steeringAngle = 0;
            previousTargetForce = 0;
            previousTargetSpeed = 0;
        }

        public void Update(Simulation simulation, float dt, float targetSteeringAngle, float targetSpeedFraction, bool zoom, bool brake)
        {
            var steeringAngleDifference = targetSteeringAngle - steeringAngle;
            var maximumChange = SteeringSpeed * dt;
            var steeringAngleChange = MathF.Min(maximumChange, MathF.Max(-maximumChange, steeringAngleDifference));
            var previousSteeringAngle = steeringAngle;
            
            steeringAngle = MathF.Min(MaximumSteeringAngle, MathF.Max(-MaximumSteeringAngle, steeringAngle + steeringAngleChange));
            if (steeringAngle != previousSteeringAngle)
            {
                float leftSteeringAngle;
                float rightSteeringAngle;

                float steeringAngleAbs = MathF.Abs(steeringAngle);

                if (AckermanSteering > 0 && steeringAngleAbs > 1e-6)
                {
                    float turnRadius = MathF.Abs(WheelBaseLength * MathF.Tan(MathF.PI * 0.5f - steeringAngleAbs));
                    var wheelBaseHalfWidth = WheelBaseWidth * 0.5f;
                    if (steeringAngle > 0)
                    {
                        rightSteeringAngle = MathF.Atan(WheelBaseLength / (turnRadius - wheelBaseHalfWidth));
                        rightSteeringAngle = steeringAngle + (rightSteeringAngle - steeringAngleAbs) * AckermanSteering;
                        
                        leftSteeringAngle = MathF.Atan(WheelBaseLength / (turnRadius + wheelBaseHalfWidth));
                        leftSteeringAngle = steeringAngle + (leftSteeringAngle - steeringAngleAbs) * AckermanSteering;
                    }
                    else
                    {
                        rightSteeringAngle = MathF.Atan(WheelBaseLength / (turnRadius + wheelBaseHalfWidth));
                        rightSteeringAngle = steeringAngle - (rightSteeringAngle - steeringAngleAbs) * AckermanSteering;

                        leftSteeringAngle = MathF.Atan(WheelBaseLength / (turnRadius - wheelBaseHalfWidth));
                        leftSteeringAngle = steeringAngle - (leftSteeringAngle - steeringAngleAbs) * AckermanSteering;
                    }
                }
                else
                {
                    leftSteeringAngle = steeringAngle;
                    rightSteeringAngle = steeringAngle;
                }

                //By guarding the constraint modifications behind a state test, we avoid waking up the car every single frame.
                //(We could have also used the ApplyDescriptionWithoutWaking function and then explicitly woke the car up when changes occur.)
                Car.Steer(simulation, Car.FrontLeftWheel, leftSteeringAngle);
                Car.Steer(simulation, Car.FrontRightWheel, rightSteeringAngle);
            }
            float newTargetSpeed, newTargetForce;
            bool allWheels;
            if (brake)
            {
                newTargetSpeed = 0;
                newTargetForce = BrakeForce;
                allWheels = true;
            }
            else if (targetSpeedFraction > 0)
            {
                newTargetForce = zoom ? ForwardForce * ZoomMultiplier : ForwardForce;
                newTargetSpeed = targetSpeedFraction * (zoom ? ForwardSpeed * ZoomMultiplier : ForwardSpeed);
                allWheels = false;
            }
            else if (targetSpeedFraction < 0)
            {
                newTargetForce = zoom ? BackwardForce * ZoomMultiplier : BackwardForce;
                newTargetSpeed = targetSpeedFraction * (zoom ? BackwardSpeed * ZoomMultiplier : BackwardSpeed);
                allWheels = false;
            }
            else
            {
                newTargetForce = IdleForce;
                newTargetSpeed = 0;
                allWheels = true;
            }
            if (previousTargetSpeed != newTargetSpeed || previousTargetForce != newTargetForce)
            {
                previousTargetSpeed = newTargetSpeed;
                previousTargetForce = newTargetForce;
                Car.SetSpeed(simulation, Car.FrontLeftWheel, newTargetSpeed, newTargetForce);
                Car.SetSpeed(simulation, Car.FrontRightWheel, newTargetSpeed, newTargetForce);
                if (allWheels)
                {
                    Car.SetSpeed(simulation, Car.BackLeftWheel, newTargetSpeed, newTargetForce);
                    Car.SetSpeed(simulation, Car.BackRightWheel, newTargetSpeed, newTargetForce);
                }
                else
                {
                    Car.SetSpeed(simulation, Car.BackLeftWheel, 0, 0);
                    Car.SetSpeed(simulation, Car.BackRightWheel, 0, 0);
                }
            }
        }
    }
}