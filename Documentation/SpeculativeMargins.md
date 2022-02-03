# What's a speculative margin?
The maximum distance at which collision pairs generate speculative contacts.

Speculative contacts are contacts with negative depth. They're still solved, but they don't push back against motion unless the velocity is high enough that the involved collidables are expected to come into contact within the next frame.

Here's a picture!

The ball is heading towards the ground with a high enough velocity that the velocity expanded bounding box intersects the ground's bounding box. Similarly, since the collidables are configured to have no maximum speculative margin in this example, a speculative contact is created. The solver will detect and push back the part of velocity which would result in penetration. In the next frame, the ball and ground are in contact.

This is a form of continuous collision detection in the sense that it can avoid bodies tunneling through each other.

# Do I need to care about speculative margins?
Most of the time, you don't. Consider a body or static created by just specifying the collision shape like so:
```cs
var dynamicBoxShape = new Box(1, 1, 1);
Simulation.Bodies.Add(BodyDescription.CreateDynamic(
    new Vector3(10, 5, 0), dynamicBoxShape.ComputeInertia(1), Simulation.Shapes.Add(dynamicBoxShape), 0.01f));
Simulation.Statics.Add(new StaticDescription(new Vector3(0, -0.5f, 0), Simulation.Shapes.Add(new Box(2500, 1, 2500))));
```
This creates `CollidableDescription` from the `TypedIndex` returned by `Simulation.Shapes.Add`. When no other information is specified, a `CollidableDescription` defaults to a `ContinuousDetection` mode of `ContinuousDetection.Passive`. More details about what that means will come later, but the short version is that:
1. The bounding box is expanded by the whole velocity of the body, if the collidable is associated with a body.
2. The maximum speculative margin is `float.MaxValue`. In other words, there's no upper limit.
3. No sweep tests are used. Contacts are simply created from closest features.

Taken together, this makes most stuff just work. Performance stays high since speculative contacts only get created if the velocity is high enough to warrant them, and high velocity collisions tend to have robust behavior since speculative contacts get generated.

For most use cases, sticking with the default is a high performance and high quality option.

# What are ghost collisions?

# Do speculative margins have any other surprising side effects?
Yes! Speculative contacts are mostly incompatible with the traditional approach to bounciness- a coefficient of restitution which sets an opposing velocity goal along a contact normal proportional to the incoming velocity. That's why you won't find a 0 to 1 `CoefficientOfRestitution` anywhere in the library.

Instead, all contacts are springs. In `INarrowPhaseCallbacks.ConfigureContactManifold` you can customize a pair's  `PairMaterialProperties` which include a `SpringSettings` and `MaximumRecoveryVelocity`. Using a sufficiently high `MaximumRecoveryVelocity` and reducing the `SpringSettings.DampingRatio` to 0 will minimize the amount of energy damped out during a bounce. There is a bit complexity here- the `Frequency` must be low enough that the simulation can actually represent it. If the contact is trying to make a bounce happen at 240hz, but the integrator timestep is only 60hz, the unrepresentable motion will get damped out and the body won't bounce as much.

For more information, see the [BouncinessDemo](../Demos/Demos/BouncinessDemo.cs).

