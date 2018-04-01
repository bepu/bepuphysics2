# bepuphysics v2

This is the repo for the bepuphysics v2 library, a complete rewrite of the C# 3d rigid body physics engine [BEPUphysics v1](https://github.com/bepu/bepuphysics1).

The library is still in alpha. Many important features are missing, and many bugs likely remain.

The BepuPhysics and BepuUtilities libraries target .NET Standard 2.0 and should work on any supported platform. The demos are built on .NET Core 2.0 and require DX11.

The physics engine heavily uses System.Numerics.Vectors types, so to get good performance, you'll need a compiler which can consume those types (like RyuJIT).

To build the source, you'll need a recent version of Visual Studio with the .NET Core workload installed. Demos.sln references all relevant projects.

## Features

The alpha contains a basic featureset:
- Spheres, capsules, boxes
- Simple compounds of the above, mainly for use in smaller shapes where acceleration structures wouldn't be helpful
- Highly nonidiomatic APIs
- Automatic sleeping/waking management
- Ball socket, hinge, swivel hinge, and swing limit
- Speediness

It notably does *not* yet include:
- Convex hulls, cylinders, cones, or other complex shapes 
- Efficient compounds supporting many children
- Meshes
- Bounciness, other than the frequency/damping ratio tuning
- Continuous collision detection, other than the speculative margin
- Scene-wide queries like ray casts or volume queries
- Shape casts
- Many useful constraint types

## Links

If you are one of the brave early adopters, you can report bugs [here on github](../../issues). 

Visit the [forums](https://forum.bepuentertainment.com) for discussion and questions.

Check the [roadmap](Documentation/roadmap.md) for a high level look at where things are going.

There exists some sparse proto-documentation:

[Q&A](Documentation/QuestionsAndAnswers.md)

[Stability Tips](Documentation/StabilityTips.md)

[Performance Tips](Documentation/PerformanceTips.md)

[Packaging and Versioning](Documentation/PackagingAndVersioning.md)

If you have too many dollars, we are willing to consume them on [patreon](https://www.patreon.com/bepu).

![](https://raw.githubusercontent.com/bepu/bepuphysics1/master/Documentation/images/readme/angelduck.png)