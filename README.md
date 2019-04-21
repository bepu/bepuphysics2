# bepuphysics v2
<p align="center"><a href="https://www.youtube.com/watch?v=sfgC_eNx9M8" target="_blank"><img src="Documentation/images/youtubeLink.png" width="700" height="394" border="0" /></a></p>

This is the repo for the bepuphysics v2 library, a complete rewrite of the C# 3d rigid body physics engine [BEPUphysics v1](https://github.com/bepu/bepuphysics1).

The BepuPhysics and BepuUtilities libraries target .NET Standard 2.0 and should work on any supported platform. The demos are built on .NET Core 2.1 and require DX11.

The physics engine heavily uses System.Numerics.Vectors types, so to get good performance, you'll need a compiler which can consume those types (like RyuJIT).

To build the source, you'll need a recent version of Visual Studio with the .NET Core workload installed. Demos.sln references all relevant projects. For more information, see [Building](Documentation/Building.md).

## Features

- Spheres, capsules, boxes, triangles, cylinders, and convex hulls
- Compounds of the above
- Meshes
- A [whole bunch of constraint types](BepuPhysics/Constraints/)
- [Newts](Demos/Demos/NewtDemo.cs)
- Linear and angular continuous collision detection
- Extremely low cost sleep states for resting bodies
- Efficient scene-wide ray and sweep queries
- [Character controller example](Demos/Demos/CharacterDemo.cs)
- At least somewhat extensible collision pipeline, with [example custom voxel collidable](Demos/Demos/CustomVoxelCollidableDemo.cs)
- Highly nonidiomatic APIs
- Super speediness
- And a bunch of other miscellaneous stuff!

## Links

Report bugs [here on github](../../issues). 

Visit the [forums](https://forum.bepuentertainment.com) for discussion and questions.

[Building](Documentation/Building.md)

[Getting Started](Documentation/GettingStarted.md)

[Upgrading from v1, concept mapping](Documentation/UpgradingFromV1.md)

[Q&A](Documentation/QuestionsAndAnswers.md)

[Stability Tips](Documentation/StabilityTips.md)

[Performance Tips](Documentation/PerformanceTips.md)

[Packaging and Versioning](Documentation/PackagingAndVersioning.md)

Check the [roadmap](Documentation/roadmap.md) for a high level look at where things are going.

If you have too many dollars, we are willing to consume them on [patreon](https://www.patreon.com/bepu).

![](https://raw.githubusercontent.com/bepu/bepuphysics1/master/Documentation/images/readme/angelduck.png)