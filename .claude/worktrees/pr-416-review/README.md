# bepuphysics v2
<p align="center">
<a href="https://www.youtube.com/watch?v=sfgC_eNx9M8" target="_blank"><img src="Documentation/images/youtubeLink.png" width="375" height="211" border="0" /></a>
<a href="https://www.youtube.com/watch?v=tjtwSq3u6Dg" target="_blank"><img src="Documentation/images/youtubeLink24.png" width="375" height="211" border="0" /></a></p>

This is the repo for the bepuphysics v2 library, a complete rewrite of the C# 3d rigid body physics engine [BEPUphysics v1](https://github.com/bepu/bepuphysics1).

The BepuPhysics and BepuUtilities libraries target .NET 8 and should work on any supported platform. The demos application, Demos.sln, uses DX11 by default. There is also a Demos.GL.sln that uses OpenGL and should run on other platforms. The demos can be run from the command line (in the repo root directory) with `dotnet run --project Demos/Demos.csproj -c Release` or `dotnet run --project Demos.GL/Demos.csproj -c Release`.

The physics engine heavily uses `System.Numerics.Vectors` types, so to get good performance, you'll need a compiler which can consume those types (like RyuJIT).

To build the source, the easiest option is a recent version of Visual Studio with the .NET desktop development workload installed. Demos.sln references all relevant projects. For more information, see [Building](Documentation/Building.md).

## Features

- Spheres, capsules, boxes, triangles, cylinders, and convex hulls
- Compounds of the above
- Meshes
- A [whole bunch of constraint types](BepuPhysics/Constraints/)
- [Newts](Demos/Demos/NewtDemo.cs)
- Linear and angular continuous collision detection
- Extremely low cost sleep states for resting bodies
- Efficient scene-wide ray and sweep queries
- [Character controller example](Demos/Demos/Characters/CharacterDemo.cs)
- At least somewhat extensible collision pipeline, with [example custom voxel collidable](Demos/Demos/CustomVoxelCollidableDemo.cs)
- Highly nonidiomatic APIs
- Super speediness
- And a bunch of other miscellaneous stuff!

## Links

Report bugs [on the issues tab](../../issues). 

Use the [discussions tab](../../discussions) for... discussions. And questions.

There's a [discord server](https://discord.gg/ssa2XpY). I'll be focusing on github for long-form content, but if you like discord, you can discord. 

[Documentation pages](https://docs.bepuphysics.com/) in a conventional form factor exist! (If I've broken the docs page, see the [raw repo versions](https://github.com/bepu/bepuphysics2/tree/master/Documentation) as a backup or [github pages](https://bepu.github.io/bepuphysics2/) if I just broke the domain redirect.) 

If you have too many dollars, I'm willing to consume them through [github sponsors](https://www.github.com/sponsors/RossNordby). Please do not give me any amount of money that feels even slightly painful. Development is not conditional on sponsorships, and I already have a goodly number of dollars.

![](https://raw.githubusercontent.com/bepu/bepuphysics1/master/Documentation/images/readme/angelduck.png)
