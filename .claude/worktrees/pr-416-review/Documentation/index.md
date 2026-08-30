---
_disableToc: false
---
# bepuphysics docs!

There are [conceptual](https://docs.bepuphysics.com/GettingStarted.html) *and* [API](https://docs.bepuphysics.com/api/BepuPhysics.html) docs!

See [Getting Started](GettingStarted.md) for an introduction to the library.

The BepuPhysics and BepuUtilities libraries target .NET 8 and should work on any supported platform.

The physics engine heavily uses `System.Numerics.Vectors` types, so to get good performance, you'll need a compiler which can consume those types (like RyuJIT).

The demos application, Demos.sln, uses DX11 by default. There is also a Demos.GL.sln that uses OpenGL and should run on other platforms. The demos can be run from the command line (in the repo root directory) with `dotnet run --project Demos/Demos.csproj -c Release` or `dotnet run --project Demos.GL/Demos.csproj -c Release`.

To build the source, the easiest option is a recent version of Visual Studio with the .NET desktop development workload installed. Demos.sln references all relevant projects. For more information, see [Building](Building.md).
