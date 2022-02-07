# Building

## Library

The easiest way to build the library is using the latest version of Visual Studio with the .NET desktop development workload installed to open and build the `Library.sln`.

The library tends to use the latest C# language features. At the time of writing, it requires C# 9.0.

`BepuPhysics.csproj` uses T4 templates for code generation in a few places. If changes are made to the templates, you'll need a build pipeline that can process them (like Visual Studio). The repository contains the original generated .cs files, so if no changes are made, the templates do not need to be evaluated.

The libraries target .NET 6.

## Demos

`Demos.sln` contains all the projects necessary to build and run the demos application. The default demo renderer uses DX11, and the content pipeline's shader compiler requires the Windows SDK. There is also a Demos.GL.sln that uses OpenGL and should run on other platforms. The demos can be run from the command line (in the repo root directory) with `dotnet run --project Demos/Demos.csproj -c Release` or `dotnet run --project Demos.GL/Demos.csproj -c Release`.

The demos applications target .NET 6.

## Build Configurations

The library has the usual `Debug` and `Release` build configurations, but also has `ReleaseNoProfiling`.

`ReleaseNoProfiling` removes the library's profiling functionality; any attempt to request profiling data will just return zeroes.

For production use, `Release` or `ReleaseNoProfiling` is recommended.

## Compilation Symbols

Profiling features are enabled by including the `PROFILE` compilation symbol for BepuPhysics.csproj.

Some extra checks for data validity can be enabled with the `CHECKMATH` compilation symbol for BepuPhysics.csproj and BepuUtilities.csproj.

`LEAKDEBUG` forces the `BufferPool` to track extra data about allocations. It comes with extremely high overhead, but can be useful for narrowing down the source of memory leaks and other similar bugs. `LEAKDEBUG` will only work if `DEBUG` is also defined.

## Runtime

The library makes heavy use of SIMD intrinsics through `System.Numerics.Vectors` and `System.Runtime.Intrinsics`. Good performance requires a IL to native assembly compiler which is aware of these intrinsics. Right now, that means something like CoreCLR's RyuJIT. Other runtimes may not support the intrinsics and may suffer massive slowdowns- sometimes 10 to 100 times slower, if they run at all.

Performance scales up with higher SIMD machine widths. Machines with full rate AVX2 will tend to significantly outperform SSE-limited machines.

