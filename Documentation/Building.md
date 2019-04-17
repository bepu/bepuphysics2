# Building

## Library

The easiest way to build the library is using the latest version of Visual Studio with the .NET Core workload installed to open and build the `Library.sln`.

The library tends to use the latest C# language features. At the time of writing, it requires C# 7.3. 8.0 will likely be adopted soon after its stable release.

`BepuPhysics.csproj` uses T4 templates for code generation in a few places. If changes are made to the templates, you'll need a build pipeline that can process them (like Visual Studio). The repository contains the original generated .cs files, so if no changes are made, the templates do not need to be evaluated.

The libraries target .NET Standard 2.0. They currently do some [unusual/questionable things](https://github.com/bepu/bepuphysics2/issues/59) that make building the library directly targeting .NET Core difficult. Sorry. (Note that .NET Core applications can still consume the library- the library itself just has to be built for .NET Standard 2.0.)

## Demos

`Demos.sln` contains all the projects necessary to build and run the demos application. The demo renderer uses DX11, and the content pipeline's shader compiler requires the Windows SDK. The demos application targets .NET Core 2.1.

## Build Configurations

The library has the usual `Debug` and `Release` build configurations, but also has `ReleaseStrip` and `ReleaseStripNoProfiling`.

Both 'strip' variants remove memory initialization flags from methods in the library. This can save noticeable time at runtime due to the library's heavy reliance on stack allocation.

`ReleaseStripNoProfiling` removes the library's profiling functionality; any attempt to request profiling data will just return zeroes.

For production use, `ReleaseStrip` or `ReleaseStripNoProfiling` is recommended.

## Compilation Symbols

Profiling features are enabled by including the `PROFILE` compilation symbol for BepuPhysics.csproj.

Some extra checks for data validity can be enabled with the `CHECKMATH` compilation symbol for BepuPhysics.csproj and BepuUtilities.csproj.

`LEAKDEBUG` forces the `BufferPool` to track extra data about allocations. It comes with extremely high overhead, but can be useful for narrowing down the source of memory leaks and other similar bugs. `LEAKDEBUG` will only work if `DEBUG` is also defined.

## Runtime

The library makes heavy use of SIMD intrinsics through `System.Numerics.Vectors`. Good performance requires a IL to native assembly compiler which is aware of these intrinsics. Right now, that means something like CoreCLR's RyuJIT. Other runtimes may not support the intrinsics and may suffer massive slowdowns- sometimes 10 to 100 times slower, if they run at all.

Performance scales up with higher SIMD machine widths. Machines with full rate AVX2 will tend to significantly outperform SSE-limited machines.

