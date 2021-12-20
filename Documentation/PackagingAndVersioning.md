# Packaging and Versioning

This project does not use semantic versioning. When upgrading to a newer version, expect breaking changes. 

Breaking changes should be obvious and appear as compile errors. "Sneaky" breaking changes that significantly change behavior without a compile error will be avoided if at all possible. One notable exception to this is determinism- do not expect different versions of the library to produce identical simulation results.

NuGet packages will be made available, but they will not cover all possible features. Prerelease packages are published automatically on [github](https://github.com/orgs/bepu/packages?repo_name=bepuphysics2) and [nuget](https://www.nuget.org/packages/BepuPhysics). The main branch should be kept in a relatively stable state; cloning the source is often a good choice.

The library has a variety of conditional compilation symbols. Rather than publishing a combinatorial mess to NuGet, the expectation is that users of any conditional logic will clone the source.

Given the above and the general nature of the library's API, cloning the source and referencing the project is often the best way to include the library.