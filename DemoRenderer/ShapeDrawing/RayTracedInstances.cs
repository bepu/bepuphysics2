using BepuUtilities;
using DemoContentLoader;
using SharpDX.Direct3D11;
using System;
using System.Numerics;
using System.Runtime.InteropServices;
using Quaternion = BepuUtilities.Quaternion;

namespace DemoRenderer.ShapeDrawing
{
    /// <summary>
    /// GPU-relevant information for the rendering of a single sphere instance.
    /// </summary>
    public struct SphereInstance
    {
        public Vector3 Position;
        public float Radius;
        public Vector3 PackedOrientation;
        public uint PackedColor;
    }
    /// <summary>
    /// GPU-relevant information for the rendering of a single capsule instance.
    /// </summary>
    public struct CapsuleInstance
    {
        public Vector3 Position;
        public float Radius;
        public ulong PackedOrientation;
        public float HalfLength;
        public uint PackedColor;
    }
    /// <summary>
    /// GPU-relevant information for the rendering of a single cylinder instance.
    /// </summary>
    public struct CylinderInstance
    {
        public Vector3 Position;
        public float Radius;
        public ulong PackedOrientation;
        public float HalfLength;
        public uint PackedColor;
    }

}
