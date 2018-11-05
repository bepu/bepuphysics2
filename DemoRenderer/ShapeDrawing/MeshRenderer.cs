using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using DemoContentLoader;
using SharpDX.Direct3D11;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Quaternion = BepuUtilities.Quaternion;

namespace DemoRenderer.ShapeDrawing
{
    [StructLayout(LayoutKind.Explicit, Size = 48)]
    public struct MeshInstance
    {
        [FieldOffset(0)]
        public Vector3 Position;
        [FieldOffset(12)]
        public uint PackedColor;
        [FieldOffset(16)]
        public ulong PackedOrientation;
        [FieldOffset(24)]
        public int VertexStart;
        [FieldOffset(28)]
        public int VertexCount;
        [FieldOffset(32)]
        public Vector3 Scale;
    }
    public class MeshRenderer : IDisposable
    {
        MeshCache meshCache;

        ConstantsBuffer<RasterizedVertexConstants> vertexConstants;

        StructuredBuffer<MeshInstance> instances;

        VertexShader vertexShader;
        PixelShader pixelShader;

        public MeshRenderer(Device device, MeshCache meshCache, ShaderCache cache, int maximumInstancesPerDraw = 2048)
        {
            this.meshCache = meshCache;
            instances = new StructuredBuffer<MeshInstance>(device, maximumInstancesPerDraw, $"Mesh Instances");

            vertexConstants = new ConstantsBuffer<RasterizedVertexConstants>(device, debugName: $"Mesh Renderer Vertex Constants");

            vertexShader = new VertexShader(device, cache.GetShader(@"ShapeDrawing\RenderMeshes.hlsl.vshader"));
            pixelShader = new PixelShader(device, cache.GetShader(@"ShapeDrawing\RenderMeshes.hlsl.pshader"));
        }

        public unsafe void Render(DeviceContext context, Camera camera, Int2 screenResolution, Span<MeshInstance> instances, int start, int count)
        {
            //Examine the set of instances and batch them into groups using the same mesh data.
            var batches = new QuickDictionary<ulong, QuickList<MeshInstance>, PrimitiveComparer<ulong>>(16, meshCache.Pool);
            var end = start + count;
            for (int i = start; i < end; ++i)
            {
                ref var instance = ref instances[i];
                ref var id = ref Unsafe.As<int, ulong>(ref instance.VertexStart);

                if (batches.GetTableIndices(ref id, out var tableIndex, out var elementIndex))
                {
                    //The id was already present.
                    batches.Values[elementIndex].Add(instance, meshCache.Pool);
                }
                else
                {
                    //There is no batch for this vertex region, so create one.
                    var newCount = batches.Count + 1;
                    if (newCount > batches.Keys.Length)
                    {
                        batches.Resize(newCount, meshCache.Pool);
                        //Resizing will change the table indices, so we have to grab it again.
                        batches.GetTableIndices(ref id, out tableIndex, out _);
                    }
                    batches.Keys[batches.Count] = id;
                    ref var listSlot = ref batches.Values[batches.Count];
                    listSlot = new QuickList<MeshInstance>(64, meshCache.Pool);
                    listSlot.Add(instance, meshCache.Pool);
                    batches.Table[tableIndex] = newCount;
                    batches.Count = newCount;
                }
            }

            var vertexConstantsData = new RasterizedVertexConstants
            {
                Projection = Matrix.Transpose(camera.Projection), //compensate for the shader packing.
                CameraPosition = camera.Position,
                CameraRight = camera.Right,
                CameraUp = camera.Up,
                CameraBackward = camera.Backward,
            };
            vertexConstants.Update(context, ref vertexConstantsData);

            //This assumes that render states have been set appropriately for opaque rendering.
            context.InputAssembler.InputLayout = null;
            context.InputAssembler.PrimitiveTopology = SharpDX.Direct3D.PrimitiveTopology.TriangleList;
            context.VertexShader.Set(vertexShader);
            context.VertexShader.SetConstantBuffer(0, vertexConstants.Buffer);
            context.VertexShader.SetShaderResource(0, this.instances.SRV);
            context.VertexShader.SetShaderResource(1, meshCache.TriangleBuffer.SRV);
            context.PixelShader.Set(pixelShader);

            for (int i = 0; i < batches.Count; ++i)
            {
                ref var batch = ref batches.Values[i];
                var batchVertexCount = batch[0].VertexCount;
                while (batch.Count > 0)
                {
                    var subbatchStart = Math.Max(0, batch.Count - this.instances.Capacity);
                    var subbatchCount = batch.Count - subbatchStart;
                    this.instances.Update(context, new Span<MeshInstance>((MeshInstance*)batch.Span.Memory + subbatchStart, subbatchCount));
                    context.DrawInstanced(batchVertexCount, subbatchCount, 0, 0);
                    batch.Count -= subbatchCount;
                }
                batch.Dispose(meshCache.Pool);
            }
            batches.Dispose(meshCache.Pool);

        }

        bool disposed;
        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
                vertexShader.Dispose();
                pixelShader.Dispose();
                instances.Dispose();
                vertexConstants.Dispose();
            }
        }

#if DEBUG
        ~MeshRenderer()
        {
            Helpers.CheckForUndisposed(disposed, this);
        }
#endif
    }
}
