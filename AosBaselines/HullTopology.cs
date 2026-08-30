using BepuPhysics.Collidables;
using BepuUtilities;
using System.Diagnostics;
using System.Numerics;

namespace AosBaselines;

/// <summary>
/// Harness-side per-hull precompute for the relaxed-equality hull-hull experiments: scalar vertex positions, CSR
/// vertex adjacency, a unique edge list with the two adjacent face normals (gauss arcs), and the face list with
/// normals and CCW vertex loops. Also carries an optional bundle-repacked SIMD layout for horizontal single-pair SIMD.
/// Built once per hull (amortized per shape, like the engine's own preprocessing); construction validates the topology
/// (closed 2-manifold: V - E + F = 2, every edge shared by exactly 2 faces) and throws on violation.
/// </summary>
public sealed class HullTopology
{
    public int VertexCount;
    public int EdgeCount;
    public int FaceCount;

    /// <summary>Hull-local vertex positions, indexed by linear vertex index (no bundle padding phantoms).</summary>
    public Vector3[] Vertices = [];

    /// <summary>CSR starts into <see cref="AdjacentVertices"/>; length VertexCount + 1.</summary>
    public int[] VertexAdjacencyStarts = [];
    /// <summary>Concatenated neighbor-vertex lists; each undirected edge contributes one entry to each endpoint's list.</summary>
    public int[] AdjacentVertices = [];
    /// <summary>Edge index for each <see cref="AdjacentVertices"/> entry (same CSR layout): the edge connecting the
    /// owning vertex to that neighbor. Gives O(degree) enumeration of a vertex's incident edges with their gauss arcs.</summary>
    public int[] VertexEdges = [];

    public struct Edge
    {
        /// <summary>Linear indices of the endpoints; Start &lt; End (canonical order, not winding order).</summary>
        public int Start, End;
        /// <summary>Indices of the two adjacent faces. Face0 is the face that walks the edge Start-&gt;End in its CCW loop.</summary>
        public int Face0, Face1;
    }
    /// <summary>Unique (undirected) edges. The gauss arc of edge i spans FaceNormals[Face0] to FaceNormals[Face1].</summary>
    public Edge[] Edges = [];

    /// <summary>Face normals (outward, normalized; from the engine's bounding planes) indexed by face.</summary>
    public Vector3[] FaceNormals = [];
    /// <summary>Face plane offsets: dot(pointOnFace, FaceNormals[f]) = FaceOffsets[f].</summary>
    public float[] FaceOffsets = [];
    /// <summary>
    /// True support offsets per face normal: max over ALL hull vertices of dot(v, FaceNormals[f]). The engine's
    /// bounding-plane offset uses only the face's pivot vertex against the area-averaged normal, so on hulls with
    /// merged near-coplanar faces it can understate the hull's actual extent along that normal; SAT-style interval
    /// tests need the true support value.
    /// </summary>
    public float[] FaceSupportOffsets = [];
    /// <summary>CSR starts into <see cref="FaceVertices"/>; length FaceCount + 1.</summary>
    public int[] FaceStarts = [];
    /// <summary>Concatenated CCW (right-handed) vertex loops as linear vertex indices.</summary>
    public int[] FaceVertices = [];
    /// <summary>
    /// Edge indices per face, CSR-indexed by <see cref="FaceStarts"/> (each loop has exactly as many edges as vertices).
    /// Slot k of face f is the edge from loop vertex (k-1 mod n) to loop vertex k, matching the clip's edge enumeration.
    /// </summary>
    public int[] FaceEdges = [];
    /// <summary>
    /// Adjacent face per face-loop edge, CSR-indexed by <see cref="FaceStarts"/>: slot k of face f is the face on the
    /// other side of <see cref="FaceEdges"/> slot k. This is the face-adjacency graph (faces sharing an edge).
    /// </summary>
    public int[] AdjacentFaces = [];
    /// <summary>CSR starts into <see cref="VertexFaces"/>; length VertexCount + 1.</summary>
    public int[] VertexFaceStarts = [];
    /// <summary>Concatenated incident-face lists per vertex (each face-loop membership contributes one entry).</summary>
    public int[] VertexFaces = [];

    //SIMD-friendly repack: SoA arrays padded to a multiple of Vector<float>.Count by duplicating the last real entry,
    //so a Vector<float> load at any bundle start is valid and padded lanes are harmless for max/min style scans.
    /// <summary>Vertex coordinates, padded; load bundle i via new Vector&lt;float&gt;(VerticesX, i * Vector&lt;float&gt;.Count).</summary>
    public float[] VerticesX = [], VerticesY = [], VerticesZ = [];
    /// <summary>Edge direction (Vertices[End] - Vertices[Start], unnormalized), padded like the vertices.</summary>
    public float[] EdgeDirX = [], EdgeDirY = [], EdgeDirZ = [];
    /// <summary>Edge start position, padded.</summary>
    public float[] EdgeStartX = [], EdgeStartY = [], EdgeStartZ = [];
    /// <summary>Adjacent face normals per edge (gauss arc endpoints), padded.</summary>
    public float[] EdgeNormal0X = [], EdgeNormal0Y = [], EdgeNormal0Z = [];
    public float[] EdgeNormal1X = [], EdgeNormal1Y = [], EdgeNormal1Z = [];

    static int Linear(HullVertexIndex v) => (v.BundleIndex << BundleIndexing.VectorShift) + v.InnerIndex;

    static float[] Pad(int count, Func<int, float> get)
    {
        var padded = (count + Vector<float>.Count - 1) / Vector<float>.Count * Vector<float>.Count;
        var result = new float[Math.Max(padded, Vector<float>.Count)];
        for (int i = 0; i < result.Length; ++i)
            result[i] = get(Math.Min(i, count - 1));
        return result;
    }

    public static HullTopology Create(ref ConvexHull hull)
    {
        var topology = new HullTopology();
        var faceCount = hull.FaceToVertexIndicesStart.Length;
        topology.FaceCount = faceCount;

        //Vertex count isn't stored in ConvexHull; recover it from the face loops (every hull vertex belongs to a face).
        int vertexCount = 0;
        int totalLoopLength = 0;
        for (int f = 0; f < faceCount; ++f)
        {
            hull.GetVertexIndicesForFace(f, out var loop);
            totalLoopLength += loop.Length;
            for (int k = 0; k < loop.Length; ++k)
            {
                var linear = Linear(loop[k]);
                if (linear >= vertexCount)
                    vertexCount = linear + 1;
            }
        }
        topology.VertexCount = vertexCount;
        topology.Vertices = new Vector3[vertexCount];
        for (int i = 0; i < vertexCount; ++i)
            hull.GetPoint(i, out topology.Vertices[i]);

        topology.FaceNormals = new Vector3[faceCount];
        topology.FaceOffsets = new float[faceCount];
        topology.FaceStarts = new int[faceCount + 1];
        topology.FaceVertices = new int[totalLoopLength];
        topology.FaceEdges = new int[totalLoopLength];

        //Each directed edge (start->end) appears exactly once across all CCW face loops; its twin appears in exactly one other face.
        var edgeMap = new Dictionary<(int, int), int>();
        var edges = new List<Edge>();
        int nextFaceVertex = 0;
        for (int f = 0; f < faceCount; ++f)
        {
            BundleIndexing.GetBundleIndices(f, out var planeBundle, out var planeInner);
            Vector3Wide.ReadSlot(ref hull.BoundingPlanes[planeBundle].Normal, planeInner, out topology.FaceNormals[f]);
            topology.FaceOffsets[f] = hull.BoundingPlanes[planeBundle].Offset[planeInner];

            hull.GetVertexIndicesForFace(f, out var loop);
            topology.FaceStarts[f] = nextFaceVertex;
            int previous = Linear(loop[loop.Length - 1]);
            for (int k = 0; k < loop.Length; ++k)
            {
                int current = Linear(loop[k]);
                topology.FaceVertices[nextFaceVertex] = current;
                var key = previous < current ? (previous, current) : (current, previous);
                if (edgeMap.TryGetValue(key, out var edgeIndex))
                {
                    ref var edge = ref System.Runtime.InteropServices.CollectionsMarshal.AsSpan(edges)[edgeIndex];
                    if (edge.Face1 != -1)
                        throw new InvalidOperationException($"Topology violation: edge ({key.Item1},{key.Item2}) touched by more than two faces.");
                    edge.Face1 = f;
                }
                else
                {
                    edgeIndex = edges.Count;
                    edgeMap.Add(key, edgeIndex);
                    //This face walks previous->current; if previous < current this face owns the canonical direction.
                    edges.Add(new Edge { Start = key.Item1, End = key.Item2, Face0 = f, Face1 = -1 });
                }
                topology.FaceEdges[nextFaceVertex++] = edgeIndex;
                previous = current;
            }
        }
        topology.FaceStarts[faceCount] = nextFaceVertex;
        topology.Edges = edges.ToArray();
        topology.EdgeCount = topology.Edges.Length;

        //Validate: closed convex 2-manifold.
        foreach (ref var edge in topology.Edges.AsSpan())
        {
            if (edge.Face1 == -1)
                throw new InvalidOperationException($"Topology violation: edge ({edge.Start},{edge.End}) has only one adjacent face.");
        }
        if (vertexCount - topology.EdgeCount + faceCount != 2)
            throw new InvalidOperationException($"Topology violation: Euler characteristic V-E+F = {vertexCount}-{topology.EdgeCount}+{faceCount} = {vertexCount - topology.EdgeCount + faceCount}, expected 2.");

        //Normalize face order to the documented contract: Face0 is the face whose CCW loop walks Start->End, which for
        //outward CCW windings makes (End - Start) parallel to +cross(FaceNormals[Face0], FaceNormals[Face1]). The
        //insertion loop above assigned Face0 to whichever face touched the edge first, which only matches ~half the
        //time; gauss-arc consumers (Minkowski face tests) need the sign to be consistent, so fix it via the normals.
        foreach (ref var edge in topology.Edges.AsSpan())
        {
            var direction = topology.Vertices[edge.End] - topology.Vertices[edge.Start];
            var arcCross = Vector3.Cross(topology.FaceNormals[edge.Face0], topology.FaceNormals[edge.Face1]);
            if (Vector3.Dot(direction, arcCross) < 0)
                (edge.Face0, edge.Face1) = (edge.Face1, edge.Face0);
        }

        //Face adjacency across each face-loop edge (fill after all Face0/Face1 assignments are complete; the later
        //Face0/Face1 normalization swap doesn't change the {Face0, Face1} set, so ordering against it is irrelevant).
        topology.AdjacentFaces = new int[totalLoopLength];
        for (int f = 0; f < faceCount; ++f)
        {
            for (int k = topology.FaceStarts[f]; k < topology.FaceStarts[f + 1]; ++k)
            {
                ref var edge = ref topology.Edges[topology.FaceEdges[k]];
                topology.AdjacentFaces[k] = edge.Face0 == f ? edge.Face1 : edge.Face0;
            }
        }

        //CSR vertex->incident-face lists from the face loops.
        topology.VertexFaceStarts = new int[vertexCount + 1];
        for (int i = 0; i < totalLoopLength; ++i)
            ++topology.VertexFaceStarts[topology.FaceVertices[i] + 1];
        for (int i = 0; i < vertexCount; ++i)
            topology.VertexFaceStarts[i + 1] += topology.VertexFaceStarts[i];
        topology.VertexFaces = new int[totalLoopLength];
        var vertexFaceFill = new int[vertexCount];
        for (int f = 0; f < faceCount; ++f)
        {
            for (int k = topology.FaceStarts[f]; k < topology.FaceStarts[f + 1]; ++k)
            {
                var v = topology.FaceVertices[k];
                topology.VertexFaces[topology.VertexFaceStarts[v] + vertexFaceFill[v]++] = f;
            }
        }

        //CSR vertex adjacency from the unique edges (each undirected edge contributes both directions).
        topology.VertexAdjacencyStarts = new int[vertexCount + 1];
        foreach (ref var edge in topology.Edges.AsSpan())
        {
            ++topology.VertexAdjacencyStarts[edge.Start + 1];
            ++topology.VertexAdjacencyStarts[edge.End + 1];
        }
        for (int i = 0; i < vertexCount; ++i)
            topology.VertexAdjacencyStarts[i + 1] += topology.VertexAdjacencyStarts[i];
        topology.AdjacentVertices = new int[topology.EdgeCount * 2];
        topology.VertexEdges = new int[topology.EdgeCount * 2];
        var fill = new int[vertexCount];
        for (int e = 0; e < topology.Edges.Length; ++e)
        {
            ref var edge = ref topology.Edges[e];
            var startSlot = topology.VertexAdjacencyStarts[edge.Start] + fill[edge.Start]++;
            topology.AdjacentVertices[startSlot] = edge.End;
            topology.VertexEdges[startSlot] = e;
            var endSlot = topology.VertexAdjacencyStarts[edge.End] + fill[edge.End]++;
            topology.AdjacentVertices[endSlot] = edge.Start;
            topology.VertexEdges[endSlot] = e;
        }

        //True per-face support offsets (see field docs; O(F*V) once per hull, amortized like the rest of the precompute).
        topology.FaceSupportOffsets = new float[faceCount];
        for (int f = 0; f < faceCount; ++f)
        {
            var normal = topology.FaceNormals[f];
            var best = float.MinValue;
            for (int i = 0; i < vertexCount; ++i)
                best = float.MaxNative(best, Vector3.Dot(topology.Vertices[i], normal));
            topology.FaceSupportOffsets[f] = best;
        }

        //SIMD repack.
        var vertices = topology.Vertices;
        topology.VerticesX = Pad(vertexCount, i => vertices[i].X);
        topology.VerticesY = Pad(vertexCount, i => vertices[i].Y);
        topology.VerticesZ = Pad(vertexCount, i => vertices[i].Z);
        var edgeArray = topology.Edges;
        topology.EdgeDirX = Pad(topology.EdgeCount, i => vertices[edgeArray[i].End].X - vertices[edgeArray[i].Start].X);
        topology.EdgeDirY = Pad(topology.EdgeCount, i => vertices[edgeArray[i].End].Y - vertices[edgeArray[i].Start].Y);
        topology.EdgeDirZ = Pad(topology.EdgeCount, i => vertices[edgeArray[i].End].Z - vertices[edgeArray[i].Start].Z);
        topology.EdgeStartX = Pad(topology.EdgeCount, i => vertices[edgeArray[i].Start].X);
        topology.EdgeStartY = Pad(topology.EdgeCount, i => vertices[edgeArray[i].Start].Y);
        topology.EdgeStartZ = Pad(topology.EdgeCount, i => vertices[edgeArray[i].Start].Z);
        var faceNormals = topology.FaceNormals;
        topology.EdgeNormal0X = Pad(topology.EdgeCount, i => faceNormals[edgeArray[i].Face0].X);
        topology.EdgeNormal0Y = Pad(topology.EdgeCount, i => faceNormals[edgeArray[i].Face0].Y);
        topology.EdgeNormal0Z = Pad(topology.EdgeCount, i => faceNormals[edgeArray[i].Face0].Z);
        topology.EdgeNormal1X = Pad(topology.EdgeCount, i => faceNormals[edgeArray[i].Face1].X);
        topology.EdgeNormal1Y = Pad(topology.EdgeCount, i => faceNormals[edgeArray[i].Face1].Y);
        topology.EdgeNormal1Z = Pad(topology.EdgeCount, i => faceNormals[edgeArray[i].Face1].Z);
        return topology;
    }

    /// <summary>
    /// Builds topology for every hull in a set. Precompute time is measured here so benches can report it separately;
    /// it is amortized per shape and must never be included in per-pair timing.
    /// </summary>
    public static HullTopology[] CreateForSet(HullSet set, out double totalMilliseconds)
    {
        var start = Stopwatch.GetTimestamp();
        var topologies = new HullTopology[set.Hulls.Length];
        for (int i = 0; i < set.Hulls.Length; ++i)
            topologies[i] = Create(ref set.Hulls[i]);
        totalMilliseconds = (Stopwatch.GetTimestamp() - start) * 1000.0 / Stopwatch.Frequency;
        return topologies;
    }

    public static void ReportSetStatistics(string label, HullTopology[] topologies, double precomputeMilliseconds)
    {
        int minV = int.MaxValue, maxV = 0, minF = int.MaxValue, maxF = 0;
        long sumV = 0, sumE = 0, sumF = 0;
        foreach (var topology in topologies)
        {
            minV = Math.Min(minV, topology.VertexCount);
            maxV = Math.Max(maxV, topology.VertexCount);
            minF = Math.Min(minF, topology.FaceCount);
            maxF = Math.Max(maxF, topology.FaceCount);
            sumV += topology.VertexCount;
            sumE += topology.EdgeCount;
            sumF += topology.FaceCount;
        }
        Console.WriteLine($"{label}: {topologies.Length} hulls, vertices {minV}-{maxV} (mean {(double)sumV / topologies.Length:F1}), " +
            $"edges mean {(double)sumE / topologies.Length:F1}, faces {minF}-{maxF} (mean {(double)sumF / topologies.Length:F1}); " +
            $"topology checks passed; precompute {precomputeMilliseconds:F2} ms total ({precomputeMilliseconds * 1000.0 / topologies.Length:F1} us/hull).");
    }
}
