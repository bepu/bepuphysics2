/*[META]
vs
ps
[META]*/
#include "RasterizedCommon.hlsl"

struct Instance
{
	float3 Position;
	uint PackedColor;
	float4 Orientation;
	float3 HalfExtents;
	float Padding0;
};

StructuredBuffer<Instance> Instances : register(t0);

PSInput VSMain(uint vertexId : SV_VertexId)
{
	//The vertex id is used to position each vertex. 
	//Each AABB has 8 vertices; the position is based on the 3 least significant bits.
	int instanceId = vertexId >> 3;
	PSInput output;
	Instance instance = Instances[instanceId];
	//Note that we move the instance location into camera local translation.
	output.InstancePosition = instance.Position - CameraPosition;
	output.PackedColor = instance.PackedColor;
	output.Orientation = instance.Orientation;

	//Convert the vertex id to local AABB coordinates, and then into view space.
	//Note that this id->coordinate transformation requires consistency with the index buffer
	//to ensure proper triangle winding. A set bit in a given position makes it higher along the axis.
	//So vertexId&1 == 1 => +x, vertexId&2 == 2 => +y, and vertexId&4 == 4 => +z.
	float3 aabbCoordinates = float3((vertexId & 1) << 1, vertexId & 2, (vertexId & 4) >> 1) - 1;
	float3x3 orientation = ConvertToRotationMatrix(output.Orientation);

	float3 vertexPosition = output.InstancePosition + mul(instance.HalfExtents * aabbCoordinates, orientation);
	float3 vertexViewPosition = mul(float3x3(CameraRight, CameraUp, CameraBackward), vertexPosition);

	output.ToAABB = vertexPosition;
	output.Position = mul(float4(vertexViewPosition, 1), Projection);
	return output;
}

float3 PSMain(PSInput input) : SV_Target0
{
	float3 dpdx = ddx(input.ToAABB);
	float3 dpdy = ddy(input.ToAABB);
	float3 normal = normalize(cross(dpdy, dpdx));
	return ShadeSurface(
		input.ToAABB, normal, UnpackR11G11B10_UNorm(input.PackedColor), dpdx, dpdy,
		input.InstancePosition, input.Orientation);
}