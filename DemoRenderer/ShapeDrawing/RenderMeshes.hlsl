/*[META]
vs
ps
[META]*/
#include "RasterizedCommon.hlsl"

struct Instance
{
	float3 Position;
	uint PackedColor;
	uint2 PackedOrientation;
	uint VertexStart;
	uint VertexCount;
	float3 Scale;
	float Padding;
};

StructuredBuffer<Instance> Instances : register(t0);
StructuredBuffer<float3> Vertices : register(t1);

PSInput VSMain(uint vertexId : SV_VertexId, uint instanceId : SV_InstanceId)
{
	Instance instance = Instances[instanceId];
	float3 localVertex = instance.Scale * Vertices[vertexId + instance.VertexStart];
	PSInput output;
	//Note that we move the instance location into camera local translation.
	output.InstancePosition = instance.Position - CameraPosition;
	output.PackedColor = instance.PackedColor;
	output.Orientation = UnpackOrientation(instance.PackedOrientation);
	
	float3x3 orientation = ConvertToRotationMatrix(output.Orientation);

	float3 vertexPosition = output.InstancePosition + mul(localVertex, orientation);
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