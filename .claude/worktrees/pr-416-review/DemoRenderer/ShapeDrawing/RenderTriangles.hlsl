/*[META]
vs
ps
[META]*/
#include "RasterizedCommon.hlsl"

struct Instance
{
	float3 A;
	uint PackedColor;
	float3 B;
	float X;
	float3 C;
	float Y;
	uint2 PackedOrientation;
	float Z;
	float Padding;
};

StructuredBuffer<Instance> Instances : register(t0);

PSInput VSMain(uint vertexId : SV_VertexId)
{
	int instanceId = vertexId / 3;
	int triangleVertexId = vertexId - instanceId * 3;
	PSInput output;
	Instance instance = Instances[instanceId];
	float3 instancePosition = float3(instance.X, instance.Y, instance.Z);
	//Note that we move the instance location into camera local translation.
	output.InstancePosition = instancePosition - CameraPosition;
	output.PackedColor = instance.PackedColor;
	output.Orientation = UnpackOrientation(instance.PackedOrientation);

	float3x3 orientation = ConvertToRotationMatrix(output.Orientation);

	//Note winding swap. Just for consistency with the culling mode.
	float3 localVertex = triangleVertexId == 0 ? instance.A : triangleVertexId == 1 ? instance.C : instance.B;

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