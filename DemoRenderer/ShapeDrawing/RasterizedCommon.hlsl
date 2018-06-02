#include "Common.hlsl"
cbuffer VertexConstants : register(b0)
{
	float4x4 Projection;
	float3 CameraPosition;
	float Padding0;
	float3 CameraRight;
	float Padding1;
	float3 CameraUp;
	float Padding2;
	float3 CameraBackward;
};
struct PSInput
{
	float4 Position : SV_Position;
	float3 ToAABB : RayDirection;
	nointerpolation float3 InstancePosition : Position;
	nointerpolation uint PackedColor : PackedColor;
	nointerpolation float4 Orientation : Orientation;
};
