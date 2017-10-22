/*[META]
vs
ps
[META]*/
#include "WholeScreenTriangle.hlsl"
cbuffer Constants : register(b0)
{
	float4x4 NDCToOffset;
};

struct PSInput
{
	float4 Position : SV_Position;
	float3 Offset : Offset;
};

PSInput VSMain(uint vertexId : SV_VertexId)
{
	PSInput output;
	//This is drawn with depth testing enabled. Don't want to be far clipped. Note reversed depth.
	output.Position = float4(GetWholeScreenTriangleVertexNDC(vertexId), 0.0000001, 1);
	float4 unprojected = mul(output.Position, NDCToOffset);
	output.Offset = unprojected.xyz / unprojected.w;
	return output;
}

float3 PSMain(PSInput input) : SV_Target0
{
	float3 direction = normalize(input.Offset);
	//We'll want to cache and reuse this direction and lighting values in the other object shaders once they have actual lighting. Just pull it into some common include.
	float3 toSunDirection = normalize(float3(0.37, 0.93, 0.3));
	float sunDot = dot(direction, toSunDirection);
	float sunContribution = sunDot * sunDot;
	sunContribution = sunContribution * sunContribution;
	sunContribution = sunContribution * sunContribution;
	return 0.125 + 
		saturate(direction.y) * float3(0.128, 0.283, .855) +
		sunContribution * (sunDot >= 0 ? 0.5 : 0.2);
}