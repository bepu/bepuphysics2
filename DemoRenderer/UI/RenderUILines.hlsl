/*[META]
vs
ps
[META]*/
#include "ColorUnpacking.hlsl"
cbuffer VertexConstants : register(b0)
{
	float2 PackedToScreenScale;
	float2 ScreenToNDCScale;
};


struct ScreenLineInstance
{
	/// Start location. X is stored in the lower 16 bits, Y in the upper 16. Should be scaled by the PackedToScreenScale.
	uint PackedStart;
	/// End location. X is stored in the lower 16 bits, Y in the upper 16. Should be scaled by the PackedToScreenScale.
	uint PackedEnd;
	/// Radius of the line in screen pixels.
	float Radius;
	/// Color, packed in a UNORM manner such that bits 0 through 10 are R, bits 11 through 21 are G, and bits 22 through 31 are B.
	uint PackedColor;
};


StructuredBuffer<ScreenLineInstance> Instances : register(t0);

struct PSInput
{
	float4 Position : SV_Position;
	nointerpolation float2 Start : Start;
	nointerpolation float2 LineDirection : LineDirection;
	nointerpolation float LineLength : LineLength;
	nointerpolation float Radius : Radius;
	nointerpolation float3 Color : Color;
};
#define SampleRadius 0.70710678118
PSInput VSMain(uint vertexId : SV_VertexId)
{
	//The vertex id is used to position each vertex. 
	//Each quad has 4 vertices; the position is based on the 2 least significant bits.
	int quadIndex = vertexId >> 2;
	ScreenLineInstance instance = Instances[quadIndex];
	PSInput output;
	output.Start = float2(instance.PackedStart & 0xFFFF, instance.PackedStart >> 16) * PackedToScreenScale;
	float2 end = float2(instance.PackedEnd & 0xFFFF, instance.PackedEnd >> 16) * PackedToScreenScale;
	output.Radius = instance.Radius;
	output.Color = UnpackR11G11B10_UNorm(instance.PackedColor);

	float2 quadCoordinates = float2(vertexId & 1, (vertexId >> 1) & 1);
	float2 offset = end - output.Start;
	output.LineLength = length(offset);
	output.LineDirection = output.LineLength > 1e-5 ? offset / output.LineLength : float2(1,0);
	float2 verticalAxis = float2(-output.LineDirection.y, output.LineDirection.x);

	//Pad a little bit to avoid clipping.
	float geometryRadius = instance.Radius + SampleRadius;
	float radiusTimesTwo = geometryRadius * 2;
	float2 localSpan = float2(output.LineLength + radiusTimesTwo, radiusTimesTwo);
	float2 localOffset = localSpan * quadCoordinates;

	float2 screenPosition = output.Start + 
		output.LineDirection * (localOffset.x - geometryRadius) +
		verticalAxis * (localOffset.y - geometryRadius);
	//Bring the screen position into NDC for use as the SV_Position. Note the negation along Y;
	//NDC +1 is up, while in screenspace/texture space +1 is down.
	output.Position = float4(screenPosition * ScreenToNDCScale + float2(-1.0, 1.0), 0.5, 1);
	return output;
}

float4 PSMain(PSInput input) : SV_Target0
{
	//Measure the distance from the line in screen pixels. First, compute the closest point.
	float2 perp = float2(-input.LineDirection.y, input.LineDirection.x);
	float2 offset = input.Position.xy - input.Start;
	float alongLine = clamp(dot(offset, input.LineDirection), 0, input.LineLength);
	float distance = length(offset - alongLine * input.LineDirection) - input.Radius;
	//This distance is measured in screen pixels. Treat every pixel as having a set radius.
	//If the line's distance is beyond the sample radius, then there is zero coverage.
	//If the distance is 0, then the sample is half covered.
	//If the distance is less than -sampleRadius, then it's fully covered.
	float alpha = saturate(0.5 - distance / (SampleRadius * 2));
	return float4(input.Color * alpha, alpha);
}