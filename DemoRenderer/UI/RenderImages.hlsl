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


struct ImageInstance
{
	//Packed location of the minimum corner of the glyph. Lower 16 bits is X, upper 16 bits is Y. Should be scaled by PackedToScreenScale.
	uint PackedMinimum;
	//Packed horizontal axis used by the glyph. Lower 16 bits is X, upper 16 bits is Y. UNORM packed across a range from -1.0 at 0 to 1.0 at 65534.
	uint PackedHorizontalAxis;
	//Packed width and height. Width is in the lower 16 bits, height is in the upper 16 bits.
	uint PackedSize;
	//RGBA color, packed in a UNORM manner such that bits 0 through 7 are R, bits 8 through 15 are G, bits 16 through 23 are B, and bits 24 through 31 are A.
	uint PackedColor;
};

StructuredBuffer<ImageInstance> Instances : register(t0);

struct PSInput
{
	float4 Position : SV_Position;
	float2 UV : TextureCoordinates;
	nointerpolation float4 Color : Color; //Could leave this packed; might be faster than passing it up. Shrug.
};

PSInput VSMain(uint vertexId : SV_VertexId)
{
	//The vertex id is used to position each vertex. 
	//Each quad has 4 vertices; the position is based on the 2 least significant bits.
	int quadIndex = vertexId >> 2;
	ImageInstance instance = Instances[quadIndex];
	float2 minimum = float2(instance.PackedMinimum & 0xFFFF, instance.PackedMinimum >> 16) * PackedToScreenScale;
	float2 horizontalAxis = float2(
		(instance.PackedHorizontalAxis & 0xFFFF) - 32767.0,
		(instance.PackedHorizontalAxis >> 16) - 32767.0) / 32767.0; //Note input range up to 65534.
	float2 instanceSize = float2(
		(instance.PackedSize & 0xFFFF) * (4096.0 / 65535.0),
		(instance.PackedSize >> 16) * (4096.0 / 65535.0));

	PSInput output;
	float2 quadCoordinates = float2(vertexId & 1, (vertexId >> 1) & 1);
	float2 localOffset = instanceSize * quadCoordinates;
	float2 verticalAxis = float2(-horizontalAxis.y, horizontalAxis.x);
	float2 screenPosition = minimum +
		localOffset.x * horizontalAxis + localOffset.y * verticalAxis;
	//Bring the screen position into NDC for use as the SV_Position. Note the negation along Y;
	//NDC +1 is up, while in screenspace/texture space +1 is down.
	output.Position = float4(
		screenPosition * ScreenToNDCScale + float2(-1.0, 1.0), 0.5, 1);
	output.UV = quadCoordinates;
	output.Color = UnpackR8G8B8A8_UNorm(instance.PackedColor);
	return output;
}

SamplerState Sampler : register(s0);
Texture2D<unorm float4> Source : register(t0);

float4 PSMain(PSInput input) : SV_Target0
{
	float4 tintedSample = Source.Sample(Sampler, input.UV) * input.Color;
	return float4(tintedSample.xyz * tintedSample.w, tintedSample.w);
}