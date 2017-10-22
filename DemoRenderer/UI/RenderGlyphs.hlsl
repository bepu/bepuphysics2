/*[META]
vs
ps
[META]*/
#include "ColorUnpacking.hlsl"
cbuffer VertexConstants : register(b0)
{
	float2 PackedToScreenScale;
	float2 ScreenToNDCScale;
	float2 InverseAtlasResolution;
};


struct GlyphInstance
{
	//Packed location of the minimum corner of the glyph. Lower 16 bits is X, upper 16 bits is Y. Should be scaled by PackedToScreenScale.
	uint PackedMinimum;
	/// Packed horizontal axis used by the glyph. Lower 16 bits is X, upper 16 bits is Y. UNORM packed across a range from -1.0 at 0 to 1.0 at 65534.
	uint PackedHorizontalAxis;
	//Scale to apply to the source glyph. UNORM packed across a range of 0.0 at 0 to 16.0 at 65535.
	//Index of the source character description in the description set.
	//Lower 16 bits contain scale, upper 16 bits contain source id.
	uint PackedScaleAndSourceId;
	//Color, packed in a UNORM manner such that bits 0 through 10 are R, bits 11 through 21 are G, and bits 22 through 31 are B.
	uint PackedColor;
};

struct GlyphSource
{
	float2 Minimum; //In texels, but offset such that 0,0 would be at UV 0,0. Texel corners, not centers.
	uint PackedSpan; //Lower 16 bits X, upper 16 bits Y. In texels.
	float DistanceScale;
};

StructuredBuffer<GlyphInstance> Instances : register(t0);
StructuredBuffer<GlyphSource> Sources : register(t1);

struct PSInput
{
	float4 Position : SV_Position;
	float2 AtlasUV : TextureCoordinates;
	nointerpolation float DistanceScale : DistanceScale;
	nointerpolation float3 Color : Color; //Could leave this packed; might be faster than passing it up. Shrug.
};

#define SampleRadius 0.5

PSInput VSMain(uint vertexId : SV_VertexId)
{
	//The vertex id is used to position each vertex. 
	//Each quad has 4 vertices; the position is based on the 2 least significant bits.
	int quadIndex = vertexId >> 2;
	GlyphInstance instance = Instances[quadIndex];
	float2 minimum = float2(instance.PackedMinimum & 0xFFFF, instance.PackedMinimum >> 16) * PackedToScreenScale;
	float2 horizontalAxis = float2(
		(instance.PackedHorizontalAxis & 0xFFFF) - 32767.0,
		(instance.PackedHorizontalAxis >> 16) - 32767.0) / 32767.0; //Note input range up to 65534.
	float instanceScale = (instance.PackedScaleAndSourceId & 0xFFFF) * (16.0 / 65535.0);
	uint sourceId = instance.PackedScaleAndSourceId >> 16;
	//Note that we find the source minimum/span by a second indirection.
	//The assumption here is that the glyph source data will be in the cache and hit over and over,
	//while the instances are just used once.
	//This could make sense if you're rendering a book or something, but for very low character counts,
	//this is just added overhead. For the purposes of the demo, eh, whatever, it'll be fine.
	GlyphSource source = Sources[sourceId];
	float2 span = float2(source.PackedSpan & 0xFFFF, source.PackedSpan >> 16);
	//Including a little bit of padding on the quad's (with consistent UVs) helps avoid geometry clipping
	//which would otherwise cause visible aliasing.
	const float screenPadding = SampleRadius;
	const float atlasPadding = screenPadding / instanceScale;
	float2 paddedSpan = span + 2 * atlasPadding;


	PSInput output;
	float2 quadCoordinates = float2(vertexId & 1, (vertexId >> 1) & 1);
	float2 localOffset = instanceScale * paddedSpan * quadCoordinates - screenPadding;
	float2 verticalAxis = float2(-horizontalAxis.y, horizontalAxis.x);
	float2 screenPosition = minimum +
		localOffset.x * horizontalAxis + localOffset.y * verticalAxis;
	//Bring the screen position into NDC for use as the SV_Position. Note the negation along Y;
	//NDC +1 is up, while in screenspace/texture space +1 is down.
	output.Position = float4(
		screenPosition * ScreenToNDCScale + float2(-1.0, 1.0), 0.5, 1);
	output.AtlasUV = (source.Minimum - atlasPadding + paddedSpan * quadCoordinates) * InverseAtlasResolution;
	output.DistanceScale = source.DistanceScale * instanceScale;
	output.Color = UnpackR11G11B10_UNorm(instance.PackedColor);
	return output;
}

SamplerState Sampler : register(s0);
Texture2D<snorm float> Atlas : register(t0);

float4 PSMain(PSInput input) : SV_Target0
{
	float screenDistance = Atlas.Sample(Sampler, input.AtlasUV) * input.DistanceScale;
	//This distance is measured in screen pixels. Treat every pixel as having a set radius.
	//If the glyph's distance is beyond the sample radius, then there is zero coverage.
	//If the distance is 0, then the sample is half covered.
	//If the distance is less than -sampleRadius, then it's fully covered.
	float alpha = saturate(0.5 - screenDistance / (SampleRadius * 2));
	return float4(input.Color * alpha, alpha);
}