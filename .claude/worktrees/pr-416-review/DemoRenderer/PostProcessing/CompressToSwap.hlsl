/*[META]
vs
ps
[META]*/
#include "WholeScreenTriangle.hlsl"
cbuffer Constants : register(b0)
{
	float InverseGamma;
};

Texture2D<float3> Color : register(t0);

struct PSInput
{
	float4 Position : SV_Position;
};

PSInput VSMain(uint vertexId : SV_VertexId)
{
	PSInput output;
	output.Position = float4(GetWholeScreenTriangleVertexNDC(vertexId), 0.5, 1);
	return output;
}

float4 PSMain(PSInput input) : SV_Target0
{
	const float ditherWidth = 1.0 / 255.0;
	//Compute dither amount from screen position. There are more principled ways of doing this, but shrug.
	float dither = input.Position.x * input.Position.x + input.Position.y;
	dither = asuint(dither) * 776531419;
	dither = asuint(dither) * 961748927;
	dither = asuint(dither) * 217645199;
	dither = (asuint(dither) & 65535) / 65535.0;

	//Note saturate to get rid of warning. Down the road, if you do goofy stuff like HDR, you could insert the tonemapping operator in here.
	float3 adjustedColor = pow(saturate(Color[input.Position.xy]), InverseGamma);

	adjustedColor = saturate(adjustedColor + ditherWidth * (dither - 0.5));
	return float4(adjustedColor, 1);
}