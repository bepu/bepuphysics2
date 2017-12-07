/*[META]
vs
ps
[META]*/
cbuffer VertexConstants : register(b0)
{
	float4x4 ViewProjection;
	float2 NDCToScreenScale; //0 to 2 => 0 to resolution
	float2 Padding0;
	float3 CameraForward;
	float TanAnglePerPixel;
	float3 CameraRight;
	float Padding1;
	float3 CameraPosition;
};

#include "ColorUnpacking.hlsl"
struct LineInstance
{
	float3 Start;
	uint PackedBackgroundColor;
	float3 End;
	uint PackedColor;
};

StructuredBuffer<LineInstance> Instances : register(t0);

struct PSInput
{
	float4 Position : SV_Position;
	float3 ToBox : ToBoxOffset;
	nointerpolation float3 Start : LineStart;
	nointerpolation float3 Direction : LineDirection;
	nointerpolation float Length : LineLength;
	nointerpolation float3 Color : ScreenLineColor0;
	nointerpolation float3 BackgroundColor : ScreenLineColor1;
	float PixelSize : WorldPixelSize;
};
#define InnerRadius 0.5
#define OuterRadius 0.75
#define SampleRadius 0.70710678118
PSInput VSMain(uint vertexId : SV_VertexId)
{
	int instanceId = vertexId >> 3;
	LineInstance instance = Instances[instanceId];
	PSInput output;

	//Project the start and end points into NDC.
	//Output the line start, direction, and length in screenspace for use by the pixel shader. This is similar to the UI line renderer.
	float3 worldLine = instance.End - instance.Start;
	float worldLineLength = length(worldLine);
	worldLine = worldLineLength > 1e-7 ? worldLine / worldLineLength : float3(1, 0, 0);
	output.Start = instance.Start - CameraPosition;
	output.Direction = worldLine;
	output.Length = worldLineLength;
	output.Color = UnpackR11G11B10_UNorm(instance.PackedColor);
	output.BackgroundColor = UnpackR11G11B10_UNorm(instance.PackedBackgroundColor);
	//How wide is a pixel at this vertex, approximately?
	//Convert the vertex id to local box coordinates.
	//Note that this id->coordinate transformation requires consistency with the index buffer
	//to ensure proper triangle winding. A set bit in a given position makes it higher along the axis.
	//So vertexId&1 == 1 => +x, vertexId&2 == 2 => +y, and vertexId&4 == 4 => +z.
	float3 boxCoordinates = float3(vertexId & 1, (vertexId & 2) >> 1, (vertexId & 4) >> 2);
	float3 endpoint = boxCoordinates.z > 0 ? instance.End : instance.Start;
	//Note that distance is used instead of z. Resizing lines based on camera orientation is a bit odd.
	float distance = length(endpoint - CameraPosition);
	float pixelSize = distance * TanAnglePerPixel;
	output.PixelSize = pixelSize;


	float3 worldLineX = cross(CameraForward, worldLine);
	float worldLineXLength = length(worldLineX);
	if (worldLineXLength < 1e-7)
	{
		worldLineX = cross(CameraRight, worldLine);
		worldLineXLength = length(worldLineX);
	}
	worldLineX /= worldLineXLength;
	float3 worldLineY = cross(worldLine, worldLineX);
	//Use the pixel size in world space to pad out the bounding box.
	const float paddingInPixels = OuterRadius + SampleRadius;

	float worldPadding = paddingInPixels * pixelSize;
	float3 paddingX = worldPadding * worldLineX;
	float3 paddingY = worldPadding * worldLineY;
	float3 paddingZ = worldPadding * worldLine;
	float3 position = boxCoordinates.z > 0 ?
		instance.End + paddingZ :
		instance.Start - paddingZ;
	position += (boxCoordinates.x * 2 - 1) * paddingX;
	position += (boxCoordinates.y * 2 - 1) * paddingY;
	output.ToBox = position - CameraPosition;
	output.Position = mul(float4(position, 1), ViewProjection);
	return output;
	//A couple of notes:
	//1) More accurate depths could be calculated for the vertices rather than simply assuming that they have the same depth as the endpoint.
	//That assumption overestimates the depth of the vertices associated with the near point and underestimates the depth of the far point.
	//Only bother if actually matters.
	//2) The pixel shader could interpolate linear depth and offset it with conservative depth output.
	//That would allow smoother transitions between lines during overlap.
}

struct PSOutput
{
	float3 Color : SV_Target0;
};

PSOutput PSMain(PSInput input)
{
	PSOutput output;
	//Treat the view ray as a plane. Construct the plane's normal from the rayDirection and vector of closest approach between the two lines (lineDirection x rayDirection).
	//The plane normal is:
	//N = rayDirection x (lineDirection x rayDirection) / ||lineDirection x rayDirection||
	//(The vector triple product has some identities, but bleh.)
	//tLine = dot(lineStart - origin, N) / -dot(N, lineDirection)
	//Doing some algebra and noting that the origin is 0 here, that becomes:
	//tLine = dot(lineStart, rayDirection x (lineDirection x rayDirection)) / -||lineDirection x rayDirection||^2

	float3 rayDirection = input.ToBox;
	float3 lineCrossRay = cross(input.Direction, rayDirection);
	float3 numer = cross(rayDirection, lineCrossRay);
	float denom = -dot(lineCrossRay, lineCrossRay);
	//If the lines are parallel, just use the line start.
	float tLine = denom > -1e-7f ? 0 : dot(input.Start, numer) / denom;

	//The true tLine must be from 0 to lineLength.
	tLine = clamp(tLine, 0, input.Length);

	float3 closestOnLine = input.Start + tLine * input.Direction;
	float3 closestOnRay = rayDirection * (dot(closestOnLine, rayDirection) / dot(rayDirection, rayDirection));
	float lineDistance = distance(closestOnRay, closestOnLine);
	float lineScreenDistance = lineDistance / input.PixelSize;

	float innerDistance = lineScreenDistance - InnerRadius;
	//This distance is measured in screen pixels. Treat every pixel as having a set radius.
	//If the line's distance is beyond the sample radius, then there is zero coverage.
	//If the distance is 0, then the sample is half covered.
	//If the distance is less than -sampleRadius, then it's fully covered.
	float innerColorScale = saturate(0.5 - innerDistance / (SampleRadius * 2));
	//TODO: For now, don't bother using a falloff on the outer radius.
	output.Color = input.Color * innerColorScale + input.BackgroundColor * (1 - innerColorScale);
	//output.Color = 0;
	return output;
}