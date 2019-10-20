float3 UnpackR11G11B10_UNorm(uint packed)
{
	const uint mask11 = (1 << 11) - 1;
	const uint mask10 = (1 << 10) - 1;
	uint r = packed & mask11;
	uint g = (packed >> 11)& mask11;
	uint b = packed >> 22;

	return float3(
		r * (1.0 / mask11),
		g * (1.0 / mask11),
		b * (1.0 / mask10));

}

float4 UnpackR8G8B8A8_UNorm(uint packed)
{
	const float scale = 1.0 / 255.0;
	return float4(
		(packed & 255) * scale,
		((packed >> 8) & 255) * scale,
		((packed >> 16) & 255) * scale,
		((packed >> 24) & 255) * scale);
}