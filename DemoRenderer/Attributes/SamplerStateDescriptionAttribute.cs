using SharpDX.Direct3D11;
using SharpDX.Mathematics.Interop;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DemoRenderer.Attributes
{
    internal class SamplerStateDescriptionAttribute : Attribute
    {
        //     Filtering method to use when sampling a texture (see SharpDX.Direct3D11.Filter).
        public Filter Filter = Filter.MinMagMipLinear;

        //     Method to use for resolving a u texture coordinate that is outside the 0 to 1
        //     range (see SharpDX.Direct3D11.TextureAddressMode).
        public TextureAddressMode AddressU = TextureAddressMode.Clamp;

        //     Method to use for resolving a v texture coordinate that is outside the 0 to 1
        //     range.
        public TextureAddressMode AddressV = TextureAddressMode.Clamp;

        //     Method to use for resolving a w texture coordinate that is outside the 0 to 1
        //     range.
        public TextureAddressMode AddressW = TextureAddressMode.Clamp;

        //     Offset from the calculated mipmap level. For example, if Direct3D calculates
        //     that a texture should be sampled at mipmap level 3 and MipLODBias is 2, then
        //     the texture will be sampled at mipmap level 5.
        public float MipLodBias = 0f;

        //     Clamping value used if D3D11_FILTER_ANISOTROPIC or D3D11_FILTER_COMPARISON_ANISOTROPIC
        //     is specified in Filter. Valid values are between 1 and 16.
        public int MaximumAnisotropy = 16;

        //     A function that compares sampled data against existing sampled data. The function
        //     options are listed in SharpDX.Direct3D11.Comparison.
        public Comparison ComparisonFunction = Comparison.Never;

        //     Border color to use if D3D11_TEXTURE_ADDRESS_BORDER is specified for AddressU,
        //     AddressV, or AddressW. Range must be between 0.0 and 1.0 inclusive.
        public RawColor4 BorderColor = default(RawColor4);

        //     Lower end of the mipmap range to clamp access to, where 0 is the largest and
        //     most detailed mipmap level and any level higher than that is less detailed.
        public float MinimumLod = float.MinValue;

        //     Upper end of the mipmap range to clamp access to, where 0 is the largest and
        //     most detailed mipmap level and any level higher than that is less detailed. This
        //     value must be greater than or equal to MinLOD. To have no upper limit on LOD
        //     set this to a large value such as D3D11_FLOAT32_MAX.
        public float MaximumLod = float.MaxValue;

    }
}
