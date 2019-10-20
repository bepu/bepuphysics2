using DemoContentLoader;
using DemoRenderer;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using BepuPhysics;
using DemoRenderer.UI;
using System.IO;
using DemoUtilities;

namespace Demos.Demos.Sponsors
{
    public class SponsorDemo : Demo
    {
        struct Sponsor
        {
            public string Name;
            public RenderableImage RewardImage;
        }
        List<Sponsor> tier1Sponsors = new List<Sponsor>();

        RenderableImage CreateRewardImage(string rewardImagePath, ContentArchive content, RenderSurface surface)
        {
            var textureContent = content.Load<Texture2DContent>(rewardImagePath);
            return new RenderableImage(surface.Device, surface.Context, textureContent, Path.GetFileNameWithoutExtension(rewardImagePath));
        }
        void Add(List<Sponsor> sponsors, string name, string rewardImagePath, ContentArchive content, RenderSurface surface)
        {
            Sponsor sponsor;
            sponsor.RewardImage = CreateRewardImage(rewardImagePath, content, surface);
            sponsor.Name = name;
            sponsors.Add(sponsor);

        }
        public override void LoadGraphicalContent(ContentArchive content, RenderSurface surface)
        {
            Add(tier1Sponsors, "T. Rex", @"Content\Sponsors\angerybite.png", content, surface);
        }
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-30, 8, -60);
            camera.Yaw = MathF.PI * 3f / 4;
            camera.Pitch = 0;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            for (int i = 0; i < tier1Sponsors.Count; ++i)
            {
                var sponsor = tier1Sponsors[i];
                renderer.ImageBatcher.Draw(sponsor.RewardImage, new Vector2(150, 150), 100);
            }
        }

        protected override void OnDispose()
        {
            foreach (var sponsor in tier1Sponsors)
            {
                sponsor.RewardImage.Dispose();
            }
        }
    }
}
