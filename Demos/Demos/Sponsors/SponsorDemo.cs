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
using System.Diagnostics;

namespace Demos.Demos.Sponsors
{
    public class SponsorDemo : Demo
    {
        struct Sponsor
        {
            public string Name;
            public RenderableImage RewardImage;
        }
        List<string> sponsors0 = new List<string>();
        List<Sponsor> sponsors1 = new List<Sponsor>();
        List<Sponsor> sponsors2 = new List<Sponsor>();
        List<Sponsor> sponsors3 = new List<Sponsor>();

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
            //I tried to reach out to all historical backers, but given the timescales involved, it's understandable that not everyone responded.
            //I don't want to publish their full names without permission, but I also want to give them some credit, so the initials will have to do for now.
            //If you're a supporter and don't see yourself on here, get in touch and I'll add you.

            //These supporters are those who gave 5 dollars a month (or historical backers of roughly equivalent or greater total contribution).
            sponsors0.Add("J. G.");
            sponsors0.Add("J. Y.");
            sponsors0.Add("J. F.");
            sponsors0.Add("C. S.");
            sponsors0.Add("R. W.");

            //These supporters are those who gave 10 dollars a month (or historical backers of roughly equivalent or greater total contribution).
            //They get a poorly drawn animal!
            Add(sponsors1, "Y. V.", @"Content\Sponsors\smugturt.png", content, surface);
            Add(sponsors1, "R. S. K.", @"Content\Sponsors\squidhanginout.png", content, surface);
            Add(sponsors1, "G. W.", @"Content\Sponsors\angerybite.png", content, surface);
            Add(sponsors1, "N.", @"Content\Sponsors\normalgiraffe.png", content, surface);
            Add(sponsors1, "M. H.", @"Content\Sponsors\tarbeenus.png", content, surface);
            Add(sponsors1, "K. C.", @"Content\Sponsors\physquirrel.png", content, surface);
            Add(sponsors1, "P. S.", @"Content\Sponsors\borb.png", content, surface);
            Add(sponsors1, "A different P. S.", @"Content\Sponsors\pleasedfrog.png", content, surface);
            Add(sponsors1, "E. C.", @"Content\Sponsors\disturbingkoala.png", content, surface);
            Add(sponsors1, "L. K. L.", @"Content\Sponsors\bork.png", content, surface);
            Add(sponsors1, "A. L.", @"Content\Sponsors\waryoctopus.png", content, surface);
            Add(sponsors1, "D. P.", @"Content\Sponsors\contentsluginthevoid.png", content, surface);

            //These supporters are those who gave 50 dollars a month (or historical backers of roughly equivalent or greater total contribution).
            //They get a larger entry, a bit more text if desired, and a physically simulated entry in this demo.
            Add(sponsors2, @"M. T.", @"Content\Sponsors\goose.png", content, surface);
            Add(sponsors2, @"A. R.", @"Content\Sponsors\scootybun.png", content, surface);
            Add(sponsors2, @"Alex 'mcmonkey' Goodwin, @mcmonkey4eva on github and twitter", @"Content\Sponsors\mcmonkey.png", content, surface);
            Add(sponsors2, @"A. K. D.", @"Content\Sponsors\behattedpenguin.png", content, surface);
            Add(sponsors2, @"CM", @"Content\Sponsors\healthyostrich.png", content, surface);

            //These supporters are those who gave 1000 dollars a month (or historical backers of roughly equivalent or greater total contribution).
            Add(sponsors3, @"K. H. & F.", @"Content\Sponsors\spooky.png", content, surface);
        }
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-30, 8, -60);
            camera.Yaw = MathF.PI * 3f / 4;
            camera.Pitch = 0;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));
        }


        static void Get(int sponsorCount, int maximumBatchSize, double time, float timePerBatch, float fadeTime, out float alpha, out int start, out int end)
        {
            var batchCount = (int)MathF.Ceiling((float)sponsorCount / maximumBatchSize);
            if (batchCount > 1)
            {
                var batchIndex = (time / timePerBatch) % batchCount;
                //Fade time covers the time from full alpha to 0 and back to full alpha again. In other words, the full time for a given batch is:
                //[0, 0.5 * fadeTime) [0.5 * fadeTime, timePerBatch - 0.5 * fadeT6ime ) [timePerBatch - 0.5 * fadeTime, timePerBatch)
                Debug.Assert(fadeTime <= timePerBatch);
                var batchProgress = (float)(timePerBatch * (batchIndex - MathF.Truncate((float)batchIndex)));
                var inverseFadeTime = 1f / fadeTime;
                alpha = MathF.Min(MathF.Min(1f, batchProgress * inverseFadeTime), inverseFadeTime * timePerBatch - batchProgress * inverseFadeTime);
                start = (int)batchIndex * maximumBatchSize;
                end = Math.Min(start + maximumBatchSize, sponsorCount);
            }
            else
            {
                alpha = 1;
                start = 0;
                end = sponsorCount;
            }
        }

        static float DrawSponsors(string groupTitle, List<Sponsor> sponsors, Vector2 position, Vector2 mousePosition, Renderer renderer, TextBuilder text, Font font,
            double time, int maximumBatchSize, float timePerBatch, float fadeTime, float fontSize, float padding, float lineSpacing)
        {
            var initialY = position.Y;
            renderer.TextBatcher.Write(text.Clear().Append(groupTitle), position, fontSize * 1.5f, new Vector4(1), font);
            position += new Vector2(fontSize * 0.5f, lineSpacing);
            Get(sponsors.Count, maximumBatchSize, time, timePerBatch, fadeTime, out var alpha, out var start, out var end);
            for (int i = start; i < end; ++i)
            {
                var sponsor = sponsors[i];
                renderer.TextBatcher.Write(text.Clear().Append(sponsor.Name), position, fontSize, new Vector4(new Vector3(1), alpha), font);
                var width = GlyphBatch.MeasureLength(sponsor.Name, font, fontSize);
                var boundingBoxMin = position - new Vector2(padding, padding + fontSize);
                var boundingBoxMax = position + new Vector2(width + padding, padding);
                if (Vector2.Min(boundingBoxMax, Vector2.Max(boundingBoxMin, mousePosition)) == mousePosition)
                {
                    //Mouse is within the mouseover region. Show reward image.
                    //(I wondered what to do with the sponsor images for a while. It seems odd to display them always, since it puts the focus on weird animal pictures 
                    //as much as on the sponsor themselves. At the same time, gotta advertise the sweet weird animal pictures you could get by being a sponsor.
                    //Mouseover seems like a decent compromise.)
                    renderer.ImageBatcher.Draw(sponsor.RewardImage, Vector2.Max(Vector2.Zero, position + new Vector2(width + 8f * padding, fontSize * -0.4f - 2.5f * lineSpacing)), 5 * lineSpacing, new Vector4(new Vector3(1), alpha));
                }
                position.Y += lineSpacing;
            }
            return position.Y - initialY;
        }

        float DrawSponsors(string groupTitle, List<string> sponsors, Vector2 position, Renderer renderer, TextBuilder text, Font font,
            double time, int maximumBatchSize, float timePerBatch, float fadeTime, float fontSize, float lineSpacing)
        {
            var initialY = position.Y;
            renderer.TextBatcher.Write(text.Clear().Append(groupTitle), position, fontSize * 1.5f, new Vector4(1), font);
            position += new Vector2(fontSize * 0.5f, lineSpacing);
            Get(sponsors.Count, maximumBatchSize, time, timePerBatch, fadeTime, out var alpha, out var start, out var end);
            for (int i = start; i < end; ++i)
            {
                var sponsor = sponsors[i];
                renderer.TextBatcher.Write(text.Clear().Append(sponsor), position, fontSize, new Vector4(new Vector3(1), alpha), font);
                position.Y += lineSpacing;
            }
            return position.Y - initialY;
        }

        double time;
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            base.Update(window, camera, input, dt);
            time += dt;
        }
        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            var integralMousePosition = input.MousePosition;
            var mousePosition = new Vector2(integralMousePosition.X, integralMousePosition.Y);
            renderer.TextBatcher.Write(text.Clear().Append("Mouseover entries to view additional very important tier rewards."), new Vector2(224, 48), 16, new Vector3(1), font);
            renderer.TextBatcher.Write(text.Clear().Append("Are you a sponsor, but missing from this list? Want a different name/nickname for your entry? Send me a message!"), new Vector2(224, 68), 16, new Vector3(1), font);
            DrawSponsors("Super duper sponsors", sponsors3, new Vector2(200, 124), mousePosition, renderer, text, font, time, 1, 4, 0.25f, 32, 6, 48);
            DrawSponsors("Very neat sponsors", sponsors2, new Vector2(200, 124 + 192), mousePosition, renderer, text, font, time, 4, 4, 0.25f, 24, 4, 36);
            DrawSponsors("Sponsors", sponsors1, new Vector2(200, 124 + 192 + 192), mousePosition, renderer, text, font, time, 4, 4, 0.25f, 24, 4, 36);
            DrawSponsors("Smaller sponsors who are still cool", sponsors0, new Vector2(200, 124 + 192 * 3), renderer, text, font, time, 4, 4, 0.25f, 16, 24);
        }

        protected override void OnDispose()
        {
            foreach (var sponsor in sponsors1)
            {
                sponsor.RewardImage.Dispose();
            }
            foreach (var sponsor in sponsors2)
            {
                sponsor.RewardImage.Dispose();
            }
            foreach (var sponsor in sponsors3)
            {
                sponsor.RewardImage.Dispose();
            }
        }
    }
}
