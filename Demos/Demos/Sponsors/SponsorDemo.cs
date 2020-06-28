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
using BepuUtilities.Collections;
using BepuPhysics.Collidables;
using Demos.Demos.Characters;
using Helpers = DemoRenderer.Helpers;

namespace Demos.Demos.Sponsors
{
    public struct Sponsor
    {
        public string Name;
        public RenderableImage RewardImage;
    }

    public class SponsorDemo : Demo
    {
        List<string> sponsors0 = new List<string>();
        List<Sponsor> sponsors1 = new List<Sponsor>();
        List<Sponsor> sponsors2 = new List<Sponsor>();
        List<Sponsor> sponsors3 = new List<Sponsor>();

        QuickList<SponsorNewt> newts;
        StaticHandle overlordNewtHandle;

        RenderableImage CreateRewardImage(string rewardImagePath, ContentArchive content, RenderSurface surface)
        {
            var textureContent = content.Load<Texture2DContent>(rewardImagePath);
            return new RenderableImage(surface.Device, surface.Context, textureContent, debugName: Path.GetFileNameWithoutExtension(rewardImagePath));
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
            Add(sponsors1, "Gavin-Williams", @"Content\Sponsors\angerybite.png", content, surface);
            Add(sponsors1, "N.", @"Content\Sponsors\normalgiraffe.png", content, surface);
            Add(sponsors1, "M. H.", @"Content\Sponsors\tarbeenus.png", content, surface);
            Add(sponsors1, "K. C.", @"Content\Sponsors\physquirrel.png", content, surface);
            Add(sponsors1, "P. S.", @"Content\Sponsors\borb.png", content, surface);
            Add(sponsors1, "A different P. S.", @"Content\Sponsors\pleasedfrog.png", content, surface);
            Add(sponsors1, "E. C.", @"Content\Sponsors\disturbingkoala.png", content, surface);
            Add(sponsors1, "L. K. L.", @"Content\Sponsors\bork.png", content, surface);
            Add(sponsors1, "A. L.", @"Content\Sponsors\waryoctopus.png", content, surface);
            Add(sponsors1, "D. P.", @"Content\Sponsors\contentsluginthevoid.png", content, surface);
            Add(sponsors1, "Creative-House.org", @"Content\Sponsors\handicat.png", content, surface);
            Add(sponsors1, "vietnt", @"Content\Sponsors\raisondetre.png", content, surface);

            //These supporters are those who gave 50 dollars a month (or historical backers of roughly equivalent or greater total contribution).
            //They get a larger entry, a bit more text if desired, and a physically simulated entry in this demo.
            //You may notice that this tier does not have much in the way of restrictions on the "name"/text.
            Add(sponsors2, @"M. T.", @"Content\Sponsors\goose.png", content, surface);
            Add(sponsors2, @"Yo Mamma wants you to go to bed", @"Content\Sponsors\spide.png", content, surface);
            Add(sponsors2, @"A. R.", @"Content\Sponsors\scootybun.png", content, surface);
            Add(sponsors2, @"Alex 'mcmonkey' Goodwin, @mcmonkey4eva on github and twitter", @"Content\Sponsors\mcmonkey.png", content, surface);
            Add(sponsors2, @"A. K. D.", @"Content\Sponsors\behattedpenguin.png", content, surface);
            Add(sponsors2, @"Cornmaiden xoxoxo", @"Content\Sponsors\healthyostrich.png", content, surface);

            //These supporters are those who gave 1000 dollars a month (or historical backers of roughly equivalent or greater total contribution).
            Add(sponsors3, @"K. H. & F.", @"Content\Sponsors\spooky.png", content, surface);

        }

        Vector2 newtArenaMin, newtArenaMax;
        Random random;
        CharacterControllers characterControllers;
        QuickList<SponsorCharacterAI> characterAIs;
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(130, 50, 130);
            camera.Yaw = -MathF.PI * 0.25f;
            camera.Pitch = 0.4f;

            characterControllers = new CharacterControllers(BufferPool);
            Simulation = Simulation.Create(BufferPool, new CharacterNarrowphaseCallbacks(characterControllers), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));

            DemoMeshHelper.LoadModel(content, BufferPool, @"Content\newt.obj", new Vector3(-10, 10, -10), out var newtMesh);
            var newtShape = Simulation.Shapes.Add(newtMesh);
            newts = new QuickList<SponsorNewt>(sponsors2.Count, BufferPool);
            newtArenaMin = new Vector2(-100);
            newtArenaMax = new Vector2(100);
            random = new Random(6);
            for (int i = 0; i < sponsors2.Count; ++i)
            {
                ref var newt = ref newts.AllocateUnsafely();
                newt = new SponsorNewt(Simulation, newtShape, 0, newtArenaMin, newtArenaMax, random, i);
            }

            const float floorSize = 240;
            const float wallThickness = 200;
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -10f, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(floorSize, 20, floorSize)), 0.1f)));
            Simulation.Statics.Add(new StaticDescription(new Vector3(floorSize * -0.5f - wallThickness * 0.5f, -5, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(wallThickness, 30, floorSize + wallThickness * 2)), 0.1f)));
            Simulation.Statics.Add(new StaticDescription(new Vector3(floorSize * 0.5f + wallThickness * 0.5f, -5, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(wallThickness, 30, floorSize + wallThickness * 2)), 0.1f)));
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -5, floorSize * -0.5f - wallThickness * 0.5f), new CollidableDescription(Simulation.Shapes.Add(new Box(floorSize, 30, wallThickness)), 0.1f)));
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -5, floorSize * 0.5f + wallThickness * 0.5f), new CollidableDescription(Simulation.Shapes.Add(new Box(floorSize, 30, wallThickness)), 0.1f)));

            const int characterCount = 1000;
            characterAIs = new QuickList<SponsorCharacterAI>(characterCount, BufferPool);
            var characterCollidable = new CollidableDescription(Simulation.Shapes.Add(new Capsule(0.5f, 1f)), 0.1f);
            for (int i = 0; i < characterCount; ++i)
            {
                var position2D = newtArenaMin + (newtArenaMax - newtArenaMin) * new Vector2((float)random.NextDouble(), (float)random.NextDouble());
                var targetPosition = 0.5f * (newtArenaMin + (newtArenaMax - newtArenaMin) * new Vector2((float)random.NextDouble(), (float)random.NextDouble()));
                characterAIs.AllocateUnsafely() = new SponsorCharacterAI(characterControllers, characterCollidable, new Vector3(position2D.X, 5, position2D.Y), targetPosition);
            }

            const int hutCount = 30;
            var hutBoxShape = new Box(0.4f, 2, 3);
            hutBoxShape.ComputeInertia(20, out var obstacleInertia);
            var obstacleDescription = BodyDescription.CreateDynamic(new Vector3(), obstacleInertia, new CollidableDescription(Simulation.Shapes.Add(hutBoxShape), 0.1f), new BodyActivityDescription(1e-2f));

            for (int i = 0; i < hutCount; ++i)
            {
                var position2D = newtArenaMin + (newtArenaMax - newtArenaMin) * new Vector2((float)random.NextDouble(), (float)random.NextDouble());
                ColosseumDemo.CreateRing(Simulation, new Vector3(position2D.X, 0, position2D.Y), hutBoxShape, obstacleDescription, 5, 2, random.Next(1, 5));

            }

            var overlordNewtShape = newtMesh;
            overlordNewtShape.Scale = new Vector3(60, 60, 60);
            overlordNewtHandle = Simulation.Statics.Add(new StaticDescription(new Vector3(0, 10, -floorSize * 0.5f - 70), new CollidableDescription(Simulation.Shapes.Add(overlordNewtShape), 0.1f)));
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

        double realTime;
        double simulationTime;
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            const float simulationDt = 1 / 60f;
            Simulation.Timestep(simulationDt, ThreadDispatcher);
            for (int i = 0; i < newts.Count; ++i)
            {
                newts[i].Update(Simulation, simulationTime, 0, newtArenaMin, newtArenaMax, random, 1f / simulationDt);
            }
            for (int i = 0; i < characterAIs.Count; ++i)
            {
                characterAIs[i].Update(characterControllers, Simulation, ref newts, newtArenaMin, newtArenaMax, random);
            }
            simulationTime += simulationDt;
            realTime += dt;
        }
        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            var viewProjection = camera.ViewProjection;
            var integerResolution = renderer.Surface.Resolution;
            var resolution = new Vector2(integerResolution.X, integerResolution.Y);
            for (int i = 0; i < newts.Count; ++i)
            {
                newts[i].Render(Simulation, sponsors2, renderer, viewProjection, resolution, text, font);
            }

            //We'll hardcode the overlord newts. Not going to be a problem, I suspect.
            {
                var worldTextPosition = Simulation.Statics.Poses[Simulation.Statics.HandleToIndex[overlordNewtHandle.Value]].Position + new Vector3(0, 48, 0);
                Helpers.GetScreenLocation(worldTextPosition, viewProjection, resolution, out var screenspacePosition);
                const float nameHeight = 14;
                var name = sponsors3[0].Name;
                var nameLength = GlyphBatch.MeasureLength(name, font, nameHeight);
                screenspacePosition.X -= nameLength * 0.5f;
                renderer.TextBatcher.Write(text.Clear().Append(name), screenspacePosition, nameHeight, new Vector3(0.3f, 0f, 0f), font);
            }

            var integralMousePosition = input.MousePosition;
            var mousePosition = new Vector2(integralMousePosition.X, integralMousePosition.Y);
            renderer.TextBatcher.Write(text.Clear().Append("Mouseover entries to view additional very important tier rewards."), new Vector2(32, resolution.Y - 50), 14, new Vector3(1), font);
            renderer.TextBatcher.Write(text.Clear().Append("Are you a sponsor, but missing from this list? Want a different name/nickname for your entry? Send me a message!"), new Vector2(32, resolution.Y - 32), 14, new Vector3(1), font);
            DrawSponsors("Super duper sponsors", sponsors3, new Vector2(32, resolution.Y - 480), mousePosition, renderer, text, font, realTime, 1, 4, 0.25f, 32, 6, 48);
            DrawSponsors("Very neat sponsors", sponsors2, new Vector2(32, resolution.Y - 288), mousePosition, renderer, text, font, realTime, 4, 4, 0.25f, 24, 4, 36);
            DrawSponsors("Sponsors", sponsors1, new Vector2(renderer.Surface.Resolution.X - 512, resolution.Y - 480), mousePosition, renderer, text, font, realTime, 4, 4, 0.25f, 24, 4, 36);
            DrawSponsors("Smaller sponsors who are still cool", sponsors0, new Vector2(renderer.Surface.Resolution.X - 512, resolution.Y - 288), renderer, text, font, realTime, 4, 4, 0.25f, 16, 24);

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
