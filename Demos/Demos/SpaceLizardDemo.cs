using BepuPhysics;using BepuPhysics.Collidables;using BepuPhysics.CollisionDetection;using BepuPhysics.Constraints;using BepuUtilities;using DemoContentLoader;using DemoRenderer;using System;using System.Collections.Generic;using System.Linq;using System.Numerics;using System.Reflection;
using System.Runtime.CompilerServices;namespace Demos.Demos{









    //Document all methods in a standard way so that completion can use them.
    //Parallelize and use vector and SIMD whenever appropriate.
    //Fix bulk spell errors using a plugin and its bug.
    //Hope that quantum scales scale similarly with larger scales or find a relation between Lorentz-like or whatever.
    //Use AI search and FFT methods for 3-axis.

    /// <summary>
    //    /// // the cyclic deformation should be used to convert the angular momentum to translation along the curve. 
    /// // The cyclic deformation is a process
    // of applying a force to a material in a cyclic manner, causing it to deform in a cyclic manner. This process can be used to convert angular momentum to translation
    // along a convex or other curved surface. The variation principle should be applied according to Greens theorem, and the force should be applied in two axes to avoid
    // gaining any kinetic energy moving orthogonally (against gravity). The cyclic deformation can be used to move the material along the curve without the need for a
    // plumb line or other control.
//Swimming is a form of locomotion that involves propelling oneself through a liquid medium by using the arms, legs, and/or tail. The momentum of the arms and legs
// is used to move the body forward, while the arms and legs are moved in a geodesic pattern to create a wave-like motion. This wave-like motion creates a net gain
// in kinetic energy, allowing the swimmer to move forward. Additionally, the arms and legs can be moved out along the geodesic of a curve and kicked out so that more
// geodesics are crossed. This helps to create an angular momentum change, which can be accounted for in an expanding space. Experiments in flat space and various 2
//D shell dynamics, expansion ratios, and Lorentzian effects can also be used to understand swimming.
//noncommutative aspects of phase gait is key and it must be tuned..  

    public class SpaceLizardDemo : Demo    {

        struct SubgroupFilteredCallbacks : INarrowPhaseCallbacks
        {
            public CollidableProperty<SubgroupCollisionFilter> CollisionFilters;
            public PairMaterialProperties Material;

            public SubgroupFilteredCallbacks(CollidableProperty<SubgroupCollisionFilter> filters)
            {
                CollisionFilters=filters;
                Material=new PairMaterialProperties(0, 2, new SpringSettings(30, 1));
            }
            public SubgroupFilteredCallbacks(CollidableProperty<SubgroupCollisionFilter> filters, PairMaterialProperties material)
            {
                CollisionFilters=filters;
                Material=material;
            }


            public void Initialize(Simulation simulation)
            {
                CollisionFilters.Initialize(simulation);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
            {
                //It's impossible for two statics to collide, and pairs are sorted such that bodies always come before statics.
                if (b.Mobility!=CollidableMobility.Static)
                {
                    return SubgroupCollisionFilter.AllowCollision(CollisionFilters[a.BodyHandle], CollisionFilters[b.BodyHandle]);
                }
                return a.Mobility==CollidableMobility.Dynamic||b.Mobility==CollidableMobility.Dynamic;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
            {
                return true;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
            {
                pairMaterial=Material;
                return true;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
            {
                return true;
            }

            public void Dispose()
            {
                CollisionFilters.Dispose();
            }


        }



        /// </summary>        ///       This is a public struct called SimpleMaterial.        ///      It contains fields: SpringSettings, FrictionCoefficient, and MaximumRecoveryVelocity. The SpringSettings field        ///      is of type SpringSettings, which is a struct that contains information about a spring, such as its stiffness and damping. The FrictionCoefficient field is a float        ///       that represents the coefficient of friction between two surfaces. The MaximumRecoveryVelocity field is a float that represents the maximum velocity at which an object        ///    can recover from a collision. three


        public struct SimpleMaterial
        {
            public SpringSettings SpringSettings;
            public float FrictionCoefficient;
            public float MaximumRecoveryVelocity;
        }



        public unsafe struct FrictionCallbacks : INarrowPhaseCallbacks
        {
            public CollidableProperty<SimpleMaterial> CollidableMaterials;

            public void Initialize(Simulation simulation)
            {
                //The callbacks get created before the simulation so that they can be given to the simulation. The property needs a simulation reference, so we hand it over in the initialize.
                CollidableMaterials.Initialize(simulation);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
            {
                //While the engine won't even try creating pairs between statics at all, it will ask about kinematic-kinematic pairs.
                //Those pairs cannot emit constraints since both involved bodies have infinite inertia. Since most of the demos don't need
                //to collect information about kinematic-kinematic pairs, we'll require that at least one of the bodies needs to be dynamic.
                return a.Mobility==CollidableMobility.Dynamic||b.Mobility==CollidableMobility.Dynamic;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
            {
                return true;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
            {
                //For the purposes of this demo, we'll use multiplicative blending for the friction and choose spring properties according to which collidable has a higher maximum recovery velocity.
                var a = CollidableMaterials[pair.A];
                var b = CollidableMaterials[pair.B];
                pairMaterial.FrictionCoefficient=a.FrictionCoefficient*b.FrictionCoefficient;
                pairMaterial.MaximumRecoveryVelocity=MathF.Max(a.MaximumRecoveryVelocity, b.MaximumRecoveryVelocity);
                pairMaterial.SpringSettings=pairMaterial.MaximumRecoveryVelocity==a.MaximumRecoveryVelocity ? a.SpringSettings : b.SpringSettings;
                return true;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
            {
                return true;
            }

            public void Dispose()
            {
            }

        }

        /// <summary>
        /// The Panels of the chains for now they are the same,        /// might have a reduction factor or ratio or amount later        /// , or taper the shape for stability          ///  </summary>                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            public class BodySegment
        public class BodySegment
        {

            static public int NumFields => 4;

            public float Length = 1f;
            public float Width = 1f;
            public float Thickness = 0.2f;
            public float Density = 100;

            public BodySegment()
            {
                SetDefaults();
            }


            // Set the default values for the body segment
            void SetDefaults()
            {
                Length=1f;
                Width=1f;
                Thickness=0.2f;
                Density=100;
            }

            // Constructor with parameters
            public BodySegment(float length, float width, float thickness, float density)
            {
                Length=length;
                Width=width;
                Thickness=thickness;
                Density=density;
            }

            // Calculate the volume of the body segment
            public float Volume => Length*Width*Thickness;

            // Calculate the mass of the body segment
            public float Mass => Volume*Density;

            // Create a box shape for the body segment
            public Box Shape => new Box(Length, Width, Thickness);

            // Create Moment of Inertia for this Body
            public BodyInertia InertiaTensor => Shape.ComputeInertia(Mass);




            /// <summary>
            ///   /To avoid use of capsules and overlap for now,  extends a given side of a shape by a given thickness factor. The function takes three parameters: side, dir, and factor. to the side
            // of the shape that is being extended. The dir parameter is the direction in which the side is being extended. The factor parameter is an optional parameter that specifies
            // the amount by which the side is being extended 1, or -1 front away fromn cam , or left -1, right 1 The function returns the extended side by multiplying the thickness by the factor and then adding it to the side
            /// </summary>
            /// <param name="side"></param>
            /// <param name="dir"></param>
            /// <param name="factor"></param>
            /// <returns></returns>

            public float ExtendByThicknessFactor(float side, float dir, float factor = 1.0f)
            => side+Thickness*factor*dir;



            public void SetParams(float[] param)            {

                FieldInfo[] fields = GetType().GetFields(BindingFlags.Public|BindingFlags.Instance);                if (fields.Length==param.Length)                {                    for (int i = 0; i<fields.Length; i++)                    {
                        //TODO  META  datat for tunable
                        if (fields[i].FieldType!=typeof(float))                        {                            continue;                        }                        fields[i].SetValue(this, param[i]);                    }                }                else                {                    Console.WriteLine("Number of parameters passed in does not match the number of fields.");                }            }

        }

        /// <summary>
        ///         For a DNA like description to generate and optimize the creature.. the opmizer adn adjjust and maximize these
        /// </summary>
        public class Phenotype
        {

            public Vector3 Position;
            public int NumNeck;
            public int NumTail;
            public int NumWingSeg;
            internal float[]? Parameters;




            /// <param name="position"></param>            /// <param name="numNeck"></param>            /// <param name="numTai"></param>            /// <param name="numWingSeg"></param>            /// <param name="wing"></param>            /// <param name="neck"></param>            /// <param name="tail"></param>            /// <param name="nexusSegment"></param>

            public Phenotype(Vector3 position, int numNeck, int numTai,
                int numWingSeg, BodySegment wing, BodySegment neck,
                BodySegment tail, BodySegment nexusSegment)
            {
                Position=position;
                NumNeck=numNeck;
                NumTail=numTai;
                Wing=wing;
                Neck=neck;
                Tail=tail;
                NexusSegment=nexusSegment;

            }

            public Phenotype(Vector3 position,
                int numNeck, int numTail, int numWingSeg, float[]
                parameters, BodySegment wing, BodySegment neck, BodySegment tail, BodySegment nexusSegment) :
                this(position, numNeck, numTail, numWingSeg)
            {
                Parameters=parameters;
                Wing=wing;
                Neck=neck;
                Tail=tail;
                NexusSegment=nexusSegment;
            }

            public Phenotype(Vector3 position, int numNeck, int numTail, int numWingSeg)
            {
                Position=position;
                NumNeck=numNeck;
                NumTail=numTail;
                NumWingSeg=numWingSeg;
            }



            /// <summary>
            /// Caluclate center of mass of all the partcole susing intertia
            /// </summary>
            //    public Vector2  CenterMassSytem => 

            public static int NumFields => BodySegment.NumFields;


            public BodySegment Wing { get; set; } = new BodySegment();
            public BodySegment Neck { get; set; } = new BodySegment();
            public BodySegment Tail { get; set; } = new BodySegment();
            public BodySegment NexusSegment { get; set; } = new BodySegment();


            internal float GetSpeed()
            {
                return 0;
            }





            static BodyHandle AddBody<TShape>(TShape shape, float mass, in RigidPose pose, Simulation simulation) where TShape : unmanaged, IConvexShape
            {
                //Note that this always registers a new shape instance. You could be more clever/efficient and share shapes, but the goal here is to show the most basic option.
                //Also, the cost of registering different shapes isn't that high for tiny implicit shapes.
                var shapeIndex = simulation.Shapes.Add(shape);
                var description = BodyDescription.CreateDynamic(pose,
                    shape.ComputeInertia(mass), shapeIndex, 0.01f);
                return simulation.Bodies.Add(description);
            }

            static RigidPose GetWorldPose(Vector3 localPosition, Quaternion localOrientation, RigidPose ragdollPose)
            {
                RigidPose worldPose;
                RigidPose.Transform(localPosition, ragdollPose, out worldPose.Position);
                QuaternionEx.ConcatenateWithoutOverlap(localOrientation, ragdollPose.Orientation, out worldPose.Orientation);
                return worldPose;
            }
            public static void GetCapsuleForLineSegment(Vector3 start, Vector3 end, float radius, out Capsule capsule, out Vector3 position, out Quaternion orientation)
            {
                position=0.5f*(start+end);

                var offset = end-start;
                capsule.HalfLength=0.5f*offset.Length();
                capsule.Radius=radius;
                //The capsule shape's length is along its local Y axis, so get the shortest rotation from Y to the current orientation.
                var cross = Vector3.Cross(offset/capsule.Length, new Vector3(0, 1, 0));
                var crossLength = cross.Length();
                orientation=crossLength>1e-8f ?
                    QuaternionEx.CreateFromAxisAngle(cross/crossLength, (float)Math.Asin(crossLength)) : Quaternion.Identity;
            }

            public static Quaternion CreateBasis(Vector3 z, Vector3 x)
            {
                //For ease of use, don't assume that x is perpendicular to z, nor that either input is normalized.
                Matrix3x3 basis;
                basis.Z=Vector3.Normalize(z);
                basis.Y=Vector3.Normalize(Vector3.Cross(basis.Z, x));
                basis.X=Vector3.Cross(basis.Y, basis.Z);
                QuaternionEx.CreateFromRotationMatrix(basis, out var toReturn);
                return toReturn;
            }

            static AngularMotor BuildAngularMotor()
            {
                //By default, these motors use nonzero softness (inverse damping) to damp the relative motion between ragdoll pieces.
                //If you set the damping to 0 and then set the maximum force to some finite value (75 works reasonably well), the ragdolls act more like action figures.
                //You could also replace the AngularMotors with AngularServos and provide actual relative orientation goals for physics-driven animation.
                return new AngularMotor { TargetVelocityLocalA=new Vector3(), Settings=new MotorSettings(float.MaxValue, 0.01f) };
            }



            public static void GetSide(BodySegment bodySegment, Vector3 direction, out float dir, out float side)
            {
                dir=direction.X==0 ? direction.Z : direction.X;
                side=direction.X==0 ? bodySegment.Width : bodySegment.Length;
            }

        }


        //    bool showDome = true;
        bool showDome = true;
        //Set up a camer an a curve dk+ domes wiht no frication

        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {

            float sphereRadius = 200;
            float groundThickness = 40;

            float domeHeight = 0.1f;

            float startY = showDome ? 20 : 2;
            var originSpawn = new Vector3(0, startY, 0);     //star it 20 meters up

            camera.Position=new Vector3(0, 20, showDome ? sphereRadius-60 : 4);
            camera.Yaw=-0;//
            camera.Pitch=MathHelper.Pi*0.25f;

            var collisionFilters = new CollidableProperty<SubgroupCollisionFilter>();

            var collidableMaterials = new CollidableProperty<SimpleMaterial>();

            Simulation=Simulation.Create(BufferPool, new FrictionCallbacks() { CollidableMaterials=collidableMaterials }, new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0), 0, 0), new SolveDescription(4, 1));


            //    new SubgroupFilteredCallbacks(collisionFilters
            //Simulation=Simulation.Create(BufferPool, new FrictionCallbacks() { CollidableMaterials=collidableMaterials,
            //    , new SubgroupFilteredCallbacks(collisionFilters),, new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0), 0, 0), new SolveDescription(4, 1));

            //       Simulation=Simulation.Create(BufferPool, new SubgroupFilteredCallbacks(collisionFilters),
            //          new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));




            var origin = new Vector3(0, 20, 0);



            Phenotype phenotype = new Phenotype(originSpawn, 5, 10, 16);


            phenotype.Neck.Thickness=0.2f;
            phenotype.Neck.Width=1f;
            phenotype.Neck.Length=1f;



            phenotype.NexusSegment.Thickness=0.2f;
            phenotype.NexusSegment.Width=1f;
            phenotype.NexusSegment.Length=1f;





            phenotype.Tail.Thickness=0.5f;
            phenotype.Tail.Width=1f;
            phenotype.Tail.Length=1f;
            phenotype.NexusSegment.Thickness=0.5f;
            phenotype.NexusSegment.Width=1f;
            phenotype.NexusSegment.Length=1f;

            phenotype.NumTail=10;
            phenotype.NumNeck=5;
            phenotype.NumWingSeg=8;

            //BuildPhenotype(phenotype
            //    , Simulation);



            CreateStaticGround(groundThickness, collidableMaterials);


            if (showDome)
            {
                Simulation.Statics.Add(new
                 StaticDescription(new
                 Vector3(0, -sphereRadius*(1-domeHeight), 0), Simulation.Shapes.Add(new Sphere(sphereRadius))));

            }

            //Simulation.Statics.Add(new StaticDescription(new Vector3(0, -groundThickness/2, 0),
            //    Simulation.Shapes.Add(new Box(1500, groundThickness, 1500))));
            //Simulation.Statics.Add(new

            //    StaticDescription(new
            //    Vector3(0, -sphereRadius*(1-domeHeight), 0), Simulation.Shapes.Add(new Sphere(sphereRadius)))
            //{

            //});
            //;


            BuildPhenotype(phenotype
    , Simulation, collidableMaterials);
            //        Simulation.Statics.Add(new StaticDescription(new Vector3(0, -0.5f, 0),
            //         Simulation.Shapes.Add(new Box(30000, 1, 30000))));
        }



        public void BuildPhenotype(Phenotype phenotype, Simulation simulation, CollidableProperty<SimpleMaterial> simpleMaterial)
        {


            //Unlike the bounciness demo, we don't have to worry about including strong substepping to help with stiff contact constraint bounces. Friction is easier. 
            //     var collidableMaterials = new CollidableProperty<SimpleMaterial>();

            var nexus =

    AddBoxBody(phenotype.Position, phenotype.NexusSegment, simulation, simpleMaterial);



            SpringSettings hingeSpring = new SpringSettings(30, 1);
            //var neck = BuildChain(
            //    phenotype.NumNeck,
            //         phenotype.Neck,
            //    phenotype.Position+new Vector3(0, 0, phenotype.NexusSegment.Length/2f),
            //    simulation,
            //    Vector3.UnitZ,
            //    hingeSpring, simpleMaterial);

            //var tail = BuildChain(phenotype.NumTail,
            //    nexus, phenotype.NexusSegment, phenotype.Tail,
            //     phenotype.Position-new Vector3(0, 0, phenotype.NexusSegment.Length/2f), simulation, -Vector3.UnitZ, hingeSpring, simpleMaterial);

            //var wingLeft = BuildChain(phenotype.NumWingSeg,
            //    nexus, phenotype.NexusSegment, phenotype.Wing, phenotype.Position+new Vector3(-phenotype.NexusSegment.Width/2f, 0, 0), simulation, -Vector3.UnitX, hingeSpring, simpleMaterial);
            //var wingRight = BuildChain(phenotype.NumWingSeg,
            //    nexus, phenotype.NexusSegment, phenotype.Wing, phenotype.Position+new Vector3(phenotype.NexusSegment.Width/2f, 0, 0), simulation, Vector3.UnitX, hingeSpring, simpleMaterial);



            // // Bug: The wingRight variable is not being used.     
            //    var wingLeft = BuildChain(phenotype.numWingSeg, phenotype.Wing, phenotype.Position+new Vector3(-phenotype.NexusSegment.Width/2f, 0, 0), simulation);

            // Assuming nexus body is at position (0, 0, 0)
            //// Add bodies in appropriate positions
            //simulation.Bodies.GetDescription(neck[0], out var neckDescription);
            //neckDescription.Pose.Position=phenotype.Position+new Vector3(0, 0, phenotype.NexusSegment.Length/2);
            //simulation.Bodies.ApplyDescription(neck[0], neckDescription);

            //simulation.Bodies.GetDescription(tail[0], out var tailDescription);

            //tailDescription.Pose.Position=phenotype.Position+new Vector3(0, 0, -phenotype.NexusSegment.Length/2);
            //simulation.Bodies.ApplyDescription(tail[0], tailDescription);

            //simulation.Bodies.GetDescription(wingLeft[0], out var wingLeftDescription);
            //wingLeftDescription.Pose.Position=phenotype.Position+new Vector3(-phenotype.NexusSegment.Width/2, 0, 0);
            //simulation.Bodies.ApplyDescription(wingLeft[0], wingLeftDescription);

            //simulation.Bodies.GetDescription(wingRight[0], out var wingRightDescription);
            //wingRightDescription.Pose.Position=phenotype.Position+new Vector3(phenotype.NexusSegment.Width/2, 0, 0);
            //simulation.Bodies.ApplyDescription(wingRight[0], wingRightDescription);

            //            // Bug: The wingRightDescription variable is not being used
            // Connect neck, tail and wings to nexus with hinges



            BodySegment bodySegment = phenotype.NexusSegment;

            var localOffsetA = new Vector3(bodySegment.Thickness/2f+bodySegment.Length/2, 0, 0);
            var localOffsetB = new Vector3(-bodySegment.Thickness/2f-bodySegment.Length/2, 0, 0);

            // tod put all thsi in the chain builder , pass in nexus..
            //   var hinge = BuildHinge( bodySegment, localOffsetA, localOffsetB, direction, springSettings, Quaternion.Identity, Quaternion.Identity);

            //  BuildHinge(nexus, neck[0],  simulation);
            //AddHinge(nexus, tail[0], simulation);
            //AddHinge(nexus, wingLeft[0], simulation);



            //  AddHinge(nexus, wingRight[0], simulation);

            // Connect consecutive segments of neck, tail and wings with hinges
            //for (int i = 1; i<neck.Count; i++)
            //    AddHinge(neck[i-1], neck[i], simulation);

            //for (int i = 1; i<tail.Count; i++)
            //    AddHinge(tail[i-1], tail[i], simulation);

            //for (int i = 1; i<wingLeft.Count; i++)
            //{
            //    AddHinge(wingLeft[i-1], wingLeft[i], simulation);
            //    AddHinge(wingRight[i-1], wingRight[i], simulation);


            //for (int i = 1; i<neck.Count; i++)
            //{
            //    AddHinge(neck[i-1], neck[i], simulation);
            //    localOffsetA=new Vector3(currentSegment.Thickness/2f+bodySegment.Length/2, 0, 0);
            //    var localOffsetB = new Vector3(-currentSegment.Thickness/2f-bodySegment.Length/2, 0, 0);

            //    var hinge = BuildHinge(segmentHandles[i-1], bodyHandle, localOffsetA, localOffsetB, direction, springSettings, Quaternion.Identity, Quaternion.Identity);
            //}

        }

        private void AddSegment(Vector3 startPos, Simulation simulation, Vector3 direction, CollidableProperty<SimpleMaterial> simpleMaterial, List<BodyHandle> segmentHandles, int i, BodySegment currentSegment, out float dir, out float side, out BodyHandle bodyHandle)
        {
            Phenotype.GetSide(currentSegment, direction, out dir, out side);

            var position = startPos+(direction*currentSegment.ExtendByThicknessFactor(side, dir)*i);

            bodyHandle=AddBoxBody(position, currentSegment, simulation, simpleMaterial);
            // Compute volume and mass of segments


            segmentHandles.Add(bodyHandle);
        }




































        /// Add a body dynamic ,it will collide with no friction, no elastic, andn non self collide, leng is along Z, widht is X,  thickness is verticl Y

        /////This method adds a body to a given simulation. The parameters are the length, width, thickness, and density of the body, as well as the simulation to which the body
            // should be added. The method returns a BodyHandle, which is an object that can be used to access and manipulate the body that was added.
            //non elastic, rigid, not left collide and no friction.  box  type shape with
        public BodyHandle AddBody(float length, float width, float thickness, float density, Simulation simulation)
        {
            var shape = new Box(length, width, thickness);
            var inertia = shape.ComputeInertia(density);
            var pose = new RigidPose
            {
                Position=new Vector3(0, 0, 0),
                Orientation=Quaternion.Identity
            };

            var description = BodyDescription.CreateDynamic(pose, inertia,
                new CollidableDescription(simulation.Shapes.Add(shape), 0.1f), new BodyActivityDescription(0.01f));
            return simulation.Bodies.Add(description);
        }

        ////There is no mention of friction in the method, but the CollidableDescription constructor takes a friction parameter. This should be set to 0 to ensure that the
        // body does not have any friction.








        //public BodyHandle AddBody(BodySegment segment, Vector2 location, float density = 1.0f, bool isFrictionCollidable = false, bool isDynamic = true)
        //{
        //    Body body = new Body(segment, location, density, isFrictionCollidable, isDynamic);
        //    return body;
        //}



        private unsafe void CreateStaticGround(float groundThickness, CollidableProperty<SimpleMaterial> collidableMaterials)
        {


            //   var groundDescript = new StaticDescription(new Vector3(0, -groundThickness/2, 0), typeIndex);



            //        collidableMaterials.Allocate(Simulation.Statics.Add(groundDescript), Simulation.Shapes.Add(new Box(2500, 30, 2500)))=
            //new SimpleMaterial { FrictionCoefficient=0, MaximumRecoveryVelocity=2, SpringSettings=new SpringSettings(30, 1) };


            collidableMaterials.Allocate(Simulation.Statics.Add(new StaticDescription(new Vector3(0, -15f, 0), Simulation.Shapes.Add(new Box(2500, 30, 2500)))))=
    new SimpleMaterial { FrictionCoefficient=1, MaximumRecoveryVelocity=2, SpringSettings=new SpringSettings(30, 1) };

            //Bug: The assignment operator (=) should not be used in this line of code. The correct syntax should be "collidableMaterials.Allocate(Simulation.Statics.Add(ground
            //Descript))" without the assignment operator.

        }


        public BodyHandle AddBoxBody(Vector3 pos, BodySegment body, Simulation simulation, CollidableProperty<SimpleMaterial> collidableMaterials)
        {
            var shape = body.Shape;


            var pose = new RigidPose
            {
                Position=pos,
                Orientation=Quaternion.Identity

            };


            var typeIndex = Simulation.Shapes.Add(new Box(body.Width, body.Thickness, body.Length));


            var boxDescription = BodyDescription.CreateDynamic(pose, new Vector3(0, 0, 0), body.InertiaTensor, typeIndex, 1e-2f);

            BodyHandle handle = simulation.Bodies.Add(boxDescription);




            /// Add a body dynamic ,it will collide with no friction, no elastic, and non self collide, length is along Z, widht is X,  thickness is verticl Y
            collidableMaterials.Allocate(handle)=
    new SimpleMaterial
    {
        FrictionCoefficient=0,
        MaximumRecoveryVelocity=2,
        SpringSettings=new SpringSettings(30, 1)
    };

            return handle;


        }



















































































































































































































































































































        /*List<BodyHandle> BuildChain(int segments, BodySegment bodySegment, Vector3 startPos, Simulation simulation, Vector3 direction, SpringSettings springSettings){var segmentHandles = new List<BodyHandle>();for (int i = 0; i<segments; i++){var position = startPos+direction*bodySegment.Length*i;// Compute volume and mass of segmentsvar segmentVolume = bodySegment.Volume;var segmentMass = segmentVolume*bodySegment.Density;var shape = new Box(bodySegment.Length, bodySegment.Width, bodySegment.Thickness); // Assuming sphere shape for simplicity, you should choose a shape that fits your needsvar collidable = new CollidableDescription(simulation.Shapes.Add(shape), 0.1f); var activity = new BodyActivityDescription(0.01f); // Assuming 0.01 sleep thresholdvar bodyDescription = BodyDescription.CreateDynamic(position, shape.ComputeInertia(segmentMass), collidable, activity);var handle = simulation.Bodies.Add(bodyDescription);segmentHandles.Add(handle);// Connect current segment with the previous one using a hinge jointif (i>0){Hinge hinge = BuildHindge1DOF(bodySegment, springSettings);simulation.Solver.Add(segmentHandles[i-1], handle, hinge);}//simulation.Solver.Add(segmentHandles[i-1], handle, ref hingeDescription);//                // Connect current segment with the previous one using a hinge joint//                var hingeDescription = new HingeDescription(//                    BodyReference.GetOffsetInstanceBodySpace(position-direction*segmentLength/2, simulation.Bodies.GetDescription(segmentHandles[i-1])),//                    BodyReference.GetOffsetInstanceBodySpace(position+direction*segmentLength/2, simulation.Bodies.GetDescription(handle)),//                    direction // Assuming hinge axis is along the chain//                );//   simulation.Solver.Add(segmentHandles[i-1], handle, ref hingeDescription);//    }}return segmentHandles;}*/

        //private static Hinge BuildHindge1DOF(BodySegment bodySegment, SpringSettings springSettings)
        //{
        //    // Connect current segment with the previous one using a hinge joint
        //    var hinge = new Hinge
        //    {
        //        LocalHingeAxisA=QuaternionEx.Transform(new Vector3(1, 0, 0), QuaternionEx.Conjugate(Quaternion.Identity)),
        //        LocalOffsetA=new Vector3(bodySegment.Length/2, 0, 0),
        //        LocalHingeAxisB=QuaternionEx.Transform(new Vector3(1, 0, 0), QuaternionEx.Conjugate(Quaternion.Identity)),
        //        LocalOffsetB=new Vector3(-bodySegment.Length/2, 0, 0),
        //        SpringSettings=springSettings // Customize your spring settings
        //    };
        //    return hinge;
        //}


        public Hinge BuildHinge(BodyHandle handleA, BodyHandle handleB, Vector3 localOffsetA, Vector3 localOffsetB, Vector3 direction, SpringSettings springSettings, Quaternion orientationA, Quaternion orientationB)
        {

            Vector3 orthoDir = new Vector3(-direction.Z, 0, direction.X);

            // Vector3 orthoDir = new Vector3(0, 1, 0);


            return new Hinge
            {
                LocalOffsetA=localOffsetA,
                LocalOffsetB=localOffsetB,
                LocalHingeAxisA=QuaternionEx.Transform(orthoDir, QuaternionEx.Conjugate(orientationA)),
                LocalHingeAxisB=QuaternionEx.Transform(orthoDir, QuaternionEx.Conjugate(orientationB)),
                SpringSettings=springSettings

            };
        }




        //    Then, let's modify the BuildChain function. The first joint will be a ball socket, and the rest will be hinges.
        //  This code will create a chain of segments with the specified density, adding a ball socket constraint for the first joint and hinge constraints for the rest.Note that you will need to add your own logic for setting the CollidableDescription and BodyActivityDescription in the BodyDescription.CreateDynamic call.




        /// <summary>
        /// try just adding a mass somewher to simulate a moement shift.. but it will probalby need to be a harmonic function to move on geodestic in correct phase.
        /// mabye deformaations on 3 ahxi or more wwill be tried.. we will borrow fromthe rag doll toehr types of joints likely and use iteractive CAD design ,
        /// maxied with manual
        /// 
        /// </summary>
        /// <param name="body"></param>
        /// <param name="segments"></param>
        /// <param name="segmentLength"></param>
        /// <param name="density"></param>
        /// <param name="startPos"></param>
        /// <param name="endPos"></param>
        /// <param name="simulation"></param>
        /// <returns></returns>
        //Here's how you might add an additional mass to your body. You'll want to create a new BodyDescription with the desired mass, and add it to your simulation:
        static BodyHandle AddSolenoidKicker(BodyHandle body, int segments, float segmentLength, float density, Vector3 startPos, Vector3 endPos, Simulation simulation)
        {
            float additionalMass = 5f;
            var additionalMassPosition = new Vector3(0, 0.5f, 0);  // Adjust as needed.
            var additionalMassShape = new Sphere(0.1f);  // Adjust as needed.
            var additionalMassInertia = new BodyInertia { InverseMass=1.0f/additionalMass };
            var additionalMassDescription = new BodyDescription
            {
                Pose=new RigidPose { Position=additionalMassPosition+additionalMassPosition },
                Collidable=new CollidableDescription { Shape=simulation.Shapes.Add(additionalMassShape) },
                LocalInertia=additionalMassInertia
            };
            var additionalMassHandle = simulation.Bodies.Add(additionalMassDescription);
            //    Then, you can connect this additional mass to your body using a constraint that allows movement along one axis, like a PointOnLineServo.You can then apply your SolenoidForce to this additional mass.
            return additionalMassHandle;


        }





        //public void ApplyThrusterForces(BodyHandle spacecraft, Vector3 thrustForce, Simulation simulation)
        //{


        //    // For each wing
        //    for (int i = 0; i<spacecraft.BodyParts.Length; i++)
        //    {
        //        // Get the body handle and its current pose
        //        BodyHandle bodyHandle = spacecraft.BodyParts[i];
        //        RigidPose pose = simulation.Bodies.GetPose(bodyHandle);

        //        // Compute the position of the thr


        //    }
        //}


        //public void ApplyThrusterForces(int wing1, int wind2, Vector3 thrustForce, Simulation simulation, Phenotype phenotype)
        //{



        //    // For each wing
        //    for (int i = 0; i<phenotype.BodyParts.Length; i++)
        //    {
        //        // Get the body handle and its current pose
        //        BodyHandle bodyHandle = phenotype.BodyParts[i];


        //        var bodyReference = Simulation.Bodies[reference.BodyHandle];
        //        RigidPose pose=bodyReference.Pose;
        //        // Compute the position of the thr


        //    }
        //}
    }
}






//        User
//        i thihnk so but to be clear think of a star fish but with jsut a long body, it will travel that way..and two legs or wings..teh throuster wil be on the wing tips with are triangles and teh bending will be so that the wings(probablby two  chains of boxes) can confrom to a large shpere if placed on it with gravity.. if it were placed on a flat surface or a concave sligth  pit, it woudl also wlex that way, it both teh main mody and the wings
//        ChatGPT
//I see, your spacecraft's design seems rather innovative! It is capable of adjusting its structure according to the surface it is placed on. It is meant to act as a sort of "flexible structure" that can conform to various shapes like a sphere, flat surface or a concave pit.

//The method to achieve this would be similar to the snake-like structure we discussed earlier, but with more of a starfish configuration. The body parts would be linked with hinge joints that allow them to adjust according to the surface. This flexible structure seems more akin to a soft robotics approach.The thrusters at the tips of the wings will then propel it in the direction needed.

//To simulate this, you will want to build a main body(spine) and the wings(legs) separately, then attach them together. Each segment should be linked with a hinge constraint to give it the freedom to move. The wings(legs) should also be able to rotate around their attachment points to the body, similar to the shoulder joint of a human.

//As for the structure adjusting to surfaces, the physics simulation will naturally make the structure rest in a position of lowest potential energy.So, when placed on a sphere, the spacecraft will adjust its shape to spread its weight evenly.This behavior should occur naturally in a physics simulation without the need for any extra code.

//Finally, the thrusters can be simulated by applying forces at the wing tips in the direction of the desired movement.Make sure to adjust the strength of these forces according to the thrust that the thrusters would produce.

//Keep in mind that this kind of dynamic simulation is quite complex and will require a lot of tweaking to get right.However, it is certainly feasible with the right physics engine and some patience.

//Please let me know if you have any more questions or if something is unclear!f you have any more questions or if something is unclear!