using Demos.Demos;
using Demos.SpecializedTests;
using System;
using Xunit;

namespace DemoTests
{
    public class Fountain
    {
        [Fact]
        public void Test()
        {
            TestUtilities.TestDeterminism<FountainStressTestDemo>(2, 4096);
        }
    }
}
