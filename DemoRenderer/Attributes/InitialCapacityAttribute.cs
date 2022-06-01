using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DemoRenderer.Attributes
{
    internal class InitialCapacityAttribute : Attribute
    {
        public int Value { get; }
        public InitialCapacityAttribute(int value)
        {
            Value = value;
        }
    }
}
