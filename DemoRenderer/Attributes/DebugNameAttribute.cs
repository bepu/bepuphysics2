using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DemoRenderer.Attributes
{
    internal class DebugNameAttribute : Attribute
    {
        public string Name { get; }
        public DebugNameAttribute(string name)
        {
            Name = name;
        }
    }
}
