using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DemoRenderer.Attributes
{
    internal class ResourceAttribute : Attribute
    {
        public string Path { get; }
        public ResourceAttribute(string path)
        {
            Path = path;
        }
    }
}
