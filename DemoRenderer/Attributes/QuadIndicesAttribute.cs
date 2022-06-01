using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DemoRenderer.Attributes
{
    internal class QuadIndicesAttribute : Attribute
    {
        public int Count { get; }
        public QuadIndicesAttribute(int count)
        {
            Count = count;
        }
    }
}
