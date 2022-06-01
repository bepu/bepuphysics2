using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DemoRenderer.Attributes
{
    internal class BoxIndicesAttribute : Attribute
    {
        public int Count { get; }
        public BoxIndicesAttribute(int count)
        {
            Count = count;
        }
    }
}
