using System.IO;
using DemoContentLoader;

namespace DemoContentBuilder
{
    public static class GLSLBuilder
    {
        public static GLSLContent Build(Stream dataStream)
        {
            using (var reader = new StreamReader(dataStream))
                return new GLSLContent(reader.ReadToEnd());
        }
    }
}
