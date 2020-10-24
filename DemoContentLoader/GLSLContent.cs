namespace DemoContentLoader
{
    public class GLSLContent : IContent
    {
        public ContentType ContentType => ContentType.GLSL;
        public readonly string Source;

        public GLSLContent(string source) => Source = source;
    }
}
