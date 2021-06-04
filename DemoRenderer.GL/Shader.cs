using System;
using OpenTK.Graphics.OpenGL4;

namespace DemoRenderer
{
    public class Shader : Disposable
    {
        private readonly int program = GL.CreateProgram();
        private readonly int vao = GL.GenVertexArray();

        private void Compile(ShaderType type, string source, Action action)
        {
            var handle = GL.CreateShader(type);
            try
            {
                GL.ShaderSource(handle, source);
                GL.CompileShader(handle);
                var error = GL.GetShaderInfoLog(handle);
                if (error != string.Empty) throw new Exception(error);
                GL.AttachShader(program, handle);
                try
                {
                    action();
                }
                finally
                {
                    GL.DetachShader(program, handle);
                }
            }
            finally
            {
                GL.DeleteShader(handle);
            }
        }

        public Shader(string vertex, string fragment) =>
            Compile(ShaderType.VertexShader, vertex, () =>
            Compile(ShaderType.FragmentShader, fragment, () =>
            {
                GL.LinkProgram(program);
                var error = GL.GetProgramInfoLog(program);
                if (error != string.Empty) throw new Exception(error);
            }));
        public void Use()
        {
            GL.UseProgram(program);
            GL.BindVertexArray(vao);
        }
        protected override void DoDispose()
        {
            GL.DeleteProgram(program);
            GL.DeleteVertexArray(vao);
        }
    }
}
