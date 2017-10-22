using DemoRenderer;
using System;
using System.Collections.Generic;
using System.Text;

namespace Demos
{
    /// <summary>
    /// Constructs a demo from the set of available demos on demand.
    /// </summary>
    public class DemoSet
    {
        struct Option
        {
            public string Name;
            public Func<Camera, Demo> Builder;
        }

        List<Option> options = new List<Option>();
        void AddOption<T>(string name) where T : Demo, new()
        {
            options.Add(new Option
            {
                Builder = (camera) =>
                {
                    //Note that the actual work is done in the Initialize function rather than a constructor.
                    //The 'new T()' syntax actually uses reflection and repackages exceptions in an inconvenient way.
                    //By using Initialize instead, the stack trace and debugger will go right to the source.
                    var demo = new T();
                    demo.Initialize(camera);
                    return demo;
                },
                Name = name
            });
        }

        public DemoSet()
        {
            //Could use a text template or something here, but, on the other hand, ehhhhhhhhhhhhhhhhhhhhhhhhh
            AddOption<SimpleDemo>(nameof(SimpleDemo));
        }

        public string GetName(int index)
        {
            return options[index].Name;
        }

        public Demo Build(int index, Camera camera)
        {
            return options[index].Builder(camera);
        }
    }
}
