using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace DemoContentBuilder
{
    /// <summary>
    /// Holds already-loaded files in memory to avoid hitting the disk repeatedly.
    /// </summary>
    /// <remarks>Because memory really isn't an issue.</remarks>
    public class ShaderFileCache
    {
        Dictionary<string, string> cache = new Dictionary<string, string>();

        public bool TryLoad(string path, out string shaderText)
        {
            lock (cache)
            {
                if (cache.TryGetValue(path, out shaderText))
                {
                    return true;
                }
            }

            //Have to look it up on disk.
            //Note that other threads may currently be looking it up too. We could have a waiting system, but the error handling involved in that is gross. The cache should be hit most of the time.
            //This is an optimization, after all, not a correctness issue.
            if (File.Exists(path))
            {
                using (var stream = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read))
                {
                    using (StreamReader reader = new StreamReader(stream))
                    {
                        shaderText = reader.ReadToEnd();
                    }
                }
                lock (cache)
                {
                    //Add or replace the path in the cache.
                    cache[path] = shaderText;
                }
                return true;
            }
            //This file does not exist.
            return false;


        }
    }
}
