using System;
using System.Diagnostics;

namespace DemoRenderer
{
    public abstract class Disposable : IDisposable
    {
#if DEBUG
        ~Disposable()
        {
            Debug.Fail($"An object of type {GetType()} was not disposed prior to finalization.");
        }
#endif
        protected abstract void DoDispose();
        public void Dispose()
        {
            DoDispose();
            GC.SuppressFinalize(this);
        }
    }
}
