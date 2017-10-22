namespace BepuUtilities.Collections
{
    /// <summary>
    /// Defines a type able to match an element.
    /// </summary>
    /// <typeparam name="T">Type of the object to match.</typeparam>
    public interface IPredicate<T>
    {
        //We're assuming here that the inlining will be good enough that we won't pay extra for passing by ref under any circumstance. This isn't always the case.
        bool Matches(ref T item);
    }


}
