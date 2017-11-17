using System;
using System.Collections.Generic;
using System.Text;

namespace BepuPhysics
{
    /// <summary>
    /// Provides functionality for efficiently activating previously deactivated bodies and their associated islands.
    /// </summary>
    public class IslandActivator
    {
        /// <summary>
        /// Activates a body if it is inactive. All bodies that can be found by traversing the constraint graph from the body will also be activated.
        /// If the body is already active, this does nothing.
        /// </summary>
        /// <param name="bodyHandle">Handle of the body to activate.</param>
        public void ActivateBody(int bodyHandle)
        {
        }

        /// <summary>
        /// Activates any inactive bodies associated with a constraint. All bodies that can be found by traversing the constraint graph from the constraint referenced bodies will also be activated.
        /// If all bodies associated with the constraint are already active, this does nothing.
        /// </summary>
        /// <param name="constraintHandle"></param>
        public void ActivateConstraint(int constraintHandle)
        {

        }
    }
}
