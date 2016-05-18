def Create3dLine(axes, style=None):
    """Creates a new 3D line on the given axes."""
    if style is None:
        line = axes.plot((0, 0), (0, 0), (0, 0))[0]
    else:
        line = axes.plot((0, 0), (0, 0), (0, 0), **style)[0]

    return line


def DrawFrame():
    """Draw the current frame of the simulation onto the plot."""
    R = CalculateR(attitude)

    # Transform the arms into the inertia frame and store them as a flattened
    # list.
    px = (R * xArm).A.flatten()
    py = (R * yArm).A.flatten()
    up = (R * upArm).A.flatten()

    x = position[0, 0]
    y = position[1, 0]
    z = position[2, 0]

    # Plot both arms in the body-frame's x axis.
    xLine.set_data((x-px[0], x+px[0]), (y-px[1], y+px[1]))
    xLine.set_3d_properties((z-px[2], z+px[2]))

    # Plot both arms in the body-frame's y axis.
    yLine.set_data((x-py[0], x+py[0]), (y-py[1], y+py[1]))
    yLine.set_3d_properties((z-py[2], z+py[2]))

    # Plot the up vector
    upLine.set_data((x+px[0], x+up[0]), (y+px[1], y+up[1]))
    upLine.set_3d_properties((z+px[2], z+up[2]))

    axes.set_xlim(x-margins, x+margins)
    axes.set_ylim(y-margins, y+margins)
    axes.set_zbound(z-margins, z+margins)