# Description resources

W06 launch reads the frozen external T1_23dof.urdf, resolves external mesh URIs,
and adds the identity base_link→trunk alias in memory. No upstream asset is copied.
`state.rviz` uses odom as Fixed Frame and displays the basic model, axes, odom and TF.
Launch with `ros2 launch t1_bringup simulation.launch.py rviz:=true`.

W07 still owns complete FK validation, footprint and scene integration.
LiDAR mounting/frame is reserved for W08; SLAM/navigation are not enabled.
