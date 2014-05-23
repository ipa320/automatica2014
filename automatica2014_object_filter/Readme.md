Calibration


0. start bringup
roslaunch automatica2014_bringup scenario.launch or roslaunch automatica2014_bringup all.launch
roslaunch cob_fiducials fiducials.launch




1. put transformation from base_link (of cell) to cam3d_calibration_link in cell.urdf.xacro
2. restart bringup
3. detect marker
roslaunch automatica2014_object_filter calibration.launch
4. put transformation into cam3d_env_origi in call.urdf.xacro
    signs: 
        x_urdf = x_console
        y_urdf = -y_console
        z_urdf = z_console
5. restart bringup
6. check in rviz if link "marker" is equal to "cam3d_calibration_link"
