# Buoyancy Plugin for Gazebo

This package provides a Gazebo plugin that simulates buoyancy for models in a ROS 2 environment.

## How it works

The plugin attaches to a model in Gazebo and applies buoyancy forces to specified links. The forces are calculated based on the fluid density, the submerged volume of the link, and the center of buoyancy. The plugin also applies linear and angular damping to simulate the drag of the fluid.

## Configuration

The plugin is configured through an SDF file. Here is an example of how to configure the plugin for a model:

```xml
<gazebo>
  <plugin name="buoyancy" filename="libbuoyancy_plugin.so">
    <fluid_density>997.0</fluid_density>
    <surface_z>0.0</surface_z>
    <link>
      <name>base_link</name>
      <waterplane_area>0.5</waterplane_area>
      <height>0.2</height>
      <linear_damping>10.0</linear_damping>
      <angular_damping>10.0</angular_damping>
      <k_righting>1000.0</k_righting>
      <cob>
        <x>0</x>
        <y>0</y>
        <z>0.1</z>
      </cob>
    </link>
  </plugin>
</gazebo>
```

### Parameters

- `fluid_density`: The density of the fluid in kg/m^3. Defaults to `997.0` (water).
- `surface_z`: The z-coordinate of the fluid surface in meters. Defaults to `0.0`.
- `link`: A block that configures a link to be affected by buoyancy. You can have multiple `link` blocks.
    - `name`: The name of the link.
    - `waterplane_area`: The area of the link at the waterplane in m^2.
    - `height`: The height of the link in meters.
    - `linear_damping`: The linear damping coefficient.
    - `angular_damping`: The angular damping coefficient.
    - `k_righting`: The righting moment coefficient. This is a simplified model to provide a restoring torque.
    - `cob`: The center of buoyancy relative to the link's center of gravity.

## Building

This is a regular ROS 2 package. You can build it using `colcon`:

```bash
colcon build --packages-select buoyancy_plugin
```

## Usage

To use the plugin, you need to add it to your model's urdf/xacro file as shown in the configuration example. You can then launch your Gazebo simulation as usual.
