# autoware_fps_view_rviz_plugin

RViz view controller based on the default FPS view, with video-game style keyboard movement.

## How to use

1. Build and source the workspace, then start RViz.
2. In the **Views** panel, add **KeyboardFPS**. This also activates **KeyboardFPSTool**, which captures all keys so W/A/S/D do not switch other tools (for example **S** would otherwise select the Select tool).
3. Click the 3D view, then hold W/A/S/D/Q/E to move. Releasing a key stops that movement immediately.
4. Press Escape or click another toolbar tool to restore normal RViz shortcuts. That also stops camera motion and releases the keyboard so other tools and other applications can use it.

## Keyboard

| Key | Action |
| --- | --- |
| W / S | Forward / reverse |
| A / D | Left / right |
| Q / E | Down / up |
| Shift | Speed boost |
| F | Focus on the object under the cursor (default RViz) |
| Z | Reset the view (default RViz) |
| v | Activate KeyboardFPSTool |

## Properties

These are editable in **Panels > Tool Properties** (side panel) while KeyboardFPSTool is selected, and also in the **Views** panel:

- **Movement Speed**: meters per second while a key is held (default `1.0`)
- **Boost Factor**: extra multiplier while Shift is held (`speed * (1 + boost)`)
- **Fly Mode**: when enabled, movement follows the look direction; when disabled, WASD stays on the XY plane and Q/E change world height
