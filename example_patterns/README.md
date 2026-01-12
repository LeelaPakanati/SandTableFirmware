# .thr Pattern File Format

## Overview
The `.thr` format contains polar coordinates (theta, rho) for the Sisyphus table to follow.

## File Format
```
# Comments start with # or //
theta1 rho1
theta2 rho2
...
```

### Parameters
- **theta**: Angle in radians (typically 0 to 2π ≈ 6.28)
- **rho**: Normalized radius from 0.0 to 1.0
  - 0.0 = center (0mm)
  - 0.5 = halfway out (225mm)
  - 1.0 = outer edge (450mm)

### Supported Separators
- Space: `1.57 0.5`
- Comma: `1.57,0.5`
- Tab: `1.57	0.5`

## Example Files Included

### circle.thr
A simple circle at 50% radius (225mm from center)

### spiral.thr
An inward spiral from the outer edge to the center

## Uploading Files to ESP32

### Method 1: Using PlatformIO
1. Place your .thr file in the `data/` folder
2. Run: `pio run --target uploadfs`

### Method 2: Using ESP32 File Upload Plugin
1. Install ESP32FS plugin for Arduino IDE
2. Place files in `data/` folder
3. Use Tools → ESP32 Sketch Data Upload

### Method 3: Programmatically via Serial/Web
Upload files through a web interface or serial commands (requires custom implementation)

## Usage in Code

### Basic Usage
```cpp
void loop() {
  if (!polarControl.processNextMove()) {
    // Load and run a pattern file
    if (polarControl.loadAndRunFile("/circle.thr")) {
      Serial.println("Pattern started!");
    }
  } else {
    delay(10);  // Allow time for motion processing
  }
}
```

### Custom Scaling
```cpp
// Scale to different max radius (e.g., 400mm instead of 450mm)
polarControl.loadAndRunFile("/pattern.thr", 400.0);
```

## Creating Custom Patterns

### Python Script Example
```python
import math

# Generate a rose pattern
with open('rose.thr', 'w') as f:
    f.write("# Rose pattern\\n")
    for i in range(628):  # 0 to 2π with 100 steps
        theta = i / 100.0
        rho = abs(math.sin(3 * theta))  # 3-petal rose
        f.write(f"{theta:.3f} {rho:.3f}\\n")
```

### MATLAB/Octave Example
```matlab
theta = linspace(0, 2*pi, 100);
rho = abs(sin(5*theta));  % 5-petal rose

fid = fopen('rose.thr', 'w');
fprintf(fid, '# Rose pattern\\n');
for i = 1:length(theta)
    fprintf(fid, '%.3f %.3f\\n', theta(i), rho(i));
end
fclose(fid);
```

## Tips

1. **Resolution**: More points = smoother motion but larger file
   - Recommended: 50-200 points per full rotation
   - High detail: 300-500 points

2. **File Size**: ESP32 LittleFS has limited space
   - Keep files under 100KB for best performance
   - Compress repetitive patterns programmatically

3. **Smooth Transitions**: Ensure consecutive points don't jump too far
   - Large jumps may cause rapid acceleration
   - Motion planner will handle acceleration automatically

4. **Return to Start**: Add final point matching the first for seamless loops
   ```
   0.0 0.5
   ...
   6.28 0.5
   0.0 0.5  # Return to start
   ```

5. **Testing**: Start with simple patterns (circle.thr) to verify scaling
