# STM32 LIDAR 3D Mapper

A real-time 3D LIDAR scanning system using an STM32 Nucleo F401RE microcontroller with a servo-controlled LD19 LIDAR sensor.

## System Overview

- **Microcontroller**: STM32 Nucleo F401RE
- **LIDAR Sensor**: LD19 (230400 baud, 12-point packet structure)
- **Servo Motor**: For elevation/pitch control (0-180°)
- **Communication**: USB Serial (230400 baud) to PC for data visualization
- **Visualization**: Real-time 3D point cloud plotting (Python matplotlib)

## Hardware Setup

### Pin Connections

#### LIDAR LD19 to STM32 Nucleo F401RE

| Component | Pin | Board Location | Notes |
|-----------|-----|-----------------|-------|
| LIDAR TX (data out) | **D2 (PA10)** | CN9 | **RX pin - receives data from LIDAR** |
| LIDAR RX (config in) | **D8 (PA9)** | CN9 | **TX pin - optional LIDAR configuration** |
| LIDAR GND | GND | CN9 | Ground |
| LIDAR VCC | +5V | CN6 | Power supply |

#### Servo Motor to STM32

| Component | Pin | Board Location |
|-----------|-----|-----------------|
| Servo Signal | **D7 (PB10)** | CN5/CN9 |
| Servo GND | GND | CN6 |
| Servo VCC | +5V | CN6 |

**⚠️ CRITICAL**: The LIDAR TX output must connect to D2 (PA10), which is the RX pin for HardwareSerial. Incorrect pin connections will result in no data being received.

## Software

### Firmware (Arduino/PlatformIO)

**File**: `src/main.cpp`

#### Key Functions:
- `setup()`: Initialize Serial2 (LIDAR @ 230400 baud) and USB Serial
- `loop()`: Read LIDAR packets, control servo sweep, output parsed data
- `processPacket()`: Parse LD19 packet format, calculate angles, validate CRC
- `CalcCRC()`: Verify packet integrity using 8-bit checksum

#### Output Format:
```
P <servo_angle> <distance_mm> <intensity> <lidar_angle_deg>
```

**Example**:
```
P 45 1250 255 180.50
```
- Servo angle: 45°
- Distance: 1250 mm
- Intensity: 255 (confidence)
- LIDAR angle: 180.50°

### Python Visualization Tools

#### `verify_lidar.py`
Diagnostic tool to verify data reception without visualization.
```bash
python verify_lidar.py
```
**Output**: Raw data from LIDAR or connection errors

#### `visualize_lidar.py`
Real-time 3D point cloud visualization.
```bash
python visualize_lidar.py
```
- Auto-detects STM32 board or prompts for COM port
- Displays 3D scatter plot of LIDAR points
- Updates every 50 points for responsive UI
- Close plot window to exit

#### Coordinate System

Points are converted from spherical (distance, elevation, azimuth) to Cartesian (X, Y, Z):

```python
elevation = servo_angle - 90°  # 0° = horizon
azimuth = lidar_angle
x = distance * cos(elevation) * cos(azimuth)
y = distance * cos(elevation) * sin(azimuth)
z = distance * sin(elevation)
```

## Build & Upload

### Using PlatformIO (CLI)

```bash
# Build
platformio run

# Upload to board
platformio run --target upload --upload-port COM8

# Monitor serial output
platformio device monitor --port COM8 --baud 230400
```

### Using PlatformIO (VS Code)

1. Build: `Ctrl+Alt+B`
2. Upload: `Ctrl+Alt+U`
3. Monitor: Click "PIO" → "Monitor" in the terminal

## Troubleshooting

### No Data Received

1. **Check pin connections** - Most common issue
   - LIDAR TX → D2 (PA10), not any other pin
   - Use multimeter to verify continuity
   
2. **Verify baud rate** - Should be 230400
   - Check firmware: `#define LIDAR_BAUD 230400`
   - Check Python script serial port setup
   
3. **Test with verification script**:
   ```bash
   python verify_lidar.py
   ```
   - Should show "RX: P <values>" messages
   - If blank, LIDAR is not transmitting

4. **Enable debug output** in firmware:
   - Already included in startup messages
   - Check `Serial.println()` in setup()

### COM Port Issues

- **"PermissionError"**: Port is locked by another process
  - Close PlatformIO monitor, other serial tools
  - Restart the script
  
- **Port not found**: STM32 driver missing
  - Install ST-Link drivers: https://www.st.com/en/development-tools/st-link-v2.html
  - Restart VS Code/computer

### Visualization Issues

- **No points in 3D plot**: Frames not reaching Python
  - Verify `verify_lidar.py` shows data first
  - Check coordinate conversion in `visualize_lidar.py`
  
- **Plot frozen/unresponsive**: Matplotlib issue
  - Close and rerun script
  - Reduce plotting rate by increasing batch size

## LD19 LIDAR Protocol

### Packet Structure (47 bytes)

| Byte(s) | Field | Format | Description |
|---------|-------|--------|-------------|
| 0 | Header | 0x54 | Fixed start byte |
| 1 | VerLen | Fixed | Protocol version/length |
| 2-3 | Speed | uint16_t LE | Rotation speed (RPM) |
| 4-5 | StartAngle | uint16_t LE | Start angle (0.01° units) |
| 6-41 | Data[12][3] | 3×uint8 | 12 points: 2 bytes dist + 1 byte intensity |
| 42-43 | EndAngle | uint16_t LE | End angle (0.01° units) |
| 44-45 | Timestamp | uint16_t | Packet timestamp |
| 46 | CRC | uint8_t | Sum of bytes 0-45 |

**Angle Calculation**: `actual_angle_deg = raw_value / 100`

## Dependencies

### Firmware
- Arduino framework (PlatformIO)
- `Servo` library (included)

### Python
```bash
pip install pyserial numpy matplotlib
```

## Performance

- **Update Rate**: ~12 Hz (LD19 packet rate)
- **Range**: 0.05 - 12 m
- **Field of View**: 360° (horizontal) × servo range (vertical)
- **Max Servo Speed**: 1 degree per 15ms ≈ 67°/sec

## Future Improvements

- [ ] Add configuration commands to LIDAR
- [ ] Implement point cloud filtering (confidence threshold)
- [ ] Save point clouds to PCD/PLY format
- [ ] ROS2 integration (see `ros2_ws/` directory)
- [ ] Web-based 3D viewer

## File Structure

```
2D_Lidar-mapping/
├── platformio.ini          # Build configuration
├── README.md              # This file
├── verify_lidar.py        # Diagnostic tool
├── visualize_lidar.py     # 3D visualization
├── src/
│   └── main.cpp           # Firmware source
├── include/               # Header files
├── lib/                   # Libraries
├── test/                  # Tests
├── scripts/
│   └── lidar_3d_viewer.py # Alternative viewer
└── ros2_ws/              # ROS2 integration (optional)
```

## License

[Specify your license here]

## Support

For issues or questions:
1. Check troubleshooting section above
2. Review pin connections
3. Test with `verify_lidar.py` first
4. Check firmware startup messages via serial monitor

---

**Last Updated**: December 2025
