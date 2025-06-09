# Example Package

## Plane Calibration

Run the object classification launch file with debugging enabled and ensure the table is empty:

```bash
ros2 launch example object_classification.launch.py debug:=true
```

After several frames the node computes the plane distance and coefficients and saves them to `config/object_classification_plane_distance.yaml`. Debug mode then disables automatically.
