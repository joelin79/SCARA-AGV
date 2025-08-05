import pyrealsense2 as rs

import pyrealsense2 as rs

# Start pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline_profile = pipeline.start(config)

# Get stream profile and extract intrinsics
color_stream = pipeline_profile.get_stream(rs.stream.color)
video_profile = color_stream.as_video_stream_profile()
intrinsics = video_profile.get_intrinsics()

# Print intrinsic parameters
print("Width:", intrinsics.width)
print("Height:", intrinsics.height)
print("Fx:", intrinsics.fx)
print("Fy:", intrinsics.fy)
print("Ppx:", intrinsics.ppx)
print("Ppy:", intrinsics.ppy)
print("Distortion Model:", intrinsics.model)
print("Distortion Coefficients:", intrinsics.coeffs)

# Stop the pipeline
pipeline.stop()