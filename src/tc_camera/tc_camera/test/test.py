import pyrealsense2 as rs

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
pipeline.start(config)

# Wait for the first frame to arrive
frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()

# Get the intrinsic matrix and distortion coefficients
intrinsics = depth_frame.get_profile().as_video_stream_profile().get_intrinsics()

fx = intrinsics.fx
fy = intrinsics.fy
ppx = intrinsics.ppx
ppy = intrinsics.ppy

distortion = intrinsics.coeffs

print("Intrinsic matrix:")
print("fx: ", fx)
print("fy: ", fy)
print("ppx: ", ppx)
print("ppy: ", ppy)

print("Distortion coefficients: ", distortion)

# Stop streaming
pipeline.stop()
