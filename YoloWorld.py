from ultralytics import YOLOWorld

# Initialize a YOLO-World model
model = YOLOWorld("yolov8s-world.pt")

# Define custom classes
model.set_classes(["block"])

# Execute prediction on an image
results = model.predict("C:/Users/admin/OneDrive/文件/S_10493966_0.jpg")

# Show results
results[0].show()