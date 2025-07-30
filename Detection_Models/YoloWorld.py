from ultralytics import YOLOWorld
import ssl

ssl._create_default_https_context = ssl._create_unverified_context

# Initialize a YOLO-World model
model = YOLOWorld("../yolov8s-world.pt")

# Define custom classes
model.set_classes(["small green blocks"])

# Execute prediction on an image
results = model.predict(
    source="/Users/joelin/Downloads/IMG_7724.jpeg",
    conf=0.1
)
# Show results
results[0].show()