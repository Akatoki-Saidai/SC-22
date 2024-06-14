import cv2
import supervision as sv
from ultralytics import YOLOv10
import numpy as np
# Define the path to the weights file
# Load the model
model = YOLOv10("best.pt")

def process_webcam(output_file):
    cap = cv2.VideoCapture(0)  # Replace with 0 for the default webcam

    if not cap.isOpened():
        print("Error: Could not open camera .")
        return

    # Get the width, height, and fps of the input video
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Use 'XVID' for .avi
    out = cv2.VideoWriter(output_file, fourcc, fps, (width, height))

    bounding_box_annotator = sv.BoundingBoxAnnotator()
    label_annotator = sv.LabelAnnotator()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model(frame,conf=0.70)[0]
        #print(results.boxes.conf)
        #print(np.argmax(results.boxes.conf))
        detections = sv.Detections.from_ultralytics(results)

        annotated_frame = bounding_box_annotator.annotate(scene=frame, detections=detections)
        annotated_frame = label_annotator.annotate(scene=annotated_frame, detections=detections)

        # Write the annotated frame to the output file
        out.write(annotated_frame)

        cv2.imshow("Camera", annotated_frame)
        if cv2.waitKey(25) & 0xFF == ord("q"):
            break

    cap.release()
    out.release()
    cv2.destroyAllWindows()

def webcam(output_file: str = "Camera_Movie.mp4"):
    process_webcam(output_file)

if __name__ == "__main__":
    webcam()
