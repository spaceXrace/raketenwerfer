import cv2
import mediapipe as mp

# Initialize MediaPipe solutions
mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
pose = mp_pose.Pose(min_detection_confidence=0.75, min_tracking_confidence=0.75)

def main():
    # Open the webcam
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    try:
        while cap.isOpened():
            ret, frame = cap.read()

            if not ret:
                print("Error: Failed to capture frame.")
                break

            # Convert the frame to RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Human detection using MediaPipe Pose
            results = pose.process(rgb_frame)

            XPos, YPos = 0, 0

            if results.pose_landmarks:
                # Get head landmarks
                nose = results.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE]
                left_eye = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_EYE]
                right_eye = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_EYE]

                h, w, _ = frame.shape

                # Calculate bounding box around the head based on landmarks
                x_min = int(min(nose.x, left_eye.x, right_eye.x) * w)
                x_max = int(max(nose.x, left_eye.x, right_eye.x) * w)
                y_min = int(min(nose.y, left_eye.y, right_eye.y) * h)
                y_max = int(max(nose.y, left_eye.y, right_eye.y) * h)

                head_width = x_max - x_min
                head_height = y_max - y_min

                closest_x = int(nose.x * w)
                closest_y = int(nose.y * h)

                XPos = (nose.x - 0.5) * 2
                YPos = -1 * (nose.y - 0.5) * 2

                # Draw landmarks and bounding box
                mp_drawing.draw_landmarks(
                    frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                    mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
                    mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=2, circle_radius=2),
                )

                cv2.rectangle(
                    frame,
                    (x_min, y_min),
                    (x_max, y_max),
                    (0, 255, 0), 2
                )

            # Display the frame
            cv2.imshow('Human Detection', frame)

            # Break the loop on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Programm beendet.")

    # Release resources
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
