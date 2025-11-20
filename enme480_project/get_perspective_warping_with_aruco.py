import cv2
import numpy as np
import keyboard

# Mouse callback function
def mouse_callback(event, x, y, flags, param):
    global selected_points, frame

    if event == cv2.EVENT_LBUTTONDOWN and len(selected_points) < 4:
        selected_points.append((x, y))

    return frame

 
if __name__ == '__main__':
    selected_points = []
    frame = None
    # Turn on Laptop's webcam
    cap = cv2.VideoCapture("/dev/video0")
    
    ret, frame = cap.read()

    while ret:
        ret, frame = cap.read()
   
        if len(selected_points) > 0:
            points_in_image_frame = selected_points
            for ii in range(len(selected_points)):
                cv2.circle(frame, selected_points[ii], 3, (0, 0, 255), -1)

        ####################################### YOUR CODE STARTS HERE #######################################

        # Fill up the array with the points you have chosen. The sequence of points must be similar to the sequence in which you will be clicking on the image!

        points_in_table_frame = np.array([[], [], [], []], dtype=np.float32)

        ####################################### YOUR CODE ENDS HERE #######################################
        
        if len(selected_points) == 4:
            points_in_image_frame = np.array(points_in_image_frame, dtype=np.float32).reshape(4,2)
            perspective_matrix = cv2.getPerspectiveTransform(points_in_image_frame, points_in_table_frame)
            result = cv2.warpPerspective(frame, perspective_matrix, (550, 450))

            cv2.circle(result, (175, 175), 3, (255, 0, 0), -1)
            cv2.imshow('frame1', result) # Transformed Capture
    
        # Wrap the transformed image
        cv2.imshow('Camera Frame', frame) # Initial Capture
        frame = cv2.setMouseCallback("Camera Frame", mouse_callback)

        if cv2.waitKey(int(1 / 60 * 1000)) & 0xFF == ord('q'):
            np.save("perspective_matrix.npy", perspective_matrix)
            break

    cap.release()
    cv2.destroyAllWindows()
