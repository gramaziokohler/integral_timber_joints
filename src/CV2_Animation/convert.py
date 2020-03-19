# Install Open CV for python if you don't have it:
# pip install opencv-python
import cv2
import os

# Settings
image_folder = 'images'
video_name = 'video.avi'
fourcc = cv2.VideoWriter_fourcc(*'H264')
frame_per_second = 10

images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
images.sort() # Sort by file name. Avoid mixing 0, 1, 2, with 10, 11, 12. Pad hsorter numbers with zeros. 
frame = cv2.imread(os.path.join(image_folder, images[0]))
height, width, layers = frame.shape

video = cv2.VideoWriter(video_name, fourcc, frame_per_second, (width,height))

for counter, image in enumerate(images):
    video.write(cv2.imread(os.path.join(image_folder, image)))
    print('Image %s of %s (%s)         '% (counter,len(images), image), end='\r')
print('Convertion completed')
cv2.destroyAllWindows()
video.release()
