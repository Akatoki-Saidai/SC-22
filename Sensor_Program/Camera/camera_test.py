import cv2

camera = cv2.VideoCapture(0)
print('A')
while True:
  ret, frame = camera.read()
  print(ret)
  if not ret:
    break

  cv2.imshow("Frame", frame)
  key = cv2.waitKey(1)
  
  # Escキーを入力されたら画面を閉じる
  if key == 27:
    break
print('B')
camera.release()
cv2.destroyAllWindows()
