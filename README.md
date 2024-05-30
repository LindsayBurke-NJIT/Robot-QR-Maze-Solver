# Robot-QR-Maze-Solver
This project uses the LocoRobo LocoXtreme with an ultrasonic sensor and Raspberry Pi camera to navigate an arbitrary maze equipped with instructional QR codes.

<h2>Details</h2>
<ul>
  <li>QR Codes contain text that says either "right" or "left"</li>
  <li>QR Codes can be red or blue, with red meaning 50% turn speed and blue meaning 100% turn speed.</li>
</ul>

<h2>Tools</h2>
<ul>
  <li>Multiprocessing is used to simultaneously conduct a camera process and driving process. The camera process is responsible for color detection and QR code scanning. The driving process is in charge of handling logic in deciding which direction to drive.</li>
  <li>The OpenCV library is used for computer vision, which allows the robot to interpret the turn speed and direction from the QR codes.</li>
</ul>
