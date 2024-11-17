# AprilTag 
AprilTag je vizuálny fiduciálny systém, ktorý je užitočný pre širokú škálu úloh, vrátane rozšírenej reality, robotiky a kalibrácie kamier. Ciele môžu byť vytvorené pomocou bežnej tlačiarne a softvér na detekciu AprilTag vypočítava presnú 3D pozíciu, orientáciu a identitu tagov vo vzťahu k kamere. Knižnica AprilTag je implementovaná v jazyku C bez externých závislostí. Je navrhnutá tak, aby sa ľahko integrovala do iných aplikácií a bola prenosná aj na vstavané zariadenia. Výkon v reálnom čase možno dosiahnuť aj na procesoroch na úrovni mobilných telefónov. 

**GitHub Repo:** [AprilRobotics](https://github.com/AprilRobotics/apriltag)

### Technical Description
![image](https://github.com/user-attachments/assets/d39981f3-a413-46a8-ac0a-bdeb5ba20779)

- Dizajn fiducialnych značiek a kódovací systém sú založené na takmer optimálnom lexikografickom kódovacom systéme, pričom detekčný softvér je robustný voči svetelným podmienkam a uhlu pohľadu. Tento systém je podrobnejšie opísaný v týchto článkoch o AprilTag (AprilTag: [Robustný a flexibilný vizuálny fiduciálny systém](https://april.eecs.umich.edu/papers/details.php?name=olson2011tags), ICRA 2011; AprilTag 2: [Efektívna a robustná detekcia fiducialov](https://april.eecs.umich.edu/papers/details.php?name=wang2016iros), IROS 2016; [Flexible Layouts for Fiducial Tags, v recenznom konaní](https://april.eecs.umich.edu/papers/details.php?name=krogius2019iros)). Jednou z užitočných aplikácií AprilTagov je kalibrácia kamier (AprilCal: [Asistovaná a opakovateľná kalibrácia kamier](https://april.eecs.umich.edu/papers/details.php?name=richardson2013iros), IROS 2013).
- AprilTagy sú koncepčne podobné QR kódom, pretože sú typom dvojrozmerného čiarového kódu. Sú však navrhnuté tak, aby kódovali oveľa menšie množstvo dát (medzi 4 a 12 bitov), čo umožňuje ich robustnejšiu detekciu a z väčšej vzdialenosti. Navyše sú optimalizované pre vysokú lokalizačnú presnosť – môžete vypočítať presnú 3D pozíciu AprilTagu vo vzťahu ku kamere.
- ARToolkit má veľmi podobný cieľ ako AprilTagy, avšak, ako ukazujú naše technické články, AprilTag prekonáva ARToolkit z hľadiska miery detekcie a presnosti.

## AprilTag Detector

ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="/dev/video0" -p pixel_format:="mjpeg2rgb" -p camera_info_url:="file://$HOME/calibration_data/ost.yaml" -r image_raw:=/camera/image_raw -r camera_info:=/camera/camera_info 

ros2 run apriltag_ros apriltag_node --ros-args -r /camera/image_rect:='/camera/image_raw' -r /camera/camera_info:='/camera/camera_info_throttled' --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml

ros2 run topic_tools throttle messages /camera/camera_info 5.0 /camera/camera_info_throttled


[image_transport] Topics '/image_rect' and '/camera_info' do not appear to be synchronized. In the last 10s:
