# AprilTag 
AprilTag je vizuálny fiduciálny systém, ktorý je užitočný pre širokú škálu úloh, vrátane rozšírenej reality, robotiky a kalibrácie kamier. Ciele môžu byť vytvorené pomocou bežnej tlačiarne a softvér na detekciu AprilTag vypočítava presnú 3D pozíciu, orientáciu a identitu tagov vo vzťahu k kamere. Knižnica AprilTag je implementovaná v jazyku C bez externých závislostí. Je navrhnutá tak, aby sa ľahko integrovala do iných aplikácií a bola prenosná aj na vstavané zariadenia. Výkon v reálnom čase možno dosiahnuť aj na procesoroch na úrovni mobilných telefónov. 

**Official Website**: [Apriltag](https://april.eecs.umich.edu/software/apriltag) | 
**GitHub Repo:** [AprilRobotics](https://github.com/AprilRobotics/apriltag) |
**Developed By**: [The APRIL Robotics Laboratory at the University of Michigan](https://april.eecs.umich.edu) 

### Technical Description
![image](https://github.com/user-attachments/assets/d39981f3-a413-46a8-ac0a-bdeb5ba20779)

- Dizajn fiducialnych značiek a kódovací systém sú založené na takmer optimálnom lexikografickom kódovacom systéme, pričom detekčný softvér je robustný voči svetelným podmienkam a uhlu pohľadu. Tento systém je podrobnejšie opísaný v týchto článkoch o AprilTag (AprilTag: [Robustný a flexibilný vizuálny fiduciálny systém](https://april.eecs.umich.edu/papers/details.php?name=olson2011tags), ICRA 2011; AprilTag 2: [Efektívna a robustná detekcia fiducialov](https://april.eecs.umich.edu/papers/details.php?name=wang2016iros), IROS 2016; [Flexible Layouts for Fiducial Tags, v recenznom konaní](https://april.eecs.umich.edu/papers/details.php?name=krogius2019iros)). Jednou z užitočných aplikácií AprilTagov je kalibrácia kamier (AprilCal: [Asistovaná a opakovateľná kalibrácia kamier](https://april.eecs.umich.edu/papers/details.php?name=richardson2013iros), IROS 2013).
- AprilTagy sú koncepčne podobné QR kódom, pretože sú typom dvojrozmerného čiarového kódu. Sú však navrhnuté tak, aby kódovali oveľa menšie množstvo dát (medzi 4 a 12 bitov), čo umožňuje ich robustnejšiu detekciu a z väčšej vzdialenosti. Navyše sú optimalizované pre vysokú lokalizačnú presnosť – môžete vypočítať presnú 3D pozíciu AprilTagu vo vzťahu ku kamere.
- ARToolkit má veľmi podobný cieľ ako AprilTagy, avšak, ako ukazujú naše technické články, AprilTag prekonáva ARToolkit z hľadiska miery detekcie a presnosti.

## Environment Setup
Predtým, ako začneme používať akékoľvek balíky v ROS2, je potrebné nastaviť pracovné prostredie, v ktorom budeme tieto balíky testovať. Skúsil som niekoľko spôsobov inštalácie: najprv som sa pokúsil nainštalovať ROS2 Humble na Windows cez WSL, a následne som sa rozhodol nainštalovať ROS2 Humble v virtuálnom stroji pomocou VirtualBoxu, pretože pri práci s WSL som narazil na problémy s pripojením vstavané kamery. Nižšie sú uvedené kroky inštalácie pre oba prístupy:

> Je potrebné stiahnuť Ubuntu 22.04 (Jammy Jellyfish), pretože ROS2 Humble je podporovaný iba na tejto verzii, a taktiež používame práve Humble, pretože balíky NVIDIA ISAAC ROS sú podporované iba na tejto verzii. Je to spravodlivé porovnávacie prostredie pre balíky AprilTag, ktoré bežia na CPU, a pre AprilTag NVIDIA ISAAC ROS, ktoré bežia na GPU a sú navrhnuté na urýchlené spracovanie dát.
    
### WSL 
1. **Kontrola systémových požiadaviek**: Na používanie WSL je potrebné, aby na vašom počítači bola nainštalovaná Windows 10 verzie 2004 alebo vyššej, alebo Windows 11. Takisto sa uistite, že máte zapnuté funkcie virtualizácie v BIOS-e.

2. **Inštalácia WSL**: Otvorte Windows PowerShell s právami správcu a zadajte nasledujúci príkaz, potom stlačte Enter:
   ```bash
   wsl --install
   ```
   Tento príkaz automaticky nainštaluje WSL a všetky potrebné komponenty.

3. **Aktivácia potrebných komponentov Windows**: Prejdite do Ovládacieho panela, otvorte položku «Programy a funkcie» a vyberte «Zapnúť alebo vypnúť komponenty Windows». Uistite sa, že sú aktivované tieto komponenty: `Virtual Machine Platform`, `Windows Hypervisor Platform`, `Windows Subsystem for Linux`. Potom kliknite na «OK» a reštartujte operačný systém Windows.

4. **Inštalácia Ubuntu cez Microsoft Store**: Po reštartovaní systému otvorte Microsoft Store, vyhľadajte verziu `Ubuntu 22.04 LTS` (alebo inú stabilnú verziu, ktorá vám vyhovuje) a nainštalujte ju. Počas inštalácie vám bude ponúknuté, aby ste si vybrali používateľské meno a heslo pre váš nový systém.

5. **Spustenie Ubuntu**: Po dokončení inštalácie otvorte terminál Ubuntu, ktorý sa objaví v ponuke «Štart» (alebo jednoducho zadajte «Ubuntu» do vyhľadávania Windows). V termináli Ubuntu odporúčame vykonať nasledujúce príkazy:
   ```bash
   sudo apt update
   sudo apt upgrade
   sudo apt autoremove
   ```
   (Pre udržiavanie systému aktuálneho).

6. **Aktualizácia a nastavenie WSL 2**: Po inštalácii sa bude predvolene používať WSL 1. Ak chcete prepnúť na WSL 2, ktorý je rýchlejší a podporuje Docker a ďalšie nástroje pre vývoj, vykonajte nasledujúce kroky. Otvorte Windows PowerShell s právami správcu a zadajte príkaz:
   ```bash
   wsl --set-default-version 2
   ```
   Aby ste sa uistili, že WSL 2 je aktivovaný, zadajte príkaz:
   ```bash
   wsl --list --verbose
   ```
   Tento príkaz zobrazí zoznam nainštalovaných distribúcií a ich verzie.

7. **Inštalácia ROS2 Humble**: Otvorte terminál WSL, ktorý sa objaví v ponuke «Štart» (alebo jednoducho zadajte «wsl» do vyhľadávania Windows). Potom nainštalujte metaooperačný systém ROS2 Humble podľa pokynov v oficiálnej [dokumentácii](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) na inštaláciu. 
8. **Vytvorenie pracovného priestoru**: Pre vytvorenie pracovného priestoru v ROS2 postupujte nasledovne. Vytvorte nový adresár pre pracovný priestor, napríklad `perception_ws`, a prejdite doň:
   ```bash
   mkdir -p ~/perception_ws/src
   cd ~/perception_ws/src
   ```
   Potom vykonajte príkaz na inicializáciu pracovného priestoru:
   ```bash
   colcon build
   ```
   Následne je potrebné aktivovať pracovný priestor:
   ```bash
   source ~/perception_ws/install/setup.bash
   ```
   Teraz môžete pridávať balíky do adresára `src` a kompilovať ich pomocou príkazu `colcon build`.

9. **Inštalácia balíkov**: Aby ste mohli otestovať balík [AprilTag_Ros](https://github.com/christianrauch/apriltag_ros), je potrebné najprv vykonať kalibráciu kamery. Na to nainštalujte balík na kalibráciu kamery pomocou nasledujúceho príkazu:
   ```bash
   sudo apt install ros-humble-camera-calibration
   ```
   Takisto na zapnutie kamery a zabezpečenie kompatibility s ROS2 je potrebné nainštalovať jeden z ovládačov, ako napríklad [usb_cam](https://github.com/ros-drivers/usb_cam) alebo [opencv_cam](https://github.com/clydemcqueen/opencv_cam). 
10. **Problém**: Pri pokuse o zapnutie kamery cez balíky `usb_cam` alebo `opencv_cam` sa objavil problém, ktorý spočíval v potrebe nainštalovať novú verziu jadra Linuxu, pretože WSL neposkytuje priamy prístup k jadru Linuxu a zariadeniam pripojeným k systému. Na vyriešenie tohto problému som našiel [návod](https://iz6.ru/semka-video-s-vebkamery-s-pomoshhju-opencv-v-wsl2/), ktorý popisuje, ako nainštalovať správnu verziu jadra, a tento návod mi pomohol odstrániť ťažkosti. Hlavná vec, ktorú treba vedieť, je, že v súbore `.wslconfig` je potrebné nastaviť minimálne 4 GB operačnej pamäte alebo viac, pretože táto verzia vyžaduje značné množstvo pamäte. Tu je príklad konfigurácie pre súbor `.wslconfig`:

    ```
    [wsl2]
    memory=6GB
    processors=2
    swap=0
    localhostForwarding=true
    ```
11. **Pokus o kalibráciu kamery**: Po úspešnej zostave jadra môžeme použiť príkaz `usbipd` na pripojenie a priradenie USB zariadenia v WSL. To nám umožní zapnúť kameru cez WSL a vyskúšať jej kalibráciu pomocou príkazu:

    ```bash
    ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.015 \
      --ros-args -r image:=/image_raw
    ```

    Na zapnutie kamery v inom termináli môžeme použiť balíčky `usb_cam` alebo `opencv_cam`, napríklad:

    ```bash
    ros2 run usb_cam usb_cam_node_exe
    ```

    alebo

    ```bash
    ros2 run opencv_cam opencv_cam_main
    ```

    Bohužiaľ, oba balíčky na zapnutie kamery nefungujú správne a pri ich spustení sa objavujú chyby, ktorých riešenie si vyžaduje veľa času. Ako alternatívu sme sa rozhodli prejsť na používanie virtuálnehо stroja, pretože ten podporuje plnohodnotné ovládače, čo rieši tento problém.

### Virtual Box 
1. **Inštalácia VirtualBox**: Najprv si stiahnite najnovšiu verziu [VirtualBox](https://www.virtualbox.org/wiki/Downloads) z oficiálnej webovej stránky, pričom vyberte verziu pre váš operačný systém. Po stiahnutí spustite inštalačný súbor a postupujte podľa pokynov na obrazovke. Po dokončení inštalácie otvorte VirtualBox. Zobrazí sa okno s rozhraním programu.

2. **Stiahnutie a inštalácia Ubuntu 22.04**: Prejdite na oficiálnu stránku [Ubuntu](https://ubuntu.com/download/desktop) a stiahnite si ISO obraz verzie Ubuntu 22.04 LTS (Jammy Jellyfish).

3. **Vytvorenie virtuálneho stroja**: Otvorte VirtualBox a kliknite na **New**. Zadajte názov virtuálneho stroja, napríklad **Linux**. Následne vyberte typ operačného systému **Linux** a verziu **Ubuntu (64-bit)**. Pridajte stiahnutý ISO obraz. Nastavte veľkosť pamäte, napríklad 4-8 GB (odporúča sa minimálne 4 GB). Vytvorte nový virtuálny pevný disk, zvoľte typ **VDI (VirtualBox Disk Image)** a formát **Dynamically allocated**. Nastavte veľkosť disku, napríklad 20-30 GB.

4. **Inštalácia Ubuntu**: Po vytvorení virtuálneho stroja ho vyberte a kliknite na **Start**. Spustí sa inštalačný proces Ubuntu. Postupujte podľa pokynov na obrazovke:
    - Vyberte jazyk inštalácie.
    - Nastavte rozloženie klávesnice.
    - Vytvorte používateľa a nastavte heslo.
    - Po dokončení inštalácie vás systém vyzve na reštartovanie virtuálneho stroja.
5. **Inštalácia rozšírení**: Na pripojenie kamery k virtuálnemu stroju je potrebné stiahnuť príslušný balík rozšírení pre VirtualBox (Extension Pack). Najskôr si ho stiahnite z [oficiálnej stránky](https://www.oracle.com/virtualization/technologies/vm/downloads/virtualbox-downloads.html) a potom nainštalujte podľa pokynov. Po inštalácii budete môcť vo virtuálnom stroji v záložke **Devices (Zariadenia)** vybrať a používať vstavanú kameru. ([Tutorial](https://www.youtube.com/watch?v=uQNKTNv6ETw&t=182s))
6. **Inštalácia ROS2 Humble**: Otvorte terminál v Ubuntu spustenom na virtuálnom stroji a postupujte podľa pokynov oficiálnej [dokumentácie na inštaláciu ROS2 Humble](https://docs.ros.org/en/humble/Installation.html). Po inštalácii sa uistite, že všetky potrebné závislosti sú nainštalované. To zahŕňa Python, pip a ďalšie balíky pre správnu funkčnosť ROS2. Napríklad, spustite nasledujúce príkazy pre inštaláciu základných nástrojov:

    ```bash
    sudo apt update
    sudo apt install python3-pip python3-colcon-common-extensions build-essential
    ```

    Tieto balíky budú potrebné na zostavenie a testovanie ROS2 projektov. Po inštalácii sa odporúča reštartovať systém a overiť funkčnosť ROS2 pomocou príkazu:

    ```bash
    ros2 --version
    ```
7. **Vytvorenie pracovného priestoru**: Pre vytvorenie pracovného priestoru v ROS2 postupujte nasledovne. Vytvorte nový adresár pre pracovný priestor, napríklad `perception_ws`, a prejdite doň:
   ```bash
   mkdir -p ~/perception_ws/src
   cd ~/perception_ws/src
   ```
   Potom vykonajte príkaz na inicializáciu pracovného priestoru:
   ```bash
   colcon build
   ```
   Následne je potrebné aktivovať pracovný priestor:
   ```bash
   source ~/perception_ws/install/setup.bash
   ```
   Teraz môžete pridávať balíky do adresára `src` a kompilovať ich pomocou príkazu `colcon build`.
8. **Inštalácia balíkov:** V tomto kroku môžeme nainštalovať potrebné balíky, konkrétne [AprilTag Detector](https://github.com/ros-misc-utilities/apriltag_detector/blob/master/apriltag_detector/README.md) a [Apriltag_ROS](https://github.com/christianrauch/apriltag_ros), ktoré budú podrobne rozobrané v nasledujúcich kapitolách. A samozrejme, je potrebné nainštalovať aj sprievodné balíky, ktoré sú nevyhnutné pre prácu s balíkom Apriltag, napríklad [usb_cam](https://github.com/ros-drivers/usb_cam) alebo [ros2_video_streamer](https://github.com/klintan/ros2_video_streamer).

### Docker
1. ddf
2. 


## AprilTag Detector
...

**Github Repo:** [AprilTag Detector](https://github.com/ros-misc-utilities/apriltag_detector/blob/master/apriltag_detector/README.md)

https://github.com/user-attachments/assets/6754d298-5c5f-4afd-a693-d837e96173bb

![tags_detection](https://github.com/user-attachments/assets/ad69f67f-529f-4b87-be01-8984d99efb8c)

![apriltag_detector_rosgraph](https://github.com/user-attachments/assets/293b4aa4-181c-4020-82c8-840eb3c40b6a)

## AprilTag Ros 
...

## AprilTag NVIDIA ISAAC ROS 
...

## Comparison 
...

## Conclusion
...

## Sources: 
1. [Install ROS2 on Windows (with WSL2) Video](https://www.youtube.com/watch?v=F3n0SMAFheM&t=413s)
2. [Installation ROS2 Humble Ubuntu (deb packages)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
3. [VirtualBox](https://www.virtualbox.org/wiki/Downloads)
4. [Ubuntu 22.04](https://releases.ubuntu.com/jammy/)
5. [Online/Offline Camera Calibration in ROS 2](https://medium.com/starschema-blog/offline-camera-calibration-in-ros-2-45e81df12555#:~:text=Follow%20this%20this%20step-by-step%20guide%20to%20learn%20how,camera%20for%20computer%20vision%20applications%20in%20ROS%202)
6. [How to give VM Ubuntu access to host machine integrated camera (Youtube)](https://www.youtube.com/watch?v=uQNKTNv6ETw&t=182s)

