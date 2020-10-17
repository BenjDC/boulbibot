# Notes pour le code teensy

Dépendances du code teensy boulbibot :
* Encoder http://www.pjrc.com/teensy/td_libs_Encoder.html
* rosserial 

## librairie rosserial

La librairie rosserial arduino doit être regénérée (la version 0.7.9 sur le site d'arduino ne fonctionne pas !). Etapes pour génerer la librairie : 
* installer rosserial_arduino : `sudo apt install ros-melodic-rosserial-arduino`
* à partir du dossier teensyCode/lib generer la librairie : `rosrun rosserial_arduino make_libraries.py`
* patcher le define necessaire pour le support de teensy 4.0 : ajouter `|| defined(__IMXRT1062__)` ligne 44 du fichier `ros_hardware.h`

**TODO** un script qui fait la generation et la correction de la librairie ros