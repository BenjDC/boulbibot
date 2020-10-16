# Notes pour le code teensy

* il faut regénérer la librairie rosserial à partir de rosserial_arduino : attention le teensy 4.0 n'est pas supporté il faut ajouter "|| defined(__IMXRT1062__)" ligne 44 du fichier ros_hardware.h (un script qui fait tout ça serait plutôt cool)